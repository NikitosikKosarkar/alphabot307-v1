#include <iostream>
#include <string>
#include <mosquitto.h>
#include <sstream>
#include <vector>
#include <thread>
#include <cstring>
#include <fstream>
#include <linux/gpio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <atomic>
#include <memory>

class GPIOController {
private:
    std::vector<int> gpio_pins = {12, 13, 6, 20, 21, 26};
    int gpio_chip_fd;

    void gpio_write(int offset, uint8_t value) {
        struct gpiohandle_request req;
        struct gpiohandle_data data;

        memset(&req, 0, sizeof(req));
        memset(&data, 0, sizeof(data));

        strncpy(req.consumer_label, "robot_control", sizeof(req.consumer_label) - 1);
        req.consumer_label[sizeof(req.consumer_label) - 1] = '\0';
        req.lineoffsets[0] = offset;
        req.lines = 1;
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;

        if (ioctl(gpio_chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
            std::cerr << "Error requesting line handle for GPIO " << offset << ": " << strerror(errno) << std::endl;
            return;
        }

        data.values[0] = value;
        if (ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
            std::cerr << "Error setting line value for GPIO " << offset << " to " << (int)value << ": " << strerror(errno) << std::endl;
        }

        close(req.fd);
    }


public:
    GPIOController() {
        gpio_chip_fd = open("/dev/gpiochip0", O_RDWR);
        if (gpio_chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO device /dev/gpiochip0: " + std::string(strerror(errno)));
        }

        for (int pin : gpio_pins) {
            gpio_write(pin, 0);
        }
        stop();
    }

    void forward(int time_ms) {
        std::cout << "Moving forward for " << time_ms << " ms..." << std::endl;

        gpio_write(6, 1);
        gpio_write(26, 1);
        gpio_write(12, 0);
        gpio_write(13, 1);
        gpio_write(20, 1);
        gpio_write(21, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
        stop();
    }

    void stop() {
        std::cout << "Stopping..." << std::endl;

        gpio_write(6, 1);
        gpio_write(26, 1);
        gpio_write(12, 0);
        gpio_write(13, 0);
        gpio_write(20, 0);
        gpio_write(21, 0);
    }

    void right(int time_ms) {
        std::cout << "Turning right for " << time_ms << " ms..." << std::endl;


        gpio_write(6, 1);
        gpio_write(26, 1);
        gpio_write(12, 0);
        gpio_write(13, 1);
        gpio_write(20, 0);
        gpio_write(21, 1);


        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
        stop();
    }

    void left(int time_ms) {
        std::cout << "Turning left for " << time_ms << " ms..." << std::endl;


        gpio_write(6, 1);
        gpio_write(26, 1);
        gpio_write(12, 1);
        gpio_write(13, 0);
        gpio_write(20, 1);
        gpio_write(21, 0);


        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
        stop();
    }

    ~GPIOController() {
        stop();
        if (gpio_chip_fd >= 0) {
            close(gpio_chip_fd);
        }
    }
};

class MqttGpioReceiver {
public:
    MqttGpioReceiver(const char* host, int port, const char* topic)
        : topic_(topic), running_(false), last_gpio_data_ptr_(nullptr) {
        mosquitto_lib_init();
        mosq_ = mosquitto_new("robot_client", true, this);

        if (!mosq_) {
            throw std::runtime_error("Failed to create Mosquitto instance");
        }

        mosquitto_message_callback_set(mosq_, message_callback);

        if (mosquitto_connect(mosq_, host, port, 60) != MOSQ_ERR_SUCCESS) {
            std::string err_msg = "Failed to connect to MQTT broker: ";
            throw std::runtime_error(err_msg + "check network and broker settings.");
        }

        if (mosquitto_subscribe(mosq_, nullptr, topic_, 1) != MOSQ_ERR_SUCCESS) {
            throw std::runtime_error("Failed to subscribe to topic: " + std::string(topic_));
        }

        running_ = true;
        thread_ = std::thread(&MqttGpioReceiver::loop, this);
    }

    ~MqttGpioReceiver() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

    std::string get_gpio_data() {
        std::shared_ptr<const std::string> data_ptr = last_gpio_data_ptr_.exchange(nullptr, std::memory_order_acq_rel);
        if (data_ptr) {
            return *data_ptr;
        }
        return "";
    }

private:
    static void message_callback(struct mosquitto* mosq, void* obj,
        const struct mosquitto_message* msg) {
        MqttGpioReceiver* receiver = static_cast<MqttGpioReceiver*>(obj);

        auto new_payload_ptr = std::make_shared<const std::string>(static_cast<char*>(msg->payload), msg->payloadlen);

        receiver->last_gpio_data_ptr_.store(new_payload_ptr, std::memory_order_release);
    }

    void loop() {
        while (running_) {
            int rc = mosquitto_loop(mosq_, 100, 1);
            if (rc != MOSQ_ERR_SUCCESS && rc != MOSQ_ERR_NO_CONN && rc != MOSQ_ERR_CONN_LOST) {
                 std::cerr << "Mosquitto loop error: " << mosquitto_strerror(rc) << std::endl;
                 if(rc == MOSQ_ERR_CONN_LOST) {
                     std::this_thread::sleep_for(std::chrono::seconds(1));
                     mosquitto_reconnect(mosq_);
                 } else if (rc == MOSQ_ERR_NO_CONN) {
                     mosquitto_reconnect(mosq_);
                 }
            }
        }
    }

    struct mosquitto* mosq_;
    const char* topic_;
    std::atomic<std::shared_ptr<const std::string>> last_gpio_data_ptr_;
    std::thread thread_;
    std::atomic<bool> running_;
};

class GpioProcessor {
public:
    GpioProcessor(GPIOController& gpio, MqttGpioReceiver& receiver)
        : gpio_(gpio), receiver_(receiver) {}

    void run() {
        std::cout << "Robot GPIO control started. Waiting for MQTT commands on topic..." << std::endl;
        while (true) {
            std::string gpio_data = receiver_.get_gpio_data();

            if (gpio_data.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            std::cout << "Received command: " << gpio_data << std::endl;

            if (gpio_data == "forward") {
                gpio_.forward(200);
            }
            else if (gpio_data == "stop") {
                gpio_.stop();
            }
            else if (gpio_data == "left") {
                gpio_.left(100);
            }
            else if (gpio_data == "right") {
                gpio_.right(100);
            }
            else {
                std::cerr << "Unknown command: " << gpio_data << std::endl;
            }
        }
    }

private:
    GPIOController& gpio_;
    MqttGpioReceiver& receiver_;
};

int main() {
    try {
        std::cout << "Initializing GPIOController..." << std::endl;
        GPIOController gpio;

        std::cout << "Initializing MqttGpioReceiver..." << std::endl;
        MqttGpioReceiver receiver("192.168.1.100", 1883, "robot/gpio");

        std::cout << "Initializing GpioProcessor..." << std::endl;
        GpioProcessor processor(gpio, receiver);

        processor.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Critical Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}