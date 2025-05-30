import lgpio
import time
import threading
from .common_defs import Wheel, WheelState, log_info, log_error, log_success, DEFAULT_ACTION_DURATION_S


class Engine:
    # GPIO Pin definitions (BCM numbering)
    PIN_IN1 = 12  # Left Motor
    PIN_IN2 = 13  # Left Motor
    PIN_IN3 = 20  # Right Motor
    PIN_IN4 = 21  # Right Motor
    PIN_ENA = 6  # Left Motor PWM
    PIN_ENB = 26  # Right Motor PWM

    def __init__(self):
        self._gpio_chip_handle = -1
        self._action_timer = None
        self._timer_lock = threading.Lock()
        self._motor_control_lock = threading.Lock()
        log_info("Инициализация двигателя (с авто-остановкой)")

    def initialize_gpio(self) -> bool:
        try:
            self._gpio_chip_handle = lgpio.gpiochip_open(0)
            if self._gpio_chip_handle < 0:
                log_error(
                    f"Ошибка при открытии GPIO chip 0: {lgpio.exceptions.lgError(self._gpio_chip_handle).getMessage()}")
                return False

            pins_to_claim = [self.PIN_IN1, self.PIN_IN2, self.PIN_IN3, self.PIN_IN4, self.PIN_ENA, self.PIN_ENB]
            for pin in pins_to_claim:
                ret = lgpio.gpio_claim_output(self._gpio_chip_handle, pin, 0)  # Default low
                if ret != lgpio.SUCCESS:
                    log_error(f"Ошибка при инициализации GPIO pin {pin}: {lgpio.exceptions.lgError(ret).getMessage()}")
                    self.cleanup_gpio()
                    return False

            # Enable motors (set ENA and ENB to HIGH)
            # For PWM, you would use lgpio.tx_pwm here instead of gpio_write
            lgpio.gpio_write(self._gpio_chip_handle, self.PIN_ENA, 1)
            lgpio.gpio_write(self._gpio_chip_handle, self.PIN_ENB, 1)
            log_success("GPIO инициализирован успешно.")
            return True
        except lgpio.exceptions.lgError as e:
            log_error(f"LGIO Exception during GPIO initialization: {e}")
            if self._gpio_chip_handle >= 0:
                lgpio.gpiochip_close(self._gpio_chip_handle)
                self._gpio_chip_handle = -1
            return False
        except Exception as e:
            log_error(f"Unexpected error during GPIO initialization: {e}")
            if self._gpio_chip_handle >= 0:
                lgpio.gpiochip_close(self._gpio_chip_handle)
                self._gpio_chip_handle = -1
            return False

    def cleanup_gpio(self):
        log_info("Очистка GPIO...")
        self.stop()  # Ensure motors are stopped
        if self._gpio_chip_handle >= 0:
            try:
                # Set motor control pins to low before releasing
                lgpio.gpio_write(self._gpio_chip_handle, self.PIN_ENA, 0)
                lgpio.gpio_write(self._gpio_chip_handle, self.PIN_ENB, 0)

                pins_to_release = [self.PIN_IN1, self.PIN_IN2, self.PIN_IN3, self.PIN_IN4, self.PIN_ENA, self.PIN_ENB]
                for pin in pins_to_release:
                    lgpio.gpio_free(self._gpio_chip_handle, pin)
                lgpio.gpiochip_close(self._gpio_chip_handle)
                self._gpio_chip_handle = -1
                log_success("GPIO очищен.")
            except lgpio.exceptions.lgError as e:
                log_error(f"LGIO Exception during GPIO cleanup: {e}")
            except Exception as e:
                log_error(f"Unexpected error during GPIO cleanup: {e}")

    def _control_wheel_impl(self, wheel: Wheel, state: WheelState):
        with self._motor_control_lock:
            if self._gpio_chip_handle < 0:
                return

            pin_A, pin_B = (0, 0)

            if wheel == Wheel.LEFT:
                pin_A, pin_B = self.PIN_IN1, self.PIN_IN2
            elif wheel == Wheel.RIGHT:
                pin_A, pin_B = self.PIN_IN4, self.PIN_IN3  # Note: IN4, IN3 for right based on C++ (typical for L298N)

            level_A, level_B = 0, 0

            if state == WheelState.FORWARD:
                level_A, level_B = 1, 0
            elif state == WheelState.REVERSE:
                level_A, level_B = 0, 1
            # Else STOP, which is 0,0 (already set)

            # The C++ code had a swap for the left wheel.
            # This implies that for WheelState.FORWARD, IN1=LOW, IN2=HIGH would make it go forward.
            # Or, IN1/IN2 are wired "backwards" relative to IN3/IN4.
            # Let's assume the definition of WheelState.FORWARD is "physical forward"
            # and adjust pins if necessary.
            # The C++ comment: "Python code used REVERSE for forward"
            # "In control_wheel_impl I added std::swap for the left wheel if its logic HIGH/LOW for forward/reverse
            # is inverted compared to the right, which often бывает."
            # "My WheelState::FORWARD for lgpio означает "типичное" движение вперед."
            # If 'typical' forward for IN1/IN2 is IN1=1, IN2=0, and for some reason left motor moves backward,
            # then we need to swap level_A and level_B for the LEFT wheel.
            # Let's stick to the C++ logic which introduced the swap for the LEFT wheel.
            if wheel == Wheel.LEFT:
                level_A, level_B = level_B, level_A  # Swap for left wheel

            try:
                lgpio.gpio_write(self._gpio_chip_handle, pin_A, level_A)
                lgpio.gpio_write(self._gpio_chip_handle, pin_B, level_B)
            except lgpio.exceptions.lgError as e:
                log_error(f"LGIO Exception during wheel control: {e}")
            except Exception as e:
                log_error(f"Unexpected error during wheel control: {e}")

    def _start_action_timer_impl(self, duration_s: float):
        with self._timer_lock:
            self._cancel_previous_action_timer_impl()  # Cancel any existing timer
            self._action_timer = threading.Timer(duration_s, self._auto_stop_motors_impl)
            self._action_timer.start()

    def _cancel_previous_action_timer_impl(self):
        # This lock is already acquired by the caller (_start_action_timer_impl or stop)
        if self._action_timer and self._action_timer.is_alive():
            self._action_timer.cancel()
        self._action_timer = None

    def _auto_stop_motors_impl(self):
        log_info(f"Авто-остановка после {DEFAULT_ACTION_DURATION_S} сек.")
        self._control_wheel_impl(Wheel.LEFT, WheelState.STOP)
        self._control_wheel_impl(Wheel.RIGHT, WheelState.STOP)
        with self._timer_lock:  # Ensure timer is cleared
            self._action_timer = None

    def forward(self, duration_s: float = DEFAULT_ACTION_DURATION_S):
        log_info(f"Начало движения вперёд (на {duration_s} сек)...")
        # C++ code: control_wheel_impl(Wheel::LEFT, WheelState::FORWARD);
        # C++ code: control_wheel_impl(Wheel::RIGHT, WheelState::FORWARD);
        # The C++ notes state: "Python code used REVERSE for forward".
        # "Я постарался отразить это в C++ control_wheel_impl с учетом возможной инверсии для одного из колес,
        # но forward теперь вызывает WheelState::FORWARD для обоих."
        # This means WheelState.FORWARD in C++ (and here) should correspond to physical forward.
        # The swap inside _control_wheel_impl handles the left motor's potential inversion.
        self._control_wheel_impl(Wheel.LEFT, WheelState.FORWARD)
        self._control_wheel_impl(Wheel.RIGHT, WheelState.FORWARD)
        self._start_action_timer_impl(duration_s)

    def stop(self):
        log_info("Команда STOP: Остановка двигателя...")
        with self._timer_lock:
            self._cancel_previous_action_timer_impl()
        self._control_wheel_impl(Wheel.LEFT, WheelState.STOP)
        self._control_wheel_impl(Wheel.RIGHT, WheelState.STOP)
        log_success("Двигатель остановлен по команде STOP")



    def turn_left(self, duration_s: float = DEFAULT_ACTION_DURATION_S):
        log_info(f"Поворот налево (на {duration_s} сек)...")
        # C++: control_wheel_impl(Wheel::LEFT, WheelState::REVERSE);
        # C++: control_wheel_impl(Wheel::RIGHT, WheelState::FORWARD);
        self._control_wheel_impl(Wheel.LEFT, WheelState.REVERSE)
        self._control_wheel_impl(Wheel.RIGHT, WheelState.FORWARD)
        self._start_action_timer_impl(duration_s)

    def turn_right(self, duration_s: float = DEFAULT_ACTION_DURATION_S):
        log_info(f"Поворот направо (на {duration_s} сек)...")
        # C++: control_wheel_impl(Wheel::LEFT, WheelState::FORWARD);
        # C++: control_wheel_impl(Wheel::RIGHT, WheelState::REVERSE);
        self._control_wheel_impl(Wheel.LEFT, WheelState.FORWARD)
        self._control_wheel_impl(Wheel.RIGHT, WheelState.REVERSE)
        self._start_action_timer_impl(duration_s)

    def __del__(self):
        self.cleanup_gpio()
