
from machine import Pin, PWM
import utime

#PIN DEFINITIONS

# Motor Driver (TB6612 control pins on PCB)
PWMA_PIN = 15
AIN1_PIN = 19
AIN2_PIN = 20
PWMB_PIN = 18
BIN1_PIN = 16
BIN2_PIN = 17
STBY_PIN = 14

# Ultrasonic Sensor (HC-SR04 header) 
ECHO_PIN = 8
TRIG_PIN = 9

#  On-board RGB LED 
RED_PIN   = 26
GREEN_PIN = 27
BLUE_PIN  = 28

#  Button 
BUTTON_PIN = 22


# CONSTANTS


MAX_DUTY = 65535
WORK_SPEED = 0.6
DISTANCE_THRESHOLD = 20.0

LOW_BATTERY_TIME = 45_000     # 45 seconds
CRITICAL_TIME = 55_000        # 55 seconds


# MOTOR SETUP


stby = Pin(STBY_PIN, Pin.OUT)
stby.value(1)

pwma = PWM(Pin(PWMA_PIN))
pwmb = PWM(Pin(PWMB_PIN))
pwma.freq(1000)
pwmb.freq(1000)

ain1 = Pin(AIN1_PIN, Pin.OUT)
ain2 = Pin(AIN2_PIN, Pin.OUT)
bin1 = Pin(BIN1_PIN, Pin.OUT)
bin2 = Pin(BIN2_PIN, Pin.OUT)

def set_motors(left, right, scale=1.0):
    left *= scale
    right *= scale

    # LEFT motor (A channel)
    if left > 0:
        ain1.value(1); ain2.value(0)
        pwma.duty_u16(int(MAX_DUTY * left))
    elif left < 0:
        ain1.value(0); ain2.value(1)
        pwma.duty_u16(int(MAX_DUTY * -left))
    else:
        pwma.duty_u16(0)

    # RIGHT motor (B channel)
    if right > 0:
        bin1.value(1); bin2.value(0)
        pwmb.duty_u16(int(MAX_DUTY * right))
    elif right < 0:
        bin1.value(0); bin2.value(1)
        pwmb.duty_u16(int(MAX_DUTY * -right))
    else:
        pwmb.duty_u16(0)

def stop_motors():
    pwma.duty_u16(0)
    pwmb.duty_u16(0)


# ULTRASONIC SETUP

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

def measure_distance(timeout_us=30000):
    # Trigger pulse
    trig.value(0)
    utime.sleep_us(5)
    trig.value(1)
    utime.sleep_us(10)
    trig.value(0)

    # Wait for echo HIGH (start)
    t0 = utime.ticks_us()
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), t0) > timeout_us:
            return 999.0  # no reading -> treat as far away

    start = utime.ticks_us()

    # Wait for echo LOW (end)
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout_us:
            return 999.0

    end = utime.ticks_us()
    duration = utime.ticks_diff(end, start)

    # cm
    return (duration * 0.0343) / 2


# RGB LED SETUP

red_pwm = PWM(Pin(RED_PIN))
green_pwm = PWM(Pin(GREEN_PIN))
blue_pwm = PWM(Pin(BLUE_PIN))

for p in (red_pwm, green_pwm, blue_pwm):
    p.freq(1000)

def set_led(r, g, b):
    # r,g,b expected 0.0 to 1.0
    red_pwm.duty_u16(int(MAX_DUTY * r))
    green_pwm.duty_u16(int(MAX_DUTY * g))
    blue_pwm.duty_u16(int(MAX_DUTY * b))


#  BUTTON (FIXED: ACTIVE HIGH + PULL_DOWN)


button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

def button_pressed():
    return button.value() == 1   #  ACTIVE HIGH

def wait_button_release():
    # simple debounce: wait until released, then settle
    while button.value() == 1:
        utime.sleep_ms(10)
    utime.sleep_ms(80)


#  SYSTEM CHECK (5Hz BLINK)

def system_check():
    print("System check...")

    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < 2000:
        set_led(1, 1, 1)
        utime.sleep(0.1)
        set_led(0, 0, 0)
        utime.sleep(0.1)

    print("Entering PAUSE mode")


#  PAUSE MODE

def pause_mode():
    print("PAUSE MODE")
    stop_motors()
    t0 = utime.ticks_ms()

    while True:
        if button_pressed():
            wait_button_release()
            return "WORK"

        # breathing green while waiting
        phase = (utime.ticks_ms() - t0) / 1000
        brightness = abs((phase % 1) - 0.5) * 2
        set_led(0, brightness, 0)
        utime.sleep_ms(20)

#  WORK MODE

def work_mode(work_time):
    print("WORK MODE")
    last = utime.ticks_ms()

    while True:
        now = utime.ticks_ms()
        dt = utime.ticks_diff(now, last)
        last = now
        work_time += dt

        # --- CRITICAL BATTERY BEHAVIOR ---
        if work_time >= CRITICAL_TIME:
            stop_motors()
            t = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), t) < 5000:
                set_led(1, 0, 0)
                utime.sleep(0.05)
                set_led(0, 0, 0)
                utime.sleep(0.05)
            stop_motors()
            set_led(0, 0, 0)
            return None

        # --- LOW BATTERY BEHAVIOR ---
        if work_time >= LOW_BATTERY_TIME:
            speed_scale = 0.5
            set_led(0, 0, 1)   # blue
        else:
            speed_scale = 1.0
            set_led(0, 1, 0)   # green

        # --- pause toggle ---
        if button_pressed():
            stop_motors()
            wait_button_release()
            return work_time

        d = measure_distance()

        if d < DISTANCE_THRESHOLD:
            set_motors(-0.5, -0.5, speed_scale)
            utime.sleep(0.4)
            set_motors(0.5, -0.5, speed_scale)
            utime.sleep(0.5)
        else:
            set_motors(WORK_SPEED, WORK_SPEED, speed_scale)

        utime.sleep_ms(20)


#  MAIN LOOP

def main():
    system_check()
    mode = "PAUSE"
    work_time = 0

    while True:
        if mode == "PAUSE":
            mode = pause_mode()
        else:
            result = work_mode(work_time)
            if result is None:
                print("SYSTEM SHUTDOWN ✅")
                break
            work_time = result
            mode = "PAUSE"

main()


