from machine import Pin, PWM
import utime

# =========================
# PIN DEFINITIONS (YOUR PCB)
# =========================
# Motor Driver (TB6612)
PWMA_PIN = 15
AIN1_PIN = 13
AIN2_PIN = 14

PWMB_PIN = 16
BIN1_PIN = 18
BIN2_PIN = 17

STBY_PIN = 12

# Ultrasonic Sensor (HC-SR04)
ECHO_PIN = 8
TRIG_PIN = 9

# On-board RGB LED (per your pinout)
BLUE_PIN  = 26
GREEN_PIN = 27
RED_PIN   = 28

# Button
BUTTON_PIN = 22


# =========================
# CONSTANTS / TUNING
# =========================
MAX_DUTY = 65535

MAX_PWM = 0.70
WORK_SPEED = 0.45
DISTANCE_THRESHOLD = 20.0  # cm

# ✅ From your calibration + direction test:
RIGHT_REVERSED = False
FORWARD_SIGN = -1          # this makes the robot drive forward

BACKUP_SPEED = 0.40
BACKUP_TIME_S = 0.35

TURN_SPEED = 0.40
TURN_TIME_S = 0.45

# === CLOSE-PROXIMITY TURN THRESHOLD ===
TURN_THRESHOLD_MM = 12.7        # 0.5 inch
TURN_THRESHOLD_CM = TURN_THRESHOLD_MM / 10.0  # 1.27 cm


# =========================
# MOTOR SETUP
# =========================
stby = Pin(STBY_PIN, Pin.OUT)
stby.value(0)  # disabled until ready

pwma = PWM(Pin(PWMA_PIN))
pwmb = PWM(Pin(PWMB_PIN))
pwma.freq(1000)
pwmb.freq(1000)

ain1 = Pin(AIN1_PIN, Pin.OUT)
ain2 = Pin(AIN2_PIN, Pin.OUT)
bin1 = Pin(BIN1_PIN, Pin.OUT)
bin2 = Pin(BIN2_PIN, Pin.OUT)

def stop_motors():
    # Coast stop
    ain1.value(0); ain2.value(0)
    bin1.value(0); bin2.value(0)
    pwma.duty_u16(0)
    pwmb.duty_u16(0)

def set_motors(left, right, scale=1.0):
    left *= scale
    right *= scale

    # Define what "forward" means for THIS robot
    left *= FORWARD_SIGN
    right *= FORWARD_SIGN

    # Optional wiring correction (OFF for your build)
    if RIGHT_REVERSED:
        right = -right

    # Clamp
    left = max(-MAX_PWM, min(MAX_PWM, left))
    right = max(-MAX_PWM, min(MAX_PWM, right))

    # LEFT motor (A)
    if left > 0:
        ain1.value(1); ain2.value(0)
        pwma.duty_u16(int(MAX_DUTY * left))
    elif left < 0:
        ain1.value(0); ain2.value(1)
        pwma.duty_u16(int(MAX_DUTY * -left))
    else:
        ain1.value(0); ain2.value(0)
        pwma.duty_u16(0)

    # RIGHT motor (B)
    if right > 0:
        bin1.value(1); bin2.value(0)
        pwmb.duty_u16(int(MAX_DUTY * right))
    elif right < 0:
        bin1.value(0); bin2.value(1)
        pwmb.duty_u16(int(MAX_DUTY * -right))
    else:
        bin1.value(0); bin2.value(0)
        pwmb.duty_u16(0)


# =========================
# ULTRASONIC SETUP
# =========================
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

def measure_distance(timeout_us=30000):
    # Trigger pulse
    trig.value(0)
    utime.sleep_us(5)
    trig.value(1)
    utime.sleep_us(10)
    trig.value(0)

    # Wait for echo HIGH
    t0 = utime.ticks_us()
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), t0) > timeout_us:
            return None

    start = utime.ticks_us()

    # Wait for echo LOW
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout_us:
            return None

    end = utime.ticks_us()
    duration = utime.ticks_diff(end, start)

    # cm
    return (duration * 0.0343) / 2

def read_distance_cm():
    # Filter: average valid samples; ignore invalid/missed echoes
    vals = []
    for _ in range(5):
        d = measure_distance()
        if d is not None and 3 <= d <= 300:
            vals.append(d)
        utime.sleep_ms(15)
    return (sum(vals) / len(vals)) if vals else None


# =========================
# RGB LED SETUP
# =========================
red_pwm = PWM(Pin(RED_PIN))
green_pwm = PWM(Pin(GREEN_PIN))
blue_pwm = PWM(Pin(BLUE_PIN))
for p in (red_pwm, green_pwm, blue_pwm):
    p.freq(1000)

def set_led(r, g, b):
    red_pwm.duty_u16(int(MAX_DUTY * r))
    green_pwm.duty_u16(int(MAX_DUTY * g))
    blue_pwm.duty_u16(int(MAX_DUTY * b))


# =========================
# BUTTON SETUP
# =========================
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

def button_pressed():
    return button.value() == 1

def wait_button_release():
    while button.value() == 1:
        utime.sleep_ms(10)
    utime.sleep_ms(80)


# =========================
# MODES
# =========================
def system_check():
    print("System check...")
    t0 = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), t0) < 1200:
        set_led(1, 1, 1)
        utime.sleep(0.1)
        set_led(0, 0, 0)
        utime.sleep(0.1)
    print("Ready.")

def pause_mode():
    print("PAUSE MODE (press button to start)")
    stop_motors()
    t0 = utime.ticks_ms()
    while True:
        if button_pressed():
            wait_button_release()
            return
        # breathing green
        phase = (utime.ticks_ms() - t0) / 1000
        brightness = abs((phase % 1) - 0.5) * 2
        set_led(0, brightness, 0)
        utime.sleep_ms(20)

def work_mode():
    print("WORK MODE")
    while True:
        if button_pressed():
            stop_motors()
            wait_button_release()
            return

        d = read_distance_cm()
        # print("d =", d)  # uncomment for debugging

        if d is None:
            # No reliable echo -> keep moving forward
            set_led(0, 1, 1)
            set_motors(WORK_SPEED, WORK_SPEED)

        elif d <= TURN_THRESHOLD_CM:
        # VERY close (0.5 inch or less): immediate left turn
            set_led(1, 0, 1)  # purple = very close

            # Pivot left
            set_motors(TURN_SPEED, -TURN_SPEED)
            utime.sleep(TURN_TIME_S)

            stop_motors()
            utime.sleep(0.1)

        elif d < DISTANCE_THRESHOLD:
            # Normal obstacle avoidance (backup + turn)
            set_led(1, 0, 0)  # red

            # Back up
            set_motors(-BACKUP_SPEED, -BACKUP_SPEED)
            utime.sleep(BACKUP_TIME_S)

            # Turn left
            set_motors(TURN_SPEED, -TURN_SPEED)
            utime.sleep(TURN_TIME_S)

            stop_motors()
            utime.sleep(0.1)


        else:
            # Clear path -> forward
            set_led(0, 1, 0)
            set_motors(WORK_SPEED, WORK_SPEED)

        utime.sleep_ms(30)


# =========================
# MAIN
# =========================
def main():
    stop_motors()
    stby.value(1)  # enable motor driver outputs
    utime.sleep_ms(200)

    system_check()

    while True:
        pause_mode()
        work_mode()

main()


