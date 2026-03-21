from machine import PWM, Pin
import time
import math

# ── Boundary Box (set/imported externally) ────────────────────────────────────
# Format: (x_min_mm, y_min_mm, x_max_mm, y_max_mm)
# Set this before running any motion, e.g.: from boundary_config import BOUNDARY_BOX
x_min = -1000
x_max =  1000
y_min = -1000
y_max =  1000
BOUNDARY_BOX = (x_min, y_min, x_max, y_max)  # (x_min, y_min, x_max, y_max) in mm

# ── Robot Physical Constants ──────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 58.42
ENCODER_PPR       = 1200      # !! Set this: encoder pulses per wheel revolution
WHEEL_BASE_MM     = 61.0    # !! Set this: distance from robot centre to wheel contact (mm)

# MM_PER_TICK is computed once ENCODER_PPR is set above
MM_PER_TICK = (WHEEL_DIAMETER_MM * math.pi) / ENCODER_PPR if ENCODER_PPR > 0 else 0.0

BITS = 65535  # Max 16-bit PWM duty cycle value

# ── Motor Board 1 ─────────────────────────────────────────────────────────────
AIN1 = PWM(Pin(0))
AIN2 = PWM(Pin(1))
SLP1 = Pin(2, Pin.OUT)
FLT1 = Pin(4, Pin.IN)   # subject to change

# ── Motor Board 2 ─────────────────────────────────────────────────────────────
FLT2 = Pin(13, Pin.IN)  # subject to change
BIN1 = PWM(Pin(8))
BIN2 = PWM(Pin(9))
SLP2 = Pin(10, Pin.OUT)
CIN1 = PWM(Pin(12))
CIN2 = PWM(Pin(11))

# ── LED ───────────────────────────────────────────────────────────────────────
LED = PWM(Pin(25))

# ── PWM Frequencies ───────────────────────────────────────────────────────────
for pwm in [AIN1, AIN2, BIN1, BIN2, CIN1, CIN2]:
    pwm.freq(20000)

# ── Encoder Pins ──────────────────────────────────────────────────────────────
# !! Change pin numbers below to match your wiring !!
ENC_A_A = Pin(27, Pin.IN, Pin.PULL_UP)
ENC_A_B = Pin(28, Pin.IN, Pin.PULL_UP)

ENC_B_A = Pin(19, Pin.IN, Pin.PULL_UP)
ENC_B_B = Pin(18, Pin.IN, Pin.PULL_UP)

ENC_C_A = Pin(17, Pin.IN, Pin.PULL_UP)
ENC_C_B = Pin(16, Pin.IN, Pin.PULL_UP)

# ── Encoder Counts ────────────────────────────────────────────────────────────
encoder_a = 0
encoder_b = 0
encoder_c = 0

# ── Encoder Interrupt Handlers ────────────────────────────────────────────────
# Rising edge on A channel; B channel sampled for direction (B=0 → +1, B=1 → -1)
def enc_a_handler(_pin):
    global encoder_a
    encoder_a += 1 if ENC_A_B.value() == 0 else -1

def enc_b_handler(_pin):
    global encoder_b
    encoder_b += 1 if ENC_B_B.value() == 0 else -1

def enc_c_handler(_pin):
    global encoder_c
    encoder_c += 1 if ENC_C_B.value() == 0 else -1

ENC_A_A.irq(trigger=Pin.IRQ_RISING, handler=enc_a_handler)
ENC_B_A.irq(trigger=Pin.IRQ_RISING, handler=enc_b_handler)
ENC_C_A.irq(trigger=Pin.IRQ_RISING, handler=enc_c_handler)

# ── Robot Pose ────────────────────────────────────────────────────────────────
pose_x     = 0.0  # mm
pose_y     = 0.0  # mm
pose_theta = 0.0  # radians

_prev_a = 0  # encoder snapshot used for incremental pose updates
_prev_b = 0
_prev_c = 0

_SQRT3_2 = math.sqrt(3) / 2.0

def update_pose():
    global pose_x, pose_y, pose_theta, _prev_a, _prev_b, _prev_c

    dA = (encoder_a - _prev_a) * MM_PER_TICK
    dB = (encoder_b - _prev_b) * MM_PER_TICK
    dC = (encoder_c - _prev_c) * MM_PER_TICK
    _prev_a, _prev_b, _prev_c = encoder_a, encoder_b, encoder_c

    # Displacement in the robot's local frame
    local_x = (2.0 / 3.0) * (-dA + 0.5 * dB + 0.5 * dC)
    local_y = (2.0 / 3.0) * (-_SQRT3_2 * dB + _SQRT3_2 * dC)
    dtheta  = (dA + dB + dC) / (3.0 * WHEEL_BASE_MM) if WHEEL_BASE_MM > 0 else 0.0

    # Rotate local displacement into world frame using current heading
    cos_t = math.cos(pose_theta)
    sin_t = math.sin(pose_theta)
    pose_x     += local_x * cos_t - local_y * sin_t
    pose_y     += local_x * sin_t + local_y * cos_t
    pose_theta += dtheta

# ── Boundary Check ────────────────────────────────────────────────────────────
def boundary():
    """Returns True if the robot is inside BOUNDARY_BOX, False if it has exited."""
    if BOUNDARY_BOX is None:
        return True  # no boundary configured — always allow movement
    x_min, y_min, x_max, y_max = BOUNDARY_BOX
    return x_min <= pose_x <= x_max and y_min <= pose_y <= y_max

# ── Helpers ───────────────────────────────────────────────────────────────────
def reset_encoders():
    global encoder_a, encoder_b, encoder_c, _prev_a, _prev_b, _prev_c
    encoder_a = encoder_b = encoder_c = 0
    _prev_a   = _prev_b   = _prev_c   = 0

def reset_pose():
    global pose_x, pose_y, pose_theta
    pose_x = pose_y = pose_theta = 0.0

def motors_enable():
    SLP1.value(1)
    SLP2.value(1)

def motors_disable():
    SLP1.value(0)
    SLP2.value(0)

def stop():
    set_motor_a(0)
    set_motor_b(0)
    set_motor_c(0)

def set_motor_a(duty):
    if duty >= 0:
        AIN1.duty_u16(duty)
        AIN2.duty_u16(0)
    else:
        AIN1.duty_u16(0)
        AIN2.duty_u16(-duty)

def set_motor_b(duty):
    if duty >= 0:
        BIN1.duty_u16(duty)
        BIN2.duty_u16(0)
    else:
        BIN1.duty_u16(0)
        BIN2.duty_u16(-duty)

def set_motor_c(duty):
    if duty >= 0:
        CIN1.duty_u16(duty)
        CIN2.duty_u16(0)
    else:
        CIN1.duty_u16(0)
        CIN2.duty_u16(-duty)
        
# ── Motion Functions ──────────────────────────────────────────────────────────
def move_to_position(target_a, target_b, target_c, speed):
    while True:
        update_pose()
        if not boundary():
            stop()
            break

        error_a = target_a - encoder_a
        error_b = target_b - encoder_b
        error_c = target_c - encoder_c

        if abs(error_a) < 10 and abs(error_b) < 10 and abs(error_c) < 10:
            break

        set_motor_a(speed if error_a > 0 else -speed if error_a < 0 else 0)
        set_motor_b(speed if error_b > 0 else -speed if error_b < 0 else 0)
        set_motor_c(speed if error_c > 0 else -speed if error_c < 0 else 0)

    stop()

def move_small(angle, distance_mm, speed):
    if distance_mm < 0:
        angle += 180
        distance_mm = -distance_mm

    # Convert angle to radians and compute target encoder counts (offset from current position)
    angle_rad = math.radians(angle)
    target_a = encoder_a + int((math.cos(angle_rad)               * distance_mm) / MM_PER_TICK)
    target_b = encoder_b + int((math.cos(angle_rad - 2*math.pi/3) * distance_mm) / MM_PER_TICK)
    target_c = encoder_c + int((math.cos(angle_rad + 2*math.pi/3) * distance_mm) / MM_PER_TICK)

    move_to_position(target_a, target_b, target_c, speed)

def spin_in_place(speed, time_sec, direction):
    s = speed if direction == "left" else -speed
    deadline = time.ticks_add(time.ticks_ms(), int(time_sec * 1000))
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        update_pose()
        if not boundary():
            break
        set_motor_a(-s)
        set_motor_b(s)
        set_motor_c(s)
    stop()

# ── Startup ───────────────────────────────────────────────────────────────────
time.sleep(10)  # allow Thonny to connect before code runs
motors_enable()
reset_encoders()
reset_pose()

spin_in_place(40000, 5, "left")

stop()

