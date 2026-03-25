from machine import PWM, Pin
import time
import math

# ── Boundary Box ──────────────────────────────────────────────────────────────
x_min = -1000
x_max =  1000
y_min = -1000
y_max =  1000
BOUNDARY_BOX = (x_min, y_min, x_max, y_max)

# ── Robot Physical Constants ──────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 58.42
ENCODER_PPR       = 1200
WHEEL_BASE_MM     = 61.0

MM_PER_TICK = (WHEEL_DIAMETER_MM * math.pi) / ENCODER_PPR if ENCODER_PPR > 0 else 0.0

BITS = 65535

# ── PID Gains ─────────────────────────────────────────────────────────────────
# Position PID (used in move_to_position / move_small)
POS_KP = 400.0  # must be > stiction_duty/threshold = ~40000/150 ≈ 267
POS_KI =  0.5   # !! Tune
POS_KD =  5.0   # !! Tune


# ── PID Controller ────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, out_min=-BITS, out_max=BITS):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral  = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            dt = 0.001
        self._integral  += error * dt
        derivative       = (error - self._prev_error) / dt
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, out))

# ── Motor Board 1 ─────────────────────────────────────────────────────────────
AIN1 = PWM(Pin(0))
AIN2 = PWM(Pin(1))
SLP1 = Pin(2, Pin.OUT)
FLT1 = Pin(4, Pin.IN, Pin.PULL_DOWN)

# ── Motor Board 2 ─────────────────────────────────────────────────────────────
FLT2 = Pin(13, Pin.IN, Pin.PULL_DOWN)
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
def enc_a_handler(_pin):
    global encoder_a
    encoder_a += 1 if ENC_A_A.value() == ENC_A_B.value() else -1

def enc_b_handler(_pin):
    global encoder_b
    encoder_b += 1 if ENC_B_A.value() == ENC_B_B.value() else -1

def enc_c_handler(_pin):
    global encoder_c
    encoder_c += 1 if ENC_C_A.value() == ENC_C_B.value() else -1

ENC_A_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc_a_handler)
ENC_B_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc_b_handler)
ENC_C_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc_c_handler)

# ── Robot Pose ────────────────────────────────────────────────────────────────
pose_x     = 0.0
pose_y     = 0.0
pose_theta = 0.0

_prev_a = 0
_prev_b = 0
_prev_c = 0

_SQRT3_2 = math.sqrt(3) / 2.0

def update_pose():
    global pose_x, pose_y, pose_theta, _prev_a, _prev_b, _prev_c

    dA = (encoder_a - _prev_a) * MM_PER_TICK
    dB = (encoder_b - _prev_b) * MM_PER_TICK
    dC = (encoder_c - _prev_c) * MM_PER_TICK
    _prev_a, _prev_b, _prev_c = encoder_a, encoder_b, encoder_c

    local_x = (2.0 / 3.0) * (-dA + 0.5 * dB + 0.5 * dC)
    local_y = (2.0 / 3.0) * (-_SQRT3_2 * dB + _SQRT3_2 * dC)
    dtheta  = (dA + dB + dC) / (3.0 * WHEEL_BASE_MM) if WHEEL_BASE_MM > 0 else 0.0

    cos_t = math.cos(pose_theta)
    sin_t = math.sin(pose_theta)
    pose_x     += local_x * cos_t - local_y * sin_t
    pose_y     += local_x * sin_t + local_y * cos_t
    pose_theta += dtheta

    print("  [pose] x={:.1f} y={:.1f} theta={:.3f} | enc a={} b={} c={}".format(
        pose_x, pose_y, pose_theta, encoder_a, encoder_b, encoder_c))

# ── Boundary Check ────────────────────────────────────────────────────────────
def boundary():
    if BOUNDARY_BOX is None:
        return True
    x_min, y_min, x_max, y_max = BOUNDARY_BOX
    inside = x_min <= pose_x <= x_max and y_min <= pose_y <= y_max
    if not inside:
        print("  BOUNDARY FAILURE! x={:.1f} y={:.1f} (box: x[{},{}] y[{},{}])".format(
            pose_x, pose_y, x_min, x_max, y_min, y_max))
    return inside

# ── Helpers ───────────────────────────────────────────────────────────────────
def reset_encoders():
    global encoder_a, encoder_b, encoder_c, _prev_a, _prev_b, _prev_c
    encoder_a = encoder_b = encoder_c = 0
    _prev_a   = _prev_b   = _prev_c   = 0
    print("[reset_encoders] done")

def reset_pose():
    global pose_x, pose_y, pose_theta
    pose_x = pose_y = pose_theta = 0.0
    print("[reset_pose] done")

def motors_enable():
    SLP1.value(1)
    SLP2.value(1)
    print("[motors_enable] SLP1 and SLP2 HIGH")

def motors_disable():
    SLP1.value(0)
    SLP2.value(0)
    print("[motors_disable] SLP1 and SLP2 LOW")

def stop():
    print("[stop] CALLED — zeroing all motors")
    set_motor_a(0)
    set_motor_b(0)
    set_motor_c(0)
    print("[stop] done")

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
    # CIN1/CIN2 are physically swapped — invert here so positive duty = forward
    if duty >= 0:
        CIN1.duty_u16(0)
        CIN2.duty_u16(duty)
    else:
        CIN1.duty_u16(-duty)
        CIN2.duty_u16(0)

# ── Motion Functions ──────────────────────────────────────────────────────────
def move_to_position(target_a, target_b, target_c, max_speed=BITS):
    print("[move_to_position] CALLED target=({},{},{}) max_speed={}".format(
        target_a, target_b, target_c, max_speed))

    # PID disabled — open-loop bang-bang: drive each motor at fixed speed toward target
    # pid_a = PID(POS_KP, POS_KI, POS_KD, out_min=-max_speed, out_max=max_speed)
    # pid_b = PID(POS_KP, POS_KI, POS_KD, out_min=-max_speed, out_max=max_speed)
    # pid_c = PID(POS_KP, POS_KI, POS_KD, out_min=-max_speed, out_max=max_speed)

    THRESHOLD = 150
    deadline = time.ticks_add(time.ticks_ms(), 10000)  # 10s timeout
    while True:
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            print("[move_to_position] TIMEOUT")
            break

        error_a = target_a - encoder_a
        error_b = target_b - encoder_b
        error_c = target_c - encoder_c

        print("  [move_to_position] errors: a={} b={} c={} | enc: a={} b={} c={}".format(
            error_a, error_b, error_c, encoder_a, encoder_b, encoder_c))

        if abs(error_a) < THRESHOLD and abs(error_b) < THRESHOLD and abs(error_c) < THRESHOLD:
            print("[move_to_position] TARGET REACHED")
            break

        # Bang-bang: fixed speed in direction of error, 0 if already within threshold
        da = max_speed if error_a > THRESHOLD else (-max_speed if error_a < -THRESHOLD else 0)
        db = max_speed if error_b > THRESHOLD else (-max_speed if error_b < -THRESHOLD else 0)
        dc = max_speed if error_c > THRESHOLD else (-max_speed if error_c < -THRESHOLD else 0)

        set_motor_a(da)
        set_motor_b(db)
        set_motor_c(dc)

        time.sleep_ms(20)

    stop()
    print("[move_to_position] DONE")

def move_small(angle, distance_mm, speed):
    print("[move_small] CALLED angle={} dist={}mm speed={}".format(angle, distance_mm, speed))
    if distance_mm < 0:
        angle += 180
        distance_mm = -distance_mm

    angle_rad = math.radians(angle)
    target_a = encoder_a + int((math.cos(angle_rad)               * distance_mm) / MM_PER_TICK)
    target_b = encoder_b + int((math.cos(angle_rad - 2*math.pi/3) * distance_mm) / MM_PER_TICK)
    target_c = encoder_c + int((math.cos(angle_rad + 2*math.pi/3) * distance_mm) / MM_PER_TICK)

    print("  [move_small] targets: a={} b={} c={}".format(target_a, target_b, target_c))
    move_to_position(target_a, target_b, target_c, speed)
    print("[move_small] DONE")

def spin_in_place(speed, time_sec, direction):
    # speed: raw PWM duty (0–65535)
    # direction: "left" (CCW) or "right" (CW)
    print("[spin_in_place] CALLED speed={} time={}s direction={}".format(speed, time_sec, direction))

    duty = speed if direction == "left" else -speed

    deadline = time.ticks_add(time.ticks_ms(), int(time_sec * 1000))
    loop_count = 0

    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if FLT1.value() or FLT2.value():
            print("  [spin_in_place] FAULT DETECTED! FLT1={} FLT2={}".format(FLT1.value(), FLT2.value()))
            stop()
            return

        set_motor_a(duty)
        set_motor_b(duty)
        set_motor_c(duty)

        loop_count += 1
        if loop_count % 10 == 0:
            remaining = time.ticks_diff(deadline, time.ticks_ms())
            print("  [spin_in_place] loop={} duty={} enc a={} b={} c={} FLT1={} FLT2={} remaining={}ms".format(
                loop_count, duty, encoder_a, encoder_b, encoder_c,
                FLT1.value(), FLT2.value(), remaining))

        time.sleep_ms(50)

    stop()
    print("[spin_in_place] DONE — ran {} loops".format(loop_count))

# ── Startup ───────────────────────────────────────────────────────────────────
print("=== STARTUP ===")
time.sleep(10)
print("[startup] enabling motors")
motors_enable()
print("[startup] resetting encoders and pose")
reset_encoders()
reset_pose()
print("[startup] FLT1={} FLT2={}".format(FLT1.value(), FLT2.value()))
print("[startup] calling spin_in_place")

try:
    move_small(0,   200, 45000)   # forward 200mm
    time.sleep(1)
    move_small(180, 200, 45000)   # backward 200mm
    time.sleep(1)
    move_small(90,  200, 45000)   # strafe right 200mm
    time.sleep(1)
    move_small(270, 200, 45000)   # strafe left 200mm
    print("=== DONE ===")
except KeyboardInterrupt:
    print("=== STOPPED BY USER ===")
stop()
