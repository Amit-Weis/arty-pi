from machine import Pin, PWM
import time
import math

# Global Constants (mm)
wheelDiameter = 58.42
distFromCenter = 61
CPR = 12 
gearRatio = 100.37
ticksPerWheelRev = CPR * gearRatio
bits = 65535
sqrt3Inverse = 1.0 / math.sqrt(3)   # ≈ 0.5774
 
positionTolerance = 3.0   # mm — stop when this close
slowDownRadius   = 20.0   # mm — start decelerating within this range
minPWM           = 42000  # floor so motors don't stall at low duty

pi = math.pi
mmPerTick = (pi * wheelDiameter)/ticksPerWheelRev

SQRT3_2 = math.sqrt(3) / 2.0

# Boundary Box (mm)
x_min = -1000
x_max =  1000
y_min = -1000
y_max =  1000
BOUNDARY_BOX = (x_min, y_min, x_max, y_max)

# Motor Board 1
leftM1 = PWM(Pin(0))
leftM2 = PWM(Pin(1))
SLP1 = Pin(2, Pin.OUT)
FLT1 = Pin(4, Pin.IN, Pin.PULL_DOWN)

# Motor Board 2
FLT2 = Pin(13, Pin.IN, Pin.PULL_DOWN)
frontM1 = PWM(Pin(8))
frontM2 = PWM(Pin(9))
SLP2 = Pin(10, Pin.OUT)
rightM1 = PWM(Pin(12))
rightM2 = PWM(Pin(11))

# Led Pin
LED = PWM(Pin(25))

# PWM Frequency Setup
for pwm in [rightM1, rightM2, frontM1, frontM2, leftM1, leftM2]:
    pwm.freq(20000)

# Encoder Pins
ENC_L_A = Pin(27, Pin.IN, Pin.PULL_UP)
ENC_L_B = Pin(28, Pin.IN, Pin.PULL_UP)

ENC_F_A = Pin(19, Pin.IN, Pin.PULL_UP)
ENC_F_B = Pin(18, Pin.IN, Pin.PULL_UP)

ENC_R_A = Pin(17, Pin.IN, Pin.PULL_UP)
ENC_R_B = Pin(16, Pin.IN, Pin.PULL_UP)

# Encoder Counting
encoder_l = 0
encoder_f = 0
encoder_r = 0

def update_encoder_right(pin):
    global encoder_r
    if ENC_R_A.value() == ENC_R_B.value():
        encoder_r += 1
    else:
        encoder_r -= 1

def update_encoder_front(pin):
    global encoder_f
    if ENC_F_A.value() == ENC_F_B.value():
        encoder_f += 1
    else:
        encoder_f -= 1

def update_encoder_left(pin):
    global encoder_l
    if ENC_L_A.value() == ENC_L_B.value():
        encoder_l += 1
    else:
        encoder_l -= 1

ENC_R_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder_right)
ENC_F_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder_front)
ENC_L_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder_left)

# Robot Position and Orientation (every CW/CCW is from the outside looking in)
x = 0.0
y = 0.0
theta = 0.0

prevRight = 0
prevFront = 0
prevLeft = 0

def update_pose():
    global x, y, prevRight, prevFront, prevLeft, theta

    changeRight = (encoder_r - prevRight) * mmPerTick
    changeFront = (encoder_f - prevFront) * mmPerTick
    changeLeft = (encoder_l - prevLeft) * mmPerTick

    prevRight = encoder_r
    prevFront = encoder_f
    prevLeft = encoder_l

    # the y-axis is perp to front (so when front goes, x changes positively)
    deltaX = (changeFront - changeLeft * 0.5 + changeRight * 0.5)
    deltaY = (+changeLeft * SQRT3_2 - changeRight * SQRT3_2)

    x += deltaX
    y += deltaY

    deltaTheta = (changeRight + changeFront + changeLeft) / (3 * distFromCenter)
    theta += deltaTheta

# Check for outside boundary when calling move-to
def checkBoundary():
    global x, y
    if x > x_max or x < x_min or y > y_max or y < y_min:
        return False
    else: return True

# Setup motor movement
def moveFrontCw(speed):
    if speed >= 0:
        frontM1.duty_u16(0)
        frontM2.duty_u16(speed)
    else:
        frontM1.duty_u16(-speed)
        frontM2.duty_u16(0)

def moveRightCw(speed):
    if speed >= 0:
        rightM1.duty_u16(speed)
        rightM2.duty_u16(0)
    else:
        rightM1.duty_u16(0)
        rightM2.duty_u16(-speed)

def moveLeftCw(speed):
    if speed >= 0:
        leftM1.duty_u16(0)
        leftM2.duty_u16(speed)
    else:
        leftM1.duty_u16(-speed)
        leftM2.duty_u16(0) 

# Helper functions 
def resetEncoders():
    global encoder_l, encoder_f, encoder_r
    encoder_l = 0
    encoder_f = 0
    encoder_r = 0

def resetPose():
    global x, y, theta
    x = 0.0
    y = 0.0
    theta = 0.0

def stopAll():
    moveFrontCw(0)
    moveRightCw(0)
    moveLeftCw(0)
    SLP1.value(0)
    SLP2.value(0)

def stopSpin():
    moveFrontCw(0)
    moveRightCw(0)
    moveLeftCw(0)

def enableMotors():
    SLP1.value(1)
    SLP2.value(1)

def disableMotors():
    SLP1.value(0)
    SLP2.value(0)

# Stop if FLT is triggered
def checkFaults():
    if FLT1.value() == 1 or FLT2.value() == 1:
        stopAll()
        return True
    return False

def spin_in_place(speed, direction):
    if direction == 'cw':
        moveFrontCw(speed)
        moveRightCw(speed)
        moveLeftCw(speed)
    elif direction == 'ccw':
        moveFrontCw(-speed)
        moveRightCw(-speed)
        moveLeftCw(-speed)

def spin_to_angle(target_angle, speed):
    global theta
    if target_angle == 0 and temp_theta != 0:
        temp_theta = target_angle-theta
        while (temp_theta > 0.01 or temp_theta < -0.01) and not checkFaults():
            if temp_theta > 0:
                checkFaults()
                while spin_in_place(speed, 'cw'):
                    update_pose()
                    temp_theta = target_angle-theta
                    if temp_theta >= 0.01:
                        break
            else:
                checkFaults()
                while spin_in_place(speed, 'ccw'):
                    update_pose()
                    temp_theta = target_angle-theta
                    if temp_theta <= -0.01:
                        break
            update_pose()

def backward(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(-speed)
        moveRightCw(speed)
        moveFrontCw(0)
    stopSpin()

def forward(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(speed)
        moveRightCw(-speed)
        moveFrontCw(0)
    stopSpin()

def right(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(int(-speed*0.67)) #empirically derive
        moveRightCw(int(-speed*0.67))
        moveFrontCw(int(speed))
    stopSpin()

def left(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(int(speed*0.7)) #emperically derive
        moveRightCw(int(speed*0.7))
        moveFrontCw(int(-speed))
    stopSpin()

def backward_right(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(-speed)
        moveRightCw(int(speed*0.725)) #emperically derive
        moveFrontCw(int(speed*0.85))
    stopSpin()

def backward_left(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(int(-speed*0.65))
        moveRightCw(speed)
        moveFrontCw(int(-speed*0.85))
    stopSpin()

def forward_right(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(int(-speed*0.7))
        moveRightCw(-speed)
        moveFrontCw(int(speed*0.85))
    stopSpin()

def forward_left(speed, duration):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        moveLeftCw(speed)
        moveRightCw(int(speed*0.7))
        moveFrontCw(int(-speed*0.85))
    stopSpin()

#Main code
print("=== STARTUP ===")
time.sleep(5)
print("[startup] enabling motors")
enableMotors()
print("[startup] resetting encoders and pose")
resetEncoders()
resetPose()
print("[startup] FLT1={} FLT2={}".format(FLT1.value(), FLT2.value()))
print("[startup] calling move_to right")

try:
    forward_left(60000, 0.75)
    time.sleep(5)

    stopAll()

except KeyboardInterrupt:
    print("=== STOPPED BY USER ===")
    stopAll()

