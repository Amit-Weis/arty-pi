"""
movement_listener.py  —  runs on the Raspberry Pi Pico (MicroPython)
Listens for serial commands from robot_controller.py on the Pi and drives motors.

Serial protocol (Pi → Pico):  letter + 5-digit speed + newline
  S00000  →  stop
  F20000  →  move forward at duty 20000
  L15000  →  spin left  (CCW)
  R15000  →  spin right (CW)
  X12000  →  slow search spin

Save this as main.py on the Pico so it auto-starts on boot.
"""

from machine import PWM, Pin
import sys
import select
import time
import math

# ── Motor Board 1 ─────────────────────────────────────────────────────────────
AIN1 = PWM(Pin(0));  AIN1.freq(20000)
AIN2 = PWM(Pin(1));  AIN2.freq(20000)
SLP1 = Pin(2, Pin.OUT)
FLT1 = Pin(4, Pin.IN)

# ── Motor Board 2 ─────────────────────────────────────────────────────────────
FLT2 = Pin(13, Pin.IN)
BIN1 = PWM(Pin(8));  BIN1.freq(20000)
BIN2 = PWM(Pin(9));  BIN2.freq(20000)
SLP2 = Pin(10, Pin.OUT)
CIN1 = PWM(Pin(12)); CIN1.freq(20000)
CIN2 = PWM(Pin(11)); CIN2.freq(20000)

# ── Motor helpers ──────────────────────────────────────────────────────────────
def motors_enable():
    SLP1.value(1)
    SLP2.value(1)

def motors_disable():
    SLP1.value(0)
    SLP2.value(0)

def set_motor_a(duty):
    if duty >= 0: AIN1.duty_u16(duty);  AIN2.duty_u16(0)
    else:         AIN1.duty_u16(0);     AIN2.duty_u16(-duty)

def set_motor_b(duty):
    if duty >= 0: BIN1.duty_u16(duty);  BIN2.duty_u16(0)
    else:         BIN1.duty_u16(0);     BIN2.duty_u16(-duty)

def set_motor_c(duty):
    if duty >= 0: CIN1.duty_u16(duty);  CIN2.duty_u16(0)
    else:         CIN1.duty_u16(0);     CIN2.duty_u16(-duty)

def stop():
    set_motor_a(0)
    set_motor_b(0)
    set_motor_c(0)

# ── Omni-drive motion primitives ───────────────────────────────────────────────
# 3-wheel omni: wheels at 0°, 120°, 240° around the robot.
# For forward (angle=0): A gets cos(0°)=1.0, B gets cos(-120°)=-0.5, C gets cos(120°)=-0.5
# Scale B and C to half the duty of A to match force vectors.

def move_forward(speed):
    half = speed // 2
    set_motor_a(speed)
    set_motor_b(-half)
    set_motor_c(-half)

def move_backward(speed):
    half = speed // 2
    set_motor_a(-speed)
    set_motor_b(half)
    set_motor_c(half)

def spin_right(speed):
    """Clockwise (CW) spin in place — robot turns right."""
    set_motor_a(-speed)
    set_motor_b(speed)
    set_motor_c(speed)

def spin_left(speed):
    """Counter-clockwise (CCW) spin — robot turns left."""
    set_motor_a(speed)
    set_motor_b(-speed)
    set_motor_c(-speed)

# ── Serial command parsing ─────────────────────────────────────────────────────
# Non-blocking: we read available chars, buffer them, process complete lines.
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
rx_buf = ""

def read_serial():
    """Return a complete command line if one is available, else None."""
    global rx_buf
    while poll.poll(0):
        rx_buf += sys.stdin.read(1)
    if '\n' in rx_buf:
        line, rx_buf = rx_buf.split('\n', 1)
        return line.strip()
    return None

def parse_cmd(line):
    """Parse e.g. 'F20000' → ('F', 20000). Returns (None, 0) on bad input."""
    if len(line) >= 1:
        letter = line[0].upper()
        try:
            speed = int(line[1:]) if len(line) > 1 else 0
        except ValueError:
            speed = 0
        return letter, speed
    return None, 0

# ── Startup ────────────────────────────────────────────────────────────────────
time.sleep(3)    # let USB enumerate before we start listening
motors_enable()
print("movement_listener ready")

# ── Main loop ──────────────────────────────────────────────────────────────────
current_cmd   = 'S'
current_speed = 0

while True:
    # Check for new command from Pi
    line = read_serial()
    if line:
        cmd, speed = parse_cmd(line)
        if cmd in ('S', 'F', 'B', 'L', 'R', 'X'):
            current_cmd   = cmd
            current_speed = speed

    # Execute current command continuously (set each iteration so it keeps running)
    if current_cmd == 'S':
        stop()
    elif current_cmd == 'F':
        move_forward(current_speed)
    elif current_cmd == 'B':
        move_backward(current_speed)
    elif current_cmd == 'R':
        spin_right(current_speed)
    elif current_cmd == 'L':
        spin_left(current_speed)
    elif current_cmd == 'X':          # search — slow spin
        spin_right(current_speed)

    # Small yield to not hammer the CPU solid
    time.sleep_ms(10)
