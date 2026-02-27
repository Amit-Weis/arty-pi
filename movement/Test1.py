from machine import Pin, PWM  
import time

# Global Vars
bits = 65535

# Board 1
SLP1 = Pin(5, Pin.OUT)
FLT1 = Pin(0, Pin.IN, Pin.PULL_DOWN)

# Motor A
AIN1 = PWM(Pin(26))
AIN2 = PWM(Pin(27))
ENC_A_A = Pin(15, Pin.IN, Pin.PULL_UP)
ENC_A_B = Pin(14, Pin.IN, Pin.PULL_UP)

# Motor B
BIN1 = PWM(Pin(28))
BIN2 = PWM(Pin(6))
ENC_B_A = Pin(16, Pin.IN, Pin.PULL_UP)
ENC_B_B = Pin(17, Pin.IN, Pin.PULL_UP)

# Board 2
SLP2 = Pin(9, Pin.OUT)
FLT2 = Pin(11, Pin.IN, Pin.PULL_DOWN) 

# Motor C 
CIN1 = PWM(Pin(4)) 
CIN2 = PWM(Pin(7))   
ENC_C_A = Pin(13, Pin.IN, Pin.PULL_UP)  
ENC_C_B = Pin(12, Pin.IN, Pin.PULL_UP)  

# LED
LED = Pin(25, Pin.OUT)

# PWM Frequencies
for pwm in [AIN1, AIN2, BIN1, BIN2, CIN1, CIN2]:
    pwm.freq(20000)

# Encoder Counts
encoder_a = 0
encoder_b = 0
encoder_c = 0

#
def encoder_a_isr(pin): 
    global encoder_a
    if ENC_A_A.value() == ENC_A_B.value():
        encoder_a += 1 
    else:
        encoder_a -= 1

def encoder_b_isr(pin):
    global encoder_b
    if ENC_B_A.value() == ENC_B_B.value():
        encoder_b += 1
    else:
        encoder_b -= 1

def encoder_c_isr(pin):  
    global encoder_c
    if ENC_C_A.value() == ENC_C_B.value(): 
        encoder_c += 1
    else:
        encoder_c -= 1

ENC_A_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_a_isr)
ENC_B_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_b_isr)
ENC_C_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_c_isr)


def stop():
    AIN1.duty_u16(0)
    AIN2.duty_u16(0)
    BIN1.duty_u16(0)
    BIN2.duty_u16(0)
    CIN1.duty_u16(0)
    CIN2.duty_u16(0)
    LED.value(0)
    SLP1.value(0)
    SLP2.value(0)
    print ("END")
    
def pause():
    AIN1.duty_u16(0)
    AIN2.duty_u16(0)
    BIN1.duty_u16(0)
    BIN2.duty_u16(0)
    CIN1.duty_u16(0)
    CIN2.duty_u16(0)

def checkFault():
    if FLT1.value() == 1 or FLT2.value() == 1:
        stop()
        return True
    return False

FLT1.irq(trigger=Pin.IRQ_RISING, handler=lambda pin: stop())
FLT2.irq(trigger=Pin.IRQ_RISING, handler=lambda pin: stop())

def motorAForward(speed_prcnt):
    if checkFault(): return
    duty = int((speed_prcnt / 100) * bits)
    AIN1.duty_u16(0)
    AIN2.duty_u16(duty)   

def motorAReverse(speed_prcnt):
    if checkFault(): return
    duty = int((speed_prcnt / 100) * bits)
    AIN1.duty_u16(duty)  
    AIN2.duty_u16(0)

def motorBForward(speed_prcnt):
    if checkFault(): return
    duty = int((speed_prcnt / 100) * bits)
    BIN1.duty_u16(0)
    BIN2.duty_u16(duty)  

def motorBReverse(speed_prcnt):
    if checkFault(): return
    duty = int((speed_prcnt / 100) * bits)
    BIN1.duty_u16(duty)   
    BIN2.duty_u16(0)

def motorCForward(speed_prcnt):
    if checkFault(): return
    duty = int((speed_prcnt / 100) * bits)
    CIN1.duty_u16(0)
    CIN2.duty_u16(duty)   

def motorCReverse(speed_prcnt):
    if checkFault(): return
    duty = int((speed_prcnt / 100) * bits)
    CIN1.duty_u16(duty)   
    CIN2.duty_u16(0)

#Start
SLP1.value(1)
SLP2.value(1)  
LED.value(1)
print("Test sequence begins in 1s")
time.sleep(1)

try:
    while True:
        print("Motor A: Forward")
        for speed in range(0, 101, 5):  
            if checkFault(): break
            motorAForward(speed)
            print(f"Forward {speed}% | Enc A: {encoder_a}")  
            time.sleep(0.1)
        time.sleep(1)
        pause()
        time.sleep(0.5)

        print("Motor A: Reverse")
        for speed in range(0, 101, 5): 
            if checkFault(): break
            motorAReverse(speed)
            print(f"Reverse {speed}% | Enc A: {encoder_a}")  
            time.sleep(0.1)
        time.sleep(1)
        pause()
        time.sleep(0.5)

        print("Motor B: Forward")
        for speed in range(0, 101, 5): 
            if checkFault(): break
            motorBForward(speed)
            print(f"Forward {speed}% | Enc B: {encoder_b}")
            time.sleep(0.1)
        time.sleep(1)
        pause()
        time.sleep(0.5)

        print("Motor B: Reverse")
        for speed in range(0, 101, 5): 
            if checkFault(): break
            motorBReverse(speed)
            print(f"Reverse {speed}% | Enc B: {encoder_b}") 
            time.sleep(0.1)
        time.sleep(1)
        pause()
        time.sleep(0.5)

        print("Motor C: Forward")
        for speed in range(0, 101, 5):  
            if checkFault(): break
            motorCForward(speed)
            print(f"Forward {speed}% | Enc C: {encoder_c}")  
            time.sleep(0.1)
        time.sleep(1)
        pause()
        time.sleep(0.5)

        print("Motor C: Reverse")
        for speed in range(0, 101, 5): 
            if checkFault(): break
            motorCReverse(speed)
            print(f"Reverse {speed}% | Enc C: {encoder_c}") 
            time.sleep(0.1)
        time.sleep(1)
        pause()
        time.sleep(0.5)

except KeyboardInterrupt:
    stop()
    SLP1.value(0) 
    SLP2.value(0)
    print("Stopped cleanly.")