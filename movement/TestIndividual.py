from machine import Pin, PWM 
import time
        
SLP2 = Pin(, Pin.OUT)
2
SLP2.value(1)

print("ON")

time.sleep(20)

SLP2.value(0)

print("OFF")