from machine import Pin, PWM 
import time
        
SLP2 = Pin(9, Pin.OUT)

SLP2.value(1)

try:
    while True:
        SLP2.value(1)
        
except KeyboardInterrupt:
    SLP2.value(0)
    print("Stopped.")