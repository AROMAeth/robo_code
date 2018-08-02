

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
control_pins = [7,11,13,15]

for pin in control_pins:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

halfstep_seq = [
  [1,0,0,0],
  [1,1,0,0],
  [0,1,0,0],
  [0,1,1,0],
  [0,0,1,0],
  [0,0,1,1],
  [0,0,0,1],
  [1,0,0,1]
]

# speed from 0 to 1 (one being the fastest)
# steps 50 steps = one rotation
def move_backward(steps, speed):
  for i in range(steps):
    for halfstep in range(8):
      for pin in range(4):
        GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
      time.sleep(max(0.001/speed,0.001))

def move_forward(steps, speed):
  for i in range(steps):
    for halfstep in range(7,-1,-1):
      for pin in range(4):
        GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
      time.sleep(max(0.001/speed,0.001))



for k in range(1,10,1):
	move_forward(50,0.1)

time.sleep(0.5)
#move_forward(50,0.25)
time.sleep(1)
#move_backward(500,0.5)


GPIO.cleanup()
