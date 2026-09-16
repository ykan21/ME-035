from machine import Pin
import neopixel
import time

# Initialize Neopixels on Pin 15 (2 LEDs total)
lights = neopixel.NeoPixel(Pin(15), 2)

# Define buttons on GPIO 34 and GPIO 35 (Input-only pins)
button1 = Pin(34, Pin.IN)
button2 = Pin(35, Pin.IN)

# Track current state for each light (True = ON, False = OFF)
light0_state = False
light1_state = False

# Previous button states to detect new presses (active LOW -> default unpressed is 1)
last_b1_state = 1
last_b2_state = 1

# Turn both LEDs off initially
lights[0] = (0, 0, 0)
lights[1] = (0, 0, 0)
lights.write()

while True:
    b1_current = button1.value()
    b2_current = button2.value()

    # Detect Button 1 press (Transition from 1 to 0)
    if last_b1_state == 1 and b1_current == 0:
        light0_state = not light0_state  # Toggle light state
        
        if light0_state:
            lights[0] = (20, 0, 20)  # Turn light 0 purple
        else:
            lights[0] = (0, 0, 0)     # Turn light 0 off
            
        lights.write()
        time.sleep_ms(50)  # Debounce delay

    # Detect Button 2 press (Transition from 1 to 0)
    if last_b2_state == 1 and b2_current == 0:
        light1_state = not light1_state  # Toggle light state
        
        if light1_state:
            lights[1] = (20, 0, 20)  # Turn light 1 purple as well
        else:
            lights[1] = (0, 0, 0)     # Turn light 1 off
            
        lights.write()
        time.sleep_ms(50)  # Debounce delay

    # Save state for the next check cycle
    last_b1_state = b1_current
    last_b2_state = b2_current

    time.sleep_ms(10)  # Short pause to keep CPU usage low