
import os
import time
import pigpio
import multiprocessing

# Define pin numbers
reset_pin = 4
blue_led = 17
red_led = 27
green_led = 22
button_pin = 23

# Kill and restart pigpio daemon
os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
time.sleep(1)  # Wait for pigpio daemon to initialize

# Connect to pigpio daemon
pwm = pigpio.pi()
if not pwm.connected:
    print("❌ Could not connect to pigpio daemon.")
    exit(1)

# Set output pins
for pin in [reset_pin, blue_led, red_led, green_led]:
    pwm.set_mode(pin, pigpio.OUTPUT)
    pwm.write(pin, 0)  # Set initial LOW

# Set button pin as input with pull-up
pwm.set_mode(button_pin, pigpio.INPUT)
pwm.set_pull_up_down(button_pin, pigpio.PUD_UP)

# Example function that might use the pins
def check_button():
    while True:
        button_state = pwm.read(button_pin)
        if button_state == 0:
            print("Button pressed")
        time.sleep(0.1)

# Launch button check in a separate process (example)
if __name__ == "__main__":
    try:
        p = multiprocessing.Process(target=check_button)
        p.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
        p.terminate()
        p.join()
        pwm.stop()
