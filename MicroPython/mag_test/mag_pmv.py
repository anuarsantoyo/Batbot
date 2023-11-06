from machine import Pin, Timer
import time
# Define the GPIO pin connected to the PWM output of the sensor
pwm_pin = Pin('Y4', Pin.IN)

# Initialize the PWM object

def pulse_in(pin, value):
    print("Waiting for pulse:", value)  # Debug print
    start_time = time.ticks_us()

    while pin.value() != value:
        pass

    while pin.value() == value:
        pass

    duration = time.ticks_diff(time.ticks_us(), start_time)
    print("Pulse duration for", value, ":", duration)  # Debug print
    return duration
def read_pwm_duty(pin):
    high_time = pulse_in(pin, 1)
    low_time = pulse_in(pin, 0)
    total_time = high_time + low_time

    # Calculate duty cycle
    duty_cycle = high_time / total_time
    return duty_cycle
def pwm_to_angle(duty_cycle):
    # Assuming 0% duty cycle = 0 degrees and 100% duty cycle = 360 degrees
    return 360 * duty_cycle

duty_cycle = read_pwm_duty(pwm_pin)
angle = pwm_to_angle(duty_cycle)
print("Measured Angle:", angle)
