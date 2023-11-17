import gc
from pyb import Servo, Accel, Timer, LED
import pyb

# Constants for PID
Kp = 3.0  # Proportional gain
Ki = 0  # Integral gain
Kd = 0  # Derivative gain

# Initialize the Servo on pin X1
servo = Servo(1)

# Initialize the accelerometer
accel = Accel()

# The target setpoint for the accelerometer
setpoint = 0

# PID control variables
integral = 0
previous_error = 0
pid_flag = False

# PID control function to be called within the main loop
def calculate_pid():
    global integral, previous_error, setpoint

    # Read the accelerometer value on one axis (e.g., x-axis)
    sensor_value = accel.x()

    # Calculate the error
    error = setpoint - sensor_value

    # Proportional term
    P = Kp * error

    # Integral term
    integral += error
    I = Ki * integral

    # Derivative term
    D = Kd * (error - previous_error)

    # Calculate the control variable
    control_variable = P + I + D

    # Make sure the control variable is within the servo's valid range
    control_variable = max(min(control_variable, 90), -90)

    # Update the previous error
    previous_error = error

    # Set the control variable to the servo
    print(control_variable, error)
    servo.angle(control_variable)

# Timer interrupt callback function
def pid_timer_callback(timer):
    global pid_flag
    pid_flag = True

# Main function
def main():
    global pid_flag  # Declare pid_flag as global to modify it within this function

    # Explicitly invoke garbage collection
    gc.collect()

    # Setup the timer with a lower frequency
    timer = Timer(2, freq=5)  # Adjust the frequency as necessary
    timer.callback(pid_timer_callback)

    while True:
        if pid_flag:
            calculate_pid()  # Perform PID calculation
            pid_flag = False  # Reset the flag

        # Other tasks can be performed here
        pyb.delay(10)  # Delay to prevent locking up the processor

# Run the main function
main()

