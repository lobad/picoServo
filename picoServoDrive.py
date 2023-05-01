from machine import Pin, PWM, ADC
import utime
from simple_pid import PID
from machine import Timer

# Global variables
prev_pulse_time = utime.ticks_us()
pulse_interval = None
timer = Timer(-1)
# Potentiometer pin
potentiometer = ADC(Pin(27))

# L298 motor driver pins
in1 = Pin(4, Pin.OUT)
in2 = Pin(5, Pin.OUT)
ena = PWM(Pin(6))
ena.freq(100)

# GRBL CNC shield pins
step_pin = Pin(0, Pin.IN, Pin.PULL_DOWN)  # Step input
dir_pin = Pin(1, Pin.IN, Pin.PULL_DOWN)  # Direction input

# Onboard LED
led = Pin("LED", Pin.OUT)

def read_potentiometer_position():
    return potentiometer.read_u16()  # Returns a value between 0 and 65535

# Motor control variables
target_position = read_potentiometer_position()  # Desired position
print("Initial potentiometer value:", target_position)
Kp = 200  # Proportional gain
Ki = 0  # Integral gain
Kd = 1000  # Derivative gain
steps_per_rev = 200  # Number of steps per revolution for DC motor

pid = PID(Kp, Ki, Kd, setpoint=target_position, output_limits=(-65535, 65535))

def set_motor_speed(speed):
    if speed > 0:
        in1.value(1)
        in2.value(0)
        ena.duty_u16(speed)
    elif speed < 0:
        in1.value(0)
        in2.value(1)
        ena.duty_u16(-speed)
    else:
        in1.value(0)
        in2.value(0)
        ena.duty_u16(0)

prev_potentiometer_position = None

def pid_control_loop():
    global pid, target_position, prev_potentiometer_position
    
    potentiometer_position = read_potentiometer_position()
    # print("Potentiometer value:", potentiometer_position)  # Print potentiometer value
    
    if prev_potentiometer_position is not None:
        position_diff = abs(potentiometer_position - prev_potentiometer_position)
        # print("Position difference:", position_diff)

    # Set the target position based on some external input or requirement
    # For example, use a second potentiometer to set the target position
    # target_position = read_second_potentiometer_position()
    
    pid.setpoint = target_position
    speed = int(pid(potentiometer_position))
    # print("Motor speed:", speed)
    # print("potVal:", potentiometer_position)
    set_motor_speed(speed)
    
    prev_potentiometer_position = potentiometer_position


def blink_led(times, delay_ms):
    for _ in range(times):
        led.value(1)
        utime.sleep_ms(delay_ms)
        led.value(0)
        utime.sleep_ms(delay_ms)
        
def reset_pulse_interval(timer):
    global pulse_interval
    pulse_interval = None
    pid.output_limits = (-65535, 65535)
    
# Define steps_per_inch based on your motor and linear motion system
steps_per_inch = (30000 * 0) / (10)


def step_isr(pin):
    global target_position, prev_pulse_time, pulse_interval
    
    current_time = utime.ticks_us()
    pulse_interval = utime.ticks_diff(current_time, prev_pulse_time)
    prev_pulse_time = current_time
    
    steps = 1  # Define the number of steps per interrupt
    direction = dir_pin.value()  # Read the direction pin value (0 or 1)

    # Set the target position based on the direction
    if direction:
        target_position += steps * (65535 // steps_per_rev)
    else:
        target_position -= steps * (65535 // steps_per_rev)

    # Print the received signal, direction, and target position
    print("Received signal on step pin")
    print("Direction:", "Forward" if direction else "Backward")
    print("Target position:", target_position)

    led.value(1)  # Turn LED on when receiving signal
    led.value(0)  # Turn LED off after processing signal

    # Update the motor speed based on the feed rate (in inches per minute)
    if pulse_interval is not None:
        # Convert pulse interval to feed rate
        feed_rate = 1_000_000 / pulse_interval  # inches per minute
        
        # Convert feed rate to steps per second
        steps_per_second = (feed_rate * steps_per_inch) / 60

        # Convert steps per second to motor speed
        motor_speed = int(steps_per_second * (65535 // steps_per_rev))
        set_motor_speed(motor_speed)
        
# Blink the LED five times when the program starts
blink_led(2, 550)
print("connected")

# Configure the step pin to trigger an interrupt on rising edge
step_pin.irq(trigger=Pin.IRQ_RISING, handler=step_isr)

# Self-tuning variables
tuning_iterations = 2  # Move the motor back and forth twice
tuning_step_size = 500  # Set a small value for the motor movement

def self_tuning():
    print("starting self-tuning")
    global target_position, Kp, Ki, Kd

    for i in range(tuning_iterations):
        initial_position = read_potentiometer_position()
        target_position += tuning_step_size
        print(f"Moving motor {tuning_step_size} steps forward in iteration {i + 1}")
        
        while abs(read_potentiometer_position() - target_position):
            pid_control_loop()
            utime.sleep_ms(50)  # Adjust the delay as needed
        
        final_position = read_potentiometer_position()
        diff = abs(final_position - initial_position)

        print(f"Potentiometer value: {initial_position} -> {final_position}")

        # Adjust the PID gains based on the difference
        Kp *= diff / tuning_step_size
        Ki *= diff / tuning_step_size
        Kd *= diff / tuning_step_size
        print(f"Updated PID gains: Kp={Kp}, Ki={Ki}, Kd={Kd}")

        initial_position = final_position
        target_position -= tuning_step_size
        print(f"Moving motor {tuning_step_size} steps backward in iteration {i + 1}")
        
        while abs(read_potentiometer_position() - target_position):
            pid_control_loop()
            utime.sleep_ms(50)  # Adjust the delay as needed
        
        final_position = read_potentiometer_position()
        diff = abs(final_position - initial_position)

        print(f"Potentiometer value: {initial_position} -> {final_position}")

        # Adjust the PID gains based on the difference
        Kp *= diff / tuning_step_size
        Ki *= diff / tuning_step_size
        Kd *= diff / tuning_step_size
        print(f"Updated PID gains: Kp={Kp}, Ki={Ki}, Kd={Kd}")

        # Update the PID controller with new gains
        pid = PID(Kp, Ki, Kd, setpoint=target_position, output_limits=(-65535, 65535))


def move_motor_steps(steps, direction):
    global target_position
    step_size = 65535 // steps_per_rev

    if direction:
        target_position += steps * step_size
    else:
        target_position -= steps * step_size

    while abs(read_potentiometer_position() - target_position):
        pid_control_loop()
        utime.sleep_ms(50)  # Adjust the delay as needed


def main_loop():
    while True:
        pid_control_loop()
        utime.sleep_ms(50)  # Adjust the delay as needed
        
# Blink the LED seven times when the program starts
blink_led(5, 100)

# Perform self-tuning
# self_tuning()

# Start the main loop
print("I am a servo now")
main_loop()
