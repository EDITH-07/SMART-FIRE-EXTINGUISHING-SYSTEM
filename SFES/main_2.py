from time import sleep
import RPi.GPIO as GPIO
import time

#########CV2###########
from ultralytics import YOLO
import cv2
import math
import json
import numpy as np

# IP camera stream URL
ip_camera_url = 'rtsp://admin@192.168.1.106:554/'

# Running real-time from IP camera
cap = cv2.VideoCapture(ip_camera_url)
model = YOLO('9k_y5_small.pt')

# Reading the classes
classnames = ['fire', 'Fire', 'smoke']

confidence_threshold = 0.6  # Set the confidence threshold to 60%

# Object distance variables
focal = 450
pixels = 30
width = 4

# Define helper function to calculate distance
def calculate_distance(pixels):
    return (width * focal) / pixels

# Initialize coordinate lists
x_values = []
y_values = []
z_values = []

# Initialize counter variable
coordinate_counter = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    result = model(frame, stream=True)

    # Getting bbox, confidence, and class names information to work with
    for info in result:
        boxes = info.boxes
        for box in boxes:
            confidence = box.conf[0]
            confidence = math.ceil(confidence * 100)
            Class = int(box.cls[0])
            if confidence > confidence_threshold * 100:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 5)
                if Class < len(classnames):  # Check if Class is within valid range
                    class_name = classnames[Class]

                    # Calculate and display distance
                    pixels_covered = x2 - x1
                    distance = calculate_distance(pixels_covered)

                    # Append coordinates to the respective lists
                    x_values.append(x1)
                    y_values.append(y1)
                    z_values.append(distance)

                    coordinate_counter += 1

                    label = f'{class_name} {confidence}%'
                    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(frame, (x1, y1), (x1 + label_width, y1 - label_height - 10), (0, 0, 255), -1)
                    cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    distance_label = f'Distance: {round(distance, 2)} cm'
                    cv2.putText(frame, distance_label, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow('frame', frame)
    cv2.waitKey(1)

    # Break the loop if the desired number of coordinates is reached6
    if coordinate_counter >= 10:
        break

# Print the coordinate lists
print("X Values:", x_values)
print("Y Values:", y_values)
print("Z Values:", z_values)

# Release the IP camera stream and close the window
cap.release()
cv2.destroyAllWindows()


X, Y = 0, 0
avg_x = 0
avg_y = 0 
for i in range(len(x_values)):
    X += x_values[i]
    Y += y_values[i]

avg_x = X / 10
avg_y = Y / 10

print("X: ", avg_x)
print("Y: ", avg_y)


#########MOTOR#########

PUL = 17  # Stepper Drive Pulses
DIR = 27  # Controller Direction Bit (High for Controller default / LOW to Force a Direction Change).
ENA = 22  # Controller Enable Bit (High to Enable / LOW to Disable).
DIRI = 14  # Status Indicator LED - Direction
ENAI = 15  # Status indicator LED - Controller Enable

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(DIRI, GPIO.OUT)
GPIO.setup(ENAI, GPIO.OUT)



print('Initialization Completed')

# Motor configuration
steps_per_unit = 100  # Number of steps per unit of movement in the X-axis
if(avg_x<=200):
    if(avg_x<=100):
        a=50
    elif(avg_x<=150):
        a=62.5
elif(avg_x<=400):
    if(avg_x<=300):
        a=75
    else:
        a=87.5
elif(avg_x==400):
    a=100
elif(avg_x<=600):
    if(avg_x<=450):
        a=112.5
    elif(avg_x<=500):
        a=125
elif(avg_x<=550):
    if(avg_x<=300):
        a=150
    else:
        a=200
        
# Get the desired coordinate from the user (within the range of 0 to 640)
coordinate = (a)
y_Axis = avg_y
# Validate the coordinate to ensure it is within the range
coordinate = max(0, min(200, coordinate))

# Calculate the distance to move in the X-axis based on the coordinate
distance = coordinate / steps_per_unit

# Calculate the number of steps required
steps = int(distance * steps_per_unit)

delay = 0.0025  # Delay between PUL pulses - controls motor speed

GPIO.output(ENA, GPIO.HIGH)
GPIO.output(ENAI, GPIO.HIGH)
print('ENA set to HIGH - Controller Enabled')

# Set the direction based on the distance
if distance >= 0:
    GPIO.output(DIR, GPIO.LOW)
    GPIO.output(DIRI, GPIO.LOW)
    print(f'DIR set to LOW - Moving Forward at {delay}')
else:
    GPIO.output(DIR, GPIO.HIGH)
    GPIO.output(DIRI, GPIO.HIGH)
    print(f'DIR set to HIGH - Moving Backward at {delay}')

print(f'Moving to coordinate {coordinate} units')

# Step the motor
for _ in range(steps):
    GPIO.output(PUL, GPIO.HIGH)
    sleep(delay)
    GPIO.output(PUL, GPIO.LOW)
    sleep(delay)

# Configure the GPIO mode
GPIO.setmode(GPIO.BCM)

# Specify the GPIO pin number
servo_pin = 18

# Set the PWM frequency and duty cycle limits
pwm_frequency = 50  # 50 Hz
duty_cycle_min = 2.5  # in percentage (left position)
duty_cycle_max = 12.5  # in percentage (right position)

# Calculate the duty cycle range
duty_cycle_range = duty_cycle_max - duty_cycle_min

# Configure the GPIO pin for PWM
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, pwm_frequency)

# Function to convert a coordinate to a duty cycle value
def convert_coordinate_to_duty_cycle(coordinate):
    # Normalize the coordinate to a value between 0 and 1
    normalized_coordinate = (coordinate - axis_min) / (axis_max - axis_min)
    # Calculate the duty cycle value based on the normalized coordinate
    duty_cycle = duty_cycle_min + (normalized_coordinate * duty_cycle_range)
    return duty_cycle

# Example coordinates for the axis
axis_min = 480  # Minimum coordinate value
axis_max = 0  # Maximum coordinate value

if(y_Axis<=240):
    b=240
elif(y_Axis<=360):
    b=240
elif(y_Axis<=480):
    b=240
# Example target coordinates
target_coordinates = [b]

# Move the servo motor to the target coordinates
try:
    # Start the PWM
    pwm.start(duty_cycle_min)
    
    for coordinate in target_coordinates:
        # Convert the coordinate to a duty cycle value
        duty_cycle = convert_coordinate_to_duty_cycle(coordinate)
        # Move the servo motor to the specified position
        pwm.ChangeDutyCycle(duty_cycle)
        # Wait for a brief moment
        time.sleep(3)

    # Stop the PWM
    pwm.stop()

finally:
    print(" ")

###########RELAY################
# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin connected to the relay
relay_pin = 23

# Setup the relay pin as an output
GPIO.setup(relay_pin, GPIO.OUT)

try:
    # Turn on the relay
    GPIO.output(relay_pin, GPIO.HIGH)
    print("Relay turned on.")
    
    # Wait for 5 seconds
    time.sleep(10)
    
    # Turn off the relay
    GPIO.output(relay_pin, GPIO.LOW)
    print("Relay turned off.")
    
finally:
    print(" ")

# Reverse the direction
GPIO.output(DIR, not GPIO.input(DIR))
GPIO.output(DIRI, not GPIO.input(DIRI))
print('Reversing Direction')

# Calculate the number of steps to move back to the original position
steps_back = steps

print('Moving back to the original position')

# Step the motor back
for _ in range(steps_back):
    GPIO.output(PUL, GPIO.HIGH)
    sleep(delay)
    GPIO.output(PUL, GPIO.LOW)
    sleep(delay)

GPIO.cleanup()
print('Movement Completed')