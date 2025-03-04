#! /usr/bin/python3

from gpiozero import PhaseEnableMotor, RotaryEncoder 
import time
import numpy as np
import traceback
# import yaml
# import encoders

## Define which serial port your sensors are on: 
#serial_device_name = '/dev/ttyS0'
#serial_device_name = '/dev/ttyUSB0'

## Define a temporary function using Python "lambda" functionality to print colored text
# see https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal/3332860#3332860
# search that page for "CircuitSacul" to find that answer
colored = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'

# Define the Motors: 
e0 = RotaryEncoder(17,27,max_steps=0,bounce_time=None)
e1 = RotaryEncoder(23,24,max_steps=0,bounce_time=None)
m0 = PhaseEnableMotor(5,12)
m1 = PhaseEnableMotor(6,13)

def testMotors(left_dir,right_dir,speed): #Runs the motors for ~2 seconds
    cmd0 = left_dir*speed
    if cmd0 > 0:
        m0.forward(cmd0)
    else: 
        m0.backward(-cmd0)
    
    cmd1 = right_dir*speed
    if cmd1 > 0: 
        m1.forward(cmd1)
    else: 
        m1.backward(-cmd1)

    time.sleep(2.0)
    m0.stop()
    m1.stop()
    time.sleep(.5)

def getEncoderRawValues(): #clears out the serial port and waits for the next encoder values
    leftEnc = e0.steps
    rightEnc = e1.steps
    return [leftEnc,rightEnc]

ENCODER_MOVE_THRESH = 250;  #counts to move before encoder is considered 'moved'

#states for test.  MOTOR
MOTOR_SPIN_LEFT = 0
MOTOR_SPIN_RIGHT = 1
MOTOR_DIR = 2
ENCODER_DIR = 3

test_over = False;
test_passed = True;

#current state for the test..  Starts at MOTOR_SPIN_LEFT
state = MOTOR_SPIN_LEFT

# # # with open("robot_info.yaml", 'r') as stream:
    # # # robot_info = yaml.safe_load(stream)

print(colored(100,200,200,"Starting test in 5 seconds.  Please place your robot on the ground."))
time.sleep(5.0)

robot_info = []

# with open("robot_info.yaml", 'r') as stream:
    # robot_info = yaml.safe_load(stream)

# left_motor_dir = robot_info["left_motor_sign"];
# right_motor_dir = robot_info["right_motor_sign"];
# left_encoder_dir = robot_info["left_encoder_sign"];
# right_encoder_dir = robot_info["right_encoder_sign"];
left_motor_dir = 1
right_motor_dir = -1
left_encoder_dir = 1
right_encoder_dir = -1

while not test_over:
    if (state == MOTOR_SPIN_LEFT):
        print(colored(255,100,10,"Testing Left Motor Connections"))
        testMotors(left_motor_dir,0,0.2)
        txt = input(colored(10,150,255,"did the left motor spin? (y/n)  "))
        if (txt == "y"):
            print(colored(0,200,0,"left motor connection good."))
            state = MOTOR_SPIN_RIGHT;
            continue;
        elif (txt == "n"):
            print(colored(255,0,0,"Hardware problem: Left Motor: Check all motor connections and try again (if connections look good, ask a TA for help)"))
            test_passed = False;
            break;
    elif (state == MOTOR_SPIN_RIGHT):
        print(colored(255,100,10,"Testing Right Motor Connections"))
        testMotors(0,right_motor_dir,0.2)
        txt = input(colored(10,150,255,"did the right motor spin? (y/n) "))
        if (txt == "y"):
            print(colored(0,200,0,"right motor connection good."))
            state = MOTOR_DIR;
            continue;
        elif (txt == "n"):
            print(colored(255,0,0,"Hardware problem: Right Motor: Check all motor connections and try again (if connections look good, ask a TA for help)"))
            test_passed = False;
            break;
    elif (state == MOTOR_DIR):
        print(colored(255,10,10,"Testing Motor Directions..."))
        testMotors(left_motor_dir,right_motor_dir,0.2)
        txt = input(colored(10,150,255,"did the robot move forward(ish)? (y/n)  "))
        if (txt == "y"):
            print(colored(0,200,0,"Motor signs good."))
            state = ENCODER_DIR
            continue;
        elif (txt == "n"):
            print(colored(255,0,0,"Please flip the signs of left_motor_sign or right_motor_sign in robot_params.yaml for the motors that drove backwards -OR- you can swap the M1 and M2 connections on that motor.  Re-run this script to test again."))
            test_passed = False;
            break;
    elif (state == ENCODER_DIR): #this just tests the encoder direction. pure code.
        print(colored(255,100,10,"Testing Encoders..."))
        [e0_init,e1_init] = getEncoderRawValues();
        testMotors(left_motor_dir,0,0.2);  #move left motor only
        [e0_left,e1_left] = getEncoderRawValues();
        testMotors(0,right_motor_dir,0.2); #move right motor only
        [e0_right,e1_right] = getEncoderRawValues();
        print(colored(180,180,0,"encoder cnts: " + str(e0_init) + "," + str(e0_left) + "," + str(e0_right) + " " + str(e1_init) + "," + str(e1_left) + "," + str(e1_right)))

        if ((e0_left-e0_init)*left_encoder_dir) > ENCODER_MOVE_THRESH:
            print(colored(0,200,0,"left encoder good."))
        elif ((e0_left-e0_init)*left_encoder_dir) < -ENCODER_MOVE_THRESH:
            print(colored(255,0,0,"change left_encoder_sign in robot_info.yaml -OR- swap the A and B leads on the left motor. Re-run this script to test again."))
            test_passed = False;
        else: #encoder hasn't moved much.. probably a hardware problem.
            print(colored(255,0,0,"Hardware problem: low encoder count change: check left encoder wiring and try again (if connections look good, ask a TA for help)"))
            test_passed = False;

        if ((e1_right-e1_init)*right_encoder_dir) > ENCODER_MOVE_THRESH:
            print(colored(0,200,0,"right encoder good."))
        elif ((e1_right-e1_init)*right_encoder_dir) < -ENCODER_MOVE_THRESH:
            print(colored(255,0,0,"change right_encoder_sign in robot_info.yaml -OR- swap the A and B leads on the right motor. Re-run this script to test again."))
            test_passed = False;
        else: #encoder hasn't moved much.. probably a hardware problem.
            print(colored(255,0,0,"Hardware problem: low encoder count change: check right encoder wiring and try again (if connections look good, ask a TA for help)"))
            test_passed = False;    
        test_over = True;


if (test_passed):
    print(colored(0,255,0,"\nTest PASSED! your motors and encoders agree and are going the correct direction!\n"))
else:
    print(colored(255,0,0,"\nFAILED.  Please take recommended action and try again!\n"))
