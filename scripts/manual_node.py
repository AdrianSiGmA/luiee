#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

buzzer_value_prev = 0
LED_green_value_prev = 0
LED_red_value_prev = 0
LED_blue_value_prev = 0
manualDataArray = [0,0,0,0,0,0]
# [mot_right_pwm,mot_left_pwm,LED_green,LED_red,LED_blue,buzzer]

# =================== CALLBACK de ROS ==============================
def CallBack_Buzzer(buzzer_data):
    global buzzer_value_prev

    if buzzer_data.data == 1 and buzzer_value_prev == 0:
        manualDataArray[5] = 255
        buzzer_value_prev = 1
    elif buzzer_data.data == 0 and buzzer_value_prev == 1:
        manualDataArray[5] = 0
        buzzer_value_prev = 0

def CallBack_LED_green(LED_green_data):
    global LED_green_value_prev

    if LED_green_data.data == 1 and LED_green_value_prev == 0:
        manualDataArray[2] = 255
        LED_green_value_prev = 1
    elif LED_green_data.data == 0 and LED_green_value_prev == 1:
        manualDataArray[2] = 0
        LED_green_value_prev = 0

def CallBack_LED_red(LED_red_data):
    global LED_red_value_prev

    if LED_red_data.data == 1 and LED_red_value_prev == 0:
        manualDataArray[3] = 255
        LED_red_value_prev = 1
    elif LED_red_data.data == 0 and LED_red_value_prev == 1:
        manualDataArray[3] = 0
        LED_red_value_prev = 0

def CallBack_LED_blue(LED_blue_data):
    global LED_blue_value_prev

    if LED_blue_data.data == 1 and LED_blue_value_prev == 0:
        manualDataArray[4] = 255
        LED_blue_value_prev = 1
    if LED_blue_data.data == 0 and LED_blue_value_prev == 1:
        manualDataArray[4] = 0
        LED_blue_value_prev = 0

def CallBack_Joystick(joystick_data):
    
    joystick_x_value = joystick_data.angular.z
    joystick_y_value = joystick_data.linear.x
    V = (1-abs(joystick_x_value))*(joystick_y_value)+joystick_y_value
    W = (1-abs(joystick_y_value))*joystick_x_value+joystick_x_value
    R = (V+W)/2
    L = (V-W)/2

    manualDataArray[0] = int(200*R);
    manualDataArray[1] = int(200*L);

# ----------------- Publisher and Subscriber definition --------------------
# Name of the node. False = no random numbers
rospy.init_node('manual_node', anonymous=False)
print("Initializing manual_node...")

# Published definition. Int32MultiArray because of PWM (0-255 int)
pub = rospy.Publisher('/mini_luiee_topic', Int32MultiArray, queue_size=10)

# Subscriber definition.
rospy.Subscriber('/buzzer_btn', Bool, CallBack_Buzzer)
rospy.Subscriber('/LED_green_btn', Bool, CallBack_LED_green)
rospy.Subscriber('/LED_red_btn', Bool, CallBack_LED_red)
rospy.Subscriber('/LED_blue_btn', Bool, CallBack_LED_blue)
rospy.Subscriber('/cmd_vel', Twist, CallBack_Joystick)

# Data definition
manual_data = Int32MultiArray()

# Program frequency
rate = rospy.Rate(50)

    # ======================== While loop ===================================
def main():
    while not rospy.is_shutdown():
        manual_data.data = manualDataArray
        pub.publish(manual_data)
        rate.sleep()

# ================= LOOP =========================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
