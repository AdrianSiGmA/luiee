#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

Kx = 80
K_saturation = 100
NV = 127
Kx_aruco = 80
K_saturation_aruco = 100
NV_aruco = 127
aruco_marker_array = [0,0,0]
max_sat_z = 1.3
max_sat_flag = 0
max_sat_flag_aruco = 0


buzzer_value_prev = 0
LED_green_value_prev = 0
LED_red_value_prev = 0
LED_blue_value_prev = 0
DataArray = [0,0,0,0,0,0]
# [mot_right_pwm,mot_left_pwm,LED_green,LED_red,LED_blue,buzzer]

# =================== CALLBACK de ROS ==============================
def CallBack_arUco(aruco_marker_data):
    global max_sat_flag
    global max_sat_flag_aruco

    # READ from the color_marker_topic
    aruco_marker_array[0] = aruco_marker_data.transforms[0].transform.translation.x;
    aruco_marker_array[1] = aruco_marker_data.transforms[0].transform.translation.y;
    aruco_marker_array[2] = aruco_marker_data.transforms[0].transform.translation.z;

    if aruco_marker_array[2] > max_sat_z and max_sat_flag == 0:  # still not saturated enough
        DataArray[1] = int(NV - aruco_marker_array[0] * Kx - (1/aruco_marker_array[2]) * K_saturation);
        DataArray[0] = int(NV + aruco_marker_array[0] * Kx - (1/aruco_marker_array[2]) * K_saturation);
    if aruco_marker_array[2] > max_sat_z and max_sat_flag_aruco == 0:  # still not saturated enough
        DataArray[1] = int(NV_aruco-aruco_marker_array[0]*Kx_aruco-(1/aruco_marker_array[2])*K_saturation_aruco);
        DataArray[0] = int(NV_aruco+aruco_marker_array[0]*Kx_aruco-(1/aruco_marker_array[2])*K_saturation_aruco);
    '''else:    # Stop
        DataArray[1] = 0
        DataArray[0] = 0
        max_sat_flag = 1
        max_sat_flag_aruco = 1'''

    # velocity delimitation
    if DataArray[0] < 0:
        DataArray[0] = 0;
    elif DataArray[0] > 200:
    	DataArray[0] = 200;
    if DataArray[1] < 0:
        DataArray[1] = 0;
    elif DataArray[1] > 200:
        DataArray[1] = 200;

    DataArray[5] = abs(DataArray[1] - DataArray[0])

def CallBack_Buzzer(buzzer_data):
    global buzzer_value_prev

    if buzzer_data.data == 1 and buzzer_value_prev == 0:
        DataArray[5] = 255
        buzzer_value_prev = 1
    elif buzzer_data.data == 0 and buzzer_value_prev == 1:
        DataArray[5] = 0
        buzzer_value_prev = 0

def CallBack_LED_green(LED_green_data):
    global LED_green_value_prev

    if LED_green_data.data == 1 and LED_green_value_prev == 0:
        DataArray[2] = 255
        LED_green_value_prev = 1
    elif LED_green_data.data == 0 and LED_green_value_prev == 1:
        DataArray[2] = 0
        LED_green_value_prev = 0

def CallBack_LED_red(LED_red_data):
    global LED_red_value_prev

    if LED_red_data.data == 1 and LED_red_value_prev == 0:
        DataArray[3] = 255
        LED_red_value_prev = 1
    elif LED_red_data.data == 0 and LED_red_value_prev == 1:
        DataArray[3] = 0
        LED_red_value_prev = 0

def CallBack_LED_blue(LED_blue_data):
    global LED_blue_value_prev

    if LED_blue_data.data == 1 and LED_blue_value_prev == 0:
        DataArray[4] = 255
        LED_blue_value_prev = 1
    if LED_blue_data.data == 0 and LED_blue_value_prev == 1:
        DataArray[4] = 0
        LED_blue_value_prev = 0

def CallBack_Joystick(joystick_data):
    
    joystick_x_value = joystick_data.angular.z
    joystick_y_value = joystick_data.linear.x
    V = (1-abs(joystick_x_value))*(joystick_y_value)+joystick_y_value
    W = (1-abs(joystick_y_value))*joystick_x_value+joystick_x_value
    R = (V+W)/2
    L = (V-W)/2

    DataArray[0] = int(200*R);
    DataArray[1] = int(200*L);

# ----------------- Publisher and Subscriber definition --------------------
# Name of the node. False = no random numbers
rospy.init_node('mini_luiee_node', anonymous=False)
print("Initializing mini_luiee_node...")

# Published definition. Int32MultiArray because of PWM (0-255 int)
pub = rospy.Publisher('/mini_luiee_topic', Int32MultiArray, queue_size=10)

# Subscriber definition
rospy.Subscriber('/tf', TFMessage, CallBack_arUco)

# Subscriber definition.
rospy.Subscriber('/buzzer_btn', Bool, CallBack_Buzzer)
rospy.Subscriber('/LED_green_btn', Bool, CallBack_LED_green)
rospy.Subscriber('/LED_red_btn', Bool, CallBack_LED_red)
rospy.Subscriber('/LED_blue_btn', Bool, CallBack_LED_blue)
rospy.Subscriber('/cmd_vel', Twist, CallBack_Joystick)

# Data definition
mini_luiee_data = Int32MultiArray()

# Program frequency
rate = rospy.Rate(50)

    # ======================== While loop ===================================
def main():
    while not rospy.is_shutdown():
        mini_luiee_data.data = DataArray
        pub.publish(mini_luiee_data)
        rate.sleep()

# ================= LOOP =========================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
