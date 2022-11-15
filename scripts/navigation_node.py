#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

Kx_color = 0.85
K_saturation_color = 0.0045
NV_color = 140			# with 9V
color_marker_array = [0.0,0.0,0.0]
max_sat_color = 16500 	# Established, but it can vary ~ 20,000 or 37,000 of the whole color screen
max_sat_flag_color = 0	# To stop the rover

navigationDataArray = [0,0,2,0,0,0,0,0,0,0,0,0]

# =================== CALLBACK de ROS ==============================
def CallBack_color(color_marker_data):
    global max_sat_flag_color

    # READ from the color_marker_topic
    color_marker_array[0] = color_marker_data.data[0];
    color_marker_array[1] = color_marker_data.data[1];
    color_marker_array[2] = color_marker_data.data[2];

    if color_marker_array[2] < max_sat_color and max_sat_flag_color == 0:  # still not saturated enough
        navigationDataArray[0] = int(NV_color-(color_marker_array[0]-160)*Kx_color);
        navigationDataArray[1] = int(NV_color+(color_marker_array[0]-160)*Kx_color);
    else:    # Stop
        navigationDataArray[0] = 0
        navigationDataArray[1] = 0
        max_sat_flag_color = 1
    
    #if there is nothing to track
    if color_marker_array[0] == 0 and color_marker_array[1] == 0 and color_marker_array[2] == 0:
        navigationDataArray[0] = 0;
        navigationDataArray[1] = 0;

    # velocity delimitation (in the navigation mode, the rover will not use negative motor values)
    if navigationDataArray[0] < 0:
        navigationDataArray[0] = 0;
    elif navigationDataArray[0] > 255:
        navigationDataArray[0] = 255;
    if navigationDataArray[1] < 0:
        navigationDataArray[1] = 0;
    elif navigationDataArray[1] > 255:
        navigationDataArray[1] = 255;

    navigationDataArray[3] = abs(navigationDataArray[0] - navigationDataArray[1])   # Max 255 for Buzzer

# ----------------- Publisher and Subscriber definition --------------------
# Name of the node. False = no random numbers
rospy.init_node('navigation_node', anonymous=False)
print("Initializing mini_luiee_node...")

# Published definition. Int32MultiArray because of PWM (0-255 int)
pub = rospy.Publisher('/luiee_topic', Int32MultiArray, queue_size=10)

# Subscriber definition. NEVER run the aruco and the color algorithms at the same time!
rospy.Subscriber('/color_marker_topic', Float32MultiArray, CallBack_color)

# Data definition
navigation_data = Int32MultiArray()

# Program frequency
rate = rospy.Rate(10) # 50hz

# ======================== While loop ===================================
def main():
    while not rospy.is_shutdown():
        navigation_data.data = navigationDataArray
        pub.publish(navigation_data)
        rate.sleep()

# ================= LOOP =========================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
