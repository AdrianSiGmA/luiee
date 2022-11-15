import rospy
import tf2_ros
import tf
import threading
import geometry_msgs.msg
import tf_conversions
rospy.init_node('Test_Lookup_TF')


class Mimic():
    def __init__(self):
        self.tf_broadcast = tf2_ros.TransformBroadcaster()
        self.rate=rospy.Rate(100)
        self.x = 1.0
        self.y = 0.0
    def set_xy (self,x,y):
        (self.x, self.y) = (x,y)
    def run(self):
        while not rospy.is_shutdown():
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp=rospy.Time.now()
            t.header.frame_id = 'C922_1'
            t.child_frame_id = 'ID1'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcast.sendTransform(t)
            self.rate.sleep()


class ArucoMarker():
    def __init__(self, id_name, point_of_reference):
        self.name=id_name
        self.por = point_of_reference

        self.tf_listener = tf.TransformListener()

        self.x = None
        self.y = None
        self.angle = None
        self.rate=rospy.Rate(1)
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.daemon=True
        self.run_thread.start()
        
    
    def run (self):
        while not rospy.is_shutdown():
            tf_tuple = None
            try:
                self.tf_listener.waitForTransform(self.por, self.name, rospy.Time(0), rospy.Duration(3))
                tf_tuple = self.tf_listener.lookupTransform(self.por, self.name, rospy.Time(0))
                location_vec = tf_tuple[0]
                self.x = location_vec[0]
                self.y = location_vec[1]
                
                
            ## Do MATH
            except:
                pass 

           

            self.rate.sleep()

        
        
ar = ArucoMarker('ID1', 'C922_1')
mimic=Mimic()
mimic.run()
rospy.spin()