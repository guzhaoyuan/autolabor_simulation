#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

t265_pub = rospy.Publisher('t265/odom', Odometry, queue_size=10)
uwb_pub = rospy.Publisher('dwm/odom', Odometry, queue_size=10)

first_vo = True
first_uwb = 0

first_vo_data = Odometry()
first_uwb_data = Odometry()

def vo_callback(data):
    global first_vo
    global t265_pub
    global first_vo_data
    if first_vo == True:
        first_vo_data = data
        first_vo = False
        # break
    
    new_data = data
    new_data.header.frame_id = "odom"
    new_data.child_frame_id = "base_link"
    new_data.pose.pose.position.x = data.pose.pose.position.x - first_vo_data.pose.pose.position.x
    new_data.pose.pose.position.y = data.pose.pose.position.y - first_vo_data.pose.pose.position.y
    new_data.pose.pose.position.z = 0.0
    t265_pub.publish(new_data)

def uwb_callback(data):
    global first_uwb 
    global uwb_pub
    global first_uwb_data

    if first_uwb < 5:
        first_uwb_data = data
        first_uwb = first_uwb + 1
        return
    
    new_data = data
    new_data.header.frame_id = "map"
    new_data.child_frame_id = "base_link"
    # new_data.pose.pose.position.x = data.pose.pose.position.x - first_uwb_data.pose.pose.position.x
    # new_data.pose.pose.position.y = data.pose.pose.position.y - first_uwb_data.pose.pose.position.y
    new_data.pose.pose.position.z = 0.0
    uwb_pub.publish(new_data)
    
def main():
    rospy.init_node('odom_filter', anonymous=True)
    rospy.Subscriber("t265/odom/sample", Odometry, vo_callback)
    rospy.Subscriber("dwm_odom", Odometry, uwb_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass