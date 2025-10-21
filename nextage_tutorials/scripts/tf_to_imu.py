#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import Imu

if __name__ == '__main__':
    rospy.init_node('tf_to_imu')

    base_frame = rospy.get_param("~base_frame", "CHEST_JOINT0_Link")
    sensor_frame = rospy.get_param("~sensor_frame", "RARM_JOINT5_Link")
    imu_topic = rospy.get_param("~imu_topic", "/imu/data")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher(imu_topic, Imu, queue_size=10)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(base_frame, sensor_frame, rospy.Time(0))
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = sensor_frame
            imu_msg.orientation = trans.transform.rotation
            pub.publish(imu_msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"TF not available yet: {e}")
        rate.sleep()
