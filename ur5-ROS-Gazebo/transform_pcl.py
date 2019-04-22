#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2


class TransformPointCloud:
    def __init__(self):
        self.config = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("/depth/points", PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber("/depth/point", PointCloud2,
                                    self.point_cloud_callback, queue_size=2)


    def point_cloud_callback(self, msg):
        lookup_time = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", lookup_time.secs, lookup_time.nsecs)
        source_frame = "camera_link"
        target_frame = "points_frame" 
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
                                                    rospy.Duration(10))
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            return
        cloud_out = PointCloud2(msg, trans)
        self.pub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    rospy.spin()