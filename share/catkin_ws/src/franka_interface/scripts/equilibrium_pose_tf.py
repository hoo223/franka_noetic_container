#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


class EquilibriumPoseTfBridge:
    def __init__(self):
        self.topic_name = rospy.get_param(
            "~topic", "/cartesian_impedance_example_controller/equilibrium_pose"
        )
        self.child_frame = rospy.get_param("~child_frame", "equilibrium_pose_target")
        self.default_parent = rospy.get_param("~default_parent", "panda_link0")

        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.topic_name, PoseStamped, self.pose_callback, queue_size=10)

        rospy.loginfo(
            "equilibrium_pose_tf_bridge started (topic: %s, child_frame: %s)",
            self.topic_name,
            self.child_frame,
        )

    def pose_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.default_parent
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        self.br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_tf_bridge")
    EquilibriumPoseTfBridge()
    rospy.spin()
