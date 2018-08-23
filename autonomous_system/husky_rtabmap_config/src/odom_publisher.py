import geometry_msgs.msg
import nav_msgs.msg
import tf2_ros
import rospy

broadcaster = tf2_ros.TransformBroadcaster()
tstamped = geometry_msgs.msg.TransformStamped()


def publish_tf_message(data):
    """
    :type data: nav_msgs.msg.Odometry
    """

    tstamped.header.stamp = rospy.Time.now()
    tstamped.header.frame_id = "map"
    tstamped.child_frame_id = "odom"

    tstamped.transform.translation.x = data.pose.pose.position.x
    tstamped.transform.translation.y = data.pose.pose.position.y
    tstamped.transform.translation.z = data.pose.pose.position.z

    tstamped.transform.rotation.x = data.pose.pose.orientation.x
    tstamped.transform.rotation.y = data.pose.pose.orientation.y
    tstamped.transform.rotation.z = data.pose.pose.orientation.z
    tstamped.transform.rotation.w = data.pose.pose.orientation.w

    broadcaster.sendTransform(tstamped)


rospy.init_node("odom_publisher_rtabmap")
sub = rospy.Subscriber("/rtabmap/odom", nav_msgs.msg.Odometry, callback=publish_tf_message, queue_size=10)
rospy.spin()
