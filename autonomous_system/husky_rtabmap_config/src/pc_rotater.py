
#!/usr/bin/python
import tf2_ros
import rospy
import tf2_sensor_msgs
import sensor_msgs.msg, sensor_msgs
import geometry_msgs.msg

quat_static = geometry_msgs.msg.TransformStamped()
quat_static.transform.rotation.z = quat_static.transform.rotation.x = 0.70711
quat_static.child_frame_id = "zed_actual_frame"
quat_static.header.frame_id = "zed_actual_frame"


def pc_fixer(data):
    """
    :type data: sensor_msgs.msg.PointCloud2
    """
    data.header.frame_id = "zed_actual_frame"
    better_pointcloud.publish(data)

if __name__ == "__main__":
    rospy.init_node("pointcloud_fixer")

    tfbuffer = tf2_ros.Buffer()
    tlisten = tf2_ros.TransformListener(tfbuffer)

    better_pointcloud = rospy.Publisher('/camera/points_fixed', sensor_msgs.msg.PointCloud2, queue_size=1)
    bad_pointcloud = rospy.Subscriber('/camera/point_cloud/cloud_registered', sensor_msgs.msg.PointCloud2, callback=pc_fixer, queue_size=1)
rospy.spin()
