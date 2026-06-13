#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

def callback(msg):
    data = msg.data

    if len(data) % 3 != 0:
        rospy.logwarn("Invalid point cloud array size. len(data) must be multiple of 3.")
        return

    points = []

    for i in range(0, len(data), 3):
        unity_x = data[i + 0]
        unity_y = data[i + 1]
        unity_z = data[i + 2]

        ros_x = unity_z
        ros_y = -unity_x
        ros_z =  unity_y

        points.append([ros_x, ros_y, ros_z])
        # points.append([unity_x,unity_y,unity_z])

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]

    cloud_msg = point_cloud2.create_cloud(header, fields, points)
    pub.publish(cloud_msg)

    rospy.loginfo(f"Published PointCloud2: {len(points)} points")

def main():
    global pub

    rospy.init_node("unity_array_to_pointcloud2")

    pub = rospy.Publisher("/unity_pointcloud", PointCloud2, queue_size=1)
    rospy.Subscriber("/Depth_savedata", Float32MultiArray, callback, queue_size=1)

    rospy.loginfo("Waiting for /unity_pointcloud_array ...")
    rospy.spin()

if __name__ == "__main__":
    main()