#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# because of transformations
import tf

from tf2_msgs.msg import TFMessage
import geometry_msgs.msg

def makePointsMsg(data, height, width):

    # see https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743

    pointcloud_msg = PointCloud2()

    pointcloud_msg.header = Header()
    pointcloud_msg.header.frame_id = "map"
    pointcloud_msg.header.stamp = rospy.Time.now()

    # Define the point fields (attributes)        
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    pointcloud_msg.fields = fields

    pointcloud_msg.height = height
    pointcloud_msg.width = width

    # Float occupies 4 bytes. Each point then carries 16 bytes.
    pointcloud_msg.point_step = len(fields) * 4 

    total_num_of_points = pointcloud_msg.height * pointcloud_msg.width
    pointcloud_msg.row_step = pointcloud_msg.point_step * width # TODO: changed this to width because it seemed right, need to check
    pointcloud_msg.is_dense = True # if is set to True, assumes that all points are valid.

    pointcloud_msg.data = data
    pointcloud_msg.is_bigendian = True

    transf1 = geometry_msgs.msg.TransformStamped()
    transf1.header.stamp = rospy.Time.now()
    transf1.header.frame_id = "map"
    transf1.child_frame_id = "base"

    transf1.transform.translation.x = 0
    transf1.transform.translation.y = 0
    transf1.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    transf1.transform.rotation.x = quat[0]
    transf1.transform.rotation.y = quat[1]
    transf1.transform.rotation.z = quat[2]
    transf1.transform.rotation.w = quat[3]


    transf2 = geometry_msgs.msg.TransformStamped()
    transf2.header.stamp = rospy.Time.now()
    transf2.header.frame_id = "base"
    transf2.child_frame_id = "robot"

    transf2.transform.translation.x = 0
    transf2.transform.translation.y = 0
    transf2.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    transf2.transform.rotation.x = quat[0]
    transf2.transform.rotation.y = quat[1]
    transf2.transform.rotation.z = quat[2]
    transf2.transform.rotation.w = quat[3]


    # Create and fill pose message for publishing
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base"
    pose.pose.pose.position.x = 0
    pose.pose.pose.position.y = 0
    pose.pose.pose.position.z = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose.pose.pose.orientation.x = quat[0]
    pose.pose.pose.orientation.y = quat[1]
    pose.pose.pose.orientation.z = quat[2]
    pose.pose.pose.orientation.w = quat[3]

    # Since tf transforms do not have a covariance, pose is filled with
    # a zero covariance.
    pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0]


    return pointcloud_msg, [transf1, transf2], pose

def talker():
    pointPub = rospy.Publisher('points', PointCloud2, queue_size=10)
    tfPub = rospy.Publisher('tf', TFMessage, queue_size=10)
    posePub = rospy.Publisher("pose", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # dummy points
    spacing = 10
    size = 100
    points = []
    for i in range(size):
        for j in range(size):
            # points += [spacing*j, spacing*i, spacing*j]
            points += [spacing*i, spacing*j, 10]

    print("publishing points: ", points)

    while not rospy.is_shutdown():
        msg, transforms, pose = makePointsMsg(points, size, size)
        # rospy.loginfo(msg)
        pointPub.publish(msg)
        tfPub.publish(transforms)
        posePub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

