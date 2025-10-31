#!/usr/bin/env python
import rospy
import json
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def load_initial_position():
    try:

        file_path = '/home/er/myagv_ros/src/assistant/data/10.json'
        with open(file_path, 'r') as file:
            data = json.load(file)
            pos_x = data.get("pos_x", 0.0)
            pos_y = data.get("pos_y", 0.0)
            pos_z = data.get("pos_z", 0.0)
            ori_x = data.get("ori_x", 0.0)
            ori_y = data.get("ori_y", 0.0)
            ori_z = data.get("ori_z", 0.0)
            ori_w = data.get("ori_w", 1.0)


            initial_position = PoseStamped()
            initial_position.header.stamp = rospy.Time.now()
            initial_position.header.frame_id = "map" 
            initial_position.pose.position.x = pos_x
            initial_position.pose.position.y = pos_y
            initial_position.pose.position.z = pos_z
            initial_position.pose.orientation.x = ori_x
            initial_position.pose.orientation.y = ori_y
            initial_position.pose.orientation.z = ori_z
            initial_position.pose.orientation.w = ori_w

            return initial_position
    except Exception as e:
        rospy.logerr(f"Failed to load initial position: {e}")
        return None

def publish_initial_pose():
    initial_position = load_initial_position()
    if initial_position:
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose = initial_position.pose
        pose_msg.pose.covariance = [0] * 36  
        
        rospy.sleep(1)
        pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('publish_initial_pose')
    publish_initial_pose()