#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String

def start_line_trace(data):
    rospy.loginfo("Received command: %s", data.data)
    if data.data == "start_line_trace":
        rospy.loginfo("Starting line trace script...")

        
        script_path = "/home/er/ey_ws/linetrace_start.py"
        rospy.set_param("linetrace_script_path", script_path)

        
        rospy.loginfo("Running line trace script at: %s", rospy.get_param("linetrace_script_path"))
        result = subprocess.run(["python3", rospy.get_param("linetrace_script_path")], capture_output=True)
        rospy.loginfo("Line trace script result: %s", result.stdout.decode())

    elif data.data == "end_line_trace":
        rospy.loginfo("ending line trace script...")

        
        script_path = "/home/er/ey_ws/linetrace_end.py"
        rospy.set_param("linetrace_script_path", script_path)

        
        rospy.loginfo("Running line trace script at: %s", rospy.get_param("linetrace_script_path"))
        result = subprocess.run(["python3", rospy.get_param("linetrace_script_path")], capture_output=True)
        rospy.loginfo("Line trace script result: %s", result.stdout.decode())

def line_trace_node():
    rospy.init_node('line_trace_node', anonymous=True)
    rospy.Subscriber("line_trace_command", String, start_line_trace)
    rospy.spin()

if __name__ == '__main__':
    line_trace_node()