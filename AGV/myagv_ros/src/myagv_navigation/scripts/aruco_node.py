#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String

def handle_aruco_command(data):
    """
    Callback function to handle commands for running the aruco script.
    """
    rospy.loginfo("Received command: %s", data.data)
    
    if data.data == "run_aruco":
        rospy.loginfo("Executing aruco.py script...")

        # Define the script path
        script_path = "/home/er/ey_ws/aruco.py"
        rospy.set_param("aruco_script_path", script_path)

        # Execute the script
        result = subprocess.run(["python3", rospy.get_param("aruco_script_path")], capture_output=True, text=True)
        
        # Log the result of the script execution
        if result.returncode == 0:
            rospy.loginfo("aruco.py script executed successfully.")
            rospy.loginfo("Output: %s", result.stdout)
        else:
            rospy.logerr("Failed to execute aruco.py script.")
            rospy.logerr("Error: %s", result.stderr)

    else:
        rospy.logwarn("Unknown command received: %s", data.data)

def aruco_node():
    """
    Initializes the aruco_node and subscribes to the topic.
    """
    rospy.init_node('aruco_node', anonymous=True)
    rospy.Subscriber("aruco_command", String, handle_aruco_command)
    rospy.spin()

if __name__ == '__main__':
    try:
        aruco_node()
    except rospy.ROSInterruptException:
        pass