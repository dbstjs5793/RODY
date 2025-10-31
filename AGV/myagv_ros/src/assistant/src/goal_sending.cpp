#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <json/json.h>
#include <tf/transform_listener.h>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <std_msgs/String.h>
namespace beast = boost::beast;
namespace net = boost::asio;
namespace websocket = boost::beast::websocket;
using tcp = boost::asio::ip::tcp;
bool goal_arrived_flag = false;
bool goal_error_flag =false;
bool navigation_flag=false;
bool itemstorage_flag=false;
bool patrol_flag=false;
int item_cnt=0;
int cnt = 0;
class GoalSending {
public:
    GoalSending() : ac_("move_base", true) {}
    void openfile(const std::string& filename) {
        // Close the file if it's already open
        if (in_.is_open()) {
            in_.close();  // Close the file if it's open
        }
        // Open the file for reading in binary mode
        in_.open(filename, std::ios::in | std::ios::binary);
        if (!in_.is_open()) {
            std::cout << "Error opening file: " << filename << std::endl;
            return;
        }
        if (reader_.parse(in_, root_)) {
            goal_point[0] = root_["robot_pose"]["pos_x"].asDouble();
            goal_point[1] = root_["robot_pose"]["pos_y"].asDouble();
            goal_point[2] = root_["robot_pose"]["pos_z"].asDouble();
            goal_point[3] = root_["robot_pose"]["ori_x"].asDouble();
            goal_point[4] = root_["robot_pose"]["ori_y"].asDouble();
            goal_point[5] = root_["robot_pose"]["ori_z"].asDouble();
            goal_point[6] = root_["robot_pose"]["ori_w"].asDouble();
        }
        std::cout << "goal=" << goal_point[0] << " "
                  << goal_point[1] << " "
                  << goal_point[2] << " "
                  << goal_point[3] << " "
                  << goal_point[4] << " "
                  << goal_point[5] << " "
                  << goal_point[6] << std::endl;
    }
    void goalPointPub() {
        std::thread goalThread([this]() {
            while (!ac_.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            std::cout << "go received send goal" << std::endl;
            goal_.target_pose.header.frame_id = "map";
            goal_.target_pose.header.stamp = ros::Time::now();
            goal_.target_pose.pose.position.x = goal_point[0];
            goal_.target_pose.pose.position.y = goal_point[1];
            goal_.target_pose.pose.position.z = goal_point[2];
            goal_.target_pose.pose.orientation.x = goal_point[3];
            goal_.target_pose.pose.orientation.y = goal_point[4];
            goal_.target_pose.pose.orientation.z = goal_point[5];
            goal_.target_pose.pose.orientation.w = goal_point[6];
            ROS_INFO("Sending goal");
            ac_.sendGoal(goal_);
            ac_.waitForResult();
            if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("OK, the base moved to the goal");
                goal_arrived_flag = true;
            }
            else
            {
                ROS_ERROR("The base failed to move for some reason");
                goal_error_flag = true;
            }
        });
        goalThread.detach();
    }
    void stopAndReset() {
        std::cout << "Stopping the robot and canceling the current goal." << std::endl;
        // Cancel goal
        ac_.cancelGoal();
        std::cout << "Cancel goal called." << std::endl;
        // Wait for confirmation of cancellation
        ros::Duration(1.0).sleep();  // Wait for confirmation
        // Check goal status after cancellation
        if (ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
            std::cout << "Goal successfully preempted (canceled)." << std::endl;
        } else if (ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            std::cout << "Goal is still active and has not been canceled." << std::endl;
        } else if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            std::cout << "Goal has already succeeded." << std::endl;
        } else if (ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
            std::cout << "Goal was aborted." << std::endl;
        } else if (ac_.getState() == actionlib::SimpleClientGoalState::REJECTED) {
            std::cout << "Goal was rejected." << std::endl;
        } else if (ac_.getState() == actionlib::SimpleClientGoalState::LOST) {
            std::cout << "Goal lost." << std::endl;
        }
        std::cout << "Goal cancelation check complete." << std::endl;
    }
private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    move_base_msgs::MoveBaseGoal goal_;
    std::ifstream in_;
    Json::Value root_;
    Json::Reader reader_;
    double goal_point[7];
};
class WebSocketClient {
public:
    WebSocketClient(const std::string& host, const std::string& port, ros::NodeHandle& nh)
        : host_(host), port_(port), resolver_(ioc_), ws_(ioc_), stopRequested(false) {
            line_trace_pub = nh.advertise<std_msgs::String>("line_trace_command", 10);
            aruco_pub = nh.advertise<std_msgs::String>("aruco_command", 10);}
    void connect(GoalSending& goalSender) {
        try {
            auto const results = resolver_.resolve(host_, port_);
            net::connect(ws_.next_layer(), results.begin(), results.end());
            ws_.handshake(host_, "/");
            std::cout << "Connected to WebSocket server at ws://" << host_ << ":" << port_ << std::endl;
            // Start the read thread asynchronously
            std::thread ws_thread(&WebSocketClient::asyncReadMessages, this);
            // Main loop for handling messages
            while (!stopRequested) {
                std::string received_message = popMessage();
                if (!received_message.empty()) {
                    if (received_message == "Navigation"){
                        navigation_flag=true;
                    }
                    else if (received_message == "Item_Storage"){
                        itemstorage_flag=true;
                        if (item_cnt==0){
                            item_cnt=1;
                        }
                        else{
                            item_cnt=0;
                        }
                    } 
                    else if (received_message == "Patrol_Start")
                    {
                        patrol_flag=true;
                    }
                    else if (received_message == "Patrol_Stop") {
                        patrol_flag = false;
                        std::cout << "Patrol stopped." << std::endl;
                    }
                    if (navigation_flag){
                        if (received_message == "STOP") {
                            std::cout << "STOP signal received, stopping the robot..." << std::endl;
                            goalSender.stopAndReset();  // Stop the robot
                        } 
                        else if (received_message == "base") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/1.json");
                            std::cout << "base" << std::endl;
                        } 
                        else if (received_message == "Convenience Store") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/2.json");
                            std::cout << "Goal Convenience Store" << std::endl;
                        } 
                        else if (received_message == "Restroom") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/3.json");
                            std::cout << "Goal Restroom" << std::endl;
                        } 
                        else if (received_message == "Cinema") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/4.json");
                            std::cout << "Goal Cinema" << std::endl;
                        } 
                        else if (received_message == "Foodcourt") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/5.json");
                            std::cout << "Goal Foodcourt" << std::endl;
                        } 
                        else if (received_message == "Bookstore") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/6.json");
                            std::cout << "Goal Bookstore" << std::endl;
                        } 
                        else if (received_message == "go") {
                            std::cout << "Go" << std::endl;
                            goalSender.goalPointPub();
                        }

                    }
                    if (itemstorage_flag){
                        if (received_message == "base") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/1.json");
                            std::cout << "base" << std::endl;
                        } 
                        else if (received_message == "Convenience Store") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/2.json");
                            std::cout << "Goal Convenience Store" << std::endl;
                        }
                        else if (received_message == "Restroom") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/3.json");
                            std::cout << "Goal Restroom" << std::endl;
                        }
                        else if (received_message == "Cinema") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/4.json");
                            std::cout << "Goal Cinema" << std::endl;
                        }
                        else if (received_message == "Foodcourt") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/5.json");
                            std::cout << "Goal Foodcourt" << std::endl;
                        }
                        else if (received_message == "Bookstore") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/6.json");
                            std::cout << "Goal Bookstore" << std::endl;
                        }
                        else if (received_message == "save") {
                            goalSender.openfile("/home/er/myagv_ros/src/assistant/data/7.json");
                            std::cout << "save_Room" << std::endl;
                        }
                        else if (received_message == "go") {
                            goalSender.goalPointPub();
                        }
                        else if (received_message == "start_line_trace") {                    
                            std_msgs::String msg;
                            msg.data = "start_line_trace";
                            std::cout << "start" << std::endl;
                            ROS_INFO("Published start_line_trace");
                            line_trace_pub.publish(msg);
                            ROS_INFO("Published start_line_trace message to line_trace_command topic");
                            
                        }
                        else if (received_message == "Back") {                    
                            std_msgs::String msg;
                            msg.data = "end_line_trace";
                            std::cout << "start" << std::endl;
                            ROS_INFO("Published end_line_trace");
                            line_trace_pub.publish(msg);
                            ROS_INFO("Published end_line_trace message to line_trace_command topic");
                            
                        }
                    }

                    if (received_message == "quit") {
                        std::cout << "Received quit message, closing connection..." << std::endl;
                        break;
                    }
                }
                if (patrol_flag) {
                    goalSender.openfile("/home/er/myagv_ros/src/assistant/data/8.json");
                    goalSender.goalPointPub();

                    while (!goal_arrived_flag && patrol_flag) {
                        ros::Duration(0.1).sleep(); // Wait until goal is reached or patrol_flag is turned off
                    }
                    if (!patrol_flag) break; // Exit patrol loop if patrol_flag is false

                    goal_arrived_flag = false; // Reset flag
                    goalSender.openfile("/home/er/myagv_ros/src/assistant/data/9.json");
                    goalSender.goalPointPub();

                    while (!goal_arrived_flag && patrol_flag) {
                        ros::Duration(0.1).sleep();
                    }
                    goal_arrived_flag = false; // Reset flag
            }
                 if (goal_arrived_flag)
                {
                    goal_arrived_flag = false;
                    if (navigation_flag){
                        if (cnt == 0)
                        {
                            message="start";
                            cnt = 1;
                            WriteMessage(message); 
                        }
                        else if (cnt == 1)
                        {
                            message="arrive";
                            cnt = 2;
                            WriteMessage(message); 
                        }
                        else{
                            std_msgs::String msg;
                            msg.data = "run_aruco";
                            ROS_INFO("Published run_aruco");
                            aruco_pub.publish(msg); 
                            ROS_INFO("Published run_aruco message to aruco_command topic");
                            cnt=0; //base
                            navigation_flag=false;
                        }
                    }
                    else if (itemstorage_flag){
                        if (item_cnt==1){

                            if (cnt == 0)
                            {
                                message="store";
                                cnt = 1;
                                WriteMessage(message); 
                            }
                            else if (cnt == 1) {
                                
                                message="line_start";
                                WriteMessage(message);
                                cnt = 2;                                
                            }
                            else {
                                std_msgs::String msg;
                                msg.data = "run_aruco";
                                ROS_INFO("Published run_aruco");
                                aruco_pub.publish(msg); 
                                ROS_INFO("Published run_aruco message to aruco_command topic");
                                cnt=0;//base
                                itemstorage_flag=false;
                            }
                        }else{
                            if (cnt==0){
                                message="line_start";
                                WriteMessage(message);
                                cnt=1;
                            }
                            else if (cnt==1){
                                message="retrieve";
                                WriteMessage(message);
                                cnt=2;      
                            }
                            else{
                                std_msgs::String msg;
                                msg.data = "run_aruco";
                                ROS_INFO("Published run_aruco");
                                aruco_pub.publish(msg); 
                                ROS_INFO("Published run_aruco message to aruco_command topic"); 
                                cnt=0;
                                //base
                                itemstorage_flag=false;
                            }
                        }
                        
                    }    
                }
                else if(goal_error_flag)
                {
                    message="error";
                    WriteMessage(message);
                    goal_error_flag = false;
                }
            

                
                }
            ws_.close(websocket::close_code::normal);
            ws_thread.join();
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }


private:
    std::string host_;
    std::string port_;
    net::io_context ioc_;
    tcp::resolver resolver_;
    websocket::stream<tcp::socket> ws_;
    std::mutex mtx_;
    std::queue<std::string> messageQueue_;
    std::atomic<bool> stopRequested;
    std::string message = "";
    std::thread ws_thread;
    ros::Publisher line_trace_pub;
    ros::Publisher aruco_pub;
    void WriteMessage(const std::string& message) {
    try {
        beast::flat_buffer buffer;
        auto const size = message.size();
        auto buffer_ptr = buffer.prepare(size);
        boost::asio::buffer_copy(buffer_ptr, boost::asio::buffer(message));
        buffer.commit(size);
        ws_.write(buffer.data());
        std::cout << "Sent message: " << message << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error while writing message: " << e.what() << std::endl;
    }
    }
    void asyncReadMessages() {
        try {
            while (true) {
                beast::flat_buffer buffer;
                ws_.read(buffer);
                std::string message = boost::beast::buffers_to_string(buffer.data());
                std::lock_guard<std::mutex> lock(mtx_);
                messageQueue_.push(message);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error while reading messages: " << e.what() << std::endl;
        }
    }
    std::string popMessage() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!messageQueue_.empty()) {
            std::string message = messageQueue_.front();
            messageQueue_.pop();
            return message;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return "";  // Return empty if no message is available
    }
    
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "a_goals_sender");
    ros::NodeHandle n;
    GoalSending goal;
    std::string host = "172.30.1.31";
    std::string port = "81";
    WebSocketClient client(host, port,n);
    client.connect(goal);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}