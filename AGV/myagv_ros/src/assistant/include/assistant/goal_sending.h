// goal_sending.h

#ifndef GOAL_SENDING_H
#define GOAL_SENDING_H

#include <string>
#include <ros/ros.h>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "auto_dock/websocket_client.h"

namespace auto_dock {

    class GoalSending {
    public:
        GoalSending(const std::string& host, const std::string& port, WebSocketClient* client);
        
        void openfile(const std::string& filename);
        void goalPointPub();
        void stopAndReset();

    private:
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
        move_base_msgs::MoveBaseGoal goal_;
        WebSocketClient* client_;
        net::io_context ioc_;
        tcp::resolver resolver_;
        websocket::stream<tcp::socket> ws_;
        std::ifstream in_;
        Json::Value root_;
        Json::Reader reader_;
        double goal_point[7];
    };

}  // namespace auto_dock

#endif // GOAL_SENDING_H
