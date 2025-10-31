#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <json/json.h>
#include <unordered_map>
#include "auto_dock/goal_sending.h"

namespace beast = boost::beast;
namespace net = boost::asio;
namespace websocket = boost::beast::websocket;
using tcp = boost::asio::ip::tcp;

namespace auto_dock {

    class WebSocketClient {
    public:
        WebSocketClient(const std::string& host, const std::string& port)
            : host_(host), port_(port), resolver_(ioc_), ws_(ioc_), stopRequested(false) {}

  
        void connect(auto_dock::GoalSending& goalSender) {
            try {
           
                auto const results = resolver_.resolve(host_, port_);
                net::connect(ws_.next_layer(), results.begin(), results.end());
                ws_.handshake(host_, "/");
                std::cout << "Connected to WebSocket server at ws://" << host_ << ":" << port_ << std::endl;


                std::thread ws_thread(&WebSocketClient::asyncReadMessages, this);

    
                while (!stopRequested) {
                    std::string received_message = popMessage();
                    if (!received_message.empty()) {
                        handleMessage(received_message, goalSender);
                    }
                }

  
                ws_.close(websocket::close_code::normal);
                ws_thread.join();
            } catch (const std::exception& e) {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }

        
        void sendMessage(const std::string& message) {
            ws_.write(boost::asio::buffer(message));
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

        // ť���� �޽����� �ϳ� ������ �Լ�
        std::string popMessage() {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!messageQueue_.empty()) {
                std::string message = messageQueue_.front();
                messageQueue_.pop();
                return message;
            }
            return "";  // �޽����� ������ �� ���ڿ� ��ȯ
        }

        // ���� �޽����� ���� ó�� �Լ�
        void handleMessage(const std::string& message, auto_dock::GoalSending& goalSender) {
            static const std::unordered_map<std::string, std::string> goal_map = {
                {"C401", "/home/er/myagv_ros/src/auto_dock/data/1.json"},
                {"Convenience Store", "/home/er/myagv_ros/src/auto_dock/data/2.json"},
                {"Restroom", "/home/er/myagv_ros/src/auto_dock/data/3.json"},
                {"Cinema", "/home/er/myagv_ros/src/auto_dock/data/4.json"},
                {"Foodcourt", "/home/er/myagv_ros/src/auto_dock/data/5.json"},
                {"Bookstore", "/home/er/myagv_ros/src/auto_dock/data/6.json"},
                {"Rest_Room", "/home/er/myagv_ros/src/auto_dock/data/7.json"}
            };

            if (message == "STOP") {
                std::cout << "STOP signal received, stopping the robot..." << std::endl;
                goalSender.stopAndReset();
            } else if (goal_map.find(message) != goal_map.end()) {
                goalSender.openfile(goal_map.at(message));
                std::cout << "Goal: " << message << std::endl;
                goalSender.goalPointPub();
            } else if (message == "quit") {
                std::cout << "Received quit message, closing connection..." << std::endl;
                stopRequested = true;
            }
        }
    };

}  // namespace auto_dock

#endif  // WEBSOCKET_CLIENT_H
