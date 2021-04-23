#include <string>
#include <iostream>
#include <SimpleAmqpClient/SimpleAmqpClient.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

int main() {
    std::string queue_name = "com.gdcccn.rer.device";

    AmqpClient::Channel::ptr_t channel = AmqpClient::Channel::Create("192.168.20.2", 5672, "rer", "gdcc@2021", "rer");
    channel->DeclareQueue(queue_name, false, true, false, false);
    std::string consumer_tag = channel->BasicConsume(queue_name, "");
    //第二个参数为消费者名称，返回值也是消费者名称。

    int receiveCount = 0;
    while (receiveCount < 2) {
        AmqpClient::Envelope::ptr_t envelope = channel->BasicConsumeMessage(consumer_tag);
        std::string buffer = envelope->Message()->Body();
        std::cout << "Receive Message from RabbitMQ: " << buffer << std::endl;
        // Parse String to JSON ptree
        std::stringstream stream(buffer);
        boost::property_tree::ptree strTree;
        try {
            boost::property_tree::json_parser::read_json(stream, strTree);
            std::string command = strTree.get<std::string>("command");
            std::string msgType = strTree.get<std::string>("msgType");
            std::string msgId = strTree.get<std::string>("msgId");
            std::string replyQueue = strTree.get<std::string>("replyQueue");
            boost::property_tree::ptree msgData = strTree.get_child("msgData");

            std::cout << "Parse Result: " << std::endl;
            std::cout << "    command: " << command << std::endl;
            std::cout << "    msgType: " << msgType << std::endl;
            std::cout << "    msgId: " << msgId << std::endl;
            std::cout << "    replyQueue: " << replyQueue << std::endl;
            boost::property_tree::ptree reply_json;
            boost::property_tree::ptree reply_msg;
            reply_json.put("command", command);
            reply_json.put("msgType", "rpc_response");
            reply_json.put("msgId", msgId);

            reply_msg.put("status", 1);
            reply_json.put_child("msgData", reply_msg);
            std::stringstream s;
            boost::property_tree::json_parser::write_json(s, reply_json);
            std::string outstr = s.str();
            std::cout << "Reply Json: " << outstr << std::endl;
            channel->BasicPublish("", replyQueue, AmqpClient::BasicMessage::Create(outstr));
            if (command == "startTraining") {
                std::string teachingVideoId = msgData.get<std::string>("teachingVideoId");
                std::cout << "Going to Start Training, teachingVideoId is " << teachingVideoId << std::endl;
            }
        }
        catch (boost::property_tree::ptree_error &e) {
            std::cout << "Error Occur when parsing JSON" << std::endl;
            continue;
        }
        receiveCount++;
    }

    //关闭消费者。
    channel->BasicCancel(consumer_tag);
}
