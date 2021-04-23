#include <SimpleAmqpClient/SimpleAmqpClient.h>
#include <iostream>

int main() {
    std::string queue_name = "com.gdcccn.rer.device";

    AmqpClient::Channel::ptr_t channel = AmqpClient::Channel::Create("192.168.20.2", 5672, "rer", "gdcc@2021", "rer");
    //创建channel

    channel->DeclareQueue(queue_name, false, true, false, false);

    std::string message = "Test RabbitMQ!!!";

    channel->BasicPublish("", queue_name, AmqpClient::BasicMessage::Create(message));
    //第一个是exchange名称，第二个参数是routing_key（此处可理解为消息会被送往的队列）。

    std::cout << "[x] send " << message << std::endl;
}

