#include <ns-3_adapter.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(NS3AdapterTest, testOnConnectHandler)
{
    /*int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    //char* argv[] {c[0], c[1]};
    char **argv;
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << worker.getDriverStatus().status);
    worker.onConnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OPERATIONAL);*/
}

TEST(NS3AdapterTest, testOnDisconnectHandler)
{
   /* int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << worker.getDriverStatus().status);
    worker.onDisconnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);*/

}

TEST(NS3AdapterTest, testOnMsgReceivedHandler)
{
/*    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << worker.getDriverStatus().status);
    worker.onMessageReceivedHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);
*/

}

TEST(NS3AdapterTest, testpackMessage)
{

}

TEST(NS3AdapterTest, testonOutboundMessage)
{
    /*int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray array1;

    uint8_t msg = 8;

    array1.content.push_back(msg);

    cav_msgs::ByteArray::ConstPtr message = new cav_msgs::ByteArray::ConstPtr(array1);


    worker.onOutboundMessage(message);
    EXPECT_EQ(worker.getMsgQueue().back(), message->content.back());*/

}

TEST(NS3AdapterTest, testSendMessageSrv)
{
    
}

TEST(NS3AdapterTest, testSendMessageFromQueue)
{
    
}

TEST(NS3AdapterTest, testloadWaveConfig)
{
    
}