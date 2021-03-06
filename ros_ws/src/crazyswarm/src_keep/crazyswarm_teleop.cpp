#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Land.h>
#include "crazyflie_driver/getPosSetPoint.h"

namespace Xbox360Buttons {

    enum {
        Green  = 0,
        Red    = 1,
        Blue   = 2,
        Yellow = 3,
        LB     = 4,
        RB     = 5,
        Back   = 6,
        Start  = 7,
        COUNT  = 8,
    };

}

class Manager
{
public:

    Manager()
        : m_subscribeJoy()
        , m_serviceEmergency()
        , m_serviceTakeoff()
        , m_serviceLand()
    {
        ros::NodeHandle nh;
        m_subscribeJoy = nh.subscribe("/joy", 1, &Manager::joyChanged, this);

        ROS_INFO("Wait for services...");

        ros::service::waitForService("/emergency");
        m_serviceEmergency = nh.serviceClient<std_srvs::Empty>("/emergency");
        ros::service::waitForService("/takeoff");
        m_serviceTakeoff = nh.serviceClient<crazyflie_driver::Takeoff>("/takeoff");
        ros::service::waitForService("/land");
        m_serviceLand = nh.serviceClient<crazyflie_driver::Land>("/land");
        ros::service::waitForService("/getServerPosSetPoint");
        m_servicePosSp = nh.serviceClient<crazyflie_driver::getPosSetPoint>("/getServerPosSetPoint");

        ROS_INFO("Manager ready.");
    }

    ~Manager()
    {
    }

private:
    void joyChanged(
        const sensor_msgs::Joy::ConstPtr& msg)
    {
        fflush(stdout);
        static std::vector<int> lastButtonState(Xbox360Buttons::COUNT);

        if (msg->buttons.size() >= Xbox360Buttons::COUNT
            && lastButtonState.size() >= Xbox360Buttons::COUNT)
        {
            if (msg->buttons[Xbox360Buttons::Red] == 1 && lastButtonState[Xbox360Buttons::Red] == 0) {
                emergency();
            }
            if (msg->buttons[Xbox360Buttons::Start] == 1 && lastButtonState[Xbox360Buttons::Start] == 0) {
                takeoff();
            }
            if (msg->buttons[Xbox360Buttons::Back] == 1 && lastButtonState[Xbox360Buttons::Back] == 0) {
                land();
            }
            if (msg->buttons[Xbox360Buttons::LB] == 1 && lastButtonState[Xbox360Buttons::LB] == 0) {
                getPosSp();
            }
        }

        lastButtonState = msg->buttons;
    }

    void emergency()
    {
        ROS_INFO("emergency requested...");
        std_srvs::Empty srv;
        m_serviceEmergency.call(srv);
        ROS_INFO("Done.");
    }

    void takeoff()
    {
        crazyflie_driver::Takeoff srv;
        srv.request.groupMask = 0;
        srv.request.height = 0.5;
        srv.request.duration = ros::Duration(2.0);
        m_serviceTakeoff.call(srv);
    }

    void land()
    {
        crazyflie_driver::Land srv;
        srv.request.groupMask = 0;
        srv.request.height = 0.05;
        srv.request.duration = ros::Duration(3.5);
        m_serviceLand.call(srv);
    }

    void getPosSp()
    {
        ROS_INFO("Service get position setpoint is being called!");
        crazyflie_driver::getPosSetPoint srv;
        srv.request.group = 0;
        srv.request.id = 2;
        m_servicePosSp.call(srv);
    }

private:
    ros::Subscriber m_subscribeJoy;

    ros::ServiceClient m_serviceEmergency;
    ros::ServiceClient m_serviceTakeoff;
    ros::ServiceClient m_serviceLand;
    ros::ServiceClient m_servicePosSp;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  Manager manager;
  ros::spin();

  return 0;
}
