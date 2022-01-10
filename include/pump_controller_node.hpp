#ifndef PUMP_CONTROLLER_NODE_HPP
#define PUMP_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "pigpiod_if2.h"
#include "pump_controller/ControlPump.h"
#include "pump_controller/ControlPumpAction.h"

class PumpController
{
    public:
        PumpController();
        ~PumpController();
        bool controlPump(pump_controller::ControlPump::Request &req,
                         pump_controller::ControlPump::Response &res);
        void controlPump(const pump_controller::ControlPumpGoal::ConstPtr &goal);
        void clear();

    private:
        ros::NodeHandle nh, private_nh;
        actionlib::SimpleActionServer<pump_controller::ControlPumpAction> pump_action_server;
        ros::ServiceServer set_port_srv;
        int pi;
        double step_angle;
        std::string rasp_addr;
        std::string rasp_port;
        std::string serial_tty;
        int handle;
};

#endif // PUMP_CONTROLLER_NODE_HPP