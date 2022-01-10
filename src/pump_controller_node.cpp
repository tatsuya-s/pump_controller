#include "pump_controller_node.hpp"

PumpController::PumpController() :
    private_nh("~"),
    pump_action_server(nh, "pump_action", boost::bind(&PumpController::controlPump, this, _1), false),
    step_angle(1.8),
    frequency(300),
    rasp_addr(""),
    rasp_port(""),
    serial_tty("/dev/ttyUSB0")
{
    this->pump_action_server.start();
    this->set_port_srv = this->nh.advertiseService("control_pump", &PumpController::controlPump, this);

    this->private_nh.getParam("step_angle", this->step_angle);
    this->private_nh.getParam("frequency", this->frequency);
    this->private_nh.getParam("rasp_addr", this->rasp_addr);
    this->private_nh.getParam("rasp_port", this->rasp_port);
    this->private_nh.getParam("serial_tty", this->serial_tty);

    char* char_addr = this->rasp_addr.length() > 0 ? const_cast<char*>(this->rasp_addr.c_str()) : NULL;
    char* char_port = this->rasp_port.length() > 0 ? const_cast<char*>(this->rasp_port.c_str()) : NULL;
    ROS_INFO("IP address: %s, Port: %s\n", char_addr, char_port);
    this->pi = pigpio_start(char_addr, char_port);

    if (this->pi < 0)
    {
        ROS_ERROR("GPIO error");
        ros::shutdown();
    }

    char* char_tty = this->serial_tty.length() > 0 ? const_cast<char*>(this->serial_tty.c_str()) : NULL;
    this->handle = serial_open(this->pi, char_tty, 9600, 0);
    if (this->handle < 0)
    {
        ROS_ERROR("Serial error");
        ros::shutdown();
    }

    ROS_INFO("Started pump action server");
}

PumpController::~PumpController()
{
}

bool PumpController::controlPump(pump_controller::ControlPump::Request &req,
                                 pump_controller::ControlPump::Response &res)
{
    try
    {
        const int value = req.pulse * (req.direction ? 1 : -1);
        std::string data = std::to_string(value) + "\n";
        serial_write(this->pi, this->handle, const_cast<char*>(data.c_str()), data.size());

        const double duration = req.pulse / this->frequency;
        ros::Duration(duration).sleep();

        res.success = true;
        res.time = duration;
        res.revolution = this->step_angle / 360 * value;
    }
    catch(...)
    {
        ROS_ERROR("Couldn't control pump");
        ros::Duration(1.0).sleep();
        res.success = false;
        res.revolution = 0;
        res.time = 0;
    }

    return true;
}


void PumpController::controlPump(const pump_controller::ControlPumpGoal::ConstPtr &goal)
{
    pump_controller::ControlPumpFeedback feedback;
    pump_controller::ControlPumpResult result;
    bool is_first = true;
    bool success = false;
    ros::Rate rate(1000);
    ros::Time start_time = ros::Time::now();
    ros::Duration duration;

    const double total_time = goal->pulse / this->frequency;
    const int total_value = goal->pulse * (goal->direction ? 1 : -1);

    while (ros::ok())
    {
        if (this->pump_action_server.isPreemptRequested())
        {
            this->pump_action_server.setPreempted();
            success = false;
            break;
        }

        duration = ros::Time::now() - start_time;

        if (is_first)
        {
            std::string data = std::to_string(total_value) + "\n";
            serial_write(this->pi, this->handle, const_cast<char*>(data.c_str()), data.size());
            is_first = false;
        }

        if (duration.toSec() >= total_time)
        {
            success = true;
            break;
        }

        feedback.time = duration.toSec();
        feedback.revolution = this->step_angle / 360 * (feedback.time * this->frequency) * (goal->direction ? 1 : -1);
        this->pump_action_server.publishFeedback(feedback);

        rate.sleep();
    }

    if (success)
    {
        result.time = duration.toSec();
        result.revolution = this->step_angle / 360 * total_value;

        this->pump_action_server.setSucceeded(result);
    }
}

void PumpController::clear()
{
    serial_close(this->pi, this->handle);
    pigpio_stop(this->pi);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pump_controller_node");

    PumpController my_controller;

    ros::spin();

    my_controller.clear();

    return 0;
}
