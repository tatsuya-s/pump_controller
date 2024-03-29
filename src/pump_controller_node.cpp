#include "pump_controller_node.hpp"

PumpController::PumpController() :
    private_nh("~"), 
    pump_action_server(nh, "pump_action", boost::bind(&PumpController::controlPump, this, _1), false), 
    ena_pin(4), 
    dir_pin(17), 
    pwm_pin(18), 
    pwm_duty(500000), // 50%
    step_angle(1.8), 
    use_const_vel(true), 
    rasp_addr(""), 
    rasp_port("")
{
    this->pump_action_server.start();
    this->set_port_srv = this->nh.advertiseService("control_pump", &PumpController::controlPump, this);

    this->private_nh.getParam("ena_pin", this->ena_pin);
    this->private_nh.getParam("dir_pin", this->dir_pin);
    this->private_nh.getParam("pwm_pin", this->pwm_pin);
    this->private_nh.getParam("pwm_duty", this->pwm_duty);
    this->private_nh.getParam("step_angle", this->step_angle);
    this->private_nh.getParam("use_const_vel", this->use_const_vel);
    this->private_nh.getParam("rasp_addr", this->rasp_addr);
    this->private_nh.getParam("rasp_port", this->rasp_port);

    char* char_addr = this->rasp_addr.length() > 0 ? const_cast<char*>(this->rasp_addr.c_str()) : NULL;
    char* char_port = this->rasp_port.length() > 0 ? const_cast<char*>(this->rasp_port.c_str()) : NULL;
    ROS_INFO("IP address: %s, Port: %s\n", char_addr, char_port);
    this->pi = pigpio_start(char_addr, char_port);

    if (this->pi < 0)
    {
        ROS_ERROR("GPIO error");
        ros::shutdown();
    }
    else 
    {
        ROS_INFO("Started pump action server");
    }

    set_mode(this->pi, this->ena_pin, PI_OUTPUT);
    set_mode(this->pi, this->dir_pin, PI_OUTPUT);
    set_mode(this->pi, this->pwm_pin, PI_OUTPUT);
    gpio_write(this->pi, this->ena_pin, PI_OFF);
    hardware_PWM(this->pi, this->pwm_pin, 0, 0);
}

PumpController::~PumpController() 
{
}

bool PumpController::controlPump(pump_controller::ControlPump::Request &req, 
                                 pump_controller::ControlPump::Response &res)
{
    try 
    {
        gpio_write(this->pi, this->ena_pin, PI_ON);
        gpio_write(this->pi, this->dir_pin, req.direction);

        ros::Time start_time = ros::Time::now();
        hardware_PWM(this->pi, this->pwm_pin, req.frequency, this->pwm_duty);

        ros::Duration(req.time).sleep();

        gpio_write(this->pi, this->ena_pin, PI_OFF);
        hardware_PWM(this->pi, this->pwm_pin, 0, 0);
        ros::Duration duration = ros::Time::now() - start_time;

        res.success = true;
        res.time = duration.toSec();
        res.revolution = static_cast<int>(this->step_angle / 360 * req.frequency * 60 * duration.toSec());
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
    const double wake_up_time = 0.5;
    const double fall_asleep_time = goal->time - wake_up_time;
    double frequency = 0;

    while (ros::ok()) 
    {
        if (this->pump_action_server.isPreemptRequested())
        {
            this->pump_action_server.setPreempted();
            success = false;
            break;
        }

        duration = ros::Time::now() - start_time;

        if (use_const_vel) 
        {
            if (is_first) 
            {
                gpio_write(this->pi, this->ena_pin, PI_ON);
                gpio_write(this->pi, this->dir_pin, goal->direction);
                hardware_PWM(this->pi, this->pwm_pin, goal->frequency, this->pwm_duty);
                is_first = false;
            }
        }
        else 
        {
            if (wake_up_time > duration.toSec()) 
            {
                frequency = goal->frequency * (0.5 - 0.5 * std::cos(duration.toSec() / wake_up_time * M_PI));
                gpio_write(this->pi, this->ena_pin, PI_ON);
                gpio_write(this->pi, this->dir_pin, goal->direction);
                hardware_PWM(this->pi, this->pwm_pin, frequency, this->pwm_duty);
            }
            else if (duration.toSec() > fall_asleep_time) 
            {
                frequency = goal->frequency * (0.5 + 0.5 * std::cos((duration.toSec() - fall_asleep_time) / wake_up_time * M_PI));
                gpio_write(this->pi, this->ena_pin, PI_ON);
                gpio_write(this->pi, this->dir_pin, goal->direction);
                hardware_PWM(this->pi, this->pwm_pin, frequency, this->pwm_duty);
            }
        }

        if (duration.toSec() >= goal->time) 
        {
            success = true;
            break;
        }

        feedback.time = duration.toSec();
        feedback.revolution = static_cast<int>(this->step_angle / 360 * goal->frequency * 60 * duration.toSec());
        this->pump_action_server.publishFeedback(feedback);

        rate.sleep();
    }

    gpio_write(this->pi, this->ena_pin, PI_OFF);
    hardware_PWM(this->pi, this->pwm_pin, 0, 0);

    if (success)
    {
        result.time = duration.toSec();
        result.revolution = static_cast<int>(this->step_angle / 360 * goal->frequency * 60 * duration.toSec());

        this->pump_action_server.setSucceeded(result);
    }
}

void PumpController::clear()
{
    set_mode(this->pi, this->ena_pin, PI_INPUT);
    set_mode(this->pi, this->dir_pin, PI_INPUT);
    set_mode(this->pi, this->pwm_pin, PI_INPUT);
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
