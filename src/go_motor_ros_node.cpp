#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <go_motor_ros/MotorParamsConfig.h>

#include <unistd.h>
#include <math.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

class GoMotorRosNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // Dynamic reconfigure server
    dynamic_reconfigure::Server<go_motor_ros::MotorParamsConfig> dr_server_;
    
    // Publishers and subscribers
    ros::Subscriber position_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber torque_sub_;
    ros::Publisher joint_state_pub_;
    
    // Timer for control loop
    ros::Timer control_timer_;
    
    // Motor communication
    SerialPort* serial_;
    MotorCmd cmd_;
    MotorData data_;
    
    // Parameters
    std::string motor_type_;
    std::string serial_port_;
    double control_frequency_;
    double command_timeout_;
    
    // Motor control parameters (from dynamic reconfigure)
    double kp_;
    double kd_;
    int motor_id_;
    
    // Motor command parameters (from topics)
    double target_position_;
    double target_velocity_;
    double target_torque_;
    
    // Safety and timeout management
    ros::Time last_position_cmd_time_;
    ros::Time last_velocity_cmd_time_;
    ros::Time last_torque_cmd_time_;
    bool motor_stopped_by_timeout_;
    bool any_command_received_;
    
    // Motor type enum
    MotorType motor_type_enum_;
    
public:
    GoMotorRosNode() : pnh_("~")
    {
        // Initialize parameters
        pnh_.param("motor_type", motor_type_, std::string("GO_M8010_6"));
        // You can use /dev/serial/by-id/xxxx
        // Example /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9HN1N7-if00-port0
        pnh_.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        pnh_.param("control_frequency", control_frequency_, 100.0);
        pnh_.param("command_timeout", command_timeout_, 1.0);  // Default 1 second timeout
        pnh_.param("kp", kp_, 10.0);
        pnh_.param("kd", kd_, 1.0);
        pnh_.param("id", motor_id_, 0);
        
        // Initialize target values
        target_position_ = 0.0;
        target_velocity_ = 0.0;
        target_torque_ = 0.0;
        
        // Initialize safety variables
        last_position_cmd_time_ = ros::Time(0);
        last_velocity_cmd_time_ = ros::Time(0);
        last_torque_cmd_time_ = ros::Time(0);
        motor_stopped_by_timeout_ = false;
        any_command_received_ = false;
        
        // Set motor type enum
        if (motor_type_ == "A1")
            motor_type_enum_ = MotorType::A1;
        else if (motor_type_ == "B1")
            motor_type_enum_ = MotorType::B1;
        else if (motor_type_ == "GO_M8010_6")
            motor_type_enum_ = MotorType::GO_M8010_6;
        else {
            ROS_WARN("Unknown motor type: %s, using GO_M8010_6", motor_type_.c_str());
            motor_type_enum_ = MotorType::GO_M8010_6;
        }
        
        // Initialize serial communication
        try {
            serial_ = new SerialPort(serial_port_);
            ROS_INFO("Serial port %s opened successfully", serial_port_.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to open serial port %s: %s", serial_port_.c_str(), e.what());
            ROS_WARN("Running without serial communication (simulation mode)");
            serial_ = nullptr;
        }
        
        // Initialize motor command structure
        cmd_.motorType = motor_type_enum_;
        data_.motorType = motor_type_enum_;
        cmd_.mode = queryMotorMode(motor_type_enum_, MotorMode::FOC);
        cmd_.id = motor_id_;
        cmd_.kp = kp_;
        cmd_.kd = kd_;
        cmd_.q = 0.0;
        cmd_.dq = 0.0;
        cmd_.tau = 0.0;
        
        // Setup dynamic reconfigure
        dynamic_reconfigure::Server<go_motor_ros::MotorParamsConfig>::CallbackType f;
        f = boost::bind(&GoMotorRosNode::dynamicReconfigureCallback, this, _1, _2);
        dr_server_.setCallback(f);
        
        // Setup subscribers
        position_sub_ = nh_.subscribe("motor_command/position", 1, 
                                     &GoMotorRosNode::positionCallback, this);
        velocity_sub_ = nh_.subscribe("motor_command/velocity", 1,
                                     &GoMotorRosNode::velocityCallback, this);
        torque_sub_ = nh_.subscribe("motor_command/torque", 1,
                                   &GoMotorRosNode::torqueCallback, this);
        
        // Setup publisher
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("motor_state", 10);
        
        // Setup control timer
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                                        &GoMotorRosNode::controlCallback, this);
        
        ROS_INFO("Go Motor ROS Node initialized");
        ROS_INFO("Motor Type: %s", motor_type_.c_str());
        ROS_INFO("Serial Port: %s", serial_port_.c_str());
        ROS_INFO("Control Frequency: %.1f Hz", control_frequency_);
        ROS_INFO("Command Timeout: %.1f sec", command_timeout_);
        ROS_INFO("Motor ID: %d", motor_id_);
        ROS_INFO("Initial kp: %.2f, kd: %.2f", kp_, kd_);
        ROS_INFO("Motor Mode: FOC (Field Oriented Control)");
        
        if (!serial_) {
            ROS_WARN("Running in simulation mode - no actual motor control");
        } else {
            ROS_WARN("MOTOR CONTROL TIPS:");
            ROS_WARN("1. Set kp > 0 for position control (try kp=10.0)");
            ROS_WARN("2. Use rqt_reconfigure to adjust gains safely");
            ROS_WARN("3. Start with small position commands (±0.5 rad)");
            ROS_WARN("4. Monitor current CMD vs RECV values in logs");
        }
    }
    
    ~GoMotorRosNode()
    {
        if (serial_) {
            delete serial_;
        }
    }
    
    void dynamicReconfigureCallback(go_motor_ros::MotorParamsConfig &config, uint32_t level)
    {
        ROS_INFO("Dynamic reconfigure: kp=%.2f, kd=%.2f, id=%d", 
                 config.kp, config.kd, config.id);
        
        kp_ = config.kp;
        kd_ = config.kd;
        motor_id_ = config.id;
        
        // Update motor command structure
        cmd_.id = motor_id_;
        cmd_.kp = kp_;
        cmd_.kd = kd_;
    }
    
    void positionCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        target_position_ = msg->data;
        last_position_cmd_time_ = ros::Time::now();
        any_command_received_ = true;
        if (motor_stopped_by_timeout_) {
            ROS_INFO("Motor restarted by position command: %.3f", target_position_);
            motor_stopped_by_timeout_ = false;
        }
        ROS_DEBUG("Received position command: %.3f", target_position_);
    }
    
    void velocityCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        target_velocity_ = msg->data;
        last_velocity_cmd_time_ = ros::Time::now();
        any_command_received_ = true;
        if (motor_stopped_by_timeout_) {
            ROS_INFO("Motor restarted by velocity command: %.3f", target_velocity_);
            motor_stopped_by_timeout_ = false;
        }
        ROS_DEBUG("Received velocity command: %.3f", target_velocity_);
    }
    
    void torqueCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        target_torque_ = msg->data;
        last_torque_cmd_time_ = ros::Time::now();
        any_command_received_ = true;
        if (motor_stopped_by_timeout_) {
            ROS_INFO("Motor restarted by torque command: %.3f", target_torque_);
            motor_stopped_by_timeout_ = false;
        }
        ROS_DEBUG("Received torque command: %.3f", target_torque_);
    }
    
    void controlCallback(const ros::TimerEvent& event)
    {
        try {
            // Match exact behavior of unitree_actuator_sdk example
            // Set ALL parameters every cycle like the working example
            cmd_.motorType = motor_type_enum_;
            data_.motorType = motor_type_enum_;
            cmd_.mode = queryMotorMode(motor_type_enum_, MotorMode::FOC);
            cmd_.id = motor_id_;
            
            double gear_ratio = queryGearRatio(motor_type_enum_);
            
            // Check for command timeout
            ros::Time now = ros::Time::now();
            bool position_timeout = (now - last_position_cmd_time_).toSec() > command_timeout_;
            bool velocity_timeout = (now - last_velocity_cmd_time_).toSec() > command_timeout_;
            bool torque_timeout = (now - last_torque_cmd_time_).toSec() > command_timeout_;
            
            // If no command received yet, use timeout behavior
            if (!any_command_received_) {
                position_timeout = velocity_timeout = torque_timeout = true;
            }
            
            // Stop motor if all commands have timed out
            bool all_commands_timeout = position_timeout && velocity_timeout && torque_timeout;
            
            if (all_commands_timeout && !motor_stopped_by_timeout_) {
                ROS_WARN("Motor command timeout (%.1f sec) - stopping motor for safety", command_timeout_);
                motor_stopped_by_timeout_ = true;
            }
            
            // Set motor command - handle timeouts properly for each control mode
            if (motor_stopped_by_timeout_) {
                // Emergency stop: zero torque mode (manual section 7.6)
                cmd_.q = 0.0;
                cmd_.dq = 0.0;
                cmd_.tau = 0.0;
                cmd_.kp = 0.0;  // Zero stiffness when stopped
                cmd_.kd = 0.0;  // Zero damping for true zero torque mode
            } else {
                // Normal operation - set commands based on what's active
                cmd_.q = (position_timeout ? 0.0 : target_position_) * gear_ratio;
                cmd_.dq = (velocity_timeout ? 0.0 : target_velocity_) * gear_ratio;
                cmd_.tau = (torque_timeout ? 0.0 : target_torque_) / gear_ratio;
                
                // Set gains based on control mode (manual section 7.2-7.4)
                // Check if commands are active (not timed out) - zero values are valid commands!
                bool has_position_cmd = !position_timeout;
                bool has_velocity_cmd = !velocity_timeout;
                bool has_torque_cmd = !torque_timeout;
                
                // Determine control mode based on active commands
                // Priority: Position > Velocity > Torque (if multiple commands active)
                if (has_position_cmd) {
                    // Position control mode (manual 7.2): kp > 0, kd >= 0
                    cmd_.kp = kp_;
                    cmd_.kd = kd_;
                    ROS_DEBUG_THROTTLE(5.0, "Using Position Control Mode: kp=%.3f, kd=%.3f", cmd_.kp, cmd_.kd);
                } else if (has_velocity_cmd) {
                    // Speed control mode (manual 7.3): kp = 0, kd > 0  
                    cmd_.kp = 0.0;
                    cmd_.kd = kd_;
                    ROS_DEBUG_THROTTLE(5.0, "Using Velocity Control Mode: kp=%.3f, kd=%.3f", cmd_.kp, cmd_.kd);
                } else if (has_torque_cmd) {
                    // Torque control mode (manual 7.5): kp = 0, kd = 0
                    cmd_.kp = 0.0;
                    cmd_.kd = 0.0;
                    ROS_DEBUG_THROTTLE(5.0, "Using Torque Control Mode: kp=%.3f, kd=%.3f", cmd_.kp, cmd_.kd);
                } else {
                    // No active commands - damping mode (manual 7.4): kp = 0, kd > 0
                    cmd_.kp = 0.0;
                    cmd_.kd = kd_;
                    ROS_DEBUG_THROTTLE(5.0, "Using Damping Mode (no commands): kp=%.3f, kd=%.3f", cmd_.kp, cmd_.kd);
                }
            }
            
            // Send command and receive data
            if (serial_) {
                ROS_INFO_THROTTLE(2.0, "Control Mode - pos_cmd=%.3f(timeout=%s), vel_cmd=%.3f(timeout=%s), torque_cmd=%.3f(timeout=%s)", 
                         target_position_, position_timeout?"YES":"NO",
                         target_velocity_, velocity_timeout?"YES":"NO", 
                         target_torque_, torque_timeout?"YES":"NO");
                ROS_DEBUG_THROTTLE(1.0, "Sending cmd: type=%d, mode=%d, id=%d, q=%.3f, dq=%.3f, tau=%.3f, kp=%.3f, kd=%.3f",
                         (int)cmd_.motorType, (int)cmd_.mode, cmd_.id, cmd_.q, cmd_.dq, cmd_.tau, cmd_.kp, cmd_.kd);
                
                try {
                    serial_->sendRecv(&cmd_, &data_);
                    
                    ROS_DEBUG_THROTTLE(1.0, "Received data: q=%.3f, dq=%.3f, tau=%.3f, temp=%d, merror=%d",
                             data_.q, data_.dq, data_.tau, data_.temp, data_.merror);
                } catch (const std::exception& e) {
                    ROS_WARN("Communication error: %s", e.what());
                    // Small delay to avoid flooding the bus
                    usleep(1000);
                }
                
                // Publish joint state with actual motor data
                sensor_msgs::JointState joint_state;
                joint_state.header.stamp = ros::Time::now();
                joint_state.name.push_back("motor_" + std::to_string(motor_id_));
                joint_state.position.push_back(data_.q / gear_ratio);
                joint_state.velocity.push_back(data_.dq / gear_ratio);
                joint_state.effort.push_back(data_.tau * gear_ratio);
                
                joint_state_pub_.publish(joint_state);
                
                // Log motor state and command periodically (every 1000 cycles = 1 second at 1000Hz)
                static int log_counter = 0;
                if (++log_counter >= 1000) {
                    ROS_DEBUG("Motor %d - ROS CMD: pos=%.3f, vel=%.3f, torque=%.3f, kp=%.3f, kd=%.3f | MOTOR CMD: q=%.3f, dq=%.3f, tau=%.3f | RECV: pos=%.3f, vel=%.3f, torque=%.3f | GEAR: %.1f", 
                             motor_id_,
                             // ROS command values (what we received from ROS topics)
                             target_position_,
                             target_velocity_, 
                             target_torque_,
                             kp_,
                             kd_,
                             // Motor command values (what we sent to motor)
                             cmd_.q,
                             cmd_.dq,
                             cmd_.tau,
                             // Received values (what motor reported)
                             data_.q / gear_ratio,
                             data_.dq / gear_ratio,
                             data_.tau * gear_ratio,
                             // Debug info
                             gear_ratio
                           );
                    log_counter = 0;
                }
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Error in control callback: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_motor_ros_node");
    
    try {
        GoMotorRosNode node;
        
        ROS_INFO("Go Motor ROS Node running...");
        ROS_INFO("Use rqt_reconfigure to adjust kp, kd, id parameters");
        ROS_INFO("Send commands via rostopic:");
        ROS_INFO("  rostopic pub /go_motor_ros/motor_command/position std_msgs/Float64 'data: 1.0'");
        ROS_INFO("  rostopic pub /go_motor_ros/motor_command/velocity std_msgs/Float64 'data: 2.0'");
        ROS_INFO("  rostopic pub /go_motor_ros/motor_command/torque std_msgs/Float64 'data: 0.5'");
        
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    return 0;
}