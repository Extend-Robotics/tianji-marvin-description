#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <array>
#include <algorithm>
#include <sstream>
#include "ctime"
#include "stdio.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "marvin_robot.h"
#include "marvin_msgs/msg/jointcmd_arm.hpp"
#include "marvin_msgs/msg/jointcmd_body.hpp"
#include "marvin_msgs/msg/jointcmd_head.hpp"
#include "marvin_msgs/msg/jointfeedback.hpp"
#include "marvin_msgs/msg/robot_info.hpp"
#include "marvin_msgs/srv/int.hpp"
#include "marvin_msgs/srv/velratio.hpp"
#include "marvin_msgs/srv/motor_err_code.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <poll.h>
#include <iostream>
class MarvinRobotNode : public rclcpp::Node
{
public:
    MarvinRobotNode() : Node("marvin_robot_node")
    {
        rclcpp::SensorDataQoS sensor_data_qos;
        rclcpp::QoS control_qos(rclcpp::KeepLast(1));
        control_qos.best_effort();
        control_qos.durability_volatile();
        this->declare_parameter<std::vector<std::string>>("left_joints");
        this->declare_parameter<std::vector<std::string>>("right_joints");
        this->declare_parameter<std::vector<std::string>>("head_joints");
        this->declare_parameter<std::vector<std::string>>("body_joints");
        this->declare_parameter<std::vector<std::string>>("lift_joints", std::vector<std::string>{});
        this->declare_parameter<std::vector<double>>("left_pd.k", default_k);
        this->declare_parameter<std::vector<double>>("left_pd.d", default_d);
        this->declare_parameter<std::vector<double>>("right_pd.k", default_k);
        this->declare_parameter<std::vector<double>>("right_pd.d", default_d);
        this->declare_parameter<std::vector<double>>("body_pd.k");
        this->declare_parameter<std::vector<double>>("body_pd.d");
        this->declare_parameter<std::string>("robot_ip", "192.168.1.190");
        this->declare_parameter<std::vector<double>>("left_tool");
        this->declare_parameter<std::vector<double>>("right_tool");
        this->declare_parameter<int>("speed_ratio",50);
        this->declare_parameter<int>("accel_ratio",50);
        this->declare_parameter<std::string>("robot_type", "unknown");
        this->declare_parameter<std::string>("robot_info_topic", "info/robot_info");
        this->declare_parameter<double>("robot_info_publish_period_s", 5.0);
        this->declare_parameter<std::vector<int64_t>>("mode1_targets", std::vector<int64_t>{1, 1, 1, 1});
        this->declare_parameter<std::vector<int64_t>>("mode3_targets", std::vector<int64_t>{3, 1, 1, 1});
        this->declare_parameter<std::vector<int64_t>>("arm_supported_control_modes", std::vector<int64_t>{2, 3});
        this->declare_parameter<std::vector<int64_t>>("body_supported_control_modes", std::vector<int64_t>{1, 2});
        this->declare_parameter<std::vector<int64_t>>("lift_supported_control_modes", std::vector<int64_t>{1});
        
        JointNames_A = this->get_parameter("left_joints").as_string_array();
        JointNames_B = this->get_parameter("right_joints").as_string_array();
        JointNames_Head = this->get_parameter("head_joints").as_string_array();
        JointNames_Body = this->get_parameter("body_joints").as_string_array();
        JointNames_Lift = this->get_parameter("lift_joints").as_string_array();
        arm_active_ = !JointNames_A.empty() || !JointNames_B.empty();
        head_active_ = !JointNames_Head.empty();
        body_active_ = !JointNames_Body.empty();
        lift_active_ = !JointNames_Lift.empty();
        std::cout << "Arm active: " << arm_active_ << std::endl;
        std::cout << "Head active: " << head_active_ << std::endl;
        std::cout << "Body active: " << body_active_ << std::endl;
        std::cout << "Lift active: " << lift_active_ << std::endl;
        std::cout << "Joint names loading" << std::endl;
        JointNames_All.clear();
        for (size_t i = 0; i < 7; i++)
        {
            JointNames_All.push_back(JointNames_A[i]);
        }
        for (size_t i = 0; i < 7; i++)
        {
            JointNames_All.push_back(JointNames_B[i]);
        }
        for (size_t i = 0; i < JointNames_Lift.size(); i++)
        {
            JointNames_All.push_back(JointNames_Lift[i]);
        }
        for (size_t i = 0; i < JointNames_Body.size(); i++)
        {
            JointNames_All.push_back(JointNames_Body[i]);
        }
        for (size_t i = 0; i < JointNames_Head.size(); i++)
        {
            JointNames_All.push_back(JointNames_Head[i]);
        }

        std::cout << "Joint names loaded from parameters:" << std::endl;
        joint_cmd_body_.assign(JointNames_Body.size(), 0.0);
        joint_cmd_lift_.assign(JointNames_Lift.size(), 0.0);
        arm_supported_control_modes_ = this->get_parameter("arm_supported_control_modes").as_integer_array();
        body_supported_control_modes_ = this->get_parameter("body_supported_control_modes").as_integer_array();
        lift_supported_control_modes_ = this->get_parameter("lift_supported_control_modes").as_integer_array();
        
        marvin_robot_ = new MarvinRobot();
        

        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        if (marvin_robot_->init(robot_ip))
        {
            RCLCPP_INFO(this->get_logger(), "Robot successfully connected.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to the robot.");
        }
        robot_type_ = this->get_parameter("robot_type").as_string();
        controller_version_ = marvin_robot_->GetControllerVersion();
        controller_version_text_ = format_controller_version(controller_version_);
        RCLCPP_INFO(
            this->get_logger(),
            "Robot info initialized: type=%s, controller_version=%d (%s)",
            robot_type_.c_str(),
            controller_version_,
            controller_version_text_.c_str());

        marvin_robot_->ClearFault();
        marvin_robot_->SetActiveComponents(arm_active_, head_active_, body_active_, lift_active_);
        marvin_robot_->SetModeTargets(1, this->get_parameter("mode1_targets").as_integer_array());
        marvin_robot_->SetModeTargets(3, this->get_parameter("mode3_targets").as_integer_array());
        marvin_robot_->SetMode(0);
        std::vector<double> kA = this->get_parameter("left_pd.k").as_double_array();
        std::vector<double> dA = this->get_parameter("left_pd.d").as_double_array();
        std::vector<double> kB = this->get_parameter("right_pd.k").as_double_array();
        std::vector<double> dB = this->get_parameter("right_pd.d").as_double_array();
        std::vector<double> kl = this->get_parameter("body_pd.k").as_double_array();
        std::vector<double> dl = this->get_parameter("body_pd.d").as_double_array();
        int speed_ratio = this->get_parameter("speed_ratio").as_int();
        int accel_ratio = this->get_parameter("accel_ratio").as_int();
        RCLCPP_INFO(this->get_logger(), "speed ratio %d",speed_ratio);
        RCLCPP_INFO(this->get_logger(), "acc ratio %d",accel_ratio);
        marvin_robot_->SetKDA(kA, dA);
        marvin_robot_->SetKDB(kB, dB);
        marvin_robot_->SetKDL(kl,dl);
        marvin_robot_->set_speed_arm(100, 100);
        marvin_robot_->set_speed_body(speed_ratio, accel_ratio);
        marvin_robot_->set_speed_head(100, 100);
        std::vector<double> toolA = this->get_parameter("left_tool").as_double_array();
        std::vector<double> toolB = this->get_parameter("right_tool").as_double_array();
        double dynPara_A[10]= {toolA[0], toolA[1], toolA[2], toolA[3], toolA[4], toolA[5], toolA[6], toolA[7], toolA[8], toolA[9]};
        double dynPara_B[10]= {toolB[0], toolB[1], toolB[2], toolB[3], toolB[4], toolB[5], toolB[6], toolB[7], toolB[8], toolB[9]};
        marvin_robot_->init_tool_dyna(dynPara_A, dynPara_B);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MarvinRobotNode::timer_ctrl_callback, this));



        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_feedback_pub_ = this->create_publisher<marvin_msgs::msg::Jointfeedback>("info/joint_feedback", sensor_data_qos);
        cmd_pos_pub_A_ = this->create_publisher<marvin_msgs::msg::JointcmdArm>("info/cmd_pos_A", sensor_data_qos);
        cmd_pos_pub_B_ = this->create_publisher<marvin_msgs::msg::JointcmdArm>("info/cmd_pos_B", sensor_data_qos);
        cmd_pos_pub_body_ = this->create_publisher<marvin_msgs::msg::JointcmdBody>("info/cmd_pos_body", sensor_data_qos);
        cmd_pos_pub_head_ = this->create_publisher<marvin_msgs::msg::JointcmdHead>("info/cmd_pos_head", sensor_data_qos);
        joint_cmd_sub_A_ = this->create_subscription<marvin_msgs::msg::JointcmdArm>(
            "control/joint_cmd_A", control_qos,
            std::bind(&MarvinRobotNode::joint_cmd_callback_A, this, std::placeholders::_1));
        joint_cmd_sub_B_ = this->create_subscription<marvin_msgs::msg::JointcmdArm>(
            "control/joint_cmd_B", control_qos,
            std::bind(&MarvinRobotNode::joint_cmd_callback_B, this, std::placeholders::_1));
        joint_cmd_sub_body_ = this->create_subscription<marvin_msgs::msg::JointcmdBody>(
            "control/joint_cmd_body", control_qos,
            std::bind(&MarvinRobotNode::joint_cmd_callback_body, this, std::placeholders::_1));
        joint_cmd_sub_head_ = this->create_subscription<marvin_msgs::msg::JointcmdHead>(
            "control/joint_cmd_head", control_qos,
            std::bind(&MarvinRobotNode::joint_cmd_callback_head, this, std::placeholders::_1));
        set_mode_service_ = this->create_service<marvin_msgs::srv::Int>(
            "control/set_mode",
            std::bind(&MarvinRobotNode::set_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
        clear_fault_service_ = this->create_service<std_srvs::srv::Trigger>(
            "control/clear_fault",
            std::bind(&MarvinRobotNode::clearfault_callback, this, std::placeholders::_1, std::placeholders::_2));

        set_ready_service_ = this->create_service<std_srvs::srv::Trigger>(
            "control/set_ready",
            std::bind(&MarvinRobotNode::set_ready_callback, this, std::placeholders::_1, std::placeholders::_2));
        arm_state_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("info/robot_state", 10);
        cmd_state_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("info/robot_cmd_state", 10);
        robot_info_pub_ = this->create_publisher<marvin_msgs::msg::RobotInfo>(
            this->get_parameter("robot_info_topic").as_string(), 10);
        const double robot_info_publish_period_s = std::max(
            0.1,
            this->get_parameter("robot_info_publish_period_s").as_double());
        robot_info_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(robot_info_publish_period_s)),
            std::bind(&MarvinRobotNode::publish_robot_info, this));
        publish_robot_info();

        rclcpp::on_shutdown(std::bind(&MarvinRobotNode::Shutdown, this));
        RCLCPP_INFO(this->get_logger(), "Marvin Robot Node initialized.");
    }

private:

    

    void timer_ctrl_callback()
    {
        if (marvin_robot_->is_connected())
        {
            ReadRobotState();
            if (motion_mode && all_ready)
            {   
                marvin_robot_->PosCmd(joint_cmd_A_, joint_cmd_B_,
                                     joint_cmd_head_, joint_cmd_body_, joint_cmd_lift_);
            }
           
            
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Robot not connected. Skipping state retrieval.");
            usleep(5000000);
        }
        
    }

    static std::string format_controller_version(const int version)
    {
        if (version < 0)
        {
            return "error:" + std::to_string(version);
        }
        const int major = (version >> 16) & 0xff;
        const int minor = (version >> 8) & 0xff;
        const int patch = version & 0xff;
        std::ostringstream out;
        out << major << "." << minor << "." << patch;
        return out.str();
    }

    void publish_robot_info()
    {
        marvin_msgs::msg::RobotInfo msg;
        msg.header.stamp = this->now();
        msg.robot_type = robot_type_;
        msg.controller_version = controller_version_;
        msg.controller_version_text = controller_version_text_;
        robot_info_pub_->publish(msg);
    }

    void ReadRobotState()
    {
        std::vector<float> arm_joint_positions, arm_joint_velocities, arm_joint_efforts;
        std::vector<float> body_joint_positions, body_joint_velocities, body_joint_efforts;
        std::vector<float> lift_joint_positions, lift_joint_velocities, lift_joint_efforts;
        std::vector<float> head_joint_positions;
        marvin_robot_->ReadState_arm(arm_joint_positions, arm_joint_velocities, arm_joint_efforts);
        marvin_robot_->ReadState_body(body_joint_positions, body_joint_velocities, body_joint_efforts);
        marvin_robot_->ReadState_lift(lift_joint_positions, lift_joint_velocities, lift_joint_efforts);
        marvin_robot_->ReadState_head(head_joint_positions);
        
        sensor_msgs::msg::JointState joint_state_msg;
        marvin_msgs::msg::Jointfeedback joint_feedback_msg;

        joint_feedback_msg.header.stamp = this->now();
        joint_state_msg.header.stamp = this->now();
 

        joint_state_msg.name = JointNames_All;

        joint_state_msg.position.resize(JointNames_All.size(), 0.0);
        joint_state_msg.velocity.resize(JointNames_All.size(), 0.0);
        joint_state_msg.effort.resize(JointNames_All.size(), 0.0);
        for (size_t i = 0; i < 14; ++i)
        {
            joint_state_msg.position[i] = static_cast<double>(arm_joint_positions[i]);
            joint_state_msg.velocity[i] = static_cast<double>(arm_joint_velocities[i]);
            joint_state_msg.effort[i] = static_cast<double>(arm_joint_efforts[i]);
            joint_feedback_msg.arm_positions[i] = static_cast<double>(arm_joint_positions[i]);
            joint_feedback_msg.arm_velocities[i] = static_cast<double>(arm_joint_velocities[i]);
            joint_feedback_msg.arm_efforts[i] = static_cast<double>(arm_joint_efforts[i]);

        }
        if (!all_ready){
             for (size_t i = 0;  i < 7; ++i)
            {
                joint_cmd_A_[i] = arm_joint_positions[i];
                joint_cmd_B_[i] = arm_joint_positions[i+7];

            }
        }
        size_t joint_state_offset = 14;
        size_t feedback_body_index = 0;
        for (size_t i = 0; i < JointNames_Lift.size(); ++i)
        {
            if (i >= lift_joint_positions.size())
            {
                break;
            }
            joint_state_msg.position[joint_state_offset + i] = static_cast<double>(lift_joint_positions[i]);
            joint_state_msg.velocity[joint_state_offset + i] = static_cast<double>(lift_joint_velocities[i]);
            joint_state_msg.effort[joint_state_offset + i] = static_cast<double>(lift_joint_efforts[i]);
            if (feedback_body_index < joint_feedback_msg.body_positions.size()) {
                joint_feedback_msg.body_positions[feedback_body_index] = static_cast<double>(lift_joint_positions[i]);
                joint_feedback_msg.body_velocities[feedback_body_index] = static_cast<double>(lift_joint_velocities[i]);
                joint_feedback_msg.body_efforts[feedback_body_index] = static_cast<double>(lift_joint_efforts[i]);
                ++feedback_body_index;
            }
            if (!all_ready && i < joint_cmd_lift_.size()){
                joint_cmd_lift_[i] = lift_joint_positions[i];
            }
        }
        joint_state_offset += JointNames_Lift.size();

        for (size_t i = 0; i < JointNames_Body.size(); ++i)
        {
            if (i >= body_joint_positions.size())
            {
                break;
            }
            joint_state_msg.position[joint_state_offset + i] = static_cast<double>(body_joint_positions[i]);
            joint_state_msg.velocity[joint_state_offset + i] = static_cast<double>(body_joint_velocities[i]);
            joint_state_msg.effort[joint_state_offset + i] = static_cast<double>(body_joint_efforts[i]);
            if (feedback_body_index < joint_feedback_msg.body_positions.size()) {
                joint_feedback_msg.body_positions[feedback_body_index] = static_cast<double>(body_joint_positions[i]);
                joint_feedback_msg.body_velocities[feedback_body_index] = static_cast<double>(body_joint_velocities[i]);
                joint_feedback_msg.body_efforts[feedback_body_index] = static_cast<double>(body_joint_efforts[i]);
                ++feedback_body_index;
            }
            if (!all_ready && i < joint_cmd_body_.size()){
                joint_cmd_body_[i] = body_joint_positions[i];
            }
        }
        joint_state_offset += JointNames_Body.size();

        for (size_t i = 0; i < JointNames_Head.size(); ++i)
        {
            if (i >= head_joint_positions.size())
            {
                break;
            }
            joint_state_msg.position[joint_state_offset + i] = static_cast<double>(head_joint_positions[i]);
            joint_state_msg.velocity[joint_state_offset + i] = 0.0;
            joint_state_msg.effort[joint_state_offset + i] = 0.0;
            if (i < joint_feedback_msg.head_positions.size()) {
                joint_feedback_msg.head_positions[i] = static_cast<double>(head_joint_positions[i]);
                joint_feedback_msg.head_velocities[i] = 0.0;
                joint_feedback_msg.head_efforts[i] = 0.0;
            }
            if (!all_ready){
                joint_cmd_head_[i] = head_joint_positions[i];
            }
        }
        joint_feedback_pub_->publish(joint_feedback_msg);

        if(counter_%5==0 ){
            joint_state_pub_->publish(joint_state_msg);
            PublishCmdPositions();
            int robot_state_cur[6],robot_state_cmd[6],robot_state_err[6];
            marvin_robot_->GetState(robot_state_cur, robot_state_cmd, robot_state_err);
            const bool arms_in_control_mode = !arm_active_ || (contains_mode(arm_supported_control_modes_, robot_state_cur[0]) && contains_mode(arm_supported_control_modes_, robot_state_cur[1]));
            const bool body_in_control_mode = !body_active_ || contains_mode(body_supported_control_modes_, robot_state_cur[3]);
            const bool lift_in_control_mode = !lift_active_ || contains_mode(lift_supported_control_modes_, robot_state_cur[4]);
            motion_mode = arms_in_control_mode && body_in_control_mode && lift_in_control_mode;

            std_msgs::msg::Int16MultiArray state_msg;
            for (int i = 0; i < 6; i++) {
                state_msg.data.push_back(static_cast<int16_t>(robot_state_cur[i]));
            }
            arm_state_pub_->publish(state_msg);
            counter_ = 0;
        }
        counter_++;
    }

    void PublishCmdPositions()
    {
        std::vector<double> cmd_A, cmd_B, cmd_body, cmd_lift, cmd_head;
        marvin_robot_->ReadCmdPos_arm(cmd_A, cmd_B);
        marvin_robot_->ReadCmdPos_body(cmd_body);
        marvin_robot_->ReadCmdPos_lift(cmd_lift);
        marvin_robot_->ReadCmdPos_head(cmd_head);

        marvin_msgs::msg::JointcmdArm cmd_A_msg;
        marvin_msgs::msg::JointcmdArm cmd_B_msg;
        marvin_msgs::msg::JointcmdBody cmd_body_msg;
        marvin_msgs::msg::JointcmdHead cmd_head_msg;
        const auto stamp = this->now();
        cmd_A_msg.header.stamp = stamp;
        cmd_B_msg.header.stamp = stamp;
        cmd_body_msg.header.stamp = stamp;
        cmd_head_msg.header.stamp = stamp;

        for (size_t i = 0; i < cmd_A_msg.positions.size() && i < cmd_A.size(); ++i) {
            cmd_A_msg.positions[i] = cmd_A[i];
        }
        for (size_t i = 0; i < cmd_B_msg.positions.size() && i < cmd_B.size(); ++i) {
            cmd_B_msg.positions[i] = cmd_B[i];
        }
        size_t body_msg_index = 0;
        for (size_t i = 0; body_msg_index < cmd_body_msg.positions.size() && i < JointNames_Lift.size() && i < cmd_lift.size(); ++i) {
            cmd_body_msg.positions[body_msg_index++] = cmd_lift[i];
        }
        for (size_t i = 0; body_msg_index < cmd_body_msg.positions.size() && i < JointNames_Body.size() && i < cmd_body.size(); ++i) {
            cmd_body_msg.positions[body_msg_index++] = cmd_body[i];
        }
        for (size_t i = 0; i < cmd_head_msg.positions.size() && i < cmd_head.size(); ++i) {
            cmd_head_msg.positions[i] = cmd_head[i];
        }

        cmd_pos_pub_A_->publish(cmd_A_msg);
        cmd_pos_pub_B_->publish(cmd_B_msg);
        cmd_pos_pub_body_->publish(cmd_body_msg);
        cmd_pos_pub_head_->publish(cmd_head_msg);
    }

    bool contains_mode(const std::vector<int64_t>& supported_modes, int state) const
    {
        return std::find(supported_modes.begin(), supported_modes.end(), static_cast<int64_t>(state)) != supported_modes.end();
    }










    void set_ready_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (marvin_robot_->is_connected())
        {
            all_ready = true;
            response->success = true;
            response->message = "Robot is ready.";
            RCLCPP_INFO(this->get_logger(), "Robot is ready.");
        }
        else
        {
            response->success = false;
            response->message = "Robot not connected. Cannot set ready state.";
            RCLCPP_WARN(this->get_logger(), "Robot not connected. Cannot set ready state.");
        }
    }

    void joint_cmd_callback_A(const marvin_msgs::msg::JointcmdArm::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if ((now - msg->header.stamp).seconds() > 0.1)
        {
            return;
        }

        if (motion_mode)
        {
    
            if (msg->positions.size() == 7)
            {

                for (size_t i = 0; i < msg->positions.size(); ++i)
                {
                    joint_cmd_A_[i] = msg->positions[i];
                }
            }
            else
            {
                std::cout << msg->positions.size() << std::endl;
                RCLCPP_ERROR(this->get_logger(), "Received joint command with incorrect number of positions.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Robot not in postion or impedance mode.");
        }
    }

    void joint_cmd_callback_B(const marvin_msgs::msg::JointcmdArm::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if ((now - msg->header.stamp).seconds() > 0.1)
        {
            return;
        }

        if (motion_mode)
        {
         

            if (msg->positions.size() == 7)
            {

                for (size_t i = 0; i < msg->positions.size(); ++i)
                {
                    joint_cmd_B_[i] = msg->positions[i];
                }
            }
            else
            {
                std::cout << msg->positions.size() << std::endl;
                RCLCPP_ERROR(this->get_logger(), "Received joint command with incorrect number of positions.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Robot not in position or impedance mode.");
        }
    }

        void joint_cmd_callback_body(const marvin_msgs::msg::JointcmdBody::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if ((now - msg->header.stamp).seconds() > 0.1)
        {
            return;
        }

        if (motion_mode)
        {
          

            const size_t expected_positions = JointNames_Lift.size() + JointNames_Body.size();
            if (true)
            {
                size_t src = 0;
                for (size_t i = 0; i < joint_cmd_lift_.size(); ++i)
                {
                    joint_cmd_lift_[i] = msg->positions[src++];
                }
                for (size_t i = 0; i < joint_cmd_body_.size(); ++i)
                {
                    joint_cmd_body_[i] = msg->positions[src++];
                }
            }
            else
            {
                std::cout << msg->positions.size() << std::endl;
                RCLCPP_ERROR(this->get_logger(), "Received joint command with incorrect number of positions. Expected %zu.", expected_positions);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Robot not in position or impedance mode.");
        }
    }

        void joint_cmd_callback_head(const marvin_msgs::msg::JointcmdHead::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if ((now - msg->header.stamp).seconds() > 0.1)
        {
            return;
        }

        if (motion_mode)
        {
           

            const size_t expected_positions = joint_cmd_head_.size();
            if (msg->positions.size() == expected_positions)
            {

                const size_t active_head_dof = std::min(JointNames_Head.size(), joint_cmd_head_.size());
                for (size_t i = 0; i < active_head_dof; ++i)
                {
                    joint_cmd_head_[i] = msg->positions[i];
                }
            }
            else
            {
                std::cout << msg->positions.size() << std::endl;
                RCLCPP_ERROR(this->get_logger(), "Received joint command with incorrect number of positions. Expected %zu.", expected_positions);
            }
        }
    }



    void set_mode_callback(const std::shared_ptr<marvin_msgs::srv::Int::Request> request,
                           std::shared_ptr<marvin_msgs::srv::Int::Response> response)
    {   
        control_enabled = false;
        if (open_tool_ch){
            open_tool_ch = false;
            RCLCPP_INFO(this->get_logger(), "Tool communication channel closed for mode change.");
        }
        if (marvin_robot_->is_connected())
        {
            if (!all_ready)
            {
                response->success = false;
                response->message = "Robot not ready. Cannot set mode.";
                RCLCPP_WARN(this->get_logger(), "Robot not ready. Cannot set mode.");
                control_enabled = true;
                return;
            }
            int mode = request->data;
            if (mode != 0 && mode != 1 && mode != 3)
            {
                response->success = false;
                response->message = "Invalid mode requested. Valid modes are 0, 1, or 3. 2 is not used";
                RCLCPP_ERROR(this->get_logger(), "Invalid mode requested: %d. Valid modes are 0, 1, or 3. 2 is not used", mode);
                control_enabled = true;
                return;
            }
           
            if (marvin_robot_->SetMode(mode))
            {
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "Mode set to %d successfully.", mode);
                control_enabled = true;
            }
            else
            {
                response->success = false;
                RCLCPP_ERROR(this->get_logger(), "Failed to set mode to %d.", mode);
                control_enabled = true;
            }
        }
        else
        {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Robot not connected. Cannot set mode.");
            control_enabled = true;
        }
    }

        





    void clearfault_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        (void)response;
        marvin_robot_->ClearFault();
    }






    void Shutdown()
    {
        if (marvin_robot_->is_connected())
        {
            marvin_robot_->ClearFault();

            marvin_robot_->servo_off();
            marvin_robot_->ClearFault();
            close(sock0);
            close(sock1);
            std::cout << "Shutdown complete. Robot servos turned off and faults cleared." << std::endl;
        }
        else
        {
            std::cout << "Robot not connected. No shutdown actions performed." << std::endl;
        }
    }

    std::vector<std::string> JointNames_A, JointNames_B, JointNamesBoth, JointNames_Head, JointNames_Body, JointNames_Lift, JointNames_All;

    MarvinRobot* marvin_robot_;
    rclcpp::TimerBase::SharedPtr timer_, timer2_;
    rclcpp::TimerBase::SharedPtr robot_info_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_A_pub_;
    rclcpp::Subscription<marvin_msgs::msg::JointcmdArm>::SharedPtr joint_cmd_sub_A_, joint_cmd_sub_B_;
     rclcpp::Subscription<marvin_msgs::msg::JointcmdBody>::SharedPtr joint_cmd_sub_body_;
    rclcpp::Subscription<marvin_msgs::msg::JointcmdHead>::SharedPtr joint_cmd_sub_head_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_A_, collision_sub_B_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Jog_subscriber;

    rclcpp::Service<marvin_msgs::srv::Int>::SharedPtr set_mode_service_;
    rclcpp::Service<marvin_msgs::srv::Int>::SharedPtr vel_ratio_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_ready_service_;
    rclcpp::Publisher<marvin_msgs::msg::Jointfeedback>::SharedPtr joint_feedback_pub_;
    rclcpp::Publisher<marvin_msgs::msg::JointcmdArm>::SharedPtr cmd_pos_pub_A_, cmd_pos_pub_B_;
    rclcpp::Publisher<marvin_msgs::msg::JointcmdBody>::SharedPtr cmd_pos_pub_body_;
    rclcpp::Publisher<marvin_msgs::msg::JointcmdHead>::SharedPtr cmd_pos_pub_head_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr arm_state_pub_, cmd_state_pub_;
    rclcpp::Publisher<marvin_msgs::msg::RobotInfo>::SharedPtr robot_info_pub_;
    rclcpp::Service<marvin_msgs::srv::MotorErrCode>::SharedPtr motor_err_code_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
    std::vector<double> default_k = {2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0};
    std::vector<double> default_d = {0.6, 0.6, 0.6, 0.4, 0.2, 0.2, 0.2};
    std::vector<double> default_kl = {12, 12, 12, 8, 6, 4};
    std::vector<double> default_dl = {4, 4, 4, 3, 2, 1};


    bool collision_status_A = true, collision_status_B = true;
    bool jog_mode = false;
    std::array<double, 7> joint_cmd_A_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 7> joint_cmd_B_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 3> joint_cmd_head_ = {0.0, 0.0, 0.0};
    std::vector<double> joint_cmd_body_;
    std::vector<double> joint_cmd_lift_;
    std::vector<int64_t> arm_supported_control_modes_;
    std::vector<int64_t> body_supported_control_modes_;
    std::vector<int64_t> lift_supported_control_modes_;

    bool all_ready = false, motion_mode = false;
    bool arm_active_ = true, head_active_ = true, body_active_ = true, lift_active_ = true;
    int counter_=0;
    unsigned char data_ptrA_in[256], data_ptrB_in[256];
    unsigned char data_ptrA_out[256] = {0};
    unsigned char data_ptrB_out[256] = {0};
    long ret_chA=1, ret_chB=1;
    long dataA_len = 0, dataB_len = 0;
    bool open_tool_ch = false;
    int sock_;
    std::atomic<bool> stop_;
    std::thread writer_thread_;
    std::thread reader_thread_;
    int sock0, sock1;
    bool verbose_logging = false;
    bool pub_joint_states = true;
    std::string robot_type_ = "unknown";
    int controller_version_ = -1;
    std::string controller_version_text_ = "unknown";

    std::atomic<bool> control_enabled;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarvinRobotNode>());
    rclcpp::shutdown();
    return 0;
}
