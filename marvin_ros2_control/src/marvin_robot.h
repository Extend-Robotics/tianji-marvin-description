#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "ctime"
#include "stdio.h"
#include "L1Robot.h"
#include "math.h"
#include <array>
#include <thread>
#include <atomic>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>

constexpr double D2R = 3.14159265358979323846 / 180.0;
constexpr double R2D = 180.0 / 3.14159265358979323846;

class MarvinRobot
{
public:
    MarvinRobot();
    ~MarvinRobot();

    bool init(const std::string &robot_ip);
    int GetControllerVersion() const;

    bool init_tool_dyna(double dynPara_A[10], double dynPara_B[10]);

    bool is_connected() const;

    bool ReadState_arm(std::vector<float> &joint_positions, std::vector<float> &joint_velocities, std::vector<float> &joint_efforts);

    bool ReadState_head(std::vector<float> &joint_positions);

    bool ReadState_body(std::vector<float> &joint_positions, std::vector<float> &joint_velocities, std::vector<float> &joint_efforts);
    bool ReadState_lift(std::vector<float> &joint_positions, std::vector<float> &joint_velocities, std::vector<float> &joint_efforts);
    bool ReadCmdPos_arm(std::vector<double> &joint_cmd_positions_A, std::vector<double> &joint_cmd_positions_B);
    bool ReadCmdPos_head(std::vector<double> &joint_cmd_positions);
    bool ReadCmdPos_body(std::vector<double> &joint_cmd_positions);
    bool ReadCmdPos_lift(std::vector<double> &joint_cmd_positions);


    bool PosCmd(const std::array<double, 7> &joint_cmd_A,
                             const std::array<double, 7> &joint_cmd_B,
                             const std::array<double, 3> &joint_cmd_head,
                             const std::vector<double> &joint_cmd_body,
                             const std::vector<double> &joint_cmd_lift);
    bool emptyCmd();

    bool SetMode(int mode);
    void SetModeTargets(int mode, const std::vector<int64_t> &targets);
    void SetActiveComponents(bool arm_active, bool head_active, bool body_active, bool lift_active);

    void servo_off();



    void GetState(int (&state_cur)[6],int (&state_cmd)[6],int (&state_err)[6]) const;
    void GetState(int (&state_cur)[6]) const;

    void ClearFault();

    void GetJointHome_A(double *joint_home);

    void GetJointHome_B(double *joint_home);

    void SetKDA(std::vector<double> &k, std::vector<double> &d);
    void SetKDB(std::vector<double> &k, std::vector<double> &d);
    void SetKDL(std::vector<double> &k, std::vector<double> &d);
    void set_speed_arm(int vel_scale, int acc_scale);
    void set_speed_body(int vel_scale, int acc_scale);
    void set_speed_head(int vel_scale, int acc_scale);

    void init_can(const std::string &iface0, const std::string &iface1);
    bool verbose_logging = false;

    double Joint_pos_A[7], Joint_vel_A[7], Joint_eff_A[7];
    double Joint_pos_B[7], Joint_vel_B[7], Joint_eff_B[7];
    double pgA[4][4], pgB[4][4];
    double Joint_pos_head[3], Joint_vel_head[3], Joint_eff_head[3];
    double Joint_pos_body[6], Joint_vel_body[6], Joint_eff_body[6];

private:
    std::string robot_ip_;
    bool connected_;

    std::vector<int> parse_ip(const std::string &ip);

    int open_can_socket(const std::string &ifname);
    void forward_to_sdk();
    void forward_to_sockets();
    bool sdk_read_canA(struct can_frame &frame);
    bool sdk_read_canB(struct can_frame &frame);
    bool sdk_write_canA(const struct can_frame &frame);
    bool sdk_write_canB(const struct can_frame &frame);
    
    int sock0 = -1;
    int sock1 = -1;
    std::atomic<bool> stop_{false};
    std::thread writer_thread_;
    std::thread reader_thread_;

    double joint_home_A_[7] = {73.599, -63.546, -21.671, -48.365, 0.024, 0.005, 0.008};
    double joint_home_B_[7] = {-73.599, -63.546, 21.671, -48.365, 0.024, 0.005, 0.008};
    double acc_limit, vel_limit;

    double K_A[7] = {5, 3, 2, 3, 2, 1, 1};
    double D_A[7] = {0.4, 0.3, 0.2, 0.3, 0.2, 0.1, 0.1};
    double K_B[7] = {5, 3, 2, 3, 2, 1, 1};
    double D_B[7] = {0.4, 0.3, 0.2, 0.3, 0.2, 0.1, 0.1};
    double K_L[6] = {12, 12, 12, 8, 6, 4};
    double D_L[6] = {4, 4, 4, 3, 2, 1};
    double K_Ac[7] = {1, 1, 1, 1, 1, 1, 1};
    double D_Ac[7] = {0.4, 0.3, 0.2, 0.3, 0.2, 0.1, 0.1};
    double K_Bc[7] = {1, 1, 1, 1, 1, 1, 1};
    double D_Bc[7] = {0.4, 0.3, 0.2, 0.3, 0.2, 0.1, 0.1};
    std::array<int, 4> mode1_targets_ = {1, 1, 1, 1};
    std::array<int, 4> mode3_targets_ = {3, 1, 1, 1};
    bool arm_active_ = true;
    bool head_active_ = true;
    bool body_active_ = true;
    bool lift_active_ = true;

    FX_INT32 ErrCode_A, ErrCode_B, ErrCode_Head, ErrCode_Body, ErrCode_Lift;
    FX_INT32 ArmState_A, ArmState_B, Head_State, Body_State, Lift_State;
    FX_INT32 ArmState_A_cmd, ArmState_B_cmd, Head_State_cmd, Body_State_cmd, Lift_State_cmd;
    int low_spd_flag_A=0, low_spd_flag_B=0, low_spd_flag_Body=0, low_spd_flag_Head=0;
    long MotorErrcodeA[7] = {-1, -1, -1, -1, -1, -1, -1};
    long MotorErrcodeB[7] = {-1, -1, -1, -1, -1, -1, -1};
    long MotorErrcodeHead[3] = {-1, -1, -1};
    long MotorErrcodeBody[6] = {-1, -1, -1, -1, -1, -1};
};
