#include "marvin_robot.h"
#include <iostream>
#include <cstring>
#include <unistd.h>

namespace
{
constexpr unsigned int kL1LogLevel = 0;
constexpr unsigned int kL1TimeoutMs = 1000;
constexpr FXChnType kTerminalChannel = static_cast<FXChnType>(1);

bool l1_ok(int ret)
{
    return ret == FUNC_RET_SUCCESS;
}

bool l1_send()
{
    return l1_ok(FX_L1_Comm_Send());
}

bool switch_obj_to_target(FXObjType obj, int target_state, double vel_ratio, double acc_ratio, double k[7], double d[7])
{
    switch (target_state)
    {
    case 0:
        return l1_ok(FX_L1_State_SwitchToIdle(obj, kL1TimeoutMs));
    case 1:
        return l1_ok(FX_L1_State_SwitchToPositionMode(obj, kL1TimeoutMs, vel_ratio, acc_ratio));
    case 2:
        return l1_ok(FX_L1_State_SwitchToPDMode(obj, kL1TimeoutMs, vel_ratio, acc_ratio, k, d));
    case 3:
        return l1_ok(FX_L1_State_SwitchToCollaborativeRelease(obj, kL1TimeoutMs));
    default:
        std::cout << "[ERROR] marvin_robot: unsupported target state " << target_state << std::endl;
        return false;
    }
}
}

    MarvinRobot::MarvinRobot() {}
    bool MarvinRobot::init(const std::string &robot_ip)
    {
        std::vector<int> ip_parts = parse_ip(robot_ip);

        if (ip_parts.size() != 4)
        {
            std::cout << "[ERROR] marvin_robot: Invalid IP address format." << std::endl;
            return false;
        }

        long link_ms = FX_L1_System_Link(
            static_cast<unsigned char>(ip_parts[0]),
            static_cast<unsigned char>(ip_parts[1]),
            static_cast<unsigned char>(ip_parts[2]),
            static_cast<unsigned char>(ip_parts[3]),
            kL1LogLevel);
        std::cout << "Link ms: " << link_ms << std::endl;
        if (link_ms < 0)
        {
            std::cout << "[ERROR] marvin_robot: Failed to link to robot at IP: " << robot_ip << std::endl;
            connected_ = false;
            return false;
        }
        else
        {
            connected_ = true;
            std::cout << "[INFO] marvin_robot: Successfully linked to robot at IP: " << robot_ip
                      << ", latency: " << link_ms << " ms" << std::endl;

            FX_L1_Comm_Clear(0);
            FX_L1_Runtime_SetSpeedRatio(FX_OBJ_ARM0, 100.0, 100.0);
            FX_L1_Runtime_SetSpeedRatio(FX_OBJ_ARM1, 100.0, 100.0);
            FX_L1_Runtime_SetSpeedRatio(FX_OBJ_BODY, 100.0, 100.0);
            FX_L1_Runtime_SetSpeedRatio(FX_OBJ_HEAD, 100.0, 100.0);
            FX_L1_Runtime_SetJointKD(FX_OBJ_ARM0, K_A, D_A);
            FX_L1_Runtime_SetJointKD(FX_OBJ_ARM1, K_B, D_B);
            FX_L1_Comm_Send();
            usleep(5000);

            init_can("vcan0", "vcan1");
            return true;
        }

    }

   

    bool MarvinRobot::is_connected() const
    {
        return connected_;
    }

    int MarvinRobot::GetControllerVersion() const
    {
        return FX_L1_System_GetControllerVersion();
    }

    bool MarvinRobot::init_tool_dyna(double dynPara_A[10], double dynPara_B[10])
    {
        FX_L1_Comm_Clear(0);
        FX_L1_Runtime_SetToolD(FX_OBJ_ARM0, dynPara_A);
        FX_L1_Runtime_SetToolD(FX_OBJ_ARM1, dynPara_B);
        return l1_send();
    }

    bool MarvinRobot::ReadState_arm(std::vector<float>& joint_positions, std::vector<float>& joint_velocities, std::vector<float>& joint_efforts){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        const ROBOT_SG* pRobotSG = FX_L1_Fbk_GetSG();
        if (pRobotRT == nullptr || pRobotSG == nullptr) {
            return false;
        }
        StateCtr state_A = pRobotRT->m_ARMS[0].m_ARM_State;
        StateCtr state_B = pRobotRT->m_ARMS[1].m_ARM_State;
        ArmState_A = state_A.m_CurState;
        ArmState_A_cmd = state_A.m_CmdState;
        ArmState_B = state_B.m_CurState;
        ArmState_B_cmd = state_B.m_CmdState;
        ErrCode_A = state_A.m_ERRCode;
        ErrCode_B = state_B.m_ERRCode;
        low_spd_flag_A = pRobotSG->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag;
        low_spd_flag_B = pRobotSG->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag;
        low_spd_flag_Body = 1;
        low_spd_flag_Head = 1;

        joint_positions.clear();
        joint_velocities.clear();
        joint_efforts.clear();

        for (int i = 0; i < 7; ++i) {
            Joint_pos_A[i] = pRobotRT->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i] * D2R ;
            joint_positions.push_back(Joint_pos_A[i]);
            Joint_vel_A[i] = pRobotRT->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Vel[i] * D2R;
            joint_velocities.push_back(Joint_vel_A[i]);
            Joint_eff_A[i] = pRobotRT->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
            joint_efforts.push_back(Joint_eff_A[i]);
        }
        for (int i = 0; i < 7; ++i) {
            Joint_pos_B[i] = pRobotRT->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i] * D2R ;
            joint_positions.push_back(Joint_pos_B[i]);
            Joint_vel_B[i] = pRobotRT->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Vel[i] * D2R;
            joint_velocities.push_back(Joint_vel_B[i]);
            Joint_eff_B[i] = pRobotRT->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
            joint_efforts.push_back(Joint_eff_B[i]);
        }
        
        return true;
    }

    bool MarvinRobot::ReadState_head(std::vector<float>& joint_positions){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }
        auto state = pRobotRT->m_HEAD.m_HEAD_State;
        Head_State = state.m_CurState;
        Head_State_cmd = state.m_CmdState;
        ErrCode_Head = state.m_ERRCode;
        HEAD_OUT RT_out = pRobotRT->m_HEAD.m_HEAD_OUT;
        
        joint_positions.clear();
        Joint_pos_head[1] = RT_out.m_HEAD_FBK_Joint_Pos[0] * D2R ;
        Joint_pos_head[0] = RT_out.m_HEAD_FBK_Joint_Pos[1] * D2R ;
        Joint_pos_head[2] = RT_out.m_HEAD_FBK_Joint_Pos[2] * D2R ;
        for (int i = 0; i < 3; ++i) {     
            joint_positions.push_back(Joint_pos_head[i]);
        }
        
        return true;
    }

    bool MarvinRobot::ReadState_body(std::vector<float>& joint_positions, std::vector<float>& joint_velocities, std::vector<float>& joint_efforts){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }
        auto state = pRobotRT->m_BODY.m_BODY_State;
        Body_State = state.m_CurState;
        Body_State_cmd = state.m_CmdState;
        ErrCode_Body = state.m_ERRCode;
        BODY_OUT rt_body =  pRobotRT->m_BODY.m_BODY_OUT;
        joint_positions.clear();
        joint_velocities.clear();
        joint_efforts.clear();
        
        for (int i = 0; i < 6; ++i) {
            Joint_pos_body[i] = rt_body.m_BODY_FBK_Joint_Pos[i] * D2R ;
            joint_positions.push_back(Joint_pos_body[i]);
            Joint_vel_body[i] = rt_body.m_BODY_FBK_Joint_Vel[i] * D2R;
            joint_velocities.push_back(Joint_vel_body[i]);
            Joint_eff_body[i] = rt_body.m_BODY_FBK_Joint_SensorTor[i];
            joint_efforts.push_back(Joint_eff_body[i]);
        }

       

        return true;
    }

    bool MarvinRobot::ReadState_lift(std::vector<float>& joint_positions, std::vector<float>& joint_velocities, std::vector<float>& joint_efforts){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }
        auto state = pRobotRT->m_LIFT.m_LIFT_State;
        Lift_State = state.m_CurState;
        Lift_State_cmd = state.m_CmdState;
        ErrCode_Lift = state.m_ERRCode;
        LIFT_OUT rt_lift = pRobotRT->m_LIFT.m_LIFT_OUT;

        joint_positions.clear();
        joint_velocities.clear();
        joint_efforts.clear();

        for (int i = 0; i < 2; ++i) {
            joint_positions.push_back(rt_lift.m_LIFT_FBK_Joint_Pos[i] * 0.001);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);
        }

        return true;
    }

    bool MarvinRobot::ReadCmdPos_arm(std::vector<double>& joint_cmd_positions_A, std::vector<double>& joint_cmd_positions_B){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }

        joint_cmd_positions_A.clear();
        joint_cmd_positions_B.clear();

        for (int i = 0; i < 7; ++i) {
            joint_cmd_positions_A.push_back(pRobotRT->m_ARMS[0].m_ARM_IN.m_ARM_CMD_Joint_Pos[i] * D2R);
            joint_cmd_positions_B.push_back(pRobotRT->m_ARMS[1].m_ARM_IN.m_ARM_CMD_Joint_Pos[i] * D2R);
        }

        return true;
    }

    bool MarvinRobot::ReadCmdPos_head(std::vector<double>& joint_cmd_positions){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }
        HEAD_IN rt_head = pRobotRT->m_HEAD.m_HEAD_IN;

        joint_cmd_positions.clear();
        joint_cmd_positions.push_back(rt_head.m_HEAD_CMD_Joint_Pos[1] * D2R);
        joint_cmd_positions.push_back(rt_head.m_HEAD_CMD_Joint_Pos[0] * D2R);
        joint_cmd_positions.push_back(rt_head.m_HEAD_CMD_Joint_Pos[2] * D2R);

        return true;
    }

    bool MarvinRobot::ReadCmdPos_body(std::vector<double>& joint_cmd_positions){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }
        BODY_IN rt_body = pRobotRT->m_BODY.m_BODY_IN;

        joint_cmd_positions.clear();
        for (int i = 0; i < 6; ++i) {
            joint_cmd_positions.push_back(rt_body.m_BODY_CMD_Joint_Pos[i] * D2R);
        }

        return true;
    }

    bool MarvinRobot::ReadCmdPos_lift(std::vector<double>& joint_cmd_positions){
        const ROBOT_RT* pRobotRT = FX_L1_Fbk_GetRT();
        if (pRobotRT == nullptr) {
            return false;
        }
        LIFT_IN rt_lift = pRobotRT->m_LIFT.m_LIFT_IN;

        joint_cmd_positions.clear();
        for (int i = 0; i < 2; ++i) {
            joint_cmd_positions.push_back(rt_lift.m_LIFT_CMD_Joint_Pos[i] * 0.001);
        }

        return true;
    }



        

    bool MarvinRobot::PosCmd(const std::array<double, 7>& joint_cmd_A,
    const std::array<double, 7>& joint_cmd_B,
    const std::array<double, 3>& joint_cmd_head,
    const std::vector<double>& joint_cmd_body,
    const std::vector<double>& joint_cmd_lift){
        double joint_cmd_A_deg[7];
        double joint_cmd_B_deg[7];
        double joint_cmd_head_deg[3];
        double joint_cmd_body_deg[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double joint_cmd_lift_mm[2] = {0.0, 0.0};
        for (int i = 0; i < 7; ++i){
            joint_cmd_A_deg[i] = joint_cmd_A[i] * R2D;
            joint_cmd_B_deg[i] = joint_cmd_B[i] * R2D;
        }
        joint_cmd_head_deg[0] =  joint_cmd_head[1] * R2D;
        joint_cmd_head_deg[1] =  joint_cmd_head[0] * R2D;
        joint_cmd_head_deg[2] = joint_cmd_head[2] * R2D;
        for (size_t i = 0; i < joint_cmd_body.size() && i < 6; ++i){
            joint_cmd_body_deg[i] = joint_cmd_body[i] * R2D;
        }
        for (size_t i = 0; i < joint_cmd_lift.size() && i < 2; ++i){
            joint_cmd_lift_mm[i] = joint_cmd_lift[i] * 1000.0;
        }
        FX_L1_Comm_Clear(0);
        FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, joint_cmd_A_deg);
        FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM1, joint_cmd_B_deg);
        FX_L1_Runtime_SetJointPosCmd(FX_OBJ_BODY, joint_cmd_body_deg);
        FX_L1_Runtime_SetJointPosCmd(FX_OBJ_LIFT, joint_cmd_lift_mm);
        FX_L1_Runtime_SetJointPosCmd(FX_OBJ_HEAD, joint_cmd_head_deg);


        return l1_send();
    }

    bool MarvinRobot::emptyCmd(){
        FX_L1_Comm_Clear(0);
        return l1_send();
    }

    void MarvinRobot::SetModeTargets(int mode, const std::vector<int64_t> &targets)
    {
        if (targets.size() < 4)
        {
            std::cout << "[ERROR] marvin_robot: mode target config must have arm, head, body, lift." << std::endl;
            return;
        }

        std::array<int, 4> parsed_targets = {
            static_cast<int>(targets[0]),
            static_cast<int>(targets[1]),
            static_cast<int>(targets[2]),
            static_cast<int>(targets[3])
        };

        if (mode == 1)
        {
            mode1_targets_ = parsed_targets;
        }
        else if (mode == 3)
        {
            mode3_targets_ = parsed_targets;
        }
    }

    void MarvinRobot::SetActiveComponents(bool arm_active, bool head_active, bool body_active, bool lift_active)
    {
        arm_active_ = arm_active;
        head_active_ = head_active;
        body_active_ = body_active;
        lift_active_ = lift_active;
    }

    bool MarvinRobot::SetMode(int mode){
        double K_body[7] = {K_L[0], K_L[1], K_L[2], K_L[3], K_L[4], K_L[5], 0.0};
        double D_body[7] = {D_L[0], D_L[1], D_L[2], D_L[3], D_L[4], D_L[5], 0.0};
        bool success = true;

        if (mode == 0){
            if (arm_active_) {
                success &= switch_obj_to_target(FX_OBJ_ARM0, 0, 100.0, 100.0, K_A, D_A);
                success &= switch_obj_to_target(FX_OBJ_ARM1, 0, 100.0, 100.0, K_B, D_B);
            }
            if (body_active_) {
                success &= switch_obj_to_target(FX_OBJ_BODY, 0, 100.0, 100.0, K_body, D_body);
            }
            if (lift_active_) {
                success &= switch_obj_to_target(FX_OBJ_LIFT, 0, 100.0, 100.0, K_body, D_body);
            }
            if (head_active_) {
                success &= switch_obj_to_target(FX_OBJ_HEAD, 0, 100.0, 100.0, K_body, D_body);
            }
        }
        else if (mode == 1) {
            if (arm_active_) {
                success &= switch_obj_to_target(FX_OBJ_ARM0, mode1_targets_[0], 10.0, 10.0, K_A, D_A);
                success &= switch_obj_to_target(FX_OBJ_ARM1, mode1_targets_[0], 10.0, 10.0, K_B, D_B);
            }
            usleep(500000);
            if (body_active_) {
                success &= switch_obj_to_target(FX_OBJ_BODY, mode1_targets_[2], 10.0, 10.0, K_body, D_body);
            }
            if (lift_active_) {
                success &= switch_obj_to_target(FX_OBJ_LIFT, mode1_targets_[3], 10.0, 10.0, K_body, D_body);
            }
            if (head_active_) {
                success &= switch_obj_to_target(FX_OBJ_HEAD, mode1_targets_[1], 10.0, 10.0, K_body, D_body);
            }
        }

        else if (mode == 3) {
            if (arm_active_) {
                FX_L1_Comm_Clear(0);
                FX_L1_Runtime_SetJointKD(FX_OBJ_ARM0, K_A, D_A);
                FX_L1_Runtime_SetJointKD(FX_OBJ_ARM1, K_B, D_B);
                success &= l1_send();
            }
            usleep(100000);
            if (arm_active_) {
                success &= switch_obj_to_target(FX_OBJ_ARM0, mode3_targets_[0], 100.0, 100.0, K_A, D_A);
                success &= switch_obj_to_target(FX_OBJ_ARM1, mode3_targets_[0], 100.0, 100.0, K_B, D_B);
            }

            usleep(500000);
            if (body_active_) {
                success &= switch_obj_to_target(FX_OBJ_BODY, mode3_targets_[2], 100.0, 100.0, K_body, D_body);
            }
            if (lift_active_) {
                success &= switch_obj_to_target(FX_OBJ_LIFT, mode3_targets_[3], 100.0, 100.0, K_body, D_body);
            }
            if (head_active_) {
                success &= switch_obj_to_target(FX_OBJ_HEAD, mode3_targets_[1], 100.0, 100.0, K_body, D_body);
            }
        }
        else {
            std::cout << "[ERROR] marvin_robot: Invalid mode." << std::endl;
            return false;
        }
        
        usleep(100000);
        return success;
    }

    void MarvinRobot::servo_off()
    {
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kL1TimeoutMs);
        FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kL1TimeoutMs);
        FX_L1_State_SwitchToIdle(FX_OBJ_BODY, kL1TimeoutMs);
        FX_L1_State_SwitchToIdle(FX_OBJ_HEAD, kL1TimeoutMs);
        usleep(100000);
    }



    void MarvinRobot::GetState(int (&state_out)[6]) const
    {
        state_out[0] = ArmState_A;
        state_out[1] = ArmState_B;
        state_out[2] = Body_State;
        state_out[3] = Head_State;
        state_out[4] = Lift_State;
        state_out[5] = 0;
    }
    void MarvinRobot::GetState(int (&state_cur)[6],int (&state_cmd)[6],int (&state_err)[6]) const
    {
        state_cur[0] = ArmState_A;
        state_cur[1] = ArmState_B;
        state_cur[2] = Head_State;
        state_cur[3] = Body_State;
        state_cur[4] = Lift_State;
        state_cur[5] = 0;
        state_cmd[0] = ArmState_A_cmd;
        state_cmd[1] = ArmState_B_cmd;
        state_cmd[2] = Head_State_cmd;
        state_cmd[3] = Body_State_cmd;
        state_cmd[4] = Lift_State_cmd;
        state_cmd[5] = 0;
        state_err[0] = ErrCode_A;
        state_err[1] = ErrCode_B;
        state_err[2] = ErrCode_Head;
        state_err[3] = ErrCode_Body;
        state_err[4] = ErrCode_Lift;
    }
    

    
    void MarvinRobot::ClearFault()
    {
        unsigned int system_errorcode = 0;
        FX_L1_State_ResetError(FX_OBJ_ARM0, kL1TimeoutMs, &system_errorcode);
        usleep(100000);
        FX_L1_State_ResetError(FX_OBJ_ARM1, kL1TimeoutMs, &system_errorcode);
        usleep(100000);
        FX_L1_State_ResetError(FX_OBJ_BODY, kL1TimeoutMs, &system_errorcode);
        usleep(100000);
        FX_L1_State_ResetError(FX_OBJ_HEAD, kL1TimeoutMs, &system_errorcode);
        usleep(100000);
        

    }
    void MarvinRobot::GetJointHome_A(double *joint_home)
    {
        for (int i = 0; i < 7; ++i)
        {
            joint_home[i] = joint_home_A_[i];
        }
    }
    void MarvinRobot::GetJointHome_B(double *joint_home)
    {
        for (int i = 0; i < 7; ++i)
        {
            joint_home[i] = joint_home_B_[i];
        }
    }
    void MarvinRobot::SetKDA(std::vector<double> &k, std::vector<double> &d)
    {
        if (k.size() == 7 && d.size() == 7)
        {
            for (size_t i = 0; i < 7; ++i)
            {
                K_A[i] = k[i];
                D_A[i] = d[i];
            }
        }
        else
        {
            std::cout << "[ERROR] marvin_robot: K and D must have exactly 7 elements each." << std::endl;
        }
    }
    void MarvinRobot::SetKDB(std::vector<double> &k, std::vector<double> &d)
    {
        if (k.size() == 7 && d.size() == 7)
        {
            for (size_t i = 0; i < 7; ++i)
            {
                K_B[i] = k[i];
                D_B[i] = d[i];
            }
        }
        else
        {
            std::cout << "[ERROR] marvin_robot: K and D must have exactly 7 elements each." << std::endl;
        }
    }
    void MarvinRobot::SetKDL(std::vector<double> &k, std::vector<double> &d)
    {
        if (k.size() == 6 && d.size() == 6)
        {
            for (size_t i = 0; i < 6; ++i)
            {
                K_L[i] = k[i];
                D_L[i] = d[i];
            }
            FX_L1_Comm_Clear(0);
            FX_L1_Runtime_SetBodyPD(K_L, D_L);
            FX_L1_Comm_Send();
            usleep(5000);
        }
        else
        {
            std::cout << "[ERROR] marvin_robot: K and D must have exactly 7 elements each." << std::endl;
        }
    }

    void MarvinRobot::set_speed_arm(int vel_scale, int acc_scale){
        FX_L1_Comm_Clear(0);
        FX_L1_Runtime_SetSpeedRatio(FX_OBJ_ARM0, static_cast<double>(vel_scale), static_cast<double>(acc_scale));
        FX_L1_Runtime_SetSpeedRatio(FX_OBJ_ARM1, static_cast<double>(vel_scale), static_cast<double>(acc_scale));
        FX_L1_Comm_Send();
        usleep(5000);

     
    }
        void MarvinRobot::set_speed_body(int vel_scale, int acc_scale){
        FX_L1_Comm_Clear(0);
        FX_L1_Runtime_SetSpeedRatio(FX_OBJ_BODY, static_cast<double>(vel_scale), static_cast<double>(acc_scale));
        FX_L1_Runtime_SetSpeedRatio(FX_OBJ_LIFT, static_cast<double>(vel_scale), static_cast<double>(acc_scale));
        FX_L1_Comm_Send();
        usleep(5000);

     
    }
        void MarvinRobot::set_speed_head(int vel_scale, int acc_scale){
        FX_L1_Comm_Clear(0);
        FX_L1_Runtime_SetSpeedRatio(FX_OBJ_HEAD, static_cast<double>(vel_scale), static_cast<double>(acc_scale));
        FX_L1_Comm_Send();
        usleep(5000);
    }
 
    std::vector<int> MarvinRobot::parse_ip(const std::string &ip)
    {
        std::vector<int> ip_parts;
        std::stringstream ss(ip);
        std::string segment;
        while (std::getline(ss, segment, '.'))
        {
            ip_parts.push_back(std::stoi(segment));
        }
        return ip_parts;
    }






    MarvinRobot::~MarvinRobot()
    {
        stop_ = true;
        if (writer_thread_.joinable()) writer_thread_.join();
        if (reader_thread_.joinable()) reader_thread_.join();
        if (sock0 >= 0) close(sock0);
        if (sock1 >= 0) close(sock1);
    }

    void MarvinRobot::init_can(const std::string &iface0, const std::string &iface1)
    {
        sock0 = open_can_socket(iface0);
        sock1 = open_can_socket(iface1);
        stop_ = false;
        writer_thread_ = std::thread(&MarvinRobot::forward_to_sdk, this);
        reader_thread_ = std::thread(&MarvinRobot::forward_to_sockets, this);
    }

    int MarvinRobot::open_can_socket(const std::string &ifname)
    {
        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CAN"), "Failed to create CAN socket for interface %s", ifname.c_str());
            return -1;
        }

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CAN"), "No such interface: %s", ifname.c_str());
            close(sock);
            return -1;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CAN"), "Failed to bind to CAN interface: %s", ifname.c_str());
            close(sock);
            return -1;
        }

        RCLCPP_INFO(rclcpp::get_logger("CAN"), "Successfully opened CAN socket on interface %s", ifname.c_str());
        return sock;
    }

    void MarvinRobot::forward_to_sdk()
    {
        struct pollfd fds[2];
        fds[0].fd = sock0;
        fds[0].events = POLLIN;
        fds[1].fd = sock1;
        fds[1].events = POLLIN;

        while (!stop_)
        {
            int ret = poll(fds, 2, 1);
            if (ret < 0)
            {
                perror("poll");
                break;
            }
            else if (ret == 0)
            {
                continue;
            }

            if (fds[0].revents & POLLIN)
            {
                struct can_frame frameA{};
                int nbytesA = read(sock0, &frameA, sizeof(frameA));
                if (nbytesA == sizeof(frameA))
                {
                    if (verbose_logging)
                    {
                        std::cout << "[vcan0] CAN frame received: ID=0x"
                                << std::hex << frameA.can_id
                                << " DLC=" << std::dec << (int)frameA.can_dlc
                                << " Data=[";
                        for (int i = 0; i < frameA.can_dlc; i++)
                        {
                            std::cout << std::hex << std::uppercase << (int)frameA.data[i];
                            if (i < frameA.can_dlc - 1) std::cout << " ";
                        }
                        std::cout << "]" << std::dec << std::endl;
                    }
                    sdk_write_canA(frameA);
                }
            }

            if (fds[1].revents & POLLIN)
            {
                struct can_frame frameB{};
                int nbytesB = read(sock1, &frameB, sizeof(frameB));
                if (nbytesB == sizeof(frameB))
                {
                    if (verbose_logging)
                    {
                        std::cout << "[vcan1] CAN frame received: ID=0x"
                                << std::hex << frameB.can_id
                                << " DLC=" << std::dec << (int)frameB.can_dlc
                                << " Data=[";
                        for (int i = 0; i < frameB.can_dlc; i++)
                        {
                            std::cout << std::hex << std::uppercase << (int)frameB.data[i];
                            if (i < frameB.can_dlc - 1) std::cout << " ";
                        }
                        std::cout << "]" << std::dec << std::endl;
                    }
                    sdk_write_canB(frameB);
                }
            }
        }
    }

    void MarvinRobot::forward_to_sockets()
    {
        while (!stop_)
        {
            struct can_frame frameA{}, frameB{};
            if (sdk_read_canA(frameA))
            {
                if (sock0 >= 0)
                {
                    int nbytes = write(sock0, &frameA, sizeof(frameA));
                    if (nbytes != sizeof(frameA))
                        std::cerr << "[vcan0] Failed to write CAN frame" << std::endl;
                }
            }
            else if (sdk_read_canB(frameB))
            {
                if (sock1 >= 0)
                {
                    int nbytes = write(sock1, &frameB, sizeof(frameB));
                    if (nbytes != sizeof(frameB))
                        std::cerr << "[vcan1] Failed to write CAN frame" << std::endl;
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    bool MarvinRobot::sdk_read_canA(struct can_frame &frame)
    {
        unsigned char data[64];
        memset(data, 0, sizeof(data));
        FXChnType ret_ch = kTerminalChannel;
        unsigned int receiving_time = 0;

        int len = FX_L1_Terminal_GetData(FX_TERMINAL_ARM0, 0, &ret_ch, data, &receiving_time);
        if (len <= 4)
            return false;

        frame.can_id = (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        frame.can_dlc = static_cast<__u8>(len - 4);
        if (frame.can_dlc > 8) frame.can_dlc = 8;
        memcpy(frame.data, &data[4], frame.can_dlc);
        return true;
    }

    bool MarvinRobot::sdk_read_canB(struct can_frame &frame)
    {
        unsigned char data[64];
        memset(data, 0, sizeof(data));
        FXChnType ret_ch = kTerminalChannel;
        unsigned int receiving_time = 0;

        int len = FX_L1_Terminal_GetData(FX_TERMINAL_ARM1, 0, &ret_ch, data, &receiving_time);
        if (len <= 4)
            return false;

        frame.can_id = (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        frame.can_dlc = static_cast<__u8>(len - 4);
        if (frame.can_dlc > 8) frame.can_dlc = 8;
        memcpy(frame.data, &data[4], frame.can_dlc);
        return true;
    }

    bool MarvinRobot::sdk_write_canA(const struct can_frame &frame)
    {
        unsigned char data[64];
        memset(data, 0, sizeof(data));
        data[0] = frame.can_id & 0xFF;
        data[1] = (frame.can_id >> 8) & 0xFF;
        data[2] = (frame.can_id >> 16) & 0xFF;
        data[3] = (frame.can_id >> 24) & 0xFF;
        memcpy(&data[4], frame.data, frame.can_dlc);
        unsigned int size = 4 + frame.can_dlc;
        unsigned int sending_time = 0;

        return l1_ok(FX_L1_Terminal_SetData(FX_TERMINAL_ARM0, kTerminalChannel, 0, data, size, &sending_time));
    }

    bool MarvinRobot::sdk_write_canB(const struct can_frame &frame)
    {
        unsigned char data[64];
        memset(data, 0, sizeof(data));
        data[0] = frame.can_id & 0xFF;
        data[1] = (frame.can_id >> 8) & 0xFF;
        data[2] = (frame.can_id >> 16) & 0xFF;
        data[3] = (frame.can_id >> 24) & 0xFF;
        memcpy(&data[4], frame.data, frame.can_dlc);
        unsigned int size = 4 + frame.can_dlc;
        unsigned int sending_time = 0;
        return l1_ok(FX_L1_Terminal_SetData(FX_TERMINAL_ARM1, kTerminalChannel, 0, data, size, &sending_time));
    }
