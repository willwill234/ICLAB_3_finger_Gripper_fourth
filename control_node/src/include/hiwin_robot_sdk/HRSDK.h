﻿#include <cstdint>	// std::uint8_t
#include <vector>

#ifndef HRSDK_HRSDK_H_
#define HRSDK_HRSDK_H_

#ifdef HRSDK_HRSDK_H_
#define HRSDK_API __declspec(dllexport)
#else
#define HRSDK_API __declspec(dllimport)
#endif

typedef int HROBOT;
#ifdef __cplusplus
extern "C" {
#endif

enum ConnectionLevels {
	kDisconnection = -1,
	kMonitor = 0,
	kController
};

enum OperationModes {
	kManual = 0,
	kAuto
};

enum LogLevels {
	INFOS = 0,
	WARNINGS = 1,
	ERRORS = 2,
	EXPORT_SDK_INITIAL_LOG = 3,
	EXPORT_SDK_LOG = 4,
	NONES = 5
};

enum SpaceOperationTypes {
	kCartesian = 0,
	kJoint,
	kTool
};

enum SpaceOperationDirection {
	kPositive = 1,
	kNegative = -1,
};

enum JointCoordinates {
	kJoint1 = 0,
	kJoint2,
	kJoint3,
	kJoint4,
	kJoint5,
	kJoint6
};

enum CartesianCoordinates {
	kCartesianX = 0,
	kCartesianY,
	kCartesianZ,
	kCartesianA,
	kCartesianB,
	kCartesianC
};

enum RobotMotionStatus {
	kIdle = 1,
	kRunning,
	kHold,
	kDelay,
	kWait
};

/* Connection Command */
typedef void(__stdcall *callback_function)(uint16_t, uint16_t, uint16_t*, int);
HRSDK_API HROBOT __stdcall open_connection(const char* address, int level, callback_function f);
HRSDK_API void __stdcall close_connection(HROBOT robot);
HRSDK_API int __stdcall set_connection_level(HROBOT robot, int Mode);
HRSDK_API int __stdcall get_connection_level(HROBOT robot);
HRSDK_API int __stdcall get_hrsdk_version(char* version);

/* Register Command */
HRSDK_API int __stdcall set_timer(HROBOT robot, int index, int time);
HRSDK_API int __stdcall get_timer(HROBOT robot, int index);
HRSDK_API int __stdcall set_timer_start(HROBOT robot, int index);
HRSDK_API int __stdcall set_timer_stop(HROBOT robot, int index);
HRSDK_API int __stdcall set_counter(HROBOT robot, int index, int co);
HRSDK_API int __stdcall get_counter(HROBOT robot, int index);
HRSDK_API int __stdcall set_pr_type(HROBOT robot, int prNum, int coorType);
HRSDK_API int __stdcall get_pr_type(HROBOT robot, int prNum);
HRSDK_API int __stdcall set_pr_coordinate(HROBOT robot, int prNum, double *coor);
HRSDK_API int __stdcall get_pr_coordinate(HROBOT robot, int pr, double* coor);
HRSDK_API int __stdcall set_pr_tool_base(HROBOT robot, int prNum, int toolNum, int baseNum);
HRSDK_API int __stdcall get_pr_tool_base(HROBOT robot, int pr, int* tool_base);
HRSDK_API int __stdcall set_pr(HROBOT robot, int prNum, int coorType, double *coor, int tool, int base);
HRSDK_API int __stdcall get_pr(HROBOT robot, int pr_num, int* coor_type, double *coor, int* tool, int* base);
HRSDK_API int __stdcall remove_pr(HROBOT robot, int pr_num);

/* System Variable Command */
HRSDK_API int __stdcall set_acc_dec_ratio(HROBOT robot, int acc);
HRSDK_API int __stdcall get_acc_dec_ratio(HROBOT robot);
HRSDK_API int __stdcall set_acc_time(HROBOT robot, double value);
HRSDK_API double __stdcall get_acc_time(HROBOT robot);
HRSDK_API int __stdcall set_ptp_speed(HROBOT robot, int vel);
HRSDK_API int __stdcall get_ptp_speed(HROBOT robot);
HRSDK_API int __stdcall set_lin_speed(HROBOT robot, double vel);
HRSDK_API double __stdcall get_lin_speed(HROBOT robot);
HRSDK_API int __stdcall set_override_ratio(HROBOT robot, int vel);
HRSDK_API int __stdcall get_override_ratio(HROBOT robot);
HRSDK_API int __stdcall get_robot_id(HROBOT robot, char* robot_id);
HRSDK_API int __stdcall set_robot_id(HROBOT robot, char* robot_id);
HRSDK_API int __stdcall set_smooth_length(HROBOT robot, double r);
HRSDK_API int __stdcall get_alarm_code(HROBOT robot, int &count, uint64_t* alarm_code);

/* Input and Output Command */
HRSDK_API int __stdcall get_digital_input(HROBOT robot, int index);
HRSDK_API int __stdcall get_digital_output(HROBOT robot, int index);
HRSDK_API int __stdcall set_digital_output(HROBOT robot, int index, bool v);
HRSDK_API int __stdcall get_robot_input(HROBOT robot, int index);
HRSDK_API int __stdcall get_robot_output(HROBOT robot, int index);
HRSDK_API int __stdcall set_robot_output(HROBOT robot, int index, bool v);
HRSDK_API int __stdcall get_valve_output(HROBOT robot, int index);
HRSDK_API int __stdcall set_valve_output(HROBOT robot, int index, bool v);
HRSDK_API int __stdcall get_function_input(HROBOT robot, int index);
HRSDK_API int __stdcall get_function_output(HROBOT robot, int index);

/* Coordinate System Command */
HRSDK_API int __stdcall set_base_number(HROBOT robot, int state);
HRSDK_API int __stdcall get_base_number(HROBOT robot);
HRSDK_API int __stdcall define_base(HROBOT robot, int baseNum, double *coor);
HRSDK_API int __stdcall get_base_data(HROBOT robot, int num, double* coor);
HRSDK_API int __stdcall set_tool_number(HROBOT robot, int num);
HRSDK_API int __stdcall get_tool_number(HROBOT robot);
HRSDK_API int __stdcall define_tool(HROBOT robot, int toolNum, double *coor);
HRSDK_API int __stdcall get_tool_data(HROBOT robot, int num, double* coor);
HRSDK_API int __stdcall enable_joint_soft_limit(HROBOT v, bool enable);
HRSDK_API int __stdcall enable_cart_soft_limit(HROBOT robot, bool enable);
HRSDK_API int __stdcall set_joint_soft_limit(HROBOT robot, double* low_limit, double* high_limit);
HRSDK_API int __stdcall set_cart_soft_limit(HROBOT robot, double* low_limit, double* high_limit);
HRSDK_API int __stdcall get_joint_soft_limit_config(HROBOT robot, bool& enable, double* low_limit, double* high_limit);
HRSDK_API int __stdcall get_cart_soft_limit_config(HROBOT robot, bool& enable, double* low_limit, double* high_limit);

/* Task Command */
HRSDK_API int __stdcall set_rsr(HROBOT robot, char* filename, int index);
HRSDK_API int __stdcall get_rsr_prog_name(HROBOT robot, int rsr_index, char* file_name);
HRSDK_API int __stdcall remove_rsr(HROBOT robot, int index);
HRSDK_API int __stdcall ext_task_start(HROBOT robot, int mode, int select);
HRSDK_API int __stdcall task_start(HROBOT robot, char* task_name);
HRSDK_API int __stdcall task_hold(HROBOT robot);
HRSDK_API int __stdcall task_continue(HROBOT robot);
HRSDK_API int __stdcall task_abort(HROBOT robot);
HRSDK_API int __stdcall get_execute_file_name(HROBOT robot, char* file_name);

/* File Command */
HRSDK_API int __stdcall send_file(HROBOT sock, char* root_folder, char* from_file_path, char* to_file_path, int opt);
HRSDK_API int __stdcall download_file(HROBOT robot, char* from_file_path, char* to_file_path);

/* Controller Setting Command */
HRSDK_API int __stdcall get_hrss_mode(HROBOT robot);
HRSDK_API int __stdcall set_motor_state(HROBOT robot, int state);
HRSDK_API int __stdcall get_motor_state(HROBOT robot);
HRSDK_API int __stdcall set_operation_mode(HROBOT robot, int mode);
HRSDK_API int __stdcall get_operation_mode(HROBOT robot);
HRSDK_API int __stdcall clear_alarm(HROBOT robot);
HRSDK_API int __stdcall update_hrss(HROBOT robot, char* path);

/* Jog */
HRSDK_API int __stdcall jog(HROBOT robot, int space_type, int index, int dir);
HRSDK_API int __stdcall jog_home(HROBOT robot);
HRSDK_API int __stdcall jog_stop(HROBOT robot);

/* Motion Command */
HRSDK_API int __stdcall ptp_pos(HROBOT robot, int mode, double * p);
HRSDK_API int __stdcall ptp_axis(HROBOT robot, int mode, double * p);
HRSDK_API int __stdcall ptp_rel_pos(HROBOT robot, int mode, double * p);
HRSDK_API int __stdcall ptp_rel_axis(HROBOT robot, int mode, double * p);
HRSDK_API int __stdcall ptp_pr(HROBOT robot, int mode, int p);
HRSDK_API int __stdcall lin_pos(HROBOT robot, int mode, double smooth_value, double * p);
HRSDK_API int __stdcall lin_axis(HROBOT robot, int mode, double smooth_value, double * p);
HRSDK_API int __stdcall lin_rel_pos(HROBOT robot, int mode, double smooth_value, double * p);
HRSDK_API int __stdcall lin_rel_axis(HROBOT robot, int mode, double smooth_value, double * p);
HRSDK_API int __stdcall lin_pr(HROBOT robot, int mode, double smooth_value, int p);
HRSDK_API int __stdcall circ_pos(HROBOT robot, int mode, double * p_aux, double * p_end);
HRSDK_API int __stdcall circ_axis(HROBOT robot, int mode, double * p_aux, double * p_end);
HRSDK_API int __stdcall circ_pr(HROBOT robot, int mode, int p1, int p2);
HRSDK_API int __stdcall motion_hold(HROBOT robot);
HRSDK_API int __stdcall motion_continue(HROBOT robot);
HRSDK_API int __stdcall motion_abort(HROBOT robot);
HRSDK_API int __stdcall motion_delay(HROBOT robot, int delay);
HRSDK_API int __stdcall set_command_id(HROBOT robot, int id);
HRSDK_API int __stdcall get_command_id(HROBOT robot);
HRSDK_API int __stdcall get_command_count(HROBOT robot);
HRSDK_API int __stdcall get_motion_state(HROBOT robot);
HRSDK_API int __stdcall remove_command(HROBOT robot, int num);
HRSDK_API int __stdcall remove_command_tail(HROBOT robot, int num);

/* Manipulator Information Command */
HRSDK_API int __stdcall get_encoder_count(HROBOT robot, int32_t* EncCount);
HRSDK_API int __stdcall get_current_joint(HROBOT robot, double * joint);
HRSDK_API int __stdcall get_current_position(HROBOT robot, double * cart);
HRSDK_API int __stdcall get_current_rpm(HROBOT robot, double * rpm);
HRSDK_API int __stdcall get_device_born_date(HROBOT robot, int* YMD);
HRSDK_API int __stdcall get_operation_time(HROBOT robot, int* YMDHm);
HRSDK_API int __stdcall get_mileage(HROBOT robot, double * mil);
HRSDK_API int __stdcall get_total_mileage(HROBOT robot, double * tomil);
HRSDK_API int __stdcall get_utilization(HROBOT robot, int* utl);
HRSDK_API int __stdcall get_utilization_ratio(HROBOT robot);
HRSDK_API int __stdcall get_motor_torque(HROBOT robot, double * cur);
HRSDK_API int __stdcall get_robot_type(HROBOT robot, char* robType);
HRSDK_API int __stdcall get_hrss_version(HROBOT robot, char* ver);

extern uint16_t ts;

#ifdef __cplusplus
}
#endif
#endif // HRSDK_HRSDK_H_
