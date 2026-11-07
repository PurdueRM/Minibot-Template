#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "math.h"

#include "motor.h"
#include "dji_motor.h"



extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

float theta = 0;
float angularV;
float tx; // target vector
float ty;

float lx; // localized vector
float ly;

const float r; // omni-wheel radius
const float wr; // wheel radius
const float scaleFactor = 660.0f;

float w1;
float w2;
float w3;
float w4;

DJI_Motor_Handle_t* motor_w1;
DJI_Motor_Handle_t* motor_w2;
DJI_Motor_Handle_t* motor_w3;
DJI_Motor_Handle_t* motor_w4;

void Chassis_Task_Init()
{
    angularV = g_robot_state.chassis.omega;
    theta = PI / 4;

    


    Motor_Config_t chassis_w1 = {
        .can_bus = 1, 
        .speed_controller_id = 1, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w1 = DJI_Motor_Init(&chassis_w1, M2006);
    Motor_Config_t chassis_w2 = {
        .can_bus = 1, 
        .speed_controller_id = 2, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w2 = DJI_Motor_Init(&chassis_w2, M2006);
    Motor_Config_t chassis_w3 = {
        .can_bus = 1, 
        .speed_controller_id = 3, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w3 = DJI_Motor_Init(&chassis_w3, M2006);
    Motor_Config_t chassis_w4 = {
        .can_bus = 1, 
        .speed_controller_id = 4, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            {
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w4 = DJI_Motor_Init(&chassis_w4, M2006);

    // Init chassis hardware
}

void Chassis_Ctrl_Loop()
{
    

    tx = g_robot_state.input.vx;
    ty = g_robot_state.input.vy;

    lx =  tx*cos(theta) - ty*sin(theta);
    ly =  tx*sin(theta) + ty*cos(theta);

    w1 = (lx*scaleFactor);
    w3 = (-lx*scaleFactor);
    w2 = (ly*scaleFactor);
    w4 = (-ly*scaleFactor);

    DJI_Motor_Set_Velocity(motor_w1, w1);
    DJI_Motor_Set_Velocity(motor_w2, w2);
    DJI_Motor_Set_Velocity(motor_w3, w3);
    DJI_Motor_Set_Velocity(motor_w4, w4);


    
    // Control loop for the chassis
}