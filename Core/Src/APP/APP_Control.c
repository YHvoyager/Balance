#include "APP_Control.h"
#include "APP_PID.h"
#include "task.h"
#include "APP_MPU6050.h"
#include "qmath.h"
#include "APP_Encoder.h"
#include "APP_Motor_Control.h"
#include <math.h>

static PID_Typedef pid_velocity; //速度环的PID控制器
static PID_Typedef pid_theta; //Θ环的PID控制器
static PID_Typedef pid_theta_dot; //Θ'环的PID控制器
static PID_Typedef pid_turn; //转向环：偏航角速度 gz → 左右轮速差 ω_diff

static APP_ControlTelemetry control_telemetry;
static float omega_ref = 0.0f; //平衡车轮胎转速,单位：rad/s
static uint64_t last_time = 0; //上一次执行时间,单位：微秒

static const float lp = 0.070f; //平衡车轮胎中心到MPU6050中心的距离,单位：m
static const float g = 9.8f; //重力加速度,单位：m/s^2
static const float rw = 0.024f; //平衡车轮胎半径,单位：m

static uint8_t ctrl_prev_en = 0;    

/* 遥控指令（move 指令解析后存在这里，范围 -100 ~ +100）
 * ★ 当前只存不用：控制环还没读它们，目的是先把「解析」这条链路验证通。 */
static int8_t cmd_turn_pct = 0;
static int8_t cmd_move_pct = 0;

void APP_Control_SetCmd(int turn, int move)
{
    if (turn >  100) turn =  100;
    if (turn < -100) turn = -100;
    if (move >  100) move =  100;
    if (move < -100) move = -100;

    cmd_turn_pct = (int8_t)turn;
    cmd_move_pct = (int8_t)move;
}

void APP_Control_GetCmd(int *turn, int *move)
{
    if (turn != NULL) *turn = cmd_turn_pct;
    if (move != NULL) *move = cmd_move_pct;
}

/* 速度环的目标线速度 (m/s)。默认 0 = 原地平衡，行为和以前完全一致。
 * ★ 必须存成变量、由 Proc 每帧读，不能在 Proc 之外直接改 PID 的 SP ——
 *   Proc 里每帧都会 PID_ChangeSp()，外面设的值活不过一帧。 */
static float target_move_mps = 0.0f;

void APP_Control_SetMoveSpeed(float MoveSpeed)
{
    target_move_mps = MoveSpeed;
}

/* 转向环的目标偏航角速度 (rad/s)，正数对应 ω_diff>0 → 左轮快右轮慢。
 * 同样只存不用，由 Proc 每帧读取。 */
static float target_turn_rate = 0.0f;

void APP_Control_SetTurnRate(float rate)
{
    target_turn_rate = rate;
}

/* 转向环的两个中间量，只给调试/上位机看 */
static float last_gz_rad     = 0.0f;   // 实测偏航角速度（rad/s）
static float last_omega_diff = 0.0f;   // 转向环输出 = 左右轮速差

void APP_Control_GetTurnDebug(float *gz_rad, float *omega_diff)
{
    if (gz_rad != NULL)     *gz_rad     = last_gz_rad;
    if (omega_diff != NULL) *omega_diff = last_omega_diff;
}

//
// @简介：初始化平衡车控制系统
// 
void APP_Control_Init(void){
    PID_Init(&pid_velocity, 2.0f, 0.0f, 0.0f);


    PID_Init(&pid_theta, 9.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_theta, 12.57f, -12.57f); // Θ环的PID控制器输出范围为[-12.57, 12.57]

    PID_Init(&pid_theta_dot, 32.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_theta_dot, 60.00f, -60.00f); // Θ'环的PID控制器输出范围为[-60, 60]

    /* 转向环：输入偏航角速度误差(rad/s)，输出左右轮速差(rad/s)。
       Kp=1.0 是结构图给的初值，限幅 ±15 rad/s 防止原地甩飞。 */
    PID_Init(&pid_turn, 3.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_turn, 15.0f, -15.0f);

    last_time = micros();
}   

void APP_Control_SetAnglePID(float kp, float ki, float kd){
    pid_theta.kp = kp;
    pid_theta.ki = ki;
    pid_theta.kd = kd;
    PID_Reset(&pid_theta);
}

void APP_Control_SetRatePID(float kp, float ki, float kd){
    pid_theta_dot.kp = kp;
    pid_theta_dot.ki = ki;
    pid_theta_dot.kd = kd;
    PID_Reset(&pid_theta_dot);
}

void APP_Control_GetTelemetry(APP_ControlTelemetry *telemetry){
    if (telemetry != 0) {
        *telemetry = control_telemetry;
    }
}

//
// @简介：平衡车控制系统进程函数，用于运算，需要在while中调用
//
void APP_Control_Proc(void){
    PERIODIC(5);  //控制程序5ms执行一次

    // ================= 传感器快照：与电机是否使能无关，永远更新 =================
    // 这一段必须在使能判断【之前】。否则电机没开时 telemetry 保持上次（或全 0）的值，
    // VOFA 上看到的是一条贴着 0 的死线，看起来像"没有回传 MPU 数据"。
    float theta_deg = APP_MPU6050_GetPitch();
    float theta = theta_deg * 0.0174533f;                                //单位：弧度
    float theta_dot_dps = APP_MPU6050_GetGy() - APP_MPU6050_GetGyBias(); //单位：°/s，已去零偏
    float theta_dot = theta_dot_dps * 0.0174533f;                        //单位：弧度/秒

    control_telemetry.pitch_deg     = theta_deg;
    control_telemetry.gyro_dps      = theta_dot_dps;
    control_telemetry.pitch_acc_deg = APP_MPU6050_GetPitchAcc();
    control_telemetry.gyro_bias_dps = APP_MPU6050_GetGyBias();

    uint8_t en = APP_Motor_IsEnabled();
    control_telemetry.motor_en = en;

    if (!en) {                       // 关机：清零，绝不累积
        omega_ref = 0.0f;
        last_time = micros();
        PID_Reset(&pid_velocity);
        PID_Reset(&pid_theta);
        PID_Reset(&pid_theta_dot);
        APP_Motor_SetOmega_L(0.0f);
        APP_Motor_SetOmega_R(0.0f);

        // 未使能时，这几个量在物理上就是 0，别让上位机看到上次跑车留下的残值
        control_telemetry.theta_dot_ref      = 0.0f;
        control_telemetry.theta_dot_dot_ref  = 0.0f;
        control_telemetry.omega_ref          = 0.0f;

        ctrl_prev_en = 0;
        return;
    }
    if (!ctrl_prev_en) {             // 使能上升沿：再复位一次，这一拍只复位不输出
        omega_ref = 0.0f;
        last_time = micros();
        PID_Reset(&pid_velocity);
        PID_Reset(&pid_theta);
        PID_Reset(&pid_theta_dot);
        control_telemetry.theta_dot_ref      = 0.0f;
        control_telemetry.theta_dot_dot_ref  = 0.0f;
        control_telemetry.omega_ref          = 0.0f;
        ctrl_prev_en = 1;
        return;
    }

    uint64_t now = micros();
    float deltaT = (now - last_time) * 1e-6f; //单位：秒

        //  # -1.改变速度环设定值（由 APP_Control_SetMoveSpeed 写入，默认 0）
    PID_ChangeSp(&pid_velocity, target_move_mps);

    // # -2.读取传感器的值
    float omega = 0.5 * (EncoderGetSpeed_L() + EncoderGetSpeed_R()); //单位：rad/s

    // #2.读取传感器数据（theta、theta_dot 已在函数开头获取）
    float temp = theta * 57.29578f; //转换为角度
    if (qcos(temp) < 0.5f ) {                // |θ| > 72°，已经倒了
    omega_ref = 0.0f;
    APP_Motor_Cmd(0);                               // 直接断电，别挣扎
    return;
    }

    // # -3.计算速度环的反馈值
    float omega2 = -theta_dot * (lp + rw); // //单位：rad/s
    float omega1 = omega - omega2; //单位：rad/s
    float x_dot = omega1 * rw; //单位：m/s

    // # -4.执行速度环PID运算
    // ★ qtan() 的入参单位是【度】，而 PID 输出 / g 得到的是【弧度】，
    //   必须乘 180/π 换算。原来直接传弧度，等于把目标倾角缩小了 57.3 倍，
    //   速度环几乎不起作用：车能站住，但推一下就一直漂、收到 move 也不动。
    float theta_ref = qtan(PID_Compute(&pid_velocity, x_dot) / g * 57.29578f);

    // #1.将外环的机械平衡目标设为-4.0°，PID内部使用弧度
    PID_ChangeSp(&pid_theta, theta_ref-4.0f * 0.0174533f);

    // #2.倒下检测（theta 已经是弧度；原来这里把角度值喂给 qcos() 是单位错误）
    if (fabsf(theta_deg) > 72.0f) {          // |θ| > 72°，已经倒了
        omega_ref = 0.0f;
        APP_Motor_Cmd(0);                    // 直接断电，别挣扎
        control_telemetry.theta_dot_ref     = 0.0f;
        control_telemetry.theta_dot_dot_ref = 0.0f;
        control_telemetry.omega_ref         = 0.0f;
        control_telemetry.motor_en          = 0;
        return;
    }

    // #3.计算外环PID输出
    float theta_dot_ref = PID_Compute(&pid_theta, theta);

    // #4.改变内环的设定值SP
    PID_ChangeSp(&pid_theta_dot, theta_dot_ref);

    // #5.计算内环PID输出
    float theta_dot_dot_ref = PID_Compute(&pid_theta_dot, theta_dot);

    // #6.倒立摆的逆解算
    float x_dot_dot_ref = (g * qsin(theta) - theta_dot_dot_ref * lp) / qcos(theta); //单位：m/s^2

    // #7.计算轮胎转速
    if(last_time != 0)
    {
    omega_ref += 1.0f / rw * x_dot_dot_ref * deltaT; //单位：rad/s
    }

    // #8.转向环：gz_ref - gz → PID → 左右轮速差 ω_diff
    //    gz_ref 来自 move 指令的 turnSpeed，gz 是实测偏航角速度（去零偏后）
    PID_ChangeSp(&pid_turn, target_turn_rate);
    float gz_rad = APP_MPU6050_GetGzRate() * 0.0174533f;   // 度/s → rad/s
    float omega_diff = PID_Compute(&pid_turn, gz_rad);
    last_gz_rad     = gz_rad;
    last_omega_diff = omega_diff;

    // #9.设置电机转速（速度环基准 ± 转向差速）
    //   ★ 如果实测转向方向和指令相反，把下面两行的 + / - 互换即可
    APP_Motor_SetOmega_L(omega_ref + omega_diff);
    APP_Motor_SetOmega_R(omega_ref - omega_diff);

    control_telemetry.theta_dot_ref     = theta_dot_ref;
    control_telemetry.theta_dot_dot_ref = theta_dot_dot_ref;
    control_telemetry.omega_ref         = omega_ref;

    last_time = now;
}