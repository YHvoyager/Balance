#include "APP_Control.h"
#include "APP_PID.h"
#include "task.h"
#include "APP_MPU6050.h"
#include "qmath.h"
#include <math.h>
#include "APP_Encoder.h"
#include "APP_Motor_Control.h"

static PID_Typedef pid_velocity; //速度环的PID控制器
static PID_Typedef pid_theta; //Θ环的PID控制器
static PID_Typedef pid_theta_dot; //Θ'环的PID控制器

static APP_ControlTelemetry control_telemetry;
static float omega_ref = 0.0f; //平衡车轮胎转速,单位：rad/s
static uint64_t last_time = 0; //上一次执行时间,单位：微秒

static const float lp = 0.094f; //平衡车轮胎中心到MPU6050中心的距离,单位：m
static const float g = 9.8f; //重力加速度,单位：m/s^2
static const float rw = 0.024f; //平衡车轮胎半径,单位：m

/* ---------------------------------------------------------------------------
 * 安全限幅 + 位置环
 *
 * 【为什么加位置环】
 * 原来的 omega_ref 是纯积分：omega_ref += (1/rw)*∫x_dd dt，没有任何东西告诉
 * 车"你已经跑多远了"。前倾时 x_dd 变大 -> omega_ref 一路累积 -> 车全速冲出去，
 * 冲过头之后又没有回拉机制，所以表现为"突然超快加速然后往前倒"。
 * 推导：闭环特征根有两个，快根约 -350，慢根约 +1.9~+2.3（正根=发散）。
 * 慢根才是"只能站 10 秒"的元凶，而且加大 A/G 增益也压不下去它。
 *
 * 【做法】给 x_dd 叠一个位置弹簧，把纯积分器变成带位置反馈的二阶系统：
 *   x_dd += -POS_KP*x_est - POS_VDAMP*v_est
 * 位置只能靠轮速积分估计（编码器只有速度，没有位置计数）：
 *   v   = sgn(ω_ref) * |ω| * rw          <- 用方向指令定符号，停稳时 v 直接归零
 *   v_est 低通；x_est = ∫v_est dt
 *
 * ★ 位置环只是"叠加偏移量"，不是"覆盖" x_dd，所以即使 x_est 漂了，
 *   失效模式也只是"回原点偏一点"，theta/theta_dot 两级照旧工作。
 * ★ 关掉位置环：把 POS_KP 设成 0.0f 即可恢复原行为。
 * ★ 单独验证位置环：发 A kp=0 / G kp=0，手推车看是否弹回原点；
 *   若往一个方向一直跑，说明 POS_KP 符号反了，取负。
 * ------------------------------------------------------------------------ */
#define OMEGA_REF_MAX     20.0f   /* rad/s，ω_ref 上限 */
#define XDD_MAX            4.0f   /* m/s^2，逆解算输出上限 */
#define POS_KP             1.5f   /* 1/s^2，位置刚度 */
#define POS_VDAMP          2.0f   /* 1/s，速度阻尼，抑制回原点过冲 */
#define POS_X_MAX          0.10f  /* m，位置修正上限 */
#define POS_V_TAU          0.010f /* s，轮速低通时间常数 */
#define WHEEL_SPEED_MAX   50.0f   /* rad/s，编码器异常值保护 */

static float x_est = 0.0f;   //车身位移估计，单位：m
static float v_est = 0.0f;   //车身速度估计，单位：m/s（已低通）

static uint8_t ctrl_prev_en = 0;    

//
// @简介：初始化平衡车控制系统
// 
void APP_Control_Init(void){
    PID_Init(&pid_velocity, 1.0f, 0.0f, 0.0f);


    PID_Init(&pid_theta, 10.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_theta, 12.57f, -12.57f); // Θ环的PID控制器输出范围为[-12.57, 12.57]

    PID_Init(&pid_theta_dot, 32.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_theta_dot, 125.7f, -125.7f); // Θ'环的PID控制器输出范围为[-125.7, 125.7]
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

    uint8_t en = APP_Motor_IsEnabled();

    if (!en) {                       // 关机：清零，绝不累积 
        omega_ref = 0.0f;
        x_est = 0.0f;
        v_est = 0.0f;
        last_time = micros();
        PID_Reset(&pid_theta);
        PID_Reset(&pid_theta_dot);
        APP_Motor_SetOmega_L(0.0f);
        APP_Motor_SetOmega_R(0.0f);
        ctrl_prev_en = 0;
        return;
    }
    if (!ctrl_prev_en) {             // 使能上升沿：再复位一次，这一拍只复位不输出
        omega_ref = 0.0f;
        x_est = 0.0f;                // 位置环从原点开始
        v_est = 0.0f;
        last_time = micros();
        PID_Reset(&pid_theta);
        PID_Reset(&pid_theta_dot);
        ctrl_prev_en = 1;
        return;
    }

    uint64_t now = micros();
    float deltaT = (now - last_time) * 1e-6f; //单位：秒

    // #1.将外环的设定值SP设置为0
    PID_ChangeSp(&pid_theta, 0);

    // #2.读取传感器数据
    float theta = APP_MPU6050_GetPitch() * 0.0174533f; //单位：弧度
    float temp = theta * 57.29578f; //转换为角度
    if (qcos(temp) < 0.5f ) {                // |θ| > 72°，已经倒了
    omega_ref = 0.0f;
    APP_Motor_Cmd(0);                               // 直接断电，别挣扎
    return;
    }


    float theta_dot = (APP_MPU6050_GetGy() - APP_MPU6050_GetGyBias()) * 0.0174533f; //单位：弧度/秒

    // #3.计算外环PID输出
    float theta_dot_ref = PID_Compute(&pid_theta, theta);

    // #4.改变内环的设定值SP
    PID_ChangeSp(&pid_theta_dot, theta_dot_ref);

    // #5.计算内环PID输出
    float theta_dot_dot_ref = PID_Compute(&pid_theta_dot, theta_dot);

    // #6.倒立摆的逆解算
    float x_dot_dot_ref = (g * qsin(temp) - theta_dot_dot_ref * lp) / qcos(temp); //单位：m/s^2

    /* #6b.位置环：把 omega_ref 的无界积分器包成带位置反馈的二阶系统。
     *     方向用 omega_ref 的符号（车轮速度环稳态精确），大小用实测轮速。 */
    float wl = EncoderGetSpeed_L();
    float wr = EncoderGetSpeed_R();
    if (wl >  WHEEL_SPEED_MAX) wl =  WHEEL_SPEED_MAX;
    if (wl < -WHEEL_SPEED_MAX) wl = -WHEEL_SPEED_MAX;
    if (wr >  WHEEL_SPEED_MAX) wr =  WHEEL_SPEED_MAX;
    if (wr < -WHEEL_SPEED_MAX) wr = -WHEEL_SPEED_MAX;

    float w_meas = 0.5f * (wl + wr);                    // rad/s
    float w_sign = (omega_ref >= 0.0f) ? 1.0f : -1.0f;
    float v_now  = w_sign * fabsf(w_meas) * rw;         // m/s

    // 一阶低通：T=(tau*dt)/(tau+dt)，稳定且不用 expf
    float v_alpha = POS_V_TAU / (POS_V_TAU + deltaT);
    v_est += v_alpha * (v_now - v_est);
    x_est += v_est * deltaT;

    float pos_corr = -(POS_KP * x_est + POS_VDAMP * v_est);   // m/s^2
    if (pos_corr >  POS_X_MAX * POS_KP) pos_corr =  POS_X_MAX * POS_KP;
    if (pos_corr < -POS_X_MAX * POS_KP) pos_corr = -POS_X_MAX * POS_KP;
    x_dot_dot_ref += pos_corr;

    if (x_dot_dot_ref >  XDD_MAX) x_dot_dot_ref =  XDD_MAX;
    if (x_dot_dot_ref < -XDD_MAX) x_dot_dot_ref = -XDD_MAX;

    // #7.计算轮胎转速
    omega_ref += 1.0f / rw * x_dot_dot_ref * deltaT; //单位：rad/s
    if (omega_ref >  OMEGA_REF_MAX) omega_ref =  OMEGA_REF_MAX;
    if (omega_ref < -OMEGA_REF_MAX) omega_ref = -OMEGA_REF_MAX;


    // #9.设置电机转速
    APP_Motor_SetOmega_L(omega_ref);
    APP_Motor_SetOmega_R(omega_ref);

    control_telemetry.pitch_deg = theta * 57.29578f;
    control_telemetry.gyro_dps = theta_dot * 57.29578f;
    control_telemetry.theta_dot_ref = theta_dot_ref;
    control_telemetry.theta_dot_dot_ref = theta_dot_dot_ref;
    control_telemetry.omega_ref = omega_ref;
    control_telemetry.x_est_m = x_est;
    control_telemetry.v_est_mps = v_est;

    last_time = now;
}


