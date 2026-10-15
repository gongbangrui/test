/**
 * run_task.c — 技能赛2025 主控任务移植 (适配 master 底层)
 * 原代码: 技能赛2025/application/run_task.c (新版本)
 *
 * 数据源适配 (master 底层):
 *   HUI_data / hui[16] <- gray_sensor bitmask (master 1=黑, 旧语义 0=黑 -> 取反)
 *   data_storage[12]   <- ir_sensor.front   (0=触发, 低电平)
 *   data_storage[13]   <- ir_sensor.left    (桥左, 0=触发)
 *   data_storage[14]   <- ir_sensor.right   (桥右, 0=触发)
 *   data_storage[15]   <- 灰度全黑 (R4, 0=触发)
 *   data_storage[16]   <- 灰度全黑 (R5, 0=触发)
 *   get_rount_cnt      <- chassis_round_cnt() (CAN ecd 圈数累计)
 *   set_current        <- chassis_set_motor_rpm (4轮独立 RPM + 速度PID)
 *   yuyin(n)           <- voice_module_play(n)
 *   servo_pwm[i]       <- servo_pwm_set(pwm, i)
 */
#include "run_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "chassis_task.h"
#include "INS_task.h"
#include "gray_sensor.h"
#include "ir_sensor.h"
#include "voice_module.h"
#include "bsp_servo_pwm.h"
#include "actions.h"
#include "pid.h"

/* 俯仰角 (度), 与 debug 命令行 imu 的 p 一致 (INS_angle[1], pitch)。
 * 注: 旧代码用 [2] (roll, IMU 横装时上坡角表现为 roll), 若上电发现
 *     车头翘起时命令行 p 不变而 r 变, 此行应改回 INS_angle_go[2]。 */
#define get_yaw    INS_angle_go[0]*57.29578f
#define get_pitch  INS_angle_go[1]*57.29578f
#define get_roll   INS_angle_go[2]*57.29578f
#define sensor data_storage
#define STOP  stop(); while(1) osDelay(1);

#define get_rount_cnt chassis_round_cnt()

/* 灰度全黑阈值 (R4/R5 触发条件) */
#define R4_BLACK_CNT 14
#define R5_BLACK_CNT 14

/* ---- 旧 407 串口数据容器 (现由本任务 sensor_update() 从传感器填充) ---- */
uint8_t data_storage[27] = {1};
uint8_t hui[16] = {0};
uint16_t HUI_data = 0;

/* ---- 旧 407 串口数据容器 (由 sensor_update() 填充) ---- */
uint8_t zsp = 0xFF; /* 颜色: 0x32=红, 0x31=绿 (digit_read 转换) */


/* ---- 全局 (run_task.h extern) ---- */
const fp32* INS_angle_go;
fp32 go_yaw, go_pitch, go_yaw_inia;
int16_t v_r, v_l, V, v_delt;
int16_t set_speed_rpm[4] = {0, 0, 0, 0};
fp32 vx_run_set = 0.0f, angle_run_set = 0.0f;
fp32 angle_sum = 0;
fp32 speed_sum = 0;
fp32 biass = 0;
int last_r = 0;
int intia_rount_cnt = 0;
uint8_t qr[3] = {0};
int baojian1 = 0, baojian2 = 0, baojian3 = 0;


/* ---- 传感器刷新: 灰度 + 红外 -> 旧数据容器 ---- */
extern void sensor_update(void)
{
    gray_sensor_poll();
    const gray_sensor_t* gs = get_gray_sensor_point();
    uint16_t bm = gs->bitmask;
    HUI_data = (uint16_t)(~bm);
    for (int i = 0; i < 16; i++)
        hui[i] = ((bm >> i) & 1) ? 0 : 1;

    ir_sensor_poll();
    const ir_sensor_t* ir = get_ir_sensor_point();
    data_storage[12] = ir->front;
    data_storage[13] = ir->left;
    data_storage[14] = ir->right;
    data_storage[15] = (gs->line_count >= R4_BLACK_CNT) ? 0 : 1;
    data_storage[16] = (gs->line_count >= R5_BLACK_CNT) ? 0 : 1;
}

fp32 rex_abs(fp32 a)
{
    if (a < 0)
        a = -a;
    return a;
}

/* ---- PID 循线 (master line_track 参数) ---- */
#define TRACK_PID_KP 2.0f
#define TRACK_PID_KI 0.01f
#define TRACK_PID_KD 0.0f
#define TRACK_PID_MAX_WZ   4.0f
#define TRACK_PID_MAX_IOUT 0.3f
static pid_type_def track_pid;

/* 窄线(<4 通道) PID 循线; 宽线/路口(>=4) 不动作, 由调用者处理。
 * vx: 目标速度 m/s (如 0.5~1.0) */
void trackxian(fp32 vx)
{
    sensor_update();
    const gray_sensor_t* gs = get_gray_sensor_point();
    if (gs->line_count >= 3) return;
    PID_calc(&track_pid, gs->position, 0.0f);
    chassis_set_velocity(vx, track_pid.out);
}

void stop(void);
int detec_sensor(void);

void set_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void turn_run(int speed, fp32 angle);
void turn2(int speed, fp32 angle);
void turnleft(void);
void turnright(void);
int lukou_detect(int det);
void xpt(void);
void go12(void);
void gomen15(void);
void gomen25(void);
void gomen35(void);
void gomen4men3(void);
void gomen3men2(void);
void gomen2men15(void);
void gomen1men2(void);
void go24niu(void);
void ce1(void);
void ce2(void);
void ce3(void);
void ce4(void);
void go57xin(void);
void go2men1(void);
void go2men15(void);
void go2men25(void);
void go2men35(void);
void go2men45(void);
void go2men4(void);
void go2men1(void);
void go2men45niu(void);
void go12xin(void);
void go12xinniu(void);
void go12niu(void);
void wave_handc78(void );
void gomen2men3(void);
void go43(void);
void go36(void);
void go35(void);
void go45(void);
void go46(void);
void go315(void);
void go325(void);
void go32(void);
void go335(void);
void go43niu(void);
void go345(void);
void gomen3men45(void);
void go3men15(void);
void go3men25(void);
void go3men35(void);
void go3men45(void);
void go3men16(void);
void go3men26(void);
void go3men36(void);
void go3men46(void);
void go25(void);
void go26(void);
void gomen13(void);
void gomen23(void);
void gomen33(void);
void go72(void);
void gomen45(void);
void gomen45niu(void);
void gomen14(void);
void gomen24(void);
void gomen34(void);
void gomen46(void);
void go57(void);
void go2men47(void);
void go76(void);
void go75(void);
void go75niu(void);
void go7men14(void);
void go7men13(void);
void go7men12(void);
void go7men24(void);
void go7men23(void);
void go7men22(void);
void go7men34(void);
void go7men33(void);
void go7men32(void);
void go7men44(void);
void go7men43(void);
void go7men42(void);
void go58(void);
void go86(void);
void go85(void);
void go8men14(void);
void go8men13(void);
void go8men12(void);
void go8men24(void);
void go8men23(void);
void go8men22(void);
void go8men34(void);
void go8men33(void);
void go8men32(void);
void go8men44(void);
void go8men43(void);
void go8men42(void);
void go67(void);
void go1men47(void);
void go68(void);
void go6men11(void);
void go6men21(void);
void go6men31(void);
void go6men41(void);
void go5men11(void);
void go5men21(void);
void go5men31(void);
void go5men41(void);
void go21(void);
void go21niu(void);
void go31(void);
void go41(void);
void go41niu(void);
void go7men11(void);
void go73(void);
void go7men21(void);
void go7men31(void);
void go7men41(void);

void gomen3men2(void);
void gomen2men1(void);
void go5men44(void);
void go5men44niu(void);
void go5men44niuniu(void);
void go4men1(void);
void downplat(void);
void gohill(void);
void gobri(void);
void goqqb(void);
void set0rount(void);
void gomen1jia(void);
void go517(void);
void go57niu(void);
void go78(void);
void go6men1jia(void);
void xingomen1jia(void);
void xingomen2jia(void);
void xingo4men4jia(void);
void xingo4men3jia(void);
void go5men1jia(void);
void go5men3jia(void);
void go5men2jia(void);
void xingo3men2jia(void);
void goplat(void);
void gohome(void);
void gomen2jia(void);
void go6men2jia(void);
void go4men4niu(void);
void go6men3jia(void);
void go6men4jia(void);
void go8zhi(void);
void gozhimen1jia(void);
void gozhimen2jia(void);
void gozhimen3jia(void);
void gozhimen4jia(void);
void go4zhi3(void);
void gomen3jia(void);
void sensor_up(int k);
void wave_hand(void);
void yuyin(int a);
void trackxian1(void);
void trackxian2(void);
void trackxian3(void);
void trackxian5(void);
void trackxian8(void);
void trackxian12(void);
void trackxian15(void);
void go6men43(void);
void go3jia(void);
void go6men33(void);
void xingomen3jia(void);


void setspeed2(int speed1, int speed2)
{
    set_current(speed1, speed2, speed1, speed2);
}

fp32 get_biass3(void);

fp32 Gethuidu(int a)
{
    return hui[a - 1];
}

void turnleft()
{
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }
}

void turnright()
{
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }
}

void up_hand(void);

void gohome()
{
    while (get_roll < 15)
        trackxian5();

    while (get_roll > 2)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
}

int lukou_detect(int det) //?????det???????·?ì??10ms??????·?? ??1 ????0
{
    int num = 0;
    int lukou_num = 0;
    for (int i = 0; i < 2; i++) //????·?2?
    {
        for (int j = 0; j < 16; j++)
        {
            if (hui[j] == 0) //????????p0q???
                num++;
        }
        if (num >= (det)) //????5???П??ˇ??·?
        {
            lukou_num++;
            if (lukou_num == 1) osDelay(10); //????????10msìл???
        }
        num = 0;
    }
    if (lukou_num >= 2) return 1; //??2??ˇ·?????ˇ????·?ì????

    return 0;
}

void trackxian1()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(1000, 1000);
        break; //11111110 01111111
    case 0b1111110001111111: setspeed2(1000, 500);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(500, 1000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(1000, 500);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(1000, 500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(500, 1000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(500, 1000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(1000, 500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(500, 1000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(1000, 0);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(1000, 0);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(0, 1000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(0, 1000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(1000, 0);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(0, 1000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(1000, 0);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(1000, 0);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(0, 1000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(0, 1000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(1000, 0);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(0, 1000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(1000, 0);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(1000, 0);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(0, 1000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(0, 1000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(1000, 0);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(0, 1000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(1000, 0);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(1000, 0);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(0, 1000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(0, 1000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(1000, 0);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(0, 1000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(0, 1000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(0, 1000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(1000, 0);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(1000, 0);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(0, 1000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(1000, 0);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(0, 1000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(0, 1000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(1000, 0);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(1000, 0);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(0, 1000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(1000, 0);
        break; //01111111 11111111
    default: setspeed2(1000, 1000);
        break;
    }
}

///0=??????   1?????  2=????  3=??? 4=???  5=????  6=???  7=???  8=???????  9=???


void trackxian3()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(3000, 3000);
        break; //11111110 01111111
    case 0b1111110001111111: setspeed2(3000, 2500);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(2500, 3000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(3000, 2500);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(3000, 2500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(2500, 3000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(2500, 3000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(3000, 2500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(2500, 3000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(3000, 2000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(3000, 2000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(2000, 3000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(2000, 3000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(3000, 2000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(2000, 3000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(3000, 1500);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(3000, 1500);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(1500, 3000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(1500, 3000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(3000, 1500);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(1500, 3000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(3000, 1000);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(3000, 1000);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(1000, 3000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(1000, 3000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(3000, 1000);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(1000, 3000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(3000, 500);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(3000, 500);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(500, 3000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(500, 3000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(3000, 500);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(500, 3000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(0, 3000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(0, 3000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(3000, 0);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(3000, 0);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(0, 3000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(3000, 0);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(0, 3000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(0, 3000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(3000, 0);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(3000, 0);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(0, 3000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(3000, 0);
        break; //01111111 11111111
    default: setspeed2(3000, 3000);
        break;
    }
}


void trackxian2()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(2000, 2000);
        break; //11111110 01111111
    case 0b1111110001111111: setspeed2(2000, 1500);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(1500, 2000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(2000, 1500);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(2000, 1500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(1500, 2000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(1500, 2000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(2000, 1500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(1500, 2000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(2000, 1000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(2000, 1000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(1000, 2000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(1000, 2000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(2000, 1000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(1000, 2000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(2000, 500);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(2000, 500);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(500, 2000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(500, 2000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(2000, 500);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(500, 2000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(2000, 500);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(2000, 500);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(500, 2000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(500, 2000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(2000, 500);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(500, 2000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(2000, 500);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(2000, 500);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(500, 2000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(500, 2000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(2000, 500);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(500, 2000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(0, 2000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(0, 2000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(2000, 0);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(2000, 0);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(0, 2000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(2000, 0);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(0, 2000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(0, 2000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(2000, 0);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(2000, 0);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(0, 2000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(2000, 0);
        break; //01111111 11111111
    default: setspeed2(2000, 2000);
        break;
    }
}

///0=??????   1?????  2=????  3=??? 4=???  5=????  6=???  7=???  8=???????  9=???
void trackxian5()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(5000, 5000);
        break; //11111110 01111111		//???λ?
    case 0b1111110001111111: setspeed2(5000, 4000);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(4000, 5000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(5000, 3500);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(5000, 3500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(3500, 5000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(3500, 5000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(5000, 3500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(3500, 5000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(5000, 3000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(5000, 3000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(3000, 5000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(3000, 5000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(5000, 3000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(3000, 5000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(5000, 3000);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(5000, 3000);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(3000, 5000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(3000, 5000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(5000, 3000);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(3000, 5000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(5000, 2000);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(5000, 2000);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(2000, 5000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(2000, 5000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(5000, 2000);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(2000, 5000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(5000, 2000);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(5000, 2000);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(2000, 5000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(2000, 5000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(5000, 2000);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(2000, 5000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(1000, 5000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(1000, 5000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(5000, 1000);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(5000, 1000);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(1000, 5000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(5000, 1000);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(0, 5000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(0, 5000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(5000, 0);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(5000, 0);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(0, 5000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(5000, 0);
        break; //01111111 11111111

    default: setspeed2(5000, 5000);
        break;
    }
}

void trackxian8()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(8000, 8000);
        break; //11111110 01111111		//???λ?
    case 0b1111110001111111: setspeed2(8000, 7000);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(7000, 8000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(8000, 7000);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(8000, 6500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(6500, 8000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(6500, 8000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(8000, 6500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(6500, 8000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(8000, 6000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(8000, 6000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(6000, 8000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(6000, 8000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(8000, 6000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(6000, 8000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(8000, 5000);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(8000, 5000);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(5000, 8000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(5000, 8000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(8000, 5000);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(5000, 8000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(8000, 4000);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(8000, 4000);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(4000, 8000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(4000, 8000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(8000, 4000);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(4000, 8000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(8000, 3000);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(8000, 3000);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(3000, 8000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(3000, 8000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(8000, 3000);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(3000, 8000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(8000, 1000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(1000, 8000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(8000, 1000);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(8000, 1000);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(1000, 8000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(8000, 4000);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(1000, 8000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(1000, 8000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(8000, 1000);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(8000, 1000);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(1000, 8000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(8000, 1000);
        break; //01111111 11111111
    default: setspeed2(8000, 8000);
        break;
    }
}


void trackxian10()
{
    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(10000, 10000);
        break; //11111110 01111111		//???λ?
    case 0b1111110001111111: setspeed2(10000, 9500);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(9500, 10000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(10000, 9000);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(10000, 9000);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(9000, 10000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(9000, 10000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(10000, 9000);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(9000, 10000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(10000, 8000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(10000, 8000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(8000, 10000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(8000, 10000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(10000, 8000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(8000, 10000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(10000, 7500);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(10000, 7500);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(7500, 10000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(7500, 10000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(10000, 7500);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(7500, 10000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(10000, 6500);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(10000, 6500);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(6500, 10000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(6500, 10000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(10000, 6500);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(6500, 10000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(10000, 6000);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(10000, 6000);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(6000, 10000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(6000, 10000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(10000, 6000);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(6000, 10000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(5000, 10000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(5000, 10000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(10000, 5000);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(10000, 5000);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(5000, 10000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(10000, 5000);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(7000, 10000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(7000, 10000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(10000, 7000);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(10000, 7000);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(5000, 10000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(10000, 5000);
        break; //01111111 11111111
    default: setspeed2(10000, 10000);
        break;
    }
}


void trackxian12()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(12000, 12000);
        break; //11111110 01111111		//???λ?
    case 0b1111110001111111: setspeed2(12000, 12000);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(12000, 12000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(12000, 11500);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(12000, 11500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(11500, 12000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(11500, 12000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(12000, 11500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(11500, 12000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(12000, 11000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(12000, 11000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(11000, 12000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(11000, 12000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(12000, 11000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(11000, 12000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(12000, 10500);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(12000, 10500);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(10500, 12000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(10500, 12000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(12000, 10500);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(10500, 12000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(12000, 10000);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(12000, 10000);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(10000, 12000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(10000, 12000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(12000, 10000);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(10000, 12000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(12000, 9000);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(12000, 9000);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(9000, 12000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(9000, 12000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(12000, 9000);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(9000, 12000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(8000, 12000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(8000, 12000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(12000, 8000);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(12000, 8000);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(8000, 12000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(12000, 8000);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(7000, 12000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(7000, 12000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(12000, 7000);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(12000, 7000);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(5000, 12000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(12000, 5000);
        break; //01111111 11111111
    default: setspeed2(12000, 12000);
        break;
    }
}


void trackxian15()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0b1111111001111111: setspeed2(15000, 15000);
        break; //11111110 01111111		//???λ?
    case 0b1111110001111111: setspeed2(15000, 15000);
        break; //11111100 01111111
    case 0b1111111000111111: setspeed2(15000, 15000);
        break; //11111110 00111111
    case 0b1111111011111111: setspeed2(15000, 14500);
        break; //11111110 11111111
    case 0b1111110011111111: setspeed2(15000, 14500);
        break; //11111100 11111111
    case 0b1111111101111111: setspeed2(14500, 15000);
        break; //11111111 01111111
    case 0b1111111100111111: setspeed2(14500, 15000);
        break; //11111111 00111111
    case 0b1111100011111111: setspeed2(15000, 14500);
        break; //11111000 11111111
    case 0b1111111100011111: setspeed2(14500, 15000);
        break; //11111111 00011111

    case 0b1111110111111111: setspeed2(15000, 14000);
        break; //11111101 11111111
    case 0b1111100111111111: setspeed2(15000, 14000);
        break; //11111001 11111111
    case 0b1111111110111111: setspeed2(14000, 15000);
        break; //11111111 10111111
    case 0b1111111110011111: setspeed2(14000, 15000);
        break; //11111111 10011111
    case 0b1111000111111111: setspeed2(15000, 14000);
        break; //11110001 11111111
    case 0b1111111110001111: setspeed2(14000, 15000);
        break; //11111111 10001111

    case 0b1111101111111111: setspeed2(15000, 13500);
        break; //11111011 11111111
    case 0b1111001111111111: setspeed2(15000, 13500);
        break; //11110011 11111111
    case 0b1111111111011111: setspeed2(13500, 15000);
        break; //11111111 11011111
    case 0b1111111111001111: setspeed2(13500, 15000);
        break; //11111111 11001111
    case 0b1110001111111111: setspeed2(15000, 13500);
        break; //11100011 11111111
    case 0b1111111111000111: setspeed2(13500, 15000);
        break; //11111111 11000111

    case 0b1111011111111111: setspeed2(15000, 12500);
        break; //11110111 11111111
    case 0b1110011111111111: setspeed2(15000, 12500);
        break; //11100111 11111111
    case 0b1111111111101111: setspeed2(12500, 15000);
        break; //11111111 11101111
    case 0b1111111111100111: setspeed2(12500, 15000);
        break; //11111111 11100111
    case 0b1100011111111111: setspeed2(15000, 12500);
        break; //11000111 11111111
    case 0b1111111111100011: setspeed2(12500, 15000);
        break; //11111111 11100011

    case 0b1110111111111111: setspeed2(15000, 11500);
        break; //11101111 11111111
    case 0b1100111111111111: setspeed2(15000, 11500);
        break; //11001111 11111111
    case 0b1111111111110111: setspeed2(11500, 15000);
        break; //11111111 11110111
    case 0b1111111111110011: setspeed2(11500, 15000);
        break; //11111111 11110011
    case 0b1000111111111111: setspeed2(15000, 11500);
        break; //10001111 11111111
    case 0b1111111111110001: setspeed2(11500, 15000);
        break; //11111111 11110001

    case 0b1111111111111011: setspeed2(10000, 15000);
        break; //11111111 11111011
    case 0b1111111111111001: setspeed2(10000, 15000);
        break; //11111111 11111001
    case 0b1101111111111111: setspeed2(15000, 10000);
        break; //11011111 11111111
    case 0b1001111111111111: setspeed2(15000, 10000);
        break; //10011111 11111111
    case 0b1111111111111000: setspeed2(10000, 15000);
        break; //11111111 11111000
    case 0b0001111111111111: setspeed2(15000, 10000);
        break; //00011111 11111111

    case 0b1111111111111101: setspeed2(8000, 15000);
        break; //11111111 11111101
    case 0b1111111111111100: setspeed2(8000, 15000);
        break; //11111111 11111100
    case 0b1011111111111111: setspeed2(15000, 8000);
        break; //10111111 11111111
    case 0b0011111111111111: setspeed2(15000, 8000);
        break; //00111111 11111111
    case 0b1111111111111110: setspeed2(7000, 15000);
        break; //11111111 11111110
    case 0b0111111111111111: setspeed2(15000, 7000);
        break; //01111111 11111111
    default: setspeed2(15000, 15000);
        break;
    }
}

void gohill()
{
    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();
}

void goplat()
{
    while (data_storage[12] != 0)
        trackxian5();
    setspeed2(3000, 3000);
    osDelay(500);
    setspeed2(-2000, -2000);
    osDelay(250);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(166) < rex_abs(angle_sum))
                break;
            set_current(-5000, 5000, -5000, 5000);
            osDelay(1);
        }
        stop();
    }

    stop();
    osDelay(50);
}


void run_task(void const* argument)
{
    INS_angle_go = get_INS_angle_point();
    v_r = v_l = 0;
    const fp32 pid_params[3] = {TRACK_PID_KP, TRACK_PID_KI, TRACK_PID_KD};
    PID_init(&track_pid, PID_POSITION, pid_params, TRACK_PID_MAX_WZ, TRACK_PID_MAX_IOUT);
    osDelay(3000);


    /* ###################### 开始 start ###################### */
    for (;;)
    {
        /* 等启动: 前红外 (data_storage[12]==0 触发) 的 触发->释放 边沿 */
        // servo_pwm_set(1200,0);
        sensor_update();
        while (data_storage[12] != 0)
        {
            sensor_update();
            osDelay(50);
        }
        while (data_storage[12] == 0)
        {
            sensor_update();
            osDelay(50);
        }


        ce3();
        STOP

        go12();
        go2men4();
        tl_scan();
        gomen4men3();
        tl_scan();
        gomen35();
        go57();
        go7men34();
        go41();


        while (data_storage[12] != 0)
        {
            sensor_update();
            osDelay(50);
        }
        while (data_storage[12] == 0)
        {
            sensor_update();
            osDelay(50);
        }


        go12xin();
        go2men47();
        go57xin();
        go75();
        go5men44();
        go43();
        go32();
        go21();

        STOP
///////////////////////
        go12();
        go2men4();
        tl_scan();
        gomen45();
        go57();
        go75();
        go5men44();
        go41();


        while (data_storage[12] != 0)
        {
            sensor_update();
            osDelay(50);
        }
        while (data_storage[12] == 0)
        {
            sensor_update();
            osDelay(50);
        }


        go12xin();
        go2men45();
        go57xin();
        go75();
        go5men44();
        go41();




        /////////////////////////////
STOP
        go12();
        go2men4();
        if (tl_scan() == 1)
        {
            gomen45();
            if (qr[2] == 0x37)
            {
                yuyin(1);
                go57();
                if (baojian1 + baojian2 == 2)
                {
                    go7men42();
                    go21();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go57();
                    go7men42();
                    go21();
                }
                else if (baojian1 + baojian2 == 3)
                {
                    go7men43();
                    go31();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go57();
                    go7men43();
                    go31();
                }
                else if (baojian1 + baojian2 == 4)
                {
                    go7men44();
                    go41();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go57();
                    go7men44();
                    go41();
                }
                else
                {
                    go75();
                    go5men41();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go57xin();
                    go75();
                    go5men41();
                }
            }

            else
            {
                go58();
                if (baojian1 + baojian2 == 2)
                {
                    go8men42();
                    go21();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go58();
                    go8men42();
                    go21();
                }
                else if (baojian1 + baojian2 == 3)
                {
                    go8men43();
                    go21();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go58();
                    go8men43();
                    go31();
                }
                else if (baojian1 + baojian2 == 4)
                {
                    go7men12();
                    go21();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go57();
                    go7men12();
                    go21();
                }
                else
                {
                    go85();
                    go5men41();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    go12();
                    go2men45();
                    go58();
                    go85();
                    go5men41();
                }
            }
        }
        else
        {
            gomen4men3();
            if (tl_scan() == 1)
            {
                gomen35();
                if (qr[2] == 0x37)
                {
                    go57();
                    if (baojian1 + baojian2 == 2)
                    {
                        go7men32();
                        go21();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go57();
                        go7men32();
                        go21();
                    }
                    else if (baojian1 + baojian2 == 3)
                    {
                        go7men33();
                        go31();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go57();
                        go7men33();
                        go31();
                    }
                    else if (baojian1 + baojian2 == 4)
                    {
                        go7men34();
                        go41();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go57();
                        go7men34();
                        go41();
                    }
                    else if (baojian1 + baojian2 == 5)
                    {
                        go75();
                        go5men31();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go57();
                        go75();
                        go5men31();
                    }
                }
                else
                {
                    go58();
                    if (baojian1 + baojian2 == 2)
                    {
                        go8men32();
                        go21();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go58();
                        go8men32();
                        go21();
                    }
                    else if (baojian1 + baojian2 == 3)
                    {
                        go8men33();
                        go31();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go58();
                        go8men33();
                        go31();
                    }
                    else if (baojian1 + baojian2 == 4)
                    {
                        go8men34();
                        go41();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go58();
                        go8men34();
                        go41();
                    }
                    else
                    {
                        go85();
                        go5men31();
                        while (data_storage[12] != 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        while (data_storage[12] == 0)
                        {
                            sensor_update();
                            osDelay(50);
                        }
                        go12();
                        go2men35();
                        go58();
                        go85();
                        go5men31();
                    }
                }
            }

            else
            {
                gomen3men2();
                if (tl_scan() == 1)
                {
                    gomen25();
                    if (qr[2] == 0x38)
                    {
                        if (baojian1 + baojian2 == 2)
                        {
                            go8men22();
                            go21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go58();
                            go8men22();
                            go21();
                        }
                        else if (baojian1 + baojian2 == 3)
                        {
                            go8men23();
                            go31();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go58();
                            go8men23();
                            go31();
                        }
                        else if (baojian1 + baojian2 == 4)
                        {
                            go8men24();
                            go41();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go58();
                            go8men24();
                            go41();
                        }
                        else
                        {
                            go85();
                            go5men21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go58();
                            go85();
                            go5men21();
                        }
                    }
                    else
                    {
                        go57();
                        if (baojian1 + baojian2 == 2)
                        {
                            go7men22();
                            go21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go57();
                            go7men22();
                            go21();
                        }
                        else if (baojian1 + baojian2 == 3)
                        {
                            go7men23();
                            go21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go57();
                            go7men23();
                            go31();
                        }
                        else if (baojian1 + baojian2 == 4)
                        {
                            go7men24();
                            go21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go57();
                            go7men24();
                            go41();
                        }
                        else
                        {
                            go75();
                            go5men21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men25();
                            go57();
                            go75();
                            go5men21();
                        }
                    }
                }
                else
                {
                    gomen2men15();
                    if (qr[2] == 0x37)
                    {
                        if (baojian1 + baojian2 == 2)
                        {
                            go7men12();
                            go21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go57();
                            go7men12();
                            go21();
                        }
                        else if (baojian1 + baojian2 == 3)
                        {
                            go7men13();
                            go31();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go57();
                            go7men13();
                            go31();
                        }
                        else if (baojian1 + baojian2 == 4)
                        {
                            go7men14();
                            go41();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go57();
                            go7men14();
                            go41();
                        }
                        else
                        {
                            go75();
                            go5men11();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go57();
                            go75();
                            go5men11();
                        }
                    }
                    else
                    {
                        go58();
                        if (baojian1 + baojian2 == 2)
                        {
                            go8men12();
                            go21();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go58();
                            go8men12();
                            go21();
                        }
                        else if (baojian1 + baojian2 == 3)
                        {
                            go8men13();
                            go31();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go58();
                            go8men13();
                            go31();
                        }
                        else if (baojian1 + baojian2 == 4)
                        {
                            go8men14();
                            go41();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go58();
                            go8men14();
                            go41();
                        }
                        else
                        {
                            go85();
                            go5men11();
                            while (data_storage[12] != 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            while (data_storage[12] == 0)
                            {
                                sensor_update();
                                osDelay(50);
                            }
                            go12();
                            go2men15();
                            go58();
                            go85();
                            go5men11();
                        }
                    }
                }
            }
        }


        wave_hand();

        tl_scan() == 1; //红绿灯
        qr_scan(qr); //扫码
        qr[0];
        baojian1 = digit_read();


        go58();
        STOP


        go12();
        go2men4();


        // set0rount();
        // while(get_rount_cnt-intia_rount_cnt<20)
        // 	trackxian(0.3f);
        // while(hui[8] != 0 && hui[9] != 0)
        // 	trackxian(0.3f);
        //
        // {fp32 last_angle=0;
        //  	angle_sum =0;
        //  	last_angle=get_yaw;
        //
        //
        //  	for(int i =0;;i++){
        //  		go_yaw =get_yaw;
        //  		go_yaw_inia = last_angle-go_yaw;
        //  		if(rex_abs(go_yaw_inia)<45)
        //  			angle_sum+=go_yaw_inia;
        //  		last_angle = go_yaw;
        //  		if(rex_abs(130)<rex_abs(angle_sum))
        //  			break;
        //  		set_current(6500,-500,6500,-500);
        //  		osDelay(1);
        //  	}
        // }
        //go2men1();
        // gomen1men2();
        // gomen2men3();

        while (get_rount_cnt - intia_rount_cnt < 10)
            trackxian(0.3f);
        while (get_roll < 15)
            trackxian(0.5f);

        intia_rount_cnt = get_rount_cnt;
        while (get_rount_cnt - intia_rount_cnt < 80)
            trackxian(0.3f);

        // while(get_roll < 15)
        // 	trackxian(0.5f);
        // while(data_storage[12] != 0)
        // 	setspeed2(3000,3000);
        // osDelay(50);
        //
        // setspeed2(-2000,-2000);
        // osDelay(200);
        // stop();
        // osDelay(1000);
        // turn_run(5000,162);
        // stop();
        // osDelay(100);
        // gomen15();


        //     zsp = (run_digit_read() == 0) ? 0x32 : 0x31;  /* 红=0x32, 绿=0x31 */
        // while(get_roll < 15)
        // 		trackxian5();


        go12();
        gomen34();
        go46();

        STOP;
    }
}

//	set0rount();
//	while(get_rount_cnt-intia_rount_cnt < 40)
//		trackxian3();
//	while(hui[14] != 0 && hui[15] != 0)
//		trackxian5();
//
//	{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<45)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(28)<rex_abs(angle_sum))
//			 break;
//		 set_current(7000,500,7000,500);
//		 osDelay(1);
//	 }
// }
//
//	set0rount();
//  while(get_rount_cnt-intia_rount_cnt<30)
//    trackxian5();
//  while(get_roll < 15)
//		trackxian3();
//
//	intia_rount_cnt=get_rount_cnt;
//	while(get_rount_cnt-intia_rount_cnt<80)
//		trackxian2();
//	set0rount();
//	while(get_rount_cnt-intia_rount_cnt<10)
//		trackxian3();
//
//
//	while(hui[14] != 0 && hui[15] != 0)
//			trackxian5();
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(123)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//	set0rount();
//  while(get_rount_cnt-intia_rount_cnt<30)
//		trackxian5();
//
//	while(hui[0] != 0 && hui[1] != 0)
//		trackxian5();
//	  osDelay(200);
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(75)<rex_abs(angle_sum))
//			 break;
//		 set_current(-1500,6500,-1500,6500);
//		 osDelay(1);
//	 }
//
//
// }
//
//  set0rount();
//  while(get_rount_cnt-intia_rount_cnt<40)
//		trackxian5();
//
//	while(!(lukou_detect(5)))
//		trackxian5();
//
//	osDelay(50);
//	stop();
//	osDelay(500);


//		if(hld==0x31){

//		 if (pt2==0x35)
//		 {
//			 gomen15();
//
//			 if(pt3==0x37)
//			 {
//				 go57();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men12();
//
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//		       go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go315();
//					 go57();
//	         go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men13();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//		       go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go315();
//					 go57();
//	         go7men11();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men14();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men11();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go23();
//					 go315();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go58();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men13();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men14();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 go58();
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//				 }
//
//			 }
//
//		 }
//		else if(pt2==0x36)
//		{
//			gomen16();
//			 if(pt3==0x37)
//			 {
//				 go67();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men13();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men14();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go68();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men13();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men14();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go315();
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men11();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go315();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men11();
//				 }
//
//			 }
//		}
//
//	}
//
//		else if(hld==0x32)
//		{
//		turn_run(5000,165);
//		stop();
//		osDelay(150);
//		set0rount();
//		while(get_rount_cnt-intia_rount_cnt<30)
//			trackxian5();
//		while(hui[14] != 0 && hui[15] != 0)
//			trackxian5();
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(125)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//	  set0rount();
//		while(get_rount_cnt-intia_rount_cnt<30)
//			trackxian5();
//	  while(!(lukou_detect(5)))
//		  trackxian5();
//
//	  osDelay(50);
//	  stop();
//	  osDelay(500);
//		//men2
//				 if (pt2==0x35)
//		 {
//			 gomen25();
//
//			 if(pt3==0x37)
//			 {
//				 go57();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men22();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go325();
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men23();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go325();
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go325();
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go325();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go325();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men21();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go58();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men22();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go325();
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men23();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go325();
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go325();
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go325();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go325();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men21();
//				 }
//
//			 }
//
//		 }
//		else if(pt2==0x36)
//		{
//			gomen26();
//			 if(pt3==0x37)
//			 {
//				 go67();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men22();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go325();
//					 go57();
//					 go7men21();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men23();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go5men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go6men21();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go68();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men22();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men23();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go5men21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go6men21();
//				 }
//
//			 }
//		}
//
//
//		}
//		else if(hld==0x33)
//		{
//		turn_run(5000,165);
//		stop();
//		osDelay(150);
//		set0rount();
//		while(get_rount_cnt-intia_rount_cnt<30)
//			trackxian5();
//		while(hui[14] != 0 && hui[15] != 0)
//			trackxian5();
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(125)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//	  set0rount();
//		while(get_rount_cnt-intia_rount_cnt<30)
//			trackxian5();
//	  while(!(lukou_detect(5)))
//		  trackxian5();
//
//	  osDelay(50);
//	  stop();
//	  osDelay(500);
//			turn_run(5000,165);
//		  stop();
//		  osDelay(150);
//		  set0rount();
//		  while(get_rount_cnt-intia_rount_cnt<30)
//			  trackxian5();
//		  while(hui[14] != 0 && hui[15] != 0)
//			  trackxian5();
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(140)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//
//	set0rount();
//  while(get_rount_cnt-intia_rount_cnt < 50)
//		trackxian5();
//	set0rount();
//  while(get_rount_cnt-intia_rount_cnt < 150)
//		trackxian8();
//
//	while(hui[14] != 0 && hui[15] != 0)
//		trackxian5();
//
//	{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(135)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//  set0rount();
//  while(get_rount_cnt-intia_rount_cnt < 30)
//		trackxian5();
//	while(!(lukou_detect(5)))
//		trackxian5();
//
//	osDelay(50);
//	stop();
//	osDelay(500);
//		//men3
//			if (pt2==0x35)
//		 {
//			 gomen35();
//
//			 if(pt3==0x37)
//			 {
//				 go57();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men32();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go335();
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men33();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go335();
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men34();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go335();
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go335();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go335();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men31();
//
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go58();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men32();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go335();
//					 go57();
//					 go7men31();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men33();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go335();
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men34();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go335();
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go335();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go335();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men31();
//				 }
//
//			 }
//
//		 }
//		else if(pt2==0x36)
//		{
//			gomen36();
//			 if(pt3==0x37)
//			 {
//				 go67();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men32();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men33();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men34();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go5men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go6men31();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go68();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men32();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men33();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men34();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go5men31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go6men31();
//				 }
//
//			 }
//		}
//
//		}
//				else if(hld==0x34)
//		{
//
//		turn_run(5000,165);
//		stop();
//		osDelay(150);
//		set0rount();
//		while(get_rount_cnt-intia_rount_cnt<30)
//			trackxian5();
//		while(hui[14] != 0 && hui[15] != 0)
//			trackxian5();
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(125)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//	  set0rount();
//		while(get_rount_cnt-intia_rount_cnt<30)
//			trackxian5();
//	  while(!(lukou_detect(5)))
//		  trackxian5();
//
//	  osDelay(50);
//	  stop();
//	  osDelay(500);
//			turn_run(5000,165);
//		  stop();
//		  osDelay(150);
//		  set0rount();
//		  while(get_rount_cnt-intia_rount_cnt<30)
//			  trackxian5();
//		  while(hui[14] != 0 && hui[15] != 0)
//			  trackxian5();
//
//		{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(140)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//
//	set0rount();
//  while(get_rount_cnt-intia_rount_cnt < 50)
//		trackxian5();
//	set0rount();
//  while(get_rount_cnt-intia_rount_cnt < 150)
//		trackxian8();
//
//	while(hui[14] != 0 && hui[15] != 0)
//		trackxian5();
//
//	{fp32 last_angle=0;
//		  angle_sum =0;
//		 last_angle=get_yaw;

//
//			 for(int i =0;;i++){
//				 go_yaw =get_yaw;
//				 go_yaw_inia = last_angle-go_yaw;
//				 if(rex_abs(go_yaw_inia)<20)
//					 angle_sum+=go_yaw_inia;
//				 last_angle = go_yaw;
//				if(rex_abs(135)<rex_abs(angle_sum))
//			 break;
//		 set_current(6500,-1000,6500,-1000);
//		 osDelay(1);
//	 }
// }
//  set0rount();
//  while(get_rount_cnt-intia_rount_cnt < 30)
//		trackxian5();
//	while(!(lukou_detect(5)))
//		trackxian5();
//
//	osDelay(50);
//	stop();
//	osDelay(500);

//
//			//men4
//			if (pt2==0x35)
//		 {
//			 gomen45();
//
//			 if(pt3==0x37)
//			 {
//				 go57();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men42();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go345();
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go345();
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men44();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go345();
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go345();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go345();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men41();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go58();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men42();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go21();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go24();
//					 go43();
//					 go345();
//					 go57();
//					 go7men41();
//
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go31();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go345();
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men44();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go43();
//					 go345();
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go5men41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go345();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go57();
//					 go7men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men41();
//
//					 int speed = 0;
//		       uint16_t k;
//		       while(data_storage[12] != 0)
//	  	       osDelay(50);
//	         while(data_storage[12] == 0)
//			       osDelay(50);
//           yuyin(0);
//		       osDelay(150);
//		       up_hand();
//					 go12();
//					 go24();
//					 go43();
//					 go345();
//					 go57();
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 yuyin(10);
//					 osDelay(150);
//					 go6men41();
//				 }
//
//			 }
//
//		 }
//		else if(pt2==0x36)
//		{
//			gomen46();
//			 if(pt3==0x37)
//			 {
//				 go67();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go7men42();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go7men43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go7men44();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go75();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go5men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go76();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go6men41();
//				 }
//
//			 }
//
//			 else if(pt3==0x38)
//			 {
//				 go68();
//				 if(xiansuo1+xiansuo2==0x32)
//				 {
//					 go8men42();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go21();
//				 }
//				 else if(xiansuo1+xiansuo2==0x33)
//				 {
//					 go8men43();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go31();
//				 }
//				 else if(xiansuo1+xiansuo2==0x34)
//				 {
//					 go8men44();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x35)
//				 {
//					 go85();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go5men41();
//				 }
//				 else if(xiansuo1+xiansuo2==0x36)
//				 {
//					 go86();
//					 turn_run(-5000,337);
//					 stop();
//					 osDelay(150);
//					 up_hand();
//					 go6men41();
//				 }
//
//			 }
//		}
//
//		}
//


//	gomen16();
//	go67();
//	go7men14();
//	go41();
//  STOP
//		servo_pwm_set(1000, 0);//??
//    servo_pwm_set(1500, 1);//16
//		yuyin(0);
//		osDelay(500);
//		servo_pwm_set(1000, 2);
//		osDelay(500);
//		servo_pwm_set(2000, 3);
//		osDelay(500);
//		servo_pwm_set(2000, 0);
//		STOP
//			osDelay(1000);
//	}

//}

void turn_run(int speed, fp32 angle)
{
    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = get_yaw;
    uint32_t t0 = osKernelSysTick();


    for (int i = 0;; i++)
    {
        go_yaw = get_yaw;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(angle) < rex_abs(angle_sum))
            break;
        if (osKernelSysTick() - t0 > 10000)
            break;
        if (angle > 0)
        {
            v_r = -speed;
            v_l = +speed;
        }
        else
        {
            v_r = +speed;
            v_l = -speed;
        }
        set_current(v_r , v_l, v_r , v_l);
        osDelay(1);
    }
    stop();
}

void turn2(int speed, fp32 angle)
{
    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = get_yaw;


    for (int i = 0;; i++)
    {
        go_yaw = get_yaw;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 20)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(angle) < rex_abs(angle_sum))
            break;
        if (angle > 0)
        {
            v_r = -speed;
            v_l = +speed;
        }
        else
        {
            v_r = +speed;
            v_l = -speed;
        }
        set_current(v_r, v_l, v_r, v_l);
        osDelay(1);
    }
    stop();
}


void stop(void)
{
    set_current(0, 0, 0, 0);
}


void set0rount()
{
    intia_rount_cnt = get_rount_cnt;
}


void downplat()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
}


void gobri()
{
    if (data_storage[14] != 0 && data_storage[13] == 0)
    {
        set_current(1800, 1000, 1800, 1000);
        osDelay(130);
    }
    else if (data_storage[14] == 0 && data_storage[13] != 0)
    {
        set_current(1000, 1800, 1000, 1800);
        osDelay(130);
    }
    else
    {
        set_current(2000, 2000, 2000, 2000);

        osDelay(130);
    }
}


void goqqb()
{
    if (data_storage[14] == 0 && data_storage[13] != 0)
    {
        set_current(3000, 1000, 3000, 1000);
        osDelay(30);
    }
    else if (data_storage[14] != 0 && data_storage[13] == 0)
    {
        set_current(1000, 3000, 1000, 3000);
        osDelay(30);
    }
    else
    {
        //				set_current(3000,3000,3000,3000);
        set_current(3000, 3000, 2000, 2000);

        osDelay(30);
    }
}

void set_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    set_speed_rpm[0] = motor1;
    set_speed_rpm[1] = motor2;
    set_speed_rpm[2] = motor3;
    set_speed_rpm[3] = motor4; ////*0.8173
    chassis_set_motor_rpm(set_speed_rpm);
}

void wave_hand(void)
{
    servo_pwm_set(2500, 3);
    osDelay(200);
    servo_pwm_set(500, 1);
    osDelay(200);
    servo_pwm_set(500, 3);
    osDelay(200);
    servo_pwm_set(2500, 1);

}

void wave_handc78(void )
{
    servo_pwm_set(500, 3);
    servo_pwm_set(2500, 1);
    osDelay(200);
}

void up_hand(void)
{
    servo_pwm_set(2500, 3);
    servo_pwm_set(500, 1);
    osDelay(200);


}



void yuyin(int a)
{
    voice_module_play((voice_track_t)a);
}

void gomen13()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(320);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6000, -1500, 6000);
            osDelay(1);
        }
    }

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(85) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(80);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(1000, 5000, 1000, 5000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(85) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 10)
            trackxian3();
        while (get_roll < 15)
            trackxian5();

        while (data_storage[12] != 0)
            setspeed2(3000, 3000);
        osDelay(150);

        setspeed2(-2000, -2000);
        osDelay(200);
        stop();
        osDelay(1000);
        turn_run(5000, 165);
        stop();
        osDelay(100);
    }
}

void gomen23()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-500, 6500, -500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
}

void gomen33()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(94) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(84) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
}
void gomen45niu()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);
    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);
    // set0rount();
    // while(get_rount_cnt-intia_rount_cnt<20)
    // 	setspeed2(2500,2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(5000, 5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(1.0f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    osDelay(1000);
    baojian1 = 2;
  turn_run(5000, 175);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(500);
    wave_hand();
    yuyin(baojian1+1);
    osDelay(500);
    wave_hand();
}
void gomen45()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.8f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    // while(get_rount_cnt-intia_rount_cnt<20)
    // 	setspeed2(2500,2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.2f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-100, -100);
    osDelay(500);
    stop();
    baojian1 = digit_read();
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
    yuyin(baojian1+1);
    osDelay(500);
    wave_hand();
}

void gomen1men2()
{
    turn_run(5000, 175);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);
    while (!(lukou_detect(5)))
        trackxian(0.5f);

    stop();

    osDelay(1000);
}

void gomen2men3()
{
  turn_run(5000, 175);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.7f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        setspeed2(6000, 6000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.7f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(129) < rex_abs(angle_sum))
                break;
            set_current(7000, -1500, 7000, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.7f);
    while (!(lukou_detect(5)))
        trackxian(0.5f);

    stop();

    osDelay(1000);
}
void go12xin()
{
     yuyin(15);
    osDelay(100);
    wave_hand();
    osDelay(200);

    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000, 3000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);
set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        setspeed2(3000, 3000);
    while (get_roll > -5)
        gobri();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(3000, 3000);

    while (get_roll < 15)
        trackxian(0.5f);

    while (data_storage[12] != 0)
        trackxian(0.3f);

    setspeed2(2500, 2500);
    osDelay(400);
    // set0rount();
    // while (get_rount_cnt - intia_rount_cnt >-8 )
    //     setspeed2(-500, -500);
     stop();


    turn_run(5000, 174);
    stop();


    osDelay(500);

    yuyin(16);
    osDelay(150);
    wave_hand();
}
void go12()
{
    yuyin(15);
    osDelay(100);
    wave_hand();
    osDelay(200);

    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000, 3000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);
set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        setspeed2(3000, 3000);
    while (get_roll > -5)
        gobri();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(3000, 3000);

    while (get_roll < 15)
        trackxian(0.5f);

    while (data_storage[12] != 0)
        trackxian(0.3f);

    setspeed2(2500, 2500);
    osDelay(400);

     stop();
    qr_scan(qr);

    turn_run(5000, 165);
    stop();


    osDelay(800);

    yuyin(16);
    osDelay(150);
    wave_hand();

    osDelay(150);
}
void go1men47()
{
    yuyin(15);
    osDelay(100);
    up_hand();
    osDelay(500);
    wave_handc78();
     downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.4f);
    stop();
    osDelay(20);
    turn_run(5000, 21);
    osDelay(20);

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000,3000);




    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.4f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.4f);

    while (hui[8] != 0||hui[9]!=0)
        trackxian(0.4f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000,3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.8f);

    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 || hui[7] != 0 )
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.9f);

    while (hui[8] != 0 ||hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(79) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian(1.1f);

    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.3f);
    while (data_storage[12] != 0)
        trackxian(0.3f);
    setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    turn_run(5000, -174);
    stop();
    osDelay(150);
    yuyin(21);
    osDelay(150);
    wave_hand();



}

void go2men4()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.4f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(27) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    // osDelay(20);
    // turn_run(5000, -26);
    // osDelay(20);
    // set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.3f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 22)
         setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(23) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.8f);
    while (!(lukou_detect(5)))
        trackxian(0.5f);

    stop();

    osDelay(100);
}

void gomen4men3()
{
  turn_run(5000, 175);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[9] != 0 && hui[8] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7000, -1500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);
    while (!(lukou_detect(5)))
        trackxian(0.6f);

    stop();

    osDelay(1000);
}

void gomen215()
{
}

void go2men15()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 58)
        trackxian(0.4f);

    osDelay(20);
    turn_run(5000, -24);
    osDelay(20);
    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.2f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);
    while (hui[9] != 0 && hui[10] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);


    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);
    set0rount();
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(69) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}

void go2men25()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 58)
        trackxian(0.4f);

    osDelay(20);
    turn_run(5000, -24);
    osDelay(20);
    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.2f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);
    set0rount();
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7000, -1500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian(1.1f);
    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}

void go2men35()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);
    osDelay(20);
    turn_run(5000, -24);
    osDelay(20);
    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 4)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(7000, -1500, 7000, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian(0.8f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(95) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}
void go2men47()
{
     downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        trackxian(0.3f);
    while (hui[2] != 0 && hui[3] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(27) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    // osDelay(20);
    // turn_run(5000, -26);
    // osDelay(20);
    // set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[10] != 0 || hui[9] != 0)
        trackxian(0.5f);
    // set0rount();
    // while (get_rount_cnt - intia_rount_cnt < 10)
    //     setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(23) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000,3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.8f);

    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 || hui[7] != 0 )
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.9f);

    while (hui[8] != 0 ||hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(79) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian(1.1f);

    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.3f);
    while (data_storage[12] != 0)
        trackxian(0.3f);
    setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    turn_run(5000, -172);
    stop();
    osDelay(150);
    yuyin(21);
    osDelay(150);
    wave_hand();


}

void go2men45niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        trackxian(0.3f);
    while (hui[2] != 0 && hui[3] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(27) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    // osDelay(20);
    // turn_run(5000, -26);
    // osDelay(20);
    // set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[10] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
while (get_rount_cnt - intia_rount_cnt < 30)
    trackxian(0.8f);
while (hui[6] != 0 && hui[7] != 0)
    trackxian(1.2f);
set0rount();
while (get_rount_cnt - intia_rount_cnt < 20)
    setspeed2(5000, 5000);

set0rount();
while (get_rount_cnt - intia_rount_cnt < 110)
    trackxian(1.2f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}
void go2men45()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        trackxian(0.3f);
    while (hui[2] != 0 && hui[3] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(27) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    // osDelay(20);
    // turn_run(5000, -26);
    // osDelay(20);
    // set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[10] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}


void go2men1()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 58)
        trackxian(0.4f);

    osDelay(20);
    turn_run(5000, -24);
    osDelay(20);
    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.2f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.5f);
    while (hui[9] != 0 && hui[10] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.5f);

    while (!(lukou_detect(5)))
        trackxian(0.5f);

    stop();

    osDelay(1000);
}

void go26()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_roll < 15)
        trackxian3();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (!(lukou_detect(5)))
        trackxian5();

    osDelay(50);
    stop();
    osDelay(500);
    if (zsp == 0x32)
    {
        gomen14();
    }
    else
    {
        turn_run(5000, 165);
        stop();
        osDelay(150);
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (hui[14] != 0 && hui[15] != 0)
            trackxian5();

        {
            fp32 last_angle = 0;
            angle_sum = 0;
            last_angle = get_yaw;


            for (int i = 0;; i++)
            {
                go_yaw = get_yaw;
                go_yaw_inia = last_angle - go_yaw;
                if (rex_abs(go_yaw_inia) < 20)
                    angle_sum += go_yaw_inia;
                last_angle = go_yaw;
                if (rex_abs(125) < rex_abs(angle_sum))
                    break;
                set_current(6500, -1000, 6500, -1000);
                osDelay(1);
            }
        }
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (!(lukou_detect(5)))
            trackxian5();

        osDelay(50);
        stop();
        osDelay(500);
        if (zsp == 0x32)
        {
            gomen24();
        }
        else
        {
            turn_run(5000, 165);
            stop();
            osDelay(150);
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (hui[14] != 0 && hui[15] != 0)
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = get_yaw;


                for (int i = 0;; i++)
                {
                    go_yaw = get_yaw;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(140) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1000, 6500, -1000);
                    osDelay(1);
                }
            }

            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 50)
                trackxian5();
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 140)
                trackxian8();

            while (!(lukou_detect(5)))
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = get_yaw;


                for (int i = 0;; i++)
                {
                    go_yaw = get_yaw;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(135) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1000, 6500, -1000);
                    osDelay(1);
                }
            }
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (!(lukou_detect(5)))
                trackxian5();

            osDelay(50);
            stop();
            osDelay(500);
            if (zsp == 0x32)
            {
                gomen34();
            }
            else
            {
                gomen46();
            }
        }
    }
}

void gomen14()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(320);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6000, -1500, 6000);
            osDelay(1);
        }
    }

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(85) < rex_abs(angle_sum))
                break;
            set_current(8500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(30);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(115) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);


    stop();
}

void gomen24()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(45);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);


    stop();
}


void gomen34()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(45);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(94) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(55) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(45);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(100) < rex_abs(angle_sum))
                break;
            set_current(7000, -2000, 7000, -2000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);


    stop();
}

void gomen46()
{
    turn_run(5000, 165);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (!(lukou_detect(5)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(125) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(5);
    osDelay(150);
    wave_hand();
}

void go67()
{
    downplat();
    servo_pwm_set(700, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        goqqb();
    stop();
    osDelay(2500);
    while (hui[6] != 0 && hui[7] != 0)
    {
        set_current(-5000, 5000, -5000, 5000);
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    servo_pwm_set(2000, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();
    servo_pwm_set(700, 0);
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian3();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(6);
    osDelay(150);
    wave_hand();
}

void go68()
{
    downplat();
    servo_pwm_set(700, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        goqqb();
    stop();
    osDelay(2500);
    while (hui[6] != 0 && hui[7] != 0)
    {
        set_current(-5000, 5000, -5000, 5000);
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(120) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(150) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian3();
    while (data_storage[12] != 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(150);
    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(7);
    osDelay(150);
    wave_hand();
}

void go76()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    servo_pwm_set(2000, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    servo_pwm_set(700, 0);
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(5);
    osDelay(150);
    wave_hand();
}

void go6men11()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        trackxian5();
    //1
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();


    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian3();

    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 85)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian3();
    STOP;
    while (hui[7] != 0 && hui[6] != 0)
    {
        set_current(-5000, 5000, -5000, 5000);
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}

void go6men21()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        goqqb();
    stop();
    osDelay(2500);
    while (hui[7] != 0 && hui[6] != 0)
    {
        set_current(-5000, 5000, -5000, 5000);
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        setspeed2(3000, 3000);

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}

void go6men31()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        goqqb();
    stop();
    osDelay(2500);
    while (hui[7] != 0 && hui[6] != 0)
    {
        set_current(-5000, 5000, -5000, 5000);
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}

void go6men41()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        goqqb();
    stop();
    osDelay(2500);
    while (hui[7] != 0 && hui[6] != 0)
    {
        set_current(-5000, 5000, -5000, 5000);
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}

void go7men11()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    servo_pwm_set(2000, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    servo_pwm_set(700, 0);
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go7men21()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go7men31()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go7men41()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 340)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go5men11()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}

void go5men21()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}

void go5men31()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian2();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(400);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_roll < 15)
        trackxian3();
    while (hui[7] == 0 || hui[8] == 0 || hui[6] == 0 || hui[9] == 0 || hui[5] == 0 || hui[10] == 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(700);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(9);
    osDelay(150);
}
void go5men44niuniu()
{
     downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.1f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);

    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.3f);

    {
         fp32 last_angle = 0;
         angle_sum = 0;
         last_angle = get_yaw;


         for (int i = 0;; i++)
         {
             go_yaw = get_yaw;
             go_yaw_inia = last_angle - go_yaw;
             if (rex_abs(go_yaw_inia) < 45)
                 angle_sum += go_yaw_inia;
             last_angle = go_yaw;
             if (rex_abs(80) < rex_abs(angle_sum))
                 break;
             set_current(7000, -500, 7000, -500);
             osDelay(1);
         }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    ////////////
    stop();
    osDelay(100);
    yuyin(23);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt >-30)
        setspeed2(-2000, -2000);
    stop();
    osDelay(50);
    turn_run(3000, -89);

    ////////////
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.3f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian(1.0f);
    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 38)
        setspeed2(4500,4500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    while (get_roll < 15)
        trackxian(0.6f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    osDelay(100);
  turn_run(5000, -175);
    stop();
    osDelay(100);
    yuyin(18);
    osDelay(150);
    wave_hand();
}
void go5men44niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.1f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.1f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 38)
        setspeed2(4500,4500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.1f);
    while (get_roll < 15)
        trackxian(0.6f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt > -5)
        setspeed2(-800,-800);
    stop();
    osDelay(100);
  turn_run(5000, 175);
    stop();
    osDelay(100);
    yuyin(18);
    osDelay(150);
    wave_hand();

}
void go5men44()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        setspeed2(4000,4000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-300, -300);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(18);
    osDelay(150);
    wave_hand();

}
void go5men41()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);

    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 18)
        setspeed2(3000,3000);
    {fp32 last_angle=0;
        angle_sum =0;
        last_angle=get_yaw;


        for(int i =0;;i++){
            go_yaw =get_yaw;
            go_yaw_inia = last_angle-go_yaw;
            if(rex_abs(go_yaw_inia)<45)
                angle_sum+=go_yaw_inia;
            last_angle = go_yaw;
            if(rex_abs(35)<rex_abs(angle_sum))
                break;
            set_current(7000,500,7000,500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();
}
void go21niu()
{
    downplat();
    set0rount();
    while(hui[1] != 0 )
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000,3000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        setspeed2(3000, 3000);
    while (get_roll > -5)
        gobri();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    while(hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000, 3000);

    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();
}
void go12xinniu()
{
    yuyin(15);
    osDelay(100);
    wave_hand();
    osDelay(200);
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    set0rount();
    while(hui[14] != 0 )
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(3000,3000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        setspeed2(2000, 2000);
    while (get_roll > -5)
        gobri();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000, 3000);

    while (get_roll < 15)
        trackxian(0.5f);

    while (data_storage[12] != 0)
        trackxian(0.3f);

    setspeed2(2500, 2500);
    osDelay(400);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    turn_run(5000, -175);
    stop();
    osDelay(200);
    yuyin(16);
    osDelay(150);
    wave_hand();
}
void go12niu()
{
    yuyin(15);
    osDelay(100);
    up_hand();
    osDelay(500);
    wave_handc78();
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 57)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 18)
        setspeed2(3500,3500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 64)
        trackxian(0.5f);

    while (get_roll > -5)
        gobri();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 47)
        trackxian(0.6f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 16)
        setspeed2(3000, 3000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.6f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 39)
        trackxian(0.3f);

   while (get_roll < 15)
        trackxian(0.3f);

    while (data_storage[12] != 0)
        trackxian(0.4f);

    setspeed2(2500, 2500);
    osDelay(400);
    stop();
    setspeed2(-500, -500);
    osDelay(1000);
    stop();
    turn_run(5000, 175);
    stop();
    osDelay(200);
    yuyin(16);
    osDelay(150);
    wave_hand();
}
void go21()
{

    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000, 3000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        setspeed2(3000, 3000);
    while (get_roll > -5)
        gobri();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(3000, 3000);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();
}

void go31()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    while (hui[10] != 0 && hui[9] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 45)
        setspeed2(4500,4500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.7f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);
    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 22)
        setspeed2(3000,3000);
    {fp32 last_angle=0;
        angle_sum =0;
        last_angle=get_yaw;


        for(int i =0;;i++){
            go_yaw =get_yaw;
            go_yaw_inia = last_angle-go_yaw;
            if(rex_abs(go_yaw_inia)<45)
                angle_sum+=go_yaw_inia;
            last_angle = go_yaw;
            if(rex_abs(32)<rex_abs(angle_sum))
                break;
            set_current(500,7000,500,7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);

    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000,3000);
    {fp32 last_angle=0;
        angle_sum =0;
        last_angle=get_yaw;


        for(int i =0;;i++){
            go_yaw =get_yaw;
            go_yaw_inia = last_angle-go_yaw;
            if(rex_abs(go_yaw_inia)<45)
                angle_sum+=go_yaw_inia;
            last_angle = go_yaw;
            if(rex_abs(32)<rex_abs(angle_sum))
                break;
            set_current(7000,500,7000,500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 163);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();
}
void go41()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 37)
        setspeed2(4000,4000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 155)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4000,4000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);

     while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);

    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 18)
        setspeed2(3000,3000);
     {fp32 last_angle=0;
         angle_sum =0;
         last_angle=get_yaw;


         for(int i =0;;i++){
             go_yaw =get_yaw;
             go_yaw_inia = last_angle-go_yaw;
             if(rex_abs(go_yaw_inia)<45)
                 angle_sum+=go_yaw_inia;
             last_angle = go_yaw;
             if(rex_abs(35)<rex_abs(angle_sum))
                 break;
             set_current(7000,500,7000,500);
             osDelay(1);
         }
     }


     set0rount();
     while (get_rount_cnt - intia_rount_cnt < 10)
         trackxian(0.3f);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();
}
void go41niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);
    while(hui[4] != 0 && hui[5] != 0)
        trackxian(1.1f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 38)
        setspeed2(4500,4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(1.1f);
    while(hui[8] != 0 && hui[9] != 0)
        trackxian(1.1f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4500,4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);

     while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);

    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000,3000);
     {fp32 last_angle=0;
         angle_sum =0;
         last_angle=get_yaw;


         for(int i =0;;i++){
             go_yaw =get_yaw;
             go_yaw_inia = last_angle-go_yaw;
             if(rex_abs(go_yaw_inia)<45)
                 angle_sum+=go_yaw_inia;
             last_angle = go_yaw;
             if(rex_abs(35)<rex_abs(angle_sum))
                 break;
             set_current(7000,500,7000,500);
             osDelay(1);
         }
     }


     set0rount();
     while (get_rount_cnt - intia_rount_cnt < 10)
         trackxian(0.3f);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
  turn_run(5000, 175);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();
}
void go75niu()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.1f);
     set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian(1.1f);
    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.7f);
    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.4f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.7f);
    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.7f);
    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(3000,3000);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);
    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(5000, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(1.0f);


    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    osDelay(200);
    up_hand();
    osDelay(200);
    yuyin(25);
    osDelay(300);
    turn_run(5000, 352);
    stop();
    wave_handc78();
    osDelay(500);
  turn_run(5000, 175);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}
void go75()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.1f);
     set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian(1.0f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.7f);
    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.7f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.7f);
    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(3000,3000);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-1000, -1000);
    osDelay(200);
    stop();
    osDelay(500);
    up_hand();
    osDelay(500);
    yuyin(25);
    osDelay(400);
    turn_run(5000, 345);
    stop();
    wave_handc78();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}

void go7men12()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    servo_pwm_set(700, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    servo_pwm_set(2000, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    servo_pwm_set(700, 0);
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go7men13()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    servo_pwm_set(2000, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    servo_pwm_set(700, 0);
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(150);
    turn_run(5000, 167);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go7men14()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    servo_pwm_set(700, 0);
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go7men22()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go7men23()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(45) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go7men24()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go7men34()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go7men33()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 450)
        trackxian8();
    set0rount();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go7men32()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go7men44()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go7men43()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    set0rount();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go7men42()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void gomen15()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);


    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);
    set0rount();
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(69) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}

void gomen3men45()
{
    turn_run(5000, 175);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian(0.8f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(86) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(80);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    //		osDelay(150);
    //	  wave_handc78();
}

void gomen35()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        trackxian(0.5f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10
        )
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(80);
    stop();
    osDelay(1000);
    turn_run(5000, 170);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}

void gomen25()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(80);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}
void ce1()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.5f);

    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        setspeed2(3000,3000);
    {fp32 last_angle=0;
        angle_sum =0;
        last_angle=get_yaw;


        for(int i =0;;i++){
            go_yaw =get_yaw;
            go_yaw_inia = last_angle-go_yaw;
            if(rex_abs(go_yaw_inia)<45)
                angle_sum+=go_yaw_inia;
            last_angle = go_yaw;
            if(rex_abs(35)<rex_abs(angle_sum))
                break;
            set_current(7000,500,7000,500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 175);
    stop();
    osDelay(150);
    yuyin(24);
    osDelay(150);
    wave_hand();



    while (data_storage[12] != 0)
    {
        sensor_update();
        osDelay(50);
    }
    while (data_storage[12] == 0)
    {
        sensor_update();
        osDelay(50);
    }






    go1men47();

}
void ce2()
{

    while (get_roll < 15)
        trackxian(0.4f);

    while (data_storage[12] != 0)
        trackxian(0.3f);

    setspeed2(2500, 2500);
    osDelay(400);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    turn_run(5000, -175);
    stop();
    osDelay(200);
    yuyin(16);
    osDelay(150);
    wave_hand();




    go24niu();


}

void ce3()
{
    go12niu();
    go2men4();
    gomen4men3();
    //tl_scan();
    gomen35();
    go57niu();
    STOP
    //go75niu();

    go5men3jia();
    //go5men44niuniu();
    //go43niu();
    //go31();
    while (data_storage[12] != 0)
    {
        sensor_update();
        osDelay(50);
    }
    while (data_storage[12] == 0)
    {
        sensor_update();
        osDelay(50);
    }
   go1men47();
    go75niu();
    go5men44niuniu();
    go43niu();
    go31();
}

void ce4()
{
    // go12niu();
    //  go2men4();
    //  tl_scan();
    //  gomen45niu();
    //  go57niu();
    //  go75niu();
    //  go5men44niu();
    //  go43niu();
    //  go31();
    //  while (data_storage[12] != 0)
    //  {
    //      sensor_update();
    //      osDelay(50);
    //  }
    //  while (data_storage[12] == 0)
    //  {
    //      sensor_update();
    //      osDelay(50);
    //  }
    //  go12xinniu();
    go2men47();

    go75niu();
    go5men44niu();
    go43niu();
    go31();
}
void
go24niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.4f);
    set0rount();
    {
        fp32 last_angle = 0;
        angle_sum =0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(27) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }

    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.2f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.2f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        setspeed2(3000, 3000);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(26) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 3)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.4f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(3000, 3000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    while(hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        setspeed2(5000,5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(1.0f);
    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
    osDelay(100);
    turn_run(5000, -175);
    stop();
    osDelay(100);
    yuyin(18);
    osDelay(150);
    wave_hand();
}

void go25()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian5();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_roll < 15)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();


    //
    //  while(get_roll < 15)
    //		trackxian3();
    //
    //	intia_rount_cnt=get_rount_cnt;
    //	while(get_rount_cnt-intia_rount_cnt<80)
    //		trackxian2();
    //	set0rount();
    //	while(get_rount_cnt-intia_rount_cnt<10)
    //		trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_hand();
}

void go36()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(86) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(170);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(55) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(230);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -2500, 6500, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (hui[1] == 0 || hui[14] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui
        [7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0)
        trackxian3();

    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 86)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    //while(hui[0] != 0 && hui[1] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(20)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(-1500,6500,-1500,6500);
    //		 osDelay(1);
    //	 }
    // }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(30);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
}

void go35()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(40);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 5)
        trackxian5();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
}

void go46()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(86) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    while (get_roll < 15)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian2();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(35);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(76) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(230);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -2500, 6500, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (hui[1] == 0 || hui[14] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui
        [7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0)
        trackxian3();

    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 4);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 86)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    //while(hui[0] != 0 && hui[1] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(20)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(-1500,6500,-1500,6500);
    //		 osDelay(1);
    //	 }
    // }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(30);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
}

void go45()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(86) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    while (get_roll < 15)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian2();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(35);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(76) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();

    while (get_roll < 15)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
}
void go32()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    while (hui[10] != 0 && hui[9] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 45)
        setspeed2(4500,4500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.7f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);
    while(hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 22)
        setspeed2(3000,3000);
    {fp32 last_angle=0;
        angle_sum =0;
        last_angle=get_yaw;


        for(int i =0;;i++){
            go_yaw =get_yaw;
            go_yaw_inia = last_angle-go_yaw;
            if(rex_abs(go_yaw_inia)<45)
                angle_sum+=go_yaw_inia;
            last_angle = go_yaw;
            if(rex_abs(130)<rex_abs(angle_sum))
                break;
            set_current(-1500,6500,-1500,6500);
            osDelay(1);
        }
    }
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);

     while(hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 17)
        setspeed2(3000,3000);
    {fp32 last_angle=0;
        angle_sum =0;
        last_angle=get_yaw;


        for(int i =0;;i++){
            go_yaw =get_yaw;
            go_yaw_inia = last_angle-go_yaw;
            if(rex_abs(go_yaw_inia)<45)
                angle_sum+=go_yaw_inia;
            last_angle = go_yaw;
            if(rex_abs(32)<rex_abs(angle_sum))
                break;
            set_current(500,7000,500,7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);


    while (get_roll < 15)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 26)
        setspeed2(2000, 2000);

    stop();
    osDelay(150);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(16);
    osDelay(150);
    wave_hand();



}
void go43()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 37)
        setspeed2(4000,4000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 155)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4000,4000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 125)
        trackxian(0.9f);
  set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4000,4000);
   set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.9f);

      set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        setspeed2(4000,4000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 25)
        trackxian(0.7f);
   set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);

    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-300, -300);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(17);
    osDelay(150);
    wave_hand();


}
void go43niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    while(hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(5000,5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    while(hui[8] != 0 && hui[9] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(5000,5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(1.0f);
    while(hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(5000,5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    while(hui[8] != 0 && hui[9] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        setspeed2(4500,4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);



    while (get_roll < 15)
        trackxian(0.6f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(800);
    stop();
  turn_run(5000, 195);
    stop();
    osDelay(100);
    yuyin(17);
    osDelay(150);
    wave_hand();

}
void go325()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(150);
    wave_handc78();
}

void go335()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 430)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(150);
    wave_handc78();
}

void go345()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 430)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(500);
    yuyin(4);
    osDelay(200);
    wave_handc78();
}


void go3men16()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go3men26()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go3men36()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go3men46()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void gomen1jia()
{
    go3men15();
    go517();
    go78();
    go86();
    go6men1jia();
}


void go3men15()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (!(lukou_detect(4)))
        trackxian8();
    osDelay(100);
    stop();
    osDelay(700);

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    stop();
    osDelay(50);

    //	while(data_storage[12] != 0)
    //			trackxian5();


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[15] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, 75);
    stop();
    osDelay(100);
    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    stop();
    osDelay(100);

    //	while(data_storage[12] != 0)
    //			trackxian5();


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[15] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(150);

    while (!lukou_detect(5))
        trackxian5();


    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian10();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);
}

void go517()
{
    downplat();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
        stop();
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(95) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui[14] != 0 &&
        hui[15] != 0)
        set_current(-1000, 8000, -1000, 8000);

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while (get_roll < 15)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();
    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();
    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian8();
    while (get_roll < 15)
        trackxian5();


    gohill();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();

    while (!(lukou_detect(5)))
        trackxian5();

    turnright();

    while (!(lukou_detect(5)))
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();

    while (data_storage[12] != 0)
        trackxian3();

    //servo_pwm_set(1750, 2);

    setspeed2(3000, 3000);
    osDelay(500);
    setspeed2(-2000, -2000);
    osDelay(250);
    stop();
    osDelay(50);
    ////servo_pwm_set(2300, 0);
    osDelay(150);
    //servo_pwm_set(700, 1);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(166) < rex_abs(angle_sum))
                break;
            set_current(-5000, 5000, -5000, 5000);
            osDelay(1);
        }
        stop();
    }

    stop();
    osDelay(50);

    yuyin(6);
}

void go78()
{
    //servo_pwm_set(700, 0);
    //servo_pwm_set(2300, 1);
    //servo_pwm_set(1300, 2);
    while (get_roll > -5)
        set_current(3000, 3000, 3000, 3000);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(3000, 3000, 3000, 3000);
    osDelay(100);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(3000, 3000, 3000, 3000);

    while (get_roll < -5)
        trackxian5();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();


    while (get_roll < 15)
        trackxian5();

    gohill();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_roll < 15)
        trackxian8();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian5();
    while (data_storage[12] != 0)
        trackxian3();

    //servo_pwm_set(1750, 2);

    setspeed2(3000, 3000);
    osDelay(500);
    setspeed2(-2000, -2000);
    osDelay(240);
    stop();
    osDelay(50);
    //servo_pwm_set(2300, 0);
    osDelay(150);
    //servo_pwm_set(700, 1);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(166) < rex_abs(angle_sum))
                break;
            set_current(-5000, 5000, -5000, 5000);
            osDelay(1);
        }
        stop();
    }

    stop();
    osDelay(100);
    yuyin(7);
}

void go6men1jia()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();


    while (hui[1] == 0 || hui[14] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui
        [7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0)
        trackxian3();

    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 3);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(120);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(50);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(93) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(115) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(95) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(170);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(56) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    stop();
}

void xingomen1jia()
{
    downplat();
    intia_rount_cnt = get_rount_cnt;

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, 80);
    stop();
    osDelay(150);

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);

    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(200);

    turn_run(5000, -70);
    stop();
    osDelay(150);

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);

    go517();
    go78();
    go86();
    go6men1jia();
}

void go3men25()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(28) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (!lukou_detect(5))
        trackxian8();

    stop();
    osDelay(500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);
}
void go57niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(1.0f);


    while (hui[8] != 0 || hui[9] != 0 )
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 || hui[7] != 0 )
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(79) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.9f);

    while (hui[6] != 0 || hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 ||hui[7] != 0 )
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(1.1f);

    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 || hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(79) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian(1.1f);

    while (get_roll < 15)
        trackxian(0.6f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.3f);
    while (data_storage[12] != 0)
        trackxian(0.3f);
    setspeed2(2500, 2500);
    osDelay(450);
    stop();
    setspeed2(-500, -500);
    osDelay(600);
    stop();
    baojian2 = 3;
    turn_run(5000, -174);
    stop();
    osDelay(150);
    yuyin(21);
    osDelay(500);
    wave_hand();
        yuyin( baojian2+8);
    osDelay(150);
    wave_hand();

}
void go57()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);


    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.7f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 && hui[7] != 0 && hui[5] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.7f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);

    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.3f);
    while (data_storage[12] != 0)
        trackxian(0.3f);
    setspeed2(2500, 2500);
    osDelay(450);
    setspeed2(-100, -100);
    osDelay(500);
    stop();
    baojian2 = digit_read();
    turn_run(5000, 167);
    stop();
    osDelay(150);
    yuyin(21);
    osDelay(150);
    wave_hand();
    yuyin( baojian2+8);
    osDelay(150);
    wave_hand();

}
void go57xin()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);


    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.7f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 && hui[7] != 0 && hui[5] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.7f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.0f);

    while (get_roll < 15)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.3f);
    while (data_storage[12] != 0)
        trackxian(0.3f);
    setspeed2(2500, 2500);
    osDelay(450);
    setspeed2(-100, -100);
    osDelay(500);
    stop();
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(21);
    osDelay(150);
    wave_hand();
}
void go58()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(0.8f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        setspeed2(4500, 4500);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    servo_pwm_set(1300,0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian(1.0f);

    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    servo_pwm_set(500,0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(0.7f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 32)
        setspeed2(5000, 5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 62)
        trackxian(0.5f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.5f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.7f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(120) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.8f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(7000, -1500, 7000, -1500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian(0.3f);

    while (data_storage[12] != 0)
        trackxian(0.3f);

    setspeed2(3000, 3000);
    osDelay(450);
    setspeed2(-2000, -2000);
    osDelay(80);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(22);
    osDelay(150);
    wave_hand();
}

void go86()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (!(lukou_detect(4)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(145) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 3000, -1500, 3000);
            osDelay(1);
        }
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 75)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 168);
    stop();
    osDelay(150);
    yuyin(5);
    osDelay(150);
    wave_hand();
}

void go85()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(145) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(150);
    wave_handc78();
}

void go8men14()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men13()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men12()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go8men24()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men23()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(45) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men22()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go8men34()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men33()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 450)
        trackxian8();
    set0rount();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men32()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}


void go8men44()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men43()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    set0rount();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men42()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void gomen2jia()
{
    go3men25();
    go57();
    go78();
    go86();
    go6men2jia();
}


void go6men2jia()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();


    while (hui[1] == 0 || hui[14] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui
        [7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0)
        trackxian3();

    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 3);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(120);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(50);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(93) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    stop();
}

void xingomen2jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(26) < rex_abs(angle_sum))
                break;
            set_current(7000, 1000, 7000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();


    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 340)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);


    go57();
    go78();
    go86();
    go6men2jia();
}


void go4men4niu()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    while(hui[6] != 0 && hui[7] != 0)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        setspeed2(5000,5000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 145)
        trackxian(1.0f);
     while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian(0.8f);
    while (!(lukou_detect(5)))
        trackxian(0.5f);

    stop();

    osDelay(100);

}

void go6men3jia()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 15)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();


    while (hui[1] == 0 || hui[14] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui
        [7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0)
        trackxian3();

    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 3);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(120);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6000, -1500, 6000, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6000, -1500, 6000, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-500, 7000, -500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 45)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(98) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(80);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    stop();


    // set0rount();
    //	while(get_rount_cnt-intia_rount_cnt<20)
    //		trackxian3();
    //

    //
    //	while(hui[14] != 0 && hui[15] != 0)
    //		trackxian5();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(90)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(7000,-1000,7000,-1000);
    //		 osDelay(1);
    //	 }
    // }

    //
    // //chuyuan
    //
    // 	while(hui[0] != 0 && hui[1] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(135)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(-1000,8000,-1000,8000);
    //		 osDelay(1);
    //	 }
    // }

    //
    //
    // 	while(hui[14] != 0 && hui[15] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(135)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(8000,-1000,8000,-1000);
    //		 osDelay(1);
    //	 }
    // }
    // 	while(hui[0] != 0 && hui[1] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(135)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(-2000,8000,-2000,8000);
    //		 osDelay(1);
    //	 }
    // }
    //
    //	 set0rount();
    //	while(get_rount_cnt-intia_rount_cnt<200)
    //		trackxian5();
    //
    //	while(hui[14] != 0 && hui[15] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(80)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(8000,-1000,8000,-1000);
    //		 osDelay(1);
    //	 }
    // }
    //
    //	 set0rount();
    //	while(get_rount_cnt-intia_rount_cnt<200)
    //		trackxian5();
    //	while(hui[14] != 0 && hui[15] != 0)
    //		trackxian3();
    //
    //	{fp32 last_angle=0;
    //		  angle_sum =0;
    //		 last_angle=get_yaw;

    //
    //			 for(int i =0;;i++){
    //				 go_yaw =get_yaw;
    //				 go_yaw_inia = last_angle-go_yaw;
    //				 if(rex_abs(go_yaw_inia)<20)
    //					 angle_sum+=go_yaw_inia;
    //				 last_angle = go_yaw;
    //				if(rex_abs(80)<rex_abs(angle_sum))
    //			 break;
    //		 set_current(8000,2000,8000,2000);
    //		 osDelay(1);
    //	 }
    // }
    //
    //		while(get_roll<15)
    //			trackxian8();
    //
    //		while(get_roll>5)
    //			trackxian5();
    //
    //
    //		setspeed2(3000,3000);
    //		osDelay(300);
    //		stop();
    //		osDelay(200);
    //
    //		turn_run(5000,165);
    //
    //		yuyin(9);
    //		stop();
    //		osDelay(200);
    //		//servo_pwm_set(1750, 2);
    //
}


void go6men4jia()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = get_yaw;


    for (int i = 0;; i++)
    {
        go_yaw = get_yaw;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(60) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnleft();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);

    //servo_pwm_set(1750, 2);
}

void go5men1jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 95)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(115) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(95) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(250);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(56) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    stop();
}

void gomen2men15()
{
  turn_run(5000, 175);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7000, -1500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian(1.1f);
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7000, -1500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);
    set0rount();
    while (hui[6] != 0 && hui[7] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(69) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian(0.8f);

    while (hui[6] != 0 && hui[9] != 0)
        trackxian(0.5f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        setspeed2(2500, 2500);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(500, 6500, 500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(0.8f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian(1.2f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian(0.8f);
    while (get_roll < 15)
        trackxian(0.5f);
    while (data_storage[12] != 0)
        setspeed2(2500, 2500);
    osDelay(450);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(19);
    osDelay(150);
    wave_hand();
}

void gomen3men2()
{
  turn_run(5000, 175);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.5f);
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(0.3f);
    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(132) < rex_abs(angle_sum))
                break;
            set_current(-2000, 7000, -2000, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.7f);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        setspeed2(6000, 6000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(0.7f);

    while (hui[8] != 0 && hui[9] != 0)
        trackxian(0.5f);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(129) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7000, -1500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian(0.3f);
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian(0.7f);
    while (!(lukou_detect(5)))
        trackxian(0.6f);

    stop();

    osDelay(1000);
}


void gomen2men1()
{
    turn_run(5000, 162);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-2200, 6500, -2200, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(98) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
}


void xingo4men4jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();


    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men43();
    go3jia();
}

void go5men3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    STOP

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 95)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 360)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(58) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(50);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(115) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;

        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(98) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(120);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(7000, -500, 7000, -500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    stop();
}


void xingo4men3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();


    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men3jia();
}


void go5men2jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 95)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(83) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6500, -1000, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(82) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    stop();
}


void xingo3men2jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(28) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();


    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men1jia();
}


void go8zhi()
{
    //servo_pwm_set(1300, 2);
    //servo_pwm_set(1500, 1);
    //servo_pwm_set(1500, 0);
    while (get_roll > -5)
        set_current(3000, 3000, 3000, 3000);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(3000, 3000, 3000, 3000);
    osDelay(100);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(3000, 3000, 3000, 3000);
    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();

    while (get_roll < -5)
        trackxian3();
    downplat();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;

            set_current(-2000, 7000, -2000, 7000);
            osDelay(1);
        }
        stop();
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-2000, 5000, -2000, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;

            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
        stop();
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-1000, 8000, -1000, 8000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnright();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, 75);
    stop();
    osDelay(150);
}


void gozhimen1jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnleft();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian10();

    while (!lukou_detect(4))
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void gozhimen2jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(6))
        trackxian5();

    turnright();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(200);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui[14] != 0 &&
        hui[15] != 0)

        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void gozhimen3jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian8();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }


    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-2000, 5000, -2000, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void gozhimen4jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian8();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(60) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void go4zhi3()
{
    //servo_pwm_set(1300, 2);
    //servo_pwm_set(1500, 1);
    //servo_pwm_set(1500, 0);
    downplat();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 380)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 420)
        trackxian10();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(23) < rex_abs(angle_sum))
                break;
            set_current(10000, 7000, 10000, 7000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian10();


    while (data_storage[12] != 0)
        trackxian8();
    setspeed2(3000, 3000);
    osDelay(400);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 167);

    stop();
    osDelay(100);

    yuyin(8);

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0)
        set_current(5000, -2000, 5000, -2000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian10();

    while (get_roll < 15)
        trackxian8();

    goplat();

    yuyin(2);
}


void gomen3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    stop();
    osDelay(700);

    turn_run(6000, 165);

    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (!lukou_detect(5))
        trackxian8();
    osDelay(100);
    stop();
    osDelay(500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (!lukou_detect(4))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);
    go57();
    go78();
    go86();
    go6men33();
    go3jia();
}


void go6men43()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = get_yaw;


    for (int i = 0;; i++)
    {
        go_yaw = get_yaw;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(60) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnleft();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 460)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 500)
        trackxian10();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(2);
}

void go3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian10();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}

void go6men33()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = get_yaw;


    for (int i = 0;; i++)
    {
        go_yaw = get_yaw;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(28) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 330)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }


    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-2000, 5000, -2000, 5000);
    stop();
    osDelay(50);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 420)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 460)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 500)
        trackxian10();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(2);
}

void xingomen3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(22) < rex_abs(angle_sum))
                break;
            set_current(3000, 8000, 3000, 8000);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();


    while (!lukou_detect(4))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = get_yaw;


        for (int i = 0;; i++)
        {
            go_yaw = get_yaw;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men33();
    go3jia();
}
