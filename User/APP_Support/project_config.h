#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H



#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265358979323846f
/* ========================= 鏈哄櫒浜哄叏灞€閰嶇疆 ========================= */
/* 浠ｇ爜杩愯妯″紡 */
#define debug   0
#define release 1

/* 瓒呯骇鐢靛寮€鍏?*/
#define Cap_off 0X00
#define Cap_on  0X01

#define ROBOT_MODE        debug     // 褰撳墠浠ｇ爜妯″紡锛歞ebug/release
#define ROBOT_CAP         Cap_off   // 褰撳墠瓒呯骇鐢靛鐘舵€?

/* ========================= 浜戝彴瑙掑害 PID 鍙傛暟 ========================= */
/* pitch 杞撮檧铻轰华缁濆瑙掓帶鍒?PID */
#define PITCH_GYRO_ABSOLUTE_PID_KP         4.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI         0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD         0.05f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT    1.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT   0.0f

/* yaw 杞撮檧铻轰华缁濆瑙掓帶鍒?PID */
#define YAW_GYRO_ABSOLUTE_PID_KP           5.0f
#define YAW_GYRO_ABSOLUTE_PID_KI           0.01f
#define YAW_GYRO_ABSOLUTE_PID_KD           0.74f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT      2.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT     0.0f

/* pitch 杞寸紪鐮佸櫒鐩稿瑙掓帶鍒?PID */
#define PITCH_ENCODE_RELATIVE_PID_KP       8.8f
#define PITCH_ENCODE_RELATIVE_PID_KI       0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD       0.70f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT  2.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

/* yaw 杞寸紪鐮佸櫒鐩稿瑙掓帶鍒?PID */
#define YAW_ENCODE_RELATIVE_PID_KP         1.8f
#define YAW_ENCODE_RELATIVE_PID_KI         0.0f
#define YAW_ENCODE_RELATIVE_PID_KD         0.2f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT    0.8f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT   0.0f

/* ========================= 浜戝彴鍓嶉涓庤緭鍑洪厤缃?========================= */
#define YAW_REF_VEL_FILTER_ALPHA           0.05f  // yaw 鐩爣瑙掑樊鍒嗛€熷害浣庨€氱郴鏁帮紝瓒婂皬鍓嶉瓒婂钩婊?
#define YAW_REF_ACCEL_LIMIT                100.0f // yaw 鎯噺鍓嶉鍙傝€冨姞閫熷害闄愬箙锛岄槻姝㈤仴鎺ц緭鍏ヨ烦鍙樹骇鐢熷姏鐭╁皷宄?
#define PITCH_RELATIVE_SPEED_FILTER_ALPHA  0.20f  // pitch 缂栫爜鍣ㄥ樊鍒嗛€熷害浣庨€氱郴鏁?
#define PITCH_VELOCITY_FF_GAIN             0.1f  // pitch 閫熷害鍓嶉绯绘暟锛屽崟浣?N*m/(rad/s)

/* 鎯噺鍓嶉锛歵orque_ff = J * alpha_ref */
#define YAW_INERTIA_KGM2                   0.013  // yaw 杞姩鎯噺 J锛屽崟浣?kg*m^2

#define PITCH_EQ_MASS_KG                   1.5f   // pitch 閲嶅姏琛ュ伩浣跨敤鐨勭瓑鏁堣川閲?
#define PITCH_INERTIA_KGM2                 0.00245 // pitch 杞姩鎯噺 J锛屽崟浣?kg*m^2

/* ========================= 閬ユ帶鍣?榧犳爣杈撳叆閰嶇疆 ========================= */
#define GIMBAL_ANGLE_Z_RC_SEN              0.0000005f // 灏忛檧铻?搴曠洏鏃嬭浆瑙掗€熷害杈撳叆鐏垫晱搴?
#define YAW_CHANNEL                        2          // yaw 閬ユ帶閫氶亾
#define PITCH_CHANNEL                      3          // pitch 閬ユ帶閫氶亾
#define GIMBAL_MODE_CHANNEL                0          // 浜戝彴妯″紡鍒囨崲閫氶亾
#define WZ_CHANNEL                         2          // 搴曠洏鏃嬭浆閫氶亾
#define TURN_KEYBOARD                      KEY_PRESSED_OFFSET_F // 灏忛檧铻烘寜閿?
#define TURN_SPEED                         0.04f      // 灏忛檧铻烘棆杞€熷害
#define TEST_KEYBOARD                      KEY_PRESSED_OFFSET_B // 娴嬭瘯鎸夐敭
#define RC_DEADBAND                        10         // 閬ユ帶鍣ㄦ鍖?
#define YAW_RC_SEN                         -0.000005f // yaw 閬ユ帶鐏垫晱搴?
#define PITCH_RC_SEN                       -0.000006f // pitch 閬ユ帶鐏垫晱搴?
#define YAW_MOUSE_SEN                      0.000006f   // yaw 榧犳爣鐏垫晱搴?
#define PITCH_MOUSE_SEN                    -0.000006f  // pitch 榧犳爣鐏垫晱搴?
#define YAW_ENCODE_SEN                     0.01f      // yaw 缂栫爜鍣ㄦā寮忚緭鍏ョ伒鏁忓害
#define PITCH_ENCODE_SEN                   0.01f      // pitch 缂栫爜鍣ㄦā寮忚緭鍏ョ伒鏁忓害

/* ========================= 浜戝彴浠诲姟涓庡弽棣堢储寮曢厤缃?========================= */
#define GIMBAL_TASK_INIT_TIME              500    // 浜戝彴浠诲姟鍚姩寤舵椂锛屽崟浣?ms
#define GIMBAL_CONTROL_TIME                1      // 浜戝彴鎺у埗鍛ㄦ湡锛屽崟浣?ms
#define INS_YAW_ADDRESS_OFFSET             0      // INS yaw 瑙掓暟缁勭储寮?
#define INS_PITCH_ADDRESS_OFFSET           1      // INS pitch 瑙掓暟缁勭储寮?
#define INS_ROLL_ADDRESS_OFFSET            2      // INS roll 瑙掓暟缁勭储寮?
#define INS_GYRO_X_ADDRESS_OFFSET          0      // INS gyro x 绱㈠紩
#define INS_GYRO_Y_ADDRESS_OFFSET          1      // INS gyro y 绱㈠紩
#define INS_GYRO_Z_ADDRESS_OFFSET          2      // INS gyro z 绱㈠紩
#define GIMBAL_PITCH_MIT_INDEX             1u     // MIT 鐢垫満鍙嶉鏁扮粍涓?pitch 鐢垫満绱㈠紩
#define SHOOT_STRUM_MIT_INDEX              2u     // MIT 鐢垫満鍙嶉鏁扮粍涓嫧寮圭數鏈虹储寮?

/* ========================= strum control params ========================= */
#define SHOOT_STRUM_SINGLE_STEP_RAD        (PI * 1.0f) // click step, rad
#define SHOOT_STRUM_LONG_PRESS_MS          200U        // long press threshold, ms
#define SHOOT_STRUM_DIRECTION              1.0f        // strum direction sign
#define SHOOT_STRUM_FDB_TIMEOUT            100U        // strum feedback timeout, ms

/* ========================= 浜戝彴鏈烘闄愪綅涓庡垵濮嬪寲閰嶇疆 ========================= */
#define YAW_MAX_RELATIVE_ANGLE             1.5707963f  // yaw 鐩稿瑙掍笂闄?
#define YAW_MIN_RELATIVE_ANGLE            -1.5707963f  // yaw 鐩稿瑙掍笅闄?
#define PITCH_MAX_RELATIVE_ANGLE           0.1f        // pitch 杞欢涓婇檺锛屽崟浣?rad
#define PITCH_MIN_RELATIVE_ANGLE          -0.71f       // pitch 杞欢涓嬮檺锛屽崟浣?rad
#define HALF_ECD_RANGE                     4096        // 缂栫爜鍣ㄥ崐閲忕▼
#define ECD_RANGE                          8191        // 缂栫爜鍣ㄦ€婚噺绋?
#define GIMBAL_INIT_ANGLE_ERROR            0.1f        // 鍒濆鍖栫洰鏍囪鍏佽璇樊
#define GIMBAL_INIT_STOP_TIME              100         // 鍒濆鍖栧仠姝㈠垽瀹氭椂闂?
#define GIMBAL_INIT_TIME                   6000        // 鍒濆鍖栨€昏秴鏃舵椂闂?
#define GIMBAL_CALI_REDUNDANT_ANGLE        0.1f        // 鏍″噯鍐椾綑瑙掑害
#define GIMBAL_INIT_PITCH_SPEED            0.004f      // pitch 鍒濆鍖栭€熷害
#define GIMBAL_INIT_YAW_SPEED              0.005f      // yaw 鍒濆鍖栭€熷害
#define GIMBAL_CALI_MOTOR_SET              8000        // 鏍″噯鏃剁數鏈鸿緭鍑?
#define GIMBAL_CALI_STEP_TIME              2000        // 鏍″噯姝ラ鎸佺画鏃堕棿
#define GIMBAL_CALI_GYRO_LIMIT             0.1f        // 鏍″噯闈欐瑙掗€熷害闃堝€?
#define GIMBAL_CALI_PITCH_MAX_STEP         1           // pitch 鏈€澶ц鏍″噯姝ラ
#define GIMBAL_CALI_PITCH_MIN_STEP         2           // pitch 鏈€灏忚鏍″噯姝ラ
#define GIMBAL_CALI_YAW_MAX_STEP           3           // yaw 鏈€澶ц鏍″噯姝ラ
#define GIMBAL_CALI_YAW_MIN_STEP           4           // yaw 鏈€灏忚鏍″噯姝ラ
#define GIMBAL_CALI_START_STEP             GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP               5
#define GIMBAL_MOTIONLESS_RC_DEADLINE      10          // 杩涘叆闈欐琛屼负鐨勯仴鎺ф鍖洪槇鍊?
#define GIMBAL_MOTIONLESS_TIME_MAX         3000        // 闈欐琛屼负鏈€澶т繚鎸佹椂闂?

#define INIT_YAW_SET                       0.0f        // yaw 鍒濆鍖栫洰鏍囪
#define INIT_PITCH_SET                     0.0f        // pitch 鍒濆鍖栫洰鏍囪

/* ========================= pitch 閲嶅姏琛ュ伩閰嶇疆 ========================= */
#define PITCH_GRAVITY_COMP_MASS_KG         PITCH_EQ_MASS_KG // pitch 閲嶅姏琛ュ伩璐ㄩ噺
#define PITCH_GRAVITY_COMP_COM_FORWARD_M   0.04f            // 璐ㄥ績鍓嶅悜璺濈
#define PITCH_GRAVITY_COMP_COM_UP_M        0.02f            // 璐ㄥ績涓婂悜璺濈
#define PITCH_GRAVITY_COMP_OUTPUT_LIMIT    (T_MAX / 5.0f)   // 閲嶅姏琛ュ伩杈撳嚭闄愬箙

/* ========================= shoot bullet speed estimate ========================= */
#ifndef SHOOT_FRIC_WHEEL_RADIUS_M
#define SHOOT_FRIC_WHEEL_RADIUS_M          0.022f
#endif

#ifndef SHOOT_BULLET_42MM_MASS_KG
#define SHOOT_BULLET_42MM_MASS_KG          0.0404f
#endif

#ifndef SHOOT_FRIC_ROTATING_MASS_KG
#define SHOOT_FRIC_ROTATING_MASS_KG        0.1793f
#endif

#ifndef SHOOT_FRIC_ROTATING_INERTIA_KGM2
#define SHOOT_FRIC_ROTATING_INERTIA_KGM2   (0.5f * SHOOT_FRIC_ROTATING_MASS_KG * SHOOT_FRIC_WHEEL_RADIUS_M * SHOOT_FRIC_WHEEL_RADIUS_M)
#endif

#ifndef SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM
#define SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM 600.0f
#endif

#ifndef SHOOT_BULLET_SPEED_EST_MIN_SPEED_RATIO
#define SHOOT_BULLET_SPEED_EST_MIN_SPEED_RATIO  0.85f
#endif

#ifndef SHOOT_BULLET_SPEED_EST_WINDOW_MS
#define SHOOT_BULLET_SPEED_EST_WINDOW_MS   20U
#endif

#ifndef SHOOT_BULLET_SPEED_EST_COEFF_MPS_PER_RPM
#define SHOOT_BULLET_SPEED_EST_COEFF_MPS_PER_RPM \
    ((3.0f * SHOOT_FRIC_ROTATING_INERTIA_KGM2 * 2.0f * PI) / \
     (60.0f * SHOOT_BULLET_42MM_MASS_KG * SHOOT_FRIC_WHEEL_RADIUS_M))
#endif

/* ========================= 涓插彛涓庤鍒ょ郴缁熼厤缃?========================= */
#define USART_RX_BUF_LENGHT                64    // 涓插彛鎺ユ敹缂撳啿鍖洪暱搴?
#define REFEREE_FIFO_BUF_LENGTH            1024  // 瑁佸垽绯荤粺 FIFO 闀垮害
#define REF_PROTOCOL_FRAME_MAX_SIZE        192   // 瑁佸垽绯荤粺鏈€澶у抚闀?

/* ========================= CAN 鐢垫満 ID 閰嶇疆 ========================= */
/* CAN1: yaw DM MIT motor and strum DM MIT motor */
#define DM_YAW_CAN_ID                       0X01
#define DM_STRUM_CAN_ID                     0X03
#define DM_YAW_MASTER_ID                    0X51
#define DM_STRUM_MASTER_ID                  0X53

/* CAN2: pitch DM MIT motor */
#define DM_PIT_CAN_ID                       0X02
#define DM_PIT_MASTER_ID                    0X52
#define CAN_FRIC1_ID                        0X201 // 鎽╂摝杞?1 鐢垫満 ID
#define CAN_FRIC2_ID                        0X202 // 鎽╂摝杞?2 鐢垫満 ID
#define CAN_FRIC3_ID                        0X203 // 鎽╂摝杞?3 鐢垫満 ID

#define CAN_CHASSIS_ALL_ID                0x1FF  // chassis motor broadcast ID
#define CAN_M1_3508_ID                    0x205  // module1 3508 ID
#define CAN_M2_3508_ID                    0x206  // module2 3508 ID
#define CAN_M1_6020_ID                    0x207  // module1 6020 ID
#define CAN_M2_6020_ID                    0x208  // module2 6020 ID

#define CHASSIS_MODULE_NUM                  2U
#define CHASSIS_MOTOR_NUM                 (CHASSIS_MODULE_NUM * 2U)

/* ========================= MIT 鍗忚鍙傛暟鑼冨洿 ========================= */
#define P_MIN                              -12.5663704f // 浣嶇疆鏈€灏忓€?
#define P_MAX                               12.5663704f // 浣嶇疆鏈€澶у€?
#define V_MIN                              -30          // 閫熷害鏈€灏忓€?
#define V_MAX                               30          // 閫熷害鏈€澶у€?
#define KP_MIN                              0.0         // Kp 鏈€灏忓€?
#define KP_MAX                              500.0       // Kp 鏈€澶у€?
#define KD_MIN                              0.0         // Kd 鏈€灏忓€?
#define KD_MAX                              5.0         // Kd 鏈€澶у€?
#define T_MIN                              -10.0f       // 鍔涚煩鏈€灏忓€?
#define T_MAX                               10.0f       // 鍔涚煩鏈€澶у€?

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CONFIG_H */
