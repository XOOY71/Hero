/**
  * @file       chassis_behaviour.c
  * @brief      搴曠洏琛屼负妯″紡璁剧疆
  * @note
  *			濡傛灉瑕佹坊鍔犱竴涓柊鐨勮涓烘ā寮?
	*
  *			1.鍦?h鏂囦欢涓殑 chassis_behaviour_e鏋氫妇 涓嬫坊鍔犳柊鐨勫簳鐩樻ā寮廋HASSIS_XXX_XXX
  *
  *			2.鍦?c鏂囦欢涓殑鍑芥暟澹版槑鍖烘湯灏炬坊鍔犳柊鐨勫嚱鏁板０鏄? 骞跺湪鏂囦欢鏈熬娣诲姞鏂扮殑鍏蜂綋鎺у埗瀹炵幇鍑芥暟
  *
  *			3.鍦?c鏂囦欢涓殑 chassis_behaviour_mode_set() 鍑芥暟涓坊鍔犳柊鐨勫垽鏂? 浣縞hassis_behaviour鑳借璧嬪€兼垚CHASSIS_XXX_XXX
  *
	*			4.鍦?c鏂囦欢涓殑 chassis_behaviour_control_set() 鍑芥暟涓坊鍔犳柊鍒ゆ柇, 浠ユ璋冪敤鐩稿叧鎺у埗鍑芥暟
  *
	*     000000000000000     00               00    00     00   00         00
	*           00     0      00                00    00   00     00       00
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00
	*      00000000000     00  000000    00     000     00           000000
	*     00    00   0000     00    00   00      00   000000           00
	*       00000000000       00    00   00           000000           00
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00
	*           00            00                000 00000000000        00
	*           00  00        00          0    000      00             00
	*      000000000000        00        000  000       00          00 00
	*       00        00        000000000000            00            00
	********************************************************************************/

#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gimbal_behaviour.h"
#include "project_config.h"
#include <math.h>
#include <stdbool.h>

/* 鍑芥暟澹版槑鍖?*/
static void chassis_no_move_control											(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control	(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_yaw_hold_control										(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_spin_control												(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_release_reverse_update(fp32 *vx_set,
                                           fp32 *vy_set,
                                           bool stick_active,
                                           chassis_move_t *chassis_move_rc_to_vector);


//搴曠洏琛屼负妯″紡鍙橀噺, 浼氫繚瀛樺綋鍓嶅簳鐩樿涓烘ā寮? 鍒濆鍖栦负鏃犲姏妯″紡
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_NO_MOVE;
extern super_cap_mode_e super_cap_mode;

/**
  * @brief          鏍规嵁閬ユ帶鍣ㄥ紑鍏充綅缃拰閿洏杈撳叆璁剧疆搴曠洏琛屼负妯″紡, 骞朵负姣忕琛屼负妯″紡閫夋嫨鍚堥€傜殑搴曠洏鎺у埗妯″紡
  * @param[in]      chassis_move_mode: 搴曠洏鏁版嵁鎸囬拡
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL) return;

	//閬ユ帶鍣ㄨ缃ā寮忥紝浠ヤ笅鍙傛暟鍧囧彲閫夋嫨
	//CHASSIS_ZERO_FORCE, CHASSIS_NO_MOVE, CHASSIS_FOLLOW_GIMBAL_YAW, CHASSIS_OPEN, CHASSIS_SPIN
	if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) || chassis_move_mode->chassis_return_flag == 0)
	{
		chassis_behaviour_mode = CHASSIS_YAW_HOLD;		//涓嬫。搴曠洏yaw鐩爣瑙掗棴鐜?
	}
	else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;		//涓。闈欐妯″紡
	}
	else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && !switch_is_down(chassis_move_mode->chassis_RC->rc.s[1]))  //鐢佃剳娌℃敼
	{
		chassis_behaviour_mode = CHASSIS_SPIN;	//涓婃。灏忛檧铻烘ā寮?
	}

//	if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[0]) && switch_is_down(chassis_move_mode->chassis_RC->rc.s[1]) && chassis_move_mode->chassis_return_flag == 1)
//	{
//		chassis_behaviour_mode = CHASSIS_RETURN;	//鍙宠竟寮€鍏充负涓嬫。 涓?宸﹁竟寮€鍏充负涓嬫。 搴曠洏鍥炴
//	}

	if (chassis_move_mode->chassis_RC->rc.s[1] != 2)
	{
		chassis_move_mode->chassis_return_flag = 1;
	}

	/* 閬ユ帶鍣ㄥ湪涓嬫。锛屾牴鎹敭鐩榵銆乧銆乻hift鏉ユ敼鍙樻ā寮忥紝姣?涓绉掑埛鏂颁竴娆★紝鎵€浠ヨ涓€鐩存寜鐫€鎵嶈兘淇濊瘉妯″紡姝ｇ‘ */
	if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_ZERO_KEYBOARD))
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;		//x閿?闈欐妯″紡
	}
	else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_RELATIVE_KEYBOARD))
	{
		chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;		//c閿?璺熼殢浜戝彴妯″紡
	}
	else if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_SPIN_KEYBOARD))
	{
		chassis_behaviour_mode = CHASSIS_SPIN;	//shift閿?灏忛檧铻烘ā寮?
	}
//	else if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_RETURN_KEYBOARD))
//	{
//		chassis_behaviour_mode = CHASSIS_RETURN;	//z閿?鍥炴妯″紡
//	}

	//鏍规嵁琛屼负妯″紡閫夋嫨涓€涓簳鐩樻帶鍒舵ā寮?
	if (chassis_behaviour_mode == CHASSIS_NO_MOVE)	//闈欐妯″紡
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_MOVE; 	//閫熷害闈欐妯″紡
	}
	else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)		//璺熼殢浜戝彴妯″紡
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 	//閫熷害璺熼殢浜戝彴妯″紡
	}
	else if (chassis_behaviour_mode == CHASSIS_YAW_HOLD)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_YAW_HOLD;	//搴曠洏yaw鐩爣瑙掗棴鐜?
	}
	else if(chassis_behaviour_mode == CHASSIS_SPIN)		//灏忛檧铻烘ā寮?
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SPIN;	//閫熷害灏忛檧铻烘ā寮?
	}
//	else if (chassis_behaviour_mode == CHASSIS_RETURN)
//	{
//		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RETURN;  //搴曠洏鑷紶闆剁偣鍒颁簯鍙板綋鍓嶆柟鍚?
//	}

	{
		//瓒呯骇鐢靛寮€鍚垨鍏抽棴
		static bool pressed_Q = false, last_pressed_Q = false;
		pressed_Q = (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q);
		if(pressed_Q && !last_pressed_Q && super_cap_mode >= SUPER_CAP_PREPARED)
		{
			super_cap_mode = (super_cap_mode == SUPER_CAP_PREPARED) ? SUPER_CAP_USING : SUPER_CAP_PREPARED;
		}
		last_pressed_Q = pressed_Q;
	}
}

/**
  * @brief          鏍规嵁褰撳墠琛屼负妯″紡璋冪敤瀵瑰簲鐨勬帶鍒跺嚱鏁版潵璁剧疆搴曠洏杩愬姩鐨勪笁涓弬鏁?
  * @param[out]     vx_set, 閫氬父鎺у埗绾靛悜绉诲姩.
  * @param[out]     vy_set, 閫氬父鎺у埗妯悜绉诲姩.
  * @param[out]     wz_set, 閫氬父鎺у埗鏃嬭浆杩愬姩.
  * @param[in]      chassis_move_rc_to_vector, 鍖呮嫭搴曠洏鎵€鏈変俊鎭?
  * @retval         none
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;


	if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
	{
		chassis_no_move_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
	{
		chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_YAW_HOLD)
	{
		chassis_yaw_hold_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if(chassis_behaviour_mode == CHASSIS_SPIN)
	{
		chassis_spin_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
}

/**
  * @brief          搴曠洏涓嶇Щ鍔ㄧ殑琛屼负鐘舵€佹満涓? 搴曠洏妯″紡鏄笉璺熼殢瑙掑害
  * @author         RM
  * @param[in]      vx_set 鍓嶈繘鐨勯€熷害, 姝ｅ€?鍓嶈繘閫熷害, 	璐熷€?鍚庨€€閫熷害
  * @param[in]      vy_set 宸﹀彸鐨勯€熷害, 姝ｅ€?宸︾Щ閫熷害,   璐熷€?鍙崇Щ閫熷害
  * @param[in]      wz_set 鏃嬭浆鐨勯€熷害, 姝ｅ€?閫嗘椂閽堟棆杞? 璐熷€?椤烘椂閽堟棆杞?
  * @param[in]      chassis_move_rc_to_vector搴曠洏鏁版嵁
  * @retval         杩斿洖绌?
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

//鎶婂皬鍐欑殑鍙橀噺璧嬪€兼垚瀵瑰簲瀹忥紝鐢ㄤ簬璋冭瘯锛岃皟璇曠粨鏉熷悗搴斿垹闄ゅ彉閲忎娇鐢ㄥ畯瀹氫箟
fp32 chassis_wz_rc_sen = CHASSIS_WZ_RC_SEN;

/**
  * @brief          搴曠洏璺熼殢浜戝彴鐨勮涓虹姸鎬佹満涓? 搴曠洏妯″紡鏄窡闅忎簯鍙拌搴? 搴曠洏鏃嬭浆閫熷害浼氭牴鎹搴﹀樊璁＄畻搴曠洏鏃嬭浆鐨勮閫熷害
  * @author         RM
  * @param[in]      vx_set鍓嶈繘鐨勯€熷害, 姝ｅ€?鍓嶈繘閫熷害, 璐熷€?鍚庨€€閫熷害
  * @param[in]      vy_set宸﹀彸鐨勯€熷害, 姝ｅ€?宸︾Щ閫熷害, 璐熷€?鍙崇Щ閫熷害
  * @param[in]      angle_set搴曠洏涓庝簯鍙版帶鍒跺埌鐨勭浉瀵硅搴?
  * @param[in]      chassis_move_rc_to_vector搴曠洏鏁版嵁
  * @retval         杩斿洖绌?
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	//鏍规嵁 閬ユ帶鍣ㄧ殑閫氶亾鍊间互鍙婇敭鐩樻寜閿?寰楀嚭 涓€鑸儏鍐典笅鐨勯€熷害璁惧畾鍊?
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	int16_t wz_channel = 0;
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

	*wz_set = -(fp32)wz_channel * chassis_wz_rc_sen;
}

static void chassis_yaw_hold_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	int16_t wz_channel = 0;
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

	*wz_set = -(fp32)wz_channel;
}

fp32 chassis_spin_speed = CHASSIS_SPIN_SPEED;

/**
  * @brief          璁剧疆鍥哄畾鐨勬棆杞€熷害wz, 鍦ㄦ牴鎹仴鎺у櫒杈撳叆璁剧疆vx鍜寁y
  * @param[in]      vx_set 鍓嶈繘鐨勯€熷害, 姝ｅ€?鍓嶈繘閫熷害,	  璐熷€?鍚庨€€閫熷害
  * @param[in]      vy_set 宸﹀彸鐨勯€熷害, 姝ｅ€?宸︾Щ閫熷害,	  璐熷€?鍙崇Щ閫熷害
  * @param[in]      wz_set 鏃嬭浆閫熷害,	 姝ｅ€?閫嗘椂閽堟棆杞? 璐熷€?椤烘椂閽堟棆杞?
  * @param[in]      chassis_move_rc_to_vector搴曠洏鏁版嵁
  * @retval         none
  */
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	//鏍规嵁 閬ユ帶鍣ㄧ殑閫氶亾鍊间互鍙婇敭鐩樻寜閿?寰楀嚭 涓€鑸儏鍐典笅鐨勯€熷害璁惧畾鍊?
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	//璁剧疆瑙掗€熷害涓?鏃嬭浆閫熷害 涔樹互 姣斾緥鍥犲瓙
	*wz_set = CHASSIS_SPIN_SPEED;

	return;
}

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) return;

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	fp32 slope_percentage = 0.30f;
	bool stick_active;
	static uint8_t orientation_count[4] = {0};

	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * (CHASSIS_VX_RC_SEN);
	vy_set_channel = vy_channel * (CHASSIS_VY_RC_SEN);
	stick_active = ((vx_channel != 0) || (vy_channel != 0));

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		if (orientation_count[0] < 210){	orientation_count[0]++; }
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * (slope_percentage + (((fp32)orientation_count[0]) / 300.0f));
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		if (orientation_count[1] < 210){	orientation_count[1]++; }
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed * (slope_percentage + (((fp32)orientation_count[1]) / 300.0f));
	}

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		if (orientation_count[2] < 210){	orientation_count[2]++; }
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed * (slope_percentage + (((fp32)orientation_count[2]) / 300.0f));
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		if (orientation_count[2] < 210){	orientation_count[3]++; }
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed * (slope_percentage + (((fp32)orientation_count[3]) / 300.0f));
	}

	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)){ orientation_count[0] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY )){ orientation_count[1] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY )){ orientation_count[2] = 0; }
	if (!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)){ orientation_count[3] = 0; }

	// 娑撯偓闂冭埖鎶ゅ▔銏犻挬濠婃垿鈧喎瀹抽幐鍥︽姢
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}
	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set =  chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = -chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;

	chassis_release_reverse_update(vx_set,
	                               vy_set,
	                               stick_active,
	                               chassis_move_rc_to_vector);
}

static void chassis_release_reverse_update(fp32 *vx_set,
                                           fp32 *vy_set,
                                           bool stick_active,
                                           chassis_move_t *chassis_move_rc_to_vector)
{
#if (CHASSIS_RELEASE_REVERSE_ENABLE != 0U)
	static bool last_stick_active = false;
	static fp32 reverse_vx_set = 0.0f;
	static fp32 reverse_vy_set = 0.0f;
	static fp32 reverse_scale = 0.0f;
	static fp32 reverse_decay_time = CHASSIS_RELEASE_REVERSE_MIN_TIME;
	static bool reverse_lock_zero = false;

	fp32 plan_speed;
	fp32 decay_step;
	fp32 speed_ratio;

	if (vx_set == NULL || vy_set == NULL || chassis_move_rc_to_vector == NULL) return;

	if (stick_active)
	{
		last_stick_active = true;
		reverse_scale = 0.0f;
		reverse_lock_zero = false;
		return;
	}

	if (last_stick_active)
	{
		plan_speed = sqrtf(chassis_move_rc_to_vector->vx_plan * chassis_move_rc_to_vector->vx_plan +
		                   chassis_move_rc_to_vector->vy_plan * chassis_move_rc_to_vector->vy_plan);
		if (plan_speed > CHASSIS_RELEASE_REVERSE_LOCK_SPEED_EPS)
		{
			reverse_vx_set = -chassis_move_rc_to_vector->vx_plan;
			reverse_vy_set = -chassis_move_rc_to_vector->vy_plan;
			speed_ratio = plan_speed / CHASSIS_RELEASE_REVERSE_REF_SPEED;
			if (speed_ratio > 1.0f)
			{
				speed_ratio = 1.0f;
			}
			reverse_decay_time = CHASSIS_RELEASE_REVERSE_MIN_TIME +
			                     (CHASSIS_RELEASE_REVERSE_MAX_TIME - CHASSIS_RELEASE_REVERSE_MIN_TIME) *
			                     speed_ratio;
			reverse_scale = 1.0f;
			reverse_lock_zero = false;
		}
		last_stick_active = false;
	}

	if (reverse_scale > 0.0f)
	{
		*vx_set = reverse_vx_set * reverse_scale;
		*vy_set = reverse_vy_set * reverse_scale;

		decay_step = CHASSIS_CONTROL_TIME / reverse_decay_time;
		reverse_scale -= decay_step;
		if (reverse_scale <= 0.0f)
		{
			reverse_scale = 0.0f;
			reverse_lock_zero = true;
		}
	}

	if (reverse_lock_zero)
	{
		*vx_set = 0.0f;
		*vy_set = 0.0f;
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;

		if ((fabsf(chassis_move_rc_to_vector->vx_plan) < CHASSIS_RELEASE_REVERSE_LOCK_SPEED_EPS) &&
		    (fabsf(chassis_move_rc_to_vector->vy_plan) < CHASSIS_RELEASE_REVERSE_LOCK_SPEED_EPS))
		{
			reverse_lock_zero = false;
		}
	}
#else
	(void)vx_set;
	(void)vy_set;
	(void)stick_active;
	(void)chassis_move_rc_to_vector;
#endif
}
