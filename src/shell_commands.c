#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "control.h"
#include "motor_debug.h"
#include "pid.h"

#define SHELL_WHEEL_CAN_NODE DT_ALIAS(can_wheel)
#define SHELL_JOINT_CAN_NODE DT_ALIAS(can_joint)

static const struct device *const shell_wheel_can =
	DEVICE_DT_GET(SHELL_WHEEL_CAN_NODE);
static const struct device *const shell_joint_can =
	DEVICE_DT_GET(SHELL_JOINT_CAN_NODE);

static bool parse_bool(const char *text)
{
	return strcmp(text, "1") == 0 || strcmp(text, "on") == 0 ||
	       strcmp(text, "true") == 0 || strcmp(text, "enable") == 0;
}

static bool parse_i32(const char *text, int32_t *out)
{
	char *end = NULL;

	errno = 0;
	const long value = strtol(text, &end, 0);
	if (errno != 0 || end == text || *end != '\0') {
		return false;
	}

	*out = (int32_t)value;
	return true;
}

static bool parse_float_arg(const char *text, float *out)
{
	char *end = NULL;

	errno = 0;
	const float value = strtof(text, &end);
	if (errno != 0 || end == text || *end != '\0') {
		return false;
	}

	*out = value;
	return true;
}

static const char *can_state_name(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

static bool parse_can_bus(const char *text, const struct device **dev,
			  const char **name)
{
	if (strcmp(text, "joint") == 0 || strcmp(text, "dm") == 0 ||
	    strcmp(text, "can1") == 0) {
		*dev = shell_joint_can;
		*name = "joint/CAN1";
		return true;
	}

	if (strcmp(text, "wheel") == 0 || strcmp(text, "m3508") == 0 ||
	    strcmp(text, "can2") == 0) {
		*dev = shell_wheel_can;
		*name = "wheel/CAN2";
		return true;
	}

	return false;
}

static bool parse_wheel_id(const char *text, uint8_t *id)
{
	if (strcmp(text, "left") == 0 || strcmp(text, "l") == 0) {
		*id = APP_WHEEL_LEFT_ID;
		return true;
	}
	if (strcmp(text, "right") == 0 || strcmp(text, "r") == 0) {
		*id = APP_WHEEL_RIGHT_ID;
		return true;
	}

	int32_t value;
	if (!parse_i32(text, &value) || value <= 0 ||
	    value > DJI_M3508_MAX_ID) {
		return false;
	}

	*id = (uint8_t)value;
	return true;
}

static bool parse_dm_id(const char *text, uint8_t *id)
{
	if (strcmp(text, "left") == 0 || strcmp(text, "l") == 0) {
		*id = APP_DM_LEFT_ID;
		return true;
	}
	if (strcmp(text, "right") == 0 || strcmp(text, "r") == 0) {
		*id = APP_DM_RIGHT_ID;
		return true;
	}

	int32_t value;
	if (!parse_i32(text, &value) || value <= 0 || value > DM4340_MAX_ID) {
		return false;
	}

	*id = (uint8_t)value;
	return true;
}

static bool parse_optional_duration(size_t argc, char **argv, size_t index,
				    int32_t *duration_ms)
{
	*duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;
	if (argc <= index) {
		return true;
	}

	return parse_i32(argv[index], duration_ms);
}

static void enter_motor_debug_mode(void)
{
	control_set_enable(false);
	control_stop_motion();
}

static const char *dm_reg_name(uint8_t rid)
{
	switch (rid) {
	case 0x01:
		return "KT_Value";
	case 0x02:
		return "OT_Value";
	case 0x03:
		return "OC_Value";
	case 0x04:
		return "ACC";
	case 0x05:
		return "DEC";
	case 0x06:
		return "MAX_SPD";
	case 0x07:
		return "MST_ID";
	case 0x08:
		return "ESC_ID";
	case 0x09:
		return "TIMEOUT";
	case 0x0a:
		return "CTRL_MODE";
	case 0x15:
		return "PMAX";
	case 0x16:
		return "VMAX";
	case 0x17:
		return "TMAX";
	case 0x23:
		return "can_br";
	case 0x3b:
		return "Imax";
	case 0x3c:
		return "VBus";
	default:
		return "reg";
	}
}

static bool dm_reg_is_u32(uint8_t rid)
{
	switch (rid) {
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0a:
	case 0x23:
	case 0x24:
	case 0x25:
		return true;
	default:
		return false;
	}
}

static const char *dm_ctrl_mode_name(uint32_t mode)
{
	switch (mode) {
	case 1:
		return "MIT";
	case 2:
		return "pos_vel";
	case 3:
		return "velocity";
	case 4:
		return "pvt";
	default:
		return "unknown";
	}
}

static int cmd_robot_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	const bool enable = parse_bool(argv[1]);
	control_set_enable(enable);
	shell_print(sh, "balance %s", enable ? "enabled" : "disabled");
	return 0;
}

static int cmd_robot_height(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	const int height = (int)strtol(argv[1], NULL, 10);
	control_set_height(height);
	shell_print(sh, "height set to %d", height);
	return 0;
}

static int cmd_robot_joy(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	const float x = strtof(argv[1], NULL);
	const float y = strtof(argv[2], NULL);
	control_set_joystick(x, y);
	shell_print(sh, "joy x=%.1f y=%.1f", (double)x, (double)y);
	return 0;
}

static int cmd_robot_motion(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	robot_motion_t motion = ROBOT_STOP;

	if (strcmp(argv[1], "forward") == 0) {
		motion = ROBOT_FORWARD;
	} else if (strcmp(argv[1], "back") == 0) {
		motion = ROBOT_BACK;
	} else if (strcmp(argv[1], "left") == 0) {
		motion = ROBOT_LEFT;
	} else if (strcmp(argv[1], "right") == 0) {
		motion = ROBOT_RIGHT;
	} else if (strcmp(argv[1], "jump") == 0) {
		motion = ROBOT_JUMP;
	} else if (strcmp(argv[1], "stop") == 0) {
		motion = ROBOT_STOP;
	} else {
		shell_error(sh, "motion must be forward/back/left/right/jump/stop");
		return -EINVAL;
	}

	control_set_motion(motion);
	shell_print(sh, "motion %s", argv[1]);
	return 0;
}

static int cmd_robot_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	control_stop_motion();
	shell_print(sh, "motion stopped");
	return 0;
}

static int cmd_robot_jump(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	control_request_jump();
	shell_print(sh, "jump requested");
	return 0;
}

static int cmd_robot_zero(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	float zero;

	if (strcmp(argv[1], "now") == 0) {
		control_status_t st;

		control_get_status(&st);
#if APP_PID_BALANCE_USE_ROLL_AXIS
		zero = APP_PID_BALANCE_AXIS_SIGN * st.roll_deg;
#else
		zero = APP_PID_BALANCE_AXIS_SIGN * st.pitch_deg;
#endif
		if (fabsf(zero) > APP_PID_BALANCE_ZERO_NOW_LIMIT_DEG) {
			shell_error(sh,
				    "zero now refused at %.2f deg; hold robot upright or use robot zero <deg>",
				    (double)zero);
			return -EINVAL;
		}
	} else {
		zero = strtof(argv[1], NULL);
	}

	control_set_angle_zero(zero);
	shell_print(sh, "balance zero set to %.2f deg", (double)zero);
	return 0;
}

static int cmd_robot_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	control_status_t st;

	control_get_status(&st);
	shell_print(sh,
		    "enable=%d wheels=%d fault=%d height=%d joy=(%.1f,%.1f) "
		    "pitch=%.2f roll=%.2f yaw=%.2f speed=%.2f lqr=%.1f "
		    "yaw_out=%.1f current=(%d,%d) joint=(%.3f,%.3f) jump=%d",
		    st.enable_request, st.wheels_enabled, st.faulted, st.height,
		    (double)st.joy_x, (double)st.joy_y,
		    (double)st.pitch_deg, (double)st.roll_deg,
		    (double)st.yaw_deg, (double)st.speed_rad_s,
		    (double)st.lqr_output, (double)st.yaw_output,
		    st.left_wheel_current, st.right_wheel_current,
		    (double)st.left_joint_position_rad,
		    (double)st.right_joint_position_rad, st.jump_phase);
	return 0;
}

static int cmd_robot_pid(const struct shell *sh, size_t argc, char **argv)
{
	control_pid_balance_params_t params;

	control_get_pid_balance_params(&params);

	if (argc == 1) {
		shell_print(sh,
			    "pid angle_p=%.2f angle_i=%.2f angle_d=%.2f gyro_p=%.2f distance_p=%.2f speed_p=%.2f limit=%d mA",
			    (double)params.angle_p, (double)params.angle_i,
			    (double)params.angle_d, (double)params.gyro_p,
			    (double)params.distance_p, (double)params.speed_p,
			    params.current_limit_ma);
		return 0;
	}

	if (argc < 3 || !parse_float_arg(argv[1], &params.angle_p) ||
	    !parse_float_arg(argv[2], &params.gyro_p)) {
		shell_error(sh,
			    "usage: robot pid [angle_p gyro_p [distance_p] [speed_p] [limit_mA]]");
		return -EINVAL;
	}

	if (argc > 3 && !parse_float_arg(argv[3], &params.distance_p)) {
		shell_error(sh, "invalid distance_p");
		return -EINVAL;
	}
	if (argc > 4 && !parse_float_arg(argv[4], &params.speed_p)) {
		shell_error(sh, "invalid speed_p");
		return -EINVAL;
	}
	if (argc > 5) {
		int32_t current_limit_ma;
		if (!parse_i32(argv[5], &current_limit_ma)) {
			shell_error(sh, "invalid limit_mA");
			return -EINVAL;
		}
		params.current_limit_ma = (int16_t)current_limit_ma;
	}

	control_set_pid_balance_params(&params);
	control_get_pid_balance_params(&params);
	shell_print(sh,
		    "pid set angle_p=%.2f gyro_p=%.2f distance_p=%.2f speed_p=%.2f limit=%d mA",
		    (double)params.angle_p, (double)params.gyro_p,
		    (double)params.distance_p, (double)params.speed_p,
		    params.current_limit_ma);
	return 0;
}

static int print_can_status(const struct shell *sh, const char *name,
			    const struct device *dev)
{
	if (!device_is_ready(dev)) {
		shell_error(sh, "%s device is not ready", name);
		return -ENODEV;
	}

	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	const int ret = can_get_state(dev, &state, &err_cnt);

	if (ret != 0) {
		shell_error(sh, "%s get state failed: %d", name, ret);
		return ret;
	}

	shell_print(sh, "%s state=%s tx_err=%u rx_err=%u", name,
		    can_state_name(state), err_cnt.tx_err_cnt,
		    err_cnt.rx_err_cnt);
	return 0;
}

static int cmd_motor_can_status(const struct shell *sh, size_t argc,
				char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		(void)print_can_status(sh, "joint/CAN1", shell_joint_can);
		(void)print_can_status(sh, "wheel/CAN2", shell_wheel_can);
		return 0;
	}

	const struct device *dev;
	const char *name;
	if (!parse_can_bus(argv[1], &dev, &name)) {
		shell_error(sh, "bus must be joint|dm|can1|wheel|m3508|can2");
		return -EINVAL;
	}

	return print_can_status(sh, name, dev);
}

static int cmd_motor_can_raw_common(const struct shell *sh, size_t argc,
				    char **argv, bool extended)
{
	const struct device *dev;
	const char *name;
	int32_t id_value;
	const int32_t max_id = extended ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK;

	if (!parse_can_bus(argv[1], &dev, &name) ||
	    !parse_i32(argv[2], &id_value) || id_value < 0 ||
	    id_value > max_id) {
		shell_error(sh,
			    "usage: motor can %s <joint|wheel|can1|can2> "
			    "<%s_id> [byte0..byte7]",
			    extended ? "rawx" : "raw",
			    extended ? "ext" : "std");
		return -EINVAL;
	}

	struct can_frame frame = {
		.flags = extended ? CAN_FRAME_IDE : 0,
		.id = (uint32_t)id_value,
		.dlc = (uint8_t)(argc - 3U),
	};

	for (size_t i = 3; i < argc; i++) {
		int32_t byte_value;

		if (!parse_i32(argv[i], &byte_value) || byte_value < 0 ||
		    byte_value > 0xff) {
			shell_error(sh, "byte%u must be 0..255",
				    (unsigned int)(i - 3U));
			return -EINVAL;
		}
		frame.data[i - 3U] = (uint8_t)byte_value;
	}

	enter_motor_debug_mode();
	const int ret = can_send(dev, &frame, K_MSEC(5), NULL, NULL);
	shell_print(sh, "%s raw%s id=0x%0*x dlc=%u ret=%d", name,
		    extended ? "x" : "", extended ? 8 : 3, frame.id,
		    frame.dlc, ret);
	return ret;
}

static int cmd_motor_can_raw(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_motor_can_raw_common(sh, argc, argv, false);
}

static int cmd_motor_can_rawx(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_motor_can_raw_common(sh, argc, argv, true);
}

static int cmd_motor_can_recover(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		can_stop(shell_joint_can);
		can_start(shell_joint_can);
		can_stop(shell_wheel_can);
		can_start(shell_wheel_can);
		(void)print_can_status(sh, "joint/CAN1", shell_joint_can);
		(void)print_can_status(sh, "wheel/CAN2", shell_wheel_can);
		return 0;
	}

	const struct device *dev;
	const char *name;
	if (!parse_can_bus(argv[1], &dev, &name)) {
		shell_error(sh, "bus must be joint|dm|can1|wheel|m3508|can2|all");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	can_stop(dev);
	can_start(dev);
	return print_can_status(sh, name, dev);
}

static int print_wheel_status(const struct shell *sh, uint8_t id)
{
	dji_m3508_motor_t motor;

	if (!motor_debug_get_m3508(id, &motor)) {
		shell_print(sh, "VESC/M3508 id=%u no feedback", id);
		return -ENODATA;
	}

	const int64_t age_ms = k_uptime_get() - motor.last_update_ms;
	const int64_t s4_age_ms = motor.last_status4_update_ms > 0 ?
				  k_uptime_get() -
					  motor.last_status4_update_ms :
				  -1;
	const int64_t s5_age_ms = motor.last_status5_update_ms > 0 ?
				  k_uptime_get() -
					  motor.last_status5_update_ms :
				  -1;
	shell_print(sh,
		    "VESC/M3508 id=%u age=%lldms erpm=%d motor_rpm=%d "
		    "angle=%.3f rad speed=%.3f rad/s cmd=%d mA "
		    "motor_current=%d mA input=%d mA vin=%.2f V "
		    "temp=%.1f/%.1fC tach=%d torque_k=%.6f "
		    "torque_est=%.3f Nm duty=%.3f s4_age=%lldms "
		    "s5_age=%lldms",
		    id, (long long)age_ms, motor.erpm, motor.speed_rpm,
		    (double)motor.angle_rad, (double)motor.speed_rad_s,
		    motor.command_current_ma, motor.motor_current_ma,
		    motor.input_current_ma,
		    (double)motor.input_voltage_mv * 0.001,
		    (double)motor.fet_temperature_cdeg * 0.1,
		    (double)motor.motor_temperature_cdeg * 0.1,
		    motor.tachometer,
		    (double)motor.current_ma_to_wheel_torque_nm,
		    (double)motor.estimated_wheel_torque_nm,
		    (double)motor.duty, (long long)s4_age_ms,
		    (long long)s5_age_ms);
	return 0;
}

static int cmd_motor_wheel_status(const struct shell *sh, size_t argc,
				  char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		(void)print_wheel_status(sh, APP_WHEEL_LEFT_ID);
		(void)print_wheel_status(sh, APP_WHEEL_RIGHT_ID);
		return 0;
	}

	uint8_t id;
	if (!parse_wheel_id(argv[1], &id)) {
		shell_error(sh, "wheel id must be left/right/1..255");
		return -EINVAL;
	}

	return print_wheel_status(sh, id);
}

static int cmd_motor_wheel_current(const struct shell *sh, size_t argc,
				   char **argv)
{
	uint8_t id;
	int32_t current;
	int32_t duration_ms;

	if (!parse_wheel_id(argv[1], &id) || !parse_i32(argv[2], &current) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor wheel current <left|right|id> "
			    "<current_mA> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_wheel_current(id, current, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set wheel current failed: %d", ret);
		return ret;
	}

	shell_print(sh, "VESC/M3508 id=%u current=%d mA for %d ms", id,
		    (int)current, duration_ms);
	return 0;
}

static int cmd_motor_wheel_rpm(const struct shell *sh, size_t argc,
			       char **argv)
{
	uint8_t id;
	int32_t rpm;
	int32_t duration_ms;

	if (!parse_wheel_id(argv[1], &id) || !parse_i32(argv[2], &rpm) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor wheel rpm <left|right|100|101> "
			    "<target_erpm> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_wheel_rpm(id, rpm, duration_ms);
	if (ret != 0) {
		shell_error(sh,
			    "rpm command only supports configured wheels "
			    "left=%u right=%u",
			    APP_WHEEL_LEFT_ID, APP_WHEEL_RIGHT_ID);
		return ret;
	}

	shell_print(sh, "VESC/M3508 id=%u target_erpm=%d for %d ms", id,
		    (int)CLAMP(rpm, -APP_VESC_DEBUG_ERPM_LIMIT,
			       APP_VESC_DEBUG_ERPM_LIMIT),
		    duration_ms);
	return 0;
}

static int cmd_motor_wheel_pair(const struct shell *sh, size_t argc,
				char **argv)
{
	int32_t left_current;
	int32_t right_current;
	int32_t duration_ms;

	if (!parse_i32(argv[1], &left_current) ||
	    !parse_i32(argv[2], &right_current) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor wheel pair <left_current_mA> "
			    "<right_current_mA> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_wheel_pair(left_current, right_current,
						   duration_ms);
	if (ret != 0) {
		shell_error(sh, "set wheel pair failed: %d", ret);
		return ret;
	}

	shell_print(sh, "VESC/M3508 pair current=(%d,%d) mA for %d ms",
		    (int)left_current, (int)right_current, duration_ms);
	return 0;
}

static int cmd_motor_wheel_stop(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enter_motor_debug_mode();
	const int ret = motor_debug_stop_wheels();
	shell_print(sh, "VESC/M3508 current stopped");
	return ret;
}

static int print_dm_status(const struct shell *sh, uint8_t id)
{
	dm4340_feedback_t fb;

	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_print(sh, "DM4340 id=%u no feedback", id);
		return -ENODATA;
	}

	const int64_t age_ms = k_uptime_get() - fb.last_update_ms;
	shell_print(sh,
		    "DM4340 id=%u err=%u age=%lldms pos=%.4f rad "
		    "vel=%.4f rad/s torque=%.3f Nm temp=%u/%uC",
		    id, fb.error, (long long)age_ms,
		    (double)fb.position_rad, (double)fb.velocity_rad_s,
		    (double)fb.torque_nm, fb.mos_temperature_c,
		    fb.rotor_temperature_c);
	return 0;
}

static int cmd_motor_dm_status(const struct shell *sh, size_t argc,
			       char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		(void)print_dm_status(sh, APP_DM_LEFT_ID);
		(void)print_dm_status(sh, APP_DM_RIGHT_ID);
		return 0;
	}

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	return print_dm_status(sh, id);
}

static int cmd_motor_dm_enable(const struct shell *sh, size_t argc,
			       char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_dm_enable(id);
	shell_print(sh, "DM4340 id=%u enable ret=%d", id, ret);
	return ret;
}

static int cmd_motor_dm_disable(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_dm_disable(id);
	shell_print(sh, "DM4340 id=%u disable ret=%d", id, ret);
	return ret;
}

static int cmd_motor_dm_zero(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_dm_save_zero(id);
	shell_print(sh, "DM4340 id=%u save zero ret=%d", id, ret);
	return ret;
}

static int cmd_motor_dm_reg(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	int32_t rid_value;
	if (!parse_dm_id(argv[1], &id) || !parse_i32(argv[2], &rid_value) ||
	    rid_value < 0 || rid_value > 0xff) {
		shell_error(sh, "usage: motor dm reg <left|right|id> <rid>");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	dm4340_param_response_t response;
	const int ret = motor_debug_dm_read_reg(id, (uint8_t)rid_value,
						&response);
	if (ret != 0) {
		shell_error(sh, "DM4340 id=%u reg 0x%02x read failed: %d",
			    id, (unsigned int)rid_value, ret);
		return ret;
	}

	const int64_t age_ms = k_uptime_get() - response.last_update_ms;
	const uint32_t u32 = response.raw_u32;
	const char *name = dm_reg_name((uint8_t)rid_value);

	if (dm_reg_is_u32((uint8_t)rid_value)) {
		if ((uint8_t)rid_value == 0x0a) {
			shell_print(sh,
				    "DM4340 id=%u %s(0x%02x) u32=%u mode=%s raw=0x%08x age=%lldms",
				    id, name, (unsigned int)rid_value, u32,
				    dm_ctrl_mode_name(u32), u32,
				    (long long)age_ms);
		} else {
			shell_print(sh,
				    "DM4340 id=%u %s(0x%02x) u32=%u raw=0x%08x age=%lldms",
				    id, name, (unsigned int)rid_value, u32,
				    u32, (long long)age_ms);
		}
	} else {
		shell_print(sh,
			    "DM4340 id=%u %s(0x%02x) f=%.6f raw=0x%08x u32=%u age=%lldms",
			    id, name, (unsigned int)rid_value,
			    (double)response.value_float, u32, u32,
			    (long long)age_ms);
	}

	return 0;
}

static int cmd_motor_dm_diag(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();

	/* Read all critical diagnostic registers */
	static const uint8_t diag_rids[] = {
		0x01, /* FAULT */
		0x02, /* WARNING */
		0x04, /* STATUS */
		0x05, /* CAN_ERR */
		0x06, /* MOTOR_ERR */
		0x09, /* TIMEOUT */
		0x0a, /* CTRL_MODE */
		0x15, /* PMAX */
		0x16, /* VMAX */
		0x17, /* TMAX */
		0x3b, /* Imax */
		0x3c, /* VBus */
	};

	dm4340_feedback_t fb;
	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_print(sh, "DM4340 id=%u no feedback", id);
	} else {
		const int64_t age_ms = k_uptime_get() - fb.last_update_ms;
		shell_print(sh,
			    "DM4340 id=%u err=0x%x age=%lldms pos=%.4f "
			    "vel=%.4f torque=%.4f temp=%u/%uC",
			    id, fb.error, (long long)age_ms,
			    (double)fb.position_rad, (double)fb.velocity_rad_s,
			    (double)fb.torque_nm, fb.mos_temperature_c,
			    fb.rotor_temperature_c);
	}

	shell_print(sh, "--- DM4340 id=%u diagnostic registers ---", id);

	for (size_t i = 0; i < ARRAY_SIZE(diag_rids); i++) {
		const uint8_t rid = diag_rids[i];
		dm4340_param_response_t response;
		const int ret = motor_debug_dm_read_reg(id, rid, &response);
		if (ret != 0) {
			shell_print(sh, "  %s(0x%02x) read failed: %d",
				    dm_reg_name(rid), (unsigned int)rid, ret);
			continue;
		}

		const char *name = dm_reg_name(rid);
		const uint32_t u32 = response.raw_u32;
		const int64_t age_ms = k_uptime_get() - response.last_update_ms;

		if (dm_reg_is_u32(rid)) {
			if (rid == 0x0a) {
				shell_print(sh,
					    "  %s(0x%02x) u32=%u mode=%s age=%lldms",
					    name, (unsigned int)rid, u32,
					    dm_ctrl_mode_name(u32),
					    (long long)age_ms);
			} else {
				shell_print(sh,
					    "  %s(0x%02x) u32=%u raw=0x%08x age=%lldms",
					    name, (unsigned int)rid, u32, u32,
					    (long long)age_ms);
			}
		} else {
			shell_print(sh,
				    "  %s(0x%02x) f=%.6f raw=0x%08x age=%lldms",
				    name, (unsigned int)rid,
				    (double)response.value_float, u32,
				    (long long)age_ms);
		}
	}

	shell_print(sh, "--- end diag ---");
	return 0;
}

static int cmd_motor_dm_pos(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float position_rad;
	float velocity_rad_s = 1.0f;
	int32_t duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &position_rad)) {
		shell_error(sh,
			    "usage: motor dm pos <left|right|id> <rad> "
			    "[vel_rad_s] [ms]");
		return -EINVAL;
	}
	if (argc > 3 && !parse_float_arg(argv[3], &velocity_rad_s)) {
		shell_error(sh, "invalid velocity");
		return -EINVAL;
	}
	if (argc > 4 && !parse_i32(argv[4], &duration_ms)) {
		shell_error(sh, "invalid duration");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_pos_vel(
		id, position_rad, velocity_rad_s, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM position failed: %d", ret);
		return ret;
	}

	shell_print(sh, "DM4340 id=%u pos=%.4f vel=%.3f for %d ms", id,
		    (double)position_rad, (double)velocity_rad_s,
		    duration_ms);
	return 0;
}

static int cmd_motor_dm_vel(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float velocity_rad_s;
	int32_t duration_ms;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &velocity_rad_s) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor dm vel <left|right|id> "
			    "<rad_s> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_velocity(id, velocity_rad_s,
						    duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM velocity failed: %d", ret);
		return ret;
	}

	shell_print(sh, "DM4340 id=%u vel=%.3f for %d ms", id,
		    (double)velocity_rad_s, duration_ms);
	return 0;
}

static int cmd_motor_dm_mit(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float position_rad;
	float velocity_rad_s;
	float kp;
	float kd;
	float torque_nm;
	int32_t duration_ms;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &position_rad) ||
	    !parse_float_arg(argv[3], &velocity_rad_s) ||
	    !parse_float_arg(argv[4], &kp) ||
	    !parse_float_arg(argv[5], &kd) ||
	    !parse_float_arg(argv[6], &torque_nm) ||
	    !parse_optional_duration(argc, argv, 7, &duration_ms)) {
		shell_error(sh,
			    "usage: motor dm mit <left|right|id> <pos_rad> "
			    "<vel_rad_s> <kp> <kd> <torque_nm> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_mit(id, position_rad,
					       velocity_rad_s, kp, kd,
					       torque_nm, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM MIT failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "DM4340 id=%u mit pos=%.4f vel=%.3f kp=%.2f kd=%.2f "
		    "t=%.3f for %d ms",
		    id, (double)position_rad, (double)velocity_rad_s,
		    (double)kp, (double)kd, (double)torque_nm, duration_ms);
	return 0;
}

static int cmd_motor_dm_nudge(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float delta_rad;
	float kp = 3.0f;
	float kd = 0.20f;
	int32_t duration_ms = 800;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &delta_rad)) {
		shell_error(sh,
			    "usage: motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [ms]");
		return -EINVAL;
	}
	if (argc > 3 && !parse_float_arg(argv[3], &kp)) {
		shell_error(sh, "invalid kp");
		return -EINVAL;
	}
	if (argc > 4 && !parse_float_arg(argv[4], &kd)) {
		shell_error(sh, "invalid kd");
		return -EINVAL;
	}
	if (argc > 5 && !parse_i32(argv[5], &duration_ms)) {
		shell_error(sh, "invalid duration");
		return -EINVAL;
	}

	dm4340_feedback_t fb;
	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_error(sh, "DM4340 id=%u has no feedback", id);
		return -ENODATA;
	}

	delta_rad = CLAMP(delta_rad, -0.10f, 0.10f);
	enter_motor_debug_mode();
	const float target_rad = fb.position_rad + delta_rad;
	const int ret = motor_debug_set_dm_mit_raw(id, target_rad, 0.0f,
						   kp, kd, 0.0f,
						   duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM nudge failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "DM4340 id=%u nudge from %.4f to %.4f rad delta=%.4f kp=%.2f kd=%.2f for %d ms",
		    id, (double)fb.position_rad, (double)target_rad,
		    (double)delta_rad, (double)kp, (double)kd, duration_ms);
	return 0;
}

static int cmd_motor_dm_wiggle(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float amplitude_rad;
	int32_t period_ms = 2000;
	float kp = 12.0f;
	float kd = 0.50f;
	int32_t duration_ms = APP_MOTOR_DEBUG_MAX_TIMEOUT_MS;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &amplitude_rad)) {
		shell_error(sh,
			    "usage: motor dm wiggle <left|right|id> <amp_rad> [period_ms] [kp] [kd] [ms]");
		return -EINVAL;
	}
	if (argc > 3 && !parse_i32(argv[3], &period_ms)) {
		shell_error(sh, "invalid period");
		return -EINVAL;
	}
	if (argc > 4 && !parse_float_arg(argv[4], &kp)) {
		shell_error(sh, "invalid kp");
		return -EINVAL;
	}
	if (argc > 5 && !parse_float_arg(argv[5], &kd)) {
		shell_error(sh, "invalid kd");
		return -EINVAL;
	}
	if (argc > 6 && !parse_i32(argv[6], &duration_ms)) {
		shell_error(sh, "invalid duration");
		return -EINVAL;
	}

	dm4340_feedback_t fb;
	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_error(sh, "DM4340 id=%u has no feedback", id);
		return -ENODATA;
	}

	amplitude_rad = app_clampf(amplitude_rad, 0.001f, 0.08f);
	period_ms = CLAMP(period_ms, 500, 5000);
	if (duration_ms <= 0) {
		duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;
	}
	duration_ms = MIN(duration_ms, APP_MOTOR_DEBUG_MAX_TIMEOUT_MS);

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_wiggle(id, fb.position_rad,
						 amplitude_rad, period_ms, kp,
						 kd, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM wiggle failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "DM4340 id=%u wiggle center=%.4f amp=%.4f period=%dms kp=%.2f kd=%.2f for %d ms",
		    id, (double)fb.position_rad, (double)amplitude_rad,
		    period_ms, (double)kp, (double)kd, duration_ms);
	return 0;
}

static int cmd_motor_dm_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_stop_dm(id);
	shell_print(sh, "DM4340 id=%u debug stopped", id);
	return ret;
}

static int cmd_motor_dm_rxlog(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_debug_dump_dm4340_rx_log();
	return 0;
}

static int cmd_motor_debug_status(const struct shell *sh, size_t argc,
				  char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_debug_output_t out;

	if (!motor_debug_get_output(&out)) {
		shell_print(sh, "motor debug inactive");
		return 0;
	}

	shell_print(sh, "motor debug active wheel=%d current=(%d,%d)",
		    out.wheel_active,
		    out.wheel_current[APP_WHEEL_LEFT_ID],
		    out.wheel_current[APP_WHEEL_RIGHT_ID]);
	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		const motor_debug_dm_command_t *cmd = &out.dm[id];
		if (cmd->mode == MOTOR_DEBUG_DM_NONE) {
			continue;
		}
		shell_print(sh,
			    "dm id=%u mode=%s pos=%.4f vel=%.3f "
			    "kp=%.2f kd=%.2f t=%.3f",
			    id, motor_debug_dm_mode_name(cmd->mode),
			    (double)cmd->position_rad,
			    (double)cmd->velocity_rad_s, (double)cmd->kp,
			    (double)cmd->kd, (double)cmd->torque_nm);
	}

	return 0;
}

static int cmd_motor_debug_stop(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enter_motor_debug_mode();
	const int ret = motor_debug_stop_all();
	shell_print(sh, "all manual motor debug stopped");
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	robot_cmds,
	SHELL_CMD_ARG(enable, NULL, "robot enable <0|1|on|off>",
		      cmd_robot_enable, 2, 0),
	SHELL_CMD_ARG(height, NULL, "robot height <32..80>",
		      cmd_robot_height, 2, 0),
	SHELL_CMD_ARG(joy, NULL, "robot joy <x:-100..100> <y:-100..100>",
		      cmd_robot_joy, 3, 0),
	SHELL_CMD_ARG(motion, NULL,
		      "robot motion <forward|back|left|right|jump|stop>",
		      cmd_robot_motion, 2, 0),
	SHELL_CMD_ARG(stop, NULL, "robot stop", cmd_robot_stop, 1, 0),
	SHELL_CMD_ARG(jump, NULL, "robot jump", cmd_robot_jump, 1, 0),
	SHELL_CMD_ARG(zero, NULL, "robot zero <deg|now>",
		      cmd_robot_zero, 2, 0),
	SHELL_CMD_ARG(status, NULL, "robot status", cmd_robot_status, 1, 0),
	SHELL_CMD_ARG(pid, NULL,
		      "robot pid [angle_p gyro_p [distance_p] [speed_p] [limit_mA]]",
		      cmd_robot_pid, 1, 5),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(robot, &robot_cmds, "wheel-leg robot control", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_wheel_cmds,
	SHELL_CMD_ARG(status, NULL, "motor wheel status [left|right|1..255|all]",
		      cmd_motor_wheel_status, 1, 1),
	SHELL_CMD_ARG(current, NULL,
		      "motor wheel current <left|right|id> <current_mA> [ms]",
		      cmd_motor_wheel_current, 3, 1),
	SHELL_CMD_ARG(rpm, NULL,
		      "motor wheel rpm <left|right|100|101> <target_erpm> [ms]",
		      cmd_motor_wheel_rpm, 3, 1),
	SHELL_CMD_ARG(pair, NULL,
		      "motor wheel pair <left_current_mA> <right_current_mA> [ms]",
		      cmd_motor_wheel_pair, 3, 1),
	SHELL_CMD_ARG(stop, NULL, "motor wheel stop", cmd_motor_wheel_stop,
		      1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_dm_cmds,
	SHELL_CMD_ARG(status, NULL, "motor dm status [left|right|id|all]",
		      cmd_motor_dm_status, 1, 1),
	SHELL_CMD_ARG(enable, NULL, "motor dm enable <left|right|id>",
		      cmd_motor_dm_enable, 2, 0),
	SHELL_CMD_ARG(disable, NULL, "motor dm disable <left|right|id>",
		      cmd_motor_dm_disable, 2, 0),
	SHELL_CMD_ARG(zero, NULL, "motor dm zero <left|right|id>",
		      cmd_motor_dm_zero, 2, 0),
	SHELL_CMD_ARG(reg, NULL, "motor dm reg <left|right|id> <rid>",
		      cmd_motor_dm_reg, 3, 0),
		SHELL_CMD_ARG(diag, NULL, "motor dm diag <left|right|id>", cmd_motor_dm_diag, 2, 0),
	SHELL_CMD_ARG(pos, NULL,
		      "motor dm pos <left|right|id> <rad> [vel_rad_s] [ms]",
		      cmd_motor_dm_pos, 3, 2),
	SHELL_CMD_ARG(vel, NULL, "motor dm vel <left|right|id> <rad_s> [ms]",
		      cmd_motor_dm_vel, 3, 1),
	SHELL_CMD_ARG(mit, NULL,
		      "motor dm mit <left|right|id> <pos_rad> <vel_rad_s> <kp> <kd> <torque_nm> [ms]",
		      cmd_motor_dm_mit, 7, 1),
	SHELL_CMD_ARG(nudge, NULL,
		      "motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [ms]",
		      cmd_motor_dm_nudge, 3, 3),
	SHELL_CMD_ARG(wiggle, NULL,
		      "motor dm wiggle <left|right|id> <amp_rad> [period_ms] [kp] [kd] [ms]",
		      cmd_motor_dm_wiggle, 3, 4),
	SHELL_CMD_ARG(stop, NULL, "motor dm stop <left|right|id>",
		      cmd_motor_dm_stop, 2, 0),
	SHELL_CMD(rxlog, NULL, "motor dm rxlog", cmd_motor_dm_rxlog),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_debug_cmds,
	SHELL_CMD_ARG(status, NULL, "motor debug status",
		      cmd_motor_debug_status, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "motor debug stop", cmd_motor_debug_stop,
		      1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_can_cmds,
	SHELL_CMD_ARG(status, NULL,
		      "motor can status [joint|dm|can1|wheel|m3508|can2|all]",
		      cmd_motor_can_status, 1, 1),
	SHELL_CMD_ARG(raw, NULL,
		      "motor can raw <joint|wheel|can1|can2> <std_id> [byte0..byte7]",
		      cmd_motor_can_raw, 3, 8),
	SHELL_CMD_ARG(rawx, NULL,
		      "motor can rawx <joint|wheel|can1|can2> <ext_id> [byte0..byte7]",
		      cmd_motor_can_rawx, 3, 8),
	SHELL_CMD_ARG(recover, NULL,
		      "motor can recover [joint|dm|can1|wheel|m3508|can2|all]",
		      cmd_motor_can_recover, 1, 1),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_cmds,
	SHELL_CMD(wheel, &motor_wheel_cmds, "VESC/M3508 wheel motor debug", NULL),
	SHELL_CMD(dm, &motor_dm_cmds, "DM4340 joint motor debug", NULL),
	SHELL_CMD(debug, &motor_debug_cmds, "manual motor debug state", NULL),
	SHELL_CMD(can, &motor_can_cmds, "CAN bus debug", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(motor, &motor_cmds, "single motor debug", NULL);
