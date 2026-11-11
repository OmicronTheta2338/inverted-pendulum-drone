extends SceneTree

const MOTOR_DYNAMICS := preload("res://simulation/motor_dynamics.gd")
const SITL_PROTOCOL := preload("res://simulation/sitl_protocol.gd")
const PILOT_INPUT := preload("res://simulation/pilot_input.gd")
const SWINGUP_RUN_TRACKER := preload("res://simulation/swingup_run_tracker.gd")
const SCENARIO_VISUALS := preload("res://simulation/scenario_visuals.gd")

var _failures := 0


func _init() -> void:
	_test_pwm_conversion()
	_test_motor_lag()
	_test_servo_packet_decoding()
	_test_state_packet_encoding()
	_test_acceleration_key_decoding()
	_test_swingup_run_tracking()
	_test_analysis_dashboard()
	if _failures == 0:
		print("SIMULATION COMPONENT TESTS PASS")
		quit(0)
	else:
		push_error("%d simulation component test(s) failed" % _failures)
		quit(1)


func _test_pwm_conversion() -> void:
	_check(is_equal_approx(MOTOR_DYNAMICS.pwm_to_thrust(1000, 19.3, 0.35), 0.0),
		"minimum PWM produces zero thrust")
	_check(is_equal_approx(MOTOR_DYNAMICS.pwm_to_thrust(2000, 19.3, 0.35), 19.3),
		"maximum PWM produces maximum thrust")
	var midpoint := 19.3 * (0.65 * 0.5 + 0.35 * 0.25)
	_check(is_equal_approx(MOTOR_DYNAMICS.pwm_to_thrust(1500, 19.3, 0.35), midpoint),
		"thrust expo matches the original curve")


func _test_motor_lag() -> void:
	var actual: Array[float] = [0.0]
	var commanded: Array[float] = [10.0]
	MOTOR_DYNAMICS.apply_first_order_lag(actual, commanded, 0.03, 0.03)
	_check(is_equal_approx(actual[0], 10.0 * (1.0 - exp(-1.0))),
		"first-order lag uses the configured time constant")

	actual[0] = 5.0
	commanded[0] = 10.0
	var sag := MOTOR_DYNAMICS.apply_lag_with_battery_sag(
		actual, commanded, 0.03, 0.03, 0.045, 0.001, 0.7
	)
	_check(is_equal_approx(sag, 0.995 * 0.995),
		"battery sag uses previous-tick realized thrust")


func _test_servo_packet_decoding() -> void:
	_check(SITL_PROTOCOL.decode_motor_pwm(PackedByteArray()).is_empty(),
		"short servo packets are rejected")
	var packet := PackedByteArray()
	packet.resize(40)
	packet.encode_u16(0, SITL_PROTOCOL.SERVO_MAGIC)
	for index in range(SITL_PROTOCOL.MOTOR_COUNT):
		packet.encode_u16(8 + index * 2, 1100 + index)
	var pwm := SITL_PROTOCOL.decode_motor_pwm(packet)
	_check(pwm.size() == 8 and pwm[0] == 1100 and pwm[7] == 1107,
		"valid servo packets decode all motor channels")


func _test_state_packet_encoding() -> void:
	var packet := SITL_PROTOCOL.encode_state(
		1.25,
		true,
		Vector3(1, 2, 3),
		Vector3(4, 5, 6),
		Vector3(7, 8, 9),
		Vector3(10, 11, 12),
		Quaternion(0.1, 0.2, 0.3, 0.9),
		[1400, 1500, 1600, 1700],
		[1200, 1300, 1400, 1500, 1600]
	)
	var text := packet.get_string_from_utf8()
	var state = JSON.parse_string(text)
	_check(text.ends_with("\n"), "state packet is newline-delimited")
	_check(state is Dictionary and state["timestamp"] == 1.25 and state["no_lockstep"],
		"state packet preserves timing fields")
	_check(state["imu"]["gyro"] == [1.0, 2.0, 3.0],
		"state packet preserves IMU axes")
	_check(state["rc"]["rc_4"] == 1700.0 and state["rc"]["rc_10"] == 1600.0,
		"state packet preserves flight and pendulum RC channels")


func _test_acceleration_key_decoding() -> void:
	_check(PILOT_INPUT.decode_acceleration_key(KEY_A) == Vector2i(PILOT_INPUT.AXIS_LATERAL, -4),
		"lateral keys map to latched levels")
	_check(PILOT_INPUT.decode_acceleration_key(KEY_V) == Vector2i(PILOT_INPUT.AXIS_VERTICAL, 0),
		"vertical center remains distinguishable from an unmapped key")
	_check(PILOT_INPUT.decode_acceleration_key(KEY_ESCAPE).x == PILOT_INPUT.AXIS_NONE,
		"unmapped keys do not change acceleration")


func _test_swingup_run_tracking() -> void:
	var tracker = SWINGUP_RUN_TRACKER.new()
	tracker.configure(1.0, 5.0)
	tracker.update(0.5, true, 2.0)
	tracker.update(0.5, true, 4.0)
	tracker.update(0.5, true, 3.0)
	_check(tracker.caught and tracker.catch_time == 0.5,
		"swing-up tracker records the first catch")
	_check(tracker.released and tracker.passed,
		"swing-up tracker releases after a passing hold")
	_check(tracker.maximum_after_catch == 4.0 and tracker.maximum_during_hold == 4.0,
		"swing-up tracker preserves grading maxima")


func _test_analysis_dashboard() -> void:
	var canvas := CanvasLayer.new()
	root.add_child(canvas)
	var default_graphs := SCENARIO_VISUALS.create_recording_graphs(canvas, 2.5, false)
	_check(default_graphs.size() == 1,
		"default recording overlay contains only lateral acceleration tracking")
	_check(default_graphs[0].anchor_bottom <= 0.25,
		"default acceleration chart leaves the vehicle visible")
	var response_graphs := SCENARIO_VISUALS.create_recording_graphs(canvas, 6.0, true)
	_check(response_graphs.size() == 2,
		"response-video overlay contains lateral and vertical acceleration charts")
	_check(response_graphs[0].anchor_right < response_graphs[1].anchor_left,
		"response-video charts do not overlap")
	_check(response_graphs[0].y_unit == "m/s²" and response_graphs[1].y_unit == "m/s²",
		"response-video charts use homogeneous acceleration units")
	_check(response_graphs[0].window_s >= 6.0 and response_graphs[1].window_s >= 6.0,
		"response-video charts retain the arm-drop transient")
	var angle_label := SCENARIO_VISUALS.create_angle_readout(canvas)
	SCENARIO_VISUALS.update_angle_readout(
		angle_label,
		[1.25, -2.5, 0.0],
		false,
		"HOLD POSITION",
		true
	)
	_check("link 3" in angle_label.text and "[deg]" in angle_label.text,
		"arm angles are displayed as compact degree values")
	_check("Pendulum balance: OFF" in angle_label.text
			and "Vehicle command: HOLD POSITION" in angle_label.text
			and "Reaction feed-forward: ON" in angle_label.text,
		"recording overlay explains the active control state")
	canvas.free()


func _check(condition: bool, description: String) -> void:
	if not condition:
		_failures += 1
		push_error("FAIL: " + description)
