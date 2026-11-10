extends SceneTree

const SENSOR := preload("res://simulation/sensor_model.gd")
var failures := 0

func config() -> Dictionary:
	return {
		"seed": 12, "sig_gyro_dps": 0.0, "sig_accel": 0.0,
		"vib_accel": 0.0, "vib_f_hover": 90.0, "sig_att_deg": 0.0,
		"sig_theta0_deg": 0.0, "sig_thetadot0_dps": 0.0,
		"encoder_lsb_deg": 0.0, "sig_relative_angle_deg": 0.0,
		"sig_relative_rate_dps": 0.0, "lat_imu_ms": 0.0,
		"lat_attitude_ms": 0.0, "lat_velocity_ms": 0.0,
		"lat_position_ms": 0.0, "lat_arm_ms": 0.0, "lat_arm_imu_ms": 0.0,
		"mass": 1.0, "gravity": 9.81, "anchor_gain": 0.0,
		"anchor_time_constant": 1.0, "arm_count": 1,
	}

func truth() -> Dictionary:
	return {"accel_ef": Vector3(0, 0, -9.81), "gyro": Vector3.ZERO,
		"quat": Quaternion.IDENTITY, "pos_ned": Vector3.ZERO,
		"vel_ned": Vector3.ZERO, "theta0": 0.0, "tdot0": 0.0}

func check(ok: bool, message: String) -> void:
	if not ok:
		failures += 1
		push_error(message)

func _init() -> void:
	var model = SENSOR.new()
	var settings := config()
	model.configure(settings)
	var input := truth()
	model.sample(input, 0.01, 9.81)
	var result: Dictionary
	for i in range(100):
		result = model.sample(input, 0.01, 9.81)
	check(result.vel_ned.length() < 1e-6 and result.pos_ned.length() < 1e-6,
		"stationary specific force must cancel gravity")
	input.accel_ef += Vector3(2, 0, 0)
	input.vel_ned = Vector3(999, 999, 999)
	input.pos_ned = Vector3(-999, -999, -999)
	for i in range(100):
		result = model.sample(input, 0.01, 9.81)
	check(result.vel_ned.distance_to(Vector3(2, 0, 0)) < 1e-4,
		"constant acceleration integrates velocity without truth correction")
	check(result.pos_ned.distance_to(Vector3(1, 0, 0)) < 1e-4,
		"constant acceleration integrates position without truth correction")
	# Replay exported IMU samples: both navigation channels must carry exactly
	# their accumulated white noise and rotor vibration, not fresh output noise.
	settings.sig_accel = 0.25
	settings.vib_accel = 0.35
	model.configure(settings)
	input = truth()
	model.sample(input, 0.0025, 9.81)
	var velocity := Vector3.ZERO
	var position := Vector3.ZERO
	for i in range(400):
		result = model.sample(input, 0.0025, 9.81)
		var a: Vector3 = result.accel_ef + Vector3(0, 0, 9.81)
		position += velocity * 0.0025 + 0.5 * a * 0.0025 * 0.0025
		velocity += a * 0.0025
	check(result.vel_ned.distance_to(velocity) < 1e-6,
		"velocity must integrate the exported noisy IMU")
	check(result.pos_ned.distance_to(position) < 1e-6 and position.length() > 1e-5,
		"position must accumulate IMU error")
	model.configure(config())
	result = model.sample(truth(), 0.01, 9.81)
	check(result.pos_ned == Vector3.ZERO and result.vel_ned == Vector3.ZERO,
		"reconfiguration resets navigation")
	# Each output delay selects an older integrated state, including startup fill.
	settings = config()
	settings.lat_velocity_ms = 2000.0 / Engine.physics_ticks_per_second
	settings.lat_position_ms = 3000.0 / Engine.physics_ticks_per_second
	model.configure(settings)
	input = truth()
	model.sample(input, 0.01, 9.81)
	input.accel_ef += Vector3(2, 0, 0)
	for i in range(10):
		result = model.sample(input, 0.01, 9.81)
	check(absf(result.vel_ned.x - 0.16) < 1e-5,
		"velocity output respects its two-sample delay")
	check(absf(result.pos_ned.x - 0.0049) < 1e-5,
		"position output respects its three-sample delay")
	if failures == 0:
		print("SENSOR NAVIGATION TESTS PASS")
	quit(1 if failures else 0)
