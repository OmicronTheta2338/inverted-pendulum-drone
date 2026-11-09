extends SceneTree
const OBSERVER := preload("res://simulation/arm_imu_observer.gd")
const CHAIN := preload("res://simulation/pendulum_chain.gd")
const SENSOR := preload("res://simulation/sensor_model.gd")
var failures := 0
func check(ok: bool, message: String) -> void:
	if not ok:
		failures += 1
		push_error(message)

func _init() -> void:
	call_deferred("run_tests")

func run_tests() -> void:
	var observer = OBSERVER.new()
	var g := Vector3(0, 9.81, 0)
	var angle := 0.7
	var local_g := Basis(Vector3.FORWARD, angle) * g
	var result: Vector2 = observer.update(g, local_g, Vector3.ZERO, Vector3.ZERO, 0.01, 0.5, 9.81)
	check(absf(result.x - angle) < 1e-5, "static arm tilt is recovered from local gravity, without pose input")
	# No useful gravity in freefall: retain gyro integration, never snap to truth.
	for i in range(100):
		result = observer.update(Vector3.ZERO, Vector3.ZERO, Vector3(0, 0, 0.3), Vector3(0, 0, 1), 0.01, 0.5, 9.81)
	check(absf(result.x - 1.7) < 1e-4 and result.y == 1.0, "freefall propagates gyro and handles body rotation")
	observer = OBSERVER.new()
	observer.update(g, g, Vector3.ZERO, Vector3.ZERO, 0.01, 0.5, 9.81)
	for i in range(200):
		result = observer.update(g, g + Vector3(2, 0, 0), Vector3.ZERO, Vector3.ZERO, 0.01, 0.5, 9.81)
	check(result.x > 0.15, "sustained acceleration biases the gravity anchor physically")
	observer = OBSERVER.new()
	observer.update(g, g, Vector3.ZERO, Vector3.ZERO, 0.01, 0.5, 9.81)
	for i in range(100):
		result = observer.update(g, Vector3(30, 0, 0), Vector3.ZERO, Vector3.ZERO, 0.01, 0.5, 9.81)
	check(absf(result.x) < 1e-6, "large dynamic acceleration is rejected")
	# Sensor integration: first-arm output must be independent of truth angle/rate.
	var settings := config()
	var input := truth()
	settings.sig_arm_accel = 0.0
	settings.sig_arm_gyro_dps = 0.0
	input.arm_accel_local = local_g
	input.arm_gyro_local = Vector3.ZERO
	var model = SENSOR.new()
	model.configure(settings)
	model.sample(input, 0.01, 0.0)
	input.theta0 = -2.0
	input.tdot0 = 100.0
	var sensed: Dictionary = model.sample(input, 0.01, 0.0)
	check(absf(sensed.theta0 - angle) < 1e-4 and absf(sensed.tdot0) < 0.001,
		"arm estimate never reads simulator angle or rate")
	settings.arm_count = 2
	settings.encoder_rate_tau = 0.000001
	model.configure(settings)
	input.theta0 = 0.0
	input.theta1 = 0.0
	input.tdot1 = 1000.0
	model.sample(input, 0.01, 0.0)
	input.theta1 = 0.01
	sensed = model.sample(input, 0.01, 0.0)
	check(absf(sensed.tdot1 - sensed.tdot0 - 1.0) < 1e-4,
		"encoder rate differentiates readings rather than truth rate")
	var chain = CHAIN.new()
	var body := RigidBody3D.new()
	body.freeze = true
	root.add_child(body)
	chain.bodies.append(body)
	var raw: Dictionary
	for i in range(3):
		var t := i * 0.01
		body.position = Vector3(t * t, 0, 0)
		raw = chain.first_arm_imu(0.01, 9.81, 0.1)
	check(raw.arm_accel_local.distance_to(Vector3(2, 9.81, 0)) < 1e-3,
		"arm IMU measures translating pivot acceleration")
	chain = CHAIN.new()
	chain.bodies.append(body)
	body.position = Vector3.ZERO
	for i in range(3):
		body.rotation.z = i * 0.01
		raw = chain.first_arm_imu(0.01, 9.81, 0.1)
	var dynamic_accel: Vector3 = raw.arm_accel_local - body.basis.inverse() * g
	check(absf(dynamic_accel.y - 0.1) < 0.002,
		"off-pivot IMU captures centripetal acceleration")
	body.free()
	if failures == 0:
		print("ARM IMU TESTS PASS")
	quit(1 if failures else 0)

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

