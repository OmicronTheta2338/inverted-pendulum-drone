class_name SensorModel
extends RefCounted

var _config: Dictionary
var _rng := RandomNumberGenerator.new()
var _history: Array = []
var _write_index := 0
var _buffer_length := 1
var _seeded := false
var _delay_imu := 1
var _delay_attitude := 3
var _delay_velocity := 8
var _delay_position := 8
var _delay_arm := 1
var _delay_arm_imu := 3
var _gyro_bias := Vector3.ZERO
var _estimated_velocity := Vector3.ZERO
var _estimated_position := Vector3.ZERO
var _navigation_initialized := false
const ARM_OBSERVER := preload("res://simulation/arm_imu_observer.gd")
var _arm_observer = ARM_OBSERVER.new()
var _arm_gyro_bias := Vector3.ZERO
var _previous_encoders: Array[float] = []
var _encoder_rates: Array[float] = []
var _vibration_phase := 0.0


func configure(config: Dictionary) -> void:
	_config = config
	_rng.seed = int(_config["seed"])

	var ticks := float(Engine.physics_ticks_per_second)
	_delay_imu = int(round(float(_config["lat_imu_ms"]) / 1000.0 * ticks))
	_delay_attitude = int(round(float(_config["lat_attitude_ms"]) / 1000.0 * ticks))
	_delay_velocity = int(round(float(_config["lat_velocity_ms"]) / 1000.0 * ticks))
	_delay_position = int(round(float(_config["lat_position_ms"]) / 1000.0 * ticks))
	_delay_arm = int(round(float(_config["lat_arm_ms"]) / 1000.0 * ticks))
	_delay_arm_imu = int(round(float(_config["lat_arm_imu_ms"]) / 1000.0 * ticks))
	_buffer_length = maxi(
		maxi(maxi(_delay_imu, _delay_attitude), maxi(_delay_velocity, _delay_position)),
		maxi(_delay_arm, _delay_arm_imu)
	) + 1
	_history.resize(_buffer_length)
	_write_index = 0
	_seeded = false
	_gyro_bias = Vector3.ZERO
	_estimated_velocity = Vector3.ZERO
	_estimated_position = Vector3.ZERO
	_navigation_initialized = false
	_arm_observer = ARM_OBSERVER.new()
	_arm_gyro_bias = Vector3.ZERO
	_previous_encoders.clear()
	_encoder_rates.clear()
	_vibration_phase = 0.0


func latency_summary() -> PackedInt32Array:
	return PackedInt32Array([
		_delay_imu,
		_delay_attitude,
		_delay_velocity,
		_delay_position,
		_delay_arm,
		_delay_arm_imu,
	])


# Add sensor imperfections to truth, store the sample in the delay ring, then
# read every channel back at its own measurement latency.
func sample(truth: Dictionary, dt: float, total_motor_thrust: float) -> Dictionary:
	var degrees_to_radians := deg_to_rad(1.0)

	# Gyro bias random walk (0.5 deg/s/sqrt(hr)) plus white noise.
	_gyro_bias += Vector3(_rng.randfn(), _rng.randfn(), _rng.randfn()) \
		* (0.5 * degrees_to_radians / sqrt(3600.0)) * sqrt(dt)
	var gyro_sigma := float(_config["sig_gyro_dps"]) * degrees_to_radians
	var gyro: Vector3 = truth["gyro"] + _gyro_bias \
		+ Vector3(_rng.randfn(), _rng.randfn(), _rng.randfn()) * gyro_sigma

	# Broadband accelerometer noise plus a throttle-coupled rotor tone.
	var acceleration: Vector3 = truth["accel_ef"] \
		+ Vector3(_rng.randfn(), _rng.randfn(), _rng.randfn()) * float(_config["sig_accel"])
	var vibration_acceleration := float(_config["vib_accel"])
	if vibration_acceleration > 0.0:
		var mass := float(_config["mass"])
		var gravity := float(_config["gravity"])
		var load := clampf(total_motor_thrust / maxf(mass * gravity, 1e-3), 0.0, 4.0)
		if load > 0.0:
			var frequency := float(_config["vib_f_hover"]) * sqrt(load)
			_vibration_phase = fposmod(_vibration_phase + TAU * frequency * dt, TAU)
			var amplitude := vibration_acceleration * load
			acceleration += Vector3(
				sin(_vibration_phase),
				sin(_vibration_phase + 2.094),
				sin(_vibration_phase + 4.189)
			) * amplitude

	var attitude_sigma := float(_config["sig_att_deg"]) * degrees_to_radians
	var attitude: Quaternion = (truth["quat"] as Quaternion) * Quaternion.from_euler(
		Vector3(_rng.randfn(), _rng.randfn(), _rng.randfn()) * attitude_sigma
	)
	# The first sample establishes the initial navigation state only. Subsequent
	# samples must never correct navigation from simulator truth. accel_ef is
	# specific force in NED, so adding +g on Down recovers kinematic acceleration.
	# Integrate the SAME noisy/vibrating sample exported to the flight stack;
	# channel delays below represent output latency, not independent observations.
	if not _navigation_initialized:
		_estimated_velocity = truth["vel_ned"]
		_estimated_position = truth["pos_ned"]
		_navigation_initialized = true
	else:
		var navigation_acceleration := acceleration + Vector3(0, 0, float(_config["gravity"]))
		_estimated_position += _estimated_velocity * dt + 0.5 * navigation_acceleration * dt * dt
		_estimated_velocity += navigation_acceleration * dt

	# Generate separate physical IMU samples, then pass ONLY measurements to
	# the observer. Arm truth angles/rates are not used for the first link.
	var theta0 := 0.0
	var theta_dot0 := 0.0
	if truth.has("arm_accel_local"):
		var ned_to_world := Basis(Vector3(0, 0, -1), Vector3(1, 0, 0), Vector3(0, -1, 0))
		var body_rotation := Basis(truth["quat"] as Quaternion)
		var body_accel := ned_to_world * (body_rotation.inverse() * acceleration)
		var body_gyro := ned_to_world * gyro
		_arm_gyro_bias += Vector3(_rng.randfn(), _rng.randfn(), _rng.randfn()) \
			* (0.5 * degrees_to_radians / sqrt(3600.0)) * sqrt(dt)
		var arm_accel: Vector3 = truth["arm_accel_local"] + Vector3(
			_rng.randfn(), _rng.randfn(), _rng.randfn()) * float(_config.get("sig_arm_accel", _config["sig_accel"]))
		var arm_gyro: Vector3 = truth["arm_gyro_local"] + _arm_gyro_bias + Vector3(
			_rng.randfn(), _rng.randfn(), _rng.randfn()) * deg_to_rad(float(_config.get("sig_arm_gyro_dps", 0.4)))
		var estimate: Vector2 = _arm_observer.update(body_accel, arm_accel,
			body_gyro, arm_gyro, dt, float(_config.get("arm_observer_tau", 0.5)), float(_config["gravity"]))
		theta0 = estimate.x
		theta_dot0 = estimate.y

	# Every subsequent encoder measures its joint relative to the previous arm.
	# Absolute angles and rates are reconstructed after the channels experience
	# their respective delays.
	var arm_count := int(_config["arm_count"])
	var encoder_step := deg_to_rad(float(_config["encoder_lsb_deg"]))
	var relative_angles: Array[float] = []
	var relative_rates: Array[float] = []
	for arm_index in range(1, arm_count):
		var relative_angle_true := float(truth["theta%d" % arm_index]) \
			- float(truth["theta%d" % (arm_index - 1)])
		var relative_angle_quantized: float = (
			round(relative_angle_true / encoder_step) * encoder_step
			if encoder_step > 0.0 else relative_angle_true
		)
		relative_angles.append(relative_angle_quantized + _rng.randfn() \
			* float(_config["sig_relative_angle_deg"]) * degrees_to_radians)
	for index in range(relative_angles.size()):
		if index >= _previous_encoders.size():
			_previous_encoders.append(relative_angles[index])
			_encoder_rates.append(0.0)
		var differentiated := wrapf(relative_angles[index] - _previous_encoders[index], -PI, PI) / maxf(dt, 1e-6)
		var rate_gain := 1.0 - exp(-dt / maxf(float(_config.get("encoder_rate_tau", 0.02)), 1e-6))
		_encoder_rates[index] += rate_gain * (differentiated - _encoder_rates[index])
		relative_rates.append(_encoder_rates[index])
		_previous_encoders[index] = relative_angles[index]

	var snapshot := {
		"accel_ef": acceleration,
		"gyro": gyro,
		"quat": attitude,
		"vel_ned": _estimated_velocity,
		"pos_ned": _estimated_position,
		"theta0": theta0,
		"tdot0": theta_dot0,
	}
	for relative_index in range(relative_angles.size()):
		snapshot["phi%d" % relative_index] = relative_angles[relative_index]
		snapshot["phidot%d" % relative_index] = relative_rates[relative_index]
	if not _seeded:
		for index in range(_buffer_length):
			_history[index] = snapshot.duplicate()
		_seeded = true
	_history[_write_index] = snapshot

	var delayed_theta0: float = _read_channel("theta0", _delay_arm_imu)
	var delayed_theta_dot0: float = _read_channel("tdot0", _delay_arm)
	var sensed := {
		"accel_ef": _read_channel("accel_ef", _delay_imu),
		"gyro": _read_channel("gyro", _delay_imu),
		"quat": _read_channel("quat", _delay_attitude),
		"vel_ned": _read_channel("vel_ned", _delay_velocity),
		"pos_ned": _read_channel("pos_ned", _delay_position),
		"theta0": delayed_theta0,
		"tdot0": delayed_theta_dot0,
	}
	var absolute_angle := delayed_theta0
	var absolute_rate := delayed_theta_dot0
	for arm_index in range(1, arm_count):
		var relative_index := arm_index - 1
		absolute_angle += _read_channel("phi%d" % relative_index, _delay_arm)
		absolute_rate += _read_channel("phidot%d" % relative_index, _delay_arm)
		sensed["theta%d" % arm_index] = absolute_angle
		sensed["tdot%d" % arm_index] = absolute_rate
	_write_index = (_write_index + 1) % _buffer_length
	return sensed


func _read_channel(key: String, depth: int):
	var read_index := (_write_index - depth + _buffer_length) % _buffer_length
	return (_history[read_index] as Dictionary)[key]
