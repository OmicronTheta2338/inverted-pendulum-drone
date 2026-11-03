class_name PendulumChain
extends RefCounted

var _imu_previous_velocity := Vector3.ZERO
var _imu_previous_position := Vector3.ZERO
var _imu_samples := 0
var _imu_previous_dt := 0.0

var bodies: Array[RigidBody3D] = []
var _previous_angles: Array[float] = []
var _has_previous_sample: Array[bool] = []


# Keep the requested prefix of the scene's pendulum chain and fully detach the
# rest. Clearing both joint paths prevents an unused joint anchoring the drone
# to the world.
func configure(
	all_bodies: Array,
	all_joints: Array,
	active_count: int
) -> void:
	_imu_samples = 0
	bodies.clear()
	_previous_angles.clear()
	_has_previous_sample.clear()
	for index in range(all_bodies.size()):
		if index < active_count:
			bodies.append(all_bodies[index])
			_previous_angles.append(0.0)
			_has_previous_sample.append(false)
		else:
			all_joints[index].node_a = NodePath("")
			all_joints[index].node_b = NodePath("")
			all_joints[index].queue_free()
			all_bodies[index].queue_free()


# Measure every active link from straight down, positive toward world +X.
# Rates use the same wrapped finite difference as the original scene scripts.
func sample(delta: float, output_size: int) -> Dictionary:
	var angles: Array[float] = []
	var rates: Array[float] = []
	angles.resize(output_size)
	rates.resize(output_size)
	angles.fill(0.0)
	rates.fill(0.0)

	for index in range(bodies.size()):
		var rod := bodies[index].global_transform.basis * Vector3(0, -1, 0)
		var angle := atan2(rod.x, -rod.y)
		var rate := 0.0
		if _has_previous_sample[index]:
			rate = wrapf(angle - _previous_angles[index], -PI, PI) / maxf(delta, 1e-6)
		_previous_angles[index] = angle
		_has_previous_sample[index] = true
		angles[index] = angle
		rates[index] = rate

	return {"angles": angles, "rates": rates}


func configure_distributed_mass(
	lengths: Array[float],
	total_mass: float,
	rod_mass_fraction: float
) -> void:
	for index in range(bodies.size()):
		var rod_mass := total_mass * rod_mass_fraction
		var tip_mass := total_mass * (1.0 - rod_mass_fraction)
		var length := lengths[index]
		var center_distance := (
			rod_mass * length * 0.5 + tip_mass * length
		) / total_mass
		var swing_inertia := (
			rod_mass * length * length / 12.0
			+ rod_mass * pow(length * 0.5 - center_distance, 2)
			+ tip_mass * pow(length - center_distance, 2)
		)
		var body := bodies[index]
		body.mass = total_mass
		body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
		body.center_of_mass = Vector3(0, -center_distance, 0)
		body.inertia = Vector3(
			swing_inertia,
			maxf(swing_inertia * 1e-3, 1e-7),
			swing_inertia
		)


func apply_damping(
	parent_body: RigidBody3D,
	air_drag_coefficient: float,
	hinge_viscosity: float,
	hinge_coulomb_friction: float,
	coulomb_smoothing_rate: float
) -> void:
	if bodies.is_empty() or parent_body.global_position.y <= 0.0:
		return
	for body in bodies:
		var velocity := body.linear_velocity
		var speed := velocity.length()
		if speed > 1e-3:
			body.apply_central_force(-air_drag_coefficient * speed * velocity)

	var parent := parent_body
	for body in bodies:
		var axis := parent.global_transform.basis * Vector3(0, 0, 1)
		_apply_hinge_friction(
			parent,
			body,
			axis,
			hinge_viscosity,
			hinge_coulomb_friction,
			coulomb_smoothing_rate
		)
		parent = body


func _apply_hinge_friction(
	parent: RigidBody3D,
	child: RigidBody3D,
	axis: Vector3,
	viscosity: float,
	coulomb_friction: float,
	smoothing_rate: float
) -> void:
	var relative_rate := (child.angular_velocity - parent.angular_velocity).dot(axis)
	var smoothed_sign := clampf(relative_rate / smoothing_rate, -1.0, 1.0)
	var torque := axis * (-(viscosity * relative_rate + coulomb_friction * smoothed_sign))
	child.apply_torque(torque)
	parent.apply_torque(-torque)


# Sensor mounted along the first rod, measured from its proximal pivot.
# Difference the actual world-space sensor position, avoiding assumptions about
# the body's automatic/custom COM. Successive interval velocities give the
# local acceleration (including centripetal and tangential terms), one tick late.
func first_arm_imu(dt: float, gravity: float, distance: float) -> Dictionary:
	if bodies.is_empty() or dt <= 0.0:
		return {}
	var body := bodies[0]
	var basis := body.global_transform.basis
	var position := body.global_transform * Vector3(0, -distance, 0)
	var velocity := (position - _imu_previous_position) / dt
	var acceleration := (velocity - _imu_previous_velocity) / maxf(0.5 * (dt + _imu_previous_dt), 1e-6)
	_imu_previous_position = position
	_imu_previous_velocity = velocity
	_imu_previous_dt = dt
	_imu_samples += 1
	# Scenes start at rest: bootstrap specific force with gravity until two
	# position intervals exist. Do not output a fictitious zero arm angle.
	if _imu_samples < 3:
		acceleration = Vector3.ZERO
	return {
		"arm_accel_local": basis.inverse() * (acceleration + Vector3(0, gravity, 0)),
		"arm_gyro_local": basis.inverse() * body.angular_velocity,
	}
