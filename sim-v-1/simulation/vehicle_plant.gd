class_name VehiclePlant
extends RefCounted


static func apply_motor_forces(
	body: RigidBody3D,
	motor_positions: Array[Vector3],
	motor_directions: Array[Vector3],
	motor_spin: Array[float],
	motor_thrust: Array[float],
	reaction_torque_coefficient: float,
	advance_ratio_coefficient: float = 0.0,
	body_drag_coefficient: float = 0.0,
	rotor_gyro_coefficient: float = 0.0
) -> void:
	var basis := body.global_transform.basis
	var velocity := body.linear_velocity
	for index in range(motor_thrust.size()):
		var thrust := motor_thrust[index]
		if thrust == 0.0:
			continue
		var world_axis := basis * motor_directions[index]
		var axial_velocity := velocity.dot(world_axis)
		if axial_velocity > 0.0:
			thrust *= maxf(0.0, 1.0 - advance_ratio_coefficient * axial_velocity)
		body.apply_force(world_axis * thrust, basis * motor_positions[index])
		body.apply_torque(
			world_axis * (motor_spin[index] * reaction_torque_coefficient * thrust)
		)

	if body_drag_coefficient > 0.0:
		var speed := velocity.length()
		if speed > 1e-3:
			body.apply_central_force(-body_drag_coefficient * speed * velocity)

	if rotor_gyro_coefficient > 0.0:
		var angular_momentum := Vector3.ZERO
		for index in range(motor_thrust.size()):
			if motor_thrust[index] > 0.0:
				angular_momentum += basis * (
					motor_directions[index]
					* (rotor_gyro_coefficient * sqrt(motor_thrust[index]) * motor_spin[index])
				)
		body.apply_torque(-body.angular_velocity.cross(angular_momentum))


static func clamp_to_ground(body: RigidBody3D) -> bool:
	var on_ground := body.global_position.y <= 0.0
	if not on_ground:
		return false

	var velocity := body.linear_velocity
	if velocity.y < 0.0:
		velocity.y = 0.0
	velocity.x *= 0.8
	velocity.z *= 0.8
	body.linear_velocity = velocity
	var transform := body.global_transform
	transform.origin.y = 0.0
	body.global_transform = transform
	return true
