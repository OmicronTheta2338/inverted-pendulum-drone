class_name ArmImuObserver
extends RefCounted

# Planar two-IMU complementary observer. Inputs use the Godot sensor axes
# (+X lateral, +Y up, +Z hinge). No simulator pose or angular rate enters here.
var body_angle := 0.0
var relative_angle := 0.0
var initialized := false

func update(body_accel: Vector3, arm_accel: Vector3, body_gyro: Vector3,
		arm_gyro: Vector3, dt: float, tau: float, gravity: float) -> Vector2:
	var body_anchor := atan2(body_accel.x, body_accel.y)
	var arm_anchor := atan2(arm_accel.x, arm_accel.y)
	# Reject near-freefall and large non-gravitational loads. This cannot detect
	# all sustained acceleration: that ambiguity is a real accelerometer limit.
	var body_valid := absf(body_accel.length() - gravity) < 0.25 * gravity
	var arm_valid := absf(arm_accel.length() - gravity) < 0.25 * gravity
	if not initialized:
		body_angle = body_anchor if body_valid else 0.0
		relative_angle = wrapf(arm_anchor - body_angle, -PI, PI) if arm_valid else 0.0
		initialized = true
	else:
		body_angle += body_gyro.z * dt
		relative_angle += (arm_gyro.z - body_gyro.z) * dt
		var gain := 1.0 - exp(-dt / maxf(tau, 1e-6))
		if body_valid:
			body_angle += gain * wrapf(body_anchor - body_angle, -PI, PI)
		if body_valid and arm_valid:
			relative_angle += gain * wrapf(arm_anchor - body_anchor - relative_angle, -PI, PI)
	return Vector2(wrapf(body_angle + relative_angle, -PI, PI), arm_gyro.z)
