class_name SitlProtocol
extends RefCounted

const SERVO_MAGIC := 18458
const SERVO_PACKET_MINIMUM_SIZE := 40
const MOTOR_COUNT := 8


static func decode_motor_pwm(packet: PackedByteArray) -> PackedInt32Array:
	if packet.size() < SERVO_PACKET_MINIMUM_SIZE or packet.decode_u16(0) != SERVO_MAGIC:
		return PackedInt32Array()
	var pwm := PackedInt32Array()
	pwm.resize(MOTOR_COUNT)
	for index in range(MOTOR_COUNT):
		pwm[index] = packet.decode_u16(8 + index * 2)
	return pwm


static func encode_state(
	timestamp: float,
	no_lockstep: bool,
	gyro_frd: Vector3,
	acceleration_frd: Vector3,
	position_ned: Vector3,
	velocity_ned: Vector3,
	attitude_ned_frd: Quaternion,
	rc: Array,
	pendulum_rc: Array
) -> PackedByteArray:
	var state := {
		"timestamp": timestamp,
		"no_lockstep": no_lockstep,
		"imu": {
			"gyro": [gyro_frd.x, gyro_frd.y, gyro_frd.z],
			"accel_body": [acceleration_frd.x, acceleration_frd.y, acceleration_frd.z],
		},
		"position": [position_ned.x, position_ned.y, position_ned.z],
		"velocity": [velocity_ned.x, velocity_ned.y, velocity_ned.z],
		"quaternion": [
			attitude_ned_frd.w,
			attitude_ned_frd.x,
			attitude_ned_frd.y,
			attitude_ned_frd.z,
		],
		"rc": {
			"rc_1": rc[0],
			"rc_2": rc[1],
			"rc_3": rc[2],
			"rc_4": rc[3],
			"rc_6": pendulum_rc[0],
			"rc_7": pendulum_rc[1],
			"rc_8": pendulum_rc[2],
			"rc_9": pendulum_rc[3],
			"rc_10": pendulum_rc[4],
		},
	}
	return (JSON.stringify(state) + "\n").to_utf8_buffer()
