class_name PilotInput
extends RefCounted

const AXIS_NONE := 0
const AXIS_LATERAL := 1
const AXIS_VERTICAL := 2

const LATERAL_LEVELS := {
	KEY_A: -4,
	KEY_S: -3,
	KEY_D: -2,
	KEY_F: -1,
	KEY_G: 0,
	KEY_H: 1,
	KEY_J: 2,
	KEY_K: 3,
	KEY_L: 4,
}

const VERTICAL_LEVELS := {
	KEY_Z: -3,
	KEY_X: -2,
	KEY_C: -1,
	KEY_V: 0,
	KEY_B: 1,
	KEY_N: 2,
	KEY_M: 3,
}


# Return (axis, level). AXIS_NONE distinguishes an unmapped key from a mapped
# center key whose commanded level is zero.
static func decode_acceleration_key(keycode: Key) -> Vector2i:
	if LATERAL_LEVELS.has(keycode):
		return Vector2i(AXIS_LATERAL, int(LATERAL_LEVELS[keycode]))
	if VERTICAL_LEVELS.has(keycode):
		return Vector2i(AXIS_VERTICAL, int(VERTICAL_LEVELS[keycode]))
	return Vector2i(AXIS_NONE, 0)


static func yaw_direction() -> int:
	var direction := 0
	if Input.is_key_pressed(KEY_E):
		direction += 1
	if Input.is_key_pressed(KEY_Q):
		direction -= 1
	return direction
