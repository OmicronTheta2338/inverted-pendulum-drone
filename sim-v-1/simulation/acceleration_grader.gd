class_name AccelerationGrader
extends RefCounted

const TRACE_WINDOW := 0.15

var _previous_velocity := Vector3.ZERO
var _report_time := 0.0
var _previous_lateral_command := 0.0
var _previous_vertical_command := 0.0
var _trace_start_time := -1.0
var _trace_seeded := false


func update(
	body: RigidBody3D,
	delta: float,
	on_ground: bool,
	simulation_time: float,
	lateral_level: int,
	vertical_level: int,
	lateral_command_limit: float,
	gravity: float,
	detailed: bool,
	periodic_reporting_enabled: bool = true
) -> void:
	var acceleration := (body.linear_velocity - _previous_velocity) / maxf(delta, 1e-6)
	_previous_velocity = body.linear_velocity
	if on_ground:
		return

	var lateral_command := lateral_level / 4.0 * lateral_command_limit
	var vertical_command := vertical_level / 3.0 * gravity
	if detailed:
		_update_trace(
			acceleration,
			simulation_time,
			lateral_command,
			vertical_command
		)

	_report_time += delta
	if _report_time < 0.25 or not periodic_reporting_enabled:
		return
	_report_time = 0.0
	if detailed:
		_print_detailed_report(
			body,
			acceleration,
			simulation_time,
			lateral_command,
			vertical_command
		)
	else:
		print("[grade3] lat cmd=%6.2f true=%6.2f err=%+5.2f | up cmd=%6.2f true=%6.2f err=%+5.2f | lat_pos=%+7.2f" % [
			lateral_command,
			acceleration.x,
			acceleration.x - lateral_command,
			vertical_command,
			acceleration.y,
			acceleration.y - vertical_command,
			body.global_position.x,
		])


func _update_trace(
	acceleration: Vector3,
	simulation_time: float,
	lateral_command: float,
	vertical_command: float
) -> void:
	if not _trace_seeded:
		_previous_lateral_command = lateral_command
		_previous_vertical_command = vertical_command
		_trace_seeded = true
	if (
		absf(lateral_command - _previous_lateral_command) > 1e-3
		or absf(vertical_command - _previous_vertical_command) > 1e-3
	):
		_trace_start_time = simulation_time
		print("[step] t=%.3f lat %6.2f->%6.2f up %6.2f->%6.2f" % [
			simulation_time,
			_previous_lateral_command,
			lateral_command,
			_previous_vertical_command,
			vertical_command,
		])
		_previous_lateral_command = lateral_command
		_previous_vertical_command = vertical_command
	if _trace_start_time < 0.0:
		return
	var trace_milliseconds := (simulation_time - _trace_start_time) * 1000.0
	if trace_milliseconds <= TRACE_WINDOW * 1000.0:
		print("[trace] dt=%6.1f lat_cmd=%6.2f lat=%7.2f up_cmd=%6.2f up=%7.2f" % [
			trace_milliseconds,
			lateral_command,
			acceleration.x,
			vertical_command,
			acceleration.y,
		])
	else:
		_trace_start_time = -1.0


func _print_detailed_report(
	body: RigidBody3D,
	acceleration: Vector3,
	simulation_time: float,
	lateral_command: float,
	vertical_command: float
) -> void:
	var forward_position := -body.global_position.z
	var forward_velocity := -body.linear_velocity.z
	var yaw_degrees := rad_to_deg(body.global_transform.basis.get_euler().y)
	print("[grade] lat cmd=%6.2f true=%6.2f err=%+5.2f | up cmd=%6.2f true=%6.2f err=%+5.2f (m/s^2)" % [
		lateral_command,
		acceleration.x,
		acceleration.x - lateral_command,
		vertical_command,
		acceleration.y,
		acceleration.y - vertical_command,
	])
	print("[fwd] t=%6.2f fwd_pos=%+7.3f fwd_vel=%+7.3f | lat_pos=%+8.2f lat_vel=%+7.2f yaw_deg=%+7.2f" % [
		simulation_time,
		forward_position,
		forward_velocity,
		body.global_position.x,
		body.linear_velocity.x,
		yaw_degrees,
	])
