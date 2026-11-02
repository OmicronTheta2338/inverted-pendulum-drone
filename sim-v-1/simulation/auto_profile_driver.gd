class_name AutoProfileDriver
extends RefCounted

var _phase := 0
var _profile_start_time := 0.0
var _park := false
var _lateral_level := 0
var _vertical_level := 1


func update(
	altitude: float,
	vertical_velocity: float,
	simulation_time: float,
	profile: Array,
	log_prefix: String,
	include_vertical_velocity_in_log: bool
) -> Dictionary:
	match _phase:
		0:
			_park = false
			_lateral_level = 0
			_vertical_level = 1
			if altitude > 25.0:
				_phase = 1
				_profile_start_time = simulation_time
				if include_vertical_velocity_in_log:
					print("%s airborne (alt=%.1f, vy=%.1f); starting profile" % [
						log_prefix, altitude, vertical_velocity
					])
				else:
					print("%s airborne (alt=%.1f); starting profile" % [log_prefix, altitude])
		1:
			_park = false
			var profile_time := simulation_time - _profile_start_time
			var active := false
			for step in profile:
				if profile_time < float(step[0]):
					_lateral_level = int(step[1])
					_vertical_level = int(step[2])
					active = true
					break
			if not active:
				_phase = 2
				print("%s profile complete at t=%.1f" % [log_prefix, profile_time])
		2:
			_park = true
			_lateral_level = 0
			_vertical_level = 0
	return {
		"park": _park,
		"lateral_level": _lateral_level,
		"vertical_level": _vertical_level,
	}
