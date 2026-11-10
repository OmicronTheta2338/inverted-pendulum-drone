extends SceneTree

class FixedSensor:
	extends RefCounted
	func sample(truth: Dictionary, _dt: float, _thrust: float) -> Dictionary:
		var sensed := truth.duplicate()
		# NED: lateral +2 m, altitude 9 m; lateral +0.4 m/s, upward +0.2 m/s.
		sensed.pos_ned = Vector3(0, 2, -9)
		sensed.vel_ned = Vector3(0, 0.4, -0.2)
		return sensed

class CaptureStack:
	extends RefCounted
	var lateral := 0.0
	var vertical := 0.0
	func set_state(_a, _v, _p, _hv, _hp, _q, _g):
		pass
	func set_command(lat, up, _yaw, _park, _n, _theta, _rate):
		lateral = lat
		vertical = up
	func set_hold_level(_enabled):
		pass
	func update(_grounded, _dt) -> PackedInt32Array:
		return PackedInt32Array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])

var failures := 0
func _init() -> void:
	call_deferred("run_tests")

func run_tests() -> void:
	for scene_path in ["res://main.tscn", "res://main3.tscn"]:
		var scene = load(scene_path).instantiate()
		root.add_child(scene)
		scene.set_physics_process(false)
		scene.sensor_model = FixedSensor.new()
		scene.sensor_noise_enabled = true
		var stack := CaptureStack.new()
		scene.flightstack = stack
		scene.balance_alt = 10.0
		scene.balance_onboard = true
		if scene_path == "res://main.tscn":
			scene.swingup_onboard = false
			scene._bal_released = true
		else:
			scene._bal_done = true
		# Truth deliberately disagrees with the estimated state.
		scene.drone.global_position = Vector3(-4, 12, 0)
		scene.drone.linear_velocity = Vector3.ZERO
		scene._service_native(0.0025)
		check_command(stack, scene_path + " balance release")
		if scene_path == "res://main.tscn":
			scene.swingup_onboard = true
			scene.swingup_alt = 10.0
			scene.swingup_run_tracker.released = true
			scene._service_native(0.0025)
			check_command(stack, "swingup release")
		scene.free()
	if failures == 0:
		print("HOLD SENSING TESTS PASS")
	quit(1 if failures else 0)

func check_command(stack: CaptureStack, label: String) -> void:
	if absf(stack.lateral - (-4.2 / 12.0)) > 1e-5 or absf(stack.vertical - (1.6 / 9.8)) > 1e-5:
		failures += 1
		push_error(label + " must use estimated position and velocity for both hold axes")
