extends Node3D
#
# Phase-8 single-arm balance test harness.
#
# The PLANT is the same Godot/Jolt physics as main.tscn: a point-mass-bob arm on
# a HingeJoint. The drone is a KINEMATIC "cart" whose lateral (world-X) motion we
# prescribe from the realised acceleration -- exactly the "acceleration is the
# control input" abstraction the real ModeAccel layer provides.
#
# This scene stands in for the drone hardware by modelling, ON THE HARNESS SIDE
# (never in the controller), the two imperfections the balance loop must survive:
#   * OUTPUT actuator: the inner (ModeAccel) loop is specified as a REFERENCE MODEL
#     H_ref(s) = 1/(tau_cl*s + 1), tau_cl ~ 8 ms, so realised accel is a first-order
#     lag of the command. The number the balance loop designs against is the
#     EFFECTIVE DELAY tau_d = tau_cl (<= 10 ms); the settling tail ("1% by 40 ms",
#     which is just tau_cl*ln(100)) is a derived restatement, not the spec.
#   * INPUT sensor noise + a fixed measurement latency fed to the controller.
#
# The controller itself is the portable C++ core, called via the ArmBalance
# GDExtension -- the SAME code that would run on the H743.
#
# Run interactively:  godot --path sim-v-1 res://balance_test.tscn
# Run headless:       BALANCE_HEADLESS=1 BALANCE_SEED=1 godot --headless \
#                        --path sim-v-1 res://balance_test.tscn

const PEND_L := 0.6
const GRAVITY := 9.8
const RUN_TIME := 15.0

# --- imperfection model (all tunable) ---
@export var latency_ms := 7.5           # fixed measurement/estimation delay
@export var act_tau_ms := 8.0           # inner-loop ref-model H_ref time const tau_cl (= effective delay tau_d)
@export var sigma_theta_deg := 0.2      # arm-angle sensor noise (1 sigma)
@export var sigma_thetadot_dps := 1.0   # arm-rate sensor noise
@export var sigma_s := 0.005            # cart-position noise [m]
@export var sigma_v := 0.02             # cart-velocity noise [m/s]
@export var noise_enabled := true

# --- scenario ---
@export var init_angle_deg := 4.0       # start this far from upright
@export var bump_time := 7.5            # mid-run cart-velocity disturbance
@export var bump_dv := 0.3              # cart velocity bump [m/s] (0 = none)

# --- controller / limits ---
@export var a_max := 12.0
@export var s_ref := 0.0
@export var use_pid := false

# --- display ---
@export var graph_window_s := 2.0       # live accel-graph visible time window [s]

# --- run control / pass thresholds ---
@export var rng_seed := 1
@export var headless := false
@export var max_angle_fail_deg := 15.0
@export var final_angle_pass_deg := 5.0

@onready var drone: RigidBody3D = $Drone
@onready var arm: RigidBody3D = $Arm
@onready var status_label: Label = $CanvasLayer/Status
@onready var graph = $CanvasLayer/AccelGraph

var ctrl                                  # ArmBalance (GDExtension)
var rng := RandomNumberGenerator.new()

var cart_s := 0.0
var cart_v := 0.0
var a_real := 0.0
var a_cmd := 0.0
var t := 0.0
var prev_theta := 0.0
var have_prev := false
var bumped := false

var act_alpha := 0.0
var lat_n := 0
var delay_buf: Array = []
var buf_idx := 0
var drone_y := 0.0
var drone_z := 0.0

# metrics
var max_angle_deg := 0.0
var sum_cmd2 := 0.0
var n_steps := 0
var saturated := false
var finished := false


func _read_env() -> void:
	if OS.get_environment("BALANCE_HEADLESS") == "1": headless = true
	var v := OS.get_environment("BALANCE_SEED");      if v != "": rng_seed = int(v)
	v = OS.get_environment("BALANCE_LATENCY_MS");     if v != "": latency_ms = float(v)
	v = OS.get_environment("BALANCE_ACT_TAU_MS");     if v != "": act_tau_ms = float(v)
	v = OS.get_environment("BALANCE_INIT_DEG");       if v != "": init_angle_deg = float(v)
	v = OS.get_environment("BALANCE_BUMP_DV");        if v != "": bump_dv = float(v)
	v = OS.get_environment("BALANCE_GRAPH_WINDOW");   if v != "": graph_window_s = float(v)
	if OS.get_environment("BALANCE_USE_PID") == "1":  use_pid = true
	if OS.get_environment("BALANCE_NO_NOISE") == "1": noise_enabled = false


func _true_theta() -> float:
	# pivot->bob direction in world; independent of cart translation (orientation only)
	var rod := arm.global_transform.basis * Vector3(0.0, -1.0, 0.0)
	return atan2(rod.x, rod.y)   # 0 = straight up, + toward +X (matches controller)


func _ready() -> void:
	_read_env()
	rng.seed = rng_seed

	# --- controller (portable C++ core via GDExtension) ---
	ctrl = ArmBalance.new()
	var cfg := {"L": PEND_L, "g": GRAVITY, "a_max": a_max, "s_ref": s_ref}
	if use_pid:
		cfg["law"] = "PID"
	ctrl.configure(cfg)

	# --- drone = kinematic cart (level, moves only along world X) ---
	drone.freeze = true
	drone.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	drone_y = drone.global_position.y
	drone_z = drone.global_position.z
	cart_s = drone.global_position.x

	# --- arm inverted (bob up) + small initial perturbation toward +X ---
	var theta0 := deg_to_rad(init_angle_deg)
	arm.gravity_scale = 1.0
	arm.global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - theta0), drone.global_position)
	arm.linear_velocity = Vector3.ZERO
	arm.angular_velocity = Vector3.ZERO

	# --- actuator lag + latency buffer ---
	var dt := 1.0 / float(Engine.physics_ticks_per_second)
	# actuator = inner-loop reference model H_ref(s) = 1/(tau_cl*s + 1)
	var tau := act_tau_ms / 1000.0
	act_alpha = 1.0 - exp(-dt / tau)
	lat_n = int(round((latency_ms / 1000.0) / dt))
	delay_buf.resize(lat_n + 1)
	var seed_meas := [theta0, 0.0, cart_s, 0.0]
	for i in range(delay_buf.size()):
		delay_buf[i] = seed_meas.duplicate()

	prev_theta = _true_theta()
	have_prev = true

	graph.setup(graph_window_s)

	print("balance_test: seed=%d latency=%.1fms noise=%d pid=%d init=%.1fdeg tau_cl=%.2fms lat_n=%d" % [
		rng_seed, latency_ms, int(noise_enabled), int(use_pid), init_angle_deg, tau * 1000.0, lat_n])


func _physics_process(dt: float) -> void:
	if finished:
		return

	# 1. true plant state (theta from Godot; theta_dot by finite difference)
	var theta := _true_theta()
	var theta_dot := 0.0
	if have_prev:
		theta_dot = (theta - prev_theta) / dt
	prev_theta = theta
	have_prev = true

	# 2. build the sensed measurement: true state + Gaussian noise
	var m_theta := theta
	var m_tdot := theta_dot
	var m_s := cart_s
	var m_v := cart_v
	if noise_enabled:
		m_theta += deg_to_rad(sigma_theta_deg) * rng.randfn()
		m_tdot  += deg_to_rad(sigma_thetadot_dps) * rng.randfn()
		m_s     += sigma_s * rng.randfn()
		m_v     += sigma_v * rng.randfn()

	# 3. push through the measurement-delay ring buffer
	delay_buf[buf_idx] = [m_theta, m_tdot, m_s, m_v]
	var ridx := (buf_idx - lat_n + delay_buf.size()) % delay_buf.size()
	var d: Array = delay_buf[ridx]
	buf_idx = (buf_idx + 1) % delay_buf.size()

	# 4. controller -> commanded cart lateral acceleration
	a_cmd = ctrl.update(d[0], d[1], d[2], d[3], dt)
	if ctrl.last_saturated():
		saturated = true

	# 5. OUTPUT actuator = reference model H_ref: realised accel chases command (first-order lag, tau_cl)
	a_real += act_alpha * (a_cmd - a_real)

	# 6. integrate the cart from the realised accel, drive the kinematic drone
	cart_v += a_real * dt
	cart_s += cart_v * dt
	drone.global_position = Vector3(cart_s, drone_y, drone_z)

	# 7. mid-run disturbance: a clean cart-velocity bump (perturbs the pivot)
	if not bumped and t >= bump_time and bump_dv != 0.0:
		cart_v += bump_dv
		bumped = true

	# 7b. live graph: target (a_cmd) vs realised (a_real) acceleration over time
	graph.push_sample(t, a_cmd, a_real)

	# 8. metrics
	var ang_deg := absf(theta) * 180.0 / PI
	if ang_deg > max_angle_deg:
		max_angle_deg = ang_deg
	sum_cmd2 += a_cmd * a_cmd
	n_steps += 1

	# 9. HUD + time
	if status_label:
		status_label.text = "t=%5.2f s\ntheta=%+6.2f deg  (max %.2f)\ns=%+5.2f m  v=%+5.2f m/s\na_cmd=%+6.2f  a_real=%+6.2f  sat=%d" % [
			t, theta * 180.0 / PI, max_angle_deg, cart_s, cart_v, a_cmd, a_real, int(saturated)]

	t += dt
	if t >= RUN_TIME:
		_finish(theta)


func _finish(theta: float) -> void:
	finished = true
	var final_deg := absf(theta) * 180.0 / PI
	var rms_cmd := sqrt(sum_cmd2 / float(max(1, n_steps)))
	var passed := (max_angle_deg < max_angle_fail_deg) and (final_deg < final_angle_pass_deg)
	print("RESULT seed=%d latency_ms=%.1f act_tau_ms=%.1f pid=%d noise=%d init_deg=%.1f | max_angle=%.2f final=%.3f rms_cmd=%.2f sat=%d => %s" % [
		rng_seed, latency_ms, act_tau_ms, int(use_pid), int(noise_enabled), init_angle_deg,
		max_angle_deg, final_deg, rms_cmd, int(saturated), ("PASS" if passed else "FAIL")])
	if status_label:
		status_label.text += "\n\n%s  (final %.2f deg, max %.2f deg)" % [
			("PASS" if passed else "FAIL"), final_deg, max_angle_deg]

	# optional CSV dump of the target/realised accel traces (for offline plotting)
	var csv_path := OS.get_environment("BALANCE_CSV")
	if csv_path != "":
		var f := FileAccess.open(csv_path, FileAccess.WRITE)
		if f:
			f.store_line("t,a_cmd,a_real")
			for i in range(graph.t_data.size()):
				f.store_line("%.5f,%.5f,%.5f" % [graph.t_data[i], graph.cmd_data[i], graph.real_data[i]])
			f.close()
			print("wrote accel CSV: %s (%d rows)" % [csv_path, graph.t_data.size()])

	if headless:
		get_tree().quit(0 if passed else 2)
