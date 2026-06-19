extends Node3D
#
# Phase-8 DOUBLE INVERTED PENDULUM balance harness.
#
# A serial double pendulum on the drone-cart: link1 (0.7 m) hinges to the drone,
# link2 (0.6 m) hinges to link1's TIP. Both links are balanced UPRIGHT by the drone's
# single lateral acceleration u = a_x (under-actuated: one input, two links).
#
# Controller: the portable C++ core (TwoArmBalanceController) via the ArmBalance2
# GDExtension -- one MIMO-LQR on x = [th1, w1, th2, w2, s, v] -> u, where th1/th2 are
# the two links' ABSOLUTE angles from vertical. Same imperfection model as the single
# arm: actuator ref-model H_ref (tau_cl), sensor noise, fixed measurement latency.
#
# Default start: a small COMMON lean (both links +4 deg) -- the natural perturbation
# of the upright stack, recovered robustly. A fully "scissored" start (set
# BALANCE2_INIT2_DEG=-5) folds the links and is the hard/near-nonlinear case.
#
# Run:      godot --path sim-v-1 res://balance2_test.tscn
# Headless: BALANCE2_HEADLESS=1 BALANCE2_SEED=1 godot --headless --path sim-v-1 res://balance2_test.tscn

const GRAVITY := 9.8
const RUN_TIME := 15.0

# MIMO-LQR gains for the double inverted pendulum (compute_lqr2.py, R=1).
const GAINS := {"k1": -213.542773, "k1d": -14.504648, "k2": 260.829154,
	"k2d": 45.149116, "ks": 1.414214, "kv": 4.525040}

# --- geometry ---
@export var L1 := 0.7                     # link1 length [m]
@export var L2 := 0.6                     # link2 length [m]

# --- imperfection model ---
@export var latency_ms := 7.5
@export var act_tau_ms := 8.0
@export var sigma_theta_deg := 0.2
@export var sigma_thetadot_dps := 1.0
@export var sigma_s := 0.005
@export var sigma_v := 0.02
@export var noise_enabled := true

# --- scenario ---
@export var init_angle_deg := 4.0         # link1 start
@export var init_angle2_deg := 4.0        # link2 start (=link1 -> common lean; try -5 for scissored)
@export var bump_time := 7.5
@export var bump_dv := 0.3

# --- controller / limits ---
@export var a_max := 12.0

# --- display ---
@export var graph_window_s := 2.0

# --- run control / pass thresholds ---
@export var rng_seed := 1
@export var headless := false
@export var max_angle_fail_deg := 15.0
@export var final_angle_pass_deg := 5.0

@onready var drone: RigidBody3D = $Drone
@onready var link1: RigidBody3D = $Link1
@onready var link2: RigidBody3D = $Link2
@onready var hinge2: HingeJoint3D = $Hinge2
@onready var status_label: Label = $CanvasLayer/Status
@onready var graph_u = $CanvasLayer/GraphU
@onready var graph_ang = $CanvasLayer/GraphAng

var ctrl                                   # ArmBalance2 (GDExtension)
var rng := RandomNumberGenerator.new()
var cart_s := 0.0
var cart_v := 0.0
var a_real := 0.0
var a_cmd := 0.0
var t := 0.0
var prev_th1 := 0.0
var prev_th2 := 0.0
var have_prev := false
var bumped := false
var act_alpha := 0.0
var lat_n := 0
var delay_buf: Array = []
var buf_idx := 0
var drone_y := 0.0
var drone_z := 0.0
var max1 := 0.0
var max2 := 0.0
var sum_cmd2 := 0.0
var n_steps := 0
var sat := false
var finished := false
var th1_log: PackedFloat32Array = PackedFloat32Array()
var th2_log: PackedFloat32Array = PackedFloat32Array()


func _read_env() -> void:
	if OS.get_environment("BALANCE2_HEADLESS") == "1": headless = true
	var v := OS.get_environment("BALANCE2_SEED");        if v != "": rng_seed = int(v)
	v = OS.get_environment("BALANCE2_LATENCY_MS");        if v != "": latency_ms = float(v)
	v = OS.get_environment("BALANCE2_ACT_TAU_MS");        if v != "": act_tau_ms = float(v)
	v = OS.get_environment("BALANCE2_INIT_DEG");          if v != "": init_angle_deg = float(v)
	v = OS.get_environment("BALANCE2_INIT2_DEG");         if v != "": init_angle2_deg = float(v)
	v = OS.get_environment("BALANCE2_BUMP_DV");           if v != "": bump_dv = float(v)
	v = OS.get_environment("BALANCE2_GRAPH_WINDOW");      if v != "": graph_window_s = float(v)
	if OS.get_environment("BALANCE2_NO_NOISE") == "1":    noise_enabled = false


func _theta1() -> float:
	var rod := link1.global_transform.basis * Vector3(0.0, -1.0, 0.0)
	return atan2(rod.x, rod.y)


func _theta2() -> float:
	var rod := link2.global_transform.basis * Vector3(0.0, -1.0, 0.0)
	return atan2(rod.x, rod.y)


func _ready() -> void:
	_read_env()
	rng.seed = rng_seed
	var dt := 1.0 / float(Engine.physics_ticks_per_second)
	var th0_1 := deg_to_rad(init_angle_deg)
	var th0_2 := deg_to_rad(init_angle2_deg)

	# --- controller (portable C++ core via GDExtension) ---
	ctrl = ArmBalance2.new()
	var cfg := {"L1": L1, "L2": L2, "g": GRAVITY, "a_max": a_max, "s_ref": 0.0}
	for k in GAINS:
		cfg[k] = GAINS[k]
	ctrl.configure(cfg)

	# --- drone = kinematic cart, moves only along world X ---
	drone.freeze = true
	drone.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	drone_y = drone.global_position.y
	drone_z = drone.global_position.z
	cart_s = drone.global_position.x

	# --- link1 inverted from the drone; link2 inverted from link1's TIP ---
	link1.global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - th0_1), drone.global_position)
	link1.linear_velocity = Vector3.ZERO
	link1.angular_velocity = Vector3.ZERO
	var tip := link1.global_transform * Vector3(0.0, -L1, 0.0)   # bob1 world position
	link2.global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - th0_2), tip)
	link2.linear_velocity = Vector3.ZERO
	link2.angular_velocity = Vector3.ZERO
	hinge2.global_transform = Transform3D(Basis.IDENTITY, tip)

	# --- actuator lag + latency buffer (single input, 6-state measurement) ---
	act_alpha = 1.0 - exp(-dt / (act_tau_ms / 1000.0))
	lat_n = int(round((latency_ms / 1000.0) / dt))
	delay_buf.resize(lat_n + 1)
	var seed_meas := [th0_1, 0.0, th0_2, 0.0, cart_s, 0.0]
	for i in range(delay_buf.size()):
		delay_buf[i] = seed_meas.duplicate()

	prev_th1 = _theta1(); prev_th2 = _theta2(); have_prev = true

	graph_u.setup(graph_window_s); graph_u.title = "single input a_x [m/s^2]"
	graph_ang.setup(graph_window_s)
	graph_ang.title = "link angles [deg]"
	graph_ang.label1 = "link1 theta (0.7 m)"
	graph_ang.label2 = "link2 theta (0.6 m)"

	print("balance2_test (DOUBLE PENDULUM): seed=%d latency=%.1fms tau=%.1f L1=%.2f L2=%.2f noise=%d init=(%.1f,%.1f)deg" % [
		rng_seed, latency_ms, act_tau_ms, L1, L2, int(noise_enabled), init_angle_deg, init_angle2_deg])


func _physics_process(dt: float) -> void:
	if finished:
		return

	# 1. true absolute angles + rates (finite difference)
	var th1 := _theta1()
	var th2 := _theta2()
	var w1 := 0.0
	var w2 := 0.0
	if have_prev:
		w1 = (th1 - prev_th1) / dt
		w2 = (th2 - prev_th2) / dt
	prev_th1 = th1; prev_th2 = th2; have_prev = true

	# 2. noisy measurement of the 6-state
	var m := [th1, w1, th2, w2, cart_s, cart_v]
	if noise_enabled:
		m[0] += deg_to_rad(sigma_theta_deg) * rng.randfn()
		m[1] += deg_to_rad(sigma_thetadot_dps) * rng.randfn()
		m[2] += deg_to_rad(sigma_theta_deg) * rng.randfn()
		m[3] += deg_to_rad(sigma_thetadot_dps) * rng.randfn()
		m[4] += sigma_s * rng.randfn()
		m[5] += sigma_v * rng.randfn()

	# 3. measurement-delay ring buffer
	delay_buf[buf_idx] = m
	var ridx := (buf_idx - lat_n + delay_buf.size()) % delay_buf.size()
	var d: Array = delay_buf[ridx]
	buf_idx = (buf_idx + 1) % delay_buf.size()

	# 4. controller -> single commanded cart lateral accel
	a_cmd = ctrl.update(d[0], d[1], d[2], d[3], d[4], d[5], dt)
	if ctrl.last_saturated():
		sat = true

	# 5. actuator ref-model lag, then integrate the single cart, drive the drone in X
	a_real += act_alpha * (a_cmd - a_real)
	cart_v += a_real * dt
	cart_s += cart_v * dt
	drone.global_position = Vector3(cart_s, drone_y, drone_z)

	# 6. mid-run cart-velocity disturbance
	if not bumped and t >= bump_time and bump_dv != 0.0:
		cart_v += bump_dv; bumped = true

	# 7. graphs + metrics
	graph_u.push_sample(t, a_cmd, a_real)
	graph_ang.push_sample(t, th1 * 180.0 / PI, th2 * 180.0 / PI)
	th1_log.append(th1 * 180.0 / PI)
	th2_log.append(th2 * 180.0 / PI)
	max1 = maxf(max1, absf(th1) * 180.0 / PI)
	max2 = maxf(max2, absf(th2) * 180.0 / PI)
	sum_cmd2 += a_cmd * a_cmd
	n_steps += 1

	if status_label:
		status_label.text = "t=%5.2f s   DOUBLE PENDULUM (1 input, 2 links)\nlink1 (0.7m): theta=%+6.2f deg  max %.2f\nlink2 (0.6m): theta=%+6.2f deg  max %.2f\na_x cmd=%+6.2f  real=%+6.2f  sat=%d   s=%+5.2f m" % [
			t, th1 * 180.0 / PI, max1, th2 * 180.0 / PI, max2, a_cmd, a_real, int(sat), cart_s]

	t += dt
	if t >= RUN_TIME:
		_finish(th1, th2)


func _finish(th1: float, th2: float) -> void:
	finished = true
	var f1 := absf(th1) * 180.0 / PI
	var f2 := absf(th2) * 180.0 / PI
	var rms := sqrt(sum_cmd2 / float(max(1, n_steps)))
	var passed := (max1 < max_angle_fail_deg) and (max2 < max_angle_fail_deg) \
		and (f1 < final_angle_pass_deg) and (f2 < final_angle_pass_deg)

	print("RESULT2C seed=%d latency_ms=%.1f tau=%.1f noise=%d | link1 max=%.2f final=%.3f | link2 max=%.2f final=%.3f | rms_u=%.2f sat=%d => %s" % [
		rng_seed, latency_ms, act_tau_ms, int(noise_enabled),
		max1, f1, max2, f2, rms, int(sat), ("PASS" if passed else "FAIL")])
	if status_label:
		status_label.text += "\n\n%s  (link1 final %.2f max %.2f | link2 final %.2f max %.2f | rms_u %.2f sat %d)" % [
			("PASS" if passed else "FAIL"), f1, max1, f2, max2, rms, int(sat)]

	var csv_path := OS.get_environment("BALANCE2_CSV")
	if csv_path != "":
		var f := FileAccess.open(csv_path, FileAccess.WRITE)
		if f:
			f.store_line("t,a_cmd,a_real,theta1_deg,theta2_deg")
			var tt: PackedFloat32Array = graph_u.t_data
			var cc: PackedFloat32Array = graph_u.cmd_data
			var rr: PackedFloat32Array = graph_u.real_data
			for i in range(tt.size()):
				f.store_line("%.5f,%.5f,%.5f,%.5f,%.5f" % [tt[i], cc[i], rr[i], th1_log[i], th2_log[i]])
			f.close()
			print("wrote accel2 CSV: %s (%d rows)" % [csv_path, tt.size()])

	if headless:
		get_tree().quit(0 if passed else 2)
