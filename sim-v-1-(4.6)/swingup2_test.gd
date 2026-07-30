extends Node3D
#
# DOUBLE-pendulum swing-up trajectory REPLAY harness (idealized 2-D cart).
#
# Sibling of swingup_test.gd, but with the SERIES double pendulum (Arm + Arm2 on hinges,
# same masses/lengths as main.tscn). A kinematic "cart" (Drone) is driven in world X and Y
# from a commanded acceleration (ax, ay) -- the "acceleration is the input" abstraction the
# real ModeAccel provides -- with NO flight stack. This isolates the double-pendulum PLANT.
#
# It replays a trajectory optimized offline by native/tools/swingup2_traj.py (iLQR):
#   MODE=openloop : drive the cart with the nominal u*(t) only. Compares Godot theta1,theta2
#                   to the nominal x*(t). Early match => model is right (ladder A); late
#                   divergence => the up-trajectory is unstable (motivates TVLQR, ladder C).
#   MODE=tvlqr    : u = u*(t) - K(t)(x_hat - x*(t)), clamped to the rails. Tracks into the
#                   catch gate, then hands off to the 6-state ArmBalance2 LQR (ladder D).
#
# Run (headless):
#   SWINGUP2_CSV=/path/traj.csv SWINGUP2_MODE=tvlqr SWINGUP2_HEADLESS=1 \
#       godot --headless --path sim-v-1 res://swingup2_test.tscn

const G := 9.8
const L1 := 0.7
const L2 := 0.6
const A_LAT_MAX := 12.0
const A_UP_MAX := 9.8

@export var mode := "openloop"           # "openloop" | "tvlqr"
@export var hold_s := 3.0                # extra time after the trajectory to let the catch settle
@export var catch_ang_deg := 8.0         # both links within this of up ...
@export var catch_rate_dps := 40.0       # ... AND slower than this -> hand off to ArmBalance2
@export var headless := false

@onready var drone: RigidBody3D = $Drone
@onready var arm: RigidBody3D = $Arm
@onready var arm2: RigidBody3D = $Arm2
@onready var status_label: Label = $CanvasLayer/Status
@onready var graph = $CanvasLayer/AccelGraph

# nominal trajectory (per knot): time, state x8, input u2, gain K16 (2x8 row-major)
var nom_x: Array = []
var nom_u: Array = []
var nom_K: Array = []
var dt_nom := 0.02
var n_knots := 0
var T_traj := 0.0

# kinematic cart state (world): lateral X, vertical Y (offset from h0)
var cart_s := 0.0
var cart_v := 0.0
var cart_h := 0.0                        # vertical offset from h0
var cart_vy := 0.0
var h0 := 0.0

var t := 0.0
var prev_th1 := 0.0
var prev_th2 := 0.0
var have_prev := false

# metrics
var max_dev1 := 0.0                      # max |theta1_meas - theta1_nom| [deg]
var max_dev2 := 0.0
var dev_log: Array = []                  # (t, dev1, dev2) checkpoints for the model-match report
var _cps := [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 4.0]
var _cp_idx := 0
var _dbg_n := 0
var _dbg := false                        # SWINGUP2_DBG=1 -> print the first 12 ticks (IC / snap check)
var balancing := false
var t_catch := 0.0
var bal_max_up := 0.0
var last_up_deg := 180.0
var reached_gate := false
var balance_ctrl                         # ArmBalance2 (GDExtension)
var finished := false


func _read_env() -> void:
	if OS.get_environment("SWINGUP2_HEADLESS") == "1": headless = true
	if OS.get_environment("SWINGUP2_DBG") == "1": _dbg = true
	var v := OS.get_environment("SWINGUP2_MODE"); if v != "": mode = v
	v = OS.get_environment("SWINGUP2_HOLD_S");    if v != "": hold_s = float(v)
	v = OS.get_environment("SWINGUP2_CATCH_ANG"); if v != "": catch_ang_deg = float(v)
	v = OS.get_environment("SWINGUP2_CATCH_RATE");if v != "": catch_rate_dps = float(v)


func _theta_down(body: RigidBody3D) -> float:
	# absolute pivot->bob direction in world; 0 = straight DOWN, + toward +X (matches nominal + main.gd)
	var rod := body.global_transform.basis * Vector3(0.0, -1.0, 0.0)
	return atan2(rod.x, -rod.y)


func _load_csv(path: String) -> bool:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		push_error("[su2] cannot open CSV: %s" % path)
		return false
	f.get_line()   # header
	while not f.eof_reached():
		var line := f.get_line().strip_edges()
		if line == "":
			continue
		var p := line.split(",")
		if p.size() < 27:
			continue
		var xs := PackedFloat64Array()
		for i in range(1, 9):    # th1,w1,th2,w2,xc,vx,yc,vy
			xs.append(float(p[i]))
		nom_x.append(xs)
		nom_u.append(Vector2(float(p[9]), float(p[10])))
		var ks := PackedFloat64Array()
		for i in range(11, 27):  # K 2x8
			ks.append(float(p[i]))
		nom_K.append(ks)
	f.close()
	n_knots = nom_x.size()
	return n_knots >= 2


func _ready() -> void:
	_read_env()
	var csv := OS.get_environment("SWINGUP2_CSV")
	if csv == "" or not _load_csv(csv):
		# NOTE: pass an ABSOLUTE path -- FileAccess does not resolve relative to the shell CWD.
		push_error("[su2] need SWINGUP2_CSV = ABSOLUTE path to a swingup2_traj.py export (got '%s')" % csv)
		finished = true
		if headless: get_tree().quit(3)
		return
	# dt_nom from the export is 0.02 (50 Hz); recover exactly from the trajectory length is not
	# possible without the t column, so read it back from the first two t values instead:
	_recover_dt(csv)
	T_traj = (n_knots - 1) * dt_nom

	# kinematic cart at the ORIGIN -- matches the scene's joint anchors (Pivot at 0, Pivot2 at
	# (0,-0.7,0)) so the arms sit in their natural hanging rest pose with ZERO joint stress. Do
	# NOT re-anchor the hinges or offset the drone: a mismatch snaps the pendulum on tick 1.
	drone.freeze = true
	drone.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	drone.global_position = Vector3.ZERO
	h0 = 0.0
	# nominal starts at exact hanging [0,0,0,0]; the scene already rests there. Just zero velocities.
	arm.linear_velocity = Vector3.ZERO
	arm.angular_velocity = Vector3.ZERO
	arm2.linear_velocity = Vector3.ZERO
	arm2.angular_velocity = Vector3.ZERO

	prev_th1 = _theta_down(arm)
	prev_th2 = _theta_down(arm2)
	have_prev = true

	balance_ctrl = ArmBalance2.new()
	balance_ctrl.configure({"L1": L1, "L2": L2, "g": G, "a_max": A_LAT_MAX, "s_ref": 0.0})
	graph.setup(4.0)
	print("[su2] mode=%s  N=%d dt=%.4f T=%.2fs  csv=%s" % [mode, n_knots, dt_nom, T_traj, csv])


func _recover_dt(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null: return
	f.get_line()
	var t0 := float((f.get_line().split(","))[0])
	var t1 := float((f.get_line().split(","))[0])
	f.close()
	dt_nom = maxf(t1 - t0, 1e-4)


func _knot(tt: float) -> int:
	return clampi(int(round(tt / dt_nom)), 0, n_knots - 1)


func _physics_process(delta: float) -> void:
	if finished:
		return
	t += delta   # advance at top so the run always terminates + flushes

	# --- measured plant state (absolute angles from down, wrapped finite-diff rates) ---
	var th1 := _theta_down(arm)
	var th2 := _theta_down(arm2)
	var w1 := 0.0
	var w2 := 0.0
	if have_prev:
		w1 = wrapf(th1 - prev_th1, -PI, PI) / delta
		w2 = wrapf(th2 - prev_th2, -PI, PI) / delta
	prev_th1 = th1
	prev_th2 = th2
	have_prev = true

	var ax := 0.0
	var ay := 0.0
	var up1 := absf(wrapf(PI - th1, -PI, PI))
	var up2 := absf(wrapf(PI - th2, -PI, PI))
	last_up_deg = maxf(up1, up2) * 180.0 / PI

	# --- gate: once both links are inside the catch region, hand off to ArmBalance2 ---
	if mode == "tvlqr" and not balancing:
		if up1 < deg_to_rad(catch_ang_deg) and up2 < deg_to_rad(catch_ang_deg) \
				and absf(w1) < deg_to_rad(catch_rate_dps) and absf(w2) < deg_to_rad(catch_rate_dps):
			balancing = true
			reached_gate = true
			t_catch = t
			print("[su2] CATCH t=%.2f  up=(%.1f,%.1f)deg rate=(%.1f,%.1f)dps  cart s=%.2f v=%.2f" % [
				t, up1 * 180 / PI, up2 * 180 / PI, w1 * 180 / PI, w2 * 180 / PI, cart_s, cart_v])

	if balancing:
		# ArmBalance2 drives lateral (angles from UP, rate flips sign); vertical = altitude hold
		var a_lat: float = balance_ctrl.update(wrapf(PI - th1, -PI, PI), -w1,
			wrapf(PI - th2, -PI, PI), -w2, cart_s, cart_v, delta)
		ax = clampf(a_lat, -A_LAT_MAX, A_LAT_MAX)
		ay = clampf(-2.0 * cart_h - 2.0 * cart_vy, -A_UP_MAX, A_UP_MAX)
		bal_max_up = maxf(bal_max_up, last_up_deg)
	else:
		# --- replay the optimized trajectory ---
		var k := _knot(t)
		var un: Vector2 = nom_u[k]
		if mode == "tvlqr":
			var xn: PackedFloat64Array = nom_x[k]
			var Kf: PackedFloat64Array = nom_K[k]
			# dx = measured - nominal, angle rows wrapped
			var dx := [
				wrapf(th1 - xn[0], -PI, PI), w1 - xn[1],
				wrapf(th2 - xn[2], -PI, PI), w2 - xn[3],
				cart_s - xn[4], cart_v - xn[5], cart_h - xn[6], cart_vy - xn[7]]
			var du_x := 0.0
			var du_y := 0.0
			for i in range(8):
				du_x += Kf[i] * dx[i]        # K row 0
				du_y += Kf[8 + i] * dx[i]    # K row 1
			# iLQR exports K with the convention u = u_nom + K*(x - x_nom) (its forward-pass sign),
			# so ADD the feedback here (the doc's "u* - K dx" folds the sign into K).
			ax = clampf(un.x + du_x, -A_LAT_MAX, A_LAT_MAX)
			ay = clampf(un.y + du_y, -A_UP_MAX, A_UP_MAX)
		else:
			ax = clampf(un.x, -A_LAT_MAX, A_LAT_MAX)
			ay = clampf(un.y, -A_UP_MAX, A_UP_MAX)

		# divergence from nominal (for the model-match / open-loop report)
		var kn: PackedFloat64Array = nom_x[_knot(t)]
		var d1: float = absf(wrapf(th1 - kn[0], -PI, PI)) * 180.0 / PI
		var d2: float = absf(wrapf(th2 - kn[2], -PI, PI)) * 180.0 / PI
		max_dev1 = maxf(max_dev1, d1)
		max_dev2 = maxf(max_dev2, d2)

	if _dbg and _dbg_n < 12:
		_dbg_n += 1
		var kd: PackedFloat64Array = nom_x[_knot(t)]
		print("  [dbg] t=%.4f dt=%.4f | meas th=(%+.1f,%+.1f) w=(%+.0f,%+.0f) | nom th=(%+.1f,%+.1f) w=(%+.0f,%+.0f) | a=(%+.2f,%+.2f) cart_s=%+.3f" % [
			t, delta, rad_to_deg(th1), rad_to_deg(th2), rad_to_deg(w1), rad_to_deg(w2),
			rad_to_deg(kd[0]), rad_to_deg(kd[2]), rad_to_deg(kd[1]), rad_to_deg(kd[3]), ax, ay, cart_s])

	# --- integrate the kinematic cart (perfect accel; no flight-stack lag) ---
	cart_v += ax * delta
	cart_s += cart_v * delta
	cart_vy += ay * delta
	cart_h += cart_vy * delta
	drone.global_position = Vector3(cart_s, h0 + cart_h, 0.0)

	graph.push_sample(t, ax, ay)
	_update_status(th1, th2, w1, w2, ax, ay)

	# checkpoints for the model-match table (open-loop only)
	if mode == "openloop" and _cp_idx < _cps.size() and t >= _cps[_cp_idx]:
		var kk: PackedFloat64Array = nom_x[_knot(t)]
		dev_log.append([t,
			absf(wrapf(th1 - kk[0], -PI, PI)) * 180.0 / PI,
			absf(wrapf(th2 - kk[2], -PI, PI)) * 180.0 / PI])
		_cp_idx += 1

	if t >= T_traj + hold_s:
		_finish()


func _update_status(th1: float, th2: float, w1: float, w2: float, ax: float, ay: float) -> void:
	if status_label == null:
		return
	status_label.text = "MODE %s   t=%5.2f / %.2f\nlink1 up=%+6.1f deg  rate=%+7.1f dps\nlink2 up=%+6.1f deg  rate=%+7.1f dps\ncart s=%+5.2f v=%+5.2f   h=%+5.2f vy=%+5.2f\na=(%+6.2f, %+6.2f)   %s" % [
		mode, t, T_traj,
		wrapf(PI - th1, -PI, PI) * 180 / PI, -w1 * 180 / PI,
		wrapf(PI - th2, -PI, PI) * 180 / PI, -w2 * 180 / PI,
		cart_s, cart_v, cart_h, cart_vy, ax, ay,
		("BALANCE" if balancing else "replay")]


func _finish() -> void:
	finished = true
	if mode == "openloop":
		print("[su2] OPENLOOP divergence table (t, |dtheta1|, |dtheta2| deg):")
		for r in dev_log:
			print("      t=%.2f  d1=%6.2f  d2=%6.2f" % [r[0], r[1], r[2]])
		print("[su2] RESULT openloop  max_dev=(%.2f, %.2f) deg  final_up=%.1f deg" % [
			max_dev1, max_dev2, last_up_deg])
		if headless:
			get_tree().quit(0)
	else:
		var passed := reached_gate and last_up_deg < 12.0 and bal_max_up < 35.0
		print("[su2] RESULT tvlqr  reached_gate=%d t_catch=%.2f  bal_max_up=%.1f  final_up=%.1f => %s" % [
			int(reached_gate), t_catch, bal_max_up, last_up_deg, ("PASS" if passed else "FAIL")])
		if headless:
			get_tree().quit(0 if passed else 2)
