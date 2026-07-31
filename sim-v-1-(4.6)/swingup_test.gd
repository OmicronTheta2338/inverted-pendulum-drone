extends Node3D
#
# Packet S0 -- single-arm ENERGY SWINGUP test harness (idealized 2-D cart).
#
# Sibling of balance_test.gd: SAME Godot/Jolt plant (point-mass-bob arm on a
# HingeJoint) and a kinematic "cart" whose motion is prescribed from the realised
# acceleration -- the "acceleration is the control input" abstraction ModeAccel
# provides. Here the cart moves in BOTH world-X (lateral) and world-Y (vertical),
# because the drone can command vertical accel too (up_stick in [-1,1] -> a_y in
# [-g, +g]). That extra axis breaks the classic cart-pole underactuation.
#
# Energy law (specific energy e = E/m; theta from straight DOWN, so theta=0 hangs,
# theta=pi is upright). The pump RATE with a 2-D pivot accel (a_x, a_y) is
#
#   de/dt = -L * thd * ( a_x*cos(theta) + a_y*sin(theta) )
#         = -L * thd * ( pivot_accel . tangent )          tangent = (cos th, sin th)
#
# so to pump energy IN we aim the pivot acceleration OPPOSITE the bob's tangential
# velocity, using both axes:
#
#   a_vec = k*(e - e_top)*sign(thd) * (cos theta, sin theta)   ->  a_x~cos th, a_y~sin th
#
# a_x has authority at bottom/top (cos th = +/-1), a_y at horizontal (sin th = 1):
# exactly complementary, so there is no dead zone. Command eases as e -> e_top, so
# the bob arrives at the top slow (the ideal catch). S0 = swing up ONLY (no catch).
#
# Run:       godot --path sim-v-1 res://swingup_test.tscn
# Headless:  SWINGUP_HEADLESS=1 godot --headless --path sim-v-1 res://swingup_test.tscn

const PEND_L := 0.7                      # link1 length -- matches main.tscn Arm + ArmBalance(L=0.7)
const GRAVITY := 9.8
const E_TOP := GRAVITY * PEND_L          # specific energy at the upright, at rest
const RUN_TIME := 12.0

# --- energy-swingup law ---
# Defaults are GENTLE (k=1) on purpose: a gentle pump arrives slow (~0.2 deg/s) though ~9-11 deg
# short of dead-center; an aggressive pump hits dead-center but at 40+ deg/s. Slow wins for the
# catch (rate, not angle, is what busts the LQR basin). Provisional -- finalize vs catch in S1.
@export var pump_gain := 1.0             # k in a = k*(e-e_top)*sign(thd)*(cos th, sin th)
@export var a_max := 12.0                # lateral authority [m/s^2]; full lat_stick = 12 (1.22 g)
@export var rate_eps := 0.05             # |thd| below this -> hold sign (it's ill-defined)

# cart centering (lateral) + altitude hold (vertical): steal authority from the pump to keep the
# pivot home. a_x -= ks*s + kv*v ; a_y -= kh*(h-h0) + khv*vy. With the 2-D pump these bound drift
# to ~1.7 m lateral / ~0.6 m altitude. Provisional -- finalize vs catch in S1.
@export var center_ks := 1.0             # lateral position gain [1/s^2]
@export var center_kv := 2.0             # lateral velocity gain [1/s]
@export var center_kh := 2.0             # altitude position gain [1/s^2]
@export var center_khv := 4.0            # altitude velocity gain [1/s]

# --- scenario ---
@export var init_hang_deg := 5.0         # start this far from HANGING (breaks symmetric rest)

# --- idealized-cart imperfections (default OFF/clean for S0) ---
@export var act_tau_ms := 8.0            # lateral inner-loop ref-model lag tau_cl
@export var act_tau_up_ms := 8.0         # vertical inner-loop ref-model lag
@export var latency_ms := 0.0            # measurement delay (0 for a clean S0)
@export var noise_enabled := false
@export var sigma_theta_deg := 0.2
@export var sigma_thetadot_dps := 1.0

# --- "reached the top" detection (grader; the seed of the S1 catch gate) ---
@export var top_angle_deg := 12.0        # within this of upright ...
@export var top_rate_dps := 60.0         # ... AND slower than this = a good (catchable) arrival

# --- S1 catch gate: hand off pump -> ArmBalance once the arrival is inside the measured basin ---
@export var catch_angle_deg := 20.0      # |angle from up| below this ...
@export var catch_rate_dps := 100.0      # ... AND |rate| below this -> switch to balance
@export var catch_settle_deg := 5.0      # end within this of upright (post-catch) to PASS

# --- S1 basin-mapping mode: spawn INVERTED at (angle, rate), run ArmBalance, grade the catch ---
@export var basin_mode := false
@export var basin_ang_deg := 5.0         # initial lean from upright [deg], + toward +X
@export var basin_rate_dps := 0.0        # initial rate [deg/s], + = leaning FURTHER from upright
@export var basin_run_time := 6.0        # settle-or-diverge window [s]
@export var basin_fail_deg := 70.0       # |angle from up| beyond this = diverged (dropped)
@export var basin_settle_deg := 5.0      # must END within this of upright to count as CAUGHT

# --- display / run ---
@export var graph_window_s := 4.0
@export var headless := false

@onready var drone: RigidBody3D = $Drone
@onready var arm: RigidBody3D = $Arm
@onready var status_label: Label = $CanvasLayer/Status
@onready var graph = $CanvasLayer/AccelGraph

var rng := RandomNumberGenerator.new()

# lateral (world-X) cart state
var cart_s := 0.0
var cart_v := 0.0
var a_real := 0.0
var a_cmd := 0.0
# vertical (world-Y) cart state
var cart_h := 0.0                        # absolute height; h0 = drone_y
var cart_vy := 0.0
var a_real_y := 0.0
var a_cmd_y := 0.0

var t := 0.0
var prev_theta := 0.0
var have_prev := false
var last_sign := 1.0

var act_alpha := 0.0
var act_alpha_up := 0.0
var lat_n := 0
var delay_buf: Array = []
var buf_idx := 0
var drone_y := 0.0                       # initial hover height = altitude-hold reference h0
var drone_z := 0.0

# metrics
var best_top_deg := 180.0                # closest approach to upright [deg]
var rate_at_best := 0.0                  # |thd| at that closest approach [deg/s]
var max_cart_s := 0.0                    # worst lateral drift magnitude [m]
var max_alt_dev := 0.0                   # worst |h - h0| vertical excursion [m]
var reached_top := false
var t_reached := 0.0
var s_at_top := 0.0                      # lateral cart position at the catch instant [m]
var v_at_top := 0.0                      # lateral cart velocity at the catch instant [m/s]
var h_at_top := 0.0                      # altitude deviation at the catch instant [m]
var vy_at_top := 0.0                     # vertical cart velocity at the catch instant [m/s]
var n_swings := 0
var prev_tdot_sign := 0.0
var finished := false

# S1 catch/handoff
var use_native_swingup := false          # SWINGUP_NATIVE=1: drive pump+gate+catch from the C++ SwingUp
var native_ctrl                          # SwingUp (GDExtension)
var balancing := false                   # false = pumping, true = ArmBalance has the pole
var t_catch := 0.0
var bal_max_up := 0.0                    # worst |angle from up| AFTER the catch [deg]
var last_up_deg := 180.0                 # most recent |angle from up| [deg]

var e_data: Array = []                   # specific-energy trace (for CSV)

# basin mode
var balance_ctrl                         # ArmBalance (GDExtension)
var basin_prev_up := 0.0
var basin_have_prev := false
var basin_max_up := 0.0
var basin_kick_w := 0.0                  # initial angular velocity (about Z) to apply on first physics tick
var basin_kick_v := Vector3.ZERO         # ... plus the matching CoM linear velocity (rotation about the pivot)
var basin_need_kick := false
var basin_dbg_n := 0


func _read_env() -> void:
	if OS.get_environment("SWINGUP_HEADLESS") == "1": headless = true
	var v := OS.get_environment("SWINGUP_GAIN");   if v != "": pump_gain = float(v)
	v = OS.get_environment("SWINGUP_AMAX");         if v != "": a_max = float(v)
	v = OS.get_environment("SWINGUP_INIT_DEG");     if v != "": init_hang_deg = float(v)
	v = OS.get_environment("SWINGUP_LATENCY_MS");   if v != "": latency_ms = float(v)
	v = OS.get_environment("SWINGUP_KS");           if v != "": center_ks = float(v)
	v = OS.get_environment("SWINGUP_KV");           if v != "": center_kv = float(v)
	v = OS.get_environment("SWINGUP_KH");           if v != "": center_kh = float(v)
	v = OS.get_environment("SWINGUP_KHV");          if v != "": center_khv = float(v)
	if OS.get_environment("SWINGUP_NOISE") == "1":  noise_enabled = true
	if OS.get_environment("SWINGUP_NATIVE") == "1": use_native_swingup = true
	if OS.get_environment("SWINGUP_BASIN") == "1":  basin_mode = true
	v = OS.get_environment("SWINGUP_BASIN_ANG");    if v != "": basin_ang_deg = float(v)
	v = OS.get_environment("SWINGUP_BASIN_RATE");   if v != "": basin_rate_dps = float(v)


func _true_theta_down() -> float:
	# pivot->bob direction in world; 0 = straight DOWN, + toward +X (matches ModeAccel + main.gd)
	var rod := arm.global_transform.basis * Vector3(0.0, -1.0, 0.0)
	return atan2(rod.x, -rod.y)


func _true_theta_up() -> float:
	# 0 = straight UP, + toward +X (ArmBalance / balance_test convention)
	var rod := arm.global_transform.basis * Vector3(0.0, -1.0, 0.0)
	return atan2(rod.x, rod.y)


func _ready() -> void:
	_read_env()
	rng.seed = 1

	# drone = kinematic cart, now free in world X (lateral) AND Y (vertical)
	drone.freeze = true
	drone.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	drone_y = drone.global_position.y
	drone_z = drone.global_position.z
	cart_s = drone.global_position.x
	cart_h = drone_y

	# actuator ref-model lags (shared by both modes)
	var dt := 1.0 / float(Engine.physics_ticks_per_second)
	act_alpha = 1.0 - exp(-dt / (act_tau_ms / 1000.0))
	act_alpha_up = 1.0 - exp(-dt / (act_tau_up_ms / 1000.0))
	graph.setup(graph_window_s)

	if basin_mode:
		_basin_setup()
		return

	# arm starts HANGING, offset a few degrees toward +X: you cannot pump up from a
	# perfectly symmetric rest (thd = 0 -> zero command), so we break symmetry.
	var theta0 := deg_to_rad(init_hang_deg)
	arm.gravity_scale = 1.0
	arm.global_transform = Transform3D(Basis(Vector3(0, 0, 1), theta0), drone.global_position)
	arm.linear_velocity = Vector3.ZERO
	arm.angular_velocity = Vector3.ZERO

	# measurement-delay ring buffer
	lat_n = int(round((latency_ms / 1000.0) / dt))
	delay_buf.resize(lat_n + 1)
	for i in range(delay_buf.size()):
		delay_buf[i] = [theta0, 0.0]

	prev_theta = _true_theta_down()
	have_prev = true

	# catch controller (same portable LQR as the balance capstone), ready for the handoff
	balance_ctrl = ArmBalance.new()
	balance_ctrl.configure({"L": PEND_L, "g": GRAVITY, "a_max": a_max, "s_ref": 0.0})

	# optional: the ENTIRE pump+gate+catch as the portable C++ SwingUp (validates the S2 controller
	# against the same idealized cart before it drives the real ModeAccel stack)
	if use_native_swingup:
		native_ctrl = SwingUp.new()
		native_ctrl.configure({
			"L": PEND_L, "g": GRAVITY, "pump_gain": pump_gain, "rate_eps": rate_eps,
			"a_max_lat": a_max, "a_max_up": GRAVITY,
			"k_s": center_ks, "k_v": center_kv, "k_h": center_kh, "k_hv": center_khv,
			"catch_angle": deg_to_rad(catch_angle_deg), "catch_rate": deg_to_rad(catch_rate_dps)})
		print("  [using native C++ SwingUp controller]")

	print("swingup_test(2D): L=%.2f k=%.2f a_max=%.1f init=%.1fdeg ks=%.2f kv=%.2f kh=%.2f khv=%.2f lat=%.1fms noise=%d" % [
		PEND_L, pump_gain, a_max, init_hang_deg, center_ks, center_kv, center_kh, center_khv, latency_ms, int(noise_enabled)])


func _basin_setup() -> void:
	# S1: place the arm INVERTED at (basin_ang_deg from up, basin_rate_dps), run ArmBalance, see if it holds.
	balance_ctrl = ArmBalance.new()
	balance_ctrl.configure({"L": PEND_L, "g": GRAVITY, "a_max": a_max, "s_ref": 0.0})
	var th_up := deg_to_rad(basin_ang_deg)
	arm.gravity_scale = 1.0
	arm.global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - th_up), drone.global_position)
	arm.linear_velocity = Vector3.ZERO
	# +basin_rate_dps means "leaning FURTHER from upright" (theta_up increasing); measured: rate_up = omega_z.
	# To rotate about the PIVOT (not the CoM/bob) we set angular velocity AND the matching CoM linear
	# velocity v = omega x r, so the pivot stays instantaneously stationary (consistent with the hinge).
	# Applied on the first physics tick (setting it in _ready gets cleared by the teleport).
	basin_kick_w = deg_to_rad(basin_rate_dps)
	basin_kick_v = Vector3(basin_kick_w * PEND_L, 0, 0)
	basin_need_kick = true
	basin_prev_up = _true_theta_up()
	basin_have_prev = true
	print("basin: ang=%.1fdeg rate=%.1fdps L=%.2f a_max=%.1f" % [basin_ang_deg, basin_rate_dps, PEND_L, a_max])


func _basin_tick(dt: float) -> void:
	if basin_need_kick:
		arm.angular_velocity = Vector3(0, 0, basin_kick_w)
		arm.linear_velocity = basin_kick_v
		basin_need_kick = false
	var th_up := _true_theta_up()
	var rate_up := 0.0
	if basin_have_prev:
		rate_up = wrapf(th_up - basin_prev_up, -PI, PI) / dt
	basin_prev_up = th_up
	basin_have_prev = true

	# ArmBalance -> lateral cart accel; vertical stays at hover (cart_h fixed)
	a_cmd = balance_ctrl.update(th_up, rate_up, cart_s, cart_v, dt)
	a_real += act_alpha * (a_cmd - a_real)
	cart_v += a_real * dt
	cart_s += cart_v * dt
	drone.global_position = Vector3(cart_s, cart_h, drone_z)

	var up_deg := absf(th_up) * 180.0 / PI
	basin_max_up = maxf(basin_max_up, up_deg)
	graph.push_sample(t, a_cmd, a_real)
	if status_label:
		status_label.text = "BASIN  ang0=%+.1f deg  rate0=%+.1f deg/s\nt=%5.2f  theta_up=%+6.1f deg  (max %.1f)\ns=%+5.2f m  a_cmd=%+6.2f" % [
			basin_ang_deg, basin_rate_dps, t, th_up * 180.0 / PI, basin_max_up, cart_s, a_cmd]

	t += dt
	if up_deg > basin_fail_deg:
		_basin_finish(false)
	elif t >= basin_run_time:
		_basin_finish(absf(th_up) * 180.0 / PI < basin_settle_deg)


func _basin_finish(caught: bool) -> void:
	finished = true
	print("BASIN_RESULT ang=%.1f rate=%.1f | max_up=%.1fdeg => %s" % [
		basin_ang_deg, basin_rate_dps, basin_max_up, ("CAUGHT" if caught else "DROP")])
	if status_label:
		status_label.text += "\n\n%s" % ("CAUGHT" if caught else "DROP")
	if headless:
		get_tree().quit(0 if caught else 2)


func _pump_accel(theta: float, theta_dot: float) -> Vector2:
	# 2-D energy-shaping swingup: aim the pivot acceleration opposite the bob's tangential
	# velocity. mag = k*(e-e_top)*sign(thd); a_x = mag*cos th, a_y = mag*sin th.
	var e := 0.5 * PEND_L * PEND_L * theta_dot * theta_dot - GRAVITY * PEND_L * cos(theta)
	var e_err := e - E_TOP                          # < 0 below the top's energy
	var sgn := last_sign
	if absf(theta_dot) > rate_eps:
		sgn = signf(theta_dot)
		last_sign = sgn
	var mag := pump_gain * e_err * sgn
	# pump projected onto each axis, minus centering (lateral) / altitude-hold (vertical)
	var ax := mag * cos(theta) - center_ks * cart_s - center_kv * cart_v
	var ay := mag * sin(theta) - center_kh * (cart_h - drone_y) - center_khv * cart_vy
	ax = clampf(ax, -a_max, a_max)
	ay = clampf(ay, -GRAVITY, GRAVITY)              # net vertical accel in [-g, +g] (ModeAccel range)
	return Vector2(ax, ay)


func _physics_process(dt: float) -> void:
	if finished:
		return
	if basin_mode:
		_basin_tick(dt)
		return

	# advance time at the TOP so the run always terminates (and flushes its log) even if a tick errors
	t += dt
	if t >= RUN_TIME:
		_finish()
		return

	# 1. true plant state (theta from DOWN; rate by wrapped finite difference)
	var theta := _true_theta_down()
	var theta_dot := 0.0
	if have_prev:
		theta_dot = wrapf(theta - prev_theta, -PI, PI) / dt
	prev_theta = theta
	have_prev = true

	# 2. sensed measurement (+ optional noise) through the latency buffer
	var m_theta := theta
	var m_tdot := theta_dot
	if noise_enabled:
		m_theta += deg_to_rad(sigma_theta_deg) * rng.randfn()
		m_tdot  += deg_to_rad(sigma_thetadot_dps) * rng.randfn()
	delay_buf[buf_idx] = [m_theta, m_tdot]
	var ridx := (buf_idx - lat_n + delay_buf.size()) % delay_buf.size()
	var d: Array = delay_buf[ridx]
	buf_idx = (buf_idx + 1) % delay_buf.size()

	# 3. control: pump until the arrival enters the measured basin, then hand off to ArmBalance.
	var th_down_s := float(d[0])
	var tdot_down_s := float(d[1])
	if use_native_swingup:
		# entire pump+gate+catch runs in the portable C++ SwingUp (the S2 controller)
		var o: Dictionary = native_ctrl.update(th_down_s, tdot_down_s, cart_s, cart_v, cart_h - drone_y, cart_vy, dt)
		a_cmd = o["a_lat"]
		a_cmd_y = o["a_up"]
		if not balancing and bool(o["balancing"]):
			balancing = true
			t_catch = t
			print("  CATCH(native) t=%.2f  angle=%.1fdeg rate=%.1fdps  cart s=%.2f v=%.2f  dh=%.2f vy=%.2f" % [
				t, wrapf(PI - th_down_s, -PI, PI) * 180.0 / PI, -tdot_down_s * 180.0 / PI, cart_s, cart_v, cart_h - drone_y, cart_vy])
	else:
		# Gate on the SENSED state converted to the up-frame (theta_up = pi - theta_down, rate flips sign).
		var th_up_s := wrapf(PI - th_down_s, -PI, PI)
		var rate_up_s := -tdot_down_s
		if not balancing and absf(th_up_s) < deg_to_rad(catch_angle_deg) and absf(rate_up_s) < deg_to_rad(catch_rate_dps):
			balancing = true
			t_catch = t
			print("  CATCH t=%.2f  angle=%.1fdeg rate=%.1fdps  cart s=%.2f v=%.2f  dh=%.2f vy=%.2f" % [
				t, th_up_s * 180.0 / PI, rate_up_s * 180.0 / PI, cart_s, cart_v, cart_h - drone_y, cart_vy])
		if balancing:
			# ArmBalance drives lateral; vertical falls back to pure altitude-hold (pump role is done)
			a_cmd = balance_ctrl.update(th_up_s, rate_up_s, cart_s, cart_v, dt)
			a_cmd_y = clampf(-center_kh * (cart_h - drone_y) - center_khv * cart_vy, -GRAVITY, GRAVITY)
		else:
			var acc := _pump_accel(th_down_s, tdot_down_s)
			a_cmd = acc.x
			a_cmd_y = acc.y

	# 4. actuator ref-models: realised accel is a first-order lag of the command (per axis)
	a_real += act_alpha * (a_cmd - a_real)
	a_real_y += act_alpha_up * (a_cmd_y - a_real_y)

	# 5. integrate the cart in X and Y, drive the kinematic drone
	cart_v += a_real * dt
	cart_s += cart_v * dt
	cart_vy += a_real_y * dt
	cart_h += cart_vy * dt
	drone.global_position = Vector3(cart_s, cart_h, drone_z)
	max_cart_s = maxf(max_cart_s, absf(cart_s))
	max_alt_dev = maxf(max_alt_dev, absf(cart_h - drone_y))

	graph.push_sample(t, a_cmd, a_real)
	var e_now := 0.5 * PEND_L * PEND_L * theta_dot * theta_dot - GRAVITY * PEND_L * cos(theta)
	e_data.append(e_now)

	# 6. metrics: closest approach to the top and how slow we were there
	var top_deg := absf(wrapf(PI - theta, -PI, PI)) * 180.0 / PI    # deg from upright
	last_up_deg = top_deg
	if balancing:
		bal_max_up = maxf(bal_max_up, top_deg)
	if top_deg < best_top_deg:
		best_top_deg = top_deg
		rate_at_best = absf(theta_dot) * 180.0 / PI
	if not reached_top and top_deg < top_angle_deg and absf(theta_dot) * 180.0 / PI < top_rate_dps:
		reached_top = true
		t_reached = t
		s_at_top = cart_s
		v_at_top = cart_v
		h_at_top = cart_h - drone_y
		vy_at_top = cart_vy
	# count rate zero-crossings (= swings) to report pumping efficiency
	var s := signf(theta_dot)
	if s != 0.0 and prev_tdot_sign != 0.0 and s != prev_tdot_sign:
		n_swings += 1
	if s != 0.0:
		prev_tdot_sign = s

	if status_label:
		status_label.text = "t=%5.2f s\ntheta_down=%+6.1f deg    from_top=%5.1f deg    thd=%+7.1f deg/s\ne=%+5.2f  e_top=%+5.2f  (dE=%+6.2f)\nLAT  a=%+6.2f  s=%+5.2f m  v=%+5.2f m/s\nVERT a=%+6.2f  dh=%+5.2f m  vy=%+5.2f m/s\nbest_top=%.1f deg @ %.0f deg/s   swings=%d   %s" % [
			t, theta * 180.0 / PI, top_deg, theta_dot * 180.0 / PI,
			e_now, E_TOP, e_now - E_TOP, a_cmd, cart_s, cart_v, a_cmd_y, cart_h - drone_y, cart_vy,
			best_top_deg, rate_at_best, n_swings, ("REACHED TOP" if reached_top else "swinging...")]


func _finish() -> void:
	finished = true
	# S1 PASS = swung up, CAUGHT, and still balanced at the end (never wildly diverged post-catch)
	var passed := balancing and last_up_deg < catch_settle_deg and bal_max_up < 30.0
	print("RESULT k=%.2f centering=(%.0f,%.0f,%.0f,%.0f) | swings=%d best_top=%.1fdeg | caught=%d t_catch=%.2f bal_max=%.1fdeg final=%.1fdeg | drift lat=%.2f alt=%.2f => %s" % [
		pump_gain, center_ks, center_kv, center_kh, center_khv, n_swings, best_top_deg,
		int(balancing), t_catch, bal_max_up, last_up_deg, max_cart_s, max_alt_dev,
		("PASS" if passed else "FAIL")])
	if status_label:
		status_label.text += "\n\n%s  (caught @ %.2fs, held max %.1f deg, final %.1f deg)\nlat drift %.2f m, alt dev %.2f m" % [
			("PASS" if passed else "FAIL"), t_catch, bal_max_up, last_up_deg, max_cart_s, max_alt_dev]

	var csv_path := OS.get_environment("SWINGUP_CSV")
	if csv_path != "":
		var f := FileAccess.open(csv_path, FileAccess.WRITE)
		if f:
			f.store_line("t,a_cmd_x,a_real_x,e,e_top")
			for i in range(graph.t_data.size()):
				var ev: float = e_data[i] if i < e_data.size() else 0.0
				f.store_line("%.5f,%.5f,%.5f,%.5f,%.5f" % [graph.t_data[i], graph.cmd_data[i], graph.real_data[i], ev, E_TOP])
			f.close()
			print("wrote swingup CSV: %s (%d rows)" % [csv_path, graph.t_data.size()])

	if headless:
		get_tree().quit(0 if passed else 2)
