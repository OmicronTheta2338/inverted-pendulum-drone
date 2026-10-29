extends Node3D
#
# TRIPLE inverted pendulum SANDBOX (stretch goal) -- isolated copy of the main.gd native path
# for a SERIES triple pendulum: link1=0.7 m, link2=link3=0.6 m, bobs 0.1 kg. Runs the forked
# native flight stack ApFlightStack3 (ModeAccel3 with the 0..3-link coupled reaction FF) in
# process; the validated main.gd / main.tscn / mode_accel.* are untouched.
#
# The plant physics (thrust curve, motor lag, IMU dv/dt, ground clamp) are byte-identical to
# main.gd. What is new: 3 arms, the 3-encoder sensor chain, the 8-state ArmBalance3 loop, and
# the 3x3 reaction-FF truth check.
#
# Env flags (mirror main.gd):
#   NUM_ARMS=1..3 (default 3)        REACTION_FF=0    -> reaction FF off
#   BALANCE_ONBOARD=1               USE_SENSOR_NOISE=1 (3-encoder chain)
#   REACT_VALIDATE=1                AUTOTEST=1 (scripted accel profile, swing test)
#   BALANCE_INIT_DEG / _INIT2_DEG / _INIT3_DEG (default 0.5 deg -- basin is sub-degree!)
#   BALANCE_ALT (default 10)        SENSOR_SEED, SENSOR_LAT_*  (as main.gd)

const SENSOR_MODEL_SCRIPT := preload("res://simulation/sensor_model.gd")
const PENDULUM_CHAIN_SCRIPT := preload("res://simulation/pendulum_chain.gd")
const MOTOR_DYNAMICS_SCRIPT := preload("res://simulation/motor_dynamics.gd")
const VEHICLE_PLANT_SCRIPT := preload("res://simulation/vehicle_plant.gd")
const PILOT_INPUT_SCRIPT := preload("res://simulation/pilot_input.gd")
const AUTO_PROFILE_DRIVER_SCRIPT := preload("res://simulation/auto_profile_driver.gd")
const ACCELERATION_GRADER_SCRIPT := preload("res://simulation/acceleration_grader.gd")
const SCENARIO_VISUALS_SCRIPT := preload("res://simulation/scenario_visuals.gd")

@onready var drone: RigidBody3D = $Drone

@export var response_video_overlay := false
@export var graph_window_seconds := 6.0

const MASS := 1.0
const INERTIA := Vector3(0.01616, 0.04272, 0.02674)
const K_M := 0.01
const GRAVITY := 9.8
const T_MAX := 19.3
const THRUST_EXPO := 0.35
const MOTOR_TAU := 0.03

var motor_pos: Array[Vector3] = [
	Vector3(-0.303, 0.023,  0.240), Vector3( 0.303, 0.023,  0.240),
	Vector3( 0.303, 0.023, -0.240), Vector3(-0.303, 0.023, -0.240),
	Vector3(-0.103, 0.000,  0.132), Vector3( 0.103, 0.000,  0.132),
	Vector3( 0.103, 0.000, -0.132), Vector3(-0.103, 0.000, -0.132),
]
var motor_dir: Array[Vector3] = [
	Vector3(0, 1, 0), Vector3(0, 1, 0), Vector3(0, 1, 0), Vector3(0, 1, 0),
	Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(1, 0, 0), Vector3(-1, 0, 0),
]
var motor_spin: Array[float] = [ 1, -1, 1, -1, 1, -1, 1, -1 ]
var motor_thrust: Array[float] = [0, 0, 0, 0, 0, 0, 0, 0]
var motor_cmd: Array[float] = [0, 0, 0, 0, 0, 0, 0, 0]

var lat_level := 0     # -4..+4  (keyboard lateral accel)
var vert_level := 0    # -3..+3  (keyboard vertical accel)
var park := false

var P: Basis
var sim_time := 0.0
var on_ground := false
const NATIVE_YAW_RATE := 0.7854

# ---- native flight stack (triple) ----
var flightstack                            # ApFlightStack3
var sensor_model = SENSOR_MODEL_SCRIPT.new()
var pendulum_chain = PENDULUM_CHAIN_SCRIPT.new()
var _imu_prev_vel := Vector3.ZERO
var _imu_accel_world := Vector3.ZERO

# ---- arms (1..3) ----
var n_arms := 3
var reaction_ff_on := true
var post_ff := true   # keep the reaction FF on after the balance run ends (env POSTREL_FF=0 disables, for diagnosis)
var arm_bodies: Array[RigidBody3D] = []
var arm_theta := [0.0, 0.0, 0.0]           # from straight-DOWN, + toward +X (native path input)
var arm_theta_dot := [0.0, 0.0, 0.0]

# ---- balance (env BALANCE_ONBOARD) ----
var balance_onboard := false
var balance_ctrl                           # ArmBalance3
var bal_graph_u                            # top strip-chart: a_cmd vs REAL realised lateral accel
var bal_graph_vertical                     # optional response-video chart: vertical command vs realised
var bal_angle_readout: Label               # compact numeric link-angle display beside the vehicle
var _overlay_balance_on := false
var _overlay_vehicle_command := "HOLD POSITION"
var _overlay_reaction_ff_on := false
var balance_init := [0.5, 0.5, 0.5]        # per-link start lean from upright [deg] (tiny: basin sub-degree)
var balance_alt := 10.0
const BAL_RUN_TIME := 15.0
# 8-state LQR gains from native/tools/compute_lqr3.py (Q=diag(400,10,400,10,400,10,2,4), R=1).
const BAL_GAINS3 := {"k1": -601.361783, "k1d": -32.818475, "k2": 1639.755052, "k2d": 149.665587,
	"k3": -1127.057775, "k3d": -173.762737, "ks": -1.414214, "kv": -5.173150}
var _bal_t := 0.0
var _bal_max := [0.0, 0.0, 0.0]
var _bal_sum_cmd2 := 0.0
var _bal_n := 0
var _bal_sat := false
var _bal_done := false
var _bal_post_t := 0.0

# ---- reaction-FF truth validation (env REACT_VALIDATE) ----
var _rv_on := false
var _rv_prev := [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var _rv_seeded := false
var _rv_err_lat := 0.0
var _rv_err_up := 0.0
var _rv_ref_lat := 0.0
var _rv_ref_up := 0.0
var _rv_n := 0

# ---- sensor model (harness-side, ON by default) -- 3-encoder chain ----
@export var sensor_noise_enabled := true
@export var sensor_seed := 1
@export var arm_imu_distance := 0.10 # metres from proximal pivot; set to actual mounting location
@export var sig_arm_accel := 0.35 # local accelerometer sigma [m/s^2]
@export var sig_arm_gyro_dps := 0.4 # local gyro sigma [deg/s]
@export var encoder_rate_tau := 0.02 # differentiated encoder low-pass [s]
@export var arm_observer_tau := 0.5 # complementary gravity correction [s]
@export var sig_gyro_dps := 0.05
@export var sig_accel := 0.35
@export var sig_att_deg := 0.1
@export var sig_enc_deg := 0.10            # joint encoder angle [deg] (magnetic, both joints)
@export var enc_lsb_deg := 0.05            # encoder quantization step [deg]
@export var lat_imu_ms := 2.5
@export var lat_ekf_att_ms := 8.0
@export var lat_ekf_vel_ms := 20.0
@export var lat_ekf_pos_ms := 20.0
@export var lat_arm_ms := 2.5              # clean encoder / differential-gyro read
@export var lat_arm_ang_imu_ms := 8.0      # link1 IMU-fused angle (deeper)

# ---- headless auto-test (AUTOTEST=1): scripted accel profile, self-excites the swing ----
var auto_test := false
var auto_profile_driver = AUTO_PROFILE_DRIVER_SCRIPT.new()
var auto_profile := [
	[0.6, 0, 0], [1.1, 1, 0], [1.6, 0, 0], [2.1, 2, 0], [2.6, 0, 0],
	[3.1, 4, 0], [3.6, 0, 0], [4.1, -2, 0], [4.6, 0, 0], [5.1, -4, 0], [5.6, 0, 0],
	[7.1, 0, 1], [7.6, 0, 0], [8.1, 0, 2], [8.6, 0, 0], [9.1, 0, -1], [9.6, 0, 0],
	[15.6, 2, 0], [16.1, 0, 0], [22.1, -2, 0], [22.6, 0, 0],
]

var acceleration_grader = ACCELERATION_GRADER_SCRIPT.new()


func _ready() -> void:
	drone.mass = MASS
	drone.inertia = INERTIA
	drone.gravity_scale = 1.0
	drone.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	drone.center_of_mass = Vector3.ZERO
	drone.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	drone.linear_damp = 0.05
	drone.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	drone.angular_damp = 0.0

	P = Basis(Vector3(0, 1, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0))

	# Plain editor "Run" (F5, no env) -> triple balance demo, noise OFF (best-case first look;
	# the basin is sub-degree so noise-on usually falls -- enable it with USE_SENSOR_NOISE=1).
	var editor_demo := OS.get_environment("NUM_ARMS") == "" and OS.get_environment("AUTOTEST") == "" \
		and OS.get_environment("BALANCE_ONBOARD") == "" and OS.get_environment("REACT_VALIDATE") == ""
	if editor_demo:
		sensor_noise_enabled = true   # F5 default matches main.gd: realistic sensing ON (else the accel
		                              # trace looks deceptively flat -- the settled triple commands ~nothing)

	var num_env := OS.get_environment("NUM_ARMS")
	n_arms = clampi(int(num_env), 1, 3) if num_env != "" else 3

	auto_test = OS.get_environment("AUTOTEST") == "1"
	reaction_ff_on = OS.get_environment("REACTION_FF") != "0"
	post_ff = OS.get_environment("POSTREL_FF") != "0"
	_rv_on = OS.get_environment("REACT_VALIDATE") == "1"
	balance_onboard = editor_demo or OS.get_environment("BALANCE_ONBOARD") == "1"
	response_video_overlay = response_video_overlay or OS.get_environment("RESPONSE_VIDEO") == "1"
	var graph_window_env := OS.get_environment("GRAPH_WINDOW_SECONDS")
	if graph_window_env != "":
		graph_window_seconds = maxf(float(graph_window_env), 1.0)

	flightstack = ApFlightStack3.new()
	print("[native3] in-process TRIPLE flight stack active (ApFlightStack3 / ModeAccel3)")
	_sensor_setup()
	_setup_arms()

	if balance_onboard:
		_balance_setup()

	print("TRIPLE sandbox. Keys: lateral A..L (-4..+4, G=0), vertical Z..M (-3..+3, V=hover), Q/E yaw, SPACE=park.")
	DisplayServer.window_move_to_foreground()
	get_window().grab_focus()


func _input(event: InputEvent) -> void:
	if auto_test or balance_onboard:
		return
	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_SPACE:
			park = true
		else:
			var command := PILOT_INPUT_SCRIPT.decode_acceleration_key(event.keycode)
			if command.x == PILOT_INPUT_SCRIPT.AXIS_LATERAL:
				lat_level = command.y
			elif command.x == PILOT_INPUT_SCRIPT.AXIS_VERTICAL:
				vert_level = command.y
			if command.x != PILOT_INPUT_SCRIPT.AXIS_NONE:
				park = false


func _physics_process(delta: float) -> void:
	sim_time += delta
	var a_total_world := (drone.linear_velocity - _imu_prev_vel) / maxf(delta, 1e-6)
	_imu_prev_vel = drone.linear_velocity
	_imu_accel_world = a_total_world - Vector3(0, -GRAVITY, 0)
	if auto_test:
		_auto_drive()
	_update_pendulum(delta)
	if _rv_on:
		_react_validate(a_total_world, delta)
	_service_native(delta)
	_apply_forces()
	_ground_clamp()
	_grade(delta)


func _service_native(delta: float) -> void:
	var basis := drone.global_transform.basis
	var grounded := drone.global_position.y <= 0.0

	var accel_ef := P * _imu_accel_world
	var vel_ned := P * drone.linear_velocity
	var pos_ned := P * drone.global_position
	var gyro_frd := P * (basis.inverse() * drone.angular_velocity)
	var R := P * basis * P.inverse()
	var quat := R.get_rotation_quaternion()
	var th := [arm_theta[0], arm_theta[1], arm_theta[2]]
	var td := [arm_theta_dot[0], arm_theta_dot[1], arm_theta_dot[2]]

	if sensor_noise_enabled:
		var truth := {
			"accel_ef": accel_ef, "gyro": gyro_frd, "quat": quat, "vel_ned": vel_ned, "pos_ned": pos_ned,
			"theta0": th[0], "theta1": th[1], "theta2": th[2], "tdot0": td[0], "tdot1": td[1], "tdot2": td[2],
		}
		truth.merge(pendulum_chain.first_arm_imu(delta, GRAVITY, arm_imu_distance))
		var total_motor_thrust := MOTOR_DYNAMICS_SCRIPT.total_thrust(motor_thrust)
		var s := sensor_model.sample(truth, delta, total_motor_thrust)
		accel_ef = s["accel_ef"]; gyro_frd = s["gyro"]; quat = s["quat"]
		vel_ned = s["vel_ned"]; pos_ned = s["pos_ned"]
		th = [s["theta0"], s["theta1"], s["theta2"]]
		td = [s["tdot0"], s["tdot1"], s["tdot2"]]

	flightstack.set_state(accel_ef, vel_ned, pos_ned, true, true, quat, gyro_frd)

	# All outer loops use the same navigation estimate as the native stack.
	# P maps world to NED; convert back without reading the rigid-body truth.
	var estimated_position := P.inverse() * pos_ned
	var estimated_velocity := P.inverse() * vel_ned

	var lat_stick := lat_level / 4.0
	var up_stick := vert_level / 3.0
	var yaw := 0.0
	var ff_arms := (n_arms if reaction_ff_on else 0)
	if balance_onboard and not _bal_done:
		# Feed ArmBalance3 the SAME sensed arm state as the reaction FF, re-expressed in ITS
		# convention: sensed th/td are from straight-DOWN -> theta_up = wrap(PI - th), rate flips sign.
		var s_lat := estimated_position.x
		var v_lat := estimated_velocity.x
		var a_cmd: float = balance_ctrl.update(
			wrapf(PI - th[0], -PI, PI), -td[0],
			wrapf(PI - th[1], -PI, PI), -td[1],
			wrapf(PI - th[2], -PI, PI), -td[2],
			s_lat, v_lat, delta)
		if balance_ctrl.last_saturated():
			_bal_sat = true
		lat_stick = clampf(a_cmd / 12.0, -1.0, 1.0)   # ACCEL_LAT_CMD_MSS = 12
		var alt_err := balance_alt - estimated_position.y
		up_stick = clampf((2.0 * alt_err - 2.0 * estimated_velocity.y) / GRAVITY, -1.0, 1.0)
		_overlay_balance_on = true
		_overlay_vehicle_command = "BALANCE"
		_overlay_reaction_ff_on = ff_arms > 0
		park = false
		_bal_record(delta, a_cmd, up_stick * GRAVITY)
	elif balance_onboard:
		# balance done -> the triple drops. FF stays on by default (POSTREL_FF=0 to test off); hold a hover.
		if not post_ff:
			ff_arms = 0
		_overlay_balance_on = false
		_overlay_vehicle_command = "HOLD POSITION"
		_overlay_reaction_ff_on = ff_arms > 0
		lat_stick = clampf((-1.5 * estimated_position.x - 3.0 * estimated_velocity.x) / 12.0, -1.0, 1.0)
		up_stick = clampf((2.0 * (balance_alt - estimated_position.y) - 2.0 * estimated_velocity.y) / GRAVITY, -1.0, 1.0)
		park = false
		var _pp := _bal_post_t
		_bal_post_t += delta
		_push_analysis_samples(
			_bal_t + _bal_post_t,
			lat_stick * 12.0,
			up_stick * GRAVITY
		)
		if _bal_post_t < 121.0 and int(_bal_post_t) != int(_pp):
			var e := drone.global_transform.basis.get_euler()
			var wmax: float = rad_to_deg(maxf(maxf(absf(arm_theta_dot[0]), absf(arm_theta_dot[1])), absf(arm_theta_dot[2])))
			print("[postrel] t=%4.1f fwd=%+.2f lat=%+.2f alt=%.2f | pitch=%+.1f yaw=%+.1f | wmax=%5.0f alat=%+6.1f | arms=(%+.0f,%+.0f,%+.0f)" % [
				_bal_post_t, drone.global_position.z, drone.global_position.x, drone.global_position.y,
				rad_to_deg(e.x), rad_to_deg(e.y), wmax, _imu_accel_world.x,
				_link_signed_deg(0), _link_signed_deg(1), _link_signed_deg(2)])
		if DisplayServer.get_name() == "headless" and _bal_post_t >= 120.0:
			get_tree().quit(0)
	else:
		yaw = float(PILOT_INPUT_SCRIPT.yaw_direction())

	# hold attitude level during the post-drop phase: the pitch-based fore/aft hold + a whipping arm form a
	# parametric pump (see diagnosis). Level attitude breaks the coupling; balance/flight keep it off.
	flightstack.set_hold_level(balance_onboard and _bal_done)
	flightstack.set_command(lat_stick, up_stick, yaw * NATIVE_YAW_RATE, park,
		ff_arms, Vector3(th[0], th[1], th[2]), Vector3(td[0], td[1], td[2]))

	var pwm: PackedInt32Array = flightstack.update(grounded, delta)
	for i in range(8):
		motor_cmd[i] = _pwm_to_thrust(pwm[i])
	_apply_motor_lag(delta)


# ---- sensor model: 3-encoder chain (see file header + plan appendix) ----
func _sensor_setup() -> void:
	if OS.get_environment("USE_SENSOR_NOISE") != "":
		sensor_noise_enabled = OS.get_environment("USE_SENSOR_NOISE") != "0"
	var v := OS.get_environment("SENSOR_SEED");        if v != "": sensor_seed = int(v)
	v = OS.get_environment("SENSOR_LAT_EKF_ATT_MS");   if v != "": lat_ekf_att_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_EKF_VEL_MS");   if v != "": lat_ekf_vel_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_EKF_POS_MS");   if v != "": lat_ekf_pos_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_ARM_IMU_MS");   if v != "": lat_arm_ang_imu_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_ARM_RATE_MS");  if v != "": lat_arm_ms = float(v)
	sensor_model.configure({
		"seed": sensor_seed,
		"sig_arm_accel": sig_arm_accel,
		"sig_arm_gyro_dps": sig_arm_gyro_dps,
		"arm_observer_tau": arm_observer_tau,
		"encoder_rate_tau": encoder_rate_tau,
		"sig_gyro_dps": sig_gyro_dps,
		"sig_accel": sig_accel,
		"vib_accel": 0.0,
		"vib_f_hover": 0.0,
		"sig_att_deg": sig_att_deg,
		"sig_relative_angle_deg": sig_enc_deg,
		"encoder_lsb_deg": enc_lsb_deg,
		"lat_imu_ms": lat_imu_ms,
		"lat_attitude_ms": lat_ekf_att_ms,
		"lat_velocity_ms": lat_ekf_vel_ms,
		"lat_position_ms": lat_ekf_pos_ms,
		"lat_arm_ms": lat_arm_ms,
		"lat_arm_imu_ms": lat_arm_ang_imu_ms,
		"mass": MASS,
		"gravity": GRAVITY,
		"arm_count": 3,
	})
	if sensor_noise_enabled:
		var delays := sensor_model.latency_summary()
		print("[sensor3] model ON seed=%d lat(imu/att/vel/pos/arm/arm_imu)=%d/%d/%d/%d/%d/%d ticks" % [
			sensor_seed, delays[0], delays[1], delays[2], delays[3], delays[4], delays[5]])


# ---- plant (byte-identical to main.gd) ----
func _apply_motor_lag(dt: float) -> void:
	MOTOR_DYNAMICS_SCRIPT.apply_first_order_lag(motor_thrust, motor_cmd, dt, MOTOR_TAU)


func _pwm_to_thrust(pwm: int) -> float:
	return MOTOR_DYNAMICS_SCRIPT.pwm_to_thrust(pwm, T_MAX, THRUST_EXPO)


func _apply_forces() -> void:
	VEHICLE_PLANT_SCRIPT.apply_motor_forces(
		drone, motor_pos, motor_dir, motor_spin, motor_thrust, K_M
	)


func _ground_clamp() -> void:
	on_ground = VEHICLE_PLANT_SCRIPT.clamp_to_ground(drone)


# ---- arms ----
func _setup_arms() -> void:
	var bodies := [$Arm, $Arm2, $Arm3]
	var pivots := [$Pivot, $Pivot2, $Pivot3]
	pendulum_chain.configure(bodies, pivots, n_arms)
	arm_bodies = pendulum_chain.bodies
	print("[arms3] %d series pendulum link(s) attached" % n_arms)


func _update_pendulum(delta: float) -> void:
	var measurement := pendulum_chain.sample(delta, arm_theta.size())
	arm_theta = measurement["angles"]
	arm_theta_dot = measurement["rates"]


# ---- balance ----
func _balance_setup() -> void:
	var lens := [0.7, 0.6, 0.6]
	var bv := OS.get_environment("BALANCE_INIT_DEG");  if bv != "": balance_init[0] = float(bv)
	bv = OS.get_environment("BALANCE_INIT2_DEG"); if bv != "": balance_init[1] = float(bv)
	bv = OS.get_environment("BALANCE_INIT3_DEG"); if bv != "": balance_init[2] = float(bv)
	bv = OS.get_environment("BALANCE_ALT");       if bv != "": balance_alt = float(bv)

	balance_ctrl = ArmBalance3.new()
	var cfg := {"L1": 0.7, "L2": 0.6, "L3": 0.6, "g": GRAVITY, "a_max": 12.0, "s_ref": 0.0}
	for k in BAL_GAINS3:
		cfg[k] = BAL_GAINS3[k]
	balance_ctrl.configure(cfg)

	# teleport to hover altitude, level, at rest; re-anchor root hinge
	drone.global_position = Vector3(0.0, balance_alt, 0.0)
	drone.linear_velocity = Vector3.ZERO
	drone.angular_velocity = Vector3.ZERO
	var dxf := drone.global_transform
	dxf.basis = Basis()
	drone.global_transform = dxf
	$Pivot.global_transform = Transform3D(Basis.IDENTITY, drone.global_position)

	# erect each link INVERTED from the previous tip, with a small lean (theta from up, + toward +X)
	var anchor := drone.global_position
	var pivots := [$Pivot, null, null]
	if n_arms >= 2: pivots[1] = $Pivot2
	if n_arms >= 3: pivots[2] = $Pivot3
	for i in range(n_arms):
		var th_up := deg_to_rad(balance_init[i])
		arm_bodies[i].global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - th_up), anchor)
		arm_bodies[i].linear_velocity = Vector3.ZERO
		arm_bodies[i].angular_velocity = Vector3.ZERO
		if i >= 1:
			pivots[i].global_transform = Transform3D(Basis.IDENTITY, anchor)
		anchor = arm_bodies[i].global_transform * Vector3(0.0, -lens[i], 0.0)   # this link's tip

	print("[bal3] BALANCE_ONBOARD active: %d link(s), alt=%.1f m, init=(%.2f,%.2f,%.2f) deg, noise=%d" % [
		n_arms, balance_alt, balance_init[0], balance_init[1], balance_init[2], int(sensor_noise_enabled)])

	SCENARIO_VISUALS_SCRIPT.frame_camera(
		$Camera3D,
		Vector3(0.0, balance_alt + 0.9, 4.2),
		Vector3(0.0, balance_alt + 0.9, 0.0)
	)
	SCENARIO_VISUALS_SCRIPT.colour_links(arm_bodies)

	# One shallow lateral chart for normal footage; RESPONSE_VIDEO=1 adds vertical tracking beside it.
	var graphs := SCENARIO_VISUALS_SCRIPT.create_recording_graphs(
		$CanvasLayer,
		graph_window_seconds,
		response_video_overlay
	)
	bal_graph_u = graphs[0]
	bal_graph_vertical = graphs[1] if graphs.size() > 1 else null
	bal_angle_readout = SCENARIO_VISUALS_SCRIPT.create_angle_readout($CanvasLayer)


# TRUTH |angle from up| of link i [deg]; 0 = upright.
func _link_up_deg(i: int) -> float:
	var rod: Vector3 = arm_bodies[i].global_transform.basis * Vector3(0, -1, 0)
	return absf(wrapf(atan2(rod.x, rod.y), -PI, PI)) * 180.0 / PI


# SIGNED angle from up [deg], + toward +X (for the angle strip-chart, so lean direction shows).
func _link_signed_deg(i: int) -> float:
	var rod: Vector3 = arm_bodies[i].global_transform.basis * Vector3(0, -1, 0)
	return wrapf(atan2(rod.x, rod.y), -PI, PI) * 180.0 / PI


func _push_analysis_samples(
	sample_time: float,
	lateral_command: float,
	vertical_command: float
) -> void:
	if not bal_graph_u:
		return
	var lateral_realised := _imu_accel_world.x
	var angles: Array[float] = []
	for index in range(n_arms):
		angles.append(_link_signed_deg(index))
	bal_graph_u.push_sample(sample_time, lateral_command, lateral_realised)
	SCENARIO_VISUALS_SCRIPT.update_angle_readout(
		bal_angle_readout,
		angles,
		_overlay_balance_on,
		_overlay_vehicle_command,
		_overlay_reaction_ff_on
	)
	if bal_graph_vertical:
		bal_graph_vertical.push_sample(
			sample_time,
			vertical_command,
			_imu_accel_world.y - GRAVITY
		)


func _bal_record(delta: float, a_cmd: float, vertical_command: float) -> void:
	for i in range(n_arms):
		_bal_max[i] = maxf(_bal_max[i], _link_up_deg(i))
	_bal_sum_cmd2 += a_cmd * a_cmd
	_bal_n += 1
	_push_analysis_samples(_bal_t, a_cmd, vertical_command)
	if _bal_n % 100 == 0:
		var eb := drone.global_transform.basis.get_euler()
		print("[bal3] t=%5.2f up=(%.2f,%.2f,%.2f) lat=%+5.2f fwd=%+5.2f alt=%5.2f pitch=%+4.1f a_cmd=%+6.2f sat=%d" % [
			_bal_t, _link_up_deg(0), _link_up_deg(1) if n_arms >= 2 else 0.0,
			_link_up_deg(2) if n_arms >= 3 else 0.0,
			drone.global_position.x, drone.global_position.z, drone.global_position.y,
			rad_to_deg(eb.x), a_cmd, int(_bal_sat)])
	_bal_t += delta
	if _bal_t >= BAL_RUN_TIME and not _bal_done:
		_bal_finish()


func _bal_finish() -> void:
	_bal_done = true
	if bal_graph_u:
		bal_graph_u.add_event(_bal_t, "arm release")
		if bal_graph_vertical:
			bal_graph_vertical.add_event(_bal_t, "arm release")
	var f := [_link_up_deg(0), _link_up_deg(1) if n_arms >= 2 else 0.0, _link_up_deg(2) if n_arms >= 3 else 0.0]
	var rms := sqrt(_bal_sum_cmd2 / float(max(1, _bal_n)))
	var passed := true
	for i in range(n_arms):
		if _bal_max[i] >= 15.0 or f[i] >= 5.0:
			passed = false
	print("RESULT_BAL3 arms=%d noise=%d ff=%d | max=(%.2f,%.2f,%.2f) final=(%.3f,%.3f,%.3f) | rms_u=%.2f sat=%d alt=%.2f => %s" % [
		n_arms, int(sensor_noise_enabled), int(reaction_ff_on),
		_bal_max[0], _bal_max[1], _bal_max[2], f[0], f[1], f[2], rms, int(_bal_sat),
		drone.global_position.y, ("PASS" if passed else "FAIL")])
	# DIAGNOSTIC (POSTREL_FREEZE=1): strip ALL arm influence after the drop -- detach every hinge and
	# freeze the arm bodies so they apply zero reaction -> the drone flies bare. If the fore/aft
	# oscillation still appears, the arms are exonerated and it's the forward-hold / hover loop itself.
	if OS.get_environment("POSTREL_FREEZE") == "1":
		for p in [$Pivot, $Pivot2, $Pivot3]:
			if is_instance_valid(p):
				p.node_a = NodePath(""); p.node_b = NodePath("")
		for a in arm_bodies:
			a.freeze = true
		print("[postrel] POSTREL_FREEZE: arms detached + frozen -> bare drone")
	# DIAGNOSTIC (POSTREL_FREEZE_DRONE=1): pin the DRONE in place so the triple swings on a FIXED pivot,
	# with no drone motion to absorb energy. If wmax still decays, the sink is numerical (Jolt solver);
	# if it swings on undecayed, the normal settling is energy coupling into the drone.
	if OS.get_environment("POSTREL_FREEZE_DRONE") == "1":
		drone.freeze = true
		drone.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
		print("[postrel] POSTREL_FREEZE_DRONE: drone pinned -> triple on a FIXED pivot")
	# On FAIL, done. On PASS (headless), fall through to the post-release phase so we can verify the drone
	# holds station once the triple drops (it quits itself after ~120 s in _service_native).
	if DisplayServer.get_name() == "headless" and not passed:
		get_tree().quit(2)


# ---- reaction-FF truth validation: TRUE R = sum_i m_i (g_vec - a_bob_i) vs the 3x3 FORMULA ----
func _react_validate(a_d: Vector3, delta: float) -> void:
	if n_arms < 1 or drone.global_position.y <= 0.0:
		return
	var dt := maxf(delta, 1e-6)
	var g := GRAVITY
	var m := [0.1, 0.1, 0.1]
	var ab := [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
	for i in range(n_arms):
		ab[i] = (arm_bodies[i].linear_velocity - _rv_prev[i]) / dt
		_rv_prev[i] = arm_bodies[i].linear_velocity
	if not _rv_seeded:
		_rv_seeded = true
		return
	# TRUE reaction (accel units); g_vec = (0,-g)
	var rt_lat := 0.0
	var rt_up := 0.0
	for i in range(n_arms):
		rt_lat += m[i] * (-ab[i].x)
		rt_up += m[i] * (-g - ab[i].y)
	# FORMULA reaction: identical math to mode_accel3.cpp's 3x3 solve, using the ACTUAL cart accel a_d
	var rf := _formula_reaction(a_d, m)
	_rv_err_lat += (rf.x - rt_lat) * (rf.x - rt_lat)
	_rv_err_up += (rf.y - rt_up) * (rf.y - rt_up)
	_rv_ref_lat += rt_lat * rt_lat
	_rv_ref_up += rt_up * rt_up
	_rv_n += 1
	if _rv_n % 400 == 0:
		var el := sqrt(_rv_err_lat / _rv_n)
		var eu := sqrt(_rv_err_up / _rv_n)
		var rl := sqrt(_rv_ref_lat / _rv_n)
		var ru := sqrt(_rv_ref_up / _rv_n)
		print("[react3] n=%d | rms_err(lat,up)=(%.4f,%.4f) rms_true=(%.4f,%.4f) rel=(%.1f%%,%.1f%%)" % [
			_rv_n, el, eu, rl, ru, 100.0 * el / maxf(rl, 1e-6), 100.0 * eu / maxf(ru, 1e-6)])


# The 3x3 coupled reaction (GDScript mirror of mode_accel3.cpp) -> (R_lat, R_up) accel units.
func _formula_reaction(a_d: Vector3, m: Array) -> Vector2:
	var gx := -a_d.x
	var gy := -(GRAVITY + a_d.y)
	var Ln := [0.7, 0.6, 0.6]
	var mu := [m[0] + m[1] + m[2], m[1] + m[2], m[2]]
	var t := [float(arm_theta[0]), float(arm_theta[1]), float(arm_theta[2])]
	var o := [float(arm_theta_dot[0]), float(arm_theta_dot[1]), float(arm_theta_dot[2])]
	# build symmetric A dd = b
	var A := [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
	var b := [0.0, 0.0, 0.0]
	for i in range(3):
		b[i] = Ln[i] * mu[i] * (gx * cos(t[i]) + gy * sin(t[i]))
		for j in range(3):
			var mx: int = max(i, j)
			A[i][j] = Ln[i] * Ln[j] * mu[mx] * cos(t[i] - t[j])
			if j != i:
				b[i] -= Ln[i] * Ln[j] * mu[mx] * sin(t[i] - t[j]) * o[j] * o[j]
	var dd := _solve3(A, b)
	var R_lat: float = mu[0] * gx
	var R_up: float = mu[0] * gy
	for i in range(3):
		R_lat -= mu[i] * Ln[i] * (dd[i] * cos(t[i]) - o[i] * o[i] * sin(t[i]))
		R_up -= mu[i] * Ln[i] * (dd[i] * sin(t[i]) + o[i] * o[i] * cos(t[i]))
	return Vector2(R_lat, R_up)


func _solve3(A: Array, b: Array) -> Array:
	var c00: float = A[1][1] * A[2][2] - A[1][2] * A[2][1]
	var c01: float = A[1][2] * A[2][0] - A[1][0] * A[2][2]
	var c02: float = A[1][0] * A[2][1] - A[1][1] * A[2][0]
	var det: float = A[0][0] * c00 + A[0][1] * c01 + A[0][2] * c02
	if absf(det) < 1e-12:
		return [0.0, 0.0, 0.0]
	var inv := 1.0 / det
	var c10: float = A[0][2] * A[2][1] - A[0][1] * A[2][2]
	var c11: float = A[0][0] * A[2][2] - A[0][2] * A[2][0]
	var c12: float = A[0][1] * A[2][0] - A[0][0] * A[2][1]
	var c20: float = A[0][1] * A[1][2] - A[0][2] * A[1][1]
	var c21: float = A[0][2] * A[1][0] - A[0][0] * A[1][2]
	var c22: float = A[0][0] * A[1][1] - A[0][1] * A[1][0]
	return [
		inv * (c00 * b[0] + c10 * b[1] + c20 * b[2]),
		inv * (c01 * b[0] + c11 * b[1] + c21 * b[2]),
		inv * (c02 * b[0] + c12 * b[1] + c22 * b[2])]


# ---- headless auto-test + grader (reaction-FF-with-swing validation, like Packet C) ----
func _auto_drive() -> void:
	var command := auto_profile_driver.update(
		drone.global_position.y,
		drone.linear_velocity.y,
		sim_time,
		auto_profile,
		"[auto3]",
		false
	)
	park = command["park"]
	lat_level = command["lateral_level"]
	vert_level = command["vertical_level"]


func _grade(delta: float) -> void:
	acceleration_grader.update(
		drone, delta, on_ground, sim_time, lat_level, vert_level,
		12.0, GRAVITY, false, auto_test or not balance_onboard
	)
