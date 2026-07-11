extends Node3D

@onready var drone: RigidBody3D = $Drone
@onready var arm: RigidBody3D = $Arm

@export var use_ardupilot := true
@export var use_native := false          # run the native C++ mode-30 flight stack IN-PROCESS (no UDP
										 # bridge, no lag). Overrides use_ardupilot when true. See native/.
@export var drive_from_pwm := true
@export var lateral_step_test := false   # set true & run scene: measures peak lateral accel + rise time
@export var enable_pendulum := true      # false -> detach & hide the arm so the bare drone flies unperturbed
@export var pendulum_test := false       # Phase 7: freeze drone, release arm, measure period/tau vs sqrt(g/L)
										 # (env PENDULUM_TEST=1 hanging, =2 inverted; no bridge, no motors)

const MASS := 1.0          # drone BODY mass [kg]. Authoritative: _ready() sets drone.mass = MASS,
						   # overriding the .tscn value. Arm/bob mass (0.1 kg) comes from main.tscn.
const INERTIA := Vector3(0.01616, 0.04272, 0.02674)
const K_M := 0.01
const GRAVITY := 9.8
const PEND_L := 0.6           # pivot->bob distance [m]; MUST match the Bob CollisionShape offset in main.tscn
const PEND_RATE_MAX := 10.0   # rad/s full-scale for the theta-dot bridge channel; MUST match mode_accel.cpp PEND_RATE_MAX
const T_MAX := 19.3          # F60PROV-LV 1950KV: 1966 g x 9.81 = 19.3 N max per motor
const THRUST_EXPO := 0.35    # prop thrust curve thrust=(1-e)x+e x^2; MUST match MOT_THST_EXPO
const MOTOR_TAU := 0.03      # motor+prop spin-up time constant [s] (estimate — measure on bench)
const SITL_PORT := 9002
const SITL_RECV_TIMEOUT_US := 100000
const RC_CENTER := 1500      # neutral PWM (LOITER: hold position / altitude / heading)
const RC_DEFLECT := 300      # PWM offset while a key is held -> commanded velocity/rate in LOITER

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
var motor_thrust: Array[float] = [0, 0, 0, 0, 0, 0, 0, 0]  # actual (lagged) thrust
var motor_cmd: Array[float] = [0, 0, 0, 0, 0, 0, 0, 0]     # commanded thrust (lag target)
var rc := [1500, 1500, 1500, 1500]   # keyboard sticks: ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw

# ACCEL mode (30) LATCHED discrete accel levels, set on key-down in _input().
# Lateral home row  A S D F G H J K L -> -4..+4 (G = 0, coast).
# Vertical bottom   Z X C V B N M     -> -3..+3 (V = 0, hover).
# Each level maps linearly to a direct acceleration command encoded in the RC PWM.
var lat_level := 0    # -4..+4
var vert_level := 0   # -3..+3

# PARK: SPACE engages auto-fly-to-(0,0,10m)-and-hold (signalled to ArduPilot on ch2).
# Pressing any lateral/vertical accel key disengages it and resumes manual control.
var park := false

# ---- headless auto-test (set env AUTOTEST=1): scripted takeoff + accel profile, keyboard ignored.
# Off by default, so manual flying is unaffected. profile rows: [t_end_s, lat_level(-4..4), vert_level(-3..3)]
var auto_test := false
var auto_phase := 0     # 0=takeoff(park), 1=profile, 2=hold(park)
var auto_t0 := 0.0
# SHORT, SIGN-ALTERNATING holds. The 40ms spec is a STEP-response property, so each command is held
# just long enough (~0.4s) to see the settle + a brief hold, then reversed. Alternating keeps velocity
# (hence drag) and altitude bounded near hover -- long same-sign holds built large velocity (drag error)
# and flew the drone away. This also matches the real use (rapid, small pendulum commands). Lateral
# (the primary catch axis) runs FIRST while the craft is settled at hover; vertical after. Each row
# is preceded by the previous row's reversal, so the craft returns toward zero velocity between tests.
# Each MEASURED step is a jump FROM ZERO (preceded by a 0-command settle), not a reversal. Reversals
# (e.g. +3 -> -3) are a 2x jump that over-drives and inflates overshoot; a real pendulum catch commands
# small corrections from ~rest, so step-from-zero is both cleaner to grade and more representative. The
# 0-command returns hold constant velocity (drag-cancelled), so velocity stays modest across the run.
var auto_profile := [
	[0.6, 0, 0],    # settle the park->manual handoff
	[1.1, 1, 0],    # lat +3
	[1.6, 0, 0],    #   -> 0
	[2.1, 2, 0],    # lat +6
	[2.6, 0, 0],    #   -> 0
	[3.1, 4, 0],    # lat +12 (max)
	[3.6, 0, 0],    #   -> 0
	[4.1, -1, 0],   # lat -3
	[4.6, 0, 0],    #   -> 0
	[5.1, -2, 0],   # lat -6
	[5.6, 0, 0],    #   -> 0
	[6.1, -4, 0],   # lat -12
	[6.6, 0, 0],    #   -> 0
	[7.1, 0, 1],    # up +3.27
	[7.6, 0, 0],    #   -> hover
	[8.1, 0, 2],    # up +6.53
	[8.6, 0, 0],    #   -> hover
	[9.1, 0, -1],   # up -3.27
	[9.6, 0, 0],    #   -> hover
	[15.6, 2, 0],   # sustained lat +6 for 6s -- dynamic-swing test (pendulum swings out ~31 deg)
	[16.1, 0, 0],   #   -> 0
	[22.1, -2, 0],  # sustained lat -6 for 6s -- dynamic swing other direction
	[22.6, 0, 0],   #   -> 0
]

# ============ bridge state ============
var udp: PacketPeerUDP
var client_ip := ""
var client_port := 0
# LOCKSTEP=1 -> tight 1:1 sync (AP blocks on our state each frame). Collapses the async-bridge round-trip
# dead-time (~20ms) to a few ms, at the cost of running slower than real-time (fine for a headless test;
# drive.py force-arms past the "loop slow" gate). Default = async no_lockstep (real-time manual flying).
var lockstep := false
var sim_time := 0.0
var on_ground := false
var P: Basis

# ---- IMU specific force from the drone's ACTUAL acceleration (finite-difference of its velocity),
# minus gravity. A real accelerometer measures every contact force on the body -- INCLUDING the
# pendulum's reaction force through the hinge -- not just the motor thrust. The old IMU used only
# (thrust+drag)/mass, so it was blind to the arm's reaction: the controller's integral then fought the
# reaction-force feed-forward (it saw the FF's extra thrust as excess accel and removed it). Using the
# true dv/dt fixes that and is identical to the old model for the bare drone (no reaction).
var _imu_prev_vel := Vector3.ZERO
var _imu_accel_world := Vector3.ZERO   # specific force (world frame): a_total - gravity

# ---- pendulum arms (0/1/2). Set from env NUM_ARMS (else ENABLE_PENDULUM -> 0 or 1). Each active arm's
# angle (from straight-down, + toward +lateral/+X) and angular rate are packed into spare RC channels
# (rc_6=count, rc_7/8=arm0 angle/rate, rc_9/10=arm1 angle/rate) so the ArduPilot controller can
# feed-forward the reaction force each arm applies on the frame. A real rig reads these from a hinge encoder.
var n_arms := 1
var reaction_ff_on := true       # env REACTION_FF=0 -> arms stay physically attached but the controller is
                                 # told 0 arms, i.e. the reaction-force FF is DISABLED (Packet C: FF-on vs FF-off)
var arm_bodies: Array[RigidBody3D] = []
var arm_prev_theta: Array[float] = []
var arm_has_prev: Array[bool] = []
# rc_6..rc_10, refreshed every physics tick in _update_pendulum(); default = 0 arms, neutral angle/rate.
var pend_rc := [1000, 1500, 1500, 1500, 1500]
# Same per-arm angle/rate as pend_rc but as raw floats (rad, rad/s) for the NATIVE path,
# which takes typed inputs instead of RC-PWM channels. Index 0/1 = arm0/arm1.
var arm_theta := [0.0, 0.0]
var arm_theta_dot := [0.0, 0.0]
# ---- reaction-FF validation (env REACT_VALIDATE=1): compare the derived reaction FORMULA against the
# TRUE reaction computed from the bobs' own accelerations (sum m_i (g_vec - a_bob_i)) -- an independent
# check of the coupled-dynamics derivation, not the controller. Each arm's CoM IS its bob.
var _rv_on := false
var _rv_prev_b1 := Vector3.ZERO
var _rv_prev_b2 := Vector3.ZERO
var _rv_seeded := false
var _rv_err_lat := 0.0
var _rv_err_up := 0.0
var _rv_ref_lat := 0.0
var _rv_ref_up := 0.0
var _rv_n := 0

# ---- native in-process flight stack (ApFlightStack GDExtension): the ported mode-30 inner
# loop + attitude/mixer, replacing the ArduPilot-over-UDP bridge to remove its round-trip lag.
var flightstack                       # ApFlightStack (or null when use_native is false)
const NATIVE_YAW_RATE := 0.7854       # rad/s (~45 deg/s) commanded per full Q/E deflection

# ---- representative sensor model (harness-side; behind a switch, OFF by default) ----
# On real hardware the flight stack reads a noisy IMU through ArduPilot's INS filtering +
# EKF3, and the arm angle/rate from a hinge encoder + arm gyro -- NOT the clean Godot truth
# the native path feeds by default. This models that (noise + measurement latency + EKF lag)
# so mode-30 can be validated under representative sensing. Grounded in Project-Plan Phase 4
# + Balance-Controller-Design 5. Imperfections live HERE (harness), never in the controller.
# Enable with USE_SENSOR_NOISE=1. The grader (_grade) stays on TRUTH, so it still measures
# realised-vs-commanded accel honestly while the controller runs on degraded sensors.
@export var sensor_noise_enabled := false
@export var sensor_pos_drift := true       # slow IMU-only position dead-reckoning drift (N/E)
@export var sensor_seed := 1
# per-channel Gaussian 1-sigma (SI). accel folds a broadband vibration RMS into one sigma.
@export var sig_gyro_dps := 0.05           # gyro noise [deg/s]              (Phase 4)
@export var sig_accel := 0.35              # accel_ef noise [m/s^2] ~5mg + ~30mg vibration RMS
@export var sig_att_deg := 0.1             # EKF attitude noise [deg]
@export var sig_vel := 0.03                # velocity NED noise [m/s]        (IMU dead-reckoning)
@export var sig_pos := 0.02                # position NED noise [m]
# Arm sensing is ASYMMETRIC by hardware (see plan appendix "Packet B - sensor layer"): arm 1
# (proximal, on the hollow central tube -> no room for a magnetic encoder) senses ANGLE via two
# IMUs relative to the body IMU. That fusion gives a clean differential-gyro RATE but a
# drift-bounded, maneuver-coupled, ~EKF-latency ANGLE (gyro integrates -> unbounded drift, so the
# accel gravity vector must anchor it; that anchor lies under acceleration). Arm 2 (distal) uses a
# magnetic encoder: clean, low-latency, LSB-quantized angle, but a noisier differentiated rate.
@export var sig_theta0_deg := 0.15         # arm-1 angle [deg]  (2xIMU fused attitude; NO LSB)
@export var sig_theta1_deg := 0.10         # arm-2 angle [deg]  (magnetic encoder)
@export var enc_lsb_deg := 0.05            # arm-2 encoder quantization step [deg] (~13-bit)
@export var sig_thetadot0_dps := 0.4       # arm-1 rate [deg/s] (differential gyro, clean)
@export var sig_thetadot1_dps := 1.0       # arm-2 rate [deg/s] (differentiated encoder, noisier)
@export var pos_drift_rate := 0.05         # dead-reckon position random-walk [m/sqrt(s)]
# per-group measurement latency / EKF lag [ms] -> ring-buffer depth (rounded to ticks)
@export var lat_imu_ms := 2.5              # accel_ef, gyro (raw INS, ~loop rate)
@export var lat_ekf_att_ms := 8.0          # EKF attitude output-observer lag
@export var lat_ekf_vel_ms := 20.0         # EKF velocity lag
@export var lat_ekf_pos_ms := 20.0         # EKF position lag
@export var lat_arm_ms := 2.5              # arm-2 encoder + differential-gyro read (clean channels)
@export var lat_arm_ang_imu_ms := 8.0      # arm-1 IMU-FUSED angle only (attitude-observer lag ~EKF att)

var sensor_rng := RandomNumberGenerator.new()
var _hist: Array = []                      # ring of per-tick sensed (noisy) state snapshots
var _sb_idx := 0
var _sb_len := 1
var _sensor_seeded := false
var _d_imu := 1                            # per-group delay depths (ticks), set in _sensor_setup
var _d_att := 3
var _d_vel := 8
var _d_pos := 8
var _d_arm := 1
var _d_arm_imu := 3                        # arm-1 IMU-fused ANGLE latency (deeper than clean arm chans)
var _gyro_bias := Vector3.ZERO             # slow gyro bias (random walk)
var _pos_drift := Vector3.ZERO             # accumulated dead-reckon position drift (NED)

# ---- diagnostics (remove once timing is understood) ----
var _diag_rt := 0           # round-trips (state replies) since last report
var _diag_calls := 0        # _physics_process calls since last report
var _diag_block_us := 0     # total wall-us spent blocked waiting for AP
var _diag_max_block_us := 0 # worst single block this report window
var _diag_timeouts := 0     # ticks where AP never answered within timeout
var _diag_t0 := 0           # usec of last report


func _ready() -> void:
	drone.mass = MASS
	drone.inertia = INERTIA
	drone.gravity_scale = 1.0
	drone.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	drone.center_of_mass = Vector3.ZERO
	drone.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	drone.linear_damp = 0.05   # low-speed drag, realistic for a small ~0.8 kg drone. Was 0.3 — an
							   # arbitrary overestimate (~0.5 g of drag at 17 m/s) that swamped the
							   # vertical axis. Real drag is ~quadratic + near-zero at the low speeds
							   # of actual use; this linear coeff is a placeholder — calibrate later.
	drone.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	drone.angular_damp = 0.0

	P = Basis(Vector3(0, 1, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0))

	if OS.get_environment("PENDULUM_TEST") != "":
		pendulum_test = true
	if pendulum_test:
		_pend_setup()
		return
	# Arm count: NUM_ARMS (0/1/2) wins; else ENABLE_PENDULUM (0 -> bare, 1 -> one arm) so the older
	# harness still works. The accel-mode calibration was done BARE; the reaction-force feed-forward in
	# the controller now lets it hold the accel spec with 1 or 2 arms too.
	var num_env := OS.get_environment("NUM_ARMS")
	if num_env != "":
		n_arms = clampi(int(num_env), 0, 2)
	else:
		var ep := OS.get_environment("ENABLE_PENDULUM")
		if ep == "0" or ep == "false":
			enable_pendulum = false
		elif ep == "1" or ep == "true":
			enable_pendulum = true
		n_arms = 1 if enable_pendulum else 0
	_setup_arms()

	auto_test = OS.get_environment("AUTOTEST") == "1"
	if auto_test:
		print("[auto] AUTOTEST mode: scripted takeoff + accel profile; keyboard ignored")
	reaction_ff_on = OS.get_environment("REACTION_FF") != "0"   # only "0" disables; arms stay attached either way
	if not reaction_ff_on:
		print("[accel] reaction-force FF DISABLED (REACTION_FF=0): arms swing but are NOT compensated")
	_rv_on = OS.get_environment("REACT_VALIDATE") == "1"
	lockstep = OS.get_environment("LOCKSTEP") == "1"
	if lockstep:
		print("[bridge] LOCKSTEP mode: tight 1:1 sync (lower latency, slower-than-real-time)")

	# NATIVE flight stack (env USE_NATIVE=1 or the export flag): run the ported mode-30 loop
	# in-process and DISABLE the UDP bridge, so there is no round-trip latency to hide.
	if OS.get_environment("USE_NATIVE") == "1":
		use_native = true
	if use_native:
		use_ardupilot = false
		flightstack = ApFlightStack.new()
		print("[native] in-process mode-30 flight stack active (no ArduPilot / no UDP bridge)")
		_sensor_setup()

	if use_ardupilot:
		udp = PacketPeerUDP.new()
		if udp.bind(SITL_PORT) == OK:
			print("SITL bridge listening on UDP ", SITL_PORT)
		else:
			push_error("UDP bind failed")

	if not drive_from_pwm:
		var hover_each := MASS * GRAVITY / 4.0
		motor_thrust = [hover_each, hover_each, hover_each, hover_each, 0, 0, 0, 0]
		motor_cmd = motor_thrust.duplicate()

	print("ACCEL mode (30) keys — focus this window. LATCHING: last key sets a held accel.")
	print("  Lateral  A S D F [G] H J K L : A=max-left(-5g) ... G=zero/coast ... L=max-right(+5g)")
	print("  Vertical Z X C [V] B N M     : Z=max-down(-g) ... V=hover ... M=max-up(+g)")
	print("  Yaw Q/E. Forward axis is pinned to zero by the controller (no key).")
	print("  SPACE = park: auto-fly to (0,0,10m) and hold (takes off too). Any accel key resumes manual.")
	print("  Arm in MAVProxy on the ground, then press SPACE to take off & hold, or an up key (B/N/M) to lift off.")

	# Drag the game window to the foreground and grab keyboard focus on launch.
	# Under Wayland/GNOME the run window often opens WITHOUT keyboard focus, so
	# Input.is_key_pressed() reads false forever and the RC sticks never leave
	# their no-key defaults (drone just sits there). This forces focus on boot.
	DisplayServer.window_move_to_foreground()
	get_window().grab_focus()


# DIAGNOSTIC: prints every key event the window actually receives. If pressing
# W/A/S/D/R prints nothing, the window has NO keyboard focus (Wayland/WM issue) —
# that's why the RC sticks stay frozen at defaults and the drone never moves.
# Remove once input is confirmed working.
func _input(event: InputEvent) -> void:
	if auto_test:
		return
	if event is InputEventKey and event.pressed and not event.echo:
		# SPACE engages PARK (auto-fly to 0,0,10 and hold). Any accel key disengages it.
		if event.keycode == KEY_SPACE:
			park = true
			print("[park] engaged -> flying to (0,0,10m) and holding")
		else:
			# Latch the discrete accel level for whichever axis key was pressed. The
			# level persists (it does NOT clear on key-up) until another key on that
			# axis is pressed — so the commanded accel is a held step, and the centre
			# key (G lateral / V vertical) is the explicit zero command.
			var accel_keys := [KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K, KEY_L,
				KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M]
			match event.keycode:
				KEY_A: lat_level = -4
				KEY_S: lat_level = -3
				KEY_D: lat_level = -2
				KEY_F: lat_level = -1
				KEY_G: lat_level = 0
				KEY_H: lat_level = 1
				KEY_J: lat_level = 2
				KEY_K: lat_level = 3
				KEY_L: lat_level = 4
				KEY_Z: vert_level = -3
				KEY_X: vert_level = -2
				KEY_C: vert_level = -1
				KEY_V: vert_level = 0
				KEY_B: vert_level = 1
				KEY_N: vert_level = 2
				KEY_M: vert_level = 3
			if event.keycode in accel_keys and park:
				park = false
				print("[park] released -> manual accel control")
		if get_window().has_focus() == false:
			print("[input] key received but window NOT focused — RC may stay frozen")


func _physics_process(delta: float) -> void:
	if pendulum_test:
		_pend_measure(delta)
		return
	if lateral_step_test:
		_run_lateral_test(delta)
		return
	_diag_calls += 1
	sim_time += delta
	# IMU specific force = the drone's real acceleration (dv/dt) minus gravity. drone.linear_velocity at
	# this point is the result of the PREVIOUS physics step (which included motor thrust, drag AND the
	# pendulum hinge reaction), so the finite-difference captures the true body accel a real accelerometer
	# would feel. One-step (~2.5 ms) latency, like a real sensor. (Gravity in world = (0,-g,0), up = +Y.)
	var a_total_world := (drone.linear_velocity - _imu_prev_vel) / maxf(delta, 1e-6)
	_imu_prev_vel = drone.linear_velocity
	_imu_accel_world = a_total_world - Vector3(0, -GRAVITY, 0)
	if auto_test:
		_auto_drive()
	_update_pendulum(delta)
	if _rv_on:
		_react_validate(a_total_world, delta)
	if use_native:
		_service_native(delta)
	elif use_ardupilot:
		_update_rc()
		_service_sitl(delta)
	_apply_forces()
	_ground_clamp()
	_grade(delta)


# Control-loop-INDEPENDENT measurement of peak lateral acceleration and how fast
# it builds (the canted-prop spin-up). Attitude is frozen level so we isolate
# pure lateral translation. Step at t=0: uprights hold weight, +x canted motors
# (M6, M7) to full thrust, the rest off. a_x peak = 2*T_MAX/mass; rise set by
# MOTOR_TAU. Real airframe = this x sin(cant angle). Set lateral_step_test=true.
var _ltest_t := -1.0
func _run_lateral_test(delta: float) -> void:
	if _ltest_t < 0.0:
		_ltest_t = 0.0
		motor_thrust = [0, 0, 0, 0, 0, 0, 0, 0]
		print("[lat-test] peak a_x = 2*T_MAX/mass = %.1f m/s^2 = %.2f g  (x sin(cant) for real airframe)" % [
			2.0 * T_MAX / MASS, 2.0 * T_MAX / MASS / 9.81])
		print("[lat-test]  t(ms)   a_x(m/s^2)   a_x(g)    v_x(m/s)")
	# freeze attitude level -> pure lateral translation, no parasitic tilt
	drone.angular_velocity = Vector3.ZERO
	var xf := drone.global_transform
	xf.basis = Basis()
	drone.global_transform = xf
	# step: uprights hold weight, +x canted full, -x canted off
	var hover_each := MASS * GRAVITY / 4.0
	motor_cmd = [hover_each, hover_each, hover_each, hover_each, 0.0, T_MAX, T_MAX, 0.0]
	_apply_motor_lag(delta)
	_apply_forces()
	var prev_t := _ltest_t
	_ltest_t += delta
	var fx: float = motor_thrust[5] + motor_thrust[6]   # M6, M7 push +x
	var ax := fx / MASS
	if int(_ltest_t / 0.025) != int(prev_t / 0.025) and _ltest_t <= 0.3:
		print("[lat-test] %5.0f   %9.2f   %6.2f   %8.3f" % [
			_ltest_t * 1000.0, ax, ax / 9.81, drone.linear_velocity.x])


# Lock-step bridge, NON-BLOCKING. Service whatever servo frames are already
# queued this physics tick and return immediately if none — Godot's 200 Hz
# physics clock paces the ping-pong with the (also 200 Hz, real-time-paced)
# ArduPilot, so we still get ~200 round-trips/s without spinning.
#
# Do NOT busy-wait here. The previous version blocked on OS.delay_usec() until a
# frame arrived; because AP only emits a frame every ~5 ms and Godot batches up
# to max_physics_steps_per_frame ticks per render, that parked the main thread
# ~100% of wall time. OS.delay_usec() never pumps the window event queue, so
# input died (keys stopped registering, RC frozen at defaults, motors at 1000).
func _service_sitl(delta: float) -> void:
	if lockstep:
		# LOCKSTEP: strict ping-pong. AP blocks on our state, so reply exactly once per servo received
		# (drained to the newest). Briefly poll for the servo so the 1:1 round-trip stays tight (a few ms)
		# instead of costing a whole physics tick. Bounded poll -> no infinite spin if AP stalls.
		var waited := 0
		while udp.get_available_packet_count() == 0 and waited < 4000:
			OS.delay_usec(50)
			waited += 50
		var got_ls := false
		while udp.get_available_packet_count() > 0:
			var pkt := udp.get_packet()
			client_ip = udp.get_packet_ip()
			client_port = udp.get_packet_port()
			if pkt.size() >= 40 and pkt.decode_u16(0) == 18458:
				if drive_from_pwm:
					for i in range(8):
						motor_cmd[i] = _pwm_to_thrust(pkt.decode_u16(8 + i * 2))
				got_ls = true
		if got_ls:
			if drive_from_pwm:
				_apply_motor_lag(delta)
			_send_state()
			_diag_rt += 1
		_diag_report()
		return

	# ASYNC (default): decouple the send from the receive. Previously Godot only sent its state when it
	# happened to receive a servo this tick (a ping-pong paced at AP's ~200 Hz), so a fresh command
	# waited up to a servo-interval just to be TRANSMITTED, then paid the async round-trip on top --
	# ~22 ms of dead-time before any response. Instead: drain all queued servos (keep newest), slew the
	# motors EVERY tick, and send fresh state EVERY tick (400 Hz). AP's recv_fdm reads a whole chunk and
	# memrchr-picks the newest complete message, so sending faster than AP consumes does NOT back up --
	# it just hands AP the freshest command/IMU immediately, cutting the command->servo latency.
	while udp.get_available_packet_count() > 0:
		var pkt := udp.get_packet()
		client_ip = udp.get_packet_ip()
		client_port = udp.get_packet_port()
		if pkt.size() >= 40 and pkt.decode_u16(0) == 18458:
			if drive_from_pwm:
				for i in range(8):
					motor_cmd[i] = _pwm_to_thrust(pkt.decode_u16(8 + i * 2))
			_diag_rt += 1
	if drive_from_pwm:
		_apply_motor_lag(delta)   # slew thrust toward the latest motor_cmd every tick (continuous, real)
	if client_port != 0:
		_send_state()             # push fresh state every tick so AP never waits for the next ping
	_diag_report()


# NATIVE in-process path: run the ported mode-30 flight stack (ApFlightStack) directly,
# with ZERO bridge latency. Feeds it the SAME NED/FRD state the UDP bridge sent (_send_state
# conversions) and the same latched keyboard command, gets 8 motor PWM back, and drives the
# EXISTING plant (thrust curve + motor lag + apply_force) so the physics is byte-identical to
# the bridge case -- only the ~20-40 ms round-trip is gone.
func _service_native(delta: float) -> void:
	var basis := drone.global_transform.basis
	var grounded := drone.global_position.y <= 0.0

	# TRUTH state in ArduPilot NED / body FRD (same P-basis conversions as _send_state)
	var accel_ef := P * _imu_accel_world              # earth-frame specific force (NED)
	var vel_ned := P * drone.linear_velocity
	var pos_ned := P * drone.global_position
	var gyro_frd := P * (basis.inverse() * drone.angular_velocity)
	var R := P * basis * P.inverse()
	var quat := R.get_rotation_quaternion()
	var th0: float = arm_theta[0]
	var th1: float = arm_theta[1]
	var td0: float = arm_theta_dot[0]
	var td1: float = arm_theta_dot[1]

	# SENSOR MODEL (harness-side; OFF by default): degrade the state mode-30 reads with
	# representative noise + measurement latency + EKF/INS lag, so the controller is validated
	# on realistic sensing rather than clean truth. The grader (_grade) stays on TRUTH.
	if sensor_noise_enabled:
		var truth := {
			"accel_ef": accel_ef, "gyro": gyro_frd, "quat": quat,
			"vel_ned": vel_ned, "pos_ned": pos_ned,
			"theta0": th0, "theta1": th1, "tdot0": td0, "tdot1": td1,
		}
		var s := _sensed_state(truth, delta)
		accel_ef = s["accel_ef"]; gyro_frd = s["gyro"]; quat = s["quat"]
		vel_ned = s["vel_ned"]; pos_ned = s["pos_ned"]
		th0 = s["theta0"]; th1 = s["theta1"]; td0 = s["tdot0"]; td1 = s["tdot1"]

	flightstack.set_state(accel_ef, vel_ned, pos_ned, true, true, quat, gyro_frd)

	# command: latched keyboard levels -> normalized sticks; Q/E -> yaw rate; SPACE -> park.
	# (auto_test drives lat_level/vert_level/park via _auto_drive, exactly as in the bridge path.)
	var lat_stick := lat_level / 4.0
	var up_stick := vert_level / 3.0
	var yaw := 0.0
	if Input.is_key_pressed(KEY_E): yaw += 1.0
	if Input.is_key_pressed(KEY_Q): yaw -= 1.0
	# reaction_ff_on=false -> tell the controller 0 arms so its reaction FF is skipped, while the physical
	# arms (built from n_arms) keep swinging -> isolates the FF's effect in the truth-based grader.
	flightstack.set_command(lat_stick, up_stick, yaw * NATIVE_YAW_RATE, park,
		(n_arms if reaction_ff_on else 0),
		Vector3(th0, th1, 0.0), Vector3(td0, td1, 0.0))

	# run the ported mode-30 loop -> 8 motor PWM, then slew the plant exactly like the bridge
	var pwm: PackedInt32Array = flightstack.update(grounded, delta)
	for i in range(8):
		motor_cmd[i] = _pwm_to_thrust(pwm[i])
	_apply_motor_lag(delta)


# ---- sensor model helpers (see the export block up top). Called only on the native path. ----
func _sensor_setup() -> void:
	if OS.get_environment("USE_SENSOR_NOISE") == "1":
		sensor_noise_enabled = true
	var v := OS.get_environment("SENSOR_SEED");         if v != "": sensor_seed = int(v)
	if OS.get_environment("SENSOR_POS_DRIFT") == "0":   sensor_pos_drift = false
	v = OS.get_environment("SENSOR_LAT_EKF_ATT_MS");    if v != "": lat_ekf_att_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_EKF_VEL_MS");    if v != "": lat_ekf_vel_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_EKF_POS_MS");    if v != "": lat_ekf_pos_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_ARM_IMU_MS");    if v != "": lat_arm_ang_imu_ms = float(v)
	sensor_rng.seed = sensor_seed

	# ring-buffer sized to the largest latency; per-group depths in ticks
	var ticks := float(Engine.physics_ticks_per_second)
	_d_imu = int(round(lat_imu_ms     / 1000.0 * ticks))
	_d_att = int(round(lat_ekf_att_ms / 1000.0 * ticks))
	_d_vel = int(round(lat_ekf_vel_ms / 1000.0 * ticks))
	_d_pos = int(round(lat_ekf_pos_ms / 1000.0 * ticks))
	_d_arm = int(round(lat_arm_ms     / 1000.0 * ticks))
	_d_arm_imu = int(round(lat_arm_ang_imu_ms / 1000.0 * ticks))
	_sb_len = maxi(maxi(maxi(_d_imu, _d_att), maxi(_d_vel, _d_pos)), maxi(_d_arm, _d_arm_imu)) + 1
	_hist.resize(_sb_len)
	_sb_idx = 0
	_sensor_seeded = false
	if sensor_noise_enabled:
		print("[sensor] model ON seed=%d lat(imu/att/vel/pos/arm/arm_imu ticks)=%d/%d/%d/%d/%d/%d drift=%d" % [
			sensor_seed, _d_imu, _d_att, _d_vel, _d_pos, _d_arm, _d_arm_imu, int(sensor_pos_drift)])


# Add fresh per-channel Gaussian noise (+ gyro bias walk / encoder quantization / dead-reckon
# drift) to the true state, push the sample into the delay ring, then read each channel back at
# its own measurement/EKF latency. Returns the sensed (noisy, delayed) snapshot.
func _sensed_state(truth: Dictionary, dt: float) -> Dictionary:
	var g := sensor_rng
	var d2r := deg_to_rad(1.0)

	# gyro: slow bias random walk (0.5 deg/s/sqrt(hr)) + white noise
	_gyro_bias += Vector3(g.randfn(), g.randfn(), g.randfn()) * (0.5 * d2r / sqrt(3600.0)) * sqrt(dt)
	var sg := sig_gyro_dps * d2r
	var gyro_n: Vector3 = truth["gyro"] + _gyro_bias + Vector3(g.randfn(), g.randfn(), g.randfn()) * sg

	# accel_ef: white noise (folds the broadband vibration RMS into one sigma)
	var accel_n: Vector3 = truth["accel_ef"] + Vector3(g.randfn(), g.randfn(), g.randfn()) * sig_accel

	# attitude: small random body rotation on the quaternion
	var sa := sig_att_deg * d2r
	var quat_n: Quaternion = (truth["quat"] as Quaternion) * Quaternion.from_euler(
		Vector3(g.randfn(), g.randfn(), g.randfn()) * sa)

	# velocity: white noise (IMU dead-reckoning)
	var vel_n: Vector3 = truth["vel_ned"] + Vector3(g.randfn(), g.randfn(), g.randfn()) * sig_vel

	# position: white noise + optional dead-reckon drift (horizontal N/E; baro holds D)
	if sensor_pos_drift:
		_pos_drift += Vector3(g.randfn(), 0.0, g.randfn()) * pos_drift_rate * sqrt(dt)
	var pos_n: Vector3 = truth["pos_ned"] + _pos_drift + Vector3(g.randfn(), g.randfn(), g.randfn()) * sig_pos

	# arm 1 (proximal): 2xIMU fused -> continuous angle (NO encoder LSB) + clean differential-gyro
	# rate. NOTE (documented simplification): the maneuver-coupled angle error -- the accel gravity
	# anchor tilting under the drone's catch acceleration -- is NOT modelled here; only its extra
	# fusion LATENCY is (read back at _d_arm_imu below). Revisit at hardware / if Packet D is fragile.
	var th0: float = float(truth["theta0"]) + g.randfn() * (sig_theta0_deg * d2r)
	var td0: float = float(truth["tdot0"]) + g.randfn() * (sig_thetadot0_dps * d2r)
	# arm 2 (distal): magnetic encoder -> quantize angle to LSB then add noise; differentiated rate.
	# NOTE (documented simplification): arm-2's ABSOLUTE angle inherits arm-1's drift (encoder is a
	# RELATIVE joint measurement); truth["theta1"] is fed absolute here, so that coupling is omitted.
	var lsb := deg_to_rad(enc_lsb_deg)
	var th1_q: float = (round(float(truth["theta1"]) / lsb) * lsb if lsb > 0.0 else float(truth["theta1"]))
	var th1: float = th1_q + g.randfn() * (sig_theta1_deg * d2r)
	var td1: float = float(truth["tdot1"]) + g.randfn() * (sig_thetadot1_dps * d2r)

	var snap := {
		"accel_ef": accel_n, "gyro": gyro_n, "quat": quat_n,
		"vel_ned": vel_n, "pos_ned": pos_n,
		"theta0": th0, "theta1": th1, "tdot0": td0, "tdot1": td1,
	}
	if not _sensor_seeded:
		for i in range(_sb_len):
			_hist[i] = snap.duplicate()
		_sensor_seeded = true
	_hist[_sb_idx] = snap

	# read each channel at its own latency (ring depth); advance the write index afterward
	var out := {
		"accel_ef": _read_ch("accel_ef", _d_imu),
		"gyro":     _read_ch("gyro", _d_imu),
		"quat":     _read_ch("quat", _d_att),
		"vel_ned":  _read_ch("vel_ned", _d_vel),
		"pos_ned":  _read_ch("pos_ned", _d_pos),
		"theta0":   _read_ch("theta0", _d_arm_imu),   # arm-1 fused angle: deeper (EKF-att-like) latency
		"theta1":   _read_ch("theta1", _d_arm),        # arm-2 encoder: clean, ~loop-rate
		"tdot0":    _read_ch("tdot0", _d_arm),         # arm-1 differential gyro: clean rate
		"tdot1":    _read_ch("tdot1", _d_arm),         # arm-2 differentiated encoder
	}
	_sb_idx = (_sb_idx + 1) % _sb_len
	return out


func _read_ch(key: String, depth: int):
	var ri := (_sb_idx - depth + _sb_len) % _sb_len
	return (_hist[ri] as Dictionary)[key]


# Print a one-line timing summary once per wall-clock second.
func _diag_report() -> void:
	var now := Time.get_ticks_usec()
	if _diag_t0 == 0:
		_diag_t0 = now
		return
	var elapsed_us := now - _diag_t0
	if elapsed_us < 1_000_000:
		return
	var secs := elapsed_us / 1_000_000.0
	var rt_hz := _diag_rt / secs
	var call_hz := _diag_calls / secs
	var rt_for_avg := _diag_rt if _diag_rt > 0 else 1
	var avg_block_ms := (float(_diag_block_us) / float(rt_for_avg)) / 1000.0
	print("[bridge] round-trips=%.0f/s  phys-calls=%.0f/s  render=%.0f fps  avg-block=%.2f ms  max-block=%.2f ms  timeouts=%d" % [
		rt_hz, call_hz, Engine.get_frames_per_second(),
		avg_block_ms, _diag_max_block_us / 1000.0, _diag_timeouts])
	_diag_rt = 0
	_diag_calls = 0
	_diag_block_us = 0
	_diag_max_block_us = 0
	_diag_timeouts = 0
	_diag_t0 = now


# First-order motor+prop spin-up lag: thrust cannot change instantly. Slew the
# actual thrust toward the commanded thrust with time constant MOTOR_TAU. This is
# what gives the vertical/attitude loops realistic phase margin — without it the
# sim reacts to throttle instantly and oscillates worse than real hardware would.
func _apply_motor_lag(dt: float) -> void:
	var alpha := 1.0 - exp(-dt / MOTOR_TAU)
	for i in range(8):
		motor_thrust[i] += (motor_cmd[i] - motor_thrust[i]) * alpha


func _pwm_to_thrust(pwm: int) -> float:
	var x := clampf((float(pwm) - 1000.0) / 1000.0, 0.0, 1.0)
	# F60PROV-LV 1950KV prop curve: thrust = (1-e)x + e x^2 (e = THRUST_EXPO).
	# ArduPilot pre-distorts its PWM via MOT_THST_EXPO=e so realized thrust stays
	# linear in the controller's command — set both to the same e or they fight.
	return T_MAX * ((1.0 - THRUST_EXPO) * x + THRUST_EXPO * x * x)


func _apply_forces() -> void:
	var basis := drone.global_transform.basis
	for i in range(8):
		var T: float = motor_thrust[i]
		if T == 0.0:
			continue
		drone.apply_force(basis * (motor_dir[i] * T), basis * motor_pos[i])
		drone.apply_torque(basis * (motor_dir[i] * (motor_spin[i] * K_M * T)))


func _ground_clamp() -> void:
	on_ground = drone.global_position.y <= 0.0
	if on_ground:
		var v := drone.linear_velocity
		if v.y < 0.0:
			v.y = 0.0
		v.x *= 0.8   # ground friction — a landed drone shouldn't skate on a frictionless floor
		v.z *= 0.8
		drone.linear_velocity = v
		var t := drone.global_transform
		t.origin.y = 0.0
		drone.global_transform = t


# Map the latched ACCEL-mode levels into RC PWM (injected into the bridge packet
# by _send_state). ModeAccel (mode 30) reads the PWM straight as a direct
# acceleration command — no shaping — so PWM is just a linear accel encoding:
#   ch1 (roll)     -> lateral accel, full rail at A / L (+-1.0 -> +-5g)
#   ch3 (throttle) -> vertical accel, 1000=-g (down), 1500=hover, 2000=+g (up)
#   ch2 (pitch)    -> forward axis: NEUTRAL; pinned to zero by the controller
#   ch4 (yaw)      -> Q/E rate, holds heading when centred
# If an axis flies inverted, flip its sign here (or set RCn_REVERSED in ArduPilot).
func _update_rc() -> void:
	rc[0] = RC_CENTER + int(round(lat_level / 4.0 * 500.0))   # lateral level -4..+4 -> 1000..2000
	rc[1] = 2000 if park else RC_CENTER                       # ch2: PARK flag (else forward neutral)

	# Vertical (ch3). While sitting on the ground at hover-or-below, rest at throttle
	# MIN (1000): ModeAccel holds GROUND_IDLE there anyway, and a mid-stick (hover)
	# value reads as a climb command to ArduPilot's arming gate ("Throttle too high",
	# AP_Arming_Copter.cpp:626), which used to block arming. Once a climb is commanded
	# (vert_level>0) or we're airborne, send the real accel mapping (1500 = hover).
	if on_ground and vert_level <= 0:
		rc[2] = 1000
	else:
		rc[2] = RC_CENTER + int(round(vert_level / 3.0 * 500.0))  # level -3..+3 -> 1000..2000

	var yaw := 0
	if Input.is_key_pressed(KEY_E): yaw += 1   # yaw right (CW)
	if Input.is_key_pressed(KEY_Q): yaw -= 1   # yaw left (CCW)
	rc[3] = RC_CENTER + yaw * RC_DEFLECT


func _send_state() -> void:
	var basis := drone.global_transform.basis
	var pos_ned := P * drone.global_position
	var vel_ned := P * drone.linear_velocity
	var gyro_frd := P * (basis.inverse() * drone.angular_velocity)

	# Specific force = the drone's TRUE acceleration minus gravity (_imu_accel_world, computed each tick
	# in _physics_process from dv/dt). This is what a real accelerometer measures: motor thrust + drag +
	# the pendulum hinge reaction -- everything except gravity. (The old code modelled only (thrust+drag)/m
	# and was blind to the arm reaction, which let the controller's integral cancel the reaction-force FF.)
	# For the bare drone this equals (thrust+drag)/m exactly. On the ground the body is clamped, so the
	# finite-difference is meaningless there -> report the static +g (level, at rest).
	var accel_frd := P * (basis.inverse() * _imu_accel_world)
	if on_ground:
		accel_frd = Vector3(0, 0, -GRAVITY)

	var R := P * basis * P.inverse()
	var q := R.get_rotation_quaternion()

	var state := {
		"timestamp": sim_time,
		# Async (no real-time handshake). With lockstep, AP blocks until Godot's
		# reply each frame; on Godot's single main thread that round-trip can't
		# reliably land inside AP's 5 ms budget, so AP's loop sat at ~178 Hz (just
		# under the 90%-of-200 arming gate) — or, if we spin to force it, input
		# starves. no_lockstep frees AP to run its loop at SIM_RATE_HZ and read
		# our state as it arrives. timestamp is still honoured (time-sync stays
		# on), so physics stays accurate; we only give up deterministic replay.
		"no_lockstep": not lockstep,
		"imu": {
			"gyro": [gyro_frd.x, gyro_frd.y, gyro_frd.z],
			"accel_body": [accel_frd.x, accel_frd.y, accel_frd.z],
		},
		"position": [pos_ned.x, pos_ned.y, pos_ned.z],
		"velocity": [vel_ned.x, vel_ned.y, vel_ned.z],
		"quaternion": [q.w, q.x, q.y, q.z],
		"rc": {
			"rc_1": rc[0], "rc_2": rc[1], "rc_3": rc[2], "rc_4": rc[3],
			# rc_6..rc_10: pendulum reaction-force feed-forward inputs (arm count, per-arm angle/rate).
			# ch5 is skipped (FLTMODE_CH). See _update_pendulum().
			"rc_6": pend_rc[0], "rc_7": pend_rc[1], "rc_8": pend_rc[2],
			"rc_9": pend_rc[3], "rc_10": pend_rc[4],
		},
	}
	udp.set_dest_address(client_ip, client_port)
	udp.put_packet((JSON.stringify(state) + "\n").to_utf8_buffer())


# ---- Independent acceleration grader (validation, SPEC §7) ----
# Truth = the rigid body's OWN kinematic acceleration (dv/dt), computed here and NEVER
# put in the state packet, so it is independent of the IMU/EKF signal the controller
# closes on. Compare against the commanded accel (the same level->accel mapping the
# firmware decodes from RC) to confirm realised base accel == command. Bare drone:
# base == CoM. Yaw is held ~0, so Godot world x = heading lateral, world y = up.
# NOTE: in sim both signals ultimately derive from Godot truth, so this validates the
# FF/trim math (incl. zero-command drift), NOT hardware bias rejection (that needs the
# Phase-4 noise model). lat true at lat_level==0 should sit at ~0 => no creep.
const GRADE_LAT_CMD_MSS := 12.0   # must match mode_accel.cpp ACCEL_LAT_CMD_MSS (levels -> +-3/6/9/12)
var _grade_prev_vel := Vector3.ZERO
var _grade_t := 0.0
# ---- transient trace: on any command change, dump per-physics-frame truth accel for a short
# window so settle time (time to enter & HOLD within 5% of command) is measured in Python at
# full 400 Hz resolution — the periodic [grade] line below is far too coarse to see a 40 ms rise.
const TRACE_WINDOW := 0.15   # seconds of per-frame trace after each command change
var _tr_prev_lat_cmd := 0.0
var _tr_prev_up_cmd := 0.0
var _tr_t0 := -1.0           # sim_time at the active step's command change (<0 = idle)
var _tr_seeded := false
func _grade(delta: float) -> void:
	var accel_world := (drone.linear_velocity - _grade_prev_vel) / maxf(delta, 1e-6)
	_grade_prev_vel = drone.linear_velocity
	if on_ground:
		return
	var a_lat_true := accel_world.x
	var a_up_true := accel_world.y
	var a_lat_cmd := lat_level / 4.0 * GRADE_LAT_CMD_MSS
	var a_up_cmd := vert_level / 3.0 * GRAVITY

	# detect a command change on either axis -> (re)open a fresh trace window
	if not _tr_seeded:
		_tr_prev_lat_cmd = a_lat_cmd
		_tr_prev_up_cmd = a_up_cmd
		_tr_seeded = true
	if absf(a_lat_cmd - _tr_prev_lat_cmd) > 1e-3 or absf(a_up_cmd - _tr_prev_up_cmd) > 1e-3:
		_tr_t0 = sim_time
		print("[step] t=%.3f lat %6.2f->%6.2f up %6.2f->%6.2f" % [
			sim_time, _tr_prev_lat_cmd, a_lat_cmd, _tr_prev_up_cmd, a_up_cmd])
		_tr_prev_lat_cmd = a_lat_cmd
		_tr_prev_up_cmd = a_up_cmd
	# emit a per-frame trace line for the window following a command change
	if _tr_t0 >= 0.0:
		var dt_ms := (sim_time - _tr_t0) * 1000.0
		if dt_ms <= TRACE_WINDOW * 1000.0:
			print("[trace] dt=%6.1f lat_cmd=%6.2f lat=%7.2f up_cmd=%6.2f up=%7.2f" % [
				dt_ms, a_lat_cmd, a_lat_true, a_up_cmd, a_up_true])
		else:
			_tr_t0 = -1.0

	_grade_t += delta
	if _grade_t >= 0.25:
		_grade_t = 0.0
		# Forward axis = NED North = -(Godot z). Must stay pinned at its captured reference (never
		# drift fore/aft). Log absolute forward pos/vel + the lateral pos so drift can be correlated
		# with the lateral maneuvers. accel_world.z is the forward accel (should be ~0).
		var fwd_pos := -drone.global_position.z
		var fwd_vel := -drone.linear_velocity.z
		var lat_pos := drone.global_position.x
		var lat_vel := drone.linear_velocity.x
		var yaw_deg := rad_to_deg(drone.global_transform.basis.get_euler().y)
		print("[grade] lat cmd=%6.2f true=%6.2f err=%+5.2f | up cmd=%6.2f true=%6.2f err=%+5.2f (m/s^2)" % [
			a_lat_cmd, a_lat_true, a_lat_true - a_lat_cmd, a_up_cmd, a_up_true, a_up_true - a_up_cmd])
		print("[fwd] t=%6.2f fwd_pos=%+7.3f fwd_vel=%+7.3f | lat_pos=%+8.2f lat_vel=%+7.2f yaw_deg=%+7.2f" % [
			sim_time, fwd_pos, fwd_vel, lat_pos, lat_vel, yaw_deg])


# Headless auto-test driver (AUTOTEST=1): sets the same latched levels the keyboard would, on a
# schedule. Phase 0 parks to auto-take-off (needs ArduPilot armed + in ACCEL, done by the python
# controller); once airborne it runs auto_profile; then holds. Grader keeps logging throughout.
func _auto_drive() -> void:
	var alt := drone.global_position.y
	var vy := drone.linear_velocity.y
	match auto_phase:
		0:  # Climb to a safe altitude, then measure IMMEDIATELY. The workflow must NOT depend on the
			# controller holding a hover (with the current calibration it can't) -- we only need to be
			# high enough that drift during the ~5 s profile can't reach the ground. Gentle (+3.3) keeps
			# the climb velocity modest so drag barely confounds the vertical measurements. The lateral
			# (catch) axis is independent of the vertical drift, so its data is clean regardless.
			park = false
			lat_level = 0
			vert_level = 1
			if alt > 25.0:
				auto_phase = 1
				auto_t0 = sim_time
				print("[auto] airborne (alt=%.1f, vy=%.1f); starting profile" % [alt, vy])
		1:  # scripted accel profile
			park = false
			var tp := sim_time - auto_t0
			var active := false
			for step in auto_profile:
				if tp < float(step[0]):
					lat_level = int(step[1])
					vert_level = int(step[2])
					active = true
					break
			if not active:
				auto_phase = 2
				print("[auto] profile complete at t=%.1f" % tp)
		2:  # done -- best-effort hold (may drift; the profile data is already captured)
			park = true
			lat_level = 0
			vert_level = 0


# ============ Phase 7: single-pendulum PHYSICS validation (no controller) ============
# Freeze the drone into a fixed anchor, release the arm, and measure its dynamics against the
# analytical pendulum. Point-mass bob: all the arm's mass is the Bob sphere at PEND_L, the rod is a
# massless visual, so the simple-pendulum sqrt(L/g) relations hold exactly. Two modes via env
# PENDULUM_TEST:
#   =1 HANGING  -> small-angle period should match  T = 2*pi*sqrt(L/g)  (1.554 s / 0.64 Hz at L=0.60)
#   =2 INVERTED -> divergence timescale should match tau = sqrt(L/g)     (0.247 s at L=0.60)
# Both derive from sqrt(g/L); matching them closes the Phase-7 checkpoint (Phase-1 numbers).
var _pend_inverted := false
var _pend_t := 0.0
var _pend_prev := 0.0
var _pend_cross: Array = []
var _pend_done := false


# Attach the first n_arms pendulums; fully detach the rest. IMPORTANT: to detach we clear BOTH joint
# node paths. Clearing only node_b (the old code) leaves node_a=Drone, which makes Godot anchor the
# DRONE to a fixed point in the world -> it can never take off (that was the 0-arm takeoff regression).
func _setup_arms() -> void:
	var bodies := [$Arm, $Arm2]
	var pivots := [$Pivot, $Pivot2]
	for i in range(bodies.size()):
		if i < n_arms:
			arm_bodies.append(bodies[i])
			arm_prev_theta.append(0.0)
			arm_has_prev.append(false)
		else:
			# Fully remove the unused arm+joint from the sim so it can't influence the drone at all
			# (a lingering joint would add phantom mass / reaction to the "bare" frame). Clear the joint
			# paths first (breaks the constraint this frame), then free the nodes.
			pivots[i].node_a = NodePath("")   # clear BOTH -> no world anchor, no coupling
			pivots[i].node_b = NodePath("")
			pivots[i].queue_free()
			bodies[i].queue_free()
	print("[arms] %d pendulum arm(s) attached" % n_arms)


# Sample each active arm's angle (from straight-down, + toward +lateral/+X) and angular rate, and pack
# them + the arm count into pend_rc (rc_6..rc_10) for _send_state. The controller uses these to cancel
# the reaction force. theta comes from the arm's orientation (rod dir = basis*(0,-1,0)); yaw is held ~0
# so world X = heading lateral, world Y = up. theta_dot is a clean finite-difference (sim theta is smooth).
func _update_pendulum(delta: float) -> void:
	pend_rc[0] = 1000 + n_arms * 100
	pend_rc[1] = 1500; pend_rc[2] = 1500; pend_rc[3] = 1500; pend_rc[4] = 1500
	arm_theta[0] = 0.0; arm_theta[1] = 0.0
	arm_theta_dot[0] = 0.0; arm_theta_dot[1] = 0.0
	for i in range(n_arms):
		var rod := arm_bodies[i].global_transform.basis * Vector3(0, -1, 0)  # pivot->bob direction (world)
		var theta := atan2(rod.x, -rod.y)                                     # 0 = straight down, + toward +X
		var dtheta := 0.0
		if arm_has_prev[i]:
			dtheta = wrapf(theta - arm_prev_theta[i], -PI, PI) / maxf(delta, 1e-6)
		arm_prev_theta[i] = theta
		arm_has_prev[i] = true
		arm_theta[i] = theta            # raw floats for the native path (no PWM round-trip)
		arm_theta_dot[i] = dtheta
		pend_rc[1 + i * 2] = 1500 + int(round(clampf(theta, -PI, PI) / PI * 500.0))
		pend_rc[2 + i * 2] = 1500 + int(round(clampf(dtheta, -PEND_RATE_MAX, PEND_RATE_MAX) / PEND_RATE_MAX * 500.0))


# Independent check of the reaction-FF derivation: the TRUE reaction is sum_i m_i (g_vec - a_bob_i)
# from each bob's finite-differenced world acceleration; the FORMULA re-derives it from (theta, rate,
# a_d) via the coupled EOM -- IDENTICAL math to mode_accel.cpp. If the derivation is right they match.
func _react_validate(a_d: Vector3, delta: float) -> void:
	if n_arms < 1 or drone.global_position.y <= 0.0:
		return
	var dt := maxf(delta, 1e-6)
	var g := GRAVITY
	var ab1 := (arm_bodies[0].linear_velocity - _rv_prev_b1) / dt
	var ab2 := Vector3.ZERO
	_rv_prev_b1 = arm_bodies[0].linear_velocity
	if n_arms >= 2:
		ab2 = (arm_bodies[1].linear_velocity - _rv_prev_b2) / dt
		_rv_prev_b2 = arm_bodies[1].linear_velocity
	if not _rv_seeded:
		_rv_seeded = true
		return
	var m1 := 0.10
	var m2 := 0.10 if n_arms >= 2 else 0.0
	# TRUE reaction (accel units = R / m_d); g_vec = (0, -g)
	var rt_lat := m1 * (-ab1.x) + m2 * (-ab2.x)
	var rt_up := m1 * (-g - ab1.y) + m2 * (-g - ab2.y)
	# FORMULA reaction (identical to mode_accel.cpp) using the ACTUAL cart accel a_d; g_eff = g_vec - a_d
	var gx := -a_d.x
	var gy := -(g + a_d.y)
	var M := m1 + m2
	var L1 := 0.6
	var L2 := 0.6
	var th1 := float(arm_theta[0])
	var w1 := float(arm_theta_dot[0])
	var th2 := float(arm_theta[1]) if n_arms >= 2 else 0.0
	var w2 := float(arm_theta_dot[1]) if n_arms >= 2 else 0.0
	var c1 := cos(th1)
	var s1 := sin(th1)
	var c2 := cos(th2)
	var s2 := sin(th2)
	var dD := th1 - th2
	var cD := cos(dD)
	var sD := sin(dD)
	var b1 := M * (gx * c1 + gy * s1) - m2 * L2 * sD * w2 * w2
	var b2 := (gx * c2 + gy * s2) + L1 * sD * w1 * w1
	var det := L1 * L2 * (M - m2 * cD * cD)
	var dd1 := (L2 * b1 - m2 * L2 * cD * b2) / det
	var dd2 := (-L1 * cD * b1 + M * L1 * b2) / det if n_arms >= 2 else 0.0
	var rf_lat := M * gx - M * L1 * (dd1 * c1 - w1 * w1 * s1)
	var rf_up := M * gy - M * L1 * (dd1 * s1 + w1 * w1 * c1)
	if n_arms >= 2:
		rf_lat -= m2 * L2 * (dd2 * c2 - w2 * w2 * s2)
		rf_up -= m2 * L2 * (dd2 * s2 + w2 * w2 * c2)
	_rv_err_lat += (rf_lat - rt_lat) * (rf_lat - rt_lat)
	_rv_err_up += (rf_up - rt_up) * (rf_up - rt_up)
	_rv_ref_lat += rt_lat * rt_lat
	_rv_ref_up += rt_up * rt_up
	_rv_n += 1
	if _rv_n % 400 == 0:
		var el := sqrt(_rv_err_lat / _rv_n)
		var eu := sqrt(_rv_err_up / _rv_n)
		var rl := sqrt(_rv_ref_lat / _rv_n)
		var ru := sqrt(_rv_ref_up / _rv_n)
		print("[react] n=%d | rms_err(lat,up)=(%.4f,%.4f) rms_true=(%.4f,%.4f) rel=(%.1f%%,%.1f%%)" % [
			_rv_n, el, eu, rl, ru, 100.0 * el / maxf(rl, 1e-6), 100.0 * eu / maxf(ru, 1e-6)])


func _pend_setup() -> void:
	use_ardupilot = false
	drive_from_pwm = false
	drone.freeze = true
	drone.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
	arm.linear_velocity = Vector3.ZERO
	arm.angular_velocity = Vector3.ZERO
	# The arm rotates about the beam axis (hinge). Rotating its basis about Z sets the release angle;
	# the bob (local -Y) swings in the world X-Y (lateral-vertical) plane.
	if OS.get_environment("PENDULUM_TEST") == "2":
		_pend_inverted = true
		# bob near straight-UP (arm flipped ~180 deg), 2 deg off the unstable top
		arm.global_transform = Transform3D(Basis(Vector3(0, 0, 1), deg_to_rad(178.0)), Vector3.ZERO)
		var tau := sqrt(PEND_L / GRAVITY)
		print("[pend] INVERTED  L=%.3f  analytic tau=sqrt(L/g)=%.4f s   (theta ~ theta0*cosh(t/tau))" % [PEND_L, tau])
	else:
		# bob near straight-DOWN, small angle -> clean small-oscillation period
		arm.global_transform = Transform3D(Basis(Vector3(0, 0, 1), deg_to_rad(8.0)), Vector3.ZERO)
		var period := TAU * sqrt(PEND_L / GRAVITY)
		print("[pend] HANGING   L=%.3f  analytic T=2pi*sqrt(L/g)=%.4f s   f=%.4f Hz" % [PEND_L, period, 1.0 / period])


func _pend_measure(delta: float) -> void:
	_pend_t += delta
	var r := arm.to_global(Vector3(0, -PEND_L, 0))   # bob world position; the pivot is at the origin
	if _pend_t < 0.006:
		# confirm the arm swings in the lateral (X-Y) plane -> r.z stays ~0 (else rotate Pivot 90 about Y)
		print("[pend] plane check: bob r=(%.3f, %.3f, %.3f)  (expect z~0 for a lateral swing)" % [r.x, r.y, r.z])
	if _pend_t > 20.0:
		print("[pend] TIMEOUT: no result in 20 s -- check the hinge axis / swing plane (see plane check)")
		get_tree().quit()
		return
	if _pend_inverted:
		var psi := atan2(r.x, r.y)          # angle from straight-up; ~0 at release
		if absf(psi) >= deg_to_rad(15.0) and not _pend_done:
			_pend_done = true
			var ratio := absf(psi) / deg_to_rad(2.0)
			var tau_meas := _pend_t / log(ratio + sqrt(ratio * ratio - 1.0))   # inverse cosh: theta=theta0*cosh(t/tau)
			var tau := sqrt(PEND_L / GRAVITY)
			print("[pend] INVERTED reached %.1f deg at t=%.4f s -> tau_meas=%.4f s  analytic=%.4f s  err=%+.2f%%" % [
				rad_to_deg(psi), _pend_t, tau_meas, tau, (tau_meas / tau - 1.0) * 100.0])
			get_tree().quit()
	else:
		var theta := atan2(r.x, -r.y)       # angle from straight-down; ~0 at rest
		if _pend_prev < 0.0 and theta >= 0.0:   # rising zero-crossings are one full period apart
			_pend_cross.append(_pend_t)
			var n := _pend_cross.size()
			if n >= 2:
				var meas: float = (_pend_cross[n - 1] - _pend_cross[0]) / float(n - 1)
				var analytic := TAU * sqrt(PEND_L / GRAVITY)
				print("[pend] periods=%d  T_meas=%.4f s (f=%.4f Hz)  analytic=%.4f s  err=%+.2f%%" % [
					n - 1, meas, 1.0 / meas, analytic, (meas / analytic - 1.0) * 100.0])
				if n >= 6:
					get_tree().quit()
		_pend_prev = theta
