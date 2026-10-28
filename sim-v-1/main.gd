extends Node3D

const SENSOR_MODEL_SCRIPT := preload("res://simulation/sensor_model.gd")
const PENDULUM_CHAIN_SCRIPT := preload("res://simulation/pendulum_chain.gd")
const MOTOR_DYNAMICS_SCRIPT := preload("res://simulation/motor_dynamics.gd")
const VEHICLE_PLANT_SCRIPT := preload("res://simulation/vehicle_plant.gd")
const SITL_PROTOCOL_SCRIPT := preload("res://simulation/sitl_protocol.gd")
const PILOT_INPUT_SCRIPT := preload("res://simulation/pilot_input.gd")
const AUTO_PROFILE_DRIVER_SCRIPT := preload("res://simulation/auto_profile_driver.gd")
const ACCELERATION_GRADER_SCRIPT := preload("res://simulation/acceleration_grader.gd")
const SCENARIO_VISUALS_SCRIPT := preload("res://simulation/scenario_visuals.gd")
const SWINGUP_RUN_TRACKER_SCRIPT := preload("res://simulation/swingup_run_tracker.gd")

@onready var drone: RigidBody3D = $Drone
@onready var arm: RigidBody3D = $Arm

@export var use_ardupilot := true
@export var use_native := false          # run the native C++ mode-30 flight stack IN-PROCESS (no UDP
										 # bridge, no lag). Overrides use_ardupilot when true. See native/.
@export var drive_from_pwm := true
@export var lateral_step_test := false   # set true & run scene: measures peak lateral accel + rise time
@export var response_video_overlay := false  # two top charts for a dedicated arm-drop recording
@export var graph_window_seconds := 6.0      # slower, easier-to-follow scrolling for recordings
@export var post_release_reaction_ff := true # comparison runs change only the post-release FF
@export var enable_pendulum := true      # false -> detach & hide the arm so the bare drone flies unperturbed
@export var pendulum_test := false       # Phase 7: freeze drone, release arm, measure period/tau vs sqrt(g/L)
										 # (env PENDULUM_TEST=1 hanging, =2 inverted; no bridge, no motors)

const MASS := 1.0          # drone BODY mass [kg]. Authoritative: _ready() sets drone.mass = MASS,
						   # overriding the .tscn value. Arm/bob mass (0.1 kg) comes from main.tscn.
const INERTIA := Vector3(0.01616, 0.04272, 0.02674)
const K_M := 0.01
const GRAVITY := 9.8
const BODY_DRAG_K := 0.03   # A6: quadratic aero drag coeff [1/m]; F = -k|v|v applied at the CoM. Near-zero at
							# the low speeds of actual use (~0.04 m/s^2 at 1 m/s), realistic ~quadratic shape.
							# Replaces the old linear linear_damp=0.05 placeholder. Calibrate at hardware.
const ARM_DRAG_K := 0.003      # A3: bob quadratic air drag [N·s²/m²] -- small (edge-on rod, low frontal area)
const HINGE_VISC := 0.002      # A2: hinge VISCOUS friction [N·m·s/rad] (bearing + encoder drag)
const HINGE_COUL := 0.003      # A2: hinge COULOMB (dry/stiction) friction [N·m]
const HINGE_OMEGA_EPS := 0.05  # Coulomb smoothing half-width [rad/s]: below this friction ramps linearly (no chatter)
const GYRO_KH := 0.0095         # A5: rotor gyro coeff = I_rotor/sqrt(k_T); h_i = KH*sqrt(T_i)*spin*axis [kg·m²/s]
const PEND_L := 0.6           # pivot->bob distance [m]; MUST match the Bob CollisionShape offset in main.tscn
const PEND_RATE_MAX := 10.0   # rad/s full-scale for the theta-dot bridge channel; MUST match mode_accel.cpp PEND_RATE_MAX
const T_MAX := 19.3          # F60PROV-LV 1950KV: 1966 g x 9.81 = 19.3 N max per motor
const THRUST_EXPO := 0.35    # prop thrust curve thrust=(1-e)x+e x^2; MUST match MOT_THST_EXPO
# Motor+prop thrust lag is ASYMMETRIC: spin-UP dumps current into the prop (fast, current-limited);
# spin-DOWN on a freewheeling ESC only sheds energy through aero drag (slower). Active motor braking
# on the real ESC would pull TAU_DOWN back toward TAU_UP. Both are estimates — measure on the bench.
const MOTOR_TAU_UP := 0.03    # thrust RISING time constant [s]
const MOTOR_TAU_DOWN := 0.045 # thrust FALLING time constant [s] (~1.5x; = TAU_UP if ESC braking is on)

# Battery sag: all 8 motors share ONE pack, so TOTAL thrust demand -- a proxy for total current, since
# prop torque ~ thrust and BLDC current ~ torque -- droops the shared pack voltage:
#     V/V_nom = 1 - BATT_SAG_K * sum(thrust_i).
# At a fixed throttle position thrust ~ V^2, so EVERY motor's realized thrust scales by (V/V_nom)^2 at
# once. This is a CORRELATED hit (unlike the per-motor lag/noise) that bites hardest at the catch's peak
# demand, and the voltage-blind flight code cannot see it to compensate. Default ~ a good low-R pack.
var batt_sag_k := 0.0008        # per-Newton voltage droop [1/N]; env BATT_SAG_K; 0 disables. BENCH-MEASURE.
const BATT_SAG_FLOOR := 0.70    # clamp: never model deeper than 30% loss (current limiting / you'd back off)
var _sag_factor := 1.0          # last tick's (V/V_nom)^2, applied to every motor (explicit -> no I<->V loop)

# A4: advance-ratio thrust droop. Moving ALONG a motor's thrust axis raises its axial inflow, and a
# fixed-pitch prop unloads (thrust falls with airspeed): T *= max(0, 1 - ADV_RATIO_K * v_axial), only for
# v_axial>0. Unlike sag this is at FIXED RPM (aero, not electrical) -> it does NOT change rotor angular
# momentum, so the gyro term keeps reading the un-drooped thrust. Small at demo speeds; conservative.
var adv_ratio_k := 0.015        # thrust droop per (m/s) of axial inflow [s/m]; env ADV_RATIO_K; 0 disables. BENCH.
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

# ---- Packet D: onboard balance mode (env BALANCE_ONBOARD=1). The balance controller runs IN-PROCESS and
# drives ModeAccel's lateral stick, closing the inverted-pendulum loop on the REAL accel inner loop (not the
# idealized first-order cart of balance_test). Vertical axis hover-holds (up_stick=0); the reaction FF carries
# the arm weight. Metrics graded on TRUTH arm angle. See Native-Flightstack-Plan.md, Packet D.
var balance_onboard := false
var balance_ctrl                          # ArmBalance (1 link) or ArmBalance2 (2 links) GDExtension
var balance_init_deg := 4.0               # link1 start lean from upright (+ -> +X)
var balance_init2_deg := 4.0              # link2 start lean (double only)
var balance_alt := 10.0                   # hover altitude to balance at [m]
const BAL_RUN_TIME := 20.0
const BAL_GAINS2 := {"k1": -213.542773, "k1d": -14.504648, "k2": 260.829154,
	"k2d": 45.149116, "ks": 1.414214, "kv": 4.525040}   # compute_lqr2.py, 0.7/0.6, R=1 (matches balance2_test)
var _bal_t := 0.0
var _bal_max1 := 0.0
var _bal_max2 := 0.0
var _bal_sum_cmd2 := 0.0
var _bal_n := 0
var _bal_sat := false
var _bal_done := false
var _bal_bump_t := 7.5
var _bal_bump_dv := 0.0                    # mid-run lateral velocity bump [m/s] (0 = none); env BALANCE_BUMP_DV
var _bal_bumped := false
var bal_graph_u                            # live strip-chart: a_cmd vs REAL realised lateral accel
var bal_graph_vertical                     # optional response-video chart: vertical command vs realised
var bal_angle_readout: Label               # compact numeric link-angle display beside the vehicle
var _overlay_balance_on := false
var _overlay_vehicle_command := "HOLD POSITION"
var _overlay_reaction_ff_on := false
var _recording_auto_exit := false
var _bal_previous_cmd := 0.0
var _bal_sum_jerk2 := 0.0
var _bal_jerk_n := 0
var _bal_released := false                 # after BAL_RUN_TIME: stop balancing, let the arms drop, hold a hover
var _bal_post_t := 0.0

# ---- SWINGUP mode (env SWINGUP=1): energy-swingup from hanging -> catch into balance, on the REAL ModeAccel.
# The portable C++ SwingUp controller drives lat_stick AND up_stick (it uses the vertical axis to pump).
var swingup_onboard := false
var swingup_ctrl                           # SwingUp (GDExtension)
var swingup_alt := 10.0                    # hover altitude to swing up at [m]
var swingup_init_deg := 5.0                # arm start lean from HANGING (breaks symmetric rest); env SWINGUP_INIT_DEG
var swingup_hold_time := 8.0               # balance this long AFTER the catch, then RELEASE (arms drop); env SWINGUP_HOLD_TIME
const SWINGUP_RUN_TIME := 18.0             # hard cap if never caught (swing up ~3-4 s + hold, then report)
var swingup_run_tracker = SWINGUP_RUN_TRACKER_SCRIPT.new()
var _su_post_t := 0.0                         # time since release (for the headless quit)
var _su_dbg := 0.0
var _su_graph_n := 0                         # tick counter to decimate the live strip-chart pushes
var _su_lat_cmd := 0.0                      # last lat_stick*12 [m/s^2], for debug
var _su_up_cmd := 0.0                       # last up_stick*g [m/s^2], for debug

# ---- headless auto-test (set env AUTOTEST=1): scripted takeoff + accel profile, keyboard ignored.
# Off by default, so manual flying is unaffected. profile rows: [t_end_s, lat_level(-4..4), vert_level(-3..3)]
var auto_test := false
var auto_profile_driver = AUTO_PROFILE_DRIVER_SCRIPT.new()
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
var pendulum_chain = PENDULUM_CHAIN_SCRIPT.new()
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
var sensor_model = SENSOR_MODEL_SCRIPT.new()

# ---- representative sensor model (harness-side; behind a switch, ON by default) ----
# On real hardware the flight stack reads a noisy IMU through ArduPilot's INS filtering +
# EKF3, and the arm angle/rate from a hinge encoder + arm gyro -- NOT the clean Godot truth
# the native path feeds by default. This models that (noise + measurement latency + EKF lag)
# so mode-30 can be validated under representative sensing. Grounded in Project-Plan Phase 4
# + Balance-Controller-Design 5. Imperfections live HERE (harness), never in the controller.
# Enable with USE_SENSOR_NOISE=1. The grader (_grade) stays on TRUTH, so it still measures
# realised-vs-commanded accel honestly while the controller runs on degraded sensors.
@export var sensor_noise_enabled := true
@export var sensor_seed := 1
@export var arm_imu_distance := 0.10 # metres from proximal pivot; set to actual mounting location
@export var sig_arm_accel := 0.35 # local accelerometer sigma [m/s^2]
@export var sig_arm_gyro_dps := 0.4 # local gyro sigma [deg/s]
@export var encoder_rate_tau := 0.02 # differentiated encoder low-pass [s]
@export var arm_observer_tau := 0.5 # complementary gravity correction [s]
# per-channel Gaussian 1-sigma (SI). accel folds a broadband vibration RMS into one sigma.
@export var sig_gyro_dps := 0.05           # gyro noise [deg/s]              (Phase 4)
@export var sig_accel := 0.25              # accel_ef broadband WHITE [m/s^2] ~5mg sensor + residual; the
										   # coherent rotor line is pulled OUT into the vib tone below (C4).
# C4: rotor-vibration TONE. Real IMU vibration is not white -- it is a throttle-coupled line at rotor
# frequency, which the estimator can partly track / be fooled by (unlike white noise). Modelled as a
# sinusoid: amplitude grows with total thrust (more throttle -> more vibration), frequency ~ sqrt(thrust)
# (an RPM proxy), decorrelated phase per axis. At hover its RMS ~ the vibration part removed from sig_accel
# above (power-preserving). NOTE: at the 400 Hz sensor tick this carries only sub-~200 Hz lines faithfully;
# true high-frequency ALIASING into the control band would need a multi-rate IMU model (not present here).
@export var vib_accel := 0.35              # tone amplitude at hover [m/s^2]; env VIB_ACCEL; 0 disables
@export var vib_f_hover := 90.0            # tone fundamental at hover [Hz]; scales with sqrt(thrust)
@export var sig_att_deg := 0.1             # EKF attitude noise [deg]
# Arm 1 uses measured local IMUs; downstream joints use quantized encoders.
@export var sig_theta1_deg := 0.10         # arm-2 angle [deg]  (magnetic encoder)
@export var enc_lsb_deg := 0.05            # arm-2 encoder quantization step [deg] (~13-bit)
@export var lat_imu_ms := 2.5              # accel_ef, gyro (raw INS, ~loop rate)
@export var lat_ekf_att_ms := 8.0          # EKF attitude output-observer lag
@export var lat_ekf_vel_ms := 20.0         # EKF velocity lag
@export var lat_ekf_pos_ms := 20.0         # EKF position lag
@export var lat_arm_ms := 2.5              # arm-2 encoder + differential-gyro read (clean channels)
@export var lat_arm_ang_imu_ms := 8.0      # arm-1 IMU-FUSED angle only (attitude-observer lag ~EKF att)

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
	drone.linear_damp = 0.0    # Godot's built-in damping is LINEAR (a=-c*v), which over-damps at low speed.
							   # Replaced by an explicit QUADRATIC drag (F=-k|v|v, BODY_DRAG_K) in _apply_forces
							   # -- realistic shape, near-zero at the low speeds of actual use. See A6.
	drone.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	drone.angular_damp = 0.0

	P = Basis(Vector3(0, 1, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0))

	if OS.get_environment("PENDULUM_TEST") != "":
		pendulum_test = true
	if pendulum_test:
		_pend_setup()
		return
	# A plain editor "Run" (F5) sets no env vars -> default to the SWING-UP demo (native + noise + 2-arm
	# double: swing up -> catch -> hold -> release). Any explicit env var (a headless test command) still
	# selects its own mode (e.g. BALANCE_ONBOARD=1 for the balance demo).
	var editor_demo := OS.get_environment("NUM_ARMS") == "" and OS.get_environment("ENABLE_PENDULUM") == "" \
		and OS.get_environment("AUTOTEST") == "" and OS.get_environment("USE_NATIVE") == "" \
		and OS.get_environment("BALANCE_ONBOARD") == "" and OS.get_environment("SWINGUP") == ""
	if editor_demo:
		sensor_noise_enabled = true

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
	if editor_demo:
		n_arms = 2
	_setup_arms()

	auto_test = OS.get_environment("AUTOTEST") == "1"
	if auto_test:
		print("[auto] AUTOTEST mode: scripted takeoff + accel profile; keyboard ignored")
	reaction_ff_on = OS.get_environment("REACTION_FF") != "0"   # only "0" disables; arms stay attached either way
	var post_ff_env := OS.get_environment("POSTREL_FF")
	if post_ff_env != "":
		post_release_reaction_ff = post_ff_env != "0"
	_recording_auto_exit = OS.get_environment("RECORDING_AUTO_EXIT") == "1"
	var graph_window_env := OS.get_environment("GRAPH_WINDOW_SECONDS")
	if graph_window_env != "":
		graph_window_seconds = maxf(float(graph_window_env), 1.0)
	if not reaction_ff_on:
		print("[accel] reaction-force FF DISABLED (REACTION_FF=0): arms swing but are NOT compensated")
	_rv_on = OS.get_environment("REACT_VALIDATE") == "1"
	var bsk := OS.get_environment("BATT_SAG_K"); if bsk != "": batt_sag_k = float(bsk)
	var ark := OS.get_environment("ADV_RATIO_K"); if ark != "": adv_ratio_k = float(ark)
	var vak := OS.get_environment("VIB_ACCEL"); if vak != "": vib_accel = float(vak)
	balance_onboard = OS.get_environment("BALANCE_ONBOARD") == "1"
	swingup_onboard = editor_demo or OS.get_environment("SWINGUP") == "1"
	if swingup_onboard:
		var siv := OS.get_environment("SWINGUP_INIT_DEG"); if siv != "": swingup_init_deg = float(siv)
		var sav := OS.get_environment("SWINGUP_ALT");      if sav != "": swingup_alt = float(sav)
		var shv := OS.get_environment("SWINGUP_HOLD_TIME"); if shv != "": swingup_hold_time = float(shv)
	lockstep = OS.get_environment("LOCKSTEP") == "1"
	if lockstep:
		print("[bridge] LOCKSTEP mode: tight 1:1 sync (lower latency, slower-than-real-time)")

	# NATIVE flight stack (env USE_NATIVE=1 or the export flag): run the ported mode-30 loop
	# in-process and DISABLE the UDP bridge, so there is no round-trip latency to hide.
	if editor_demo or OS.get_environment("USE_NATIVE") == "1" or swingup_onboard:
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

	response_video_overlay = response_video_overlay or OS.get_environment("RESPONSE_VIDEO") == "1"
	if response_video_overlay:
		print("[graphs] RESPONSE_VIDEO overlay: lateral + vertical command/realised charts")

	if balance_onboard:
		if n_arms >= 1:
			_balance_setup()
		else:
			push_warning("[bal] BALANCE_ONBOARD needs NUM_ARMS>=1; ignoring")
			balance_onboard = false

	if swingup_onboard:
		if n_arms >= 1:
			_swingup_setup()
		else:
			push_warning("[swingup] SWINGUP needs NUM_ARMS>=1; ignoring")
			swingup_onboard = false

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
			var command := PILOT_INPUT_SCRIPT.decode_acceleration_key(event.keycode)
			if command.x == PILOT_INPUT_SCRIPT.AXIS_LATERAL:
				lat_level = command.y
			elif command.x == PILOT_INPUT_SCRIPT.AXIS_VERTICAL:
				vert_level = command.y
			if command.x != PILOT_INPUT_SCRIPT.AXIS_NONE and park:
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
	_apply_arm_damping()
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
			var pwm := SITL_PROTOCOL_SCRIPT.decode_motor_pwm(pkt)
			if not pwm.is_empty():
				if drive_from_pwm:
					for i in range(8):
						motor_cmd[i] = _pwm_to_thrust(pwm[i])
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
		var pwm := SITL_PROTOCOL_SCRIPT.decode_motor_pwm(pkt)
		if not pwm.is_empty():
			if drive_from_pwm:
				for i in range(8):
					motor_cmd[i] = _pwm_to_thrust(pwm[i])
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

	# SENSOR MODEL (harness-side; ON by default): degrade the state mode-30 reads with
	# representative noise + measurement latency + EKF/INS lag, so the controller is validated
	# on realistic sensing rather than clean truth. The grader (_grade) stays on TRUTH.
	if sensor_noise_enabled:
		var truth := {
			"accel_ef": accel_ef, "gyro": gyro_frd, "quat": quat,
			"vel_ned": vel_ned, "pos_ned": pos_ned,
			"theta0": th0, "theta1": th1, "tdot0": td0, "tdot1": td1,
		}
		truth.merge(pendulum_chain.first_arm_imu(delta, GRAVITY, arm_imu_distance))
		var total_motor_thrust := MOTOR_DYNAMICS_SCRIPT.total_thrust(motor_thrust)
		var s := sensor_model.sample(truth, delta, total_motor_thrust)
		accel_ef = s["accel_ef"]; gyro_frd = s["gyro"]; quat = s["quat"]
		vel_ned = s["vel_ned"]; pos_ned = s["pos_ned"]
		th0 = s["theta0"]; th1 = s["theta1"]; td0 = s["tdot0"]; td1 = s["tdot1"]

	flightstack.set_state(accel_ef, vel_ned, pos_ned, true, true, quat, gyro_frd)

	# command: BALANCE_ONBOARD -> the balance controller drives lat_stick (closing the pendulum loop on the
	# REAL accel inner loop); else latched keyboard levels -> normalized sticks; Q/E -> yaw rate; SPACE -> park.
	# (auto_test drives lat_level/vert_level/park via _auto_drive, exactly as in the bridge path.)
	# All outer loops use the same navigation estimate as the native stack.
	# P maps world to NED; convert back without reading the rigid-body truth.
	var estimated_position := P.inverse() * pos_ned
	var estimated_velocity := P.inverse() * vel_ned

	var lat_stick := lat_level / 4.0
	var up_stick := vert_level / 3.0
	var yaw := 0.0
	var ff_arms := (n_arms if reaction_ff_on else 0)
	if swingup_onboard and not swingup_run_tracker.released:
		# SWINGUP: the portable C++ SwingUp controller drives BOTH sticks. It pumps energy (lateral +
		# vertical) from hanging, then gates into the ArmBalance LQR once inside the measured basin. th0/td0
		# are the SENSED arm state (from straight-DOWN); s,v,h,vy are the estimated drone cart state.
		var s_lat := estimated_position.x
		var v_lat := estimated_velocity.x
		var h_dev := estimated_position.y - swingup_alt
		var vy := estimated_velocity.y
		var o: Dictionary
		if n_arms >= 2:   # SwingUp2 also needs link2's sensed angle/rate
			o = swingup_ctrl.update(th0, td0, th1, td1, s_lat, v_lat, h_dev, vy, delta)
		else:
			o = swingup_ctrl.update(th0, td0, s_lat, v_lat, h_dev, vy, delta)
		lat_stick = clampf(float(o["a_lat"]) / 12.0, -1.0, 1.0)   # ACCEL_LAT_CMD_MSS=12
		up_stick = clampf(float(o["a_up"]) / GRAVITY, -1.0, 1.0)  # a_up in [-g,+g] -> up_stick in [-1,1]
		_su_lat_cmd = float(o["a_lat"]); _su_up_cmd = float(o["a_up"])
		_overlay_balance_on = bool(o["balancing"])
		_overlay_vehicle_command = "BALANCE" if _overlay_balance_on else "SWING UP"
		_overlay_reaction_ff_on = ff_arms > 0
		yaw = 0.0
		park = false
		_swingup_record(delta, bool(o["balancing"]))
	elif swingup_onboard:
		# hold elapsed -> RELEASE: stop driving the arms, let them fall; the drone just holds a hover.
		# Reaction FF stays on (n_arms passed below) so the dropping arms barely shove the frame.
		if not post_release_reaction_ff:
			ff_arms = 0
		_overlay_balance_on = false
		_overlay_vehicle_command = "HOLD POSITION"
		_overlay_reaction_ff_on = ff_arms > 0
		lat_stick = clampf((-1.5 * estimated_position.x - 3.0 * estimated_velocity.x) / 12.0, -1.0, 1.0)
		up_stick = clampf((2.0 * (swingup_alt - estimated_position.y) - 2.0 * estimated_velocity.y) / GRAVITY, -1.0, 1.0)
		park = false
		_su_post_t += delta
		_su_graph_n += 1
		if _su_graph_n % 4 == 0:
			_push_analysis_samples(
				swingup_run_tracker.elapsed + _su_post_t,
				lat_stick * 12.0,
				up_stick * GRAVITY
			)
		if (DisplayServer.get_name() == "headless" and _su_post_t >= 3.0) \
				or (_recording_auto_exit and _su_post_t >= 6.0):
			print("[swingup] post-release +%.1fs: alt=%.2f lat=%.2f (arms dropped, drone held hover)" % [
				_su_post_t, drone.global_position.y, drone.global_position.x])
			get_tree().quit(0 if swingup_run_tracker.passed else 2)
	elif balance_onboard and not _bal_released:
		# Feed the balance controller the SAME sensed arm state the reaction FF gets, re-expressed in ITS
		# convention: th0/td0 are sensed from straight-DOWN, so theta_up = wrap(PI - theta_down) and the rate
		# FLIPS SIGN (theta_dot_up = -theta_dot_down). s,v = drone lateral (world-X) pos/vel from the navigation estimate.
		var s_lat := estimated_position.x
		var v_lat := estimated_velocity.x
		var a_cmd := 0.0
		if n_arms >= 2:
			a_cmd = balance_ctrl.update(wrapf(PI - th0, -PI, PI), -td0,
				wrapf(PI - th1, -PI, PI), -td1, s_lat, v_lat, delta)
		else:
			a_cmd = balance_ctrl.update(wrapf(PI - th0, -PI, PI), -td0, s_lat, v_lat, delta)
		if balance_ctrl.last_saturated():
			_bal_sat = true
		lat_stick = clampf(a_cmd / 12.0, -1.0, 1.0)   # ACCEL_LAT_CMD_MSS=12: full lateral stick = 12 m/s^2
		# vertical: gentle altitude-hold PD -> vertical accel command. up_stick=0 alone holds zero ACCEL,
		# which permits a constant-velocity sink (esp. with the arm weight uncarried when FF is off); a
		# position/velocity PD holds altitude and decouples "stay airborne" from the reaction FF. (estimated altitude/vertical speed.)
		var alt_err := balance_alt - estimated_position.y
		up_stick = clampf((2.0 * alt_err - 2.0 * estimated_velocity.y) / GRAVITY, -1.0, 1.0)
		_overlay_balance_on = true
		_overlay_vehicle_command = "BALANCE"
		_overlay_reaction_ff_on = ff_arms > 0
		park = false
		_bal_record(delta, a_cmd, up_stick * GRAVITY)
	elif balance_onboard:
		# BAL_RUN_TIME elapsed: balance OFF -> the links drop freely; the drone just holds a hover.
		# The reaction FF stays on (n_arms passed below), so the falling arms barely shove the frame.
		if not post_release_reaction_ff:
			ff_arms = 0
		_overlay_balance_on = false
		_overlay_vehicle_command = "HOLD POSITION"
		_overlay_reaction_ff_on = ff_arms > 0
		lat_stick = clampf((-1.5 * estimated_position.x - 3.0 * estimated_velocity.x) / 12.0, -1.0, 1.0)
		up_stick = clampf((2.0 * (balance_alt - estimated_position.y) - 2.0 * estimated_velocity.y) / GRAVITY, -1.0, 1.0)
		park = false
		_bal_post_t += delta
		_push_analysis_samples(
			_bal_t + _bal_post_t,
			lat_stick * 12.0,
			up_stick * GRAVITY
		)
		if (DisplayServer.get_name() == "headless" and _bal_post_t >= 3.0) \
				or (_recording_auto_exit and _bal_post_t >= 6.0):
			print("[bal] post-release hover +%.1fs: alt=%.2f lat=%.2f fwd=%.2f (arms dropped, drone held)" % [
				_bal_post_t, drone.global_position.y, drone.global_position.x, drone.global_position.z])
			get_tree().quit(0)
	else:
		yaw = float(PILOT_INPUT_SCRIPT.yaw_direction())
	# reaction_ff_on=false -> tell the controller 0 arms so its reaction FF is skipped, while the physical
	# arms (built from n_arms) keep swinging -> isolates the FF's effect in the truth-based grader.
	flightstack.set_command(lat_stick, up_stick, yaw * NATIVE_YAW_RATE, park,
		ff_arms,
		Vector3(th0, th1, 0.0), Vector3(td0, td1, 0.0))

	# run the ported mode-30 loop -> 8 motor PWM, then slew the plant exactly like the bridge
	var pwm: PackedInt32Array = flightstack.update(grounded, delta)
	for i in range(8):
		motor_cmd[i] = _pwm_to_thrust(pwm[i])
	_apply_motor_lag(delta)


# ---- sensor model helpers (see the export block up top). Called only on the native path. ----
func _sensor_setup() -> void:
	if OS.get_environment("USE_SENSOR_NOISE") != "":
		sensor_noise_enabled = OS.get_environment("USE_SENSOR_NOISE") != "0"
	var v := OS.get_environment("SENSOR_SEED");         if v != "": sensor_seed = int(v)
	v = OS.get_environment("SENSOR_LAT_EKF_ATT_MS");    if v != "": lat_ekf_att_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_EKF_VEL_MS");    if v != "": lat_ekf_vel_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_EKF_POS_MS");    if v != "": lat_ekf_pos_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_ARM_IMU_MS");    if v != "": lat_arm_ang_imu_ms = float(v)
	v = OS.get_environment("SENSOR_LAT_ARM_RATE_MS");   if v != "": lat_arm_ms = float(v)
	sensor_model.configure({
		"seed": sensor_seed,
		"sig_arm_accel": sig_arm_accel,
		"sig_arm_gyro_dps": sig_arm_gyro_dps,
		"arm_observer_tau": arm_observer_tau,
		"encoder_rate_tau": encoder_rate_tau,
		"sig_gyro_dps": sig_gyro_dps,
		"sig_accel": sig_accel,
		"vib_accel": vib_accel,
		"vib_f_hover": vib_f_hover,
		"sig_att_deg": sig_att_deg,
		"sig_relative_angle_deg": sig_theta1_deg,
		"encoder_lsb_deg": enc_lsb_deg,
		"lat_imu_ms": lat_imu_ms,
		"lat_attitude_ms": lat_ekf_att_ms,
		"lat_velocity_ms": lat_ekf_vel_ms,
		"lat_position_ms": lat_ekf_pos_ms,
		"lat_arm_ms": lat_arm_ms,
		"lat_arm_imu_ms": lat_arm_ang_imu_ms,
		"mass": MASS,
		"gravity": GRAVITY,
		"arm_count": 2,
	})
	if sensor_noise_enabled:
		var delays := sensor_model.latency_summary()
		print("[sensor] model ON seed=%d lat(imu/att/vel/pos/arm/arm_imu ticks)=%d/%d/%d/%d/%d/%d navigation=IMU-dead-reckoning" % [
			sensor_seed, delays[0], delays[1], delays[2], delays[3], delays[4], delays[5]])


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


# First-order motor+prop lag: thrust cannot change instantly. Slew the actual thrust toward the
# commanded thrust — ASYMMETRICALLY (rising: fast TAU_UP; falling: slow TAU_DOWN, so a rapid catch
# reversal is paced by spin-DOWN) — and toward a BATTERY-SAGGED ceiling: the shared pack voltage droops
# with total thrust, so the achievable thrust for a given PWM is motor_cmd * _sag_factor. The sag reads
# last tick's realized total (explicit), which avoids a current<->voltage algebraic loop within a tick.
func _apply_motor_lag(dt: float) -> void:
	_sag_factor = MOTOR_DYNAMICS_SCRIPT.apply_lag_with_battery_sag(
		motor_thrust,
		motor_cmd,
		dt,
		MOTOR_TAU_UP,
		MOTOR_TAU_DOWN,
		batt_sag_k,
		BATT_SAG_FLOOR
	)


func _pwm_to_thrust(pwm: int) -> float:
	# F60PROV-LV 1950KV prop curve: thrust = (1-e)x + e x^2 (e = THRUST_EXPO).
	# ArduPilot pre-distorts its PWM via MOT_THST_EXPO=e so realized thrust stays
	# linear in the controller's command — set both to the same e or they fight.
	return MOTOR_DYNAMICS_SCRIPT.pwm_to_thrust(pwm, T_MAX, THRUST_EXPO)


func _apply_forces() -> void:
	VEHICLE_PLANT_SCRIPT.apply_motor_forces(
		drone, motor_pos, motor_dir, motor_spin, motor_thrust,
		K_M, adv_ratio_k, BODY_DRAG_K, GYRO_KH
	)


# A2 + A3: realistic pendulum dissipation, so the arms DECAY like a real rig instead of swinging forever.
# A3 = quadratic air drag on each bob; A2 = viscous + (smoothed) Coulomb friction at each hinge, opposing the
# RELATIVE swing rate about the hinge axis. The controllers do NOT model this (a real hinge's friction is
# unmodelled too), so it also exercises disturbance rejection. Hinge axis is body-Z of the parent link.
func _apply_arm_damping() -> void:
	pendulum_chain.apply_damping(
		drone, ARM_DRAG_K, HINGE_VISC, HINGE_COUL, HINGE_OMEGA_EPS
	)


func _ground_clamp() -> void:
	on_ground = VEHICLE_PLANT_SCRIPT.clamp_to_ground(drone)


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

	var yaw := PILOT_INPUT_SCRIPT.yaw_direction()
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

	udp.set_dest_address(client_ip, client_port)
	udp.put_packet(SITL_PROTOCOL_SCRIPT.encode_state(
		sim_time,
		not lockstep,
		gyro_frd,
		accel_frd,
		pos_ned,
		vel_ned,
		q,
		rc,
		pend_rc
	))


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
var acceleration_grader = ACCELERATION_GRADER_SCRIPT.new()
func _grade(delta: float) -> void:
	acceleration_grader.update(
		drone, delta, on_ground, sim_time, lat_level, vert_level,
		GRADE_LAT_CMD_MSS, GRAVITY, true
	)


# Headless auto-test driver (AUTOTEST=1): sets the same latched levels the keyboard would, on a
# schedule. Phase 0 parks to auto-take-off (needs ArduPilot armed + in ACCEL, done by the python
# controller); once airborne it runs auto_profile; then holds. Grader keeps logging throughout.
func _auto_drive() -> void:
	var command := auto_profile_driver.update(
		drone.global_position.y,
		drone.linear_velocity.y,
		sim_time,
		auto_profile,
		"[auto]",
		true
	)
	park = command["park"]
	lat_level = command["lateral_level"]
	vert_level = command["vertical_level"]


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
	pendulum_chain.configure(bodies, pivots, n_arms)
	arm_bodies = pendulum_chain.bodies
	# A1: give each active arm a realistic distributed (rod + tip) mass instead of a point mass at L.
	var arm_len: Array[float] = [0.7, 0.6]
	var rod_frac := 0.7
	var rf := OS.get_environment("ARM_ROD_FRAC")
	if rf != "": rod_frac = float(rf)
	pendulum_chain.configure_distributed_mass(arm_len, 0.1, rod_frac)
	print("[arms] %d pendulum arm(s) attached; distributed rod_frac=%.2f (0=point@L, 1=uniform rod)" % [n_arms, rod_frac])


# ============ Packet D: onboard balance mode ============
# Build the balance controller, teleport the drone to a hover altitude, and erect the arm(s) INVERTED with a
# small lean -- the same IC as balance2_test but on the REAL dynamical drone (ModeAccel hover-holds vertically
# while the balance loop works the lateral axis).
func _balance_setup() -> void:
	var bv := OS.get_environment("BALANCE_INIT_DEG");  if bv != "": balance_init_deg = float(bv)
	bv = OS.get_environment("BALANCE_INIT2_DEG"); if bv != "": balance_init2_deg = float(bv)
	bv = OS.get_environment("BALANCE_ALT");       if bv != "": balance_alt = float(bv)
	bv = OS.get_environment("BALANCE_BUMP_DV");   if bv != "": _bal_bump_dv = float(bv)

	# controller: portable C++ core. ArmBalance2 = MIMO-LQR for the 0.7/0.6 double; ArmBalance = single-link LQR.
	if n_arms >= 2:
		var cfg := {"L1": 0.7, "L2": 0.6, "g": GRAVITY, "a_max": 12.0, "s_ref": 0.0}
		for k in BAL_GAINS2:
			cfg[k] = BAL_GAINS2[k]
		# Lower loop gain avoids the noisy delayed plant's alternating rail hits.
		# Set BALANCE_GAIN_SCALE=1 to reproduce the original tuning.
		var gain_scale := 0.5
		var gain_env := OS.get_environment("BALANCE_GAIN_SCALE")
		if gain_env != "":
			gain_scale = float(gain_env)
		for k in BAL_GAINS2:
			cfg[k] *= gain_scale
		balance_ctrl = ArmBalance2.new()
		balance_ctrl.configure(cfg)
	else:
		balance_ctrl = ArmBalance.new()
		balance_ctrl.configure({"L": 0.7, "g": GRAVITY, "a_max": 12.0, "s_ref": 0.0})

	# teleport the drone to a hover altitude, at rest and level; re-anchor the root hinge there
	drone.global_position = Vector3(0.0, balance_alt, 0.0)
	drone.linear_velocity = Vector3.ZERO
	drone.angular_velocity = Vector3.ZERO
	var dxf := drone.global_transform
	dxf.basis = Basis()
	drone.global_transform = dxf
	$Pivot.global_transform = Transform3D(Basis.IDENTITY, drone.global_position)

	# link1 inverted from the drone (bob UP) + small lean; theta measured from straight up, + toward +X
	var th1_0 := deg_to_rad(balance_init_deg)
	arm_bodies[0].global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - th1_0), drone.global_position)
	arm_bodies[0].linear_velocity = Vector3.ZERO
	arm_bodies[0].angular_velocity = Vector3.ZERO

	# link2 (double only): inverted from link1's TIP (0.7 m along the inverted link1)
	if n_arms >= 2:
		var tip: Vector3 = arm_bodies[0].global_transform * Vector3(0.0, -0.7, 0.0)
		var th2_0 := deg_to_rad(balance_init2_deg)
		arm_bodies[1].global_transform = Transform3D(Basis(Vector3(0, 0, 1), PI - th2_0), tip)
		arm_bodies[1].linear_velocity = Vector3.ZERO
		arm_bodies[1].angular_velocity = Vector3.ZERO
		$Pivot2.global_transform = Transform3D(Basis.IDENTITY, tip)

	print("[bal] BALANCE_ONBOARD active: %d link(s), alt=%.1f m, init=(%.1f, %.1f) deg, bump_dv=%.2f" % [
		n_arms, balance_alt, balance_init_deg, balance_init2_deg, _bal_bump_dv])

	# Keep the Camera3D transform/FOV authored in main.tscn so the recording
	# perspective can be adjusted directly in the editor.
	$Camera3D.current = true

	# Normal recordings use one shallow lateral chart above the vehicle. RESPONSE_VIDEO=1
	# splits that band into lateral and vertical command/realised charts.
	var graphs := SCENARIO_VISUALS_SCRIPT.create_recording_graphs(
		$CanvasLayer,
		graph_window_seconds,
		response_video_overlay
	)
	bal_graph_u = graphs[0]
	bal_graph_vertical = graphs[1] if graphs.size() > 1 else null
	bal_angle_readout = SCENARIO_VISUALS_SCRIPT.create_angle_readout($CanvasLayer)
	SCENARIO_VISUALS_SCRIPT.colour_links(arm_bodies)


# ---- SWINGUP: portable C++ SwingUp on the REAL ModeAccel. Arm(s) start HANGING; the controller pumps up
# (lateral + vertical) then gates into ArmBalance. Grades on TRUTH link angle, auto-exits headless.
func _swingup_setup() -> void:
	swingup_run_tracker.configure(swingup_hold_time, SWINGUP_RUN_TIME)
	# tunable from env (the real stack loses energy the idealized cart didn't, so k=3 not the idealized k=1).
	# The double's basin is tiny, so its gate defaults are tighter.
	# The added realism (hinge friction + bob drag + distributed mass) drains swing energy, so the single arm
	# needs a stronger energy pump than the idealized k=3 to still reach the top; the double kept margin at 3.
	var g_k := 3.0 if n_arms >= 2 else 6.0
	var g_ca := 8.0 if n_arms >= 2 else 20.0
	var g_cr := 40.0 if n_arms >= 2 else 100.0
	var g_ks := 1.0
	var g_kv := 2.0
	var ev := OS.get_environment("SWINGUP_GAIN");       if ev != "": g_k = float(ev)
	ev = OS.get_environment("SWINGUP_CATCH_ANG");        if ev != "": g_ca = float(ev)
	ev = OS.get_environment("SWINGUP_CATCH_RATE");       if ev != "": g_cr = float(ev)
	ev = OS.get_environment("SWINGUP_KS");               if ev != "": g_ks = float(ev)
	ev = OS.get_environment("SWINGUP_KV");               if ev != "": g_kv = float(ev)
	if n_arms >= 2:
		swingup_ctrl = SwingUp2.new()
		swingup_ctrl.configure({
			"L1": 0.7, "L2": 0.6, "g": GRAVITY, "m1": 0.1, "m2": 0.1, "pump_gain": g_k,
			"a_max_lat": 12.0, "a_max_up": GRAVITY,
			"k_s": g_ks, "k_v": g_kv, "k_h": 2.0, "k_hv": 4.0,
			"catch_angle": deg_to_rad(g_ca), "catch_rate": deg_to_rad(g_cr)})
	else:
		swingup_ctrl = SwingUp.new()
		swingup_ctrl.configure({
			"L": 0.7, "g": GRAVITY, "pump_gain": g_k,
			"a_max_lat": 12.0, "a_max_up": GRAVITY,
			"k_s": g_ks, "k_v": g_kv, "k_h": 2.0, "k_hv": 4.0,
			"catch_angle": deg_to_rad(g_ca), "catch_rate": deg_to_rad(g_cr)})
	print("[swingup] cfg: arms=%d k=%.1f catch=(%.0fdeg,%.0fdps) centering=(%.1f,%.1f)" % [n_arms, g_k, g_ca, g_cr, g_ks, g_kv])

	# drone to a hover altitude, at rest and level; re-anchor the root hinge there
	drone.global_position = Vector3(0.0, swingup_alt, 0.0)
	drone.linear_velocity = Vector3.ZERO
	drone.angular_velocity = Vector3.ZERO
	var dxf := drone.global_transform
	dxf.basis = Basis()
	drone.global_transform = dxf
	$Pivot.global_transform = Transform3D(Basis.IDENTITY, drone.global_position)

	# link1 HANGING (bob DOWN) with a small +X lean so the pump has a symmetry to break
	var th0_0 := deg_to_rad(swingup_init_deg)
	arm_bodies[0].global_transform = Transform3D(Basis(Vector3(0, 0, 1), th0_0), drone.global_position)
	arm_bodies[0].linear_velocity = Vector3.ZERO
	arm_bodies[0].angular_velocity = Vector3.ZERO

	# link2 (double only): hang from link1's tip (0.7 m below)
	if n_arms >= 2:
		var tip: Vector3 = arm_bodies[0].global_transform * Vector3(0.0, -0.7, 0.0)
		arm_bodies[1].global_transform = Transform3D(Basis(Vector3(0, 0, 1), th0_0), tip)
		arm_bodies[1].linear_velocity = Vector3.ZERO
		arm_bodies[1].angular_velocity = Vector3.ZERO
		$Pivot2.global_transform = Transform3D(Basis.IDENTITY, tip)

	print("[swingup] SWINGUP active: %d link(s), alt=%.1f m, init=%.1f deg from hang" % [
		n_arms, swingup_alt, swingup_init_deg])

	# Keep the Camera3D transform/FOV authored in main.tscn.
	$Camera3D.current = true
	SCENARIO_VISUALS_SCRIPT.colour_links(arm_bodies)

	# Recording overlay: one lateral chart by default; RESPONSE_VIDEO=1 adds vertical tracking.
	# Link angles stay visible as values at the side instead of obscuring the vehicle with another plot.
	if DisplayServer.get_name() != "headless":
		var graphs := SCENARIO_VISUALS_SCRIPT.create_recording_graphs(
			$CanvasLayer,
			graph_window_seconds,
			response_video_overlay
		)
		bal_graph_u = graphs[0]
		bal_graph_vertical = graphs[1] if graphs.size() > 1 else null
		bal_angle_readout = SCENARIO_VISUALS_SCRIPT.create_angle_readout($CanvasLayer)


# TRUTH |angle from up| of a link [deg]; 0 = upright. (Same convention as the balance grader.)
func _link_up_deg(i: int) -> float:
	var rod: Vector3 = arm_bodies[i].global_transform.basis * Vector3(0, -1, 0)
	return absf(wrapf(atan2(rod.x, rod.y), -PI, PI)) * 180.0 / PI


func _link_signed_up_deg(i: int) -> float:
	var rod: Vector3 = arm_bodies[i].global_transform.basis * Vector3(0, -1, 0)
	return rad_to_deg(wrapf(atan2(rod.x, rod.y), -PI, PI))


func _push_analysis_samples(
	sample_time: float,
	lateral_command: float,
	vertical_command: float
) -> void:
	if not bal_graph_u:
		return
	var lateral_realised := _imu_accel_world.x
	var angles: Array[float] = [_link_signed_up_deg(0)]
	if n_arms >= 2:
		angles.append(_link_signed_up_deg(1))
	bal_graph_u.push_sample(sample_time, lateral_command, lateral_realised)
	SCENARIO_VISUALS_SCRIPT.update_angle_readout(
		bal_angle_readout,
		angles,
		_overlay_balance_on,
		_overlay_vehicle_command,
		_overlay_reaction_ff_on
	)
	if bal_graph_vertical:
		# _imu_accel_world is specific force (a - gravity); subtract g to recover
		# the net upward kinematic acceleration that a_vert commands around hover.
		var vertical_realised := _imu_accel_world.y - GRAVITY
		bal_graph_vertical.push_sample(
			sample_time,
			vertical_command,
			vertical_realised
		)


func _swingup_record(delta: float, balancing: bool) -> void:
	var up_deg := _link_up_deg(0)
	if n_arms >= 2:
		up_deg = maxf(up_deg, _link_up_deg(1))   # BOTH links must be up (double)
	var run_time: float = swingup_run_tracker.elapsed + delta

	# feed the live strip-charts, DECIMATED to ~100 Hz (grading above runs every tick; only the graph
	# push is throttled): a_lat cmd vs the REAL realised lateral accel + TRUTH signed link angles from up.
	_su_graph_n += 1
	if bal_graph_u and _su_graph_n % 4 == 0:
		_push_analysis_samples(run_time, _su_lat_cmd, _su_up_cmd)
	if run_time - _su_dbg >= 1.0:
		_su_dbg = run_time
		print("  [su] t=%4.1f  link_up=%5.1f  th_down=%6.1f  alt=%5.2f  x=%+5.2f vx=%+5.2f  a_lat=%+5.1f a_up=%+5.1f  %s" % [
			run_time, up_deg, rad_to_deg(arm_theta[0]), drone.global_position.y,
			drone.global_position.x, drone.linear_velocity.x, _su_lat_cmd, _su_up_cmd,
			("BAL" if balancing else "pump")])
	var was_released: bool = swingup_run_tracker.released
	swingup_run_tracker.update(delta, balancing, up_deg)
	if not was_released and swingup_run_tracker.released and bal_graph_u:
		bal_graph_u.add_event(run_time, "arm release")
		if bal_graph_vertical:
			bal_graph_vertical.add_event(run_time, "arm release")


# Grade the balance loop on TRUTH arm angle (from up), independent of the sensed signal the controller closed on.
# Track max/final angle, control effort, saturation; apply an optional mid-run cart-velocity bump; finish at 15 s.
func _bal_record(delta: float, a_cmd: float, vertical_command: float) -> void:
	var rod1: Vector3 = arm_bodies[0].global_transform.basis * Vector3(0, -1, 0)
	var th1_up := atan2(rod1.x, rod1.y)
	_bal_max1 = maxf(_bal_max1, absf(th1_up) * 180.0 / PI)
	var th2_up := 0.0
	if n_arms >= 2:
		var rod2: Vector3 = arm_bodies[1].global_transform.basis * Vector3(0, -1, 0)
		th2_up = atan2(rod2.x, rod2.y)
		_bal_max2 = maxf(_bal_max2, absf(th2_up) * 180.0 / PI)
	if _bal_t > 5.0 and delta > 0.0:
		_bal_sum_jerk2 += pow((a_cmd - _bal_previous_cmd) / delta, 2)
		_bal_jerk_n += 1
	_bal_previous_cmd = a_cmd
	_bal_sum_cmd2 += a_cmd * a_cmd
	_bal_n += 1
	_push_analysis_samples(_bal_t, a_cmd, vertical_command)

	# mid-run lateral velocity bump (perturbs the pivot), like balance_test's cart bump
	if not _bal_bumped and _bal_t >= _bal_bump_t and _bal_bump_dv != 0.0:
		drone.linear_velocity.x += _bal_bump_dv
		_bal_bumped = true

	if _bal_n % 100 == 0:
		print("[bal] t=%5.2f th1=%+6.2f th2=%+6.2f  lat=%+5.2f fwd=%+6.2f  alt=%5.2f a_cmd=%+6.2f" % [
			_bal_t, th1_up * 180.0 / PI, th2_up * 180.0 / PI, drone.global_position.x,
			drone.global_position.z, drone.global_position.y, a_cmd])

	_bal_t += delta
	if _bal_t >= BAL_RUN_TIME and not _bal_done:
		_bal_finish(th1_up, th2_up)


func _bal_finish(th1_up: float, th2_up: float) -> void:
	_bal_done = true
	var f1 := absf(th1_up) * 180.0 / PI
	var f2 := absf(th2_up) * 180.0 / PI
	var rms := sqrt(_bal_sum_cmd2 / float(max(1, _bal_n)))
	var passed := _bal_max1 < 15.0 and f1 < 5.0
	if n_arms >= 2:
		passed = passed and _bal_max2 < 15.0 and f2 < 5.0
	print("RESULT_BAL arms=%d noise=%d ff=%d | link1 max=%.2f final=%.3f | link2 max=%.2f final=%.3f | rms_u=%.2f sat=%d alt=%.2f => %s" % [
		n_arms, int(sensor_noise_enabled), int(reaction_ff_on), _bal_max1, f1, _bal_max2, f2, rms,
		int(_bal_sat), drone.global_position.y, ("PASS" if passed else "FAIL")])
	print("BALANCE_SMOOTHNESS jerk_rms=%.3f m/s^3 (after 5s)" % sqrt(_bal_sum_jerk2 / maxf(1.0, float(_bal_jerk_n))))
	# headless: a failed balance is done; a passing one goes on to prove the release-and-hover below.
	if DisplayServer.get_name() == "headless" and not passed:
		get_tree().quit(2)
		return
	_bal_released = true    # balance OFF -> arms drop freely; _service_native now just holds a hover
	if bal_graph_u:
		bal_graph_u.add_event(_bal_t, "arm release")
		if bal_graph_vertical:
			bal_graph_vertical.add_event(_bal_t, "arm release")
	print("[bal] t=%.0fs: balance released -> arms dropping, holding hover" % BAL_RUN_TIME)


# Colour a link so the two read clearly at a glance: tint its rod + add a visible bob sphere.
# Sample each active arm's angle (from straight-down, + toward +lateral/+X) and angular rate, and pack
# them + the arm count into pend_rc (rc_6..rc_10) for _send_state. The controller uses these to cancel
# the reaction force. theta comes from the arm's orientation (rod dir = basis*(0,-1,0)); yaw is held ~0
# so world X = heading lateral, world Y = up. theta_dot is a clean finite-difference (sim theta is smooth).
func _update_pendulum(delta: float) -> void:
	pend_rc[0] = 1000 + n_arms * 100
	pend_rc[1] = 1500; pend_rc[2] = 1500; pend_rc[3] = 1500; pend_rc[4] = 1500
	var measurement := pendulum_chain.sample(delta, arm_theta.size())
	arm_theta = measurement["angles"]
	arm_theta_dot = measurement["rates"]
	for i in range(n_arms):
		var theta: float = arm_theta[i]
		var dtheta: float = arm_theta_dot[i]
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
	var L1 := 0.7
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
