-- Rig controller — pin the drone to a world line and let it dart laterally fast.
--
-- This is the "cart" the inverted pendulum will sit on. It constrains the drone
-- to 1 lateral DOF (+ momentary vertical) so we can validate fast, level lateral
-- translation (Phase 5) and later hand the same interface to the C++ pendulum
-- controller (Phase 8) unchanged.
--
-- WHAT IT HOLDS (rigidly):
--   * North position  = the LINE. Drift off it is corrected by TILTING the
--     uprights (forward axis has no thruster; pendulum_6dof.lua sets
--     forward_enable=false so the tilt demand passes through).
--   * Yaw = North (0deg), so body-Y (the canted "catch" axis) == world East.
--   * Altitude = held, but the pilot can bump it momentarily (climb-rate stick).
--
-- WHAT IT FREES (fast):
--   * East = lateral velocity along the line, on the canted motors, STAYING LEVEL
--     (lateral_enable=true). This is the pendulum catch axis.
--
-- DRIVE (keyboard -> RC via the Godot JSON bridge, see main.gd _update_rc):
--   A / D  (RC1, roll)     -> lateral velocity along the line
--   R / F  (RC3, throttle) -> momentary climb / descend
--
-- HOW TO ENTER:  hover in ALT_HOLD, then switch to GUIDED. The script captures
--   the current North coord + altitude as the line and takes over. Leaving GUIDED
--   (or disarming) releases it and re-captures on the next entry.
--
-- Runs alongside pendulum_6dof.lua (which keeps the 6DoF forward/lateral routing
-- asserted). Both live in this scripts/ dir and load together.

local GUIDED        = 4       -- Copter GUIDED mode number (ArduCopter/mode.h)
local RUN_MS        = 20      -- 50 Hz when active (setpoint glue; inner loop is C++)
local IDLE_MS       = 200     -- 5 Hz when not flying the rig
local MAX_LAT_VEL   = 4.0     -- m/s, full lateral stick. Raise for more dart.
local MAX_VERT_VEL  = 1.5     -- m/s, full vertical stick (momentary)

local captured = false
local line_n   = 0.0          -- world North coord of the line (m, NED rel home)
local tgt_e    = 0.0          -- integrated lateral target (m East)
local tgt_d    = 0.0          -- integrated Down target (m, NED; alt = -tgt_d)

local function stick(ch_num)
  local ch = rc:get_channel(ch_num)
  if not ch then return 0.0 end
  return ch:norm_input_dz()   -- [-1,1], trim- and deadzone-corrected
end

function update()
  -- only run the rig when armed AND in GUIDED; otherwise idle and re-arm capture
  if not arming:is_armed() or vehicle:get_mode() ~= GUIDED then
    captured = false
    return update, IDLE_MS
  end

  local pos = ahrs:get_relative_position_NED_home()
  if not pos then return update, RUN_MS end

  -- capture the line on entry
  if not captured then
    line_n = pos:x()
    tgt_e  = pos:y()
    tgt_d  = pos:z()
    captured = true
    gcs:send_text(4, string.format("rig: locked line N=%.2f alt=%.2f", line_n, -tgt_d))
  end

  -- pilot sticks -> velocity commands on the free axes
  local v_e =  stick(1) * MAX_LAT_VEL    -- A/D  -> lateral along the line
  local v_d = -stick(3) * MAX_VERT_VEL   -- R/F  -> climb (up = -D)

  -- integrate a moving target on the free axes; North stays pinned to the line.
  -- Releasing a stick -> v=0 and the target freezes -> crisp stop & hold (no coast).
  tgt_e = tgt_e + v_e * (RUN_MS / 1000.0)
  tgt_d = tgt_d + v_d * (RUN_MS / 1000.0)

  local p = Vector3f(); p:x(line_n); p:y(tgt_e); p:z(tgt_d)
  local v = Vector3f(); v:x(0.0);    v:y(v_e);   v:z(v_d)
  local a = Vector3f()  -- zero accel FF for now; pendulum controller will use this

  -- pos + vel-FF, yaw locked to North (absolute), no yaw-rate
  vehicle:set_target_posvelaccel_NED(p, v, a, true, 0, false, 0, false)

  return update, RUN_MS
end

return update, 1000
