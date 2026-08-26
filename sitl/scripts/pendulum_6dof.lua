-- 6DoF mixer for the inverted-pendulum octacopter (switched layout)
--
-- 4 upright motors  -> throttle (lift) + roll + pitch + yaw
-- 4 canted motors   -> lateral (catch) + yaw,  paired to straddle the pivot axis
--
-- Axes (ArduPilot FRD):
--   CATCH  = lateral (Y)        <- canted motors push here (translate on thrust)
--   PIVOT  = forward/roll (X)   <- high-inertia axis, pendulum swings about it
--   FORWARD (X translation) has no thruster -> translate by TILTING (normal quad);
--     see set_forward_enable(false) below.
--
-- Factors per axis are normalised to max magnitude 1 (ArduPilot scales internally).
-- add_motor(num, roll, pitch, yaw, throttle, forward, lat, reversible, test_order)

Motors_6DoF:add_motor(0,  1.000, -1.000, -0.076, 1.000, 0.0,  0.000, false, 1)  -- M1 upright
Motors_6DoF:add_motor(1, -1.000, -1.000,  0.076, 1.000, 0.0,  0.000, false, 2)  -- M2 upright
Motors_6DoF:add_motor(2, -1.000,  1.000, -0.076, 1.000, 0.0,  0.000, false, 3)  -- M3 upright
Motors_6DoF:add_motor(3,  1.000,  1.000,  0.076, 1.000, 0.0,  0.000, false, 4)  -- M4 upright
Motors_6DoF:add_motor(4,  0.000, -0.042,  1.000, 0.000, 0.0, -1.000, false, 5)  -- M5 canted
Motors_6DoF:add_motor(5,  0.000, -0.042, -1.000, 0.000, 0.0,  1.000, false, 6)  -- M6 canted
Motors_6DoF:add_motor(6,  0.000,  0.042,  1.000, 0.000, 0.0,  1.000, false, 7)  -- M7 canted
Motors_6DoF:add_motor(7,  0.000,  0.042, -1.000, 0.000, 0.0, -1.000, false, 8)  -- M8 canted

assert(Motors_6DoF:init(8), 'pendulum 6DoF: unable to set up 8 motors')
motors:set_frame_string("PendulumDrone 6DoF")

-- Hybrid translation:
--   fore/aft (FORWARD/X) has no dedicated thruster -> translate by TILTING the
--     uprights, like a normal quad. forward_enable=false lets the position
--     controller's pitch demand pass through to the attitude controller instead
--     of being routed to a non-existent forward motor (the full-6DoF default,
--     which silently no-ops AND pins the vehicle level -> that axis becomes an
--     undamped runaway that LOITER cannot correct).
--   lateral (CATCH/Y) DOES have thrusters (the canted motors) -> stay level and
--     translate on thrust. lateral_enable=true.
function update()
    attitude_control:set_forward_enable(false)
    attitude_control:set_lateral_enable(true)
    return update, 100   -- reassert every 100 ms (flags default back to full 6DoF)
end
return update, 1000
