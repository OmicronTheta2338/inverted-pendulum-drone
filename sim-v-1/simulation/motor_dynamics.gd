class_name MotorDynamics
extends RefCounted


static func pwm_to_thrust(pwm: int, maximum_thrust: float, thrust_expo: float) -> float:
	var command := clampf((float(pwm) - 1000.0) / 1000.0, 0.0, 1.0)
	return maximum_thrust * (
		(1.0 - thrust_expo) * command + thrust_expo * command * command
	)


static func apply_first_order_lag(
	actual_thrust: Array[float],
	commanded_thrust: Array[float],
	delta: float,
	time_constant: float
) -> void:
	var alpha := 1.0 - exp(-delta / time_constant)
	for index in range(actual_thrust.size()):
		actual_thrust[index] += (commanded_thrust[index] - actual_thrust[index]) * alpha


# Apply asymmetric motor response against a ceiling derived from the previous
# tick's total realized thrust. Returning the ceiling factor keeps battery-sag
# diagnostics available to the caller without duplicating the calculation.
static func apply_lag_with_battery_sag(
	actual_thrust: Array[float],
	commanded_thrust: Array[float],
	delta: float,
	rise_time_constant: float,
	fall_time_constant: float,
	sag_per_newton: float,
	sag_floor: float
) -> float:
	var voltage_ratio := maxf(0.0, 1.0 - sag_per_newton * total_thrust(actual_thrust))
	var sag_factor := maxf(sag_floor, voltage_ratio * voltage_ratio)
	for index in range(actual_thrust.size()):
		var target := commanded_thrust[index] * sag_factor
		var time_constant := (
			rise_time_constant if target >= actual_thrust[index] else fall_time_constant
		)
		var alpha := 1.0 - exp(-delta / time_constant)
		actual_thrust[index] += (target - actual_thrust[index]) * alpha
	return sag_factor


static func total_thrust(motor_thrust: Array[float]) -> float:
	var total := 0.0
	for thrust in motor_thrust:
		total += thrust
	return total
