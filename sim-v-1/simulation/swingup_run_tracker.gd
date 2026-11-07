class_name SwingupRunTracker
extends RefCounted

var elapsed := 0.0
var caught := false
var catch_time := 0.0
var maximum_after_catch := 0.0
var maximum_during_hold := 0.0
var released := false
var passed := false

var _hold_time := 0.0
var _run_time_limit := 0.0


func configure(hold_time: float, run_time_limit: float) -> void:
	_hold_time = hold_time
	_run_time_limit = run_time_limit


func update(delta: float, balancing: bool, worst_link_degrees: float) -> void:
	elapsed += delta
	if balancing and not caught:
		caught = true
		catch_time = elapsed
		print("[swingup] CAUGHT at t=%.2f s (worst link %.1f deg from up)" % [
			elapsed, worst_link_degrees
		])

	var release_time := catch_time + _hold_time if caught else _run_time_limit
	if caught:
		maximum_after_catch = maxf(maximum_after_catch, worst_link_degrees)
		if elapsed >= maxf(catch_time + 0.5, release_time - 4.0):
			maximum_during_hold = maxf(maximum_during_hold, worst_link_degrees)

	if elapsed >= release_time and not released:
		passed = caught and maximum_during_hold < 15.0
		print("[swingup] RESULT caught=%d t_catch=%.2f hold=%.1fs max_after=%.1fdeg hold_max=%.1fdeg => %s" % [
			int(caught),
			catch_time,
			_hold_time,
			maximum_after_catch,
			maximum_during_hold,
			("PASS" if passed else "FAIL"),
		])
		print("[swingup] releasing -> arms drop, drone holds hover")
		released = true
