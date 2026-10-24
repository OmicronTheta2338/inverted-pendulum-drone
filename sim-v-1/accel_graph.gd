extends Control
#
# Live scrolling strip-chart of commanded vs realised lateral acceleration.
#
# Shows the most recent `window_s` seconds (high x-resolution) with the y-axis
# AUTO-FIT to whatever is on screen, so the small in-balance activity -- and the
# tau_cl lag between a_cmd (target) and a_real (realised) -- is actually visible
# instead of being crushed against a fixed +/- a_max scale.
#
# Fed one sample per physics tick via push_sample(); redraws once per frame.

var t_data: PackedFloat32Array = PackedFloat32Array()
var cmd_data: PackedFloat32Array = PackedFloat32Array()
var real_data: PackedFloat32Array = PackedFloat32Array()

var window_s := 2.0      # visible time span [s]
var y_floor := 0.3       # minimum y half-range so flat noise isn't over-zoomed [m/s^2]
var title := ""          # optional header label (e.g. which axis/arm this graph is)
var y_unit := ""         # explicit display unit; callers keep every series on one unit
var label1 := "a_cmd  (target)"      # legend for series 1 (cmd_data)
var label2 := "a_real (realised)"    # legend for series 2 (real_data)
var event_markers: Array[Dictionary] = []
var text_scale := 1.0

# view transform (recomputed each _draw)
var _t0 := 0.0
var _t1 := 1.0
var _ymin := -1.0
var _ymax := 1.0

const COL_BG   := Color(0.07, 0.07, 0.09, 0.72)
const COL_ZERO := Color(0.60, 0.60, 0.66, 0.95)
const COL_GRID := Color(0.30, 0.30, 0.35, 0.5)
const COL_CMD  := Color(0.35, 0.75, 1.00)   # target   = light blue
const COL_REAL := Color(1.00, 0.55, 0.20)   # realised = orange
const COL_EVENT := Color(1.00, 0.90, 0.30, 0.95)
const PLOT_LEFT := 62.0
const PLOT_TOP := 46.0
const PLOT_RIGHT := 6.0
const PLOT_BOTTOM := 26.0


func setup(p_window_s := 2.0) -> void:
	window_s = maxf(p_window_s, 0.05)
	# The 1440p movie is normally viewed much smaller in a README. Scale text
	# before rasterisation so it remains legible after GitHub shrinks the player.
	text_scale = 1.75 if OS.has_feature("movie") else 1.0


func add_event(t: float, event_label: String) -> void:
	event_markers.append({"time": t, "label": event_label})
	queue_redraw()


func push_sample(t: float, a_cmd: float, a_real: float) -> void:
	t_data.append(t)
	cmd_data.append(a_cmd)
	real_data.append(a_real)
	# Retain enough 400 Hz data for the six-second arm-drop panel, trimming in bulk.
	if t_data.size() > 12000:
		t_data = t_data.slice(t_data.size() - 6000)
		cmd_data = cmd_data.slice(cmd_data.size() - 6000)
		real_data = real_data.slice(real_data.size() - 6000)
	queue_redraw()


func _first_visible_index() -> int:
	var n := t_data.size()
	if n == 0:
		return 0
	var t_lo := t_data[n - 1] - window_s
	var i := n - 1
	while i > 0 and t_data[i - 1] >= t_lo:
		i -= 1
	return i


func _x(t: float) -> float:
	var plot := _plot_rect()
	return plot.position.x + (t - _t0) / (_t1 - _t0) * plot.size.x


func _y(a: float) -> float:
	var plot := _plot_rect()
	return plot.position.y + (1.0 - (a - _ymin) / (_ymax - _ymin)) * plot.size.y


func _plot_rect() -> Rect2:
	return Rect2(
		PLOT_LEFT * text_scale,
		PLOT_TOP * text_scale,
		maxf(size.x - (PLOT_LEFT + PLOT_RIGHT) * text_scale, 1.0),
		maxf(size.y - (PLOT_TOP + PLOT_BOTTOM) * text_scale, 1.0)
	)


func _draw_series(data: PackedFloat32Array, i0: int, n: int, col: Color) -> void:
	if n - i0 < 2:
		return
	var pts := PackedVector2Array()
	pts.resize(n - i0)
	for i in range(i0, n):
		pts[i - i0] = Vector2(_x(t_data[i]), _y(data[i]))
	draw_polyline(pts, col, 2.25 * text_scale, true)


func _nice_step(x: float) -> float:
	if x <= 0.0:
		return 1.0
	var p := pow(10.0, floorf(log(x) / log(10.0)))
	var f := x / p
	var nf := 1.0
	if f > 5.0:   nf = 10.0
	elif f > 2.0: nf = 5.0
	elif f > 1.0: nf = 2.0
	return nf * p


func _draw() -> void:
	var font := get_theme_default_font()
	draw_rect(Rect2(Vector2.ZERO, size), COL_BG, true)
	var unit_suffix := " [%s]" % y_unit if y_unit != "" else ""
	var hdr := title if title != "" else "acceleration response"
	draw_string(font, Vector2(8, 21) * text_scale,
		"%s%s — last %.2f s" % [hdr, unit_suffix, window_s],
		HORIZONTAL_ALIGNMENT_LEFT, -1, roundi(16 * text_scale), COL_ZERO)
	draw_string(font, Vector2(size.x - 220 * text_scale, 19 * text_scale), label1,
		HORIZONTAL_ALIGNMENT_LEFT, -1, roundi(15 * text_scale), COL_CMD)
	draw_string(font, Vector2(size.x - 220 * text_scale, 39 * text_scale), label2,
		HORIZONTAL_ALIGNMENT_LEFT, -1, roundi(15 * text_scale), COL_REAL)

	var n := t_data.size()
	if n < 2:
		return
	var i0 := _first_visible_index()
	var plot := _plot_rect()

	# x-range: the visible window, anchored to the right once the run outgrows it
	_t1 = t_data[n - 1]
	_t0 = _t1 - window_s
	if _t0 < t_data[0]:
		_t0 = t_data[0]
		_t1 = maxf(_t0 + window_s, t_data[n - 1])

	# y-range: auto-fit to the visible samples, padded, with a floor
	var vmin := INF
	var vmax := -INF
	for i in range(i0, n):
		vmin = minf(vmin, minf(cmd_data[i], real_data[i]))
		vmax = maxf(vmax, maxf(cmd_data[i], real_data[i]))
	var mid := 0.5 * (vmin + vmax)
	var half := maxf(0.5 * (vmax - vmin), y_floor) * 1.15
	_ymin = mid - half
	_ymax = mid + half

	# horizontal gridlines + value labels
	for k in range(5):
		var a := _ymax - (float(k) / 4.0) * (_ymax - _ymin)
		var y := _y(a)
		draw_line(Vector2(plot.position.x, y), Vector2(plot.end.x, y), COL_GRID, text_scale)
		draw_string(font, Vector2(4 * text_scale, y + 5 * text_scale), "%+.2f" % a,
			HORIZONTAL_ALIGNMENT_LEFT, 54 * text_scale, roundi(13 * text_scale), COL_ZERO)
	if _ymin < 0.0 and _ymax > 0.0:
		var yz := _y(0.0)
		draw_line(Vector2(plot.position.x, yz), Vector2(plot.end.x, yz), COL_ZERO, text_scale)

	# vertical time gridlines at a "nice" step
	var step := _nice_step(window_s / 5.0)
	var tg := ceilf(_t0 / step) * step
	while tg < _t1:
		var x := _x(tg)
		draw_line(Vector2(x, plot.position.y), Vector2(x, plot.end.y), COL_GRID, text_scale)
		draw_string(font, Vector2(x + 3 * text_scale, plot.end.y + 18 * text_scale),
			"%.2f s" % tg, HORIZONTAL_ALIGNMENT_LEFT, -1, roundi(13 * text_scale), COL_ZERO)
		tg += step

	_draw_series(cmd_data, i0, n, COL_CMD)
	_draw_series(real_data, i0, n, COL_REAL)

	for marker in event_markers:
		var event_time := float(marker["time"])
		if event_time < _t0 or event_time > _t1:
			continue
		var event_x := _x(event_time)
		draw_dashed_line(Vector2(event_x, plot.position.y), Vector2(event_x, plot.end.y),
			COL_EVENT, 2.0 * text_scale, 7.0 * text_scale)
		draw_string(font,
			Vector2(event_x + 5 * text_scale, plot.position.y + 16 * text_scale),
			str(marker["label"]), HORIZONTAL_ALIGNMENT_LEFT, -1,
			roundi(14 * text_scale), COL_EVENT)
