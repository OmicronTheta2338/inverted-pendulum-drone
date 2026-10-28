extends Control
#
# Live 3-series scrolling strip-chart (e.g. the three link angles of the triple).
# Same auto-fitting y-axis / scrolling window as accel_graph.gd, but THREE signals
# with per-link colours. Separate file so accel_graph.gd (2-series, used by the
# validated 2-arm view) stays untouched. Fed one sample/tick via push_sample().

var t_data: PackedFloat32Array = PackedFloat32Array()
var d1: PackedFloat32Array = PackedFloat32Array()
var d2: PackedFloat32Array = PackedFloat32Array()
var d3: PackedFloat32Array = PackedFloat32Array()

var window_s := 2.0
var y_floor := 0.3
var title := ""
var y_unit := ""
var label1 := "1"
var label2 := "2"
var label3 := "3"
var event_markers: Array[Dictionary] = []

var _t0 := 0.0
var _t1 := 1.0
var _ymin := -1.0
var _ymax := 1.0

const COL_BG   := Color(0.07, 0.07, 0.09, 0.72)
const COL_ZERO := Color(0.60, 0.60, 0.66, 0.95)
const COL_GRID := Color(0.30, 0.30, 0.35, 0.5)
const COL1 := Color(0.95, 0.35, 0.35)   # link1 = red
const COL2 := Color(0.40, 0.65, 1.00)   # link2 = blue
const COL3 := Color(0.40, 0.90, 0.50)   # link3 = green
const COL_EVENT := Color(1.00, 0.90, 0.30, 0.95)
const PLOT_LEFT := 62.0
const PLOT_TOP := 60.0
const PLOT_RIGHT := 6.0
const PLOT_BOTTOM := 26.0


func setup(p_window_s := 2.0) -> void:
	window_s = maxf(p_window_s, 0.05)


func add_event(t: float, event_label: String) -> void:
	event_markers.append({"time": t, "label": event_label})
	queue_redraw()


func push_sample(t: float, v1: float, v2: float, v3 := 0.0) -> void:
	t_data.append(t)
	d1.append(v1)
	d2.append(v2)
	d3.append(v3)
	if t_data.size() > 12000:
		t_data = t_data.slice(t_data.size() - 6000)
		d1 = d1.slice(d1.size() - 6000)
		d2 = d2.slice(d2.size() - 6000)
		d3 = d3.slice(d3.size() - 6000)
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
		PLOT_LEFT,
		PLOT_TOP,
		maxf(size.x - PLOT_LEFT - PLOT_RIGHT, 1.0),
		maxf(size.y - PLOT_TOP - PLOT_BOTTOM, 1.0)
	)


func _draw_series(data: PackedFloat32Array, i0: int, n: int, col: Color) -> void:
	if n - i0 < 2:
		return
	var pts := PackedVector2Array()
	pts.resize(n - i0)
	for i in range(i0, n):
		pts[i - i0] = Vector2(_x(t_data[i]), _y(data[i]))
	draw_polyline(pts, col, 2.25, true)


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
	var hdr := title if title != "" else "three-series response"
	draw_string(font, Vector2(8, 21), "%s%s — last %.2f s" % [hdr, unit_suffix, window_s],
		HORIZONTAL_ALIGNMENT_LEFT, -1, 16, COL_ZERO)
	draw_string(font, Vector2(size.x - 220, 19), label1,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 15, COL1)
	draw_string(font, Vector2(size.x - 220, 39), label2,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 15, COL2)
	draw_string(font, Vector2(size.x - 220, 59), label3,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 15, COL3)

	var n := t_data.size()
	if n < 2:
		return
	var i0 := _first_visible_index()
	var plot := _plot_rect()

	_t1 = t_data[n - 1]
	_t0 = _t1 - window_s
	if _t0 < t_data[0]:
		_t0 = t_data[0]
		_t1 = maxf(_t0 + window_s, t_data[n - 1])

	# y-range: auto-fit to the visible samples of all three series, padded, with a floor
	var vmin := INF
	var vmax := -INF
	for i in range(i0, n):
		vmin = minf(vmin, minf(d1[i], minf(d2[i], d3[i])))
		vmax = maxf(vmax, maxf(d1[i], maxf(d2[i], d3[i])))
	var mid := 0.5 * (vmin + vmax)
	var half := maxf(0.5 * (vmax - vmin), y_floor) * 1.15
	_ymin = mid - half
	_ymax = mid + half

	for k in range(5):
		var a := _ymax - (float(k) / 4.0) * (_ymax - _ymin)
		var y := _y(a)
		draw_line(Vector2(plot.position.x, y), Vector2(plot.end.x, y), COL_GRID, 1.0)
		draw_string(font, Vector2(4, y + 5), "%+.2f" % a, HORIZONTAL_ALIGNMENT_LEFT, 54, 13, COL_ZERO)
	if _ymin < 0.0 and _ymax > 0.0:
		var yz := _y(0.0)
		draw_line(Vector2(plot.position.x, yz), Vector2(plot.end.x, yz), COL_ZERO, 1.0)

	var step := _nice_step(window_s / 5.0)
	var tg := ceilf(_t0 / step) * step
	while tg < _t1:
		var x := _x(tg)
		draw_line(Vector2(x, plot.position.y), Vector2(x, plot.end.y), COL_GRID, 1.0)
		draw_string(font, Vector2(x + 3, plot.end.y + 18), "%.2f s" % tg,
			HORIZONTAL_ALIGNMENT_LEFT, -1, 13, COL_ZERO)
		tg += step

	_draw_series(d1, i0, n, COL1)
	_draw_series(d2, i0, n, COL2)
	_draw_series(d3, i0, n, COL3)

	for marker in event_markers:
		var event_time := float(marker["time"])
		if event_time < _t0 or event_time > _t1:
			continue
		var event_x := _x(event_time)
		draw_dashed_line(Vector2(event_x, plot.position.y), Vector2(event_x, plot.end.y),
			COL_EVENT, 2.0, 7.0)
		draw_string(font, Vector2(event_x + 5, plot.position.y + 16), str(marker["label"]),
			HORIZONTAL_ALIGNMENT_LEFT, -1, 14, COL_EVENT)
