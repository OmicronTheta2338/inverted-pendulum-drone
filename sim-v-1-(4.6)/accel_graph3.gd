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
var label1 := "1"
var label2 := "2"
var label3 := "3"

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


func setup(p_window_s := 2.0) -> void:
	window_s = maxf(p_window_s, 0.05)


func push_sample(t: float, v1: float, v2: float, v3 := 0.0) -> void:
	t_data.append(t)
	d1.append(v1)
	d2.append(v2)
	d3.append(v3)
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
	return (t - _t0) / (_t1 - _t0) * size.x


func _y(a: float) -> float:
	return (1.0 - (a - _ymin) / (_ymax - _ymin)) * size.y


func _draw_series(data: PackedFloat32Array, i0: int, n: int, col: Color) -> void:
	if n - i0 < 2:
		return
	var pts := PackedVector2Array()
	pts.resize(n - i0)
	for i in range(i0, n):
		pts[i - i0] = Vector2(_x(t_data[i]), _y(data[i]))
	draw_polyline(pts, col, 1.5, true)


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

	var n := t_data.size()
	if n < 2:
		return
	var i0 := _first_visible_index()

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
		draw_line(Vector2(0, y), Vector2(size.x, y), COL_GRID, 1.0)
		draw_string(font, Vector2(3, y - 3), "%+.2f" % a, HORIZONTAL_ALIGNMENT_LEFT, -1, 10, COL_ZERO)
	if _ymin < 0.0 and _ymax > 0.0:
		var yz := _y(0.0)
		draw_line(Vector2(0, yz), Vector2(size.x, yz), COL_ZERO, 1.0)

	var step := _nice_step(window_s / 5.0)
	var tg := ceilf(_t0 / step) * step
	while tg < _t1:
		var x := _x(tg)
		draw_line(Vector2(x, 0), Vector2(x, size.y), COL_GRID, 1.0)
		draw_string(font, Vector2(x + 2, size.y - 3), "%.2fs" % tg, HORIZONTAL_ALIGNMENT_LEFT, -1, 10, COL_ZERO)
		tg += step

	_draw_series(d1, i0, n, COL1)
	_draw_series(d2, i0, n, COL2)
	_draw_series(d3, i0, n, COL3)

	draw_string(font, Vector2(size.x - 148, 14), label1, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, COL1)
	draw_string(font, Vector2(size.x - 148, 29), label2, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, COL2)
	draw_string(font, Vector2(size.x - 148, 44), label3, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, COL3)
	var hdr := title if title != "" else "3-series [unit] vs t [s]"
	draw_string(font, Vector2(4, 13), "%s  (last %.2f s)" % [hdr, window_s],
		HORIZONTAL_ALIGNMENT_LEFT, -1, 10, COL_ZERO)
