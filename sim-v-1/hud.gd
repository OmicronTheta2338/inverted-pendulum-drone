extends Control

# Drag the Drone (RigidBody3D) node into "Drone Path" in the Inspector.
@export var drone_path: NodePath
@onready var drone: RigidBody3D = get_node(drone_path)

const HISTORY := 1200
const PANEL_H := 300.0
const COLS := 3
const ROWS := 2
const MARGIN := 10.0
const HEADER := 42.0      # title + legend space inside each cell

var prev_vel := Vector3.ZERO
var accel_s := Vector3.ZERO
var font: Font
var P: Basis
var channels := []
var hist := {}
var text_scale := 1.0


func _ready() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	font = ThemeDB.fallback_font
	text_scale = 1.7 if OS.has_feature("movie") else 1.0
	P = Basis(Vector3(0, 1, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0))
	if drone:
		prev_vel = drone.linear_velocity
	channels = [
		{ "title": "Acceleration",  "unit": "m/s²",  "min": 5.0,  "labels": ["X", "Y up", "Z"] },
		{ "title": "Velocity",      "unit": "m/s",   "min": 2.0,  "labels": ["X", "Y up", "Z"] },
		{ "title": "Position",      "unit": "m",     "min": 2.0,  "labels": ["X", "Y up", "Z"] },
		{ "title": "Ang. velocity", "unit": "deg/s", "min": 30.0, "labels": ["X", "Y", "Z"] },
		{ "title": "Attitude",      "unit": "deg",   "min": 10.0, "labels": ["roll", "pitch", "yaw"] },
	]
	for c in channels:
		hist[c.title] = []


func _physics_process(delta: float) -> void:
	if drone == null or delta <= 0.0:
		return
	var vel := drone.linear_velocity
	accel_s = accel_s.lerp((vel - prev_vel) / delta, 0.25)
	prev_vel = vel

	var R := P * drone.global_transform.basis * P.inverse()   # FRD -> NED
	var att := R.get_euler(EULER_ORDER_ZYX) * (180.0 / PI)    # roll, pitch, yaw (deg)

	_push("Acceleration", accel_s)
	_push("Velocity", vel)
	_push("Position", drone.global_position)
	_push("Ang. velocity", drone.angular_velocity * (180.0 / PI))
	_push("Attitude", att)


# Sample data at the physics rate (above) but only repaint once per rendered
# frame. Repainting every physics tick was redrawing ~18k line segments at
# 400+ Hz, starving the renderer and dragging the whole lock-stepped sim into
# slow motion.
func _process(_delta: float) -> void:
	queue_redraw()


func _push(key: String, v: Vector3) -> void:
	var h: Array = hist[key]
	h.append(v)
	if h.size() > HISTORY:
		h.pop_front()


func _draw() -> void:
	var vp: Vector2 = get_viewport_rect().size
	if OS.has_feature("movie"):
		_draw_movie_column(vp)
	else:
		_draw_dashboard(vp)


func _draw_movie_column(vp: Vector2) -> void:
	# Preserve the cells' former 3x2 grid dimensions, but stack all five down
	# the left below the top tracking graph so the vehicle remains unobscured.
	var safe_x := vp.x * 0.025
	var safe_bottom := vp.y * 0.04
	var grid_panel_h := minf(PANEL_H * text_scale, vp.y * 0.29)
	var grid_panel_w := vp.x - 2.0 * safe_x
	var cell_w := (
		grid_panel_w - 2.0 * MARGIN - (COLS - 1) * MARGIN
	) / COLS
	var cell_h := (
		grid_panel_h - 2.0 * MARGIN - (ROWS - 1) * MARGIN
	) / ROWS
	var panel_w := cell_w + 2.0 * MARGIN
	var panel_h := (
		2.0 * MARGIN
		+ channels.size() * cell_h
		+ (channels.size() - 1) * MARGIN
	)
	var y0 := vp.y - safe_bottom - panel_h
	draw_rect(Rect2(safe_x, y0, panel_w, panel_h), Color(0, 0, 0, 0.6))

	for index in range(channels.size()):
		var cx := safe_x + MARGIN
		var cy := y0 + MARGIN + index * (cell_h + MARGIN)
		_draw_chart(Rect2(cx, cy, cell_w, cell_h), channels[index])


func _draw_dashboard(vp: Vector2) -> void:
	var safe_x := vp.x * 0.025
	var safe_bottom := vp.y * 0.04
	var panel_h := minf(PANEL_H * text_scale, vp.y * 0.29)
	var y0 := vp.y - safe_bottom - panel_h
	var panel_w := vp.x - 2.0 * safe_x
	draw_rect(Rect2(safe_x, y0, panel_w, panel_h), Color(0, 0, 0, 0.6))
	var cell_w := (panel_w - 2.0 * MARGIN - (COLS - 1) * MARGIN) / COLS
	var cell_h := (panel_h - 2.0 * MARGIN - (ROWS - 1) * MARGIN) / ROWS
	for i in range(channels.size()):
		var col := i % COLS
		var row := i / COLS
		var cx := safe_x + MARGIN + col * (cell_w + MARGIN)
		var cy := y0 + MARGIN + row * (cell_h + MARGIN)
		_draw_chart(Rect2(cx, cy, cell_w, cell_h), channels[i])


func _draw_chart(rect: Rect2, c: Dictionary) -> void:
	var colors := [Color(1.0, 0.45, 0.45), Color(0.45, 1.0, 0.55), Color(0.5, 0.7, 1.0)]
	var h: Array = hist[c.title]
	var labels: Array = c.labels

	# title
	draw_string(font, Vector2(rect.position.x, rect.position.y + 12 * text_scale),
		"%s (%s)" % [c.title, c.unit], HORIZONTAL_ALIGNMENT_LEFT, -1,
		roundi(16 * text_scale), Color(1, 1, 1, 0.9))

	# current values, color-coded, spread across the width
	var cur: Vector3 = h[-1] if h.size() > 0 else Vector3.ZERO
	for j in range(3):
		var val: float = [cur.x, cur.y, cur.z][j]
		draw_string(font, Vector2(
			rect.position.x + j * rect.size.x / 3.0,
			rect.position.y + 34 * text_scale
		), "%s %+.2f" % [labels[j], val], HORIZONTAL_ALIGNMENT_LEFT, -1,
			roundi(14 * text_scale), colors[j])

	# plot area
	var plot := Rect2(
		rect.position.x,
		rect.position.y + HEADER * text_scale,
		rect.size.x,
		rect.size.y - HEADER * text_scale
	)
	draw_rect(plot, Color(1, 1, 1, 0.08))

	var scl: float = c.min
	for s in h:
		scl = max(scl, max(abs(s.x), max(abs(s.y), abs(s.z))))

	var mid := plot.position.y + plot.size.y * 0.5
	draw_line(Vector2(plot.position.x, mid), Vector2(plot.position.x + plot.size.x, mid),
		Color(1, 1, 1, 0.25), text_scale)

	if h.size() >= 2:
		for comp in range(3):
			var pts := PackedVector2Array()
			for k in range(h.size()):
				var v: Vector3 = h[k]
				var val: float = [v.x, v.y, v.z][comp]
				var px := plot.position.x + plot.size.x * float(k) / float(HISTORY - 1)
				var py := clampf(mid - (val / scl) * (plot.size.y * 0.5),
					plot.position.y, plot.position.y + plot.size.y)
				pts.append(Vector2(px, py))
			draw_polyline(pts, colors[comp], 2.0 * text_scale)
