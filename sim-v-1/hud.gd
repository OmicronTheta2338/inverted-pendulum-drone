extends Control

# Drag the Drone (RigidBody3D) node into "Drone Path" in the Inspector.
@export var drone_path: NodePath
@onready var drone: RigidBody3D = get_node(drone_path)

const HISTORY := 1200
const PANEL_H := 300.0
const COLS := 3
const ROWS := 2
const MARGIN := 10.0
const HEADER := 32.0      # title + legend space inside each cell

var prev_vel := Vector3.ZERO
var accel_s := Vector3.ZERO
var font: Font
var P: Basis
var channels := []
var hist := {}


func _ready() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	font = ThemeDB.fallback_font
	P = Basis(Vector3(0, 1, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0))
	if drone:
		prev_vel = drone.linear_velocity
	channels = [
		{ "title": "Acceleration",  "unit": "m/s2",  "min": 5.0,  "labels": ["X", "Y up", "Z"] },
		{ "title": "Velocity",      "unit": "m/s",   "min": 2.0,  "labels": ["X", "Y up", "Z"] },
		{ "title": "Position",      "unit": "m",     "min": 2.0,  "labels": ["X", "Y up", "Z"] },
		{ "title": "Ang. velocity", "unit": "rad/s", "min": 1.0,  "labels": ["X", "Y", "Z"] },
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
	_push("Ang. velocity", drone.angular_velocity)
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
	var y0 := vp.y - PANEL_H
	draw_rect(Rect2(0, y0, vp.x, PANEL_H), Color(0, 0, 0, 0.6))

	var cell_w := (vp.x - 2.0 * MARGIN - (COLS - 1) * MARGIN) / COLS
	var cell_h := (PANEL_H - 2.0 * MARGIN - (ROWS - 1) * MARGIN) / ROWS

	for i in range(channels.size()):
		var col := i % COLS
		var row := i / COLS
		var cx := MARGIN + col * (cell_w + MARGIN)
		var cy := y0 + MARGIN + row * (cell_h + MARGIN)
		_draw_chart(Rect2(cx, cy, cell_w, cell_h), channels[i])


func _draw_chart(rect: Rect2, c: Dictionary) -> void:
	var colors := [Color(1.0, 0.45, 0.45), Color(0.45, 1.0, 0.55), Color(0.5, 0.7, 1.0)]
	var h: Array = hist[c.title]
	var labels: Array = c.labels

	# title
	draw_string(font, Vector2(rect.position.x, rect.position.y + 12),
		"%s (%s)" % [c.title, c.unit], HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color(1, 1, 1, 0.9))

	# current values, color-coded, spread across the width
	var cur: Vector3 = h[-1] if h.size() > 0 else Vector3.ZERO
	for j in range(3):
		var val: float = [cur.x, cur.y, cur.z][j]
		draw_string(font, Vector2(rect.position.x + j * rect.size.x / 3.0, rect.position.y + 27),
			"%s %+.2f" % [labels[j], val], HORIZONTAL_ALIGNMENT_LEFT, -1, 11, colors[j])

	# plot area
	var plot := Rect2(rect.position.x, rect.position.y + HEADER, rect.size.x, rect.size.y - HEADER)
	draw_rect(plot, Color(1, 1, 1, 0.08))

	var scl: float = c.min
	for s in h:
		scl = max(scl, max(abs(s.x), max(abs(s.y), abs(s.z))))

	var mid := plot.position.y + plot.size.y * 0.5
	draw_line(Vector2(plot.position.x, mid), Vector2(plot.position.x + plot.size.x, mid),
		Color(1, 1, 1, 0.25), 1.0)

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
			draw_polyline(pts, colors[comp], 1.5)
