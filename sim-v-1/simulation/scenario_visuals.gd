class_name ScenarioVisuals
extends RefCounted

const TWO_SERIES_GRAPH := preload("res://accel_graph.gd")
const THREE_SERIES_GRAPH := preload("res://accel_graph3.gd")
# Keep both links visually separate from the blue commanded and orange realised
# graph traces without implying that either graph series belongs to one arm.
const PENDULUM_COLOUR := Color(0.30, 0.85, 0.35)


static func frame_camera(
	camera: Camera3D,
	position: Vector3,
	look_target: Vector3
) -> void:
	camera.current = true
	camera.global_position = position
	camera.look_at(look_target, Vector3.UP)


static func colour_links(links: Array[RigidBody3D]) -> void:
	for link in links:
		_colour_link(link, PENDULUM_COLOUR)


static func create_stacked_graphs(
	canvas: CanvasLayer,
	window_seconds: float,
	angle_series_count: int,
	acceleration_title: String,
	angle_title: String,
	acceleration_labels: Array[String],
	angle_labels: Array[String],
	response_title: String = "",
	response_labels: Array[String] = []
) -> Array:
	var acceleration_graph = TWO_SERIES_GRAPH.new()
	var angle_graph = (
		THREE_SERIES_GRAPH.new() if angle_series_count == 3 else TWO_SERIES_GRAPH.new()
	)
	var graphs := [acceleration_graph, angle_graph]
	if response_title != "":
		graphs.append(TWO_SERIES_GRAPH.new())
	for graph in graphs:
		canvas.add_child(graph)
		graph.setup(window_seconds)
		graph.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_layout_dashboard(graphs)
	acceleration_graph.title = acceleration_title
	acceleration_graph.y_unit = "m/s²"
	angle_graph.title = angle_title
	angle_graph.y_unit = "deg"
	angle_graph.y_floor = 0.5
	_apply_labels(acceleration_graph, acceleration_labels)
	_apply_labels(angle_graph, angle_labels)
	if graphs.size() == 3:
		var response_graph = graphs[2]
		response_graph.title = response_title
		response_graph.y_unit = "m/s²"
		response_graph.setup(maxf(window_seconds, 6.0))
		_apply_labels(response_graph, response_labels)
	return graphs


static func create_recording_graphs(
	canvas: CanvasLayer,
	window_seconds: float,
	include_vertical: bool
) -> Array:
	var lateral_graph = TWO_SERIES_GRAPH.new()
	_configure_recording_graph(
		canvas,
		lateral_graph,
		window_seconds,
		"lateral acceleration tracking",
		["a_lat commanded", "a_lat realised"]
	)
	var graphs := [lateral_graph]
	if include_vertical:
		var vertical_graph = TWO_SERIES_GRAPH.new()
		_configure_recording_graph(
			canvas,
			vertical_graph,
			maxf(window_seconds, 6.0),
			"vertical acceleration tracking",
			["a_vert commanded", "a_vert realised"]
		)
		graphs.append(vertical_graph)
		_layout_response_recording(graphs)
	else:
		_layout_default_recording(lateral_graph)
	return graphs


static func create_angle_readout(canvas: CanvasLayer) -> Label:
	var text_scale := 1.7 if OS.has_feature("movie") else 1.0
	var panel := PanelContainer.new()
	panel.anchor_left = 0.77 if text_scale > 1.0 else 0.80
	panel.anchor_right = 0.985 if text_scale > 1.0 else 0.975
	panel.anchor_top = 0.32
	panel.anchor_bottom = 0.32
	panel.offset_bottom = 235.0 * text_scale
	panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var background := StyleBoxFlat.new()
	background.bg_color = Color(0.04, 0.05, 0.07, 0.78)
	background.border_color = Color(0.55, 0.58, 0.65, 0.55)
	background.set_border_width_all(1)
	background.set_corner_radius_all(5)
	background.content_margin_left = 14.0 * text_scale
	background.content_margin_right = 14.0 * text_scale
	background.content_margin_top = 10.0 * text_scale
	background.content_margin_bottom = 10.0 * text_scale
	panel.add_theme_stylebox_override("panel", background)
	var label := Label.new()
	label.add_theme_font_size_override("font_size", roundi(19 * text_scale))
	label.add_theme_color_override("font_color", Color(0.92, 0.94, 1.0))
	label.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.9))
	label.add_theme_constant_override("shadow_offset_x", 1)
	label.add_theme_constant_override("shadow_offset_y", 1)
	panel.add_child(label)
	canvas.add_child(panel)
	return label


static func update_angle_readout(
	label: Label,
	angles_degrees: Array[float],
	pendulum_balance_on: bool,
	vehicle_command: String,
	reaction_feed_forward_on: bool
) -> void:
	if not label:
		return
	var lines := PackedStringArray(["ARM ANGLES [deg]"])
	for index in range(angles_degrees.size()):
		lines.append("link %d    %+7.2f" % [index + 1, angles_degrees[index]])
	lines.append("")
	lines.append("Pendulum balance: %s" % ("ON" if pendulum_balance_on else "OFF"))
	lines.append("Vehicle command: %s" % vehicle_command)
	lines.append("Reaction feed-forward: %s" % (
		"ON" if reaction_feed_forward_on else "OFF"
	))
	label.text = "\n".join(lines)


static func _configure_recording_graph(
	canvas: CanvasLayer,
	graph,
	window_seconds: float,
	graph_title: String,
	labels: Array[String]
) -> void:
	canvas.add_child(graph)
	graph.setup(window_seconds)
	graph.title = graph_title
	graph.y_unit = "m/s²"
	graph.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_apply_labels(graph, labels)


static func _layout_default_recording(graph) -> void:
	graph.anchor_left = 0.025
	graph.anchor_right = 0.975
	graph.anchor_top = 0.045
	graph.anchor_bottom = 0.235
	_clear_offsets(graph)


static func _layout_response_recording(graphs: Array) -> void:
	for index in range(graphs.size()):
		var graph = graphs[index]
		graph.anchor_left = 0.025 + float(index) * 0.4875
		graph.anchor_right = graph.anchor_left + 0.4625
		graph.anchor_top = 0.045
		graph.anchor_bottom = 0.255
		_clear_offsets(graph)


static func _clear_offsets(control: Control) -> void:
	control.offset_left = 0.0
	control.offset_top = 0.0
	control.offset_right = 0.0
	control.offset_bottom = 0.0


# Keep analysis charts away from capture edges and leave the vehicle unobscured on
# the right. Percentage anchors make the layout survive fullscreen, editor embed,
# and movie-writer resolutions without relying on fixed pixel bands.
static func _layout_dashboard(graphs: Array) -> void:
	const LEFT := 0.025
	const RIGHT := 0.62
	const TOP := 0.06
	const BOTTOM := 0.68
	const GAP := 0.012
	var graph_height := (BOTTOM - TOP - GAP * float(graphs.size() - 1)) / float(graphs.size())
	for index in range(graphs.size()):
		var graph = graphs[index]
		var graph_top := TOP + float(index) * (graph_height + GAP)
		graph.anchor_left = LEFT
		graph.anchor_right = RIGHT
		graph.anchor_top = graph_top
		graph.anchor_bottom = graph_top + graph_height
		graph.offset_left = 0.0
		graph.offset_top = 0.0
		graph.offset_right = 0.0
		graph.offset_bottom = 0.0


static func _apply_labels(graph, labels: Array[String]) -> void:
	if labels.size() > 0:
		graph.label1 = labels[0]
	if labels.size() > 1:
		graph.label2 = labels[1]
	if labels.size() > 2:
		graph.label3 = labels[2]


static func _colour_link(link: Node3D, colour: Color) -> void:
	var material := StandardMaterial3D.new()
	material.albedo_color = colour
	var rod := link.get_node_or_null("Rod")
	if rod:
		rod.material_override = material
	var bob := link.get_node_or_null("Bob")
	if bob:
		var marker := MeshInstance3D.new()
		var sphere := SphereMesh.new()
		sphere.radius = 0.05
		sphere.height = 0.10
		marker.mesh = sphere
		marker.material_override = material
		bob.add_child(marker)
