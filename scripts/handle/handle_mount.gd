extends Node2D

@export var blade_view_path: NodePath
@export var length_slider_path: NodePath
@export var name_field_path: NodePath
@export var confirm_button_path: NodePath
@export var cancel_button_path: NodePath


var blade_view: BladeView
var length_slider: HSlider
var name_field: LineEdit
var confirm_button: Button
var cancel_button: Button

var weapon: Weapon = null
var gm: GameModel = null

var have_attach: bool = false
var handle_fixed: bool = false
var attach_cell: Vector2i = Vector2i(-1, -1)
var handle_angle: float = 0.0

func _ready() -> void:
	blade_view     = Require.node(self, blade_view_path, "Control") as BladeView
	length_slider  = Require.node(self, length_slider_path, "HSlider")
	name_field     = Require.node(self, name_field_path, "LineEdit")
	confirm_button = Require.node(self, confirm_button_path, "Button")
	cancel_button  = Require.node(self, cancel_button_path, "Button")

	gm = get_node("/root/GameModel") as GameModel
	if gm == null:
		push_error("HandleMount: GameModel not found.")
		get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")
		return

	weapon = gm.pending_weapon
	if weapon == null:
		push_error("HandleMount: pending_weapon is null, returning to Hub.")
		get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")
		return

	blade_view.set_weapon(weapon)
	# Make blade smaller than in Forge
	blade_view.blade_scale = 0.5

	# Length slider setup
	length_slider.min_value = 20.0
	length_slider.max_value = 200.0
	length_slider.step = 1.0
	length_slider.value = weapon.handle_length
	length_slider.value_changed.connect(_on_length_changed)

	# Name field
	if weapon.name_ == "" or weapon.name_ == "Unnamed Blade":
		name_field.text = "Blade_%d" % int(Time.get_unix_time_from_system())
	else:
		name_field.text = weapon.name_

	# Existing attach?
	if weapon.attach_x >= 0 and weapon.attach_y >= 0:
		attach_cell = Vector2i(weapon.attach_x, weapon.attach_y)
		have_attach = true
		handle_fixed = true  # already chosen in a previous session
		handle_angle = weapon.handle_angle if weapon.handle_angle != 0.0 else PI * 0.5
		blade_view.set_attach_cell(attach_cell)
		blade_view.set_handle_visual(attach_cell, handle_angle, float(length_slider.value))
	else:
		blade_view.set_attach_cell(Vector2i(-1, -1))
		have_attach = false
		handle_fixed = false

	confirm_button.pressed.connect(_on_confirm_pressed)
	cancel_button.pressed.connect(_on_cancel_pressed)

	blade_view.mouse_filter = Control.MOUSE_FILTER_STOP
	blade_view.gui_input.connect(_on_blade_gui_input)

func _on_blade_gui_input(event: InputEvent) -> void:
	# First left-click: choose attach + start drag.
	# Second left-click: fix handle in place.
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		var mb: InputEventMouseButton = event
		var local: Vector2 = mb.position
		if not have_attach:
			_choose_attach_at(local)
		elif have_attach and not handle_fixed:
			handle_fixed = true
		return

	if event is InputEventMouseMotion and have_attach and not handle_fixed:
		var mm: InputEventMouseMotion = event
		var local: Vector2 = mm.position
		_update_handle_from_local_mouse(local, true)

func _choose_attach_at(local: Vector2) -> void:
	var grid_pos: Vector2i = blade_view.screen_to_grid(local)
	if grid_pos.x == -1:
		return

	var edge_cell: Vector2i = _find_nearest_edge_cell(grid_pos.x, grid_pos.y, 6)
	if edge_cell.x == -1:
		return

	attach_cell = edge_cell
	have_attach = true
	handle_fixed = false

	weapon.attach_x = attach_cell.x
	weapon.attach_y = attach_cell.y

	blade_view.set_attach_cell(attach_cell)
	_update_handle_from_local_mouse(local, true)

func _update_handle_from_local_mouse(local: Vector2, update_length: bool) -> void:
	if not have_attach or handle_fixed:
		return

	var base_px: Vector2 = blade_view.grid_to_screen(attach_cell)
	var v: Vector2 = local - base_px
	if v.length_squared() < 1.0:
		return

	handle_angle = atan2(v.y, v.x)

	if update_length:
		var raw_len: float = v.length()
		var norm: float = clampf(raw_len / maxf(1.0, blade_view.size.y), 0.1, 1.0)
		var new_len: float = lerp(float(length_slider.min_value), float(length_slider.max_value), norm)

		length_slider.set_block_signals(true)
		length_slider.value = new_len
		length_slider.set_block_signals(false)

	blade_view.set_handle_visual(attach_cell, handle_angle, float(length_slider.value))

func _find_nearest_edge_cell(cx: int, cy: int, max_radius: int) -> Vector2i:
	var best: Vector2i = Vector2i(-1, -1)
	var best_dist2: float = 1e20

	for r in range(0, max_radius + 1):
		var found_in_ring: bool = false
		for dy in range(-r, r + 1):
			for dx in range(-r, r + 1):
				var x: int = cx + dx
				var y: int = cy + dy
				if x < 0 or y < 0 or x >= Weapon.GRID_W or y >= Weapon.GRID_H:
					continue
				if not _is_edge_cell(x, y):
					continue

				var d2: float = float(dx * dx + dy * dy)
				if d2 < best_dist2:
					best_dist2 = d2
					best = Vector2i(x, y)
					found_in_ring = true
		if found_in_ring:
			break

	return best

func _is_edge_cell(x: int, y: int) -> bool:
	var d: float = weapon.get_cell(x, y)
	if d < 0.5:
		return false
	# 4-neighbor edge
	if x <= 0 or weapon.get_cell(x - 1, y) < 0.5:
		return true
	if x >= Weapon.GRID_W - 1 or weapon.get_cell(x + 1, y) < 0.5:
		return true
	if y <= 0 or weapon.get_cell(x, y - 1) < 0.5:
		return true
	if y >= Weapon.GRID_H - 1 or weapon.get_cell(x, y + 1) < 0.5:
		return true
	return false

func _on_length_changed(value: float) -> void:
	if not have_attach:
		return
	blade_view.set_handle_visual(attach_cell, handle_angle, float(value))

func _on_confirm_pressed() -> void:
	if not have_attach or not handle_fixed:
		push_warning("You must select and fix a handle before confirming.")
		return

	weapon.attach_x = attach_cell.x
	weapon.attach_y = attach_cell.y
	weapon.handle_length = float(length_slider.value)
	weapon.handle_angle = handle_angle
	weapon.name_ = name_field.text.strip_edges()
	weapon.finalize()

	var store: Node = get_node("/root/Storage")
	if store == null:
		push_error("HandleMount: Storage not found, cannot save weapon.")
	else:
		var path: String = store.save_weapon(weapon, weapon.name_)
		if path != "":
			weapon.saved_path = path

	gm.add_weapon(weapon)
	if gm.current_weapon_p1 == null:
		gm.current_weapon_p1 = weapon

	gm.pending_weapon = null
	get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")

func _on_cancel_pressed() -> void:
	# Cancel should discard current handle selection so the user can pick a new one later.
	weapon.attach_x = -1
	weapon.attach_y = -1
	weapon.handle_angle = 0.0

	have_attach = false
	handle_fixed = false
	attach_cell = Vector2i(-1, -1)
	blade_view.set_attach_cell(attach_cell)
	blade_view.set_handle_visual(attach_cell, 0.0, float(length_slider.value))

	gm.pending_weapon = weapon
	get_tree().change_scene_to_file("res://scenes/forge/Forge.tscn")
