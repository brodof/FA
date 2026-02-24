# res://scripts/forge/forge_controller.gd
# Full replacement. Adds: hold LMB to hammer repeatedly every 0.3s.
# Notes:
# - Uses a Timer (no polling hacks).
# - Works for desktop (mouse) and keeps your existing mobile tap behavior.
# - Repeats hammer strikes at the current reticle position while holding.

extends Node2D

@export var anvil_area_path: NodePath
@export var blade_view_path: NodePath
@export var reticle_path: NodePath
@export var ore_selector_path: NodePath
@export var hammer_selector_path: NodePath
@export var name_field_path: NodePath
@export var handle_slider_path: NodePath
@export var hammer_button_path: NodePath
@export var finalize_button_path: NodePath
@export var stats_label_path: NodePath
@export var back_button_path: NodePath

# Radius in GRID CELLS (Weapon grid)
@export var hammer_radius_cells: int = 7
@export var click_strength: float = 1.0
@export var min_radius_cells: int = 3
@export var max_radius_cells: int = 32

# NEW: hold-to-hammer repeat interval
@export var hold_hammer_interval_sec: float = 0.2

enum HammerShape { CIRCLE, RECT, TRIANGLE_UP, DIAMOND, LINE_H, LINE_V }

var current_hammer: int = HammerShape.CIRCLE

var anvil_area: ColorRect
var blade_view: BladeView
var reticle: ReticleView
var ore_selector: OptionButton
var hammer_selector: OptionButton
var name_field: LineEdit
var handle_slider: HSlider
var hammer_button: Button
var finalize_button: Button
var stats_label: Label
var back_button: Button

var weapon: Weapon
var ore_data: OreData = OreData.new()
var current_kernel: Array = []
var kernel_w: int = 1
var kernel_h: int = 1

# NEW: hold state
var hammer_hold_timer: Timer = null
var is_hammer_holding: bool = false

func _ready() -> void:
	anvil_area      = Require.node(self, anvil_area_path, "ColorRect")
	var raw_bv: Control = Require.node(self, blade_view_path, "Control")
	blade_view = raw_bv as BladeView
	if blade_view == null:
		push_error("Forge: BladeView missing or wrong script.")
		return

	var reticle_ctrl: Control = Require.node(self, reticle_path, "Control")
	reticle = reticle_ctrl as ReticleView
	if reticle == null:
		push_error("Forge: Reticle missing or wrong script.")
		return

	ore_selector    = Require.node(self, ore_selector_path, "OptionButton")
	hammer_selector = Require.node(self, hammer_selector_path, "OptionButton")
	name_field      = Require.node(self, name_field_path, "LineEdit")
	handle_slider   = Require.node(self, handle_slider_path, "HSlider")
	hammer_button   = Require.node(self, hammer_button_path, "Button")
	finalize_button = Require.node(self, finalize_button_path, "Button")
	stats_label     = Require.node(self, stats_label_path, "Label")
	back_button     = Require.node(self, back_button_path, "Button")

	anvil_area.mouse_filter = Control.MOUSE_FILTER_PASS
	blade_view.mouse_filter = Control.MOUSE_FILTER_IGNORE
	reticle.mouse_filter    = Control.MOUSE_FILTER_IGNORE

	_populate_ore_selector()
	_populate_hammer_selector()

	# NEW: timer for hold-to-hammer
	hammer_hold_timer = Timer.new()
	hammer_hold_timer.one_shot = false
	hammer_hold_timer.wait_time = maxf(0.05, hold_hammer_interval_sec)
	add_child(hammer_hold_timer)
	hammer_hold_timer.timeout.connect(_on_hammer_hold_tick)

	# Resume pending weapon if coming back from HandleMount via Cancel
	var gm: Node = get_node("/root/GameModel") as Node
	var resumed_weapon: Weapon = null
	if gm != null:
		resumed_weapon = gm.pending_weapon

	if resumed_weapon != null:
		weapon = resumed_weapon
		blade_view.set_weapon(weapon)

		# Select weapon's ore type in UI if possible
		var count: int = ore_selector.get_item_count()
		var selected_index: int = 0
		for i in range(count):
			if ore_selector.get_item_text(i) == weapon.ore_type:
				selected_index = i
				break
		ore_selector.select(selected_index)
	else:
		weapon = Weapon.new()
		var default_ore: String = ore_selector.get_item_text(ore_selector.get_selected())
		_set_ore_and_reset(default_ore)
		blade_view.set_weapon(weapon)

	_rebuild_kernel()
	_update_reticle_visual()

	if weapon.handle_length > 0.0:
		handle_slider.value = weapon.handle_length
	else:
		handle_slider.value = 80.0

	if weapon.name_ != "":
		name_field.text = weapon.name_
	else:
		name_field.text = "Blade_%d" % int(Time.get_unix_time_from_system())

	hammer_button.pressed.connect(_on_hammer_pressed)
	finalize_button.pressed.connect(_on_finalize_pressed)
	back_button.pressed.connect(_on_back_pressed)
	ore_selector.item_selected.connect(_on_ore_changed)
	hammer_selector.item_selected.connect(_on_hammer_changed)

	_update_stats()

func _exit_tree() -> void:
	_stop_hammer_hold()

func _input(event: InputEvent) -> void:
	var gr: Rect2 = anvil_area.get_global_rect()

	if event is InputEventMouseMotion or event is InputEventScreenTouch or event is InputEventScreenDrag:
		var p: Vector2 = event.position
		if gr.has_point(p):
			var local_pos: Vector2 = p - gr.position
			local_pos.x = clampf(local_pos.x, 0.0, anvil_area.size.x)
			local_pos.y = clampf(local_pos.y, 0.0, anvil_area.size.y)
			reticle.position = local_pos - reticle.size * 0.5

	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event

		if mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			hammer_radius_cells = clampi(hammer_radius_cells + 1, min_radius_cells, max_radius_cells)
			_rebuild_kernel()
			_update_reticle_visual()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			hammer_radius_cells = clampi(hammer_radius_cells - 1, min_radius_cells, max_radius_cells)
			_rebuild_kernel()
			_update_reticle_visual()

		# NEW: hold to hammer
		if mb.button_index == MOUSE_BUTTON_LEFT:
			if mb.pressed:
				if gr.has_point(mb.position) and not _point_in_button(hammer_button, mb.position):
					_start_hammer_hold()
			else:
				_stop_hammer_hold()

	if event is InputEventScreenTouch and event.pressed:
		# Keep mobile tap behavior as single strike.
		if gr.has_point(event.position):
			_on_hammer_pressed()

func _point_in_button(btn: Button, global_point: Vector2) -> bool:
	return btn.get_global_rect().has_point(global_point)

# NEW: hold control
func _start_hammer_hold() -> void:
	if weapon == null:
		return
	if is_hammer_holding:
		return

	is_hammer_holding = true

	# Immediate strike
	_on_hammer_pressed()

	if hammer_hold_timer != null:
		hammer_hold_timer.wait_time = maxf(0.05, hold_hammer_interval_sec)
		hammer_hold_timer.start()

func _stop_hammer_hold() -> void:
	is_hammer_holding = false
	if hammer_hold_timer != null:
		hammer_hold_timer.stop()

func _on_hammer_hold_tick() -> void:
	if not is_hammer_holding:
		return
	_on_hammer_pressed()

func _on_hammer_pressed() -> void:
	if weapon == null:
		return

	var size_px: Vector2 = anvil_area.size
	var local_center: Vector2 = reticle.position + reticle.size * 0.5
	local_center.x = clampf(local_center.x, 0.0, size_px.x)
	local_center.y = clampf(local_center.y, 0.0, size_px.y)

	var gx: float = (local_center.x / size_px.x) * float(Weapon.GRID_W)
	var gy: float = (local_center.y / size_px.y) * float(Weapon.GRID_H)
	weapon.apply_strike_push(Vector2(gx, gy), current_kernel, click_strength)
	blade_view.refresh()
	_update_stats()

func _on_finalize_pressed() -> void:
	# Quench & go to HandleMount
	var idx: int = ore_selector.get_selected()
	var ore_name: String = ore_selector.get_item_text(idx)
	weapon.ore_type = ore_name
	weapon.name_ = name_field.text.strip_edges()
	weapon.handle_length = float(handle_slider.value)

	var gm: Node = get_node("/root/GameModel") as Node
	if gm != null:
		gm.pending_weapon = weapon

	get_tree().change_scene_to_file("res://scenes/handle/HandleMount.tscn")

func _on_back_pressed() -> void:
	_stop_hammer_hold()
	get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")

func _on_ore_changed(_idx: int) -> void:
	_stop_hammer_hold()
	_set_ore_and_reset(ore_selector.get_item_text(ore_selector.get_selected()))
	blade_view.refresh()
	_update_stats()

func _on_hammer_changed(_idx: int) -> void:
	current_hammer = hammer_selector.get_selected_id()
	_rebuild_kernel()
	_update_reticle_visual()

func _set_ore_and_reset(ore_name: String) -> void:
	weapon.reset_bar(ore_data, ore_name)

func _populate_ore_selector() -> void:
	ore_selector.clear()
	for ore_name in ore_data.ores.keys():
		ore_selector.add_item(ore_name)
	ore_selector.select(0)

func _populate_hammer_selector() -> void:
	hammer_selector.clear()
	hammer_selector.add_item("Circle", HammerShape.CIRCLE)
	hammer_selector.add_item("Rectangle", HammerShape.RECT)
	hammer_selector.add_item("Triangle (Up)", HammerShape.TRIANGLE_UP)
	hammer_selector.add_item("Diamond", HammerShape.DIAMOND)
	hammer_selector.add_item("Line (H)", HammerShape.LINE_H)
	hammer_selector.add_item("Line (V)", HammerShape.LINE_V)
	hammer_selector.select(HammerShape.CIRCLE)

func _update_stats() -> void:
	if weapon == null:
		stats_label.text = "No weapon"
		return
	stats_label.text = "DMG: %.1f\nWGT: %.1f\nCTL: %.1f\nSPD: %.1f\nEDGE: %.1f" % [
		weapon.base_damage,
		weapon.base_weight,
		weapon.base_control,
		weapon.base_speed,
		weapon.base_edge
	]

func _rebuild_kernel() -> void:
	var r: int = clampi(hammer_radius_cells, min_radius_cells, max_radius_cells)
	match current_hammer:
		HammerShape.CIRCLE:
			current_kernel = _make_disc(r)
		HammerShape.RECT:
			current_kernel = _make_rect(r * 2, r)
		HammerShape.TRIANGLE_UP:
			current_kernel = _make_triangle_up(r)
		HammerShape.DIAMOND:
			current_kernel = _make_diamond(r)
		HammerShape.LINE_H:
			current_kernel = _make_line_h(r, max(1, r / 3))
		HammerShape.LINE_V:
			current_kernel = _make_line_v(r, max(1, r / 3))
		_:
			current_kernel = _make_disc(r)

	kernel_h = current_kernel.size()
	if kernel_h > 0:
		kernel_w = (current_kernel[0] as Array).size()
	else:
		kernel_w = 1
	reticle.set_kernel(current_kernel)

func _update_reticle_visual() -> void:
	if kernel_w <= 0 or kernel_h <= 0:
		return
	var cell_px: Vector2 = Vector2(
		anvil_area.size.x / float(Weapon.GRID_W),
		anvil_area.size.y / float(Weapon.GRID_H)
	)
	var px_size: Vector2 = Vector2(float(kernel_w) * cell_px.x, float(kernel_h) * cell_px.y)
	if px_size.x < 10.0:
		px_size.x = 10.0
	if px_size.y < 10.0:
		px_size.y = 10.0
	reticle.custom_minimum_size = px_size
	reticle.size = px_size
	reticle.set_kernel(current_kernel)

# --- kernel generators ---

func _make_disc(radius: int) -> Array:
	var size: int = radius * 2 + 1
	var arr: Array = []
	var rr: float = float(radius) + 0.5
	for y in range(size):
		var row: Array = []
		for x in range(size):
			var dx: float = float(x - radius)
			var dy: float = float(y - radius)
			row.append(1.0 if (dx * dx + dy * dy) <= (rr * rr) else 0.0)
		arr.append(row)
	return arr

func _make_rect(half_w: int, half_h: int) -> Array:
	var w: int = half_w * 2 + 1
	var h: int = half_h * 2 + 1
	var arr: Array = []
	for _y in range(h):
		var row: Array = []
		for _x in range(w):
			row.append(1.0)
		arr.append(row)
	return arr

func _make_triangle_up(radius: int) -> Array:
	var size: int = radius * 2 + 1
	var arr: Array = []
	for y in range(size):
		var row: Array = []
		var span: int = int(float(size) * (float(y + 1) / float(size)))
		if span < 1:
			span = 1
		if span > size:
			span = size
		var start_x: int = (size - span) / 2
		var end_x: int = start_x + span
		for x in range(size):
			if x >= start_x and x < end_x:
				row.append(1.0)
			else:
				row.append(0.0)
		arr.append(row)
	return arr

func _make_diamond(radius: int) -> Array:
	var size: int = radius * 2 + 1
	var arr: Array = []
	for y in range(size):
		var row: Array = []
		for x in range(size):
			var dx: int = abs(x - radius)
			var dy: int = abs(y - radius)
			row.append(1.0 if (dx + dy) <= radius else 0.0)
		arr.append(row)
	return arr

func _make_line_h(radius: int, thickness: int) -> Array:
	var w: int = radius * 2 + 1
	var h: int = thickness * 2 + 1
	var arr: Array = []
	for _y in range(h):
		var row: Array = []
		for _x in range(w):
			row.append(1.0)
		arr.append(row)
	return arr

func _make_line_v(radius: int, thickness: int) -> Array:
	var w: int = thickness * 2 + 1
	var h: int = radius * 2 + 1
	var arr: Array = []
	for _y in range(h):
		var row: Array = []
		for _x in range(w):
			row.append(1.0)
		arr.append(row)
	return arr
