# res://scripts/hub/weapon_row.gd
extends PanelContainer
class_name WeaponRow

signal weapon_clicked(uuid: String)

var weapon: Weapon = null

var _thumb: TextureRect
var _name_label: Label
var _stats_label: Label

var _press_pos: Vector2 = Vector2.ZERO
var _pressing: bool = false
var _drag_threshold_px: float = 10.0

var _thumb_cache: Dictionary = {} # String -> Texture2D

func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP

	# IMPORTANT: rows inside VBox can collapse if not told to expand
	size_flags_horizontal = Control.SIZE_EXPAND_FILL
	size_flags_vertical = Control.SIZE_SHRINK_CENTER

func _ready() -> void:
	_apply_row_style()
	_build_ui()
	_refresh_ui()

func setup(w: Weapon) -> void:
	weapon = w
	_refresh_ui()

func _apply_row_style() -> void:
	# Make the row visibly separated regardless of theme.
	var sb: StyleBoxFlat = StyleBoxFlat.new()
	sb.bg_color = Color(0.10, 0.10, 0.11, 1.0)
	sb.border_color = Color(0.22, 0.22, 0.24, 1.0)
	sb.set_border_width_all(1)
	sb.set_corner_radius_all(6)
	add_theme_stylebox_override("panel", sb)

	custom_minimum_size = Vector2(0.0, 92.0)

func _build_ui() -> void:
	if _thumb != null:
		return

	var pad: MarginContainer = MarginContainer.new()
	pad.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	pad.size_flags_vertical = Control.SIZE_EXPAND_FILL
	pad.add_theme_constant_override("margin_left", 10)
	pad.add_theme_constant_override("margin_right", 10)
	pad.add_theme_constant_override("margin_top", 8)
	pad.add_theme_constant_override("margin_bottom", 8)
	add_child(pad)

	var root: HBoxContainer = HBoxContainer.new()
	root.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	root.size_flags_vertical = Control.SIZE_EXPAND_FILL
	root.add_theme_constant_override("separation", 10)
	pad.add_child(root)

	_thumb = TextureRect.new()
	_thumb.custom_minimum_size = Vector2(160.0, 76.0)
	_thumb.size_flags_horizontal = Control.SIZE_SHRINK_BEGIN
	_thumb.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_thumb.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_thumb.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	_thumb.modulate = Color(1.0, 1.0, 1.0, 1.0)
	root.add_child(_thumb)

	var right: VBoxContainer = VBoxContainer.new()
	right.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	right.size_flags_vertical = Control.SIZE_EXPAND_FILL
	right.add_theme_constant_override("separation", 2)
	root.add_child(right)

	_name_label = Label.new()
	_name_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_name_label.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_name_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	_name_label.clip_text = true
	_name_label.add_theme_font_size_override("font_size", 18)
	_name_label.modulate = Color(1.0, 1.0, 1.0, 1.0)
	right.add_child(_name_label)

	_stats_label = Label.new()
	_stats_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_stats_label.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_stats_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	_stats_label.clip_text = true
	_stats_label.add_theme_font_size_override("font_size", 13)
	_stats_label.modulate = Color(0.85, 0.85, 0.90, 1.0)
	right.add_child(_stats_label)

func _refresh_ui() -> void:
	if _name_label == null or _stats_label == null or _thumb == null:
		return

	if weapon == null:
		_name_label.text = "—"
		_stats_label.text = ""
		_thumb.texture = null
		return

	_name_label.text = weapon.name_

	# If base stats are zero because older resources didn't recompute, still show something.
	var any_stats: bool = (
		weapon.base_damage != 0.0
		or weapon.base_weight != 0.0
		or weapon.base_control != 0.0
		or weapon.base_speed != 0.0
		or weapon.base_edge != 0.0
	)

	if any_stats:
		_stats_label.text = "DMG %.0f  WGT %.0f  CTL %.0f  SPD %.0f  EDGE %.0f" % [
			weapon.base_damage,
			weapon.base_weight,
			weapon.base_control,
			weapon.base_speed,
			weapon.base_edge
		]
	else:
		_stats_label.text = "DMG —  WGT —  CTL —  SPD —  EDGE —"

	_thumb.texture = _get_thumbnail_texture()

func _get_thumbnail_texture() -> Texture2D:
	if weapon == null:
		return null

	if weapon.uuid != "" and _thumb_cache.has(weapon.uuid):
		var cached: Variant = _thumb_cache[weapon.uuid]
		if cached is Texture2D:
			return cached as Texture2D

	var store: Node = get_node_or_null("/root/Storage")
	if store != null:
		if store.has_method("load_weapon_thumbnail"):
			var tex1: Variant = store.call("load_weapon_thumbnail", weapon.uuid)
			if tex1 is Texture2D:
				_thumb_cache[weapon.uuid] = tex1
				return tex1 as Texture2D
		if store.has_method("get_weapon_thumbnail"):
			var tex2: Variant = store.call("get_weapon_thumbnail", weapon.uuid)
			if tex2 is Texture2D:
				_thumb_cache[weapon.uuid] = tex2
				return tex2 as Texture2D

	var gen: Texture2D = _rasterize_weapon_to_texture(weapon)
	if weapon.uuid != "":
		_thumb_cache[weapon.uuid] = gen
	return gen

func _rasterize_weapon_to_texture(w: Weapon) -> Texture2D:
	var scale: int = 2
	var img_w: int = Weapon.GRID_W * scale
	var img_h: int = Weapon.GRID_H * scale

	var img: Image = Image.create(img_w, img_h, false, Image.FORMAT_RGBA8)
	img.fill(Color(0.0, 0.0, 0.0, 0.0))

	var blade_col: Color = w.display_color
	var outline_col: Color = Color(0.05, 0.05, 0.06, 1.0)
	var handle_col: Color = Color(0.32, 0.20, 0.10, 1.0)

	for gy in range(Weapon.GRID_H):
		for gx in range(Weapon.GRID_W):
			if w.get_cell(gx, gy) < 0.5:
				continue
			var px0: int = gx * scale
			var py0: int = gy * scale
			for sy in range(scale):
				for sx in range(scale):
					img.set_pixel(px0 + sx, py0 + sy, blade_col)

	for gy2 in range(Weapon.GRID_H):
		for gx2 in range(Weapon.GRID_W):
			if w.get_cell(gx2, gy2) < 0.5:
				continue
			var edge: bool = false
			if gx2 <= 0 or w.get_cell(gx2 - 1, gy2) < 0.5: edge = true
			elif gx2 >= Weapon.GRID_W - 1 or w.get_cell(gx2 + 1, gy2) < 0.5: edge = true
			elif gy2 <= 0 or w.get_cell(gx2, gy2 - 1) < 0.5: edge = true
			elif gy2 >= Weapon.GRID_H - 1 or w.get_cell(gx2, gy2 + 1) < 0.5: edge = true

			if edge:
				var px1: int = gx2 * scale
				var py1: int = gy2 * scale
				for sy2 in range(scale):
					for sx2 in range(scale):
						img.set_pixel(px1 + sx2, py1 + sy2, outline_col)

	if w.attach_x >= 0 and w.attach_y >= 0 and w.handle_length > 0.0:
		var ax: float = (float(w.attach_x) + 0.5) * float(scale)
		var ay: float = (float(w.attach_y) + 0.5) * float(scale)

		var len_px: float = clampf(w.handle_length * 0.6, 12.0, 90.0) * float(scale) * 0.5
		var dir: Vector2 = Vector2(cos(w.handle_angle), sin(w.handle_angle))
		if dir.length_squared() < 0.0001:
			dir = Vector2.DOWN
		dir = dir.normalized()

		var p0: Vector2 = Vector2(ax, ay)
		var p1: Vector2 = p0 + dir * len_px

		_draw_thick_line(img, p0, p1, handle_col, 6 * scale)
		_draw_filled_circle(img, p1, 4 * scale, handle_col)

	return ImageTexture.create_from_image(img)

func _draw_thick_line(img: Image, p0: Vector2, p1: Vector2, col: Color, thickness: int) -> void:
	var v: Vector2 = p1 - p0
	var dist: float = v.length()
	if dist < 0.001:
		_draw_filled_circle(img, p0, thickness / 2, col)
		return

	var steps: int = int(ceil(dist))
	if steps < 1:
		steps = 1

	for i in range(steps + 1):
		var t: float = float(i) / float(steps)
		var p: Vector2 = p0.lerp(p1, t)
		_draw_filled_circle(img, p, thickness / 2, col)

func _draw_filled_circle(img: Image, center: Vector2, radius: int, col: Color) -> void:
	var cx: int = int(round(center.x))
	var cy: int = int(round(center.y))
	var r: int = max(1, radius)
	var rr: int = r * r

	var x0: int = max(0, cx - r)
	var x1: int = min(img.get_width() - 1, cx + r)
	var y0: int = max(0, cy - r)
	var y1: int = min(img.get_height() - 1, cy + r)

	for y in range(y0, y1 + 1):
		var dy: int = y - cy
		for x in range(x0, x1 + 1):
			var dx: int = x - cx
			if dx * dx + dy * dy <= rr:
				img.set_pixel(x, y, col)

func _gui_input(event: InputEvent) -> void:
	if weapon == null:
		return

	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event
		if mb.button_index == MOUSE_BUTTON_LEFT:
			if mb.pressed:
				_pressing = true
				_press_pos = mb.position
			else:
				if _pressing:
					var d2: float = mb.position.distance_squared_to(_press_pos)
					if d2 <= _drag_threshold_px * _drag_threshold_px:
						weapon_clicked.emit(weapon.uuid)
				_pressing = false

func _get_drag_data(_at_position: Vector2) -> Variant:
	if weapon == null:
		return null

	var data: Dictionary = {
		"type": "weapon",
		"uuid": weapon.uuid,
	}

	var preview: Control
	if _thumb != null and _thumb.texture != null:
		var pr: TextureRect = TextureRect.new()
		pr.texture = _thumb.texture
		pr.custom_minimum_size = Vector2(110.0, 60.0)
		pr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
		pr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
		preview = pr
	else:
		var lb: Label = Label.new()
		lb.text = weapon.name_
		preview = lb

	set_drag_preview(preview)
	return data
