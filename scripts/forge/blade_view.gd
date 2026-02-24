extends Control
class_name BladeView

var weapon: Weapon

@export var bg_color: Color = Color(0.12, 0.12, 0.12, 1.0)
@export var edge_threshold: float = 0.5

# Factor for drawing the blade silhouette; 1.0 = full size, < 1.0 = smaller.
@export var blade_scale: float = 1.0

# Attach marker
var attach_cell: Vector2i = Vector2i(-1, -1)

# Handle ghost
var show_handle: bool = false
var handle_angle: float = 0.0     # radians
var handle_length: float = 80.0   # abstract units

func set_weapon(w: Weapon) -> void:
	weapon = w
	queue_redraw()

func refresh() -> void:
	queue_redraw()

func set_attach_cell(cell: Vector2i) -> void:
	attach_cell = cell
	queue_redraw()

func set_handle_visual(origin_cell: Vector2i, angle: float, length: float) -> void:
	attach_cell = origin_cell
	handle_angle = angle
	handle_length = length
	show_handle = (attach_cell.x >= 0 and attach_cell.y >= 0)
	queue_redraw()

# Converts a grid cell to the on-screen center position, respecting blade_scale.
func grid_to_screen(cell: Vector2i) -> Vector2:
	var gw: int = Weapon.GRID_W
	var gh: int = Weapon.GRID_H
	if gw <= 0 or gh <= 0:
		return size * 0.5

	var cell_w: float = size.x / float(gw)
	var cell_h: float = size.y / float(gh)
	var center: Vector2 = size * 0.5

	var base_px: Vector2 = Vector2(
		(float(cell.x) + 0.5) * cell_w,
		(float(cell.y) + 0.5) * cell_h
	)

	var local_blade_scale: float = blade_scale
	if local_blade_scale <= 0.0:
		local_blade_scale = 1.0

	var offset: Vector2 = (base_px - center) * local_blade_scale
	return center + offset

# Converts a screen position to grid coordinates, inverting blade_scale.
func screen_to_grid(pos: Vector2) -> Vector2i:
	var gw: int = Weapon.GRID_W
	var gh: int = Weapon.GRID_H
	if gw <= 0 or gh <= 0:
		return Vector2i(-1, -1)

	var local_blade_scale: float = blade_scale
	if local_blade_scale <= 0.0:
		return Vector2i(-1, -1)

	var cell_w: float = size.x / float(gw)
	var cell_h: float = size.y / float(gh)
	var center: Vector2 = size * 0.5

	var delta: Vector2 = pos - center
	var base_px: Vector2 = center + delta / local_blade_scale

	var gx: int = int(floor(base_px.x / cell_w))
	var gy: int = int(floor(base_px.y / cell_h))

	if gx < 0 or gy < 0 or gx >= gw or gy >= gh:
		return Vector2i(-1, -1)

	return Vector2i(gx, gy)

func _draw() -> void:
	# Background
	draw_rect(Rect2(Vector2.ZERO, size), bg_color, true)

	if weapon == null:
		return

	var gw: int = Weapon.GRID_W
	var gh: int = Weapon.GRID_H
	if gw <= 0 or gh <= 0:
		return

	var metal: Color = weapon.display_color
	var cell_w: float = size.x / float(gw)
	var cell_h: float = size.y / float(gh)
	var center: Vector2 = size * 0.5

	var local_blade_scale: float = blade_scale
	if local_blade_scale <= 0.0:
		local_blade_scale = 1.0

	# Blade fill (scaled around center)
	for y in range(gh):
		for x in range(gw):
			var d: float = weapon.get_cell(x, y)
			if d < edge_threshold:
				continue

			var base_px: Vector2 = Vector2(
				(float(x) + 0.5) * cell_w,
				(float(y) + 0.5) * cell_h
			)
			var offset: Vector2 = (base_px - center) * local_blade_scale
			var center_px: Vector2 = center + offset
			var rect_size: Vector2 = Vector2(cell_w, cell_h) * local_blade_scale
			var rect_pos: Vector2 = center_px - rect_size * 0.5
			var r: Rect2 = Rect2(rect_pos, rect_size)
			draw_rect(r, metal, true)

	# Blade outline
	var outline: Color = Color(metal.r, metal.g, metal.b, 0.85)
	for y2 in range(1, gh - 1):
		for x2 in range(1, gw - 1):
			var d2: float = weapon.get_cell(x2, y2)
			if d2 < edge_threshold:
				continue

			if weapon.get_cell(x2 + 1, y2) < edge_threshold \
			or weapon.get_cell(x2 - 1, y2) < edge_threshold \
			or weapon.get_cell(x2, y2 + 1) < edge_threshold \
			or weapon.get_cell(x2, y2 - 1) < edge_threshold:
				var base_px2: Vector2 = Vector2(
					(float(x2) + 0.5) * cell_w,
					(float(y2) + 0.5) * cell_h
				)
				var offset2: Vector2 = (base_px2 - center) * local_blade_scale
				var center_px2: Vector2 = center + offset2
				var r_outline: float = maxf(1.0, minf(cell_w, cell_h) * 0.06 * local_blade_scale)
				draw_circle(center_px2, r_outline, outline)

	# Attach marker
	if attach_cell.x >= 0 and attach_cell.y >= 0 \
	and attach_cell.x < gw and attach_cell.y < gh \
	and weapon.get_cell(attach_cell.x, attach_cell.y) >= edge_threshold:
		var attach_px: Vector2 = grid_to_screen(attach_cell)
		var r0: float = maxf(2.0, minf(cell_w, cell_h) * 0.35 * local_blade_scale)
		draw_circle(attach_px, r0, Color(1.0, 0.2, 0.1, 0.9))
		draw_circle(attach_px, r0 * 0.4, Color(0.05, 0.05, 0.05, 1.0))

	# Ghost handle
	if show_handle and attach_cell.x >= 0 and attach_cell.y >= 0:
		var base_px_handle: Vector2 = grid_to_screen(attach_cell)

		# Make it able to go longer on-screen
		# - allow larger logical length
		# - map it to a bigger fraction of viewport height
		var clamped_len: float = clampf(handle_length, 10.0, 300.0)
		var norm_len: float = clamped_len / 230.0
		var px_len: float = norm_len * size.y * 1

		var dir: Vector2 = Vector2(cos(handle_angle), sin(handle_angle))
		var tip_px: Vector2 = base_px_handle + dir * px_len

		# Make the shaft ~3x thicker than before
		var shaft_color: Color = Color(0.9, 0.75, 0.5, 0.9)
		var cap_color: Color = Color(0.2, 0.12, 0.05, 1.0)
		var base_width: float = minf(cell_w, cell_h) * 0.7
		var width_px: float = maxf(2.0, base_width * 3.0)

		draw_line(base_px_handle, tip_px, shaft_color, width_px)
		draw_circle(tip_px, width_px * 0.7, cap_color)
