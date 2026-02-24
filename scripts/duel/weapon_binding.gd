# res://scenes/weapons/weapon_binding.gd
extends Node2D
class_name WeaponBinding

@export var cell_world_scale: float = 0.7
@export var handle_world_scale: float = 0.5
@export var handle_thickness: float = 5.0

@export var default_blade_color: Color = Color(0.9, 0.9, 0.9, 1.0)
@export var default_handle_color: Color = Color(0.582, 0.496, 0.457, 1.0)

@export var grip_offset_from_butt: float = 18.0

var weapon: Resource = null
var weapon_root: WeaponRoot = null

var _geometry: WeaponGeometry = WeaponGeometry.new()
var _blade_poly_local: PackedVector2Array = PackedVector2Array()
var _convex_parts: Array[PackedVector2Array] = []
var _handle_len_world: float = 64.0

func _ready() -> void:
	visible = false

	if weapon != null:
		_rebuild_from_weapon(weapon)
	else:
		_rebuild_default()

	_push_to_root_deferred()

func set_weapon(w: Resource) -> void:
	weapon = w
	if weapon != null:
		_rebuild_from_weapon(weapon)
	else:
		_rebuild_default()

	_push_to_root_deferred()

func follow_weapon_root(root: WeaponRoot) -> void:
	weapon_root = root
	_push_to_root_deferred()

func get_convex_parts() -> Array[PackedVector2Array]:
	return _convex_parts

func get_blade_polygon_local() -> PackedVector2Array:
	return _blade_poly_local

func get_handle_len_world() -> float:
	return _handle_len_world

func get_handle_thickness() -> float:
	return handle_thickness

func get_blade_color() -> Color:
	if weapon != null:
		var c_v: Variant = weapon.get("display_color") if weapon.has_method("get") else null
		if c_v is Color:
			return c_v as Color
	return default_blade_color

func get_handle_color() -> Color:
	if weapon != null:
		var c_v: Variant = weapon.get("handle_color") if weapon.has_method("get") else null
		if c_v is Color:
			return c_v as Color
	return default_handle_color

func _push_to_root_deferred() -> void:
	if weapon_root == null:
		return
	call_deferred("_push_to_root_now")

func _handle_convex_local() -> PackedVector2Array:
	var half_t: float = handle_thickness * 0.5
	var L: float = maxf(1.0, _handle_len_world)
	# Origin = ATTACH. +X goes ATTACH -> BUTT.
	return PackedVector2Array([
		Vector2(0.0, -half_t),
		Vector2(L, -half_t),
		Vector2(L,  half_t),
		Vector2(0.0,  half_t),
	])

func _push_to_root_now() -> void:
	if weapon_root == null:
		return

	# Origin = ATTACH. +X goes ATTACH -> BUTT.
	var butt_local: Vector2 = Vector2(_handle_len_world, 0.0)
	var grip_x: float = maxf(0.0, _handle_len_world - grip_offset_from_butt)
	var grip_local: Vector2 = Vector2(grip_x, 0.0)
	weapon_root.set_attachment_points_local(butt_local, grip_local)

	# Convex blade parts + handle rect (so handle collides with ground)
	var parts: Array[PackedVector2Array] = []
	for p in _convex_parts:
		parts.append(p)
	parts.append(_handle_convex_local())
	weapon_root.set_convex_parts(parts)

	weapon_root.set_visuals(
		_blade_poly_local,
		_handle_len_world,
		handle_thickness,
		get_blade_color(),
		get_handle_color()
	)

func _rebuild_default() -> void:
	_handle_len_world = 64.0
	_blade_poly_local = PackedVector2Array([
		Vector2(-32.0, -4.0),
		Vector2(32.0, -4.0),
		Vector2(32.0, 4.0),
		Vector2(-32.0, 4.0),
	])
	_convex_parts = _geometry.convex_decompose(_blade_poly_local)

func _rebuild_from_weapon(w: Resource) -> void:
	if w == null:
		_rebuild_default()
		return

	var blade_poly: PackedVector2Array = _build_blade_polygon_from_weapon(w)
	if blade_poly.size() < 3:
		_rebuild_default()
		return

	var len_units: float = 60.0
	if w.has_method("get"):
		var hl_v: Variant = w.get("handle_length")
		if hl_v != null:
			len_units = float(hl_v)
	if len_units <= 0.0:
		len_units = 60.0

	_handle_len_world = len_units * handle_world_scale
	_blade_poly_local = blade_poly
	_convex_parts = _geometry.convex_decompose(_blade_poly_local)

# -----------------------------------------------------------------------------
# Silhouette builder (grid -> outline -> polygon)
# -----------------------------------------------------------------------------
func _build_blade_polygon_from_weapon(w: Resource) -> PackedVector2Array:
	var gw: int = 0
	var gh: int = 0
	var thresh: float = 0.5

	if w.has_method("get"):
		var gw_v: Variant = w.get("GRID_W")
		var gh_v: Variant = w.get("GRID_H")
		var th_v: Variant = w.get("THRESH")
		if gw_v != null: gw = int(gw_v)
		if gh_v != null: gh = int(gh_v)
		if th_v != null: thresh = float(th_v)

	if gw <= 0 or gh <= 0:
		return PackedVector2Array()
	if not w.has_method("get_cell"):
		return PackedVector2Array()

	var edge_map: Dictionary = {}

	for y in range(gh):
		for x in range(gw):
			var d: float = float(w.call("get_cell", x, y))
			if d < thresh:
				continue

			if y == 0 or float(w.call("get_cell", x, y - 1)) < thresh:
				_add_edge(edge_map, Vector2(float(x), float(y)), Vector2(float(x + 1), float(y)))
			if x == gw - 1 or float(w.call("get_cell", x + 1, y)) < thresh:
				_add_edge(edge_map, Vector2(float(x + 1), float(y)), Vector2(float(x + 1), float(y + 1)))
			if y == gh - 1 or float(w.call("get_cell", x, y + 1)) < thresh:
				_add_edge(edge_map, Vector2(float(x + 1), float(y + 1)), Vector2(float(x), float(y + 1)))
			if x == 0 or float(w.call("get_cell", x - 1, y)) < thresh:
				_add_edge(edge_map, Vector2(float(x), float(y + 1)), Vector2(float(x), float(y)))

	if edge_map.is_empty():
		return PackedVector2Array()

	var loops: Array = _build_loops(edge_map)
	if loops.is_empty():
		return PackedVector2Array()

	var best_index: int = 0
	var best_area_abs: float = 0.0
	for i in range(loops.size()):
		var pts_any: Variant = loops[i]
		if not (pts_any is Array):
			continue
		var pts: Array = pts_any as Array
		var area: float = _polygon_area(pts)
		var a_abs: float = absf(area)
		if a_abs > best_area_abs:
			best_area_abs = a_abs
			best_index = i

	var outer_any: Variant = loops[best_index]
	if not (outer_any is Array):
		return PackedVector2Array()
	var outer_loop: Array = outer_any as Array
	if outer_loop.size() < 3:
		return PackedVector2Array()

	var attach_x: int = 0
	var attach_y: int = 0
	var angle: float = 0.0
	if w.has_method("get"):
		var ax_v: Variant = w.get("attach_x")
		var ay_v: Variant = w.get("attach_y")
		var ang_v: Variant = w.get("handle_angle")
		if ax_v != null: attach_x = int(ax_v)
		if ay_v != null: attach_y = int(ay_v)
		if ang_v != null: angle = float(ang_v)

	var attach_center_grid: Vector2 = Vector2(float(attach_x) + 0.5, float(attach_y) + 0.5)

	var cos_a: float = cos(-angle)
	var sin_a: float = sin(-angle)

	var result: PackedVector2Array = PackedVector2Array()
	result.resize(outer_loop.size())

	for k in range(outer_loop.size()):
		var v_any: Variant = outer_loop[k]
		if not (v_any is Vector2):
			result[k] = Vector2.ZERO
			continue
		var v_corner: Vector2 = v_any as Vector2

		var v_centered: Vector2 = v_corner - attach_center_grid
		var vr: Vector2 = Vector2(
			v_centered.x * cos_a - v_centered.y * sin_a,
			v_centered.x * sin_a + v_centered.y * cos_a
		)
		result[k] = vr * cell_world_scale

	return result

func _add_edge(edge_map: Dictionary, start: Vector2, end_point: Vector2) -> void:
	var arr: Array = []
	var v: Variant = edge_map.get(start, null)
	if v is Array:
		arr = v as Array
	arr.append(end_point)
	edge_map[start] = arr

func _build_loops(edge_map: Dictionary) -> Array:
	var loops: Array = []

	# copy adjacency lists so we can consume them
	var em: Dictionary = {}
	for k_any in edge_map.keys():
		if not (k_any is Vector2):
			continue
		var k: Vector2 = k_any as Vector2
		var src_any: Variant = edge_map.get(k, null)
		var src_arr: Array = (src_any as Array) if (src_any is Array) else []
		var copy_arr: Array = []
		for i in range(src_arr.size()):
			copy_arr.append(src_arr[i])
		em[k] = copy_arr

	while not em.is_empty():
		var keys: Array = em.keys()
		var start_any: Variant = keys[0]
		if not (start_any is Vector2):
			em.erase(start_any)
			continue
		var start: Vector2 = start_any as Vector2

		var loop: Array = []
		var current: Vector2 = start
		loop.append(current)

		while true:
			if not em.has(current):
				break
			var lst_any: Variant = em.get(current, null)
			var lst: Array = (lst_any as Array) if (lst_any is Array) else []
			if lst.is_empty():
				em.erase(current)
				break

			var nxt_any: Variant = lst.pop_back()
			if lst.is_empty():
				em.erase(current)
			else:
				em[current] = lst

			if not (nxt_any is Vector2):
				break
			current = nxt_any as Vector2
			loop.append(current)

			if current == start:
				break

		if loop.size() >= 3:
			loops.append(loop)

	return loops

func _polygon_area(points: Array) -> float:
	var n: int = points.size()
	if n < 3:
		return 0.0
	var area: float = 0.0
	for i in range(n):
		var j: int = (i + 1) % n
		var p_any: Variant = points[i]
		var q_any: Variant = points[j]
		if not (p_any is Vector2 and q_any is Vector2):
			continue
		var p: Vector2 = p_any as Vector2
		var q: Vector2 = q_any as Vector2
		area += p.x * q.y - q.x * p.y
	return area * 0.5
