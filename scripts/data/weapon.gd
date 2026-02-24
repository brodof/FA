extends Resource
class_name Weapon

# Grid resolution
const GRID_W: int = 160
const GRID_H: int = 80

# Binary mode threshold for occupancy
const THRESH: float = 0.5

# Dilation/Erosion strength: how many layers edge moves per hit
const LAYERS_PER_HIT: int = 2
# 4-neighborhood = blocky, 8-neighborhood = rounder
const USE_8_NEIGHBOR: bool = false

# ---- persisted fields ----
@export var schema_version: int = 1
@export var uuid: String = ""
@export var name_: String = "Unnamed Blade"
@export var ore_type: String = "Iron"
@export var handle_length: float = 80.0
@export var saved_path: String = ""
@export var finalized: bool = false

@export var display_color: Color = Color(0.85, 0.85, 0.90, 1.0)

# handle mounting info
@export var attach_x: int = -1
@export var attach_y: int = -1
@export var handle_angle: float = 0.0  # radians

# Binary occupancy per cell: 0.0 empty, 1.0 metal
@export var density: PackedFloat32Array = PackedFloat32Array()

# Base stats (0..100)
@export var base_damage: float = 0.0
@export var base_weight: float = 0.0
@export var base_control: float = 0.0
@export var base_speed: float = 0.0
@export var base_edge: float = 0.0

# coupling for pentagon balance (tuning constants, not persisted)
const COUPLING_COEFF: float = 0.08
const DAMPING: float = 0.6
const THRESHOLD: float = 0.78

var handle_color: Color = Color(0.35, 0.22, 0.12, 1.0)

func _init() -> void:
	# This runs both for new weapons and for loaded ones;
	# for loaded ones, exported fields are overridden AFTER this.
	if uuid == "":
		uuid = _gen_uuid()

	if density.is_empty():
		density.resize(GRID_W * GRID_H)
		for i in range(GRID_W * GRID_H):
			density[i] = 0.0

# ---------- utils ----------
func _gen_uuid() -> String:
	var rng := RandomNumberGenerator.new()
	rng.seed = int(Time.get_ticks_usec())
	var hex := "0123456789abcdef"
	var parts: Array = [8, 4, 4, 4, 12]
	var s := ""
	for i in range(parts.size()):
		var n: int = parts[i]
		for _j in range(n):
			s += hex[rng.randi() % 16]
		if i < parts.size() - 1:
			s += "-"
	return s

func _idx(x: int, y: int) -> int:
	return y * GRID_W + x

func get_cell(x: int, y: int) -> float:
	if x < 0 or y < 0 or x >= GRID_W or y >= GRID_H:
		return 0.0
	return density[_idx(x, y)]

func set_cell(x: int, y: int, v: float) -> void:
	if x < 0 or y < 0 or x >= GRID_W or y >= GRID_H:
		return
	density[_idx(x, y)] = 1.0 if v >= THRESH else 0.0

# ---------- initialize starting bar ----------
func reset_bar(ore_data: OreData, ore_name: String) -> void:
	ore_type = ore_name
	display_color = ore_data.get_color(ore_name)
	var preset := ore_data.get_bar_preset(ore_name)
	var w_frac: float = float(preset["width_frac"])
	var h_frac: float = float(preset["height_frac"])

	if density.is_empty():
		density.resize(GRID_W * GRID_H)

	for i in range(GRID_W * GRID_H):
		density[i] = 0.0

	var bar_w: int = int(float(GRID_W) * w_frac)
	var bar_h: int = int(float(GRID_H) * h_frac)
	if bar_w < 2:
		bar_w = 2
	if bar_h < 2:
		bar_h = 2
	if bar_w > GRID_W - 2:
		bar_w = GRID_W - 2
	if bar_h > GRID_H - 2:
		bar_h = GRID_H - 2

	# center bar without integer-division warnings
	var x0: int = int(float(GRID_W - bar_w) / 2.0)
	var y0: int = int(float(GRID_H - bar_h) / 2.0)
	for y in range(bar_h):
		for x in range(bar_w):
			density[_idx(x0 + x, y0 + y)] = 1.0

	# reset handle mount when re-forging
	attach_x = -1
	attach_y = -1
	handle_angle = 0.0

	_recompute_stats()

# ---------- forging: local dilation/erosion (hard-edge) ----------
# center inside => local dilation under kernel
# center outside => local erosion under kernel
func apply_strike_push(center_px: Vector2, kernel: Array, strength: float) -> void:
	if finalized:
		return
	if kernel.size() == 0:
		return

	var k_h: int = kernel.size()
	var k_w: int = (kernel[0] as Array).size()
	var cx: int = int(center_px.x)
	var cy: int = int(center_px.y)
	# half extents as ints, via float division to avoid warnings
	var kx_off: int = int(float(k_w) / 2.0)
	var ky_off: int = int(float(k_h) / 2.0)

	var inside: bool = _is_inside(cx, cy)
	var iters: int = int(round(float(LAYERS_PER_HIT) * max(0.2, strength)))
	if iters < 1:
		iters = 1

	# local bbox for doing work
	var xmin: int = cx - kx_off - 2
	var xmax: int = cx + (k_w - kx_off) + 1
	var ymin: int = cy - ky_off - 2
	var ymax: int = cy + (k_h - ky_off) + 1
	if xmin < 0:
		xmin = 0
	if ymin < 0:
		ymin = 0
	if xmax > GRID_W - 1:
		xmax = GRID_W - 1
	if ymax > GRID_H - 1:
		ymax = GRID_H - 1

	for _i in range(iters):
		if inside:
			_local_dilate(xmin, ymin, xmax, ymax, cx, cy, kernel, kx_off, ky_off)
		else:
			_local_erode(xmin, ymin, xmax, ymax, cx, cy, kernel, kx_off, ky_off)

	_recompute_stats()

func _is_inside(x: int, y: int) -> bool:
	if get_cell(x, y) >= THRESH:
		return true
	if get_cell(x + 1, y) >= THRESH:
		return true
	if get_cell(x - 1, y) >= THRESH:
		return true
	if get_cell(x, y + 1) >= THRESH:
		return true
	if get_cell(x, y - 1) >= THRESH:
		return true
	return false

func _kernel_has(kernel: Array, kx: int, ky: int) -> bool:
	if ky < 0 or ky >= kernel.size():
		return false
	var row: Array = kernel[ky]
	if kx < 0 or kx >= row.size():
		return false
	return float(row[kx]) > 0.0

func _is_edge_cell(buf: PackedFloat32Array, x: int, y: int) -> bool:
	var i: int = _idx(x, y)
	if buf[i] < THRESH:
		return false
	if USE_8_NEIGHBOR:
		for yy in range(y - 1, y + 2):
			for xx in range(x - 1, x + 2):
				if xx == x and yy == y:
					continue
				if xx < 0 or yy < 0 or xx >= GRID_W or yy >= GRID_H:
					return true
				if buf[_idx(xx, yy)] < THRESH:
					return true
	else:
		if x <= 0 or buf[_idx(x - 1, y)] < THRESH:
			return true
		if x >= GRID_W - 1 or buf[_idx(x + 1, y)] < THRESH:
			return true
		if y <= 0 or buf[_idx(x, y - 1)] < THRESH:
			return true
		if y >= GRID_H - 1 or buf[_idx(x, y + 1)] < THRESH:
			return true
	return false

func _local_dilate(xmin: int, ymin: int, xmax: int, ymax: int, cx: int, cy: int, kernel: Array, kx_off: int, ky_off: int) -> void:
	var old: PackedFloat32Array = density.duplicate()
	for y in range(ymin, ymax + 1):
		for x in range(xmin, xmax + 1):
			var i: int = _idx(x, y)
			if old[i] >= THRESH:
				continue
			var kx: int = (x - cx) + kx_off
			var ky: int = (y - cy) + ky_off
			if not _kernel_has(kernel, kx, ky):
				continue
			# grow only if adjacent to current edge
			var grow: bool = false
			if USE_8_NEIGHBOR:
				for yy in range(y - 1, y + 2):
					for xx in range(x - 1, x + 2):
						if xx == x and yy == y:
							continue
						if xx < 0 or yy < 0 or xx >= GRID_W or yy >= GRID_H:
							continue
						if _is_edge_cell(old, xx, yy):
							grow = true
							break
					if grow:
						break
			else:
				if x > 0 and _is_edge_cell(old, x - 1, y):
					grow = true
				elif x < GRID_W - 1 and _is_edge_cell(old, x + 1, y):
					grow = true
				elif y > 0 and _is_edge_cell(old, x, y - 1):
					grow = true
				elif y < GRID_H - 1 and _is_edge_cell(old, x, y + 1):
					grow = true
			if grow:
				density[i] = 1.0

func _local_erode(xmin: int, ymin: int, xmax: int, ymax: int, cx: int, cy: int, kernel: Array, kx_off: int, ky_off: int) -> void:
	var old: PackedFloat32Array = density.duplicate()

	# First pass: find the closest edge cell (under the kernel) to the hammer center.
	var min_dist2: float = 1.0e20
	for y in range(ymin, ymax + 1):
		for x in range(xmin, xmax + 1):
			var i: int = _idx(x, y)
			if old[i] < THRESH:
				continue

			var kx: int = (x - cx) + kx_off
			var ky: int = (y - cy) + ky_off
			if not _kernel_has(kernel, kx, ky):
				continue
			if not _is_edge_cell(old, x, y):
				continue

			var dx: float = float(x - cx)
			var dy: float = float(y - cy)
			var d2: float = dx * dx + dy * dy
			if d2 < min_dist2:
				min_dist2 = d2

	# No eligible edge cells under kernel → nothing to erode this iteration.
	if min_dist2 >= 1.0e19:
		return

	# Only erode cells "on the side" closest to the hammer center.
	var threshold_d2: float = min_dist2 + 4.0

	for y in range(ymin, ymax + 1):
		for x in range(xmin, xmax + 1):
			var i2: int = _idx(x, y)
			if old[i2] < THRESH:
				continue

			var kx2: int = (x - cx) + kx_off
			var ky2: int = (y - cy) + ky_off
			if not _kernel_has(kernel, kx2, ky2):
				continue
			if not _is_edge_cell(old, x, y):
				continue

			var dx2: float = float(x - cx)
			var dy2: float = float(y - cy)
			var d22: float = dx2 * dx2 + dy2 * dy2
			if d22 <= threshold_d2:
				density[i2] = 0.0

# ---------- stats ----------
func _recompute_stats() -> void:
	var total: float = 0.0
	var edge_sum: float = 0.0
	var count: int = GRID_W * GRID_H

	for y in range(GRID_H):
		for x in range(GRID_W):
			var d: float = get_cell(x, y)
			total += d
			if x == 0 or y == 0 or x == GRID_W - 1 or y == GRID_H - 1:
				edge_sum += d

	var avg: float = total / float(count)
	var raw_weight: float = total / float(count)
	var raw_damage: float = avg * 1.2
	var raw_edge: float = edge_sum / float(GRID_W * 2 + GRID_H * 2)
	var compactness: float = float(GRID_H) / float(GRID_W)
	var raw_speed: float = (1.0 - raw_weight) * 0.7 + (1.0 - compactness) * 0.3
	var sym: float = _calc_symmetry()
	var raw_control: float = sym

	var stats := {
		"damage": clampf(raw_damage, 0.0, 1.0),
		"weight": clampf(raw_weight, 0.0, 1.0),
		"control": clampf(raw_control, 0.0, 1.0),
		"speed":  clampf(raw_speed,  0.0, 1.0),
		"edge":   clampf(raw_edge,   0.0, 1.0),
	}
	_apply_coupling(stats)

	base_damage = stats["damage"] * 100.0
	base_weight = stats["weight"] * 100.0
	base_control = stats["control"] * 100.0
	base_speed  = stats["speed"]  * 100.0
	base_edge   = stats["edge"]   * 100.0

func _apply_coupling(stats: Dictionary) -> void:
	var pairs := {
		"damage": ["speed", "control"],
		"weight": ["speed", "edge"],
		"control": ["edge", "damage"],
		"speed":  ["damage", "weight"],
		"edge":   ["weight", "control"],
	}
	for k in stats.keys():
		var v: float = stats[k]
		if v > THRESHOLD:
			var overflow: float = (v - THRESHOLD) * COUPLING_COEFF
			for opp in pairs[k]:
				var cur: float = float(stats[opp])
				stats[opp] = clampf(cur + overflow * DAMPING, 0.0, 1.0)
			stats[k] = clampf(v - overflow, 0.0, 1.0)

func _calc_symmetry() -> float:
	var acc: float = 0.0
	var comps: int = 0
	var half_w: int = int(float(GRID_W) / 2.0)
	for y in range(GRID_H):
		for x in range(half_w):
			var l: float = get_cell(x, y)
			var r: float = get_cell(GRID_W - 1 - x, y)
			acc += 1.0 - absf(l - r)
			comps += 1
	if comps == 0:
		return 0.0
	return acc / float(comps)

func get_effective_stats(ore_mod: Dictionary) -> Dictionary:
	var out := {
		"damage": base_damage * (1.0 + ore_mod.damage),
		"weight": base_weight * (1.0 + ore_mod.weight),
		"control": base_control * (1.0 + ore_mod.control),
		"speed":  base_speed  * (1.0 + ore_mod.speed),
		"edge":   base_edge   * (1.0 + ore_mod.edge),
	}
	for k in out.keys():
		out[k] = clampf(out[k], 0.0, 100.0)
	return out

func finalize() -> void:
	finalized = true
