extends Node
class_name WeaponGeometry

func convex_decompose(poly_in: PackedVector2Array) -> Array[PackedVector2Array]:
	var poly: PackedVector2Array = _clean_polygon(poly_in)
	if poly.size() < 3:
		return []

	var parts_any: Variant = Geometry2D.decompose_polygon_in_convex(poly)

	var out: Array[PackedVector2Array] = []
	if parts_any != null and parts_any is Array:
		var parts: Array = parts_any as Array
		for p_any: Variant in parts:
			if p_any is PackedVector2Array:
				var p: PackedVector2Array = _clean_polygon(p_any as PackedVector2Array)
				if p.size() >= 3:
					out.append(p)

	if out.is_empty():
		var pts: Array[Vector2] = []
		for v: Vector2 in poly:
			pts.append(v)
		var hull: PackedVector2Array = Geometry2D.convex_hull(pts)
		hull = _clean_polygon(hull)
		if hull.size() >= 3:
			out.append(hull)

	return out


func _clean_polygon(poly: PackedVector2Array) -> PackedVector2Array:
	var eps: float = 0.05
	var tmp: Array[Vector2] = []

	for i in range(poly.size()):
		var v: Vector2 = poly[i]
		if tmp.is_empty():
			tmp.append(v)
		else:
			if tmp[tmp.size() - 1].distance_to(v) > eps:
				tmp.append(v)

	if tmp.size() >= 2 and tmp[0].distance_to(tmp[tmp.size() - 1]) <= eps:
		tmp.pop_back()

	var changed: bool = true
	while changed and tmp.size() >= 3:
		changed = false
		var i2: int = 0
		while i2 < tmp.size() and tmp.size() >= 3:
			var a: Vector2 = tmp[(i2 - 1 + tmp.size()) % tmp.size()]
			var b: Vector2 = tmp[i2]
			var c: Vector2 = tmp[(i2 + 1) % tmp.size()]
			if _near_collinear(a, b, c, eps):
				tmp.remove_at(i2)
				changed = true
			else:
				i2 += 1

	var out := PackedVector2Array()
	for v2: Vector2 in tmp:
		out.append(v2)
	return out


func _near_collinear(a: Vector2, b: Vector2, c: Vector2, eps: float) -> bool:
	var ab: Vector2 = b - a
	var bc: Vector2 = c - b
	if ab.length_squared() < eps * eps:
		return true
	if bc.length_squared() < eps * eps:
		return true
	var cross: float = ab.x * bc.y - ab.y * bc.x
	return absf(cross) <= eps
