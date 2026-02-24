extends Control
class_name ReticleView

var kernel: Array = []

@export var fill_color: Color = Color(1, 1, 1, 0.15)
@export var outline_color: Color = Color(1, 1, 1, 0.85)

func set_kernel(k: Array) -> void:
	kernel = k
	queue_redraw()

func _draw() -> void:
	if kernel.is_empty():
		return

	var h: int = kernel.size()
	var w: int = (kernel[0] as Array).size()
	if w <= 0 or h <= 0:
		return

	var cell_w: float = size.x / float(w)
	var cell_h: float = size.y / float(h)

	# Fill
	for y in range(h):
		var row: Array = kernel[y]
		for x in range(w):
			if float(row[x]) > 0.0:
				var r := Rect2(Vector2(x * cell_w, y * cell_h), Vector2(cell_w, cell_h))
				draw_rect(r, fill_color, true)

	# Outline (4-neighbor edge)
	for y in range(h):
		for x in range(w):
			if float(kernel[y][x]) <= 0.0:
				continue
			var edge := false
			if x == 0 or y == 0 or x == w - 1 or y == h - 1:
				edge = true
			else:
				if float(kernel[y][x - 1]) <= 0.0 or float(kernel[y][x + 1]) <= 0.0 \
				or float(kernel[y - 1][x]) <= 0.0 or float(kernel[y + 1][x]) <= 0.0:
					edge = true
			if edge:
				var px := Vector2((x + 0.5) * cell_w, (y + 0.5) * cell_h)
				var rmin: float = min(cell_w, cell_h)
				draw_circle(px, rmin * 0.12, outline_color)
