extends Panel
class_name SlotPanel

@export var which: int = 1   # 1 = P1, 2 = P2

signal weapon_assigned(which: int, uuid: String)

var current_weapon_uuid: String = ""

func _ready() -> void:
	mouse_filter = MOUSE_FILTER_STOP
	_update_visual()

func _can_drop_data(_at_position: Vector2, data: Variant) -> bool:
	if typeof(data) != TYPE_DICTIONARY:
		return false

	var d: Dictionary = data
	if not d.has("type") or not d.has("uuid"):
		return false

	if String(d["type"]) != "weapon":
		return false

	return true

func _drop_data(_at_position: Vector2, data: Variant) -> void:
	if not _can_drop_data(_at_position, data):
		return

	var d: Dictionary = data
	current_weapon_uuid = String(d["uuid"])

	weapon_assigned.emit(which, current_weapon_uuid)
	_update_visual()

func _update_visual() -> void:
	var gm_node: Node = get_node_or_null("/root/GameModel")
	var w: Weapon = null

	if gm_node != null and current_weapon_uuid != "":
		w = gm_node.find_weapon_by_uuid(current_weapon_uuid)

	# background highlight
	if current_weapon_uuid != "":
		self.modulate = Color(0.85, 1.0, 0.85, 1.0)
	else:
		self.modulate = Color(1.0, 1.0, 1.0, 1.0)

	# tooltip
	var tip: String = ""
	if w != null:
		tip = "P%d: %s" % [which, w.name_]
	tooltip_text = tip

	_update_thumbnail(w)
	_update_details_label(w)

func _update_thumbnail(w: Weapon) -> void:
	var thumb: BladeView = get_node_or_null("Thumb") as BladeView

	if w != null:
		if thumb == null:
			thumb = BladeView.new()
			thumb.name = "Thumb"
			# occupy most of the panel, leave some space at bottom for details text
			thumb.anchor_left = 0.0
			thumb.anchor_top = 0.0
			thumb.anchor_right = 1.0
			thumb.anchor_bottom = 0.7
			thumb.offset_left = 0.0
			thumb.offset_top = 0.0
			thumb.offset_right = 0.0
			thumb.offset_bottom = 0.0
			thumb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			thumb.size_flags_vertical = Control.SIZE_EXPAND_FILL
			add_child(thumb)

		thumb.blade_scale = 0.6
		thumb.set_weapon(w)

		if w.attach_x >= 0 and w.attach_y >= 0:
			var attach_cell: Vector2i = Vector2i(w.attach_x, w.attach_y)
			thumb.set_attach_cell(attach_cell)

			var ang: float = w.handle_angle
			if ang == 0.0:
				ang = PI * 0.5

			var handle_len: float = w.handle_length
			if handle_len <= 0.0:
				handle_len = 80.0

			thumb.set_handle_visual(attach_cell, ang, handle_len)
		else:
			thumb.set_attach_cell(Vector2i(-1, -1))
	else:
		if thumb != null:
			thumb.queue_free()

func _update_details_label(w: Weapon) -> void:
	var details: Label = get_node_or_null("DetailsLabel") as Label
	if w == null:
		if details != null:
			details.text = "P%d: —" % which
		return

	if details == null:
		details = Label.new()
		details.name = "DetailsLabel"
		details.autowrap_mode = TextServer.AUTOWRAP_WORD
		details.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		details.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
		# bottom part of the panel
		details.anchor_left = 0.0
		details.anchor_top = 0.7
		details.anchor_right = 1.0
		details.anchor_bottom = 1.0
		details.offset_left = 4.0
		details.offset_top = 0.0
		details.offset_right = -4.0
		details.offset_bottom = -4.0
		details.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		details.size_flags_vertical = Control.SIZE_EXPAND_FILL
		add_child(details)

	var ore_data := OreData.new()
	var mods: Dictionary = ore_data.get_modifiers(w.ore_type)
	var st: Dictionary = w.get_effective_stats(mods)

	details.text = "P%d: %s\nD%.0f W%.0f C%.0f S%.0f E%.0f" % [
		which,
		w.name_,
		st["damage"],
		st["weight"],
		st["control"],
		st["speed"],
		st["edge"]
	]
