extends Node2D

@export var inv_scroll_path: NodePath
@export var inv_list_path: NodePath
@export var slot_p1_path: NodePath
@export var slot_p2_path: NodePath
@export var start_button_path: NodePath
@export var forge_button_path: NodePath
@export var quick_start_button_path: NodePath

var inv_scroll: ScrollContainer
var inv_list: VBoxContainer
var slot_p1: SlotPanel
var slot_p2: SlotPanel
var start_button: Button
var forge_button: Button
var quick_start_button: Button

func _ready() -> void:
	var req := Require

	inv_scroll        = req.node(self, inv_scroll_path, "ScrollContainer") as ScrollContainer
	inv_list          = req.node(self, inv_list_path, "VBoxContainer") as VBoxContainer

	var raw_p1: Panel = req.node(self, slot_p1_path, "Panel") as Panel
	var raw_p2: Panel = req.node(self, slot_p2_path, "Panel") as Panel
	slot_p1 = raw_p1 as SlotPanel
	slot_p2 = raw_p2 as SlotPanel

	if slot_p1 == null or slot_p2 == null:
		push_error("Hub: Slot panels missing or wrong script.")
		return

	start_button      = req.node(self, start_button_path, "Button") as Button
	forge_button      = req.node(self, forge_button_path, "Button") as Button
	quick_start_button = req.node(self, quick_start_button_path, "Button") as Button

	slot_p1.weapon_assigned.connect(_on_slot_assigned)
	slot_p2.weapon_assigned.connect(_on_slot_assigned)

	start_button.pressed.connect(_on_start_pressed)
	forge_button.pressed.connect(_on_forge_pressed)
	quick_start_button.pressed.connect(_on_quick_start_pressed)

	_populate_inventory()
	_refresh_start_state()

func _populate_inventory() -> void:
	for c in inv_list.get_children():
		c.queue_free()

	var gm: GameModel = get_node("/root/GameModel") as GameModel
	for w in gm.inventory:
		var row: WeaponRow = WeaponRow.new()
		row.setup(w)
		inv_list.add_child(row)

func _on_slot_assigned(which: int, uuid: String) -> void:
	var gm: GameModel = get_node("/root/GameModel") as GameModel
	var w: Weapon = gm.find_weapon_by_uuid(uuid)
	if w == null:
		return

	if which == 1:
		gm.current_weapon_p1 = w
	else:
		gm.current_weapon_p2 = w

	_refresh_start_state()

func _refresh_start_state() -> void:
	var gm: GameModel = get_node("/root/GameModel") as GameModel
	start_button.disabled = (gm.current_weapon_p1 == null or gm.current_weapon_p2 == null)

func _on_start_pressed() -> void:
	get_tree().change_scene_to_file("res://scenes/duel/Duel.tscn")

func _on_forge_pressed() -> void:
	get_tree().change_scene_to_file("res://scenes/forge/Forge.tscn")

func _on_quick_start_pressed() -> void:
	var gm: GameModel = get_node("/root/GameModel") as GameModel
	if gm == null:
		push_error("Hub: GameModel missing.")
		return

	var inv: Array = gm.inventory
	if inv.size() == 0:
		push_error("Hub: No weapons in inventory.")
		return

	var w1: Weapon = inv[0]
	var w2: Weapon = null

	if inv.size() >= 2:
		w2 = inv[1]
	else:
		w2 = w1
	gm.current_weapon_p1 = w1
	gm.current_weapon_p2 = w2

	slot_p1.current_weapon_uuid = w1.uuid
	slot_p1._update_visual()
 
	slot_p2.current_weapon_uuid = w2.uuid
	slot_p2._update_visual()

	_refresh_start_state()

	get_tree().change_scene_to_file("res://scenes/duel/Duel.tscn")
