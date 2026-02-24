extends Node2D

@export var player1_path: NodePath
@export var player2_path: NodePath

var player1: CharacterBody2D
var player2: CharacterBody2D
var gm: GameModel = null


func _ready() -> void:
	# Get GameModel autoload
	gm = get_node_or_null("/root/GameModel") as GameModel
	if gm == null:
		push_error("DuelController: GameModel not found, returning to Hub.")
		get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")
		return

	# Ensure both players actually have weapons
	if gm.current_weapon_p1 == null or gm.current_weapon_p2 == null:
		push_error("DuelController: One or both players have no weapon selected, returning to Hub.")
		get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")
		return

	# Resolve gladiator instances
	player1 = Require.node(self, player1_path, "CharacterBody2D") as CharacterBody2D
	player2 = Require.node(self, player2_path, "CharacterBody2D") as CharacterBody2D

	if player1 == null or player2 == null:
		push_error("DuelController: Player1 or Player2 node is missing or not a CharacterBody2D, returning to Hub.")
		get_tree().change_scene_to_file("res://scenes/hub/Hub.tscn")
		return

	# Make sure the rigs know which one is P2
	# (duel_character_rig.gd defines `is_player2` as an exported var)
	player1.set("is_player2", false)
	player2.set("is_player2", true)

	# Place them on opposite sides of the arena
	_position_players()


func _position_players() -> void:
	var viewport_rect: Rect2 = get_viewport_rect()
	var size: Vector2 = viewport_rect.size

	# Horizontal placement: 15% from left/right edges
	var margin_x: float = size.x * 0.15
	var x1: float = margin_x
	var x2: float = size.x - margin_x

	# Vertical placement: mid-height of the view.
	# If you have a floor collider, they'll fall a bit until they land.
	var start_y: float = size.y * 0.66

	if player1 != null:
		player1.global_position = Vector2(x1, start_y)
	if player2 != null:
		player2.global_position = Vector2(x2, start_y)
