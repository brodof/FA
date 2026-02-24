# res://scripts/duel/gladiator_controller/gc_input.gd
# Input: read actions, optional mouse-drag debug. Writes owner.cmd_move_x, owner.cmd_stance_y.
class_name GCInput
extends RefCounted

var _owner: Node
var _refs: GCRefs

func setup(owner: Node, refs: GCRefs) -> void:
	_owner = owner
	_refs = refs

func reset() -> void:
	pass

func tick(_dt: float) -> void:
	_read_input_actions()

func handle_input(event: InputEvent) -> void:
	if not _owner.get("debug_mouse_drag_enable") or not _owner.get("_init_done") or not _owner.get("enable_controller"):
		return
	if event is InputEventMouseButton and event.button_index == _owner.get("debug_mouse_drag_button"):
		if event.pressed:
			var mp: Vector2 = _owner.get_global_mouse_position()
			var picked: RigidBody2D = _owner._debug_mod.debug_pick_own_rb_at(mp) if _owner._debug_mod != null else null
			if picked != null:
				_owner.set("_drag_active", true)
				var rb_pelvis: RigidBody2D = _refs.rb_pelvis
				var pelvis_only: bool = _owner.get("debug_mouse_drag_pelvis_only")
				_owner.set("_drag_rb", rb_pelvis if (pelvis_only and rb_pelvis != null) else picked)
				_owner.set("_drag_offset", _owner.get("_drag_rb").global_position - mp)
		else:
			_owner.set("_drag_active", false)
			_owner.set("_drag_rb", null)

func _read_input_actions() -> void:
	if not _owner.get("input_actions_enable"):
		return
	var world_x: float = Input.get_action_strength(_owner.get("action_move_right")) - Input.get_action_strength(_owner.get("action_move_left"))
	world_x = clampf(world_x, -1.0, 1.0)
	_owner.set("cmd_move_x", world_x)
	var sy: float = Input.get_action_strength(_owner.get("action_stance_up")) - Input.get_action_strength(_owner.get("action_stance_down"))
	sy = clampf(sy, -1.0, 1.0)
	_owner.set("cmd_stance_y", sy)
