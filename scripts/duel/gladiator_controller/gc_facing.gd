# res://scripts/duel/gladiator_controller/gc_facing.gd
# Facing: bind opponent pelvis, update facing_sign from opponent X. Writes refs.opponent_pelvis, owner.facing_sign.
class_name GCFacing
extends RefCounted

var _owner: Node
var _refs: GCRefs

func setup(owner: Node, refs: GCRefs) -> void:
	_owner = owner
	_refs = refs

func reset() -> void:
	pass

func tick(_dt: float) -> void:
	_ensure_opponent_pelvis_bound()
	_update_facing_from_opponent()

func _ensure_opponent_pelvis_bound() -> void:
	if _refs.opponent_pelvis != null:
		return
	var path: NodePath = _owner.get("opponent_rb_pelvis_path") as NodePath
	if path != NodePath(""):
		var rb: RigidBody2D = _owner.get_node_or_null(path) as RigidBody2D
		if rb != null:
			_refs.opponent_pelvis = rb
			if _owner.get("debug_enable") and not _owner.get("_dbg_opp_bound_printed"):
				print(_owner._debug_mod.dbg_prefix() if _owner._debug_mod != null else "", " OPP_BOUND src=path node=", rb.get_path())
				_owner.set("_dbg_opp_bound_printed", true)
			return
	var parent: Node = _owner.get_parent()
	if parent == null:
		return
	var best: Node = null
	for c in parent.get_children():
		if c == _owner:
			continue
		if c.get_script() != _owner.get_script():
			continue
		var v: Variant = c.get("is_player2")
		if typeof(v) == TYPE_BOOL and bool(v) != _owner.get("is_player2"):
			best = c
			break
		if best == null:
			best = c
	if best == null:
		return
	var np: NodePath = NodePath("Ragdoll/RB_Pelvis")
	var vnp: Variant = best.get("rb_pelvis_path")
	if typeof(vnp) == TYPE_NODE_PATH:
		np = vnp
	var rb2: RigidBody2D = best.get_node_or_null(np) as RigidBody2D
	if rb2 != null:
		_refs.opponent_pelvis = rb2
		if _owner.get("debug_enable") and not _owner.get("_dbg_opp_bound_printed"):
			print(_owner._debug_mod.dbg_prefix() if _owner._debug_mod != null else "", " OPP_BOUND src=sibling node=", rb2.get_path())
			_owner.set("_dbg_opp_bound_printed", true)

func _update_facing_from_opponent() -> void:
	_owner.set("_dbg_dx_to_enemy", NAN)
	if _refs.rb_pelvis == null or _refs.opponent_pelvis == null:
		return
	var dx: float = _refs.opponent_pelvis.global_position.x - _refs.rb_pelvis.global_position.x
	_owner.set("_dbg_dx_to_enemy", dx)
	var deadzone: float = _owner.get("facing_deadzone_x") as float
	if absf(dx) < deadzone:
		return
	_owner.set("facing_sign", 1 if dx > 0.0 else -1)
