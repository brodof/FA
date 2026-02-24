# res://scripts/duel/gladiator_controller/gc_debug.gd
# Debug: prefix, formatting, probes, collision sanity, pick RB, mouse drag, sensor best Y.
# Main keeps _print_debug and calls this module for helpers.
class_name GCDebug
extends RefCounted

var _owner: Node
var _refs: GCRefs

func setup(owner: Node, refs: GCRefs) -> void:
	_owner = owner
	_refs = refs

func reset() -> void:
	pass

func tick(_dt: float) -> void:
	# Intentionally unused; controller calls explicit methods.
	pass

func draw_overlay() -> void:
	var o := _owner
	if o == null:
		return
	# Exact controller logic (relocated):
	if not o.debug_draw or o._rb_pelvis == null:
		return

	var y0: float = o._rb_pelvis.global_position.y - 90.0
	var y1: float = o._rb_pelvis.global_position.y + 90.0

	# Planted anchor midpoints
	if o._plant_front_active:
		o.draw_line(o.to_local(Vector2(o._plant_front_x, y0)), o.to_local(Vector2(o._plant_front_x, y1)), Color(0.2, 0.9, 0.2, 0.7), 2.0)
	if o._plant_rear_active:
		o.draw_line(o.to_local(Vector2(o._plant_rear_x, y0)), o.to_local(Vector2(o._plant_rear_x, y1)), Color(0.2, 0.6, 1, 0.7), 2.0)

	# Neutral stance targets (from stance model)
	if is_finite(o._dbg_neutral_front_x):
		o.draw_line(o.to_local(Vector2(o._dbg_neutral_front_x, y0)), o.to_local(Vector2(o._dbg_neutral_front_x, y1)), Color(0.9, 0.9, 0.9, 0.20), 1.0)
	if is_finite(o._dbg_neutral_rear_x):
		o.draw_line(o.to_local(Vector2(o._dbg_neutral_rear_x, y0)), o.to_local(Vector2(o._dbg_neutral_rear_x, y1)), Color(0.9, 0.9, 0.9, 0.20), 1.0)

	# Phase 3 planner intent (standing-only): targets + semantic states
	if o.phase3_enable:
		var r: float = 5.0
		o.draw_circle(o.to_local(o._plan_target_front), r, Color(1.0, 1.0, 1.0, 0.22))
		o.draw_circle(o.to_local(o._plan_target_rear), r, Color(1.0, 1.0, 1.0, 0.22))

		if o._rb_foot_front != null:
			var pF: Vector2 = o._rb_foot_front.global_position + Vector2(0.0, -24.0)
			var cF := Color(0.2, 0.95, 0.2, 0.75) if (o._plan_front == GCTypes.PlanFoot.PLANTED) else Color(0.95, 0.6, 0.2, 0.75)
			o.draw_circle(o.to_local(pF), 4.0, cF)
		if o._rb_foot_rear != null:
			var pR: Vector2 = o._rb_foot_rear.global_position + Vector2(0.0, -24.0)
			var cR := Color(0.2, 0.95, 0.2, 0.75) if (o._plan_rear == GCTypes.PlanFoot.PLANTED) else Color(0.95, 0.6, 0.2, 0.75)
			o.draw_circle(o.to_local(pR), 4.0, cR)

	# Support y lines
	if is_finite(o._support_y_filt):
		o.draw_line(o.to_local(Vector2(o._rb_pelvis.global_position.x - 60.0, o._support_y_filt)),
			o.to_local(Vector2(o._rb_pelvis.global_position.x + 60.0, o._support_y_filt)),
			Color(1, 1, 1, 0.25), 1.0)
	if is_finite(o._dbg_target_y):
		o.draw_line(o.to_local(Vector2(o._rb_pelvis.global_position.x - 60.0, o._dbg_target_y)),
			o.to_local(Vector2(o._rb_pelvis.global_position.x + 60.0, o._dbg_target_y)),
			Color(1, 0.2, 0.2, 0.35), 1.0)

	# Error bar pelvis->target_y
	if is_finite(o._dbg_target_y):
		o.draw_line(o.to_local(o._rb_pelvis.global_position),
			o.to_local(Vector2(o._rb_pelvis.global_position.x, o._dbg_target_y)),
			Color(1, 0.2, 0.2, 0.45), 2.0)

	# Torso up ray
	if o._rb_torso != null:
		var up: Vector2 = Vector2.UP.rotated(o._rb_torso.global_rotation)
		o.draw_line(o.to_local(o._rb_torso.global_position),
			o.to_local(o._rb_torso.global_position + up * 25.0),
			Color(1, 1, 0.2, 0.7), 2.0)

# --- Helpers (called by main / other modules) ---
func dbg_prefix() -> String:
	var p: String = "P2" if _owner.get("is_player2") else "P1"
	return p + "#" + str(_owner.get_instance_id()) + ":" + _owner.name

func fmtf(v: float) -> String:
	if not is_finite(v):
		return "nan"
	return str(snappedf(v, 0.1))

func fmt_big(v: float) -> String:
	if not is_finite(v):
		return "nan"
	var av: float = absf(v)
	if av >= 10000.0:
		return str(snappedf(v / 1000.0, 0.1)) + "k"
	return str(snappedf(v, 1.0))

func plant_state_short(st: int) -> String:
	if st == GCTypes.FootPlantState.PLANTED:
		return "PL"
	if st == GCTypes.FootPlantState.CANDIDATE:
		return "CA"
	return "SW"

func dbg_knee_range_deg() -> Vector2:
	var kfs: int = _owner.get("knee_flex_sign")
	var neg_deg: float = _owner.get("limit_knee_neg_deg") as float
	var pos_deg: float = _owner.get("limit_knee_pos_deg") as float
	if kfs < 0:
		return Vector2(-neg_deg, pos_deg)
	return Vector2(-pos_deg, neg_deg)

func dbg_lim_tau(label: String) -> float:
	var dbg_lim: Dictionary = _owner.get("_dbg_lim") as Dictionary
	var v: Variant = dbg_lim.get(label, null)
	if typeof(v) == TYPE_DICTIONARY:
		var d: Dictionary = v
		if d.has("tau"):
			return float(d["tau"])
	return 0.0

func _bit(idx: int) -> int:
	return 1 << (maxi(1, idx) - 1)

func dbg_ground_probe(foot: RigidBody2D) -> String:
	if foot == null:
		return "probe:null"
	var probe_len: float = _owner.get("debug_world_probe_len") as float
	if not is_finite(probe_len) or probe_len <= 0.0:
		probe_len = 140.0
	var from: Vector2 = foot.global_position
	var to: Vector2 = from + Vector2(0.0, maxf(10.0, probe_len))
	var q: PhysicsRayQueryParameters2D = PhysicsRayQueryParameters2D.create(from, to)
	q.collide_with_areas = false
	q.collide_with_bodies = true
	q.collision_mask = _bit(_owner.get("L_WORLD") as int)
	var rb_list: Array = _owner.get("_rb_list") as Array
	var ex: Array[RID] = [foot.get_rid()]
	for rb in rb_list:
		if rb != null:
			ex.append((rb as RigidBody2D).get_rid())
	q.exclude = ex
	var res: Dictionary = _owner.get_world_2d().direct_space_state.intersect_ray(q)
	if res.is_empty():
		return "probe:none"
	var col: Object = res.get("collider", null)
	var pos: Vector2 = res.get("position", Vector2(NAN, NAN))
	var n: Vector2 = res.get("normal", Vector2(NAN, NAN))
	var lay: int = -1
	var msk: int = -1
	if col is CollisionObject2D:
		lay = (col as CollisionObject2D).collision_layer
		msk = (col as CollisionObject2D).collision_mask
	var A: bool = (foot.collision_mask & lay) != 0 if lay != -1 else false
	var B: bool = (msk & foot.collision_layer) != 0 if msk != -1 else false
	var cname: String = col.get_class() if col != null else "null"
	return "probe:" + cname + " lay/m=" + str(lay) + "/" + str(msk) + " ok(A/B)=" + str(int(A)) + "/" + str(int(B)) + " y=" + fmtf(pos.y) + " ny=" + fmtf(n.y)

func dbg_collision_sanity_start() -> void:
	var rb_foot_front: RigidBody2D = _refs.rb_foot_front
	var rb_foot_rear: RigidBody2D = _refs.rb_foot_rear
	if rb_foot_front == null and rb_foot_rear == null:
		return
	var prefix: String = dbg_prefix()
	var world_bit: int = _bit(_owner.get("L_WORLD") as int)
	var p1_body: int = _bit(_owner.get("L_P1_BODY") as int)
	var p2_body: int = _bit(_owner.get("L_P2_BODY") as int)
	var p1_weap: int = _bit(_owner.get("L_P1_WEAPON") as int)
	var p2_weap: int = _bit(_owner.get("L_P2_WEAPON") as int)
	var need_world_mask: int = p1_body | p2_body | p1_weap | p2_weap
	print(prefix, " COLL_SANITY bits:", " L_WORLD idx=", _owner.get("L_WORLD"), " bit=", world_bit, " P1_BODY bit=", p1_body, " P2_BODY bit=", p2_body, " P1_W bit=", p1_weap, " P2_W bit=", p2_weap, " needWorldMask=", need_world_mask)
	if rb_foot_front != null:
		print(prefix, " COLL_SANITY probeF: ", dbg_world_probe_from(rb_foot_front))
	if rb_foot_rear != null:
		print(prefix, " COLL_SANITY probeR: ", dbg_world_probe_from(rb_foot_rear))
	var root: Node = _owner.get_tree().current_scene
	if root == null:
		return
	var found_any: bool = false
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is CollisionObject2D:
			var co: CollisionObject2D = n as CollisionObject2D
			if (co.collision_layer & world_bit) != 0:
				found_any = true
				dbg_check_and_optionally_fix_world_co(prefix, co, need_world_mask)
		elif n is TileMap:
			var tm: TileMap = n as TileMap
			var lay_v: Variant = tm.get("collision_layer")
			var msk_v: Variant = tm.get("collision_mask")
			if lay_v != null and msk_v != null:
				var lay: int = int(lay_v)
				if (lay & world_bit) != 0:
					found_any = true
					dbg_check_and_optionally_fix_world_obj(prefix, tm, need_world_mask)
		for c in n.get_children():
			stack.append(c)
	if not found_any:
		print(prefix, " COLL_SANITY: No world colliders found on WORLD bit=", world_bit, " (either ground isn't on L_WORLD, or it has no collision, or it's an Area2D/sprite).")

func dbg_check_and_optionally_fix_world_co(prefix: String, co: CollisionObject2D, need_world_mask: int) -> void:
	var lay: int = co.collision_layer
	var msk: int = co.collision_mask
	var missing: int = (need_world_mask & ~msk)
	print(prefix, " WORLD_CO ", co.get_path(), " layer/mask=", lay, "/", msk, (" MISSING_MASK=" + str(missing) if missing != 0 else " OK"))
	if missing != 0 and _owner.get("debug_autofix_world_masks_on_start"):
		co.collision_mask = (msk | need_world_mask)
		print(prefix, " WORLD_CO FIXED ", co.get_path(), " newMask=", co.collision_mask)

func dbg_check_and_optionally_fix_world_obj(prefix: String, obj: Object, need_world_mask: int) -> void:
	var lay: int = int(obj.get("collision_layer"))
	var msk: int = int(obj.get("collision_mask"))
	var missing: int = (need_world_mask & ~msk)
	var pth: String = ""
	if obj is Node:
		pth = str((obj as Node).get_path())
	print(prefix, " WORLD_OBJ ", pth, " layer/mask=", lay, "/", msk, (" MISSING_MASK=" + str(missing) if missing != 0 else " OK"))
	if missing != 0 and _owner.get("debug_autofix_world_masks_on_start"):
		obj.set("collision_mask", (msk | need_world_mask))
		print(prefix, " WORLD_OBJ FIXED ", pth, " newMask=", int(obj.get("collision_mask")))

func dbg_world_probe_from(foot: RigidBody2D) -> String:
	if foot == null:
		return "null"
	var probe_len: float = _owner.get("debug_world_probe_len") as float
	var len_px: float = maxf(10.0, probe_len) if is_finite(probe_len) else 140.0
	var from: Vector2 = foot.global_position
	var to: Vector2 = from + Vector2(0.0, len_px)
	var q: PhysicsRayQueryParameters2D = PhysicsRayQueryParameters2D.create(from, to)
	q.collide_with_areas = false
	q.collide_with_bodies = true
	q.collision_mask = _bit(_owner.get("L_WORLD") as int)
	var rb_list: Array = _owner.get("_rb_list") as Array
	var ex: Array[RID] = [foot.get_rid()]
	for rb in rb_list:
		if rb != null:
			ex.append((rb as RigidBody2D).get_rid())
	q.exclude = ex
	var res: Dictionary = _owner.get_world_2d().direct_space_state.intersect_ray(q)
	if res.is_empty():
		return "hit:none"
	var col: Object = res.get("collider", null)
	var pos: Vector2 = res.get("position", Vector2(NAN, NAN))
	var n: Vector2 = res.get("normal", Vector2(NAN, NAN))
	var lay: int = -1
	var msk: int = -1
	if col != null:
		var lay_v: Variant = col.get("collision_layer")
		var msk_v: Variant = col.get("collision_mask")
		if lay_v != null: lay = int(lay_v)
		if msk_v != null: msk = int(msk_v)
	var A: bool = (foot.collision_mask & lay) != 0 if lay != -1 and msk != -1 else false
	var B: bool = (msk & foot.collision_layer) != 0 if lay != -1 and msk != -1 else false
	var cname: String = col.get_class() if col != null else "null"
	return "hit=" + cname + " lay/m=" + str(lay) + "/" + str(msk) + " ok(A/B)=" + str(int(A)) + "/" + str(int(B)) + " y=" + fmtf(pos.y) + " ny=" + fmtf(n.y)

func dbg_world_probe_y(foot: RigidBody2D, exclude: Array[RID] = []) -> float:
	if foot == null:
		return NAN
	var probe_len: float = _owner.get("debug_world_probe_len") as float
	var len_px: float = maxf(10.0, probe_len) if is_finite(probe_len) else 140.0
	var from: Vector2 = foot.global_position
	var to: Vector2 = from + Vector2(0.0, len_px)
	var q: PhysicsRayQueryParameters2D = PhysicsRayQueryParameters2D.create(from, to)
	q.collide_with_areas = false
	q.collide_with_bodies = true
	q.collision_mask = _bit(_owner.get("L_WORLD") as int)
	var ex: Array[RID] = [foot.get_rid()]
	if not exclude.is_empty():
		for rid in exclude:
			ex.append(rid)
	else:
		var rb_list: Array = _owner.get("_rb_list") as Array
		for rb in rb_list:
			if rb != null:
				ex.append((rb as RigidBody2D).get_rid())
	q.exclude = ex
	var res: Dictionary = _owner.get_world_2d().direct_space_state.intersect_ray(q)
	if res.is_empty():
		return NAN
	var pos: Vector2 = res.get("position", Vector2(NAN, NAN))
	return pos.y

func debug_pick_own_rb_at(world_pos: Vector2) -> RigidBody2D:
	var ds: PhysicsDirectSpaceState2D = _owner.get_world_2d().direct_space_state
	var q: PhysicsPointQueryParameters2D = PhysicsPointQueryParameters2D.new()
	q.position = world_pos
	q.collide_with_areas = false
	q.collide_with_bodies = true
	var body_bit: int = _bit(_owner.get("L_P2_BODY") as int) if _owner.get("is_player2") else _bit(_owner.get("L_P1_BODY") as int)
	q.collision_mask = body_bit
	var rb_list: Array = _owner.get("_rb_list") as Array
	var hits: Array = ds.intersect_point(q, 16)
	for h in hits:
		var c: Object = h.get("collider")
		if c is RigidBody2D and rb_list.has(c):
			return c as RigidBody2D
	return null

func debug_apply_mouse_drag(_dt: float) -> bool:
	_owner.set("_drag_was_applied_this_frame", false)
	if not _owner.get("debug_mouse_drag_enable") or not _owner.get("_drag_active") or _owner.get("_drag_rb") == null:
		return false
	var mp: Vector2 = _owner.get_global_mouse_position()
	var drag_offset: Vector2 = _owner.get("_drag_offset") as Vector2
	_owner.set("_drag_target_world", mp + drag_offset)
	var rb: RigidBody2D = _owner.get("_drag_rb") as RigidBody2D
	var m: float = maxf(0.01, rb.mass)
	var omega: float = maxf(0.1, _owner.get("debug_mouse_drag_omega") as float)
	var zeta: float = maxf(0.05, _owner.get("debug_mouse_drag_zeta") as float)
	var k: float = m * omega * omega
	var c: float = 2.0 * zeta * m * omega
	var target: Vector2 = _owner.get("_drag_target_world") as Vector2
	var x: Vector2 = target - rb.global_position
	var v: Vector2 = rb.linear_velocity
	var F: Vector2 = x * k - v * c
	var maxF: float = maxf(0.0, _owner.get("debug_mouse_drag_force_max") as float)
	var mag: float = F.length()
	if mag > maxF and mag > 0.0001:
		F = F * (maxF / mag)
	rb.apply_central_force(F)
	_owner.set("_drag_was_applied_this_frame", true)
	return true

func phase2_sensor_best_y(sensor: Object) -> float:
	if sensor == null:
		return NAN
	var ry: float = sensor.get("contact_y_raw") as float
	if is_finite(ry):
		return ry
	return sensor.get("contact_y") as float

func _safe_get_int(obj: Object, prop: StringName, fallback: int) -> int:
	if obj == null:
		return fallback
	var v: Variant = obj.get(prop)
	if v == null:
		return fallback
	return int(v)

# --- Print debug: full implementation (controller delegates here) ---
func print_debug(front_g: bool, rear_g: bool, stabF: float, stabR: float, grounded: bool, _run01: float) -> void:
	if not _owner.get("debug_enable"):
		return
	if _owner.get("debug_print_setup_on_start") and not _owner.get("_dbg_setup_printed"):
		_owner.set("_dbg_setup_printed", true)
		print_debug_setup()
	_run_print_debug_body(front_g, rear_g, stabF, stabR, grounded)

func _run_print_debug_body(front_g: bool, rear_g: bool, stabF: float, stabR: float, grounded: bool) -> void:
	var prefix: String = dbg_prefix()
	var rb_pelvis: RigidBody2D = _refs.rb_pelvis
	var ground_grace_t: float = _owner.get("_ground_grace_t") as float
	var grounded_eff: bool = grounded or (ground_grace_t > 0.0)
	var t: float = _owner.get("_t") as float
	var support_y_filt: float = _owner.get("_support_y_filt") as float
	var dbg_target_y: float = _owner.get("_dbg_target_y") as float
	var dbg_Fy_pre: float = _owner.get("_dbg_Fy_pre") as float
	var dbg_Fy_max: float = _owner.get("_dbg_Fy_max") as float
	var dbg_Fy_cmd: float = _owner.get("_dbg_Fy_cmd") as float
	var front_state: int = _owner.get("_front_state")
	var rear_state: int = _owner.get("_rear_state")
	var dbg_prev_grounded_eff: bool = _owner.get("_dbg_prev_grounded_eff")
	var pm_feet_base_fric: float = _owner.get("_pm_feet_base_fric") as float
	var pm_feet_slide_fric: float = _owner.get("_pm_feet_slide_fric") as float
	var dbg_front_slide: bool = _owner.get("_dbg_front_slide")
	var dbg_rear_slide: bool = _owner.get("_dbg_rear_slide")
	var touchdown: bool = grounded_eff and not dbg_prev_grounded_eff
	var liftoff: bool = (not grounded_eff) and dbg_prev_grounded_eff
	if touchdown or liftoff:
		var pel_y: float = rb_pelvis.global_position.y if rb_pelvis != null else NAN
		var pel_vy: float = rb_pelvis.linear_velocity.y if rb_pelvis != null else NAN
		var kf0: float = _owner._posture_mod.dbg_knee_rel0_deg(true) if _owner._posture_mod != null else NAN
		var kr0: float = _owner._posture_mod.dbg_knee_rel0_deg(false) if _owner._posture_mod != null else NAN
		var krng: Vector2 = dbg_knee_range_deg()
		var dbg_vsupport_called: bool = _owner.get("_dbg_vsupport_called")
		var dbg_vsupport_gate: float = _owner.get("_dbg_vsupport_gate") as float
		var gate_eff2: float = maxf(dbg_vsupport_gate, 0.0001)
		var evt_satY: float = absf(dbg_Fy_cmd) / maxf(1.0, dbg_Fy_max * gate_eff2) if dbg_vsupport_called else 0.0
		var tauKF: float = dbg_lim_tau("KNEE_F")
		var tauKR: float = dbg_lim_tau("KNEE_R")
		var tauSP: float = dbg_lim_tau("SPINE")
		var evt: String = " TD" if touchdown else " LO"
		var fric_base: float = pm_feet_base_fric if is_finite(pm_feet_base_fric) else 0.0
		var fric_slide: float = pm_feet_slide_fric if is_finite(pm_feet_slide_fric) else 0.0
		var fric_F: float = fric_slide if dbg_front_slide else fric_base
		var fric_R: float = fric_slide if dbg_rear_slide else fric_base
		print(prefix, evt, " t=", snappedf(t, 0.02), " g=", int(front_g), int(rear_g), " ge=", int(grounded_eff),
			" pm=", int(_owner.get("_planner_mode")),
		" ma=", int(_owner.get("_dbg_movement_authority_mode")),
		" stepA=", int(_owner.get("_dbg_step_planner_active")),
		" p7blk=", int(_owner.get("_dbg_legacy_phase7_planner_called")),
		" sp=", int(_owner.get("_step_phase")), " sid=", int(_owner.get("_step_id")),
		" sto=", int(_owner.get("_step_timeout_active")), " sr=", int(_owner.get("_step_recover_reason")),
		" fcm=", int(_owner.get("_dbg_foot_ctrl_mode_front")), "/", int(_owner.get("_dbg_foot_ctrl_mode_rear")),
		" as=", int(_owner.get("_dbg_arb_slide_front")), "/", int(_owner.get("_dbg_arb_slide_rear")),
		" axv=", int(_owner.get("_dbg_arb_excl_violations")),
		" br=", int(_owner.get("_spawn_brace_active")),
			" brEl=", snappedf(_owner.get("_spawn_brace_elapsed") as float, 0.02),
			" brMax/hc=", snappedf(_owner.get("spawn_brace_max_sec") as float, 0.01), "/", snappedf(_owner.get("spawn_brace_hardcap_sec") as float, 0.01),
			" imp=", snappedf(_owner.get("_impact_timer") as float, 0.02),
			" stab(F/R/min)=", snappedf(stabF, 0.02), "/", snappedf(stabR, 0.02), "/", snappedf(_owner.get("plant_min_stability") as float, 0.02),
			" slide(F/R)=", int(dbg_front_slide), "/", int(dbg_rear_slide),
			" fric(base/sl/F/R)=", snappedf(fric_base, 0.02), "/", snappedf(fric_slide, 0.02), "/", snappedf(fric_F, 0.02), "/", snappedf(fric_R, 0.02),
			" al(F/R)=", snappedf(_owner.get("_truth_front_allow_t") as float, 0.02), "/", snappedf(_owner.get("_truth_rear_allow_t") as float, 0.02),
			" y=", snappedf(pel_y, 0.1), " vy=", snappedf(pel_vy, 0.1), " sy=", fmtf(support_y_filt), " yT=", fmtf(dbg_target_y),
			" errY=", snappedf(_owner.get("_dbg_err_y") as float, 1.0),
			" sat(u/l/s)=", str(_owner.get("_dbg_phase5_upr_sat")) + "/" + str(_owner.get("_dbg_phase5_lim_sat")) + "/" + str(_owner.get("_dbg_phase5_sup_sat")),
			" st5=", snappedf(_owner.get("_phase5_spine_stable_t") as float, 0.01), " spOn=", str(_owner.get("_dbg_phase5_spine_on")),
			" sink=", snappedf(_owner.get("_dbg_sink_allow") as float, 0.1), " vGate=", snappedf(dbg_vsupport_gate, 0.02),
			" Fy=", snappedf(dbg_Fy_cmd, 1.0), " satY=", snappedf(evt_satY, 2),
			" stA=", snappedf(_refs.stance_alpha, 0.02), " cmdSy=", snappedf(_owner.get("cmd_stance_y") as float, 0.02),
			" k0(F/R)=", fmtf(kf0), "/", fmtf(kr0), " kRng=", snappedf(krng.x, 0.1), "..", snappedf(krng.y, 0.1),
			" limTau(KF/KR/SP)=", snappedf(tauKF, 1.0), "/", snappedf(tauKR, 1.0), "/", snappedf(tauSP, 1.0))
	_owner.set("_dbg_prev_grounded_eff", grounded_eff)
	if _owner.get("_dbg_landing_log_t") as float > 0.0:
		return
	var debug_print_hz: float = _owner.get("debug_print_hz") as float
	var hz: float = maxf(1.0, debug_print_hz)
	var dbg_next_sample_t: float = _owner.get("_dbg_next_sample_t") as float
	if t < dbg_next_sample_t:
		return
	_owner.set("_dbg_next_sample_t", t + (1.0 / hz))
	var pel_y2: float = rb_pelvis.global_position.y if rb_pelvis != null else NAN
	var pel_vy2: float = rb_pelvis.linear_velocity.y if rb_pelvis != null else NAN
	var kf02: float = _owner._posture_mod.dbg_knee_rel0_deg(true) if _owner._posture_mod != null else NAN
	var kr02: float = _owner._posture_mod.dbg_knee_rel0_deg(false) if _owner._posture_mod != null else NAN
	var krng2: Vector2 = dbg_knee_range_deg()
	var satY2: float = absf(dbg_Fy_pre) / maxf(1.0, dbg_Fy_max)
	var tauKF2: float = dbg_lim_tau("KNEE_F")
	var tauKR2: float = dbg_lim_tau("KNEE_R")
	var tauSP2: float = dbg_lim_tau("SPINE")
	var fric_base_s: float = pm_feet_base_fric if is_finite(pm_feet_base_fric) else 0.0
	var fric_slide_s: float = pm_feet_slide_fric if is_finite(pm_feet_slide_fric) else 0.0
	var fric_F_s: float = fric_slide_s if dbg_front_slide else fric_base_s
	var fric_R_s: float = fric_slide_s if dbg_rear_slide else fric_base_s
	var p7_lg: int = 1 if (_owner.get("_phase7_exec_left_ground") as bool) else 0
	var p7_fail: int = _owner.get("_dbg_phase7_slide_fail") as int
	var gy_dbg: float = _owner.get("_support_y_filt") as float
	var fy_dbg: float = (_refs.rb_foot_front.global_position.y if _refs.rb_foot_front != null else NAN)
	var ry_dbg: float = (_refs.rb_foot_rear.global_position.y if _refs.rb_foot_rear != null else NAN)
	var clrF_dbg: float = (gy_dbg - fy_dbg) if is_finite(gy_dbg) and is_finite(fy_dbg) else NAN
	var clrR_dbg: float = (gy_dbg - ry_dbg) if is_finite(gy_dbg) and is_finite(ry_dbg) else NAN
	print(prefix, " S", " t=", snappedf(t, 0.02), " g=", int(front_g), int(rear_g), " ge=", int(grounded_eff),
		" pm=", int(_owner.get("_planner_mode")),
		" ma=", int(_owner.get("_dbg_movement_authority_mode")),
		" stepA=", int(_owner.get("_dbg_step_planner_active")),
		" p7blk=", int(_owner.get("_dbg_legacy_phase7_planner_called")),
		" sp=", int(_owner.get("_step_phase")), " sid=", int(_owner.get("_step_id")),
		" sto=", int(_owner.get("_step_timeout_active")), " sr=", int(_owner.get("_step_recover_reason")),
		" ss=", int(_owner.get("_step_support_is_front")), "/", int(_owner.get("_step_has_swing")), "/", int(_owner.get("_step_swing_is_front")),
		" fcm=", int(_owner.get("_dbg_foot_ctrl_mode_front")), "/", int(_owner.get("_dbg_foot_ctrl_mode_rear")),
		" as=", int(_owner.get("_dbg_arb_slide_front")), "/", int(_owner.get("_dbg_arb_slide_rear")),
		" aOwn=", int(_owner.get("_dbg_arb_support_owner_step")),
		" axv=", int(_owner.get("_dbg_arb_excl_violations")),
		" aBlk=", int(_owner.get("_dbg_arb_phase7_helper_blocked")),
		" sFx=", int(_owner.get("_dbg_arb_swing_force_front")), "/", int(_owner.get("_dbg_arb_swing_force_rear")),
		" sFb=", int(_owner.get("_dbg_arb_swing_force_blocked")),
		" lG=", int(_owner.get("_dbg_land_gate_front")), "/", int(_owner.get("_dbg_land_gate_rear")),
		" lDx=", fmtf(_owner.get("_dbg_land_target_dx_front") as float), "/", fmtf(_owner.get("_dbg_land_target_dx_rear") as float),
		" aSh=", int(_owner.get("_dbg_arb_shape_dual")), " n=", int(_owner.get("_dbg_arb_shape_need_front")), "/", int(_owner.get("_dbg_arb_shape_need_rear")),
		" sErr=", fmtf(_owner.get("_dbg_step_slot_err_front") as float), "/", fmtf(_owner.get("_dbg_step_slot_err_rear") as float),
		" sSep=", fmtf(_owner.get("_dbg_step_slot_sep") as float),
		" br=", int(_owner.get("_spawn_brace_active")),
		" p7=", str(_owner.get("_dbg_phase7_state")), " p7e=", str(_owner.get("_dbg_phase7_exec_phase")),
		" p7a=", str(_owner.get("_dbg_phase7_active_is_front")), " p7i=", str(_owner.get("_dbg_phase7_seq_index")),
		" p7dw=", str(_owner.get("_dbg_phase7_dir_w")), " p7df=", str(_owner.get("_dbg_phase7_dir_f")),
		" p7tF=", str(snappedf(_owner.get("_dbg_phase7_target_fx") as float, 0.1)), " p7tR=", str(snappedf(_owner.get("_dbg_phase7_target_rx") as float, 0.1)),
		" p7own=", str(_owner.get("_dbg_phase7_own")), " p7fx=", str(int(_owner.get("_phase7_exec_state_front"))), "/", str(int(_owner.get("_phase7_exec_state_rear"))), " pl=", str(int(_owner.get("_plan_front"))), "/", str(int(_owner.get("_plan_rear"))),
		" fp=", str(int(_owner.get("_phase7_force_plant_front"))), "/", str(int(_owner.get("_phase7_force_plant_rear"))),
		" st=", str(front_state), "/", str(rear_state),
		" plant=", str(int(_owner._foot_plant_mod.plant_forces_enabled_for_foot(true) if _owner._foot_plant_mod != null else false)), "/", str(int(_owner._foot_plant_mod.plant_forces_enabled_for_foot(false) if _owner._foot_plant_mod != null else false)),
		" p7sl=", str(_owner.get("_dbg_phase7_slideF")), "/", str(_owner.get("_dbg_phase7_slideR")),
		" p7lg=", str(p7_lg), " p7fail=", str(p7_fail),
		" p7clr(F/R)=", fmtf(clrF_dbg), "/", fmtf(clrR_dbg),
		" p7dvx0=", str(_owner.get("_dbg_phase7_dvx_clamped")), " p7cl=", str(_owner.get("_phase7_clamp_cross_count")), "/", str(_owner.get("_phase7_clamp_reach_count")),
		" dw(step/tgt/root)=", str(_owner.get("_dbg_dual_writer_step_phase")), "/", str(_owner.get("_dbg_dual_writer_swing_target")), "/", str(_owner.get("_dbg_root_progress_blocked_by_foot")),
		" brEl=", snappedf(_owner.get("_spawn_brace_elapsed") as float, 0.02),
		" brMax/hc=", snappedf(_owner.get("spawn_brace_max_sec") as float, 0.01), "/", snappedf(_owner.get("spawn_brace_hardcap_sec") as float, 0.01),
		" imp=", snappedf(_owner.get("_impact_timer") as float, 0.02),
		" stab(F/R/min)=", snappedf(stabF, 0.02), "/", snappedf(stabR, 0.02), "/", snappedf(_owner.get("plant_min_stability") as float, 0.02),
		" slide(F/R)=", int(dbg_front_slide), "/", int(dbg_rear_slide),
		" fric(base/sl/F/R)=", snappedf(fric_base_s, 0.02), "/", snappedf(fric_slide_s, 0.02), "/", snappedf(fric_F_s, 0.02), "/", snappedf(fric_R_s, 0.02),
		" al(F/R)=", snappedf(_owner.get("_truth_front_allow_t") as float, 0.02), "/", snappedf(_owner.get("_truth_rear_allow_t") as float, 0.02),
		" y=", snappedf(pel_y2, 0.1), " vy=", snappedf(pel_vy2, 0.1), " stA=", snappedf(_refs.stance_alpha, 0.02),
		" cmdSy=", snappedf(_owner.get("cmd_stance_y") as float, 0.02), " yT=", fmtf(dbg_target_y),
		" Fy=", snappedf(dbg_Fy_cmd, 1.0), " satY=", snappedf(satY2, 2),
		" k0(F/R)=", fmtf(kf02), "/", fmtf(kr02), " kRng=", snappedf(krng2.x, 0.1), "..", snappedf(krng2.y, 0.1),
		" limTau(KF/KR/SP)=", snappedf(tauKF2, 1.0), "/", snappedf(tauKR2, 1.0), "/", snappedf(tauSP2, 1.0),
		" dxE=", fmtf(_owner.get("_dbg_dx_to_enemy") as float), " op=", int(_refs.opponent_pelvis != null),
		" fs=", str(_owner.get("facing_sign")), " cmdX=", snappedf(_owner.get("cmd_move_x") as float, 0.02),
		" fwd=", snappedf(_owner.get("cmd_forward01") as float, 0.02), " back=", snappedf(_owner.get("cmd_backward01") as float, 0.02),
		" | stance: anchor(F/R)=", snappedf(_owner.get("_plant_front_x") as float, 0.1), "/", snappedf(_owner.get("_plant_rear_x") as float, 0.1),
		" footRb(F/R)=", fmtf(_owner.get("_dbg_foot_x_f") as float), "/", fmtf(_owner.get("_dbg_foot_x_r") as float),
		" want(F/R)=", fmtf(_owner.get("_dbg_recenter_want_f") as float), "/", fmtf(_owner.get("_dbg_recenter_want_r") as float),
		" skip(F/R)=", str(_owner.get("_dbg_recenter_skip_f")), "/", str(_owner.get("_dbg_recenter_skip_r")),
		" dx(F/R)=", fmtf(_owner.get("_dbg_recenter_dx_f") as float), "/", fmtf(_owner.get("_dbg_recenter_dx_r") as float),
		" | footRot F: cur=", fmtf(_owner.get("_dbg_foot_rot_deg_f") as float), " tgt=", fmtf(_owner.get("_dbg_foot_flat_tgt_deg_f") as float), " err=", fmtf(_owner.get("_dbg_foot_flat_err_deg_f") as float), " av=", fmtf(_owner.get("_dbg_foot_rot_av_f") as float), " n=", fmtf(_owner.get("_dbg_foot_flat_nx_f") as float), ",", fmtf(_owner.get("_dbg_foot_flat_ny_f") as float),
		" R: cur=", fmtf(_owner.get("_dbg_foot_rot_deg_r") as float), " tgt=", fmtf(_owner.get("_dbg_foot_flat_tgt_deg_r") as float), " err=", fmtf(_owner.get("_dbg_foot_flat_err_deg_r") as float), " av=", fmtf(_owner.get("_dbg_foot_rot_av_r") as float), " n=", fmtf(_owner.get("_dbg_foot_flat_nx_r") as float), ",", fmtf(_owner.get("_dbg_foot_flat_ny_r") as float))

func print_debug_setup() -> void:
	var prefix: String = dbg_prefix()
	var missing: String = ""
	if _refs.rb_pelvis == null:
		missing += " pelvis"
	if _refs.rb_torso == null:
		missing += " torso"
	if _refs.rb_foot_front == null:
		missing += " footF"
	if _refs.rb_foot_rear == null:
		missing += " footR"
	if _refs.foot_sensor_front == null:
		missing += " sensF"
	if _refs.foot_sensor_rear == null:
		missing += " sensR"
	print(prefix, " SETUP", " grav=", snappedf(_owner.get("_g") as float, 0.1), " mass=", snappedf(_owner.get("_total_mass") as float, 0.1),
		" spawnRamp=", snappedf(_owner.get("spawn_ramp_sec") as float, 0.01), " phase1b=", int(_owner.get("phase1b_stand_only")),
		" footMat(fric/bounce)=", snappedf(_owner.get("feet_friction") as float, 0.01), "/", snappedf(_owner.get("feet_bounce") as float, 0.01),
		" damp(pel/tor/footAir)=", snappedf(_owner.get("pelvis_linear_damp") as float, 0.01), "/", snappedf(_owner.get("torso_linear_damp") as float, 0.01), "/", snappedf(_owner.get("foot_linear_damp_air") as float, 0.01),
		(" missing:" + missing if missing != "" else ""))
	var krng: Vector2 = dbg_knee_range_deg()
	print(prefix, " LIMITS kneeRange(deg)=", snappedf(krng.x, 0.1), "..", snappedf(krng.y, 0.1),
		" kneeSign=", _owner.get("knee_flex_sign"),
		" kneeZeroF/R(deg)=", snappedf(rad_to_deg(_owner.get("_knee_zero_F") as float), 0.1), "/", snappedf(rad_to_deg(_owner.get("_knee_zero_R") as float), 0.1),
		" kneeZeroValid=", int(_owner.get("_knee_zero_valid")))
	if not _owner.get("debug_include_collision_setup"):
		return
	if _refs.rb_pelvis != null:
		var pel: RigidBody2D = _refs.rb_pelvis
		var tor: RigidBody2D = _refs.rb_torso
		var footF: RigidBody2D = _refs.rb_foot_front
		var footR: RigidBody2D = _refs.rb_foot_rear
		print(prefix, " COLL", " pelvis layer/mask=", pel.collision_layer, "/", pel.collision_mask,
			" torso layer/mask=", (tor.collision_layer if tor != null else -1), "/", (tor.collision_mask if tor != null else -1),
			" footF layer/mask=", (footF.collision_layer if footF != null else -1), "/", (footF.collision_mask if footF != null else -1),
			" footR layer/mask=", (footR.collision_layer if footR != null else -1), "/", (footR.collision_mask if footR != null else -1),
			" CB layer/mask=", _owner.get("collision_layer"), "/", _owner.get("collision_mask"),
			" CB disabled=", int(_owner.get("disable_characterbody_collisions")))
