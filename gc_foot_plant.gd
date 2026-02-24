# res://scripts/duel/gladiator_controller/gc_foot_plant.gd
# Foot plant state machine (SWING -> CANDIDATE -> PLANTED). Writes owner _front_state, _rear_state, _plant_*; calls owner for latch/recenter/glue.
class_name GCFootPlant
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

func apply_planted_foot_flatness(is_front: bool, strength: float) -> void:
	var o := _owner
	if o == null:
		return
	# Clear flatness debug for this foot; set to NAN if we early-return so "no flatness" is visible.
	if is_front:
		o._dbg_foot_flat_tgt_deg_f = NAN
		o._dbg_foot_flat_err_deg_f = NAN
		o._dbg_foot_flat_nx_f = NAN
		o._dbg_foot_flat_ny_f = NAN
	else:
		o._dbg_foot_flat_tgt_deg_r = NAN
		o._dbg_foot_flat_err_deg_r = NAN
		o._dbg_foot_flat_nx_r = NAN
		o._dbg_foot_flat_ny_r = NAN
	# Exact controller logic (relocated):
	if not o.enable_foot_flatness:
		return
	if strength <= 0.0:
		return

	var foot: RigidBody2D = o._rb_foot_front if is_front else o._rb_foot_rear
	var sensor: FootContactSensor = o._foot_sensor_front if is_front else o._foot_sensor_rear
	if foot == null or sensor == null:
		return
	if not sensor.grounded:
		return

	# Only operate when this foot is truly planted (pins-connected truth).
	var planted: bool = o._plant_front_active if is_front else o._plant_rear_active
	if not planted:
		if is_front:
			o._foot_flat_n_prev_f = Vector2.ZERO
		else:
			o._foot_flat_n_prev_r = Vector2.ZERO
		return

	var loc_flat: Variant = o.get("_locomotion_mod")
	if loc_flat != null and o.get("phase7_enable"):
		if not loc_flat.phase7_allow_planted_flatness(is_front):
			return

	if o.crouch_trail_toe_only_enable:
		var is_tail: bool = _is_tail_foot(is_front)
		if is_tail and (o._stance_alpha <= o.crouch_trail_toe_only_alpha):
			return

	# Blend in with plant strength so it doesn't weld at touchdown.
	var plant01: float = o._front_plant_blend if is_front else o._rear_plant_blend
	plant01 = clampf(plant01, 0.0, 1.0)
	if plant01 <= 0.001:
		return

	var n: Vector2 = sensor.contact_normal_w
	if not is_finite(n.x) or not is_finite(n.y) or n.length() < 0.2:
		n = Vector2.UP
	n = n.normalized()

	# Reject insane normals; fall back to world-up.
	var up_dot: float = clampf(n.dot(Vector2.UP), -1.0, 1.0)
	var slope_deg: float = rad_to_deg(acos(up_dot))
	if slope_deg > o.foot_flat_max_slope_deg:
		n = Vector2.UP

	# Optional smoothing to reduce jitter from flickering contact normals.
	var smooth_alpha: float = o.get("foot_flat_normal_smooth_alpha") as float
	if is_finite(smooth_alpha) and smooth_alpha > 0.0 and smooth_alpha < 1.0:
		var prev: Vector2 = o._foot_flat_n_prev_f if is_front else o._foot_flat_n_prev_r
		if prev.length() >= 0.2:
			n = prev.lerp(n, smooth_alpha).normalized()
		if is_front:
			o._foot_flat_n_prev_f = n
		else:
			o._foot_flat_n_prev_r = n

	# Tangent angle => sole parallel to ground. Pick the flat angle closest to current foot angle to avoid 180° flip.
	var t1: Vector2 = Vector2(n.y, -n.x).normalized()
	var ang1: float = atan2(t1.y, t1.x)
	var ang2: float = wrapf(ang1 + PI, -PI, PI)
	var cur: float = wrapf(foot.global_rotation, -PI, PI)
	var d1: float = absf(wrapf(ang1 - cur, -PI, PI))
	var d2: float = absf(wrapf(ang2 - cur, -PI, PI))
	var target_ang: float = ang1 if d1 <= d2 else ang2

	# omega/zeta PD in angular space
	var m: float = maxf(0.01, foot.mass)
	var omega: float = maxf(0.1, o.foot_flat_omega)
	var zeta: float = maxf(0.05, o.foot_flat_zeta)
	var k: float = m * omega * omega
	var d: float = 2.0 * zeta * m * omega

	# Store flat angle for swing-foot attitude when this foot leaves ground (Phase 7).
	if is_front:
		o._swing_flat_ref_angle_f = target_ang
	else:
		o._swing_flat_ref_angle_r = target_ang

	# Debug: why feet rotate — target, error, contact normal when flatness is applied
	var err_rad: float = wrapf(target_ang - cur, -PI, PI)
	if is_front:
		o._dbg_foot_flat_tgt_deg_f = rad_to_deg(target_ang)
		o._dbg_foot_flat_err_deg_f = rad_to_deg(err_rad)
		o._dbg_foot_flat_nx_f = n.x
		o._dbg_foot_flat_ny_f = n.y
	else:
		o._dbg_foot_flat_tgt_deg_r = rad_to_deg(target_ang)
		o._dbg_foot_flat_err_deg_r = rad_to_deg(err_rad)
		o._dbg_foot_flat_nx_r = n.x
		o._dbg_foot_flat_ny_r = n.y

	if o._posture_mod != null:
		o._posture_mod.apply_angle_pd(foot, target_ang, k, d, o.foot_flat_tau_max, strength * plant01)

# 5R2-G
# Unified foot attitude torque owner for STEP_PLANNER mode.
# This is the ONLY foot-orientation controller used in STEP_PLANNER mode:
# - planted feet: ground-parallel using contact normal
# - swing/air feet: cached flat reference (captured from last valid ground-parallel pose)
# No toe/heel "extra" torque terms here (pure angle PD only).
func apply_parallel_foot_attitude_by_mode(is_front: bool, ctrl_mode: int, strength_mul: float, attitude_mode: int = -1) -> void:
	var o := _owner
	if o == null or o._posture_mod == null:
		return

	var foot: RigidBody2D = _refs.rb_foot_front if is_front else _refs.rb_foot_rear
	var sensor: Node = _refs.foot_sensor_front if is_front else _refs.foot_sensor_rear
	if foot == null or sensor == null:
		return

	# Brief A: attitude torque ownership is separate from foot control ownership.
	# Legacy callers may omit attitude_mode (=-1), in which case we derive it from ctrl_mode.
	var eff_att_mode: int = attitude_mode
	if eff_att_mode < 0:
		match ctrl_mode:
			GCTypes.FootControlMode.PLANTED_SUPPORT, GCTypes.FootControlMode.PLANTED_CORRECT, GCTypes.FootControlMode.RECOVERY:
				eff_att_mode = GCTypes.FootAttitudeMode.GROUND_PARALLEL
			GCTypes.FootControlMode.SWING:
				eff_att_mode = GCTypes.FootAttitudeMode.AIR_PARALLEL
			_:
				eff_att_mode = GCTypes.FootAttitudeMode.DISABLED

	var _dbg_owner_mode_id: int = eff_att_mode

	if eff_att_mode == GCTypes.FootAttitudeMode.DISABLED:
		if is_front:
			o._dbg_foot_flat_tgt_deg_f = NAN
			o._dbg_foot_flat_err_deg_f = NAN
			o._dbg_foot_flat_nx_f = NAN
			o._dbg_foot_flat_ny_f = NAN
			o._dbg_foot_att_owner_f = 0
		else:
			o._dbg_foot_flat_tgt_deg_r = NAN
			o._dbg_foot_flat_err_deg_r = NAN
			o._dbg_foot_flat_nx_r = NAN
			o._dbg_foot_flat_ny_r = NAN
			o._dbg_foot_att_owner_r = 0
		return

	var c: Dictionary = _active_ground_contact(sensor, foot)
	var has_contact_normal: bool = c.has("normal")

	var ground_ref_ang: float = NAN
	var n_dbg: Vector2 = Vector2(NAN, NAN)
	if has_contact_normal:
		var n: Vector2 = (c["normal"] as Vector2)
		n_dbg = n
		if n.length_squared() > 0.0:
			n = n.normalized()

			# Only trust slope-valid ground contacts for GROUND_PARALLEL targeting.
			var max_slope_deg: float = 90.0
			if _owner != null:
				var raw_slope = _owner.get("plant_ground_max_slope_deg")
				if typeof(raw_slope) == TYPE_FLOAT or typeof(raw_slope) == TYPE_INT:
					max_slope_deg = float(raw_slope)
			var slope_deg: float = rad_to_deg(acos(clampf(n.dot(Vector2.UP), -1.0, 1.0)))
			if slope_deg <= (max_slope_deg + 0.001):
				# Two tangents exist; choose the one nearest current angle to avoid 180° flips/jitter.
				var tan_a: float = atan2(n.y, n.x) + PI * 0.5
				var tan_b: float = tan_a + PI
				var cur_ang: float = foot.global_rotation
				var da: float = absf(wrapf(tan_a - cur_ang, -PI, PI))
				var db: float = absf(wrapf(tan_b - cur_ang, -PI, PI))
				ground_ref_ang = tan_a if da <= db else tan_b
				n_dbg = n

	var swing_ref_ang: float = o._swing_flat_ref_angle_f if is_front else o._swing_flat_ref_angle_r
	var have_ground: bool = is_finite(ground_ref_ang)
	var have_swing: bool = is_finite(swing_ref_ang)

	var target_ang: float = NAN
	var have_target: bool = false

	# Brief A: reference selection follows attitude ownership, not foot control mode.
	if eff_att_mode == GCTypes.FootAttitudeMode.GROUND_PARALLEL:
		if have_ground:
			target_ang = ground_ref_ang
			have_target = true
		elif have_swing:
			target_ang = swing_ref_ang
			have_target = true
		else:
			target_ang = 0.0
			have_target = true
	elif eff_att_mode == GCTypes.FootAttitudeMode.AIR_PARALLEL:
		if have_swing:
			target_ang = swing_ref_ang
			have_target = true
		elif have_ground:
			target_ang = ground_ref_ang
			have_target = true
		else:
			target_ang = 0.0
			have_target = true

	if not have_target:
		return

	# Refresh swing flat reference when we have a valid ground target.
	if have_ground:
		if is_front:
			o._swing_flat_ref_angle_f = target_ang
		else:
			o._swing_flat_ref_angle_r = target_ang

	var k: float
	var d: float
	var tau_max: float
	var gain: float

	# Air attitude mode should use swing gains even if ctrl_mode is DISABLED (preland/settle windows).
	var use_swing_gains: bool = (
		(eff_att_mode == GCTypes.FootAttitudeMode.AIR_PARALLEL)
		or (ctrl_mode == GCTypes.FootControlMode.SWING)
		or (
			eff_att_mode == GCTypes.FootAttitudeMode.GROUND_PARALLEL
			and (not c.has("normal"))
		)
	)
	if use_swing_gains:
		k = o.swing_foot_flat_k
		d = o.swing_foot_flat_d
		tau_max = o.swing_foot_flat_tau_max
		gain = o.swing_foot_flat_strength
	else:
		var m: float = maxf(0.01, foot.mass)
		var omega: float = maxf(0.1, o.foot_flat_omega)
		var zeta: float = maxf(0.05, o.foot_flat_zeta)
		k = m * omega * omega
		d = 2.0 * zeta * m * omega
		tau_max = o.foot_flat_tau_max
		gain = 1.0

	if not (is_finite(k) and is_finite(d) and is_finite(tau_max)):
		return
	if gain <= 0.0:
		return

	var err_rad: float = wrapf(target_ang - foot.global_rotation, -PI, PI)

	if is_front:
		o._dbg_foot_flat_tgt_deg_f = rad_to_deg(target_ang)
		o._dbg_foot_flat_err_deg_f = rad_to_deg(err_rad)
		o._dbg_foot_flat_nx_f = n_dbg.x
		o._dbg_foot_flat_ny_f = n_dbg.y
	else:
		o._dbg_foot_flat_tgt_deg_r = rad_to_deg(target_ang)
		o._dbg_foot_flat_err_deg_r = rad_to_deg(err_rad)
		o._dbg_foot_flat_nx_r = n_dbg.x
		o._dbg_foot_flat_ny_r = n_dbg.y

	o._posture_mod.apply_angle_pd(foot, target_ang, k, d, tau_max, maxf(0.0, strength_mul) * gain)

func _active_ground_contact(sensor: Node, _foot: RigidBody2D) -> Dictionary:
	if sensor == null:
		return {}
	var grounded: bool = sensor.get("grounded") as bool
	if not grounded:
		return {}
	var n: Vector2 = sensor.get("contact_normal_w") as Vector2
	if not is_finite(n.x) or not is_finite(n.y) or n.length_squared() < 0.04:
		return {}
	return {"normal": n.normalized()}

func stepplanner_has_ground_contact_ref(is_front: bool) -> bool:
	if _refs == null:
		return false

	var sensor: Node = _refs.foot_sensor_front if is_front else _refs.foot_sensor_rear
	var foot: RigidBody2D = _refs.rb_foot_front if is_front else _refs.rb_foot_rear
	if sensor == null or foot == null:
		return false

	var c := _active_ground_contact(sensor, foot)
	if not c.has("normal"):
		return false

	var n: Vector2 = c["normal"]
	if n.length_squared() <= 1e-6:
		return false

	# Reject slope-invalid contacts so attitude mode does not flap between AIR/GROUND on bad normals.
	var max_slope_deg: float = 90.0
	if _owner != null:
		var raw_slope = _owner.get("plant_ground_max_slope_deg")
		if typeof(raw_slope) == TYPE_FLOAT or typeof(raw_slope) == TYPE_INT:
			max_slope_deg = float(raw_slope)
	var slope_deg: float = rad_to_deg(acos(clampf(n.normalized().dot(Vector2.UP), -1.0, 1.0)))
	return slope_deg <= (max_slope_deg + 0.001)


func update_plant_cache_midpoints() -> void:
	var o := _owner
	if o == null or o._rb_foot_front == null or o._rb_foot_rear == null:
		return
	if not o._plant_points_wired_ok:
		o._plant_front_x = o._rb_foot_front.global_position.x
		o._plant_rear_x = o._rb_foot_rear.global_position.x
		return
	if o._plant_front_active:
		o._plant_front_x = (o._planted_heel_front.x + o._planted_toe_front.x) * 0.5
	else:
		o._plant_front_x = o._rb_foot_front.to_global(o._pt_mid_f).x
	if o._plant_rear_active:
		o._plant_rear_x = (o._planted_heel_rear.x + o._planted_toe_rear.x) * 0.5
	else:
		o._plant_rear_x = o._rb_foot_rear.to_global(o._pt_mid_r).x

func apply_foot_damping_if_planted(front_g: bool, rear_g: bool) -> void:
	var o := _owner
	if o == null:
		return

	# Brief M+: in STEP_PLANNER, gc_foot_plant should not add extra planted damping force/torque
	# on foot RBs. Foot attitude torque remains owned elsewhere in gc_foot_plant.
	if int(o.get("movement_authority_mode")) == GCTypes.MovementAuthorityMode.STEP_PLANNER:
		return

	if o._plant_front_active and o._rb_foot_front != null and front_g:
		_apply_foot_damping(o, o._rb_foot_front)
	if o._plant_rear_active and o._rb_foot_rear != null and rear_g:
		_apply_foot_damping(o, o._rb_foot_rear)

func _apply_foot_damping(o: Node, foot: RigidBody2D) -> void:
	var vx: float = foot.linear_velocity.x
	var Fx: float = - vx * foot.mass * o.foot_ground_lin_damp
	var Fx_max: float = (foot.mass * o._g) * o.foot_ground_lin_force_mult
	Fx = clampf(Fx, -Fx_max, Fx_max)
	foot.apply_central_force(Vector2(Fx, 0.0))
	var tau: float = - foot.angular_velocity * o.foot_ground_ang_damp
	tau = clampf(tau, -o.foot_ground_ang_tau_max, o.foot_ground_ang_tau_max)
	foot.apply_torque(tau)

func _can_plant_now(foot: RigidBody2D) -> bool:
	if foot == null or _refs.rb_pelvis == null:
		return false
	var impact_timer: float = _owner.get("_impact_timer") as float
	var touchdown_ramp_t: float = _owner.get("_touchdown_ramp_t") as float
	var plant_touchdown_grace_sec: float = _owner.get("plant_touchdown_grace_sec") as float
	var plant_pelvis_vy_max: float = _owner.get("plant_pelvis_vy_max") as float
	var plant_foot_vy_max: float = _owner.get("plant_foot_vy_max") as float
	if impact_timer > 0.0 or touchdown_ramp_t < plant_touchdown_grace_sec:
		return true
	if absf(_refs.rb_pelvis.linear_velocity.y) > plant_pelvis_vy_max:
		return false
	if absf(foot.linear_velocity.y) > plant_foot_vy_max:
		return false
	return true

func _stepplanner_ctrl_mode_current(is_front: bool) -> int:
	if _owner == null:
		return GCTypes.FootControlMode.DISABLED

	var src_field: String = "_dbg_step_ctrl_src_f" if is_front else "_dbg_step_ctrl_src_r"
	_owner.set(src_field, 0)

	var auth_mode: int = int(_owner.get("movement_authority_mode"))
	if auth_mode != GCTypes.MovementAuthorityMode.STEP_PLANNER:
		return GCTypes.FootControlMode.DISABLED

	# Prefer final committed modes when available (post-step consumers).
	if bool(_owner.get("_arb_ctrl_modes_valid")):
		_owner.set(src_field, 1)
		return int(_owner.get("_arb_front_ctrl_mode") if is_front else _owner.get("_arb_rear_ctrl_mode"))

	# During step_foot_state() we are pre-COMMIT, so use provisional publish.
	if bool(_owner.get("_arb_pre_ctrl_modes_valid")):
		_owner.set(src_field, 2)
		return int(_owner.get("_arb_pre_front_ctrl_mode") if is_front else _owner.get("_arb_pre_rear_ctrl_mode"))

	# Last-resort migration fallback only. Keep visible so we can delete it later.
	_owner.set(src_field, 3)
	_owner.set("_dbg_step_ctrl_src_debug_fallback_hits", int(_owner.get("_dbg_step_ctrl_src_debug_fallback_hits")) + 1)
	return int(_owner.get("_dbg_foot_ctrl_mode_front") if is_front else _owner.get("_dbg_foot_ctrl_mode_rear"))

func _stepplanner_slot_target_x_current(is_front: bool) -> float:
	if _owner == null:
		return NAN

	var auth_mode: int = int(_owner.get("movement_authority_mode"))
	if auth_mode != GCTypes.MovementAuthorityMode.STEP_PLANNER:
		return NAN

	# 1) Preferred source: pipeline-published FINAL consumed slot pair (post planner override + no-cross clamp).
	if bool(_owner.get("_arb_slot_targets_valid")):
		var x_arb: float = float(_owner.get("_arb_slot_front_x") if is_front else _owner.get("_arb_slot_rear_x"))
		if is_finite(x_arb):
			return x_arb

	# 2) Fallback: current planner-published role slot (raw planner output).
	var x_step: float = float(_owner.get("_step_front_slot_x") if is_front else _owner.get("_step_rear_slot_x"))
	if is_finite(x_step):
		return x_step

	# 3) Fallback: stance planner authoritative service (role-aware).
	var stance_mod = _owner.get("_stance_planner_mod")
	if stance_mod != null:
		if stance_mod.has_method("slot_target_x_for_foot_authoritative"):
			var x_auth: float = float(stance_mod.slot_target_x_for_foot_authoritative(is_front))
			if is_finite(x_auth):
				return x_auth
		elif stance_mod.has_method("get_authoritative_slot_targets_x"):
			var slots_xy: Vector2 = stance_mod.get_authoritative_slot_targets_x()
			var x_pair: float = slots_xy.x if is_front else slots_xy.y
			if is_finite(x_pair):
				return x_pair

	return NAN

func _stepplanner_landing_gate_ok(is_front: bool, foot: RigidBody2D, sensor: Node, grounded: bool, stability01: float) -> bool:
	if _owner == null:
		return true

	var ok: bool = true
	var tgt_dx: float = NAN

	# Default debug values every call (visible in logs even when gate is bypassed)
	if is_front:
		_owner.set("_dbg_land_gate_front", 1)
		_owner.set("_dbg_land_target_dx_front", NAN)
	else:
		_owner.set("_dbg_land_gate_rear", 1)
		_owner.set("_dbg_land_target_dx_rear", NAN)

	# Only gate STEP_PLANNER swing-foot landing.
	var auth_mode: int = int(_owner.get("movement_authority_mode"))
	if auth_mode != GCTypes.MovementAuthorityMode.STEP_PLANNER:
		return true

	var ctrl_mode: int = _stepplanner_ctrl_mode_current(is_front)
	if ctrl_mode != GCTypes.FootControlMode.SWING:
		return true

	var step_phase: int = int(_owner.get("_step_phase"))
	var step_has_swing: bool = bool(_owner.get("_step_has_swing"))
	var step_swing_is_front: bool = bool(_owner.get("_step_swing_is_front"))
	var step_swing_active: bool = step_has_swing and (
		step_phase == GCTypes.StepPlannerPhase.SINGLE_SUPPORT_SWING
		or step_phase == GCTypes.StepPlannerPhase.TRANSFER
	)
	var this_is_active_swing: bool = step_swing_active and (step_swing_is_front == is_front)
	if not this_is_active_swing:
		return true

	# 1) Planner target proximity (active swing target, not raw live angle heuristics)
	var swing_target_x: float = float(_owner.get("_step_swing_target_x"))
	var foot_x: float = foot_best_x(is_front)
	var dx_tol: float = maxf(6.0, maxf(float(_owner.get("phase7_step_done_pos_eps")) * 1.5, float(_owner.get("plant_pin_slack_planted_px")) * 2.0))
	if is_finite(swing_target_x) and is_finite(foot_x):
		tgt_dx = absf(swing_target_x - foot_x)
	if not (is_finite(tgt_dx) and tgt_dx <= dx_tol):
		ok = false

	# 2) Contact normal must be valid and plantable (not toe-graze / wall-ish contact)
	if grounded:
		var c: Dictionary = _active_ground_contact(sensor, foot)
		if not c.has("normal"):
			ok = false
		else:
			var n: Vector2 = (c["normal"] as Vector2)
			if n.length_squared() > 0.0:
				n = n.normalized()
				var max_slope_deg: float = maxf(0.0, float(_owner.get("foot_flat_max_slope_deg")))
				var min_up_dot: float = cos(deg_to_rad(max_slope_deg))
				var up_dot: float = n.dot(Vector2.UP)
				if up_dot < min_up_dot:
					ok = false
			else:
				ok = false

	# 3) Soft stability floor for candidate entry/confirm in STEP_PLANNER swing landing
	var min_stab_gate: float = clampf(float(_owner.get("plant_min_stability")) * 0.5, 0.15, 0.95)
	if stability01 < min_stab_gate:
		ok = false

	if is_front:
		_owner.set("_dbg_land_gate_front", 1 if ok else 0)
		_owner.set("_dbg_land_target_dx_front", tgt_dx)
	else:
		_owner.set("_dbg_land_gate_rear", 1 if ok else 0)
		_owner.set("_dbg_land_target_dx_rear", tgt_dx)

	return ok

func step_foot_state(is_front: bool, grounded: bool, stability01: float, want_planted: bool, dt: float) -> void:
	var foot: RigidBody2D = _refs.rb_foot_front if is_front else _refs.rb_foot_rear
	var sensor: Node = _refs.foot_sensor_front if is_front else _refs.foot_sensor_rear
	if foot == null or sensor == null:
		return

	var st: int = _owner.get("_front_state") if is_front else _owner.get("_rear_state")
	var cand_t: float = _owner.get("_front_candidate_t") as float if is_front else _owner.get("_rear_candidate_t") as float
	var pb: float = _owner.get("_front_plant_blend") as float if is_front else _owner.get("_rear_plant_blend") as float

	var step_planner_authority: bool = (int(_owner.get("movement_authority_mode")) == GCTypes.MovementAuthorityMode.STEP_PLANNER)
	var owner_ctrl_mode: int = _stepplanner_ctrl_mode_current(is_front) if step_planner_authority else GCTypes.FootControlMode.DISABLED
	var owner_wants_plant: bool = want_planted
	if step_planner_authority:
		owner_wants_plant = (
			owner_ctrl_mode == GCTypes.FootControlMode.PLANTED_SUPPORT
			or owner_ctrl_mode == GCTypes.FootControlMode.PLANTED_CORRECT
			or owner_ctrl_mode == GCTypes.FootControlMode.RECOVERY
		)

	# Brief 4: per-foot Phase7 execution ownership contract gates plant-state transitions.
	var p7_force_release: bool = false
	var p7_allow_candidate_enter: bool = true
	var loc_mod = _owner.get("_locomotion_mod")
	if loc_mod != null and _owner.get("phase7_enable") and not step_planner_authority:
		p7_force_release = loc_mod.phase7_force_release_plant_state(is_front)
		p7_allow_candidate_enter = loc_mod.phase7_allow_plant_candidate_enter(is_front)
	if p7_force_release:
		st = GCTypes.FootPlantState.SWING
		cand_t = 0.0
		pb = 0.0

	var plant_allowed: bool = _owner.get("enable_foot_plant") and _owner.get("_plant_points_wired_ok") and _owner.get("_allow_plant_forces_this_frame")
	var stable_enough: bool = (stability01 >= (_owner.get("plant_min_stability") as float))
	var stepping_support_override: bool = false
	if step_planner_authority:
		# Brief B+: in STEP_PLANNER, plant-state entry/hold is owned by arbitration ctrl_mode, not legacy
		# Phase7 support flags. This breaks the "need legacy support luck before candidate" loop.
		stepping_support_override = owner_wants_plant
	else:
		stepping_support_override = _owner.get("_phase7_plan_active") and (_owner.get("_plan_support_is_front") == is_front)
	var pelvis_vy_ok: bool = true
	if _refs.rb_pelvis != null:
		var plant_pelvis_vy_max: float = _owner.get("plant_pelvis_vy_max") as float
		pelvis_vy_ok = (absf(_refs.rb_pelvis.linear_velocity.y) <= plant_pelvis_vy_max)
	var plant_foot_vy_max: float = _owner.get("plant_foot_vy_max") as float
	var stepplanner_land_gate_ok: bool = _stepplanner_landing_gate_ok(is_front, foot, sensor, grounded, stability01)
	var candidate_enter_allowed: bool = true if step_planner_authority else p7_allow_candidate_enter
	var foot_vy_ok: bool = (absf(foot.linear_velocity.y) <= plant_foot_vy_max)
	var can_plant_now_ok: bool = _can_plant_now(foot)
	var stable_or_override: bool = (stable_enough or stepping_support_override)

	var can_enter: bool = plant_allowed \
		and owner_wants_plant \
		and grounded \
		and stable_or_override \
		and pelvis_vy_ok \
		and foot_vy_ok \
		and can_plant_now_ok \
		and candidate_enter_allowed \
		and stepplanner_land_gate_ok

	var deny_mask: int = 0
	if not plant_allowed:
		deny_mask |= 1
	if not owner_wants_plant:
		deny_mask |= 2
	if not grounded:
		deny_mask |= 4
	if not stable_or_override:
		deny_mask |= 8
	if not pelvis_vy_ok:
		deny_mask |= 16
	if not foot_vy_ok:
		deny_mask |= 32
	if not can_plant_now_ok:
		deny_mask |= 64
	if not candidate_enter_allowed:
		deny_mask |= 128
	if not stepplanner_land_gate_ok:
		deny_mask |= 256
	_owner.set("_dbg_step_can_enter_deny_mask_f" if is_front else "_dbg_step_can_enter_deny_mask_r", deny_mask)

	match st:
		GCTypes.FootPlantState.SWING:
			cand_t = 0.0
			pb = 0.0
			if can_enter:
				st = GCTypes.FootPlantState.CANDIDATE
		GCTypes.FootPlantState.CANDIDATE:
			pb = 0.0
			if not can_enter:
				st = GCTypes.FootPlantState.SWING
				cand_t = 0.0
			else:
				cand_t += dt
				var touchdown_confirm_sec: float = _owner.get("touchdown_confirm_sec") as float
				if cand_t >= touchdown_confirm_sec:
					st = GCTypes.FootPlantState.PLANTED
					cand_t = 0.0
					pb = 0.0
					plant_latch_targets(is_front, foot, sensor)
		GCTypes.FootPlantState.PLANTED:
			if not plant_allowed or not grounded or not owner_wants_plant:
				st = GCTypes.FootPlantState.SWING
				cand_t = 0.0
				pb = 0.0
			else:
				var plant_ramp_sec: float = _owner.get("plant_ramp_sec") as float
				pb = minf(1.0, pb + dt / maxf(0.001, plant_ramp_sec))
				if step_planner_authority:
					# Brief M+: no planted recenter/glue direct foot forces in STEP_PLANNER.
					# Idle slot correction must come from planner-owned corrective stepping.
					_owner.set("_dbg_recenter_skip_f" if is_front else "_dbg_recenter_skip_r", 1)
				else:
					var allow_recenter: bool = _owner.get("_allow_recenter_front") if is_front else _owner.get("_allow_recenter_rear")
					if loc_mod != null and _owner.get("phase7_enable") and not step_planner_authority:
						allow_recenter = allow_recenter and loc_mod.phase7_allow_recenter(is_front)
					if allow_recenter:
						recenter_planted_targets_to_neutral(is_front, stability01, dt)
					else:
						_owner.set("_dbg_recenter_skip_f" if is_front else "_dbg_recenter_skip_r", 1)
					apply_planted_x_glue_for_foot(is_front, foot, pb, dt)

	var active: bool = (st == GCTypes.FootPlantState.PLANTED) and plant_allowed and grounded

	var owned_plant_but_swing: bool = false
	if step_planner_authority:
		owned_plant_but_swing = owner_wants_plant and grounded and (st == GCTypes.FootPlantState.SWING)
		_owner.set("_dbg_step_owned_plant_but_swing_f" if is_front else "_dbg_step_owned_plant_but_swing_r", 1 if owned_plant_but_swing else 0)
		if owned_plant_but_swing:
			_owner.set("_dbg_step_owned_plant_but_swing_hits", int(_owner.get("_dbg_step_owned_plant_but_swing_hits")) + 1)
	var px: float = foot.global_position.x
	if active:
		var planted_heel_front: Vector2 = _owner.get("_planted_heel_front")
		var planted_toe_front: Vector2 = _owner.get("_planted_toe_front")
		var planted_heel_rear: Vector2 = _owner.get("_planted_heel_rear")
		var planted_toe_rear: Vector2 = _owner.get("_planted_toe_rear")
		if is_front:
			px = (planted_heel_front.x + planted_toe_front.x) * 0.5
		else:
			px = (planted_heel_rear.x + planted_toe_rear.x) * 0.5

	# Stance-width diagnostic: actual foot RB x (anchor vs foot = glue following)
	if foot != null:
		_owner.set("_dbg_foot_x_f" if is_front else "_dbg_foot_x_r", foot.global_position.x)
	if is_front:
		_owner.set("_front_state", st)
		_owner.set("_front_candidate_t", cand_t)
		_owner.set("_front_plant_blend", pb)
		_owner.set("_plant_front_active", active)
		_owner.set("_plant_front_x", px)
	else:
		_owner.set("_rear_state", st)
		_owner.set("_rear_candidate_t", cand_t)
		_owner.set("_rear_plant_blend", pb)
		_owner.set("_plant_rear_active", active)
		_owner.set("_plant_rear_x", px)

# --- Internal plant logic (no longer on controller) ---
func plant_latch_targets(is_front: bool, foot: RigidBody2D, sensor: Node) -> void:
	if foot == null or sensor == null:
		return
	var heel_local: Vector2 = (_owner.get("_pt_heel_f") as Vector2) if is_front else (_owner.get("_pt_heel_r") as Vector2)
	var toe_local: Vector2 = (_owner.get("_pt_toe_f") as Vector2) if is_front else (_owner.get("_pt_toe_r") as Vector2)
	var heel_w: Vector2 = foot.to_global(heel_local)
	var toe_w: Vector2 = foot.to_global(toe_local)
	var yref: float = _owner.get("_truth_front_y") as float if is_front else _owner.get("_truth_rear_y") as float
	if not is_finite(yref):
		var ry: float = sensor.get("contact_y_raw") as float
		yref = ry if is_finite(ry) else (sensor.get("contact_y") as float)
	if is_finite(yref):
		var pin_mode: int = _owner.get("plant_pin_mode")
		var is_tail: bool = _is_tail_foot(is_front)
		if _owner.get("crouch_trail_toe_only_enable") and is_tail and (_owner.get("_stance_alpha") as float <= _owner.get("crouch_trail_toe_only_alpha") as float):
			pin_mode = GCTypes.PlantPinMode.TOE_ONLY
		elif _owner.get("enable_foot_flatness") and _owner.get("foot_flat_force_two_point_pins"):
			pin_mode = GCTypes.PlantPinMode.HEEL_AND_TOE
		var dy: float = 0.0
		match pin_mode:
			GCTypes.PlantPinMode.TOE_ONLY:
				dy = yref - toe_w.y
			GCTypes.PlantPinMode.HEEL_ONLY:
				dy = yref - heel_w.y
			GCTypes.PlantPinMode.HEEL_AND_TOE:
				dy = yref - ((heel_w.y + toe_w.y) * 0.5)
		heel_w.y += dy
		toe_w.y += dy
		# If we will pin both heel+toe, do NOT latch a tilted footprint.
		# Flatten targets to yref while preserving heel/toe spacing in X.
		if _owner.get("enable_foot_flatness") and _owner.get("foot_flat_force_two_point_pins"):
			var cx2: float = 0.5 * (heel_w.x + toe_w.x)
			var dx_local: float = absf(toe_local.x - heel_local.x)
			var sx: float = float(sign(toe_w.x - heel_w.x))
			if sx == 0.0:
				sx = 1.0
			var half2: float = 0.5 * dx_local * sx
			heel_w = Vector2(cx2 - half2, yref)
			toe_w  = Vector2(cx2 + half2, yref)
	if is_front:
		_owner.set("_planted_heel_front", heel_w)
		_owner.set("_planted_toe_front", toe_w)
	else:
		_owner.set("_planted_heel_rear", heel_w)
		_owner.set("_planted_toe_rear", toe_w)

func recenter_planted_targets_to_neutral(is_front: bool, stability01: float, dt: float) -> void:
	# Diagnostic: 0=ran, 1=no_allow, 2=stability, 3=deadband
	var skip_key_f: String = "_dbg_recenter_skip_f"
	var skip_key_r: String = "_dbg_recenter_skip_r"
	var key: String = skip_key_f if is_front else skip_key_r
	if not _owner.get("plant_allow_recenter_planted_targets"):
		_owner.set(key, 1)
		return
	if dt <= 0.0:
		_owner.set(key, 1)
		return
	var heel: Vector2 = (_owner.get("_planted_heel_front") as Vector2) if is_front else (_owner.get("_planted_heel_rear") as Vector2)
	var toe: Vector2 = (_owner.get("_planted_toe_front") as Vector2) if is_front else (_owner.get("_planted_toe_rear") as Vector2)
	var center_x_now: float = (heel.x + toe.x) * 0.5
	var other_heel: Vector2 = (_owner.get("_planted_heel_rear") as Vector2) if is_front else (_owner.get("_planted_heel_front") as Vector2)
	var other_toe: Vector2 = (_owner.get("_planted_toe_rear") as Vector2) if is_front else (_owner.get("_planted_toe_front") as Vector2)
	var other_anchor_x: float = (other_heel.x + other_toe.x) * 0.5
	var min_sep: float = _owner.get("phase7_min_foot_separation_px") as float
	var converged: bool = absf(center_x_now - other_anchor_x) < min_sep
	var min_stability: float = (_owner.get("plant_recenter_min_stability_when_converged") as float) if converged else (_owner.get("plant_min_stability") as float)
	if not is_finite(min_stability) or min_stability < 0.0:
		min_stability = 0.15
	if stability01 < min_stability:
		_owner.set(key, 2)
		return
	var center_x_want: float = _stepplanner_slot_target_x_current(is_front)
	if not is_finite(center_x_want):
		center_x_want = _neutral_target_x_for_foot(is_front)
	if not is_finite(center_x_want):
		# If this triggers in STEP_PLANNER after Brief D, slot authority publishing is broken upstream.
		return
	var dx: float = center_x_want - center_x_now
	# Always set diagnostic want/now so print shows target vs current even when we skip
	_owner.set("_dbg_recenter_want_f" if is_front else "_dbg_recenter_want_r", center_x_want)
	_owner.set("_dbg_recenter_now_f" if is_front else "_dbg_recenter_now_r", center_x_now)
	var deadband: float = _owner.get("plant_drift_deadband_px") as float
	if absf(dx) <= deadband:
		_owner.set(key, 3)
		_owner.set("_dbg_recenter_dx_f" if is_front else "_dbg_recenter_dx_r", dx)
		return
	_owner.set(key, 0)
	_owner.set("_dbg_recenter_dx_f" if is_front else "_dbg_recenter_dx_r", dx)
	var hz: float = maxf(0.0, _owner.get("stance_anchor_move_hz") as float)
	if converged:
		var hz_conv: float = _owner.get("stance_anchor_move_hz_when_converged") as float
		if hz_conv > 0.0:
			hz = hz_conv
	var a: float = 1.0 if hz <= 0.0 else clampf(dt * hz, 0.0, 1.0)
	var dxc: float = dx * a
	var max_move_px: float = _owner.get("stance_anchor_max_move_px_per_frame") as float
	if max_move_px > 0.0:
		dxc = clampf(dxc, -max_move_px, max_move_px)
	if is_front:
		_owner.set("_planted_heel_front", Vector2(heel.x + dxc, heel.y))
		_owner.set("_planted_toe_front", Vector2(toe.x + dxc, toe.y))
		_owner.set("_dbg_anchor_moved_front", true)
	else:
		_owner.set("_planted_heel_rear", Vector2(heel.x + dxc, heel.y))
		_owner.set("_planted_toe_rear", Vector2(toe.x + dxc, toe.y))
		_owner.set("_dbg_anchor_moved_rear", true)

# Brief J: STEP_PLANNER post-COMMIT recenter executor.
# Purpose:
# - pipeline final ownership COMMIT can upgrade feet to PLANTED_CORRECT after step_foot_state()
# - recenter_planted_targets_to_neutral() currently runs inside step_foot_state()
# - so corrective ownership can arrive "too late" and miss execution for the frame
#
# This pass lets gc_pipeline call a small post-COMMIT recenter-only executor once final _arb_* modes and
# _allow_recenter_* are authoritative for the frame.
func stepplanner_apply_postcommit_recenter(dt: float, front_grounded: bool, rear_grounded: bool, front_stability01: float, rear_stability01: float) -> void:
	if _owner == null or dt <= 0.0:
		return
	if int(_owner.get("movement_authority_mode")) != GCTypes.MovementAuthorityMode.STEP_PLANNER:
		return
	if not bool(_owner.get("_arb_ctrl_modes_valid")):
		return

	_stepplanner_apply_postcommit_recenter_one(true, front_grounded, front_stability01, dt)
	_stepplanner_apply_postcommit_recenter_one(false, rear_grounded, rear_stability01, dt)

func _stepplanner_apply_postcommit_recenter_one(is_front: bool, grounded: bool, stability01: float, dt: float) -> void:
	var ctrl_mode: int = int(_owner.get("_arb_front_ctrl_mode") if is_front else _owner.get("_arb_rear_ctrl_mode"))
	if ctrl_mode != GCTypes.FootControlMode.PLANTED_CORRECT:
		return

	# Recenter planted targets only for fully planted feet (same contract as normal planted recenter path).
	var st: int = int(_owner.get("_front_state") if is_front else _owner.get("_rear_state"))
	if st != GCTypes.FootPlantState.PLANTED:
		return

	# Use actual executor pin-active fields (not guessed names).
	var plant_active: bool = bool(_owner.get("_plant_front_active") if is_front else _owner.get("_plant_rear_active"))
	if not plant_active:
		return
	if not grounded:
		return

	var allow_recenter: bool = bool(_owner.get("_allow_recenter_front") if is_front else _owner.get("_allow_recenter_rear"))
	if not allow_recenter:
		_owner.set("_dbg_recenter_skip_f" if is_front else "_dbg_recenter_skip_r", 1)
		return

	recenter_planted_targets_to_neutral(is_front, stability01, dt)


func plant_pd_x(foot: RigidBody2D, p_now: Vector2, tgt_x: float, deadband: float, m: float, max_f: float, scale_val: float) -> Vector2:
	if not is_finite(tgt_x):
		return Vector2.ZERO
	var err: float = tgt_x - p_now.x
	if absf(err) <= deadband:
		return Vector2.ZERO
	var r: Vector2 = p_now - foot.global_position
	var vx_pt: float = foot.linear_velocity.x + (-foot.angular_velocity * r.y)
	var vel_gain: float = _owner.get("plant_drift_vel_gain") as float
	var vx_max: float = _owner.get("plant_drift_vx_max") as float
	var accel_gain: float = _owner.get("plant_drift_accel_gain") as float
	var desired_vx: float = clampf(err * vel_gain, -vx_max, vx_max)
	var ax: float = (desired_vx - vx_pt) * accel_gain
	var Fx: float = m * ax
	Fx = clampf(Fx, -max_f, max_f)
	Fx *= scale_val
	var tau: float = (-r.y * Fx)
	return Vector2(Fx, tau)

func apply_planted_x_glue_for_foot(is_front: bool, foot: RigidBody2D, plant01: float, dt: float) -> void:
	if foot == null or plant01 <= 0.0 or dt <= 0.0:
		return
	var loc_glue: Variant = _owner.get("_locomotion_mod")
	var step_planner_authority: bool = int(_owner.get("movement_authority_mode")) == GCTypes.MovementAuthorityMode.STEP_PLANNER
	if loc_glue != null and _owner.get("phase7_enable") and not step_planner_authority:
		if not loc_glue.phase7_allow_plant_glue(is_front):
			return
	var pin_mode: int = _owner.get("plant_pin_mode")
	var is_tail: bool = _is_tail_foot(is_front)
	var crouch_trail_toe_slide: bool = false
	if _owner.get("crouch_trail_toe_only_enable") and is_tail and (_owner.get("_stance_alpha") as float <= _owner.get("crouch_trail_toe_only_alpha") as float):
		pin_mode = GCTypes.PlantPinMode.TOE_ONLY
		crouch_trail_toe_slide = _owner.get("crouch_trail_toe_slide_enable")
	elif _owner.get("enable_foot_flatness") and _owner.get("foot_flat_force_two_point_pins"):
		pin_mode = GCTypes.PlantPinMode.HEEL_AND_TOE
	if crouch_trail_toe_slide and (pin_mode == GCTypes.PlantPinMode.TOE_ONLY):
		var toe_local_slide: Vector2 = (_owner.get("_pt_toe_f") as Vector2) if is_front else (_owner.get("_pt_toe_r") as Vector2)
		var toe_now_slide: Vector2 = foot.to_global(toe_local_slide)
		var hz_follow: float = maxf(0.0, _owner.get("crouch_trail_toe_slide_follow_hz") as float)
		var a_follow: float = 1.0 if hz_follow <= 0.0 else clampf(dt * hz_follow, 0.0, 1.0)
		if is_front:
			var pt: Vector2 = _owner.get("_planted_toe_front") as Vector2
			_owner.set("_planted_toe_front", Vector2(lerpf(pt.x, toe_now_slide.x, a_follow), pt.y))
		else:
			var pt: Vector2 = _owner.get("_planted_toe_rear") as Vector2
			_owner.set("_planted_toe_rear", Vector2(lerpf(pt.x, toe_now_slide.x, a_follow), pt.y))
	var spawn_ramp_sec: float = _owner.get("spawn_ramp_sec") as float
	var spawn01: float = 1.0
	if spawn_ramp_sec > 0.001:
		spawn01 = clampf(_owner.get("_t") as float / spawn_ramp_sec, 0.0, 1.0)
	var slack_swing: float = _owner.get("plant_pin_slack_px") as float
	var slack_planted: float = _owner.get("plant_pin_slack_planted_px") as float
	var slack_now: float = lerpf(slack_swing, slack_planted, clampf(plant01, 0.0, 1.0))
	slack_now = maxf(0.0, slack_now)
	var deadband: float = maxf(slack_now, _owner.get("plant_drift_deadband_px") as float)
	var m: float = maxf(0.01, foot.mass)
	var g: float = _refs.g
	var drift_mult: float = maxf(0.0, _owner.get("plant_drift_force_mult") as float)
	var recenter_mult: float = maxf(0.0, _owner.get("plant_drift_recenter_force_mult") as float)
	var recenter_err_px: float = maxf(0.0, _owner.get("plant_drift_recenter_err_px") as float)
	var heel_local: Vector2 = (_owner.get("_pt_heel_f") as Vector2) if is_front else (_owner.get("_pt_heel_r") as Vector2)
	var toe_local: Vector2 = (_owner.get("_pt_toe_f") as Vector2) if is_front else (_owner.get("_pt_toe_r") as Vector2)
	var heel_now: Vector2 = foot.to_global(heel_local)
	var toe_now: Vector2 = foot.to_global(toe_local)
	var heel_tgt_x: float = (_owner.get("_planted_heel_front") as Vector2).x if is_front else (_owner.get("_planted_heel_rear") as Vector2).x
	var toe_tgt_x: float = (_owner.get("_planted_toe_front") as Vector2).x if is_front else (_owner.get("_planted_toe_rear") as Vector2).x
	# When anchor-foot error is large (recenter case), use stronger force so foot can follow anchor.
	var err_h: float = heel_tgt_x - heel_now.x
	var err_t: float = toe_tgt_x - toe_now.x
	var blend_h: float = 0.0 if recenter_err_px <= 0.0 else clampf((absf(err_h) - deadband) / recenter_err_px, 0.0, 1.0)
	var blend_t: float = 0.0 if recenter_err_px <= 0.0 else clampf((absf(err_t) - deadband) / recenter_err_px, 0.0, 1.0)
	var max_f_heel: float = m * g * lerpf(drift_mult, recenter_mult, blend_h)
	var max_f_toe: float = m * g * lerpf(drift_mult, recenter_mult, blend_t)
	var Fx_total: float = 0.0
	var tau_total: float = 0.0
	match pin_mode:
		GCTypes.PlantPinMode.HEEL_ONLY:
			var ft_h: Vector2 = plant_pd_x(foot, heel_now, heel_tgt_x, deadband, m, max_f_heel, plant01 * spawn01)
			Fx_total += ft_h.x
			tau_total += ft_h.y
		GCTypes.PlantPinMode.TOE_ONLY:
			var ft_t: Vector2 = plant_pd_x(foot, toe_now, toe_tgt_x, deadband, m, max_f_toe, plant01 * spawn01)
			Fx_total += ft_t.x
			tau_total += ft_t.y
		GCTypes.PlantPinMode.HEEL_AND_TOE:
			var ft_h2: Vector2 = plant_pd_x(foot, heel_now, heel_tgt_x, deadband, m, max_f_heel, plant01 * spawn01 * 0.5)
			var ft_t2: Vector2 = plant_pd_x(foot, toe_now, toe_tgt_x, deadband, m, max_f_toe, plant01 * spawn01 * 0.5)
			Fx_total += (ft_h2.x + ft_t2.x)
			tau_total += (ft_h2.y + ft_t2.y)
	var tau_max: float = maxf(10.0, _owner.get("foot_ground_ang_tau_max") as float)
	tau_total = clampf(tau_total, -tau_max, tau_max)
	# 5R2-J: Foot rotation ownership hardening.
	# In STEP_PLANNER mode, plant x-glue may stabilize X (Fx), but it must NOT apply rotational torque.
	# Foot orientation is owned by gc_foot_plant.apply_parallel_foot_attitude_by_mode(...) via pipeline arbitration.
	if (_owner.get("movement_authority_mode") as int) == GCTypes.MovementAuthorityMode.STEP_PLANNER:
		tau_total = 0.0
	# Legacy Phase7 stepping also uses Fx-only glue torque suppression.
	elif _owner.get("_phase7_plan_active") and (_owner.get("_phase7_exec_phase") as int != GCTypes.Phase7ExecPhase.IDLE):
		tau_total = 0.0
	if Fx_total != 0.0:
		foot.apply_central_force(Vector2(Fx_total, 0.0))
	if tau_total != 0.0:
		foot.apply_torque(tau_total)

# Sync plant support Y into refs so vertical support (and others) can read from refs instead of calling back to owner.
func sync_plant_support_y_to_refs() -> void:
	_refs.plant_support_y_front = plant_support_y(true)
	_refs.plant_support_y_rear = plant_support_y(false)

# --- Query helpers (controller and other modules call these via owner delegate or direct module ref) ---
func foot_best_x(is_front: bool) -> float:
	var plant_front_active: bool = _owner.get("_plant_front_active")
	var plant_rear_active: bool = _owner.get("_plant_rear_active")
	if is_front:
		if plant_front_active and plant_forces_enabled_for_foot(true):
			return _owner.get("_plant_front_x") as float
		var sens: Node = _refs.foot_sensor_front
		if sens != null and sens.get("grounded"):
			return (sens.get("contact_point_w") as Vector2).x
		var rb: RigidBody2D = _refs.rb_foot_front
		if rb != null:
			return rb.global_position.x
	else:
		if plant_rear_active and plant_forces_enabled_for_foot(false):
			return _owner.get("_plant_rear_x") as float
		var sens: Node = _refs.foot_sensor_rear
		if sens != null and sens.get("grounded"):
			return (sens.get("contact_point_w") as Vector2).x
		var rb: RigidBody2D = _refs.rb_foot_rear
		if rb != null:
			return rb.global_position.x
	return NAN

func plant_forces_enabled_for_foot(is_front: bool) -> bool:
	if not _owner.get("enable_foot_plant"):
		return false
	if not _owner.get("_allow_plant_forces_this_frame"):
		return false
	if not _owner.get("_plant_points_wired_ok"):
		return false
	if is_front:
		return _owner.get("_plant_front_active") and (_owner.get("_front_plant_blend") as float > 0.0)
	return _owner.get("_plant_rear_active") and (_owner.get("_rear_plant_blend") as float > 0.0)

func plant_support_y(is_front: bool) -> float:
	var s: Node = _refs.foot_sensor_front if is_front else _refs.foot_sensor_rear
	if s == null:
		return NAN
	return s.get("contact_y") as float if s.get("contact_valid") else NAN

func _is_tail_foot(is_front: bool) -> bool:
	if _refs.rb_foot_front == null or _refs.rb_foot_rear == null:
		return not is_front
	var fx: float = _refs.rb_foot_front.global_position.x
	var rx: float = _refs.rb_foot_rear.global_position.x
	var facing_sign: float = _owner.get("facing_sign") as float
	var tail_is_front: bool = (fx < rx) if facing_sign > 0 else (fx > rx)
	return tail_is_front == is_front

func _neutral_target_x_for_foot(is_front: bool) -> float:
	# Brief D: in STEP_PLANNER mode, do NOT source slot targets from locomotion/Phase7 helpers.
	# Those paths can reintroduce legacy world-order / stale-center contamination.
	var auth_mode: int = int(_owner.get("movement_authority_mode"))
	var step_planner_authority: bool = (auth_mode == GCTypes.MovementAuthorityMode.STEP_PLANNER)

	# STEP_PLANNER fallback path: use stance planner authoritative role-aware slot service first.
	if step_planner_authority:
		var stance_mod = _owner.get("_stance_planner_mod")
		if stance_mod != null:
			if stance_mod.has_method("slot_target_x_for_foot_authoritative"):
				var x_sp: float = float(stance_mod.slot_target_x_for_foot_authoritative(is_front))
				if is_finite(x_sp):
					return x_sp
			elif stance_mod.has_method("get_authoritative_slot_targets_x"):
				var slots_xy: Vector2 = stance_mod.get_authoritative_slot_targets_x()
				var x_pair: float = slots_xy.x if is_front else slots_xy.y
				if is_finite(x_pair):
					return x_pair

	# Legacy/non-STEP_PLANNER path may still use locomotion slot target service.
	var loc_mod = _owner.get("_locomotion_mod")
	if loc_mod != null and not step_planner_authority:
		var x: float = loc_mod.slot_target_x_for_foot(is_front)
		if is_finite(x):
			return x

	# Final posture-derived fallback (kept for robustness).
	var cx: float = _owner.get("_stance_center_x") as float
	if not is_finite(cx):
		cx = _refs.rb_pelvis.global_position.x if _refs.rb_pelvis != null else 0.0
	var stance_alpha: float = clampf(_owner.get("_stance_alpha") as float, 0.0, 1.0)
	var w_stand: float = _owner.get("stance_w_stand") as float
	var w_crouch: float = _owner.get("stance_w_crouch") as float
	var full_width: float = lerpf(w_stand, w_crouch, 1.0 - stance_alpha)
	var half: float = full_width * 0.5
	var ax: float = float(_owner.get("_neutral_axis_sign"))
	var lead_is_front: bool = _owner.get("_lead_is_front")

	# Phase7 lead-role override is legacy-only. Never apply it in STEP_PLANNER mode.
	if (not step_planner_authority) and _owner.get("_phase7_plan_active"):
		lead_is_front = _owner.get("_phase7_seq_gait_lead_is_front")

	return GCMath.neutral_target_x(cx, half, ax, lead_is_front, is_front)
