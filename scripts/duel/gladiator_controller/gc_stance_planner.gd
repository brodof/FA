# res://scripts/duel/gladiator_controller/gc_stance_planner.gd
# Phase 3 standing plan + BRACE + Phase 6 RECOVER. Writes owner _plan_*, _planner_mode; uses refs for nodes.
class_name GCStancePlanner
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

# 5R2-C: authoritative slot geometry service for planner paths.
# This is posture-derived (authored pose + calibrated rig geometry via _posture_mod),
# continuous across crouch, deterministic, and independent from Phase7 planner state.
func get_authoritative_slot_targets_x() -> Vector2:
	var cx: float = NAN
	var ma: int = int(_owner.get("movement_authority_mode"))
	var slot_center_src_dbg: int = 0

	# Brief 3: in STEP_PLANNER mode, stance-slot center must come from planner-owned publish,
	# not legacy/mixed _stance_center_x.
	if ma == GCTypes.MovementAuthorityMode.STEP_PLANNER:
		if bool(_owner.get("_step_slot_center_valid")):
			cx = _owner.get("_step_slot_center_x") as float
			if is_finite(cx):
				slot_center_src_dbg = 1

		# Body-side fallback only (never foot-angle-derived).
		if not is_finite(cx):
			cx = _refs.rb_pelvis.global_position.x if _refs != null and _refs.rb_pelvis != null else NAN
			if is_finite(cx):
				slot_center_src_dbg = 2

	# Legacy modes may continue using existing stance-center integration.
	if not is_finite(cx):
		cx = _owner.get("_stance_center_x") as float
		if is_finite(cx):
			slot_center_src_dbg = 3
	if not is_finite(cx):
		cx = _refs.rb_pelvis.global_position.x if _refs != null and _refs.rb_pelvis != null else 0.0
		if is_finite(cx):
			slot_center_src_dbg = 4

	_owner.set("_dbg_slot_center_src_dbg", slot_center_src_dbg)

	var stance_alpha_raw: float = _owner.get("_stance_alpha") as float
	if not is_finite(stance_alpha_raw):
		# 5R2-J2: never allow NaN posture to poison slot geometry.
		# Default = standing (alpha=1) so crouch01=0.
		stance_alpha_raw = 1.0
	var stance_alpha: float = clampf(stance_alpha_raw, 0.0, 1.0)
	var crouch01: float = 1.0 - stance_alpha
	var w_stand: float = _owner.get("stance_w_stand") as float
	var w_crouch: float = _owner.get("stance_w_crouch") as float
	if not is_finite(w_stand):
		w_stand = 1.0
	if not is_finite(w_crouch):
		w_crouch = w_stand
	var full_width: float = lerpf(w_stand, w_crouch, crouch01)
	if not is_finite(full_width):
		full_width = w_stand
	full_width = maxf(1.0, full_width)

	var ax: float = float(_owner.get("_neutral_axis_sign"))
	if not is_finite(ax):
		ax = 0.0
	if ax == 0.0:
		ax = signf(_owner.get("facing_sign") as float)
	if not is_finite(ax) or ax == 0.0:
		ax = 1.0

	# IMPORTANT (anti-leak): slot geometry does not depend on Phase7 planner state.
	# Role comes from current authored stance role only.
	var lead_is_front: bool = _owner.get("_lead_is_front")

	var front_s: float = NAN
	var rear_s: float = NAN

	# Primary source: posture-derived asymmetry template (authored + calibrated).
	if _owner._posture_mod != null:
		var tpl: Vector2 = _owner._posture_mod.get_slot_template_axis_offsets(lead_is_front)
		front_s = tpl.x
		rear_s = tpl.y

	# Fallback only if posture template is unavailable/invalid.
	if not is_finite(front_s) or not is_finite(rear_s):
		var half_fb: float = full_width * 0.5
		front_s = half_fb if lead_is_front else -half_fb
		rear_s = -half_fb if lead_is_front else half_fb
		_owner.set("_dbg_slot_tpl_fallback", 1)
		_owner.set("_dbg_slot_mapping_mode_dbg", 2)
	else:
		_owner.set("_dbg_slot_tpl_fallback", 0)
		_owner.set("_dbg_slot_mapping_mode_dbg", 1)

		# Recenter template and normalize separation to configured gameplay width.
		var mid_s: float = 0.5 * (front_s + rear_s)
		front_s -= mid_s
		rear_s -= mid_s

		var sep_s: float = absf(front_s - rear_s)
		if sep_s > 0.001:
			var scale_s: float = full_width / sep_s
			front_s *= scale_s
			rear_s *= scale_s
		else:
			var half_bad: float = full_width * 0.5
			front_s = half_bad if lead_is_front else -half_bad
			rear_s = -half_bad if lead_is_front else half_bad
			_owner.set("_dbg_slot_tpl_fallback", 1)

	# Role-side sanity in axis space (anti-collapse / anti-sign-flip).
	var front_expected_sign: float = 1.0 if lead_is_front else -1.0
	var rear_expected_sign: float = -front_expected_sign
	var sign_bad: bool = (front_s * front_expected_sign <= 0.0) or (rear_s * rear_expected_sign <= 0.0)
	if sign_bad:
		var half_fix: float = full_width * 0.5
		front_s = half_fix if lead_is_front else -half_fix
		rear_s = -half_fix if lead_is_front else half_fix
		_owner.set("_dbg_slot_tpl_signfix", 1)
		_owner.set("_dbg_slot_role_map_valid", 0)

		var cross_block_hits: int = int(_owner.get("_dbg_slot_cross_blocked_count"))
		_owner.set("_dbg_slot_cross_blocked_count", cross_block_hits + 1)
	else:
		_owner.set("_dbg_slot_tpl_signfix", 0)
		_owner.set("_dbg_slot_role_map_valid", 1)

	# Minimum slot separation invariant (planner-level anti-collapse).
	var min_sep_cfg: float = maxf(0.0, _owner.get("phase7_min_foot_separation_px") as float)
	var min_sep: float = maxf(min_sep_cfg, full_width * 0.5)
	_owner.set("_dbg_slot_sep_min_dbg", min_sep)
	if absf(front_s - rear_s) < min_sep:
		var half_sep: float = 0.5 * min_sep
		front_s = front_expected_sign * half_sep
		rear_s = rear_expected_sign * half_sep
		_owner.set("_dbg_slot_tpl_sepfix", 1)
	else:
		_owner.set("_dbg_slot_tpl_sepfix", 0)

	var front_x: float = cx + (front_s * ax)
	var rear_x: float = cx + (rear_s * ax)

	var slot_valid: bool = is_finite(front_x) and is_finite(rear_x)
	_owner.set("_dbg_slot_src_valid", 1 if slot_valid else 0)

	_owner.set("_dbg_slot_front_x", front_x)
	_owner.set("_dbg_slot_rear_x", rear_x)
	_owner.set("_dbg_slot_sep_x", absf(front_x - rear_x))

	# Brief C mirror diagnostics (same values, explicit names for log/audit).
	_owner.set("_dbg_slot_world_front_x_dbg", front_x)
	_owner.set("_dbg_slot_world_rear_x_dbg", rear_x)
	_owner.set("_dbg_slot_sep_dbg", absf(front_x - rear_x))

	return Vector2(front_x, rear_x)

func slot_target_x_for_foot_authoritative(is_front: bool) -> float:
	var slots: Vector2 = get_authoritative_slot_targets_x()
	return slots.x if is_front else slots.y

func choose_support_foot(front_g: bool, rear_g: bool, stabF: float, stabR: float, dt: float) -> bool:
	var support_is_front: bool = _owner.get("support_is_front")
	if front_g and not rear_g:
		_owner.set("support_is_front", true)
		_owner.set("_support_candidate_is_front", true)
		_owner.set("_support_candidate_t", 0.0)
		return true
	if rear_g and not front_g:
		_owner.set("support_is_front", false)
		_owner.set("_support_candidate_is_front", false)
		_owner.set("_support_candidate_t", 0.0)
		return false
	if not front_g and not rear_g:
		return support_is_front

	var com_x: float = GCMath.compute_com_world(_refs.rb_list, _refs.rb_mass_sum).x
	var fx: float = _owner._foot_plant_mod.foot_best_x(true) if _owner._foot_plant_mod != null else NAN
	var rx: float = _owner._foot_plant_mod.foot_best_x(false) if _owner._foot_plant_mod != null else NAN
	if not is_finite(fx) or not is_finite(rx):
		return support_is_front

	var df: float = absf(com_x - fx)
	var dr: float = absf(com_x - rx)
	var proxF: float = 1.25 - clampf(df / 30.0, 0.0, 1.0)
	var proxR: float = 1.25 - clampf(dr / 30.0, 0.0, 1.0)
	var scoreF: float = proxF + 0.35 * stabF
	var scoreR: float = proxR + 0.35 * stabR
	var gap: float = scoreF - scoreR
	var flip_thresh: float = 0.20
	var want_support_front: bool = support_is_front
	if support_is_front:
		if gap < -flip_thresh:
			want_support_front = false
	else:
		if gap > flip_thresh:
			want_support_front = true

	var _support_candidate_is_front: bool = _owner.get("_support_candidate_is_front")
	var _support_candidate_t: float = _owner.get("_support_candidate_t")
	var support_swap_confirm_sec: float = _owner.get("support_swap_confirm_sec") as float

	if want_support_front == support_is_front:
		_owner.set("_support_candidate_is_front", support_is_front)
		_owner.set("_support_candidate_t", 0.0)
		return support_is_front

	if want_support_front != _support_candidate_is_front:
		_owner.set("_support_candidate_is_front", want_support_front)
		_owner.set("_support_candidate_t", 0.0)
		return support_is_front

	_support_candidate_t += dt
	_owner.set("_support_candidate_t", _support_candidate_t)
	if _support_candidate_t >= support_swap_confirm_sec:
		_owner.set("support_is_front", want_support_front)
		_owner.set("_support_candidate_is_front", want_support_front)
		_owner.set("_support_candidate_t", 0.0)
		return want_support_front

	return support_is_front

func update_standing_plan(front_g: bool, rear_g: bool, stabF: float, stabR: float, dt: float, stance_changing: bool) -> void:
	var plan_front_age: float = _owner.get("_plan_front_age") as float
	var plan_rear_age: float = _owner.get("_plan_rear_age") as float
	plan_front_age += dt
	plan_rear_age += dt
	_owner.set("_plan_front_age", plan_front_age)
	_owner.set("_plan_rear_age", plan_rear_age)

	var truth_front_allow_t: float = _owner.get("_truth_front_allow_t") as float
	var truth_rear_allow_t: float = _owner.get("_truth_rear_allow_t") as float
	var front_allow: bool = front_g or (truth_front_allow_t > 0.0)
	var rear_allow: bool = rear_g or (truth_rear_allow_t > 0.0)
	var grounded_eff: bool = front_allow or rear_allow

	var rb_pelvis: RigidBody2D = _refs.rb_pelvis
	var rb_foot_front: RigidBody2D = _refs.rb_foot_front
	var rb_foot_rear: RigidBody2D = _refs.rb_foot_rear
	var rb_torso: RigidBody2D = _refs.rb_torso

	var fx: float = rb_foot_front.global_position.x if rb_foot_front != null else NAN
	var rx: float = rb_foot_rear.global_position.x if rb_foot_rear != null else NAN
	var lead_is_front: bool = _owner.get("_lead_is_front")

	# 5R2-C: standing planner consumes the same authoritative slot geometry service
	# used by the new step planner shadow path (single source of slot truth).
	var nxF: float = slot_target_x_for_foot_authoritative(true)
	var nxR: float = slot_target_x_for_foot_authoritative(false)

	# Safety fallback only if authoritative slot service returned invalid output.
	if not is_finite(nxF) or not is_finite(nxR):
		var cx: float = _owner.get("_stance_center_x") as float
		if not is_finite(cx):
			cx = _refs.rb_pelvis.global_position.x if _refs.rb_pelvis != null else 0.0
		var stance_alpha: float = clampf(_refs.stance_alpha, 0.0, 1.0)
		var w_stand: float = _owner.get("stance_w_stand") as float
		var w_crouch: float = _owner.get("stance_w_crouch") as float
		var full_width: float = lerpf(w_stand, w_crouch, 1.0 - stance_alpha)
		var half: float = full_width * 0.5
		var ax: float = float(_owner.get("_neutral_axis_sign"))
		lead_is_front = _owner.get("_lead_is_front")
		nxF = GCMath.neutral_target_x(cx, half, ax, lead_is_front, true)
		nxR = GCMath.neutral_target_x(cx, half, ax, lead_is_front, false)

	_owner.set("_dbg_neutral_front_x", nxF)
	_owner.set("_dbg_neutral_rear_x", nxR)

	var support_y_filt: float = _refs.support_y_filt
	var support_y_last_valid: float = _owner.get("_support_y_last_valid") as float
	var stance_height01: float = _refs.stance_height01
	var leg_len_crouch: float = _owner.get("leg_len_crouch") as float
	var leg_len_stand: float = _owner.get("leg_len_stand") as float

	var gy: float = support_y_filt
	if not is_finite(gy):
		gy = support_y_last_valid
	if not is_finite(gy) and rb_pelvis != null:
		var a: float = clampf(stance_height01, 0.0, 1.0)
		var leg_len_cmd: float = lerpf(leg_len_crouch, leg_len_stand, a)
		gy = rb_pelvis.global_position.y + leg_len_cmd

	var plan_target_front: Vector2 = Vector2(nxF, gy)
	var plan_target_rear: Vector2 = Vector2(nxR, gy)
	_owner.set("_plan_target_front", plan_target_front)
	_owner.set("_plan_target_rear", plan_target_rear)

	var torso_err_deg: float = 0.0
	if rb_torso != null:
		torso_err_deg = absf(rad_to_deg(wrapf(0.0 - rb_torso.global_rotation, -PI, PI)))

	var err_y: float = NAN
	if is_finite(gy) and rb_pelvis != null:
		var a2: float = clampf(stance_height01, 0.0, 1.0)
		var leg_len_cmd2: float = lerpf(leg_len_crouch, leg_len_stand, a2)
		var pelvis_target_y: float = gy - leg_len_cmd2
		err_y = pelvis_target_y - rb_pelvis.global_position.y

	var spawn_brace_active: bool = _owner.get("_spawn_brace_active")
	if spawn_brace_active:
		_owner.set("_planner_mode", GCTypes.PlannerMode.BRACE)
		_refs.planner_mode = GCTypes.PlannerMode.BRACE

		var impact_timer: float = _owner.get("_impact_timer") as float
		var plant_min_stability: float = _owner.get("plant_min_stability") as float
		var spawn_brace_ground_stable_t: float = _owner.get("_spawn_brace_ground_stable_t") as float
		var spawn_brace_elapsed: float = _owner.get("_spawn_brace_elapsed") as float

		var ground_stable_now: bool = grounded_eff \
			and (impact_timer <= 0.0) \
			and ((front_allow and stabF >= plant_min_stability) or (rear_allow and stabR >= plant_min_stability))

		if ground_stable_now:
			spawn_brace_ground_stable_t += dt
		else:
			spawn_brace_ground_stable_t = 0.0
		_owner.set("_spawn_brace_ground_stable_t", spawn_brace_ground_stable_t)

		var spawn_brace_exit_pelvis_err_px: float = _owner.get("spawn_brace_exit_pelvis_err_px") as float
		var spawn_brace_exit_torso_err_deg: float = _owner.get("spawn_brace_exit_torso_err_deg") as float
		var spawn_brace_min_sec: float = _owner.get("spawn_brace_min_sec") as float
		var spawn_brace_exit_ground_stable_sec: float = _owner.get("spawn_brace_exit_ground_stable_sec") as float
		var pelvis_ok: bool = is_finite(err_y) and (absf(err_y) <= spawn_brace_exit_pelvis_err_px)
		var torso_ok: bool = torso_err_deg <= spawn_brace_exit_torso_err_deg

		if (spawn_brace_elapsed >= spawn_brace_min_sec) \
			and (spawn_brace_ground_stable_t >= spawn_brace_exit_ground_stable_sec) \
			and pelvis_ok and torso_ok:
			_owner.set("_spawn_brace_active", false)
			_owner.set("_spawn_brace_ground_stable_t", 0.0)
			_owner.set("_planner_mode", GCTypes.PlannerMode.IDLE)
			_refs.planner_mode = GCTypes.PlannerMode.IDLE

		var spawn_brace_max_sec: float = _owner.get("spawn_brace_max_sec") as float
		var spawn_brace_hardcap_sec: float = _owner.get("spawn_brace_hardcap_sec") as float
		var max_sec_eff: float = clampf(maxf(spawn_brace_min_sec, spawn_brace_max_sec), 0.05, 2.0)
		var hardcap_eff: float = clampf(maxf(max_sec_eff, spawn_brace_hardcap_sec), max_sec_eff, 3.0)

		if grounded_eff and (spawn_brace_elapsed >= max_sec_eff) and spawn_brace_active:
			_owner.set("_spawn_brace_active", false)
			_owner.set("_planner_mode", GCTypes.PlannerMode.IDLE)
			_refs.planner_mode = GCTypes.PlannerMode.IDLE
		elif (spawn_brace_elapsed >= hardcap_eff) and spawn_brace_active:
			_owner.set("_spawn_brace_active", false)
			_owner.set("_planner_mode", GCTypes.PlannerMode.IDLE)
			_refs.planner_mode = GCTypes.PlannerMode.IDLE

		var eligible_front: bool = front_allow and is_finite(fx)
		var eligible_rear: bool = rear_allow and is_finite(rx)
		var plan_support_is_front: bool = choose_support_foot(front_allow, rear_allow, stabF, stabR, dt)
		_owner.set("_plan_support_is_front", plan_support_is_front)

		if plan_support_is_front:
			_owner.set("_plan_front", GCTypes.PlanFoot.PLANTED if eligible_front else GCTypes.PlanFoot.SWING)
			_owner.set("_plan_rear", GCTypes.PlanFoot.SWING)
			plan_target_rear.y = gy
			_owner.set("_plan_target_rear", plan_target_rear)
		else:
			_owner.set("_plan_rear", GCTypes.PlanFoot.PLANTED if eligible_rear else GCTypes.PlanFoot.SWING)
			_owner.set("_plan_front", GCTypes.PlanFoot.SWING)
			plan_target_front.y = gy
			_owner.set("_plan_target_front", plan_target_front)

		var phase3_force_single_support_during_impact: bool = _owner.get("phase3_force_single_support_during_impact")
		if phase3_force_single_support_during_impact and impact_timer > 0.0 and grounded_eff:
			if plan_support_is_front and eligible_front:
				_owner.set("_plan_front", GCTypes.PlanFoot.PLANTED)
				_owner.set("_plan_rear", GCTypes.PlanFoot.SWING)
			elif (not plan_support_is_front) and eligible_rear:
				_owner.set("_plan_front", GCTypes.PlanFoot.SWING)
				_owner.set("_plan_rear", GCTypes.PlanFoot.PLANTED)
		return

	var phase6_enable: bool = _owner.get("phase6_enable")
	if phase6_enable:
		var enter_recover: bool = false
		var cmd_move_x: float = _owner.get("cmd_move_x") as float
		var cmd_move_deadzone: float = _owner.get("cmd_move_deadzone") as float
		var phase7_enable: bool = _owner.get("phase7_enable")
		var input_meaningful_threshold: float = _owner.get("input_meaningful_threshold") as float
		var locomotion_cmd_active: bool = phase7_enable and (absf(cmd_move_x) >= input_meaningful_threshold)
		if absf(cmd_move_x) < cmd_move_deadzone:
			cmd_move_x = 0.0
		cmd_move_x = clampf(cmd_move_x, -1.0, 1.0)

		var phase6_air_enter_vy: float = _owner.get("phase6_air_enter_vy") as float
		var phase6_air_enter_torso_err_deg: float = _owner.get("phase6_air_enter_torso_err_deg") as float
		var phase6_enter_torso_err_deg: float = _owner.get("phase6_enter_torso_err_deg") as float
		var phase6_enter_pelvis_below_px: float = _owner.get("phase6_enter_pelvis_below_px") as float
		var phase6_enter_foot_neutral_err_px: float = _owner.get("phase6_enter_foot_neutral_err_px") as float
		var impact_timer: float = _owner.get("_impact_timer") as float

		if (not grounded_eff) and (rb_pelvis != null):
			var vy: float = rb_pelvis.linear_velocity.y
			if (vy >= phase6_air_enter_vy) and (torso_err_deg >= phase6_air_enter_torso_err_deg):
				enter_recover = true
		if torso_err_deg >= phase6_enter_torso_err_deg:
			enter_recover = true
		if is_finite(err_y) and (err_y < -phase6_enter_pelvis_below_px):
			enter_recover = true
		if (not locomotion_cmd_active) and front_allow and is_finite(fx) and is_finite(nxF) and (absf(fx - nxF) >= phase6_enter_foot_neutral_err_px):
			enter_recover = true
		if (not locomotion_cmd_active) and rear_allow and is_finite(rx) and is_finite(nxR) and (absf(rx - nxR) >= phase6_enter_foot_neutral_err_px):
			enter_recover = true
		if impact_timer > 0.0:
			enter_recover = true

		var planner_mode: int = _owner.get("_planner_mode")
		var phase6_min_active_sec: float = _owner.get("phase6_min_active_sec") as float
		var phase6_exit_stable_sec: float = _owner.get("phase6_exit_stable_sec") as float
		var phase6_exit_foot_neutral_err_px: float = _owner.get("phase6_exit_foot_neutral_err_px") as float
		var phase6_exit_pelvis_err_px: float = _owner.get("phase6_exit_pelvis_err_px") as float
		var phase6_exit_torso_err_deg: float = _owner.get("phase6_exit_torso_err_deg") as float
		var phase6_swing_lift_px: float = _owner.get("phase6_swing_lift_px") as float

		if enter_recover and planner_mode != GCTypes.PlannerMode.RECOVER:
			_owner.set("_planner_mode", GCTypes.PlannerMode.RECOVER)
			_refs.planner_mode = GCTypes.PlannerMode.RECOVER
			_owner.set("_phase6_recover_hold_t", phase6_min_active_sec)
			_owner.set("_phase6_recover_exit_t", 0.0)

		var phase6_recover_hold_t: float = _owner.get("_phase6_recover_hold_t") as float
		var phase6_recover_exit_t: float = _owner.get("_phase6_recover_exit_t") as float
		if planner_mode == GCTypes.PlannerMode.RECOVER:
			phase6_recover_hold_t = maxf(0.0, phase6_recover_hold_t - dt)
			_owner.set("_phase6_recover_hold_t", phase6_recover_hold_t)

			var feet_ok: bool = true
			if is_finite(fx) and is_finite(nxF):
				feet_ok = feet_ok and (absf(fx - nxF) <= phase6_exit_foot_neutral_err_px)
			if is_finite(rx) and is_finite(nxR):
				feet_ok = feet_ok and (absf(rx - nxR) <= phase6_exit_foot_neutral_err_px)
			var pelvis_ok2: bool = is_finite(err_y) and (absf(err_y) <= phase6_exit_pelvis_err_px)
			var torso_ok2: bool = torso_err_deg <= phase6_exit_torso_err_deg
			var exit_ok: bool = grounded_eff and feet_ok and pelvis_ok2 and torso_ok2 and (impact_timer <= 0.0)

			if exit_ok:
				phase6_recover_exit_t += dt
			else:
				phase6_recover_exit_t = 0.0
			_owner.set("_phase6_recover_exit_t", phase6_recover_exit_t)

			if (phase6_recover_hold_t <= 0.0) and (phase6_recover_exit_t >= phase6_exit_stable_sec):
				_owner.set("_planner_mode", GCTypes.PlannerMode.IDLE)
				_refs.planner_mode = GCTypes.PlannerMode.IDLE
				_owner.set("_phase6_recover_exit_t", 0.0)

		var support_is_front_local: bool = choose_support_foot(front_allow, rear_allow, stabF, stabR, dt)
		_owner.set("_plan_support_is_front", support_is_front_local)

		if planner_mode == GCTypes.PlannerMode.RECOVER:
			if support_is_front_local:
				_owner.set("_plan_front", GCTypes.PlanFoot.PLANTED if front_allow else GCTypes.PlanFoot.SWING)
				_owner.set("_plan_rear", GCTypes.PlanFoot.SWING)
				if not grounded_eff:
					plan_target_rear.y = plan_target_rear.y - phase6_swing_lift_px
				else:
					plan_target_rear.y = gy
				_owner.set("_plan_target_rear", plan_target_rear)
			else:
				_owner.set("_plan_rear", GCTypes.PlanFoot.PLANTED if rear_allow else GCTypes.PlanFoot.SWING)
				_owner.set("_plan_front", GCTypes.PlanFoot.SWING)
				if not grounded_eff:
					plan_target_front.y = plan_target_front.y - phase6_swing_lift_px
				else:
					plan_target_front.y = gy
				_owner.set("_plan_target_front", plan_target_front)
		return

	# IDLE standing plan
	_owner.set("_planner_mode", GCTypes.PlannerMode.IDLE)
	_refs.planner_mode = GCTypes.PlannerMode.IDLE

	var phase3_plant_x_window_px: float = _owner.get("phase3_plant_x_window_px") as float
	var phase3_min_swing_sec: float = _owner.get("phase3_min_swing_sec") as float
	var phase3_min_planted_sec: float = _owner.get("phase3_min_planted_sec") as float
	var phase6_single_support_on_stance_change: bool = _owner.get("phase6_single_support_on_stance_change")
	lead_is_front = _owner.get("_lead_is_front")

	var eligible_front_idle: bool = front_g and (stabF >= (_owner.get("plant_min_stability") as float)) and is_finite(nxF) and is_finite(fx) \
		and (absf(fx - nxF) <= phase3_plant_x_window_px)
	var eligible_rear_idle: bool = rear_g and (stabR >= (_owner.get("plant_min_stability") as float)) and is_finite(nxR) and is_finite(rx) \
		and (absf(rx - nxR) <= phase3_plant_x_window_px)

	var force_single_support: bool = phase6_single_support_on_stance_change and stance_changing
	var plan_front: int = _owner.get("_plan_front")
	var plan_rear: int = _owner.get("_plan_rear")
	var desired_front: int = plan_front
	var desired_rear: int = plan_rear

	if eligible_front_idle and (plan_front_age >= phase3_min_swing_sec):
		desired_front = GCTypes.PlanFoot.PLANTED
	if eligible_rear_idle and (plan_rear_age >= phase3_min_swing_sec):
		desired_rear = GCTypes.PlanFoot.PLANTED

	if grounded_eff and (not eligible_front_idle) and (plan_front_age >= phase3_min_planted_sec) and (_owner.get("_impact_timer") as float <= 0.0):
		desired_front = GCTypes.PlanFoot.SWING
	if grounded_eff and (not eligible_rear_idle) and (plan_rear_age >= phase3_min_planted_sec) and (_owner.get("_impact_timer") as float <= 0.0):
		desired_rear = GCTypes.PlanFoot.SWING

	var support_is_front_local: bool = choose_support_foot(front_allow, rear_allow, stabF, stabR, dt)
	_owner.set("_plan_support_is_front", support_is_front_local)

	if force_single_support:
		var stance_plant_is_front: bool = lead_is_front
		if stance_plant_is_front and (not front_allow) and rear_allow:
			stance_plant_is_front = false
		elif (not stance_plant_is_front) and (not rear_allow) and front_allow:
			stance_plant_is_front = true
		if stance_plant_is_front:
			desired_front = GCTypes.PlanFoot.PLANTED if front_allow else GCTypes.PlanFoot.SWING
			desired_rear = GCTypes.PlanFoot.SWING
			plan_target_rear.y = gy
			_owner.set("_plan_target_rear", plan_target_rear)
		else:
			desired_rear = GCTypes.PlanFoot.PLANTED if rear_allow else GCTypes.PlanFoot.SWING
			desired_front = GCTypes.PlanFoot.SWING
			plan_target_front.y = gy
			_owner.set("_plan_target_front", plan_target_front)

	var phase3_force_single_support_during_impact: bool = _owner.get("phase3_force_single_support_during_impact")
	var impact_timer: float = _owner.get("_impact_timer") as float
	if phase3_force_single_support_during_impact and impact_timer > 0.0 and grounded_eff:
		if support_is_front_local and eligible_front_idle:
			desired_front = GCTypes.PlanFoot.PLANTED
			desired_rear = GCTypes.PlanFoot.SWING
		elif (not support_is_front_local) and eligible_rear_idle:
			desired_front = GCTypes.PlanFoot.SWING
			desired_rear = GCTypes.PlanFoot.PLANTED

	if desired_front != plan_front:
		_owner.set("_plan_front", desired_front)
		_owner.set("_plan_front_age", 0.0)
	if desired_rear != plan_rear:
		_owner.set("_plan_rear", desired_rear)
		_owner.set("_plan_rear_age", 0.0)
