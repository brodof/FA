# res://scripts/duel/gladiator_controller/gc_vertical_support.gd
# Vertical support: support Y raw/filter, leg length calibration, apply vertical force.
# All "keep body supported vertically" lives here. Uses owner/refs for state.
class_name GCVerticalSupport
extends RefCounted

var _owner: Node
var _refs: GCRefs

# Filter state (persisted across frames)
var _sy_filt: float = NAN

func setup(owner: Node, refs: GCRefs) -> void:
	_owner = owner
	_refs = refs

func reset() -> void:
	_sy_filt = NAN

func tick(_dt: float) -> void:
	# Intentionally unused; controller calls explicit methods.
	pass

func compute_support_y_raw(front_g: bool, rear_g: bool) -> float:
	var support_is_front: bool = _owner.get("support_is_front")
	var plant_front_active: bool = _owner.get("_plant_front_active")
	var plant_rear_active: bool = _owner.get("_plant_rear_active")
	if support_is_front and plant_front_active:
		var pyf: float = _refs.plant_support_y_front
		if is_finite(pyf):
			return pyf
	if (not support_is_front) and plant_rear_active:
		var pyr: float = _refs.plant_support_y_rear
		if is_finite(pyr):
			return pyr
	var truth_front_y: float = _owner.get("_truth_front_y") as float
	var truth_rear_y: float = _owner.get("_truth_rear_y") as float
	var yF: float = truth_front_y if front_g and is_finite(truth_front_y) else NAN
	var yR: float = truth_rear_y if rear_g and is_finite(truth_rear_y) else NAN
	if front_g and rear_g:
		return minf(yF, yR)
	if front_g:
		return yF
	if rear_g:
		return yR
	return NAN

func filter_support_y(raw_y: float, dt: float) -> float:
	if not is_finite(raw_y):
		return NAN
	if not is_finite(_sy_filt):
		_sy_filt = raw_y
		return _sy_filt
	var hz: float = _owner.get("support_contact_smooth_hz") as float
	var a: float = clampf(dt * hz, 0.0, 1.0)
	_sy_filt = lerpf(_sy_filt, raw_y, a)
	return _sy_filt

func exp_smooth(cur: float, target: float, hz: float, dt: float) -> float:
	if not is_finite(target):
		return cur
	if not is_finite(cur):
		return target
	if hz <= 0.0 or dt <= 0.0:
		return target
	var a: float = 1.0 - exp(-dt * hz)
	return lerpf(cur, target, clampf(a, 0.0, 1.0))

# Brief 5: STEP_PLANNER body-support ownership gate.
# Vertical support is a BODY subsystem and must follow committed per-foot arbitration ownership,
# not legacy planner labels. In STEP_PLANNER mode, only these modes may contribute support.
func _ctrl_mode_allows_vertical_support(ctrl_mode: int) -> bool:
	return ctrl_mode == GCTypes.FootControlMode.PLANTED_SUPPORT \
		or ctrl_mode == GCTypes.FootControlMode.PLANTED_CORRECT \
		or ctrl_mode == GCTypes.FootControlMode.RECOVERY

func update_leg_length_calibration(_support_y_filt: float, leg_len_cmd: float, _dt: float) -> float:
	var rest_captured: bool = _owner.get("_rest_captured")
	var leg_len_cal: float = _owner.get("_leg_len_cal") as float
	if not rest_captured:
		return leg_len_cal
	var rest_capture_time_since: float = _owner.get("_rest_capture_time_since") as float
	var leg_len_cal_after_capture_sec: float = _owner.get("leg_len_cal_after_capture_sec") as float
	var blend: float = clampf(rest_capture_time_since / maxf(0.001, leg_len_cal_after_capture_sec), 0.0, 1.0)
	return lerpf(leg_len_cal, leg_len_cmd, blend)

func support_y_from_support_foot(front_g: bool, rear_g: bool) -> float:
	var support_is_front: bool = _owner.get("support_is_front")
	var foot_sensor_front: Node = _refs.foot_sensor_front
	var foot_sensor_rear: Node = _refs.foot_sensor_rear
	var sup: Node = foot_sensor_front if support_is_front else foot_sensor_rear
	if sup != null and sup.get("grounded"):
		var yref: float = sup.get("contact_y_raw") if is_finite(sup.get("contact_y_raw") as float) else (sup.get("contact_y") as float)
		if is_finite(yref):
			if absf(yref - (sup as Node2D).global_position.y) <= 256.0:
				return yref
	if front_g and foot_sensor_front != null:
		var yF: float = foot_sensor_front.get("contact_y_raw") as float
		if not is_finite(yF):
			yF = foot_sensor_front.get("contact_y") as float
		if is_finite(yF) and absf(yF - (foot_sensor_front as Node2D).global_position.y) <= 256.0:
			return yF
	if rear_g and foot_sensor_rear != null:
		var yR: float = foot_sensor_rear.get("contact_y_raw") as float
		if not is_finite(yR):
			yR = foot_sensor_rear.get("contact_y") as float
		if is_finite(yR) and absf(yR - (foot_sensor_rear as Node2D).global_position.y) <= 256.0:
			return yR
	return NAN

func ground_y() -> float:
	if is_finite(_refs.support_y_filt):
		return _refs.support_y_filt
	var last_valid: float = _owner.get("_support_y_last_valid") as float
	if is_finite(last_valid):
		return last_valid
	if _refs.rb_pelvis != null:
		return _refs.rb_pelvis.global_position.y
	return 0.0

func apply_vertical_support(support_y_filt: float, leg_len_cmd: float, dt: float, spawn01: float, force_scale: float) -> void:
	_owner.set("_dbg_target_y", NAN)
	_owner.set("_dbg_err_y", 0.0)
	_owner.set("_dbg_vy_for_pd", 0.0)
	_owner.set("_dbg_Fy_pre", 0.0)
	_owner.set("_dbg_Fy_cmd", 0.0)
	_owner.set("_dbg_Fy_max", 1.0)
	_owner.set("_dbg_vsupport_called", false)
	_owner.set("_dbg_sink_allow", 0.0)
	_owner.set("_dbg_vsupport_gate", 0.0)

	var rb_pelvis: RigidBody2D = _refs.rb_pelvis
	if rb_pelvis == null or dt <= 0.0:
		_owner.set("_vsupport_Fy_prev", 0.0)
		return
	if not is_finite(support_y_filt) or not is_finite(leg_len_cmd):
		_owner.set("_vsupport_Fy_prev", 0.0)
		return

	var total_mass: float = _refs.total_mass
	var g: float = _refs.g
	var support_force_slew_g_per_sec: float = _owner.get("support_force_slew_g_per_sec") as float
	var gate: float = clampf(force_scale * spawn01, 0.0, 1.0)
	_owner.set("_dbg_vsupport_gate", gate)

	if gate <= 0.0001:
		var m_bleed: float = maxf(0.01, total_mass if total_mass > 0.0 else rb_pelvis.mass)
		var bleed: float = (m_bleed * g) * support_force_slew_g_per_sec * dt
		var prev: float = _owner.get("_vsupport_Fy_prev") as float
		_owner.set("_vsupport_Fy_prev", move_toward(prev, 0.0, bleed))
		return

	var truth_front_allow_t: float = _owner.get("_truth_front_allow_t") as float
	var truth_rear_allow_t: float = _owner.get("_truth_rear_allow_t") as float
	var truth_front_g: bool = _owner.get("_truth_front_g")
	var truth_rear_g: bool = _owner.get("_truth_rear_g")
	var allowF: bool = (truth_front_allow_t > 0.0) and (_refs.rb_foot_front != null)
	var allowR: bool = (truth_rear_allow_t > 0.0) and (_refs.rb_foot_rear != null)
	var groundedF: bool = allowF and truth_front_g
	var groundedR: bool = allowR and truth_rear_g

	var ma: int = int(_owner.get("movement_authority_mode"))
	var step_planner_authority: bool = (ma == GCTypes.MovementAuthorityMode.STEP_PLANNER)

	var useF: bool = false
	var useR: bool = false

	if step_planner_authority:
		# Brief 5 ownership cutover:
		# BODY support must use committed arbitration outputs when available.
		# Startup fallback (first frame / commit unavailable) may use grounded truth only.
		var arb_valid: bool = bool(_owner.get("_arb_ctrl_modes_valid"))
		if arb_valid:
			var front_ctrl_mode: int = int(_owner.get("_arb_front_ctrl_mode"))
			var rear_ctrl_mode: int = int(_owner.get("_arb_rear_ctrl_mode"))
			useF = groundedF and _ctrl_mode_allows_vertical_support(front_ctrl_mode)
			useR = groundedR and _ctrl_mode_allows_vertical_support(rear_ctrl_mode)
		else:
			# Fallback only before committed ownership exists.
			useF = groundedF
			useR = groundedR
	else:
		# Legacy path preserved for LEGACY_PHASE7 mode.
		var plan_front: int = _owner.get("_plan_front")
		var plan_rear: int = _owner.get("_plan_rear")
		var p7_exec: int = _owner.get("_phase7_exec_phase")
		var p7_swinging: bool = _owner.get("phase7_enable") and (p7_exec == GCTypes.Phase7ExecPhase.UNPLANT or p7_exec == GCTypes.Phase7ExecPhase.SWING)

		useF = groundedF and (plan_front == GCTypes.PlanFoot.PLANTED)
		useR = groundedR and (plan_rear == GCTypes.PlanFoot.PLANTED)
		if not p7_swinging:
			if (not useF) and groundedF:
				useF = true
			if (not useR) and groundedR:
				useR = true

	if not useF and not useR:
		var m_bleed2: float = maxf(0.01, total_mass if total_mass > 0.0 else rb_pelvis.mass)
		var bleed2: float = (m_bleed2 * g) * support_force_slew_g_per_sec * dt
		var prev2: float = _owner.get("_vsupport_Fy_prev") as float
		_owner.set("_vsupport_Fy_prev", move_toward(prev2, 0.0, bleed2))
		return

	_owner.set("_dbg_vsupport_called", true)

	var truth_front_stab: float = _owner.get("_truth_front_stab") as float
	var truth_rear_stab: float = _owner.get("_truth_rear_stab") as float
	var wF: float = clampf(truth_front_stab, 0.0, 1.0) if useF else 0.0
	var wR: float = clampf(truth_rear_stab, 0.0, 1.0) if useR else 0.0
	if (wF + wR) < 0.001:
		if useF and not useR:
			wF = 1.0
		elif useR and not useF:
			wR = 1.0
		else:
			wF = 0.5
			wR = 0.5
	var wsum: float = wF + wR
	wF /= wsum
	wR /= wsum

	var leg_len_cal_lerp_hz: float = _owner.get("leg_len_cal_lerp_hz") as float
	var leg_len_cal: float = _owner.get("_leg_len_cal") as float
	leg_len_cal = exp_smooth(leg_len_cal, leg_len_cmd, leg_len_cal_lerp_hz, dt)
	_owner.set("_leg_len_cal", leg_len_cal)

	var landing_t: float = _owner.get("_landing_t") as float
	var landing_ramp_sec: float = _owner.get("landing_ramp_sec") as float
	var dbg_last_touch_vy: float = _owner.get("_dbg_last_touch_vy") as float
	var landing_sink_vy_ref: float = _owner.get("landing_sink_vy_ref") as float
	var landing_sink_allow_px: float = _owner.get("landing_sink_allow_px") as float
	var landing_sink_extra_px: float = _owner.get("landing_sink_extra_px") as float
	var landing_pelvis_clearance_px: float = _owner.get("landing_pelvis_clearance_px") as float
	var support_y_deadband_px: float = _owner.get("support_y_deadband_px") as float
	var cmd_stance_y_eff: float = _owner.get("_cmd_stance_y_eff") as float
	var support_err_up_max_px: float = _owner.get("support_err_up_max_px") as float
	var support_err_up_max_px_shift: float = _owner.get("support_err_up_max_px_shift") as float
	var support_err_down_max_px: float = _owner.get("support_err_down_max_px") as float
	var support_err_down_max_px_shift: float = _owner.get("support_err_down_max_px_shift") as float
	var support_freq_hz: float = _owner.get("support_freq_hz") as float
	var support_zeta: float = _owner.get("support_zeta") as float
	var support_damp_vy_max: float = _owner.get("support_damp_vy_max") as float
	var support_damp_vy_max_shift: float = _owner.get("support_damp_vy_max_shift") as float
	var impact_timer: float = _owner.get("_impact_timer") as float
	var support_damp_vy_max_during_impact: float = _owner.get("support_damp_vy_max_during_impact") as float
	var stance_alpha: float = _refs.stance_alpha
	var support_gravity_comp: float = _owner.get("support_gravity_comp") as float
	var support_gravity_comp_stand_mult: float = _owner.get("support_gravity_comp_stand_mult") as float
	var support_gravity_comp_crouch_mult: float = _owner.get("support_gravity_comp_crouch_mult") as float
	var support_force_mult: float = _owner.get("support_force_mult") as float
	var landing_impact_vy_ref: float = _owner.get("landing_impact_vy_ref") as float
	var landing_impact_cap_boost_g: float = _owner.get("landing_impact_cap_boost_g") as float
	var support_down_mult: float = _owner.get("support_down_mult") as float
	var stand_down_mult: float = _owner.get("stand_down_mult") as float
	var support_down_mult_shift: float = _owner.get("support_down_mult_shift") as float

	var py: float = rb_pelvis.global_position.y
	var vy: float = rb_pelvis.linear_velocity.y
	var desired_y: float = support_y_filt - leg_len_cal
	var landing_alpha: float = clampf(landing_t / maxf(0.001, landing_ramp_sec), 0.0, 1.0)
	var impact01: float = clampf(dbg_last_touch_vy / maxf(1.0, landing_sink_vy_ref), 0.0, 1.0)
	var sink_allow: float = (landing_sink_allow_px + landing_sink_extra_px * impact01) * (1.0 - landing_alpha)
	_owner.set("_dbg_sink_allow", sink_allow)
	var max_target_y: float = support_y_filt - landing_pelvis_clearance_px
	var target_y: float = minf(desired_y + sink_allow, max_target_y)
	var err: float = target_y - py
	if absf(err) <= support_y_deadband_px:
		err = 0.0
	var shifting01: float = clampf(absf(cmd_stance_y_eff), 0.0, 1.0)
	var err_up: float = lerpf(support_err_up_max_px, support_err_up_max_px_shift, shifting01)
	var err_dn: float = lerpf(support_err_down_max_px, support_err_down_max_px_shift, shifting01)
	err = clampf(err, -err_up, err_dn)

	var m_eff: float = maxf(0.01, total_mass if total_mass > 0.0 else rb_pelvis.mass)
	var w: float = TAU * maxf(0.05, support_freq_hz)
	var k: float = w * w
	var d: float = 2.0 * support_zeta * w
	var vy_lim: float = support_damp_vy_max
	if shifting01 > 0.0:
		vy_lim = maxf(vy_lim, support_damp_vy_max_shift)
	if impact_timer > 0.0:
		vy_lim = maxf(vy_lim, support_damp_vy_max_during_impact)
	var vy_pd: float = clampf(vy, -vy_lim, vy_lim)
	var crouch01: float = 1.0 - clampf(stance_alpha, 0.0, 1.0)
	var gc: float = support_gravity_comp * lerpf(support_gravity_comp_stand_mult, support_gravity_comp_crouch_mult, crouch01)
	var a_pd: float = (k * err) - (d * vy_pd)
	var a_gc: float = -g * gc
	var Fy_pre: float = m_eff * (a_pd + a_gc)

	var Fy_max_up: float = m_eff * g * maxf(0.0, support_force_mult)
	if impact_timer > 0.0:
		var boost01: float = clampf(dbg_last_touch_vy / maxf(1.0, landing_impact_vy_ref), 0.0, 1.0)
		Fy_max_up += (m_eff * g * landing_impact_cap_boost_g * boost01)
	var down_mult_eff: float = maxf(support_down_mult, stand_down_mult)
	if shifting01 > 0.0:
		down_mult_eff = maxf(down_mult_eff, support_down_mult_shift)
	var Fy_max_dn: float = m_eff * g * down_mult_eff
	var Fy_clamped: float = clampf(Fy_pre, -Fy_max_up, Fy_max_dn)
	var Fy_goal: float = Fy_clamped * gate
	var dFy_max: float = (m_eff * g * support_force_slew_g_per_sec) * dt
	var vsupport_prev: float = _owner.get("_vsupport_Fy_prev") as float
	var Fy_cmd: float = move_toward(vsupport_prev, Fy_goal, dFy_max)
	_owner.set("_vsupport_Fy_prev", Fy_cmd)

	_owner.set("_dbg_target_y", target_y)
	_owner.set("_dbg_err_y", target_y - py)
	_owner.set("_dbg_vy_for_pd", vy_pd)
	_owner.set("_dbg_Fy_pre", Fy_pre)
	_owner.set("_dbg_Fy_cmd", Fy_cmd)
	_owner.set("_dbg_Fy_max", maxf(1.0, maxf(Fy_max_up, Fy_max_dn)))

	rb_pelvis.apply_central_force(Vector2(0.0, Fy_cmd))
	if Fy_cmd < 0.0:
		var down_to_feet: float = -Fy_cmd
		if useF and _refs.rb_foot_front != null:
			_refs.rb_foot_front.apply_central_force(Vector2(0.0, down_to_feet * wF))
		if useR and _refs.rb_foot_rear != null:
			_refs.rb_foot_rear.apply_central_force(Vector2(0.0, down_to_feet * wR))
