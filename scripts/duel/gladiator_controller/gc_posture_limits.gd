# res://scripts/duel/gladiator_controller/gc_posture_limits.gd
# Posture PD + soft joint limits. Stand stabilization (Phase 1B), wrap/rel angle, limit deltas.
class_name GCPostureLimits
extends RefCounted

var _owner: Node
var _refs: GCRefs

# --- Slot model calibration (Brief 3 rebuilt) ---
# We derive stance-slot asymmetry from authored posture targets + calibrated rig geometry.
# Calibration stores pelvis-frame COM-link vectors and baseline joint rel angles.
var _slot_cal_valid: bool = false

var _slot_cal_p2t_F_pf: Vector2 = Vector2.ZERO
var _slot_cal_t2s_F_pf: Vector2 = Vector2.ZERO
var _slot_cal_s2f_F_pf: Vector2 = Vector2.ZERO
var _slot_cal_p2t_R_pf: Vector2 = Vector2.ZERO
var _slot_cal_t2s_R_pf: Vector2 = Vector2.ZERO
var _slot_cal_s2f_R_pf: Vector2 = Vector2.ZERO

var _slot_cal_hip_rel_F: float = 0.0
var _slot_cal_knee_rel_F: float = 0.0
var _slot_cal_ankle_rel_F: float = 0.0
var _slot_cal_hip_rel_R: float = 0.0
var _slot_cal_knee_rel_R: float = 0.0
var _slot_cal_ankle_rel_R: float = 0.0

func setup(owner: Node, refs: GCRefs) -> void:
	_owner = owner
	_refs = refs

func reset() -> void:
	_slot_cal_valid = false

func tick(_dt: float) -> void:
	# Intentionally unused; controller calls explicit methods.
	pass

func limit_probe() -> String:
	var o := _owner
	if o == null:
		return ""

	if not o.get("enable_soft_joint_limits"):
		return ""

	var best_name: String = ""
	var best_excess: float = 0.0
	var excess: float = 0.0

	excess = limit_excess(o.get("_rb_pelvis") as RigidBody2D, o.get("_rb_thigh_front") as RigidBody2D, o.get("rest_pelvis_thigh_F") as float, o.get("limit_hip_neg_deg") as float, o.get("limit_hip_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "HIP_F"

	excess = limit_excess(o.get("_rb_pelvis") as RigidBody2D, o.get("_rb_thigh_rear") as RigidBody2D, o.get("rest_pelvis_thigh_R") as float, o.get("limit_hip_neg_deg") as float, o.get("limit_hip_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "HIP_R"

	excess = limit_excess(o.get("_rb_thigh_front") as RigidBody2D, o.get("_rb_shin_front") as RigidBody2D, o.get("rest_thigh_shin_F") as float, o.get("limit_knee_neg_deg") as float, o.get("limit_knee_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "KNEE_F"

	excess = limit_excess(o.get("_rb_thigh_rear") as RigidBody2D, o.get("_rb_shin_rear") as RigidBody2D, o.get("rest_thigh_shin_R") as float, o.get("limit_knee_neg_deg") as float, o.get("limit_knee_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "KNEE_R"

	excess = limit_excess(o.get("_rb_shin_front") as RigidBody2D, o.get("_rb_foot_front") as RigidBody2D, o.get("rest_shin_foot_F") as float, o.get("limit_ankle_neg_deg") as float, o.get("limit_ankle_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "ANKLE_F"

	excess = limit_excess(o.get("_rb_shin_rear") as RigidBody2D, o.get("_rb_foot_rear") as RigidBody2D, o.get("rest_shin_foot_R") as float, o.get("limit_ankle_neg_deg") as float, o.get("limit_ankle_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "ANKLE_R"

	excess = limit_excess(o.get("_rb_pelvis") as RigidBody2D, o.get("_rb_torso") as RigidBody2D, o.get("rest_pelvis_torso") as float, o.get("limit_spine_neg_deg") as float, o.get("limit_spine_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "SPINE"

	excess = limit_excess(o.get("_rb_torso") as RigidBody2D, o.get("_rb_head") as RigidBody2D, o.get("rest_torso_head") as float, o.get("limit_neck_neg_deg") as float, o.get("limit_neck_pos_deg") as float, o.get("limit_margin_deg") as float)
	if excess > best_excess:
		best_excess = excess
		best_name = "NECK"

	if best_name == "":
		return ""
	return best_name + ":" + str(snappedf(rad_to_deg(best_excess), 0.1)) + "deg"

func limit_excess(
	parent_rb: RigidBody2D,
	child_rb: RigidBody2D,
	rest_rel: float,
	neg_deg: float,
	pos_deg: float,
	limit_margin_deg: float
) -> float:
	if parent_rb == null or child_rb == null:
		return 0.0

	var rel_ang: float = wrapf(child_rb.global_rotation - parent_rb.global_rotation, -PI, PI)
	var delta: float = wrapf(rel_ang - rest_rel, -PI, PI)
	var min_d: float = -deg_to_rad(absf(neg_deg) + limit_margin_deg)
	var max_d: float = deg_to_rad(absf(pos_deg) + limit_margin_deg)

	if delta < min_d:
		return (min_d - delta)
	if delta > max_d:
		return (delta - max_d)
	return 0.0
func wrap_angle(a: float) -> float:
	return GCMath.wrap_angle(a)

func rel_angle(parent: RigidBody2D, child: RigidBody2D) -> float:
	return GCMath.rel_angle(parent, child)

func apply_angle_pd(rb: RigidBody2D, target_angle: float, k: float, d: float, max_tau: float, tau_scale: float) -> void:
	GCMath.apply_angle_pd(rb, target_angle, k, d, max_tau, tau_scale)

func apply_torso_world_upright(_dt: float, spawn01: float) -> void:
	var rb_torso: RigidBody2D = _refs.rb_torso
	if rb_torso == null:
		return
	var err: float = wrapf(0.0 - rb_torso.global_rotation, -PI, PI)
	var r: float = _owner.get("torso_inertia_radius_px") as float
	var I: float = rb_torso.mass * r * r
	var w: float = TAU * (_owner.get("torso_upright_freq_hz") as float)
	var k: float = I * w * w
	var d: float = 2.0 * I * w * (_owner.get("torso_upright_zeta") as float)
	var tau: float = (k * err) - (d * rb_torso.angular_velocity)
	tau = clampf(tau, -(_owner.get("torso_upright_tau_max") as float), _owner.get("torso_upright_tau_max") as float)
	rb_torso.apply_torque(tau * spawn01)

func apply_joint_pd_child_only(parent_rb: RigidBody2D, child_rb: RigidBody2D, target_rel: float, k: float, d: float, max_tau: float, scale: float, _parent_share: float = 0.10) -> void:
	if parent_rb == null or child_rb == null or scale <= 0.0:
		return
	var rel: float = wrapf(child_rb.global_rotation - parent_rb.global_rotation, -PI, PI)
	var err: float = wrapf(target_rel - rel, -PI, PI)
	var rel_w: float = child_rb.angular_velocity - parent_rb.angular_velocity
	var tau: float = (k * err) - (d * rel_w)
	tau = clampf(tau, -max_tau, max_tau) * scale
	child_rb.apply_torque(tau)
	parent_rb.apply_torque(-tau)

func limit_deltas_from_rest(rest_rel: float, neg_deg: float, pos_deg: float) -> Vector2:
	var min_rel: float = -deg_to_rad(neg_deg)
	var max_rel: float = deg_to_rad(pos_deg)
	var a: float = min_rel - rest_rel
	var b: float = max_rel - rest_rel
	if a <= b:
		return Vector2(a, b)
	return Vector2(b, a)

func apply_rel_limit_soft(parent_rb: RigidBody2D, child_rb: RigidBody2D, rest_rel: float, min_delta: float, max_delta: float, k: float, d: float, max_tau: float, scale: float, _parent_share: float = 0.10) -> void:
	if parent_rb == null or child_rb == null or scale <= 0.0:
		return
	var rel: float = wrapf(child_rb.global_rotation - parent_rb.global_rotation, -PI, PI)
	var delta: float = wrapf(rel - rest_rel, -PI, PI)
	var limit_margin_deg: float = _owner.get("limit_margin_deg") as float
	var margin: float = deg_to_rad(maxf(0.0, limit_margin_deg))
	var excess: float = 0.0
	if delta < (min_delta - margin):
		excess = delta - (min_delta - margin)
	elif delta > (max_delta + margin):
		excess = delta - (max_delta + margin)
	else:
		return
	var rel_w: float = child_rb.angular_velocity - parent_rb.angular_velocity
	var spring: float = -k * excess
	var damper: float = d * rel_w
	var tau: float = (spring - damper)
	tau = clampf(tau, -max_tau, max_tau) * scale
	child_rb.apply_torque(tau)
	parent_rb.apply_torque(-tau)

func apply_rel_limit_soft_abs0(label: String, child_rb: RigidBody2D, parent_rb: RigidBody2D, zero_rel: float, min_abs0: float, max_abs0: float, k: float, d: float, max_tau: float, limit_scale: float) -> void:
	if child_rb == null or parent_rb == null or limit_scale <= 0.0:
		return
	var rel: float = wrapf(child_rb.global_rotation - parent_rb.global_rotation, -PI, PI)
	var rel0: float = wrapf(rel - zero_rel, -PI, PI)
	if min_abs0 >= 0.0:
		if rel0 < 0.0 and absf(rel0) > (PI * 0.75):
			rel0 += TAU
	elif max_abs0 <= 0.0:
		if rel0 > 0.0 and absf(rel0) > (PI * 0.75):
			rel0 -= TAU
	var limit_oneway_neg_slack_deg: float = _owner.get("limit_oneway_neg_slack_deg") as float
	if limit_oneway_neg_slack_deg > 0.0:
		var slack: float = deg_to_rad(limit_oneway_neg_slack_deg)
		if min_abs0 >= 0.0:
			if rel0 < min_abs0 and rel0 > (min_abs0 - slack):
				rel0 = min_abs0
		elif max_abs0 <= 0.0:
			if rel0 > max_abs0 and rel0 < (max_abs0 + slack):
				rel0 = max_abs0
	var excess: float = 0.0
	if rel0 < min_abs0:
		excess = rel0 - min_abs0
	elif rel0 > max_abs0:
		excess = rel0 - max_abs0
	else:
		if _owner.get("debug_enable") and (_owner.get("_dbg_lim") as Dictionary).has(label):
			(_owner.get("_dbg_lim") as Dictionary).erase(label)
		return
	var rel_w: float = child_rb.angular_velocity - parent_rb.angular_velocity
	var phase5_sat_ratio: float = _owner.get("phase5_sat_ratio") as float
	var tau_raw: float = (-k * excess) - (d * rel_w)
	if absf(tau_raw) >= (max_tau * phase5_sat_ratio):
		_owner.set("_phase5_limit_saturated", true)
	var tau: float = clampf(tau_raw, -max_tau, max_tau) * limit_scale
	if _owner.get("debug_enable"):
		var dbg_lim: Dictionary = _owner.get("_dbg_lim") as Dictionary
		dbg_lim[label] = {"rel0_deg": rad_to_deg(rel0), "min_deg": rad_to_deg(min_abs0), "max_deg": rad_to_deg(max_abs0), "excess_deg": rad_to_deg(excess), "tau": tau, "w": rel_w}
	child_rb.apply_torque(tau)
	parent_rb.apply_torque(-tau)

func apply_soft_limits_phase1b(scale: float) -> void:
	if scale <= 0.0:
		return
	var ground_grace_t: float = _owner.get("_ground_grace_t") as float
	var support_blend: float = _owner.get("_support_blend") as float
	var grounded_eff: bool = (ground_grace_t > 0.0)
	var support_gate: float = clampf(support_blend, 0.0, 1.0)
	var non_knee_gate: float = support_gate
	if grounded_eff:
		non_knee_gate = maxf(non_knee_gate, _owner.get("landing_limit_min") as float)
	var impact_timer: float = _owner.get("_impact_timer") as float
	if impact_timer > 0.0:
		non_knee_gate *= _owner.get("impact_non_knee_limit_relax_mult") as float
	var limit_scale: float = scale * non_knee_gate
	var knee_gate: float = maxf(support_gate, _owner.get("knee_limit_min_gate_pre_support") as float)
	var touchdown_ramp_t: float = _owner.get("_touchdown_ramp_t") as float
	if grounded_eff and touchdown_ramp_t < (_owner.get("knee_touchdown_full_sec") as float):
		knee_gate = 1.0
	if impact_timer > 0.0:
		knee_gate = 1.0
	var knee_scale: float = scale * knee_gate
	# Phase 7: reduce soft limit authority on the swing leg so it doesn't fight the step target.
	var swing_limit_scale_F: float = 1.0
	var swing_limit_scale_R: float = 1.0
	if _owner.get("phase7_enable") and _owner.get("_phase7_plan_active"):
		var p7_exec: int = _owner.get("_phase7_exec_phase")
		if p7_exec == GCTypes.Phase7ExecPhase.UNPLANT or p7_exec == GCTypes.Phase7ExecPhase.SWING:
			var s: float = clampf(_owner.get("phase7_swing_posture_scale") as float, 0.0, 1.0)
			if _owner.get("_phase7_exec_is_front"):
				swing_limit_scale_F = s
			else:
				swing_limit_scale_R = s
	var limit_scale_F: float = limit_scale * swing_limit_scale_F
	var limit_scale_R: float = limit_scale * swing_limit_scale_R
	var knee_scale_F: float = knee_scale * swing_limit_scale_F
	var knee_scale_R: float = knee_scale * swing_limit_scale_R
	var k: float = _owner.get("limit_k") as float
	var dd: float = _owner.get("limit_d") as float
	var max_tau: float = (_owner.get("max_limb_tau") as float) * 1.35
	var max_spine_tau: float = _owner.get("max_spine_posture_tau") as float
	var enable_soft_joint_limits: bool = _owner.get("enable_soft_joint_limits")
	var knee_zero_valid: bool = _owner.get("_knee_zero_valid")
	var knee_flex_sign_F: int = _owner.get("_knee_flex_sign_F")
	var knee_flex_sign_R: int = _owner.get("_knee_flex_sign_R")
	var limit_hip_neg_deg: float = _owner.get("limit_hip_neg_deg") as float
	var limit_hip_pos_deg: float = _owner.get("limit_hip_pos_deg") as float
	var limit_knee_neg_deg: float = _owner.get("limit_knee_neg_deg") as float
	var limit_knee_pos_deg: float = _owner.get("limit_knee_pos_deg") as float
	var limit_ankle_neg_deg: float = _owner.get("limit_ankle_neg_deg") as float
	var limit_ankle_pos_deg: float = _owner.get("limit_ankle_pos_deg") as float
	var rest_pelvis_thigh_F: float = _owner.get("rest_pelvis_thigh_F") as float
	var rest_pelvis_thigh_R: float = _owner.get("rest_pelvis_thigh_R") as float
	var _rest_thigh_shin_F: float = _owner.get("rest_thigh_shin_F") as float
	var _rest_thigh_shin_R: float = _owner.get("rest_thigh_shin_R") as float
	var rest_shin_foot_F: float = _owner.get("rest_shin_foot_F") as float
	var rest_shin_foot_R: float = _owner.get("rest_shin_foot_R") as float
	var knee_zero_F: float = _owner.get("_knee_zero_F") as float
	var knee_zero_R: float = _owner.get("_knee_zero_R") as float
	var knee_limit_tau_mult: float = _owner.get("knee_limit_tau_mult") as float
	var impact_knee_tau_mult: float = _owner.get("impact_knee_tau_mult") as float

	if enable_soft_joint_limits:
		var hip_min: float = -deg_to_rad(limit_hip_neg_deg)
		var hip_max: float = deg_to_rad(limit_hip_pos_deg)
		apply_rel_limit_soft_abs0("HIP_F", _refs.rb_thigh_front, _refs.rb_pelvis, rest_pelvis_thigh_F, hip_min, hip_max, k, dd, max_tau, limit_scale_F)
		apply_rel_limit_soft_abs0("HIP_R", _refs.rb_thigh_rear, _refs.rb_pelvis, rest_pelvis_thigh_R, hip_min, hip_max, k, dd, max_tau, limit_scale_R)

	if enable_soft_joint_limits and knee_zero_valid:
		var knee_min0_F: float
		var knee_max0_F: float
		if knee_flex_sign_F < 0:
			knee_min0_F = -deg_to_rad(limit_knee_neg_deg)
			knee_max0_F = deg_to_rad(limit_knee_pos_deg)
		else:
			knee_min0_F = -deg_to_rad(limit_knee_pos_deg)
			knee_max0_F = deg_to_rad(limit_knee_neg_deg)
		var knee_min0_R: float
		var knee_max0_R: float
		if knee_flex_sign_R < 0:
			knee_min0_R = -deg_to_rad(limit_knee_neg_deg)
			knee_max0_R = deg_to_rad(limit_knee_pos_deg)
		else:
			knee_min0_R = -deg_to_rad(limit_knee_pos_deg)
			knee_max0_R = deg_to_rad(limit_knee_neg_deg)
		var knee_tau_max: float = max_tau * knee_limit_tau_mult
		if impact_timer > 0.0 or (grounded_eff and touchdown_ramp_t < (_owner.get("knee_touchdown_full_sec") as float)):
			knee_tau_max *= impact_knee_tau_mult
		apply_rel_limit_soft_abs0("KNEE_F", _refs.rb_shin_front, _refs.rb_thigh_front, knee_zero_F, knee_min0_F, knee_max0_F, k, dd, knee_tau_max, knee_scale_F)
		apply_rel_limit_soft_abs0("KNEE_R", _refs.rb_shin_rear, _refs.rb_thigh_rear, knee_zero_R, knee_min0_R, knee_max0_R, k, dd, knee_tau_max, knee_scale_R)

	if enable_soft_joint_limits:
		var ank_min: float = -deg_to_rad(limit_ankle_neg_deg)
		var ank_max: float = deg_to_rad(limit_ankle_pos_deg)
		apply_rel_limit_soft_abs0("ANK_F", _refs.rb_foot_front, _refs.rb_shin_front, rest_shin_foot_F, ank_min, ank_max, k, dd, max_tau, limit_scale_F)
		apply_rel_limit_soft_abs0("ANK_R", _refs.rb_foot_rear, _refs.rb_shin_rear, rest_shin_foot_R, ank_min, ank_max, k, dd, max_tau, limit_scale_R)

	var rest_pelvis_torso: float = _owner.get("rest_pelvis_torso") as float
	var limit_spine_neg_deg: float = _owner.get("limit_spine_neg_deg") as float
	var limit_spine_pos_deg: float = _owner.get("limit_spine_pos_deg") as float
	var phase5_spine_limits_enable: bool = _owner.get("phase5_spine_limits_enable")
	var phase5_upright_saturated: bool = _owner.get("_phase5_upright_saturated")
	var phase5_support_saturated: bool = _owner.get("_phase5_support_saturated")
	var phase5_limit_saturated: bool = _owner.get("_phase5_limit_saturated")
	var phase5_spine_stable_t: float = _owner.get("_phase5_spine_stable_t") as float
	var phase5_spine_reenable_sec: float = _owner.get("phase5_spine_reenable_sec") as float

	_owner.set("_dbg_phase5_spine_on", 0)
	var spine_called: bool = false
	if enable_soft_joint_limits and phase5_spine_limits_enable:
		var spine_ok: bool = (impact_timer <= 0.0) and (not phase5_upright_saturated) and (not phase5_support_saturated) and (not phase5_limit_saturated) and (phase5_spine_stable_t >= phase5_spine_reenable_sec)
		if spine_ok:
			var sp_min: float = -deg_to_rad(limit_spine_neg_deg)
			var sp_max: float = deg_to_rad(limit_spine_pos_deg)
			spine_called = true
			_owner.set("_dbg_phase5_spine_on", 1)
			apply_rel_limit_soft_abs0("SPINE", _refs.rb_torso, _refs.rb_pelvis, rest_pelvis_torso, sp_min, sp_max, k, dd, max_spine_tau, limit_scale)
	if _owner.get("debug_enable") and (not spine_called) and (_owner.get("_dbg_lim") as Dictionary).has("SPINE"):
		(_owner.get("_dbg_lim") as Dictionary).erase("SPINE")

func _slot_model_try_calibrate_from_current_pose() -> void:
	var p: RigidBody2D = _refs.rb_pelvis
	var tf: RigidBody2D = _refs.rb_thigh_front
	var sf: RigidBody2D = _refs.rb_shin_front
	var ff: RigidBody2D = _refs.rb_foot_front
	var thigh_r: RigidBody2D = _refs.rb_thigh_rear
	var sr: RigidBody2D = _refs.rb_shin_rear
	var fr: RigidBody2D = _refs.rb_foot_rear
	if p == null or tf == null or sf == null or ff == null or thigh_r == null or sr == null or fr == null:
		_slot_cal_valid = false
		return

	# IMPORTANT:
	# Slot calibration must not sample an airborne/spawn-impact/half-landed pose.
	# Calibrate only once the locomotion/plant layers have reached a stable double-support state.
	var front_grounded: bool = _owner.get("_truth_front_g") as bool
	var rear_grounded: bool = _owner.get("_truth_rear_g") as bool
	var both_grounded: bool = front_grounded and rear_grounded

	var front_planted: bool = int(_owner.get("_front_state")) == GCTypes.FootPlantState.PLANTED
	var rear_planted: bool = int(_owner.get("_rear_state")) == GCTypes.FootPlantState.PLANTED
	var both_planted: bool = front_planted and rear_planted

	var impact_clear: bool = ((_owner.get("_impact_timer") as float) <= 0.0)
	var touchdown_settled: bool = ((_owner.get("_touchdown_ramp_t") as float) >= (_owner.get("plant_touchdown_grace_sec") as float))

	var p7_idle: bool = (not (_owner.get("_phase7_plan_active") as bool)) \
		and (int(_owner.get("_phase7_exec_phase")) == GCTypes.Phase7ExecPhase.IDLE)

	var min_stab: float = _owner.get("plant_min_stability") as float
	if not is_finite(min_stab) or min_stab < 0.0:
		min_stab = 0.15
	var stabF: float = clampf(_owner.get("_truth_front_stab") as float, 0.0, 1.0)
	var stabR: float = clampf(_owner.get("_truth_rear_stab") as float, 0.0, 1.0)
	var stable_enough: bool = (stabF >= min_stab) and (stabR >= min_stab)

	var pelvis_vy_max: float = _owner.get("plant_pelvis_vy_max") as float
	if not is_finite(pelvis_vy_max) or pelvis_vy_max <= 0.0:
		pelvis_vy_max = 1.0e9
	var pelvis_vy_ok: bool = absf(p.linear_velocity.y) <= pelvis_vy_max

	if not (both_grounded and both_planted and impact_clear and touchdown_settled and p7_idle and stable_enough and pelvis_vy_ok):
		# Keep template invalid until a clean calibration window exists.
		_slot_cal_valid = false
		return

	var p_rot: float = p.global_rotation

	# Baseline link COM vectors expressed in pelvis frame.
	_slot_cal_p2t_F_pf = (tf.global_position - p.global_position).rotated(-p_rot)
	_slot_cal_t2s_F_pf = (sf.global_position - tf.global_position).rotated(-p_rot)
	_slot_cal_s2f_F_pf = (ff.global_position - sf.global_position).rotated(-p_rot)

	_slot_cal_p2t_R_pf = (thigh_r.global_position - p.global_position).rotated(-p_rot)
	_slot_cal_t2s_R_pf = (sr.global_position - thigh_r.global_position).rotated(-p_rot)
	_slot_cal_s2f_R_pf = (fr.global_position - sr.global_position).rotated(-p_rot)

	# Baseline rel angles (same convention as posture PD target_rel values).
	_slot_cal_hip_rel_F = wrapf(tf.global_rotation - p.global_rotation, -PI, PI)
	_slot_cal_knee_rel_F = wrapf(sf.global_rotation - tf.global_rotation, -PI, PI)
	_slot_cal_ankle_rel_F = wrapf(ff.global_rotation - sf.global_rotation, -PI, PI)

	_slot_cal_hip_rel_R = wrapf(thigh_r.global_rotation - p.global_rotation, -PI, PI)
	_slot_cal_knee_rel_R = wrapf(sr.global_rotation - thigh_r.global_rotation, -PI, PI)
	_slot_cal_ankle_rel_R = wrapf(fr.global_rotation - sr.global_rotation, -PI, PI)

	_slot_cal_valid = true

func _slot_model_authored_stance_targets_deg(lead_is_front: bool) -> Dictionary:
	# IMPORTANT:
	# This is the authored stance pose only (continuous with stance_alpha).
	# It intentionally excludes transient runtime modifiers (swing suppression, planted minflex, impact overrides),
	# because slot planning must be stable and deterministic.
	var a: float = clampf(_refs.stance_alpha, 0.0, 1.0)

	var hipL_deg: float = lerpf(_owner.get("stance_crouch_lead_hip_deg") as float, _owner.get("stance_tall_lead_hip_deg") as float, a)
	var kneeL_deg: float = lerpf(_owner.get("stance_crouch_lead_knee_deg") as float, _owner.get("stance_tall_lead_knee_deg") as float, a)
	var ankleL_deg: float = lerpf(_owner.get("stance_crouch_lead_ankle_deg") as float, _owner.get("stance_tall_lead_ankle_deg") as float, a)

	var hipT_deg: float = lerpf(_owner.get("stance_crouch_trail_hip_deg") as float, _owner.get("stance_tall_trail_hip_deg") as float, a)
	var kneeT_deg: float = lerpf(_owner.get("stance_crouch_trail_knee_deg") as float, _owner.get("stance_tall_trail_knee_deg") as float, a)
	var ankleT_deg: float = lerpf(_owner.get("stance_crouch_trail_ankle_deg") as float, _owner.get("stance_tall_trail_ankle_deg") as float, a)

	var hipF_deg: float = hipL_deg if lead_is_front else hipT_deg
	var kneeF_deg: float = kneeL_deg if lead_is_front else kneeT_deg
	var ankleF_deg: float = ankleL_deg if lead_is_front else ankleT_deg

	var hipR_deg: float = hipT_deg if lead_is_front else hipL_deg
	var kneeR_deg: float = kneeT_deg if lead_is_front else kneeL_deg
	var ankleR_deg: float = ankleT_deg if lead_is_front else ankleL_deg

	var fs: float = float(_owner.get("facing_sign"))
	if fs == 0.0:
		fs = 1.0
	hipF_deg *= fs
	hipR_deg *= fs
	ankleF_deg *= fs
	ankleR_deg *= fs

	var ksF: float = float(_owner.get("_knee_flex_sign_F"))
	var ksR: float = float(_owner.get("_knee_flex_sign_R"))
	if ksF == 0.0:
		ksF = 1.0
	if ksR == 0.0:
		ksR = 1.0
	kneeF_deg = absf(kneeF_deg) * ksF
	kneeR_deg = absf(kneeR_deg) * ksR

	return {
		"hipF_deg": hipF_deg, "kneeF_deg": kneeF_deg, "ankleF_deg": ankleF_deg,
		"hipR_deg": hipR_deg, "kneeR_deg": kneeR_deg, "ankleR_deg": ankleR_deg
	}

func _slot_model_predict_foot_pf(is_front: bool, hip_rel_tgt: float, knee_rel_tgt: float, ankle_rel_tgt: float) -> Vector2:
	if not _slot_cal_valid:
		_slot_model_try_calibrate_from_current_pose()
	if not _slot_cal_valid:
		return Vector2(NAN, NAN)

	var p2t_pf: Vector2
	var t2s_pf: Vector2
	var s2f_pf: Vector2
	var hip_rel0: float
	var knee_rel0: float
	var ankle_rel0: float

	if is_front:
		p2t_pf = _slot_cal_p2t_F_pf
		t2s_pf = _slot_cal_t2s_F_pf
		s2f_pf = _slot_cal_s2f_F_pf
		hip_rel0 = _slot_cal_hip_rel_F
		knee_rel0 = _slot_cal_knee_rel_F
		ankle_rel0 = _slot_cal_ankle_rel_F
	else:
		p2t_pf = _slot_cal_p2t_R_pf
		t2s_pf = _slot_cal_t2s_R_pf
		s2f_pf = _slot_cal_s2f_R_pf
		hip_rel0 = _slot_cal_hip_rel_R
		knee_rel0 = _slot_cal_knee_rel_R
		ankle_rel0 = _slot_cal_ankle_rel_R

	var d_hip: float = wrapf(hip_rel_tgt - hip_rel0, -PI, PI)
	var d_knee: float = wrapf(knee_rel_tgt - knee_rel0, -PI, PI)
	var d_ankle: float = wrapf(ankle_rel_tgt - ankle_rel0, -PI, PI)

	# Baseline vectors are in pelvis frame. Rotate each by cumulative target delta.
	var v1: Vector2 = p2t_pf.rotated(d_hip)
	var v2: Vector2 = t2s_pf.rotated(d_hip + d_knee)
	var v3: Vector2 = s2f_pf.rotated(d_hip + d_knee + d_ankle)
	return v1 + v2 + v3

func get_slot_template_axis_offsets(lead_is_front: bool) -> Vector2:
	# Returns pelvis-centered axis-space template offsets for (front, rear).
	# Positive axis = lead side along facing/neutral axis.
	# Magnitudes come from authored stance pose + calibrated rig geometry.
	var pose: Dictionary = _slot_model_authored_stance_targets_deg(lead_is_front)

	var hipF_rel: float = deg_to_rad(pose["hipF_deg"] as float)
	var kneeF_rel: float = deg_to_rad(pose["kneeF_deg"] as float)
	var ankleF_rel: float = deg_to_rad(pose["ankleF_deg"] as float)

	var hipR_rel: float = deg_to_rad(pose["hipR_deg"] as float)
	var kneeR_rel: float = deg_to_rad(pose["kneeR_deg"] as float)
	var ankleR_rel: float = deg_to_rad(pose["ankleR_deg"] as float)

	var front_pf: Vector2 = _slot_model_predict_foot_pf(true, hipF_rel, kneeF_rel, ankleF_rel)
	var rear_pf: Vector2 = _slot_model_predict_foot_pf(false, hipR_rel, kneeR_rel, ankleR_rel)
	if not is_finite(front_pf.x) or not is_finite(rear_pf.x):
		return Vector2(NAN, NAN)

	var axis_sign: float = signf(float(_owner.get("facing_sign")))
	if axis_sign == 0.0:
		axis_sign = 1.0

	# Convert pelvis-frame X to axis-space and recenter around 0.
	var front_s: float = front_pf.x * axis_sign
	var rear_s: float = rear_pf.x * axis_sign
	var mid: float = 0.5 * (front_s + rear_s)
	front_s -= mid
	rear_s -= mid

	_owner.set("_dbg_slot_tpl_front_ax", front_s)
	_owner.set("_dbg_slot_tpl_rear_ax", rear_s)
	_owner.set("_dbg_slot_tpl_sep_ax", absf(front_s - rear_s))
	_owner.set("_dbg_slot_cal_valid", 1 if _slot_cal_valid else 0)

	return Vector2(front_s, rear_s)

func apply_posture_phase1b(spawn01: float) -> void:
	if not _owner.get("enable_posture_pd"):
		return
	var a: float = _refs.stance_alpha
	var lead_is_front: bool = _owner.get("_lead_is_front")
	var hipL_deg: float = lerpf(_owner.get("stance_crouch_lead_hip_deg") as float, _owner.get("stance_tall_lead_hip_deg") as float, a)
	var kneeL_deg: float = lerpf(_owner.get("stance_crouch_lead_knee_deg") as float, _owner.get("stance_tall_lead_knee_deg") as float, a)
	var ankleL_deg: float = lerpf(_owner.get("stance_crouch_lead_ankle_deg") as float, _owner.get("stance_tall_lead_ankle_deg") as float, a)
	var hipT_deg: float = lerpf(_owner.get("stance_crouch_trail_hip_deg") as float, _owner.get("stance_tall_trail_hip_deg") as float, a)
	var kneeT_deg: float = lerpf(_owner.get("stance_crouch_trail_knee_deg") as float, _owner.get("stance_tall_trail_knee_deg") as float, a)
	var ankleT_deg: float = lerpf(_owner.get("stance_crouch_trail_ankle_deg") as float, _owner.get("stance_tall_trail_ankle_deg") as float, a)
	var hipF_deg: float = hipL_deg if lead_is_front else hipT_deg
	var kneeF_deg: float = kneeL_deg if lead_is_front else kneeT_deg
	var ankleF_deg: float = ankleL_deg if lead_is_front else ankleT_deg
	var hipR_deg: float = hipT_deg if lead_is_front else hipL_deg
	var kneeR_deg: float = kneeT_deg if lead_is_front else kneeL_deg
	var ankleR_deg: float = ankleT_deg if lead_is_front else ankleL_deg
	var support_blend: float = _owner.get("_support_blend") as float
	var support_gate: float = clampf(support_blend, 0.0, 1.0)
	var hip_gate: float = maxf(support_gate, _owner.get("hip_posture_min_gate_pre_support") as float)
	var knee_gate: float = maxf(support_gate, _owner.get("knee_posture_min_gate_pre_support") as float)
	var cmd_stance_y_eff: float = _owner.get("_cmd_stance_y_eff") as float
	var cmd_stance_deadzone: float = _owner.get("cmd_stance_deadzone") as float
	var transitioning: bool = absf(cmd_stance_y_eff) > cmd_stance_deadzone
	var k_eff: float = _owner.get("k_limb") as float
	var d_eff: float = _owner.get("d_limb") as float
	var max_tau_eff: float = _owner.get("max_limb_tau") as float
	if transitioning:
		k_eff *= _owner.get("stance_transition_gain_mult") as float
		d_eff *= _owner.get("stance_transition_gain_mult") as float
		max_tau_eff *= _owner.get("stance_transition_tau_mult") as float
	if _owner.get("_impact_timer") as float > 0.0:
		d_eff *= _owner.get("landing_posture_d_mult") as float
	var hip_scale: float = clampf(_owner.get("hip_posture_scale") as float * spawn01 * hip_gate, 0.0, 1.0)
	var knee_scale: float = clampf(_owner.get("knee_posture_scale") as float * spawn01 * knee_gate, 0.0, 1.0)
	var facing_sign: int = _owner.get("facing_sign")
	var fs: float = float(facing_sign)
	hipF_deg *= fs
	hipR_deg *= fs
	ankleF_deg *= fs
	ankleR_deg *= fs
	var ksF: float = float(_owner.get("_knee_flex_sign_F"))
	var ksR: float = float(_owner.get("_knee_flex_sign_R"))
	var kF_abs: float = absf(kneeF_deg)
	var kR_abs: float = absf(kneeR_deg)
	var knee_minflex_active: bool = false
	var minflex_deg: float = _owner.get("knee_planted_min_flex_deg") as float
	var truth_front_g: bool = _owner.get("_truth_front_g")
	var truth_rear_g: bool = _owner.get("_truth_rear_g")
	var knee_zero_valid: bool = _owner.get("_knee_zero_valid")
	if minflex_deg > 0.0:
		if cmd_stance_y_eff < -cmd_stance_deadzone:
			if truth_front_g:
				kF_abs = maxf(kF_abs, minflex_deg)
				knee_minflex_active = true
			if truth_rear_g:
				kR_abs = maxf(kR_abs, minflex_deg)
				knee_minflex_active = true
		if knee_zero_valid and _refs.rb_shin_front != null and _refs.rb_thigh_front != null and _refs.rb_shin_rear != null and _refs.rb_thigh_rear != null:
			var slack_rad: float = deg_to_rad(1.0)
			var knee_zero_F: float = _owner.get("_knee_zero_F") as float
			var knee_zero_R: float = _owner.get("_knee_zero_R") as float
			var kF0_now: float = wrapf((_refs.rb_shin_front.global_rotation - _refs.rb_thigh_front.global_rotation) - knee_zero_F, -PI, PI)
			var kR0_now: float = wrapf((_refs.rb_shin_rear.global_rotation - _refs.rb_thigh_rear.global_rotation) - knee_zero_R, -PI, PI)
			if truth_front_g and (kF0_now * ksF < -slack_rad):
				kF_abs = maxf(kF_abs, minflex_deg)
				knee_minflex_active = true
			if truth_rear_g and (kR0_now * ksR < -slack_rad):
				kR_abs = maxf(kR_abs, minflex_deg)
				knee_minflex_active = true
	kneeF_deg = kF_abs * ksF
	kneeR_deg = kR_abs * ksR
	var knee_scale_eff: float = knee_scale
	if knee_minflex_active:
		knee_scale_eff *= _owner.get("knee_minflex_posture_scale_mult") as float
	var swing_scale_F: float = 1.0
	var swing_scale_R: float = 1.0
	if _owner.get("phase7_enable") and _owner.get("_phase7_plan_active"):
		var phase7_exec_phase: int = _owner.get("_phase7_exec_phase")
		if phase7_exec_phase == GCTypes.Phase7ExecPhase.UNPLANT or phase7_exec_phase == GCTypes.Phase7ExecPhase.SWING:
			var s: float = clampf(_owner.get("phase7_swing_posture_scale") as float, 0.0, 1.0)
			if _owner.get("_phase7_exec_is_front"):
				swing_scale_F = s
			else:
				swing_scale_R = s
	var hip_scale_F: float = hip_scale * swing_scale_F
	var knee_scale_F: float = knee_scale_eff * swing_scale_F
	var ankle_scale_F: float = knee_scale * swing_scale_F
	var hip_scale_R: float = hip_scale * swing_scale_R
	var knee_scale_R: float = knee_scale_eff * swing_scale_R
	var ankle_scale_R: float = knee_scale * swing_scale_R
	if _owner.get("phase7_enable") and _owner.get("_phase7_plan_active"):
		var p7_exec: int = _owner.get("_phase7_exec_phase")
		if p7_exec == GCTypes.Phase7ExecPhase.UNPLANT or p7_exec == GCTypes.Phase7ExecPhase.SWING:
			if _owner.get("_phase7_exec_is_front"):
				ankle_scale_F = 0.0
			else:
				ankle_scale_R = 0.0

	# 5R2-J: In STEP_PLANNER mode, ankle-down foot orientation is owned by the unified
	# parallel-foot attitude controller in gc_foot_plant (pipeline-routed by FootControlMode).
	# Posture PD must NOT inject ankle posture torque in this mode.
	if (_owner.get("movement_authority_mode") as int) == GCTypes.MovementAuthorityMode.STEP_PLANNER:
		ankle_scale_F = 0.0
		ankle_scale_R = 0.0

	var hipF_target: float = deg_to_rad(hipF_deg)
	var kneeF_target: float = deg_to_rad(kneeF_deg)
	var ankleF_target: float = deg_to_rad(ankleF_deg)
	var hipR_target: float = deg_to_rad(hipR_deg)
	var kneeR_target: float = deg_to_rad(kneeR_deg)
	var ankleR_target: float = deg_to_rad(ankleR_deg)
	var limit_hip_neg_deg: float = _owner.get("limit_hip_neg_deg") as float
	var limit_hip_pos_deg: float = _owner.get("limit_hip_pos_deg") as float
	var limit_ankle_neg_deg: float = _owner.get("limit_ankle_neg_deg") as float
	var limit_ankle_pos_deg: float = _owner.get("limit_ankle_pos_deg") as float
	var limit_knee_neg_deg: float = _owner.get("limit_knee_neg_deg") as float
	var limit_knee_pos_deg: float = _owner.get("limit_knee_pos_deg") as float
	hipF_target = clampf(hipF_target, -deg_to_rad(limit_hip_neg_deg), deg_to_rad(limit_hip_pos_deg))
	hipR_target = clampf(hipR_target, -deg_to_rad(limit_hip_neg_deg), deg_to_rad(limit_hip_pos_deg))
	ankleF_target = clampf(ankleF_target, -deg_to_rad(limit_ankle_neg_deg), deg_to_rad(limit_ankle_pos_deg))
	ankleR_target = clampf(ankleR_target, -deg_to_rad(limit_ankle_neg_deg), deg_to_rad(limit_ankle_pos_deg))
	if knee_zero_valid:
		var knee_zero_F: float = _owner.get("_knee_zero_F") as float
		var knee_zero_R: float = _owner.get("_knee_zero_R") as float
		var kminF0: float
		var kmaxF0: float
		if _owner.get("_knee_flex_sign_F") < 0:
			kminF0 = -deg_to_rad(limit_knee_neg_deg)
			kmaxF0 = deg_to_rad(limit_knee_pos_deg)
		else:
			kminF0 = -deg_to_rad(limit_knee_pos_deg)
			kmaxF0 = deg_to_rad(limit_knee_neg_deg)
		var kminR0: float
		var kmaxR0: float
		if _owner.get("_knee_flex_sign_R") < 0:
			kminR0 = -deg_to_rad(limit_knee_neg_deg)
			kmaxR0 = deg_to_rad(limit_knee_pos_deg)
		else:
			kminR0 = -deg_to_rad(limit_knee_pos_deg)
			kmaxR0 = deg_to_rad(limit_knee_neg_deg)
		var kF0: float = wrapf(kneeF_target - knee_zero_F, -PI, PI)
		kF0 = clampf(kF0, kminF0, kmaxF0)
		kneeF_target = wrapf(knee_zero_F + kF0, -PI, PI)
		var kR0: float = wrapf(kneeR_target - knee_zero_R, -PI, PI)
		kR0 = clampf(kR0, kminR0, kmaxR0)
		kneeR_target = wrapf(knee_zero_R + kR0, -PI, PI)
	_owner.set("_dbg_hipF_tgt_deg", hipF_deg)
	_owner.set("_dbg_kneeF_tgt_deg", kneeF_deg)
	_owner.set("_dbg_ankleF_tgt_deg", ankleF_deg)
	_owner.set("_dbg_hipR_tgt_deg", hipR_deg)
	_owner.set("_dbg_kneeR_tgt_deg", kneeR_deg)
	_owner.set("_dbg_ankleR_tgt_deg", ankleR_deg)
	apply_joint_pd_child_only(_refs.rb_pelvis, _refs.rb_thigh_front, hipF_target, k_eff, d_eff, max_tau_eff, hip_scale_F, 0.0)
	apply_joint_pd_child_only(_refs.rb_thigh_front, _refs.rb_shin_front, kneeF_target, k_eff, d_eff, max_tau_eff, knee_scale_F, 0.10)
	apply_joint_pd_child_only(_refs.rb_shin_front, _refs.rb_foot_front, ankleF_target, k_eff, d_eff, max_tau_eff, ankle_scale_F, 0.10)
	apply_joint_pd_child_only(_refs.rb_pelvis, _refs.rb_thigh_rear, hipR_target, k_eff, d_eff, max_tau_eff, hip_scale_R, 0.0)
	apply_joint_pd_child_only(_refs.rb_thigh_rear, _refs.rb_shin_rear, kneeR_target, k_eff, d_eff, max_tau_eff, knee_scale_R, 0.10)
	apply_joint_pd_child_only(_refs.rb_shin_rear, _refs.rb_foot_rear, ankleR_target, k_eff, d_eff, max_tau_eff, ankle_scale_R, 0.10)

func dbg_knee_rel0_deg(is_front: bool) -> float:
	var thigh: RigidBody2D = _refs.rb_thigh_front if is_front else _refs.rb_thigh_rear
	var shin: RigidBody2D = _refs.rb_shin_front if is_front else _refs.rb_shin_rear
	if thigh == null or shin == null or not _owner.get("_knee_zero_valid"):
		return NAN
	var zero_rel: float = (_owner.get("_knee_zero_F") as float) if is_front else (_owner.get("_knee_zero_R") as float)
	var rel: float = wrapf(shin.global_rotation - thigh.global_rotation, -PI, PI)
	var rel0: float = wrapf(rel - zero_rel, -PI, PI)
	if rel0 < 0.0 and absf(rel0) > (PI * 0.75):
		rel0 += TAU
	return rad_to_deg(rel0)
