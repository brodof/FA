# res://scripts/duel/gladiator_controller/gc_pipeline.gd
# Godot 4.5.1
# High-level frame pipeline for the gladiator physics controller.
# Owns the main per-frame algorithm surface (Phase 2/7), while the controller
# stays as a thin host for exports, node refs, and module wiring.

extends RefCounted
class_name GCPipeline

var _host: FA_GladiatorPhysicsController = null
var _refs: GCRefs = null

func setup(host: FA_GladiatorPhysicsController, refs: GCRefs) -> void:
	_host = host
	_refs = refs


func tick(dt: float, spawn01: float, dragging: bool, drag_ended: bool) -> void:
	_tick_frame_body_rest(dt, spawn01, dragging, drag_ended)


func _tick_frame_body_rest(dt: float, spawn01: float, dragging: bool, drag_ended: bool) -> void:
	var o := _host
	if o == null:
		return

	# ---------------------------------------------------------------------------
	# Phase 2 - Fused Ground Truth (SENSOR(valid) -> PROBE(tight) -> HOLD_LAST(tight))
	# ---------------------------------------------------------------------------
	if o._ground_truth_mod != null:
		o._ground_truth_mod.update_fused_truth(dt)

	var front_g: bool = o._truth_front_g
	var rear_g: bool = o._truth_rear_g
	var grounded: bool = front_g or rear_g

	var stabF: float = o._truth_front_stab
	var stabR: float = o._truth_rear_stab

	# Pelvis probe remains support_y fallback only (not grounding truth).
	var allow_probe := o.support_allow_probe_during_spawn and (o.phase1b_stand_only or (o._t < 2.0))
	var probe_y := NAN
	if allow_probe and o._rb_pelvis != null and o._debug_mod != null:
		probe_y = o._debug_mod.dbg_world_probe_y(o._rb_pelvis, o._ray_exclude)

	# Short grace still helps, but now driven by fused truth (not NaN gates).
	if grounded:
		o._ground_grace_t = o.grounded_grace_sec
	else:
		o._ground_grace_t = maxf(0.0, o._ground_grace_t - dt)

	# Eligibility latch prevents one bad frame from dropping pins/support
	var grounded_eff: bool = grounded or (o._truth_front_allow_t > 0.0) or (o._truth_rear_allow_t > 0.0)

	# Sync shared refs for modules (per-frame state)
	if _refs != null:
		_refs.dt = dt
		_refs.front_g = front_g
		_refs.rear_g = rear_g
		_refs.grounded_eff = grounded_eff
		_refs.stabF = stabF
		_refs.stabR = stabR
		_refs.cmd_move_x_norm = o.cmd_move_x
		_refs.cmd_stance_y_eff = o._cmd_stance_y_eff
		_refs.stance_alpha = o._stance_alpha
		_refs.stance_height01 = o.stance_height01
		_refs.planner_mode = o._planner_mode
	if o._foot_plant_mod != null:
		o._foot_plant_mod.sync_plant_support_y_to_refs()

	# Phase 4: true airborne reset (not "grounded_eff", which is sticky via allow-timers).
	# "Clearly airborne" means: not grounded AND no allow grace remaining.
	var clearly_airborne: bool = (not grounded) and (o._truth_front_allow_t <= 0.0) and (o._truth_rear_allow_t <= 0.0)
	if clearly_airborne:
		o._vsupport_Fy_prev = 0.0

	if not is_finite(o._support_y_last_valid) and is_finite(probe_y):
		o._support_y_last_valid = probe_y

	# Balance truth: support foot follows COM projection (with hysteresis).
	o.support_is_front = o._stance_planner_mod.choose_support_foot(front_g, rear_g, stabF, stabR, dt) if o._stance_planner_mod != null else o.support_is_front

	# Touchdown detect (first grounded frame)
	if grounded and not o._grounded_prev:
		o._touchdown_ramp_t = 0.0

		# --- Arm impact window on touchdown (enables extra damping + stronger limits) ---
		o._dbg_last_touch_vy = maxf(0.0, o._rb_pelvis.linear_velocity.y) if o._rb_pelvis != null else 0.0
		var impact01 := clampf(o._dbg_last_touch_vy / maxf(1.0, o.landing_impact_vy_ref), 0.0, 1.0)
		o._impact_timer = maxf(o._impact_timer, o.impact_ignore_d_sec + impact01 * 0.20)

		# Phase 4: landing ramp must reliably restart at touchdown (sink uses it).
		o._landing_t = 0.0

		# Phase 4: clear vertical support slew so no stale command carries across touchdown.
		o._vsupport_Fy_prev = 0.0

		# Debug landing log window
		o._dbg_landing_log_t = o.landing_log_window_sec
		o._dbg_landing_log_accum = 0.0

		if o.phase1b_stand_only:
			o._landing_t = 0.0

		# warm start filter on first contact (prevents chasing NAN->number spike)
		var vs = o._vertical_support_mod
		var warm_y: float = (vs.support_y_from_support_foot(front_g, rear_g) if o.phase1b_stand_only else vs.compute_support_y_raw(front_g, rear_g)) if vs != null else NAN
		if not is_finite(warm_y) and is_finite(probe_y):
			warm_y = probe_y
		if not is_finite(warm_y) and is_finite(o._support_y_last_valid):
			warm_y = o._support_y_last_valid
		o._support_y_raw = warm_y
		if is_finite(o._support_y_raw):
			o._sy_filt = o._support_y_raw

		# Touchdown must NOT bake a collapsed pose into stand height.
		o._leg_len_touch = NAN

		# Initialize stand target immediately from commanded stance height.
		if o.phase1b_stand_only and is_finite(o._support_y_raw):
			var a := clampf(o.stance_height01, 0.0, 1.0)
			var leg_len_cmd_1b := lerpf(o.leg_len_crouch, o.leg_len_stand, a)
			o._stand_y_target = o._support_y_raw - leg_len_cmd_1b
	o._grounded_prev = grounded

	if o._impact_timer > 0.0:
		o._impact_timer = maxf(0.0, o._impact_timer - dt)
	o._touchdown_ramp_t += dt

	if o._dbg_landing_log_t > 0.0:
		o._dbg_landing_log_t = maxf(0.0, o._dbg_landing_log_t - dt)

	# Support blend ramp
	if grounded_eff:
		o._support_blend = minf(1.0, o._support_blend + dt * o.support_blend_in_hz)
	else:
		o._support_blend = maxf(0.0, o._support_blend - dt * o.support_blend_out_hz)

	# Stand-only block (Phase 1B): solve standing first, no stepping/planting/x-drive.
	if o.phase1b_stand_only:
		if grounded_eff:
			o._landing_t += dt
		else:
			o._landing_t = 0.0
	var stand_la: float = (clampf(o._landing_t / maxf(0.001, o.landing_ramp_sec), 0.0, 1.0) if o.phase1b_stand_only else 0.0)
	if _tick_stand_only_path(
		o.phase1b_stand_only,
		dt,
		front_g,
		rear_g,
		stabF,
		stabR,
		grounded,
		grounded_eff,
		stand_la,
		o.landing_support_min,
		dragging,
		probe_y,
		spawn01
	):
		return

	# ============================
	# Phase 2 runtime (walk)
	# ============================
	if not o.phase2_enable:
		return

	# Phase 2 runtime: delegate to helper.
	if grounded_eff:
		o._landing_t += dt
	else:
		o._landing_t = 0.0
	var landing_alpha_p2: float = clampf(o._landing_t / maxf(0.001, o.landing_ramp_sec), 0.0, 1.0)
	_tick_runtime_walk(
		dt,
		front_g,
		rear_g,
		stabF,
		stabR,
		grounded,
		grounded_eff,
		landing_alpha_p2,
		dragging,
		drag_ended,
		probe_y,
		spawn01
	)


func _tick_stand_only_path(
	phase1b_stand_only: bool,
	dt: float,
	front_g: bool,
	rear_g: bool,
	stabF: float,
	stabR: float,
	grounded: bool,
	grounded_eff: bool,
	landing_alpha: float,
	landing_support_min: float,
	dragging: bool,
	probe_y: float,
	spawn01: float
) -> bool:
	var o := _host
	if o == null:
		return false

	var run01: float = 0.0
	if phase1b_stand_only:
		# Landing window: ramp authority from 0 -> 1 after touchdown.
		if grounded_eff:
			o._landing_t += dt
		else:
			o._landing_t = 0.0
		# Phase1B must not deadlock waiting for "stable feet".
		# Give a small in-air assist so feet can become first contact.
		var air_base: float = 0.22 # in-air assist so feet can become first contact
		var phase1b_muscle_gate: float = air_base

		if grounded_eff:
			# Landing must be strong immediately; ramping here causes collapse/windmill.
			phase1b_muscle_gate = maxf(o.landing_muscle_min, landing_alpha)

		# 3) support y from chosen support foot (no plant anchors in Phase 1B)
		var vs1 = o._vertical_support_mod
		o._support_y_raw = vs1.support_y_from_support_foot(front_g, rear_g) if vs1 != null else NAN
		if not is_finite(o._support_y_raw) and is_finite(probe_y):
			o._support_y_raw = probe_y
		if not is_finite(o._support_y_raw) and is_finite(o._support_y_last_valid):
			o._support_y_raw = o._support_y_last_valid
		o._support_y_filt = vs1.filter_support_y(o._support_y_raw, dt) if vs1 != null else o._support_y_raw
		if is_finite(o._support_y_filt):
			o._support_y_last_valid = o._support_y_filt
			o._support_hold_t = o.support_hold_last_y_sec
		elif o._support_hold_t > 0.0:
			o._support_hold_t = maxf(0.0, o._support_hold_t - dt)
			o._support_y_filt = o._support_y_last_valid
		if _refs != null:
			_refs.support_y_raw = o._support_y_raw
			_refs.support_y_filt = o._support_y_filt

		# Stance alpha controls BOTH joint angles and pelvis height in 1B.
		o._stance_alpha = clampf(o.stance_height01, 0.0, 1.0)

		# Dynamic lead/trail identity (used by stance templates)
		var cmdx_1b := o.cmd_move_x
		if absf(cmdx_1b) < o.cmd_move_deadzone:
			cmdx_1b = 0.0
		cmdx_1b = clampf(cmdx_1b, -1.0, 1.0)
		if o._locomotion_mod != null:
			o._locomotion_mod.update_lead_trail(cmdx_1b, dt)

		var leg_len_cmd_1b: float = lerpf(o.leg_len_crouch, o.leg_len_stand, o._stance_alpha)

		# Update stance center (world X) slowly from COM to avoid pelvis<->feet feedback jitter.
		var com_x := GCMath.compute_com_world(o._refs.rb_list, o._refs.rb_mass_sum).x
		if not is_finite(o._stance_center_x):
			o._stance_center_x = com_x if is_finite(com_x) else (o._rb_pelvis.global_position.x if o._rb_pelvis != null else 0.0)
		else:
			var desired_cx := com_x if is_finite(com_x) else o._stance_center_x

			# Clamp inside support span only when both are planted.
			if o._front_state == GCTypes.FootPlantState.PLANTED and o._rear_state == GCTypes.FootPlantState.PLANTED:
				var fx := o._foot_plant_mod.foot_best_x(true) if o._foot_plant_mod != null else NAN
				var rx := o._foot_plant_mod.foot_best_x(false) if o._foot_plant_mod != null else NAN
				if is_finite(fx) and is_finite(rx):
					var min_x := minf(fx, rx)
					var max_x := maxf(fx, rx)
					var span := max_x - min_x
					if span < 2.0 * o.stance_center_margin_px + 1.0:
						desired_cx = (min_x + max_x) * 0.5
					else:
						var lo := min_x + o.stance_center_margin_px
						var hi := max_x - o.stance_center_margin_px
						desired_cx = clampf(desired_cx, lo, hi)

			var vs1b = o._vertical_support_mod
			o._stance_center_x = vs1b.exp_smooth(o._stance_center_x, desired_cx, o.stance_center_follow_hz, dt) if vs1b != null else desired_cx

		# Rest capture must run in 1B too (otherwise REST never clears and limits never engage).
		if not (dragging and o.debug_mouse_drag_suspend_rest_capture) and o._recovery_mod != null:
			o._recovery_mod.update_rest_capture_gate(grounded_eff, front_g, rear_g, stabF, stabR, dt)

		# Smooth the stand height target (do not snap on touchdown).
		if grounded_eff and is_finite(o._support_y_filt):
			var desired_y: float = o._support_y_filt - leg_len_cmd_1b
			if not is_finite(o._stand_y_target):
				o._stand_y_target = desired_y if is_finite(desired_y) else (o._rb_pelvis.global_position.y if o._rb_pelvis != null else NAN)
			var a_y: float = clampf(dt * o.stand_target_hz, 0.0, 1.0)
			o._stand_y_target = lerpf(o._stand_y_target, desired_y, a_y)

		# 4) pelvis height: only after real grounding.
		# IMPORTANT: never scale this to ~0 at touchdown, or you get a guaranteed collapse.
		if grounded_eff and is_finite(o._support_y_filt) and not (dragging and o.debug_mouse_drag_suspend_vertical_support):
			var support_gate: float = maxf(landing_support_min, landing_alpha)
			if o._vertical_support_mod != null:
				o._vertical_support_mod.apply_vertical_support(o._support_y_filt, leg_len_cmd_1b, dt, spawn01, support_gate)

			# Phase 5: vertical support saturation flag (commanded, post-clamp + post-gate + post-slew)
			var gate_eff: float = maxf(o._dbg_vsupport_gate, 0.0001)
			var satY_now: float = absf(o._dbg_Fy_cmd) / maxf(1.0, o._dbg_Fy_max * gate_eff)
			o._phase5_support_saturated = o._dbg_vsupport_called and (satY_now >= o.phase5_sat_ratio)

		# 5) torso stays up (always, world upright)
		if o._posture_mod != null:
			o._posture_mod.apply_torso_world_upright(dt, spawn01 * phase1b_muscle_gate)

		# 6) pelvis upright (secondary; grounded-scaled ok)
		if o._posture_mod != null:
			o._posture_mod.apply_angle_pd(o._rb_pelvis, 0.0, o.pelvis_world_upright_k, o.pelvis_world_upright_d, o.pelvis_world_upright_tau_max, spawn01 * phase1b_muscle_gate)

		# Phase 5: upright saturation flag (pelvis + torso), using effective torque scale
		var phase1b_tau_scale_upr: float = spawn01 * phase1b_muscle_gate

		if phase1b_tau_scale_upr > 0.0001 and o._rb_pelvis != null:
			var errP: float = wrapf(0.0 - o._rb_pelvis.global_rotation, -PI, PI)
			var tauP_raw: float = (o.pelvis_world_upright_k * errP) - (o.pelvis_world_upright_d * o._rb_pelvis.angular_velocity)
			var tauP_cmd: float = clampf(tauP_raw, -o.pelvis_world_upright_tau_max, o.pelvis_world_upright_tau_max) * phase1b_tau_scale_upr
			var tauP_max_cmd: float = o.pelvis_world_upright_tau_max * phase1b_tau_scale_upr
			if absf(tauP_cmd) >= (tauP_max_cmd * o.phase5_sat_ratio):
				o._phase5_upright_saturated = true

		if phase1b_tau_scale_upr > 0.0001 and o._rb_torso != null:
			var errT: float = wrapf(0.0 - o._rb_torso.global_rotation, -PI, PI)
			var rT: float = o.torso_inertia_radius_px
			var IT: float = o._rb_torso.mass * rT * rT
			var wT: float = TAU * o.torso_upright_freq_hz
			var kT: float = IT * wT * wT
			var dT: float = 2.0 * IT * wT * o.torso_upright_zeta
			var tauT_raw: float = (kT * errT) - (dT * o._rb_torso.angular_velocity)
			var tauT_cmd: float = clampf(tauT_raw, -o.torso_upright_tau_max, o.torso_upright_tau_max) * phase1b_tau_scale_upr
			var tauT_max_cmd: float = o.torso_upright_tau_max * phase1b_tau_scale_upr
			if absf(tauT_cmd) >= (tauT_max_cmd * o.phase5_sat_ratio):
				o._phase5_upright_saturated = true

		# Only flatten in-air (once grounded, the floor already enforces it and your torque just fights contacts)
		if o._rb_foot_front != null and not front_g:
			if o._posture_mod != null:
				o._posture_mod.apply_angle_pd(o._rb_foot_front, 0.0, o.foot_world_k, o.foot_world_d, o.foot_world_tau_max, spawn01 * phase1b_muscle_gate)
		if o._rb_foot_rear != null and not rear_g:
			if o._posture_mod != null:
				o._posture_mod.apply_angle_pd(o._rb_foot_rear, 0.0, o.foot_world_k, o.foot_world_d, o.foot_world_tau_max, spawn01 * phase1b_muscle_gate)

		# 7) optional mild posture (child-only; keep weak)
		if o._posture_mod != null:
			o._posture_mod.apply_posture_phase1b(spawn01 * phase1b_muscle_gate)

		# 8) soft limits (always-on)
		if o.enable_soft_joint_limits and o._rest_seeded:
			if o._posture_mod != null:
				o._posture_mod.apply_soft_limits_phase1b(spawn01)

		# Phase 5: update stable timer + latch debug bits AFTER limits (so limit saturation is included)
		if not o.phase5_spine_limits_enable:
			o._phase5_spine_stable_t = 0.0
		else:
			var phase5_stable: bool = grounded_eff and (o._impact_timer <= 0.0) \
				and (not o._phase5_upright_saturated) \
				and (not o._phase5_support_saturated) \
				and (not o._phase5_limit_saturated)
			if phase5_stable:
				o._phase5_spine_stable_t = minf(o._phase5_spine_stable_t + dt, o.phase5_spine_reenable_sec * 2.0)
			else:
				o._phase5_spine_stable_t = 0.0

		o._dbg_phase5_upr_sat = 1 if o._phase5_upright_saturated else 0
		o._dbg_phase5_sup_sat = 1 if o._phase5_support_saturated else 0
		o._dbg_phase5_lim_sat = 1 if o._phase5_limit_saturated else 0

		# Stand-only must still run planting state machine (covers the early return path).
		var phase1b_want_front: bool = front_g
		var phase1b_want_rear: bool = rear_g

		# Planner must tick whenever BRACE/Phase6/stance-change requires it, even if phase3_enable is off.
		var phase1b_planner_enable: bool = o.phase3_enable or o.phase6_enable or o._spawn_brace_active or (o._planner_mode != GCTypes.PlannerMode.IDLE) or o._phase6_stance_changing
		if phase1b_planner_enable:
			if o._stance_planner_mod != null:
				o._stance_planner_mod.update_standing_plan(front_g, rear_g, stabF, stabR, dt, o._phase6_stance_changing)

			var front_allow: bool = front_g or (o._truth_front_allow_t > 0.0)
			var rear_allow: bool = rear_g or (o._truth_rear_allow_t > 0.0)

			phase1b_want_front = (o._plan_front == GCTypes.PlanFoot.PLANTED) and front_allow
			phase1b_want_rear = (o._plan_rear == GCTypes.PlanFoot.PLANTED) and rear_allow

			# State over plan: when both feet planted or converged, allow recenter for both.
			var min_sep_1b: float = o.phase7_min_foot_separation_px
			var converged_1b_early: bool = front_g and rear_g and absf(o._plant_front_x - o._plant_rear_x) < min_sep_1b
			var both_planted_1b: bool = (o._front_state == GCTypes.FootPlantState.PLANTED) and (o._rear_state == GCTypes.FootPlantState.PLANTED)
			var recenter_ok_1b: bool = (o._touchdown_ramp_t >= o.plant_touchdown_grace_sec) and (o._impact_timer <= 0.0)
			if (both_planted_1b or converged_1b_early) and recenter_ok_1b:
				o._allow_recenter_front = true
				o._allow_recenter_rear = true
				if both_planted_1b:
					phase1b_want_front = true
					phase1b_want_rear = true
			elif o._planner_mode == GCTypes.PlannerMode.BRACE or o._planner_mode == GCTypes.PlannerMode.RECOVER:
				o._allow_recenter_front = recenter_ok_1b and (o._plan_front == GCTypes.PlanFoot.PLANTED)
				o._allow_recenter_rear = recenter_ok_1b and (o._plan_rear == GCTypes.PlanFoot.PLANTED)
			else:
				var standing_idle_1b: bool = (not o._phase7_plan_active) and both_planted_1b
				if standing_idle_1b and recenter_ok_1b:
					o._allow_recenter_front = true
					o._allow_recenter_rear = true
					phase1b_want_front = true
					phase1b_want_rear = true
				else:
					o._allow_recenter_front = false
					o._allow_recenter_rear = false

			var swing_boost: float = 1.0
			if o._planner_mode == GCTypes.PlannerMode.BRACE or o._planner_mode == GCTypes.PlannerMode.RECOVER:
				swing_boost = o.phase6_swing_force_mult_recover
			elif o._phase6_stance_changing:
				swing_boost = o.phase6_swing_force_mult_stance_change

			# "Slide" = planned SWING but still in contact. Only enable slide tricks during BRACE/RECOVER or stance changes.
			var slide_ok: bool = (o._planner_mode != GCTypes.PlannerMode.IDLE) or o._phase6_stance_changing
			# Don't enable slide tricks during the touchdown + impact catch window.
			if (o._touchdown_ramp_t < o.plant_touchdown_grace_sec) or (o._impact_timer > 0.0):
				slide_ok = false
			var front_slide: bool = slide_ok and front_g and (o._plan_front == GCTypes.PlanFoot.SWING)
			var rear_slide: bool = slide_ok and rear_g and (o._plan_rear == GCTypes.PlanFoot.SWING)
			o._dbg_front_slide = front_slide
			o._dbg_rear_slide = rear_slide

			# Per-foot friction switching: reduce friction on the slide foot so it can actually move.
			o._phase6_apply_foot_friction(front_slide, rear_slide)

			# Apply swing forces only when we're intentionally sliding a grounded foot.
			# (Prevents mid-air "pre-landing" wobble and the touchdown lateral kick.)
			if front_slide and o._rb_foot_front != null and is_finite(o._plan_target_front.x):
				var multF: float = swing_boost * o.phase6_slide_force_mult
				if o._ground_truth_mod != null:
					o._ground_truth_mod.apply_swing_force(true, o._plan_target_front, dt, spawn01 * multF)

			if rear_slide and o._rb_foot_rear != null and is_finite(o._plan_target_rear.x):
				var multR: float = swing_boost * o.phase6_slide_force_mult
				if o._ground_truth_mod != null:
					o._ground_truth_mod.apply_swing_force(false, o._plan_target_rear, dt, spawn01 * multR)

			# Unweight the slide foot while it is SWING but still grounded, so friction doesn't lock it.
			if o.phase6_swing_unweight_mult > 0.0:
				var uw: float = clampf(o.phase6_swing_unweight_mult, 0.0, 1.0)
				if front_slide and o._rb_foot_front != null:
					o._rb_foot_front.apply_central_force(Vector2(0.0, -o._rb_foot_front.mass * o._g * uw) * spawn01)
				if rear_slide and o._rb_foot_rear != null:
					o._rb_foot_rear.apply_central_force(Vector2(0.0, -o._rb_foot_rear.mass * o._g * uw) * spawn01)

		else:
			var recenter_ok: bool = (o._touchdown_ramp_t >= o.plant_touchdown_grace_sec) and (o._impact_timer <= 0.0)
			var standing_idle: bool = (not o._phase7_plan_active) and (o._front_state == GCTypes.FootPlantState.PLANTED) and (o._rear_state == GCTypes.FootPlantState.PLANTED)
			var min_sep_else: float = o.phase7_min_foot_separation_px
			var converged_else: bool = front_g and rear_g and absf(o._plant_front_x - o._plant_rear_x) < min_sep_else
			if (standing_idle or converged_else) and recenter_ok:
				o._allow_recenter_front = true
				o._allow_recenter_rear = true
				phase1b_want_front = true
				phase1b_want_rear = true
			else:
				o._allow_recenter_front = (not o.support_is_front)
				o._allow_recenter_rear = o.support_is_front

		# Converged override: keep both feet planted so recenter can run and spread them to stance width (even when Phase7 active).
		var min_sep: float = o.phase7_min_foot_separation_px
		var converged_1b: bool = front_g and rear_g and absf(o._plant_front_x - o._plant_rear_x) < min_sep
		if converged_1b:
			phase1b_want_front = true
			phase1b_want_rear = true

		o._allow_plant_forces_this_frame = o.phase1b_allow_pins and o._plant_points_wired_ok and not (dragging and o.debug_mouse_drag_suspend_pins)
		if o._foot_plant_mod != null:
			o._foot_plant_mod.step_foot_state(true, front_g, stabF, phase1b_want_front, dt)
		if o._foot_plant_mod != null:
			o._foot_plant_mod.step_foot_state(false, rear_g, stabR, phase1b_want_rear, dt)
		if o._foot_plant_mod != null:
			o._foot_plant_mod.update_plant_cache_midpoints()
			o._foot_plant_mod.apply_foot_damping_if_planted(front_g, rear_g)
		if o._foot_plant_mod != null:
			o._foot_plant_mod.apply_planted_foot_flatness(true, spawn01 * phase1b_muscle_gate)
		if o._foot_plant_mod != null:
			o._foot_plant_mod.apply_planted_foot_flatness(false, spawn01 * phase1b_muscle_gate)
		if o._recovery_mod != null:
			o._recovery_mod.update_rest_recapture_gate(front_g, rear_g, stabF, stabR, dt)

		# Visual follow (allowed; not physics)
		if o._body_root != null and o._rb_pelvis != null:
			o._body_root.global_position = o._rb_pelvis.global_position
			o._body_root.global_rotation = o._rb_pelvis.global_rotation

		if o.debug_draw:
			o.queue_redraw()

		# --- Landing micro-log (only right after touchdown; diagnostic, not spam) ---
		if o.debug_enable and o._dbg_landing_log_t > 0.0 and o._debug_mod != null:
			o._dbg_landing_log_accum += dt
			var hz := maxf(1.0, o.landing_log_hz)
			if o._dbg_landing_log_accum >= (1.0 / hz):
				o._dbg_landing_log_accum = 0.0

				var prefix: String = o._debug_mod.dbg_prefix()

				var pel_y := o._rb_pelvis.global_position.y if o._rb_pelvis != null else NAN
				var pel_vy := o._rb_pelvis.linear_velocity.y if o._rb_pelvis != null else NAN

				var kf0 := o._posture_mod.dbg_knee_rel0_deg(true) if o._posture_mod != null else NAN
				var kr0 := o._posture_mod.dbg_knee_rel0_deg(false) if o._posture_mod != null else NAN
				var krng: Vector2 = o._debug_mod.dbg_knee_range_deg()

				var satY := absf(o._dbg_Fy_pre) / maxf(1.0, o._dbg_Fy_max)

				var tauKF: float = o._debug_mod.dbg_lim_tau("KNEE_F")
				var tauKR: float = o._debug_mod.dbg_lim_tau("KNEE_R")
				var tauSP: float = o._debug_mod.dbg_lim_tau("SPINE")

				var distF: float = NAN
				var distR: float = NAN
				var dDistF: float = NAN
				var dDistR: float = NAN
				if o._rb_thigh_front != null and o._rb_shin_front != null:
					distF = o._rb_thigh_front.global_position.distance_to(o._rb_shin_front.global_position)
					if o._dbg_rest_center_dist_knee_F >= 0.0:
						dDistF = distF - o._dbg_rest_center_dist_knee_F
				if o._rb_thigh_rear != null and o._rb_shin_rear != null:
					distR = o._rb_thigh_rear.global_position.distance_to(o._rb_shin_rear.global_position)
					if o._dbg_rest_center_dist_knee_R >= 0.0:
						dDistR = distR - o._dbg_rest_center_dist_knee_R

				# Flags: what to look at first
				var flags := ""
				if satY > 0.95:
					flags += "SATY "
				if is_finite(dDistF) and absf(dDistF) > 2.0:
					flags += "SEP_F "
				if is_finite(dDistR) and absf(dDistR) > 2.0:
					flags += "SEP_R "
				if absf(tauSP) > 0.0:
					flags += "SPINE_LIM "

				print(
					prefix,
					" LAND",
					" t=", snappedf(o._t, 0.02),
					" g=", int(front_g), int(rear_g), " ge=", int(grounded_eff),
					" y=", snappedf(pel_y, 0.1),
					" vy=", snappedf(pel_vy, 0.1),
					" sy=", o._debug_mod.fmtf(o._support_y_filt),
					" yT=", o._debug_mod.fmtf(o._dbg_target_y),
					" errY=", snappedf(o._dbg_err_y, 1.0),
					" Fy=", snappedf(o._dbg_Fy_cmd, 1.0),
					" satY=", snappedf(satY, 2),
					" imp=", snappedf(o._impact_timer, 0.02),
					" td=", snappedf(o._touchdown_ramp_t, 0.02),
					" stA=", snappedf(o._stance_alpha, 0.02),
					" cmdSy=", snappedf(o.cmd_stance_y, 0.02),
					" k0(F/R)=", o._debug_mod.fmtf(kf0), "/", o._debug_mod.fmtf(kr0),
					" kRng=", snappedf(krng.x, 0.1), "..", snappedf(krng.y, 0.1),
					" limTau(KF/KR/SP)=", snappedf(tauKF, 1.0), "/", snappedf(tauKR, 1.0), "/", snappedf(tauSP, 1.0),
					" dKnee(F/R)=", o._debug_mod.fmtf(dDistF), "/", o._debug_mod.fmtf(dDistR),
					" [", flags.strip_edges(), "]"
				)

		if o.debug_enable:
			o._dbg_accum += dt
			var hz2: float = maxf(1.0, o.debug_print_hz)
			if o._dbg_accum >= (1.0 / hz2):
				o._dbg_accum = 0.0
				if o._debug_mod != null:
					o._debug_mod.print_debug(front_g, rear_g, stabF, stabR, grounded, run01)

		return true

	return false


func _tick_runtime_walk(
	dt: float,
	front_g: bool,
	rear_g: bool,
	stabF: float,
	stabR: float,
	grounded: bool,
	grounded_eff: bool,
	landing_alpha: float,
	dragging: bool,
	drag_ended: bool,
	probe_y: float,
	spawn01: float
) -> void:
	var o := _host
	if o == null:
		return

	# Never allow "air-limp" while grounded; this is the collapse-on-touchdown bug.
	o._muscle_blend = lerpf(o.air_strength_floor, 1.0, landing_alpha)
	if grounded_eff:
		o._muscle_blend = maxf(o._muscle_blend, o.landing_muscle_min)

	var com := GCMath.compute_com_world(o._refs.rb_list, o._refs.rb_mass_sum)

	# Move command -> desired world vx (cmd_move_x is world)
	var cmd := o.cmd_move_x
	if absf(cmd) < o.cmd_move_deadzone:
		cmd = 0.0
	cmd = clampf(cmd, -1.0, 1.0)

	var desired_vx: float = cmd * o.phase2_speed_px_s
	if o._locomotion_mod != null:
		o._locomotion_mod.update_lead_trail(cmd, dt)

	# Phase 7 legacy clamp only applies when legacy Phase7 is the active movement authority.
	var step_planner_authority: bool = (o.movement_authority_mode == GCTypes.MovementAuthorityMode.STEP_PLANNER)
	var legacy_phase7_authority: bool = o.phase7_enable and (o.movement_authority_mode == GCTypes.MovementAuthorityMode.LEGACY_PHASE7)
	o._dbg_movement_authority_mode = o.movement_authority_mode
	o._dbg_step_planner_active = 1 if step_planner_authority else 0

	# Brief B+: clear provisional pre-step arbitration publish every frame.
	# Prevents gc_foot_plant from consuming stale ownership if arbitration is skipped/invalid.
	o._arb_pre_front_ctrl_mode = GCTypes.FootControlMode.DISABLED
	o._arb_pre_rear_ctrl_mode = GCTypes.FootControlMode.DISABLED
	o._arb_pre_ctrl_modes_valid = false

	# Brief D: clear pipeline-published consumed slot authority every frame.
	# Prevent stale slot targets if slot selection/arbitration is skipped.
	o._arb_slot_front_x = NAN
	o._arb_slot_rear_x = NAN
	o._arb_slot_targets_valid = false

	# Brief C: clear slot-service / role-map diagnostics each frame so logs reflect THIS-FRAME truth.
	o._dbg_slot_src_valid = 0
	o._dbg_slot_mapping_mode_dbg = 0
	o._dbg_slot_center_src_dbg = 0
	o._dbg_slot_role_map_valid = 0
	o._dbg_slot_cross_blocked_count = 0
	o._dbg_slot_preland_no_cross_applied = 0
	o._dbg_slot_world_front_x_dbg = NAN
	o._dbg_slot_world_rear_x_dbg = NAN
	o._dbg_slot_sep_dbg = NAN
	o._dbg_slot_sep_min_dbg = NAN
	o._dbg_slot_front_minus_want = NAN
	o._dbg_slot_rear_minus_want = NAN

	var p7_swinging: bool = legacy_phase7_authority and (o._phase7_exec_phase == GCTypes.Phase7ExecPhase.UNPLANT or o._phase7_exec_phase == GCTypes.Phase7ExecPhase.SWING)
	o._dbg_phase7_dvx_clamped = 1 if p7_swinging else 0
	if p7_swinging:
		desired_vx *= clampf(o.phase7_swing_drive_mult, 0.0, 1.0)

	# Run planner/executor early so support + stance center inputs are ready before support_y integration.
	var planner_enable: bool = (o.movement_authority_mode == GCTypes.MovementAuthorityMode.STEP_PLANNER) \
		or o.phase7_enable or o.phase3_enable or o.phase6_enable \
		or o._spawn_brace_active or (o._planner_mode != GCTypes.PlannerMode.IDLE) or o._phase6_stance_changing
	var phase7_exec_busy_preplan: bool = o._locomotion_mod.phase7_exec_busy() if o._locomotion_mod != null else false
	if planner_enable:
		if step_planner_authority:
			var had_legacy_p7_activity: bool = o._phase7_plan_active \
				or (o._phase7_exec_phase != GCTypes.Phase7ExecPhase.IDLE) \
				or (int(o._dbg_phase7_own) != 0)
			o._legacy_phase7_force_inert()
			if had_legacy_p7_activity:
				o._dbg_dual_writer_step_phase += 1

			# Keep authored stance slot model alive (slot generation / recenter wants),
			# but Phase7 planner+executor must not co-author in STEP_PLANNER mode.
			if o._stance_planner_mod != null:
				o._stance_planner_mod.update_standing_plan(front_g, rear_g, stabF, stabR, dt, o._phase6_stance_changing)
		else:
			# Brief 2 containment:
			# while Phase 7 owns a step/executor cycle, standing planner must not co-author shared _plan_* outputs.
			if o._stance_planner_mod != null and not phase7_exec_busy_preplan:
				o._stance_planner_mod.update_standing_plan(front_g, rear_g, stabF, stabR, dt, o._phase6_stance_changing)
			o._phase7_update_locomotion_plan(cmd, dt, front_g, rear_g, grounded_eff, stabF, stabR)
			o._phase7_exec_update_state(dt, front_g, rear_g)

	# STEP_PLANNER slot-frame center publish (pre-step planner tick).
	# IMPORTANT: slot generation must follow planner-owned chassis center state, not raw pelvis x,
	# otherwise slots lag/freeze when the pelvis is physically constrained and the planner/root
	# target is still progressing.
	if step_planner_authority:
		var slot_center_x_pub: float = NAN

		# Primary source: planner/body-owned chassis center.
		if is_finite(o._stance_center_x):
			slot_center_x_pub = o._stance_center_x

		# Fallbacks (body-side only; never foot-angle-derived).
		if not is_finite(slot_center_x_pub) and is_finite(com.x):
			slot_center_x_pub = com.x
		if not is_finite(slot_center_x_pub) and o._refs != null and o._refs.rb_pelvis != null:
			slot_center_x_pub = o._refs.rb_pelvis.global_position.x
		if not is_finite(slot_center_x_pub) and o._rb_pelvis != null:
			slot_center_x_pub = o._rb_pelvis.global_position.x

		o._step_slot_center_x = slot_center_x_pub
		o._step_slot_center_valid = is_finite(slot_center_x_pub)
	else:
		o._step_slot_center_x = NAN
		o._step_slot_center_valid = false

	# 5R2-B: deterministic step planner shadow path.
	# Computes _step_* outputs in parallel for debug / future authority cutover.
	# Does NOT drive root motion or foot actuators yet.
	o._step_planner_tick_shadow(cmd, dt, front_g, rear_g, grounded_eff, stabF, stabR)

	var step_phase: int = int(o._step_phase)
	var step_outputs_valid: bool = step_planner_authority and (step_phase != GCTypes.StepPlannerPhase.DISABLED)

	var step_has_swing: bool = step_outputs_valid and bool(o._step_has_swing)
	var step_swing_is_front: bool = bool(o._step_swing_is_front)

	# Brief 2: explicit planner command bus (release vs plant).
	# These are authoritative planner intent signals for foot ownership routing.
	var step_cmd_release_front: bool = false
	var step_cmd_release_rear: bool = false
	if step_outputs_valid:
		step_cmd_release_front = bool(o._step_cmd_release_front)
		step_cmd_release_rear = bool(o._step_cmd_release_rear)

	# 5R2-J4: Slot targets must be numerically total (finite) even before planner outputs are valid.
	# Base slots come from stance planner authoritative slot service; if unavailable, use a pelvis-centered
	# ROLE-AWARE fallback (front/rear role labels, never world-left/world-right hardcoding).
	var step_front_slot_x: float = NAN
	var step_rear_slot_x: float = NAN
	var step_swing_target_x: float = NAN
	var step_slot_service_valid: bool = false
	var step_slot_used_pipeline_fallback: bool = false
	var step_slot_used_planner_override: bool = false
	var step_slot_fallback_min_sep: float = NAN

	var stance_planner = o.get("_stance_planner_mod")
	if stance_planner != null and stance_planner.has_method("get_authoritative_slot_targets_x"):
		var fb_slots: Vector2 = stance_planner.get_authoritative_slot_targets_x()
		if is_finite(fb_slots.x) and is_finite(fb_slots.y):
			step_front_slot_x = fb_slots.x
			step_rear_slot_x = fb_slots.y
			step_slot_service_valid = true

	# Role-aware pelvis fallback if stance planner slots are unavailable/invalid (keeps arbitration finite at spawn
	# without reintroducing front=world-left / rear=world-right leakage).
	if not is_finite(step_front_slot_x) or not is_finite(step_rear_slot_x):
		var px: float = (o._refs.rb_pelvis.global_position.x if o._refs != null and o._refs.rb_pelvis != null else 0.0)

		var ax_fb: float = float(o.get("_neutral_axis_sign"))
		if not is_finite(ax_fb) or ax_fb == 0.0:
			ax_fb = float(o.get("facing_sign"))
		if not is_finite(ax_fb) or ax_fb == 0.0:
			ax_fb = 1.0

		var lead_is_front_fb: bool = bool(o.get("_lead_is_front"))
		var min_sep_fb_cfg: float = maxf(0.0, float(o.get("phase7_min_foot_separation_px")))
		var min_sep_fb: float = maxf(2.0, min_sep_fb_cfg)
		step_slot_fallback_min_sep = min_sep_fb

		var half_fb: float = 0.5 * min_sep_fb
		var front_s_fb: float = +half_fb if lead_is_front_fb else -half_fb
		var rear_s_fb: float = -half_fb if lead_is_front_fb else +half_fb

		step_front_slot_x = px + (front_s_fb * ax_fb)
		step_rear_slot_x = px + (rear_s_fb * ax_fb)
		step_slot_used_pipeline_fallback = true
		o._dbg_slot_role_map_valid = 1

	# Planner overrides base slots when available and finite.
	if step_outputs_valid:
		var p_front: float = float(o._step_front_slot_x)
		var p_rear: float = float(o._step_rear_slot_x)
		if is_finite(p_front) and is_finite(p_rear):
			step_front_slot_x = p_front
			step_rear_slot_x = p_rear
			step_slot_used_planner_override = true
		var p_swing: float = float(o._step_swing_target_x)
		if is_finite(p_swing):
			step_swing_target_x = p_swing

	# Brief C: publish the FINAL consumed slot pair (post base selection + planner override) for auditing.
	var step_slots_consumed_valid: bool = is_finite(step_front_slot_x) and is_finite(step_rear_slot_x)

	# Brief D: publish the same FINAL consumed slot pair as non-debug executor authority.
	# gc_foot_plant recenter/corrective planted forces must use this source in STEP_PLANNER mode.
	o._arb_slot_front_x = step_front_slot_x
	o._arb_slot_rear_x = step_rear_slot_x
	o._arb_slot_targets_valid = step_slots_consumed_valid

	o._dbg_slot_src_valid = 1 if step_slots_consumed_valid else 0
	o._dbg_slot_world_front_x_dbg = step_front_slot_x
	o._dbg_slot_world_rear_x_dbg = step_rear_slot_x
	o._dbg_slot_sep_dbg = absf(step_front_slot_x - step_rear_slot_x) if step_slots_consumed_valid else NAN

	if step_slot_used_planner_override:
		o._dbg_slot_mapping_mode_dbg = 4
	elif step_slot_service_valid:
		o._dbg_slot_mapping_mode_dbg = 2 if int(o.get("_dbg_slot_tpl_fallback")) != 0 else 1
	elif step_slot_used_pipeline_fallback:
		o._dbg_slot_mapping_mode_dbg = 3
		if is_finite(step_slot_fallback_min_sep):
			o._dbg_slot_sep_min_dbg = step_slot_fallback_min_sep
	else:
		o._dbg_slot_mapping_mode_dbg = 0

	# Planner ownership flags (published by gc_step_planner).
	# These are not tuning flags; they control cross-module reference-frame ownership.
	var step_ref_freeze_active: bool = step_planner_authority and bool(o.get("_step_ref_freeze_active"))
	var _step_preland_shape_active: bool = step_planner_authority and bool(o.get("_step_preland_shape_active"))

	# 5R2-I: defer STEP_PLANNER swing translation/unweight until AFTER arbitration + foot-state transitions.
	var step_swing_force_ready: bool = false
	var step_swing_force_support_y: float = NAN

	# 5R2-F: support foot ownership moves to step planner in STEP_PLANNER mode.
	if step_outputs_valid:
		o.support_is_front = bool(o._step_support_is_front)
		o._dbg_arb_support_owner_step = 1
	else:
		o._dbg_arb_support_owner_step = 0

	var run01: float = clampf(absf(cmd), 0.0, 1.0)

	# Keep stance alpha alive in Phase 2
	o._stance_alpha = clampf(o.stance_height01, 0.0, 1.0)

	# Initialize stance center if needed
	if not is_finite(o._stance_center_x):
		o._stance_center_x = com.x if is_finite(com.x) else (o._rb_pelvis.global_position.x if o._rb_pelvis != null else 0.0)

	# Integrate stance center (world x)
	# During Phase7 SWING/UNPLANT: do NOT lock to support foot — let center move with step so pelvis isn't dragged backward.
	# When both feet planted after a step, use midpoint of both feet so center doesn't snap back to the unmoved support foot (torso drag opposite to movement).
	if step_outputs_valid:
		var step_slots_valid: bool = is_finite(step_front_slot_x) and is_finite(step_rear_slot_x)

		# Planner-owned reference freeze:
		# during settle/prelanding and idle in-place correction, do not re-integrate stance center from body/vx.
		# Keep the slot frame centered on the planner slot midpoint to stop pelvis/locomotion from dragging the frame.
		if step_ref_freeze_active and step_slots_valid:
			var step_target_cx_freeze: float = 0.5 * (step_front_slot_x + step_rear_slot_x)
			if o._vertical_support_mod != null:
				o._stance_center_x = o._vertical_support_mod.exp_smooth(o._stance_center_x, step_target_cx_freeze, o.phase2_idle_center_follow_hz, dt)
			else:
				o._stance_center_x = step_target_cx_freeze
		elif step_has_swing:
			# Chassis/root authority: do not let foot execution stall horizontal progression.
			o._stance_center_x += desired_vx * dt
		elif step_slots_valid:
			var step_target_cx: float = 0.5 * (step_front_slot_x + step_rear_slot_x)
			if o._vertical_support_mod != null:
				o._stance_center_x = o._vertical_support_mod.exp_smooth(o._stance_center_x, step_target_cx, o.phase2_idle_center_follow_hz, dt)
			else:
				o._stance_center_x = step_target_cx
		else:
			o._stance_center_x += desired_vx * dt
	elif o._phase7_plan_active:
		if p7_swinging:
			o._stance_center_x += desired_vx * dt
		else:
			var fx: float = o._foot_plant_mod.foot_best_x(true) if o._foot_plant_mod != null else NAN
			var rx: float = o._foot_plant_mod.foot_best_x(false) if o._foot_plant_mod != null else NAN
			var target_cx: float = NAN
			if is_finite(fx) and is_finite(rx):
				# Both feet available: use midpoint so we don't pull center toward the support foot only (avoids torso drag opposite to step).
				target_cx = 0.5 * (fx + rx)
				var dir_w: float = 0.0
				if cmd > 0.0:
					dir_w = 1.0
				elif cmd < 0.0:
					dir_w = -1.0
				target_cx += dir_w * o.phase7_support_center_bias_px
			else:
				# Single support: set stance center so support foot is already at its neutral (no net pull on support).
				var sup_x: float = o._foot_plant_mod.foot_best_x(o._plan_support_is_front) if o._foot_plant_mod != null else NAN
				if is_finite(sup_x):
					var stance_alpha: float = clampf(o._stance_alpha, 0.0, 1.0)
					var w_stand: float = o.stance_w_stand
					var w_crouch: float = o.stance_w_crouch
					var full_width: float = lerpf(w_stand, w_crouch, 1.0 - stance_alpha)
					var half: float = full_width * 0.5
					var ax: float = float(o._neutral_axis_sign)
					var lead_is_front: bool = o._phase7_seq_gait_lead_is_front if o._phase7_plan_active else o._lead_is_front
					target_cx = GCMath.center_x_for_neutral_at(sup_x, half, ax, lead_is_front, o._plan_support_is_front)
			if is_finite(target_cx):
				# Don't pull stance center backward (opposite to movement): avoids torso snapping the wrong way after a step.
				var no_back_px: float = maxf(0.0, o.get("phase7_stance_center_no_back_px") as float) if o.get("phase7_stance_center_no_back_px") != null else 0.0
				if no_back_px > 0.0 and is_finite(o._stance_center_x):
					if desired_vx < -0.1 and target_cx > o._stance_center_x + no_back_px:
						target_cx = o._stance_center_x + no_back_px
					elif desired_vx > 0.1 and target_cx < o._stance_center_x - no_back_px:
						target_cx = o._stance_center_x - no_back_px
				if o._vertical_support_mod != null:
					o._stance_center_x = o._vertical_support_mod.exp_smooth(o._stance_center_x, target_cx, o.phase7_support_center_follow_hz, dt)
				else:
					o._stance_center_x = target_cx
			else:
				o._stance_center_x += desired_vx * dt
	else:
		o._stance_center_x += desired_vx * dt

	# If a mouse drag just ended, treat current pose as the new reference.
	if drag_ended:
		var fx0 := o._foot_plant_mod.foot_best_x(true) if o._foot_plant_mod != null else NAN
		var rx0 := o._foot_plant_mod.foot_best_x(false) if o._foot_plant_mod != null else NAN
		if is_finite(fx0) and is_finite(rx0):
			o._stance_center_x = 0.5 * (fx0 + rx0)
		elif is_finite(com.x):
			o._stance_center_x = com.x
		elif o._rb_pelvis != null:
			o._stance_center_x = o._rb_pelvis.global_position.x

	# Idle behavior: when no move command, stance center follows current support/COM.
	# When STEP_PLANNER freezes reference frame, this legacy-style idle follow must be skipped,
	# otherwise it fights planner slot-frame ownership during settle/in-place correction.
	if cmd == 0.0 and not (step_planner_authority and step_ref_freeze_active):
		var follow_x := NAN
		var fx := o._foot_plant_mod.foot_best_x(true) if o._foot_plant_mod != null else NAN
		var rx := o._foot_plant_mod.foot_best_x(false) if o._foot_plant_mod != null else NAN
		var min_sep: float = o.phase7_min_foot_separation_px
		var converged: bool = is_finite(fx) and is_finite(rx) and absf(fx - rx) < min_sep
		if converged and is_finite(o._stance_center_x):
			follow_x = o._stance_center_x
		elif is_finite(fx) and is_finite(rx):
			follow_x = 0.5 * (fx + rx)
		elif is_finite(com.x):
			follow_x = com.x
		elif o._rb_pelvis != null:
			follow_x = o._rb_pelvis.global_position.x

		if is_finite(follow_x):
			if o._vertical_support_mod != null:
				o._stance_center_x = o._vertical_support_mod.exp_smooth(o._stance_center_x, follow_x, o.phase2_idle_center_follow_hz, dt)
			else:
				o._stance_center_x = follow_x

	# Refresh STEP_PLANNER slot-frame center after chassis-center integration / drag rebase.
	# The step planner tick already consumed the pre-tick publish above (previous-frame coherent),
	# but downstream executors/debug/root support should see the current-frame chassis reference.
	if step_planner_authority:
		if is_finite(o._stance_center_x):
			o._step_slot_center_x = o._stance_center_x
			o._step_slot_center_valid = true
		elif is_finite(com.x):
			o._step_slot_center_x = com.x
			o._step_slot_center_valid = true
		elif o._refs != null and o._refs.rb_pelvis != null:
			o._step_slot_center_x = o._refs.rb_pelvis.global_position.x
			o._step_slot_center_valid = is_finite(o._step_slot_center_x)
		elif o._rb_pelvis != null:
			o._step_slot_center_x = o._rb_pelvis.global_position.x
			o._step_slot_center_valid = is_finite(o._step_slot_center_x)
		else:
			o._step_slot_center_x = NAN
			o._step_slot_center_valid = false

	# Compute support Y (truth-gated by planting invariants)
	var vs2 = o._vertical_support_mod
	o._support_y_raw = vs2.compute_support_y_raw(front_g, rear_g) if vs2 != null else NAN
	if not is_finite(o._support_y_raw):
		o._support_y_raw = probe_y if is_finite(probe_y) else o._support_y_last_valid

	o._support_y_filt = vs2.filter_support_y(o._support_y_raw, dt) if vs2 != null else o._support_y_raw
	if is_finite(o._support_y_filt):
		o._support_y_last_valid = o._support_y_filt
	if _refs != null:
		_refs.support_y_raw = o._support_y_raw
		_refs.support_y_filt = o._support_y_filt

	# Upright + posture + limits (same pipeline as stand-only)
	var muscle_gate: float = o._muscle_blend
	var spawn_gate: float = spawn01
	var tau_scale_upr: float = spawn_gate * muscle_gate

	if o._posture_mod != null:
		o._posture_mod.apply_angle_pd(o._rb_pelvis, 0.0, o.pelvis_world_upright_k, o.pelvis_world_upright_d, o.pelvis_world_upright_tau_max, tau_scale_upr)
	if o._posture_mod != null:
		o._posture_mod.apply_torso_world_upright(dt, tau_scale_upr)

	# Phase 5: upright saturation flag (pelvis + torso)
	if tau_scale_upr > 0.0001 and o._rb_pelvis != null:
		var errP: float = wrapf(0.0 - o._rb_pelvis.global_rotation, -PI, PI)
		var tauP_raw: float = (o.pelvis_world_upright_k * errP) - (o.pelvis_world_upright_d * o._rb_pelvis.angular_velocity)
		var tauP_cmd: float = clampf(tauP_raw, -o.pelvis_world_upright_tau_max, o.pelvis_world_upright_tau_max) * tau_scale_upr
		var tauP_max_cmd: float = o.pelvis_world_upright_tau_max * tau_scale_upr
		if absf(tauP_cmd) >= (tauP_max_cmd * o.phase5_sat_ratio):
			o._phase5_upright_saturated = true

	if tau_scale_upr > 0.0001 and o._rb_torso != null:
		var errT: float = wrapf(0.0 - o._rb_torso.global_rotation, -PI, PI)
		var rT: float = o.torso_inertia_radius_px
		var IT: float = o._rb_torso.mass * rT * rT
		var wT: float = TAU * o.torso_upright_freq_hz
		var kT: float = IT * wT * wT
		var dT: float = 2.0 * IT * wT * o.torso_upright_zeta
		var tauT_raw: float = (kT * errT) - (dT * o._rb_torso.angular_velocity)
		var tauT_cmd: float = clampf(tauT_raw, -o.torso_upright_tau_max, o.torso_upright_tau_max) * tau_scale_upr
		var tauT_max_cmd: float = o.torso_upright_tau_max * tau_scale_upr
		if absf(tauT_cmd) >= (tauT_max_cmd * o.phase5_sat_ratio):
			o._phase5_upright_saturated = true

	# Vertical support timing split:
	# - LEGACY mode: apply here (old behavior)
	# - STEP_PLANNER mode: deferred to post-commit (5R2-L ownership cutover)
	# 5R2-L: Keep leg_len_cmd computed here, but in STEP_PLANNER mode the ACTUAL vertical support force
	# is applied later (after step_foot_state + post-transition clamp + ownership COMMIT),
	# so body support reads same-frame finalized per-foot ownership.
	var leg_len_cmd: float = lerpf(o.leg_len_crouch, o.leg_len_stand, o._stance_alpha)
	if (not step_planner_authority) and grounded_eff and is_finite(o._support_y_filt) and o._vertical_support_mod != null:
		o._vertical_support_mod.apply_vertical_support(o._support_y_filt, leg_len_cmd, dt, spawn_gate, 1.0)

		# Phase 5: vertical support saturation flag (legacy timing path only).
		var gate_eff: float = maxf(o._dbg_vsupport_gate, 0.0001)
		var satY_now: float = absf(o._dbg_Fy_cmd) / maxf(1.0, o._dbg_Fy_max * gate_eff)
		o._phase5_support_saturated = o._dbg_vsupport_called and (satY_now >= o.phase5_sat_ratio)

	# Keep airborne feet roughly flat (world angle 0) for plantability.
	# 5R2-K ownership rule:
	# In STEP_PLANNER mode this path must be inert; unified foot attitude is the single owner.
	var allow_air_pd_front: bool = o._locomotion_mod.phase7_allow_generic_air_pd(true) if o._locomotion_mod != null else true
	var allow_air_pd_rear: bool = o._locomotion_mod.phase7_allow_generic_air_pd(false) if o._locomotion_mod != null else true
	if step_planner_authority:
		allow_air_pd_front = false
		allow_air_pd_rear = false

	if o._rb_foot_front != null and not front_g and allow_air_pd_front:
		if o._posture_mod != null:
			o._posture_mod.apply_angle_pd(o._rb_foot_front, 0.0, o.foot_world_k, o.foot_world_d, o.foot_world_tau_max, spawn_gate * muscle_gate)
	if o._rb_foot_rear != null and not rear_g and allow_air_pd_rear:
		if o._posture_mod != null:
			o._posture_mod.apply_angle_pd(o._rb_foot_rear, 0.0, o.foot_world_k, o.foot_world_d, o.foot_world_tau_max, spawn_gate * muscle_gate)

	if o._posture_mod != null:
		o._posture_mod.apply_posture_phase1b(spawn_gate * muscle_gate)
	if o.enable_soft_joint_limits and o._rest_seeded:
		if o._posture_mod != null:
			o._posture_mod.apply_soft_limits_phase1b(spawn_gate)

	# Phase 5: update stable timer + latch debug bits AFTER limits
	if not o.phase5_spine_limits_enable:
		o._phase5_spine_stable_t = 0.0
	else:
		var phase5_stable: bool = grounded_eff and (o._impact_timer <= 0.0) \
			and (not o._phase5_upright_saturated) \
			and (not o._phase5_support_saturated) \
			and (not o._phase5_limit_saturated)
		if phase5_stable:
			o._phase5_spine_stable_t = minf(o._phase5_spine_stable_t + dt, o.phase5_spine_reenable_sec * 2.0)
		else:
			o._phase5_spine_stable_t = 0.0

	o._dbg_phase5_upr_sat = 1 if o._phase5_upright_saturated else 0
	o._dbg_phase5_sup_sat = 1 if o._phase5_support_saturated else 0
	o._dbg_phase5_lim_sat = 1 if o._phase5_limit_saturated else 0

	# 5R2-F: explicit per-foot arbitration outputs (pipeline stage).
	# These are the authoritative per-frame control modes in STEP_PLANNER mode.
	var front_ctrl_mode: int = GCTypes.FootControlMode.DISABLED
	var rear_ctrl_mode: int = GCTypes.FootControlMode.DISABLED
	var arb_front_slide: bool = false
	var arb_rear_slide: bool = false

	# Brief A: foot attitude ownership is a separate channel from control ownership.
	# This prevents ctrl_mode==DISABLED from creating no-attitude-owner gaps during preland/settle windows.
	var front_attitude_mode: int = GCTypes.FootAttitudeMode.DISABLED
	var rear_attitude_mode: int = GCTypes.FootAttitudeMode.DISABLED

	# Determine default planting desires
	var want_front: bool = front_g
	var want_rear: bool = rear_g

	if planner_enable:
		var front_allow: bool = front_g or (o._truth_front_allow_t > 0.0)
		var rear_allow: bool = rear_g or (o._truth_rear_allow_t > 0.0)

		if step_outputs_valid:
			# STEP_PLANNER authority path:
			# - planner decides plant/release intent
			# - pipeline applies physical overlay (friction/swing force/unweight)
			# - legacy Phase7 executor remains inert
			# Planner command bus contract:
			# - cmd_release_* owns swing/unplant intent
			# - cmd_plant_* owns planted/correction intent
			# A foot cannot be both RELEASE-owned and plant-wanted in the same frame.
			want_front = bool(o._step_cmd_plant_front) and (not step_cmd_release_front) and front_allow
			want_rear = bool(o._step_cmd_plant_rear) and (not step_cmd_release_rear) and rear_allow

			var recenter_ok_step: bool = (o._touchdown_ramp_t >= o.plant_touchdown_grace_sec) and (o._impact_timer <= 0.0)
			o._allow_recenter_front = recenter_ok_step and want_front
			o._allow_recenter_rear = recenter_ok_step and want_rear

			var step_swing_active: bool = step_has_swing and (
				step_phase == GCTypes.StepPlannerPhase.SINGLE_SUPPORT_SWING
				or step_phase == GCTypes.StepPlannerPhase.TRANSFER
			)

			# 5R2-I: Friction + swing translation are no longer written here in STEP_PLANNER mode.
			# They are arbitration-owned outputs and will be applied AFTER per-foot control modes
			# and AFTER foot state transitions (so candidate/planted can block swing force cleanly).
			if step_swing_active and is_finite(step_swing_target_x):
				step_swing_force_support_y = o._support_y_filt
				if not is_finite(step_swing_force_support_y):
					step_swing_force_support_y = o._support_y_last_valid
				if not is_finite(step_swing_force_support_y):
					if step_swing_is_front and o._rb_foot_front != null:
						step_swing_force_support_y = o._rb_foot_front.global_position.y
					elif (not step_swing_is_front) and o._rb_foot_rear != null:
						step_swing_force_support_y = o._rb_foot_rear.global_position.y
				step_swing_force_ready = is_finite(step_swing_force_support_y)

		else:
			var use_planner: bool = o._phase7_plan_active or o.phase3_enable or (o._planner_mode != GCTypes.PlannerMode.IDLE) or o._phase6_stance_changing

			if use_planner:
				want_front = ((o._plan_front == GCTypes.PlanFoot.PLANTED) or o._phase7_force_plant_front) and front_allow
				want_rear = ((o._plan_rear == GCTypes.PlanFoot.PLANTED) or o._phase7_force_plant_rear) and rear_allow

			var min_sep_p2: float = o.phase7_min_foot_separation_px
			var converged_p2: bool = front_g and rear_g and absf(o._plant_front_x - o._plant_rear_x) < min_sep_p2
			var p7_exec_idle: bool = (o._phase7_exec_phase == GCTypes.Phase7ExecPhase.IDLE) or (not o._phase7_plan_active)
			# Converged override: only when Phase 7 is idle so we don't force both feet planted during a step.
			if converged_p2 and p7_exec_idle:
				want_front = true
				want_rear = true

			var recenter_ok_p2: bool = (o._touchdown_ramp_t >= o.plant_touchdown_grace_sec) and (o._impact_timer <= 0.0)
			var both_planted_p2: bool = (o._front_state == GCTypes.FootPlantState.PLANTED) and (o._rear_state == GCTypes.FootPlantState.PLANTED)

			# Stance-shape recenter window:
			# When both feet are planted and Phase 7 is idle, allow BOTH feet to recenter.
			# This is required for asymmetric slot updates (e.g. crouch depth changes) where the moved slot may belong to the support foot.
			var standing_idle_p2: bool = (not o._phase7_plan_active) and both_planted_p2
			var dual_slot_shape_recenter_p2: bool = standing_idle_p2 and recenter_ok_p2 \
				and ((o._planner_mode == GCTypes.PlannerMode.IDLE) or o._phase6_stance_changing)

			if dual_slot_shape_recenter_p2:
				o._allow_recenter_front = true
				o._allow_recenter_rear = true
			elif (both_planted_p2 or converged_p2) and recenter_ok_p2 and p7_exec_idle:
				o._allow_recenter_front = not o.support_is_front
				o._allow_recenter_rear = o.support_is_front
			elif o._planner_mode == GCTypes.PlannerMode.BRACE or o._planner_mode == GCTypes.PlannerMode.RECOVER:
				o._allow_recenter_front = recenter_ok_p2 and (o._plan_front == GCTypes.PlanFoot.PLANTED)
				o._allow_recenter_rear = recenter_ok_p2 and (o._plan_rear == GCTypes.PlanFoot.PLANTED)
			else:
				o._allow_recenter_front = (not o.support_is_front)
				o._allow_recenter_rear = o.support_is_front

			# Keep both feet plant-desired during the same window so recenter can actually execute.
			if dual_slot_shape_recenter_p2:
				want_front = true
				want_rear = true

			var slide_ok: bool = (o._planner_mode != GCTypes.PlannerMode.IDLE) or o._phase6_stance_changing
			if (o._touchdown_ramp_t < o.plant_touchdown_grace_sec) or (o._impact_timer > 0.0):
				slide_ok = false
			var front_slide: bool = use_planner and slide_ok and front_g and (o._plan_front == GCTypes.PlanFoot.SWING) and (not o._phase7_plan_active)
			var rear_slide: bool = use_planner and slide_ok and rear_g and (o._plan_rear == GCTypes.PlanFoot.SWING) and (not o._phase7_plan_active)

			o._dbg_front_slide = front_slide or o._phase7_slide_front
			o._dbg_rear_slide = rear_slide or o._phase7_slide_rear

			if o._recovery_mod != null:
				o._recovery_mod.apply_foot_friction(front_slide or o._phase7_slide_front, rear_slide or o._phase7_slide_rear)
			else:
				o._phase6_apply_foot_friction(front_slide or o._phase7_slide_front, rear_slide or o._phase7_slide_rear)

			if use_planner:
				var swing_boost2: float = 1.0
				if o._planner_mode == GCTypes.PlannerMode.BRACE or o._planner_mode == GCTypes.PlannerMode.RECOVER:
					swing_boost2 = o.phase6_swing_force_mult_recover
				elif o._phase6_stance_changing:
					swing_boost2 = o.phase6_swing_force_mult_stance_change

				if front_slide and o._rb_foot_front != null and is_finite(o._plan_target_front.x):
					var multF2: float = swing_boost2 * o.phase6_slide_force_mult
					if o._ground_truth_mod != null:
						o._ground_truth_mod.apply_swing_force(true, o._plan_target_front, dt, spawn_gate * multF2)

				if rear_slide and o._rb_foot_rear != null and is_finite(o._plan_target_rear.x):
					var multR2: float = swing_boost2 * o.phase6_slide_force_mult
					if o._ground_truth_mod != null:
						o._ground_truth_mod.apply_swing_force(false, o._plan_target_rear, dt, spawn_gate * multR2)

				if o.phase6_swing_unweight_mult > 0.0:
					var uw2: float = clampf(o.phase6_swing_unweight_mult, 0.0, 1.0)
					if front_slide and o._rb_foot_front != null:
						o._rb_foot_front.apply_central_force(Vector2(0.0, -o._rb_foot_front.mass * o._g * uw2) * spawn_gate)
					if rear_slide and o._rb_foot_rear != null:
						o._rb_foot_rear.apply_central_force(Vector2(0.0, -o._rb_foot_rear.mass * o._g * uw2) * spawn_gate)

				o._phase7_exec_apply_forces(dt, spawn_gate, front_g, rear_g)
	else:
		var recenter_ok: bool = (o._touchdown_ramp_t >= o.plant_touchdown_grace_sec) and (o._impact_timer <= 0.0)
		var standing_idle: bool = (not o._phase7_plan_active) and (o._front_state == GCTypes.FootPlantState.PLANTED) and (o._rear_state == GCTypes.FootPlantState.PLANTED)
		if standing_idle and recenter_ok:
			# Non-planner path should still permit stance-shape correction on BOTH feet.
			o._allow_recenter_front = true
			o._allow_recenter_rear = true
		else:
			o._allow_recenter_front = (not o.support_is_front)
			o._allow_recenter_rear = o.support_is_front
		if o._recovery_mod != null:
			o._recovery_mod.apply_foot_friction(false, false)
		else:
			o._phase6_apply_foot_friction(false, false)

	# =========================================================================
	# 5R2-F: Arbitration / exclusion firewall (single owner per foot per frame)
	# Order:
	# 1) planners produce intent (already done above)
	# 2) pipeline assigns per-foot control mode
	# 3) pipeline canonicalizes want/recenter/friction outputs
	# =========================================================================
	var arb_contact_allow_front: bool = front_g or (o._truth_front_allow_t > 0.0)
	var arb_contact_allow_rear: bool = rear_g or (o._truth_rear_allow_t > 0.0)
	var arb_recenter_ok: bool = (o._touchdown_ramp_t >= o.plant_touchdown_grace_sec) and (o._impact_timer <= 0.0)

	var arb_step_recover: bool = step_outputs_valid and (step_phase == GCTypes.StepPlannerPhase.RECOVER)
	var arb_step_swing_active: bool = step_outputs_valid and step_has_swing and (
		step_phase == GCTypes.StepPlannerPhase.SINGLE_SUPPORT_SWING
		or step_phase == GCTypes.StepPlannerPhase.TRANSFER
	)

	# Brief 2: planner-release ownership is more precise than phase labels alone.
	# The planner may keep a sequence active for settle/advance bookkeeping while no longer
	# wanting RELEASE ownership on the touchdown foot.
	var arb_step_release_front: bool = step_outputs_valid and arb_step_swing_active and step_swing_is_front and step_cmd_release_front
	var arb_step_release_rear: bool = step_outputs_valid and arb_step_swing_active and (not step_swing_is_front) and step_cmd_release_rear

	var pre_want_front: bool = want_front
	var pre_want_rear: bool = want_rear
	var pre_recenter_front: bool = o._allow_recenter_front
	var pre_recenter_rear: bool = o._allow_recenter_rear

	# 5R2-H: planner-slot shape invariant (debug + arbitration inputs)
	# These describe whether STEP_PLANNER is requesting double-support slot-shape correction
	# (including support-foot correction) from authoritative planner slots.
	var arb_shape_dual: bool = false
	var arb_shape_need_front: bool = false
	var arb_shape_need_rear: bool = false
	var arb_shape_err_front: float = NAN
	var arb_shape_err_rear: float = NAN
	var arb_shape_slot_sep: float = NAN

	if step_outputs_valid:
		var front_is_support: bool = bool(o.support_is_front)
		var rear_is_support: bool = not front_is_support

		# 5R2-J: Exec-state truth (PRE-transition) for DISABLED-hole fallback only.
		# NOTE: Arbitration runs before step_foot_state(), so these are previous-frame states.
		# Do NOT use these to suppress SWING starts.
		var front_exec_planted_pre: bool = (o._front_state == GCTypes.FootPlantState.PLANTED)
		var rear_exec_planted_pre: bool = (o._rear_state == GCTypes.FootPlantState.PLANTED)

		# 5R2-J3: Clear swing-ended latch when planner phase/step changes or planner no longer swing-active for that foot.
		var prev_step_id: int = o._prev_step_id
		var prev_step_phase: int = o._prev_step_phase
		if o._step_id != prev_step_id or step_phase != prev_step_phase or not (arb_step_swing_active and step_swing_is_front):
			o._front_swing_ended_latch = false
		if o._step_id != prev_step_id or step_phase != prev_step_phase or not (arb_step_swing_active and (not step_swing_is_front)):
			o._rear_swing_ended_latch = false

		# Front foot mode
		if arb_step_recover:
			front_ctrl_mode = GCTypes.FootControlMode.RECOVERY if arb_contact_allow_front else GCTypes.FootControlMode.DISABLED
		elif arb_step_release_front and not o._front_swing_ended_latch:
			front_ctrl_mode = GCTypes.FootControlMode.SWING
		elif pre_want_front and arb_contact_allow_front:
			front_ctrl_mode = GCTypes.FootControlMode.PLANTED_SUPPORT if front_is_support else GCTypes.FootControlMode.PLANTED_CORRECT
		elif front_exec_planted_pre and arb_contact_allow_front:
			# 5R2-J fallback: previous-frame exec truth says planted and we still allow contact.
			# Keep planted ownership alive to avoid one-frame DISABLED holes.
			front_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT
		else:
			front_ctrl_mode = GCTypes.FootControlMode.DISABLED

		# Rear foot mode
		if arb_step_recover:
			rear_ctrl_mode = GCTypes.FootControlMode.RECOVERY if arb_contact_allow_rear else GCTypes.FootControlMode.DISABLED
		elif arb_step_release_rear and not o._rear_swing_ended_latch:
			rear_ctrl_mode = GCTypes.FootControlMode.SWING
		elif pre_want_rear and arb_contact_allow_rear:
			rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_SUPPORT if rear_is_support else GCTypes.FootControlMode.PLANTED_CORRECT
		elif rear_exec_planted_pre and arb_contact_allow_rear:
			# 5R2-J fallback: previous-frame exec truth says planted and we still allow contact.
			# Keep planted ownership alive to avoid one-frame DISABLED holes.
			rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT
		else:
			rear_ctrl_mode = GCTypes.FootControlMode.DISABLED

		# 5R2-J: Always-on planner-slot shape invariant (exec-truth-gated dual-plant correction).
		# Do NOT gate on planner phase labels. Real feet can both be planted before planner labels catch up.
		var shape_slots_valid: bool = is_finite(step_front_slot_x) and is_finite(step_rear_slot_x)
		var shape_both_planted: bool = (o._front_state == GCTypes.FootPlantState.PLANTED) and (o._rear_state == GCTypes.FootPlantState.PLANTED)
		var shape_contact_ok: bool = arb_contact_allow_front and arb_contact_allow_rear
		var shape_safe_window: bool = (not arb_step_recover) and arb_recenter_ok

		arb_shape_dual = shape_slots_valid and shape_both_planted and shape_contact_ok and shape_safe_window

		var shape_fx: float = o._foot_plant_mod.foot_best_x(true) if o._foot_plant_mod != null else float(o._plant_front_x)
		var shape_rx: float = o._foot_plant_mod.foot_best_x(false) if o._foot_plant_mod != null else float(o._plant_rear_x)

		if arb_shape_dual and is_finite(shape_fx) and is_finite(shape_rx):
			arb_shape_err_front = absf(step_front_slot_x - shape_fx)
			arb_shape_err_rear = absf(step_rear_slot_x - shape_rx)
			arb_shape_slot_sep = absf(step_front_slot_x - step_rear_slot_x)

			# Use existing authored-slot deadband as the correction threshold baseline.
			# This is a slot-space invariant, not an angle-space heuristic.
			var shape_eps_on: float = maxf(0.5, float(o.stance_anchor_deadband_px))
			var shape_eps_off: float = maxf(0.25, shape_eps_on * 0.5)

			# Hysteresis via prior recenter allowance prevents chattering around threshold.
			arb_shape_need_front = arb_shape_err_front > (shape_eps_off if pre_recenter_front else shape_eps_on)
			arb_shape_need_rear = arb_shape_err_rear > (shape_eps_off if pre_recenter_rear else shape_eps_on)

			# If planner wants planted contact and the foot is currently the SUPPORT foot,
			# allow arbitration to upgrade it to PLANTED_CORRECT when slot error is non-trivial.
			# This removes the old "support foot immunity" that prevented stance-shape recovery.
			if arb_shape_need_front and front_ctrl_mode == GCTypes.FootControlMode.PLANTED_SUPPORT:
				front_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT
			if arb_shape_need_rear and rear_ctrl_mode == GCTypes.FootControlMode.PLANTED_SUPPORT:
				rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT

		# Exclusion violation accounting (detect legacy/provisional leakage before canonical override)
		if front_ctrl_mode == GCTypes.FootControlMode.SWING and (pre_want_front or pre_recenter_front):
			o._dbg_arb_excl_violations += 1
		if rear_ctrl_mode == GCTypes.FootControlMode.SWING and (pre_want_rear or pre_recenter_rear):
			o._dbg_arb_excl_violations += 1

		# Canonicalize foot commands from arbitration outputs.
		match front_ctrl_mode:
			GCTypes.FootControlMode.SWING:
				want_front = false
				o._allow_recenter_front = false
				arb_front_slide = true
			GCTypes.FootControlMode.PLANTED_SUPPORT:
				want_front = arb_contact_allow_front
				o._allow_recenter_front = false
				arb_front_slide = false
			GCTypes.FootControlMode.PLANTED_CORRECT:
				want_front = arb_contact_allow_front
				o._allow_recenter_front = arb_recenter_ok and want_front
				arb_front_slide = false
			GCTypes.FootControlMode.RECOVERY:
				want_front = arb_contact_allow_front and bool(o._step_cmd_plant_front) and (not step_cmd_release_front)
				o._allow_recenter_front = false
				arb_front_slide = false
			_:
				want_front = false
				o._allow_recenter_front = false
				arb_front_slide = false

		match rear_ctrl_mode:
			GCTypes.FootControlMode.SWING:
				want_rear = false
				o._allow_recenter_rear = false
				arb_rear_slide = true
			GCTypes.FootControlMode.PLANTED_SUPPORT:
				want_rear = arb_contact_allow_rear
				o._allow_recenter_rear = false
				arb_rear_slide = false
			GCTypes.FootControlMode.PLANTED_CORRECT:
				want_rear = arb_contact_allow_rear
				o._allow_recenter_rear = arb_recenter_ok and want_rear
				arb_rear_slide = false
			GCTypes.FootControlMode.RECOVERY:
				want_rear = arb_contact_allow_rear and bool(o._step_cmd_plant_rear) and (not step_cmd_release_rear)
				o._allow_recenter_rear = false
				arb_rear_slide = false
			_:
				want_rear = false
				o._allow_recenter_rear = false
				arb_rear_slide = false

		# Friction mode is an arbitration output in STEP_PLANNER mode (not inferred by multiple systems).
		o._dbg_front_slide = arb_front_slide
		o._dbg_rear_slide = arb_rear_slide
		if o._recovery_mod != null:
			o._recovery_mod.apply_foot_friction(arb_front_slide, arb_rear_slide)
		else:
			o._phase6_apply_foot_friction(arb_front_slide, arb_rear_slide)
	else:
		# Legacy path remains as-is behavior-wise; expose a best-effort mode mirror for debug readability.
		if want_front and front_g:
			front_ctrl_mode = GCTypes.FootControlMode.PLANTED_SUPPORT if bool(o.support_is_front) else GCTypes.FootControlMode.PLANTED_CORRECT
		elif o._dbg_front_slide:
			front_ctrl_mode = GCTypes.FootControlMode.SWING
		else:
			front_ctrl_mode = GCTypes.FootControlMode.DISABLED

		if want_rear and rear_g:
			rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_SUPPORT if (not bool(o.support_is_front)) else GCTypes.FootControlMode.PLANTED_CORRECT
		elif o._dbg_rear_slide:
			rear_ctrl_mode = GCTypes.FootControlMode.SWING
		else:
			rear_ctrl_mode = GCTypes.FootControlMode.DISABLED

		arb_front_slide = bool(o._dbg_front_slide)
		arb_rear_slide = bool(o._dbg_rear_slide)

	# 5R2-H/5R2-K debug: planner-slot shape correction visibility
	# ctrl_mode/slide debug published below is provisional and may be overwritten by the post-transition
	# ownership COMMIT block. Shape debug values may also be revalidated post-step (same-frame exec truth).
	o._dbg_arb_shape_dual = 1 if arb_shape_dual else 0
	o._dbg_arb_shape_need_front = 1 if arb_shape_need_front else 0
	o._dbg_arb_shape_need_rear = 1 if arb_shape_need_rear else 0
	o._dbg_step_slot_err_front = arb_shape_err_front
	o._dbg_step_slot_err_rear = arb_shape_err_rear
	o._dbg_step_slot_sep = arb_shape_slot_sep

	o._dbg_foot_ctrl_mode_front = front_ctrl_mode
	o._dbg_foot_ctrl_mode_rear = rear_ctrl_mode
	o._dbg_arb_slide_front = 1 if arb_front_slide else 0
	o._dbg_arb_slide_rear = 1 if arb_rear_slide else 0

	# Brief B+: same-frame provisional ctrl ownership for gc_foot_plant (pre-COMMIT executor path).
	# This is a migration bridge; final truth remains the post-transition _arb_* COMMIT publish.
	if step_planner_authority and step_outputs_valid:
		o._arb_pre_front_ctrl_mode = front_ctrl_mode
		o._arb_pre_rear_ctrl_mode = rear_ctrl_mode
		o._arb_pre_ctrl_modes_valid = true

	# Legacy Phase7 helper gating (recenter) is legacy-executor-owned only.
	# In STEP_PLANNER mode, pipeline arbitration is authoritative and legacy Phase7 helpers must not rewrite
	# recenter/attitude decisions for either foot. (5R2-F + 5R2-G ownership firewall.)
	if o._locomotion_mod != null:
		if step_planner_authority:
			o._dbg_arb_phase7_helper_blocked += 1
		else:
			o._locomotion_mod.phase7_apply_recenter_gates()

	# Pins allowed this frame?
	o._allow_plant_forces_this_frame = o.phase2_allow_pins and o._plant_points_wired_ok and not (dragging and o.debug_mouse_drag_suspend_pins)

	# 5R2-J3: Track previous exec foot states to detect swing termination.
	var prev_front_state: int = o._front_state
	var prev_rear_state: int = o._rear_state

	if o._foot_plant_mod != null:
		o._foot_plant_mod.step_foot_state(true, front_g, stabF, want_front, dt)
	if o._foot_plant_mod != null:
		o._foot_plant_mod.step_foot_state(false, rear_g, stabR, want_rear, dt)

	# 5R2-J: Post-transition per-foot ownership clamp.
	# Arbitration above runs BEFORE step_foot_state(), so ctrl modes selected there can be stale for one frame.
	# Clamp using THIS-FRAME exec states to prevent:
	# - stale SWING mode after touchdown (exec is CANDIDATE/PLANTED but ctrl mode still SWING)
	# - missing SWING mode after an unplant (exec is SWING but ctrl mode stayed planted)
	if step_planner_authority and step_outputs_valid:
		var front_state_now := o._front_state
		var rear_state_now := o._rear_state

		# If exec says "not SWING", SWING mode is invalid this frame.
		if front_ctrl_mode == GCTypes.FootControlMode.SWING and front_state_now != GCTypes.FootPlantState.SWING:
			front_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT if arb_contact_allow_front else GCTypes.FootControlMode.DISABLED
		if rear_ctrl_mode == GCTypes.FootControlMode.SWING and rear_state_now != GCTypes.FootPlantState.SWING:
			rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT if arb_contact_allow_rear else GCTypes.FootControlMode.DISABLED

		# If exec says SWING and planner says swing-active for that foot, enforce SWING mode.
		if front_state_now == GCTypes.FootPlantState.SWING and arb_step_swing_active and step_swing_is_front:
			front_ctrl_mode = GCTypes.FootControlMode.SWING
		if rear_state_now == GCTypes.FootPlantState.SWING and arb_step_swing_active and (not step_swing_is_front):
			rear_ctrl_mode = GCTypes.FootControlMode.SWING

	# 5R2-K: Revalidate planner-slot shape correction AFTER step_foot_state().
	# Pre-step arbitration can miss same-frame dual-plant windows because exec states transition here.
	# Re-check using THIS-FRAME exec truth before ownership COMMIT.
	if step_planner_authority and step_outputs_valid:
		var shape_slots_valid_post: bool = is_finite(step_front_slot_x) and is_finite(step_rear_slot_x)
		var shape_both_planted_post: bool = (o._front_state == GCTypes.FootPlantState.PLANTED) and (o._rear_state == GCTypes.FootPlantState.PLANTED)
		var shape_contact_ok_post: bool = arb_contact_allow_front and arb_contact_allow_rear
		var shape_safe_window_post: bool = (not arb_step_recover) and arb_recenter_ok

		arb_shape_dual = shape_slots_valid_post and shape_both_planted_post and shape_contact_ok_post and shape_safe_window_post
		arb_shape_need_front = false
		arb_shape_need_rear = false
		arb_shape_err_front = NAN
		arb_shape_err_rear = NAN
		arb_shape_slot_sep = NAN

		var shape_fx_post: float = o._foot_plant_mod.foot_best_x(true) if o._foot_plant_mod != null else float(o._plant_front_x)
		var shape_rx_post: float = o._foot_plant_mod.foot_best_x(false) if o._foot_plant_mod != null else float(o._plant_rear_x)

		if arb_shape_dual and is_finite(shape_fx_post) and is_finite(shape_rx_post):
			arb_shape_err_front = absf(step_front_slot_x - shape_fx_post)
			arb_shape_err_rear = absf(step_rear_slot_x - shape_rx_post)
			arb_shape_slot_sep = absf(step_front_slot_x - step_rear_slot_x)

			var shape_eps_on_post: float = maxf(0.5, float(o.stance_anchor_deadband_px))
			var shape_eps_off_post: float = maxf(0.25, shape_eps_on_post * 0.5)

			arb_shape_need_front = arb_shape_err_front > (shape_eps_off_post if pre_recenter_front else shape_eps_on_post)
			arb_shape_need_rear = arb_shape_err_rear > (shape_eps_off_post if pre_recenter_rear else shape_eps_on_post)

			# Brief M+: if planner is explicitly claiming a no-input corrective STEP, do not let
			# post-step slot-shape logic solve it via PLANTED_CORRECT/recenter dragging this frame.
			var planner_idle_step_claim: bool = (
				(step_phase == GCTypes.StepPlannerPhase.DOUBLE_SUPPORT)
				and (not step_has_swing)
				and bool(o.get("_step_idle_step_correction"))
			)

			# Defensive downgrade in case a provisional path upgraded earlier.
			if planner_idle_step_claim:
				if front_ctrl_mode == GCTypes.FootControlMode.PLANTED_CORRECT:
					front_ctrl_mode = GCTypes.FootControlMode.PLANTED_SUPPORT
				if rear_ctrl_mode == GCTypes.FootControlMode.PLANTED_CORRECT:
					rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_SUPPORT

			# Only upgrade planted support -> planted correct when planner is NOT claiming a corrective step.
			# Never force swing/recovery here.
			if (not planner_idle_step_claim) and arb_shape_need_front and front_ctrl_mode == GCTypes.FootControlMode.PLANTED_SUPPORT:
				front_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT
			if (not planner_idle_step_claim) and arb_shape_need_rear and rear_ctrl_mode == GCTypes.FootControlMode.PLANTED_SUPPORT:
				rear_ctrl_mode = GCTypes.FootControlMode.PLANTED_CORRECT

	# Brief C: publish FINAL (post-step revalidated) slot-shape debug and slot-vs-want mismatch.
	# Pre-step debug is useful for arbitration inspection, but this is the frame-truth version after executor transitions.
	o._dbg_arb_shape_dual = 1 if arb_shape_dual else 0
	o._dbg_arb_shape_need_front = 1 if arb_shape_need_front else 0
	o._dbg_arb_shape_need_rear = 1 if arb_shape_need_rear else 0
	o._dbg_step_slot_err_front = arb_shape_err_front
	o._dbg_step_slot_err_rear = arb_shape_err_rear
	o._dbg_step_slot_sep = arb_shape_slot_sep

	var recenter_want_f_dbg: float = float(o.get("_dbg_recenter_want_f"))
	var recenter_want_r_dbg: float = float(o.get("_dbg_recenter_want_r"))
	o._dbg_slot_front_minus_want = (step_front_slot_x - recenter_want_f_dbg) if (is_finite(step_front_slot_x) and is_finite(recenter_want_f_dbg)) else NAN
	o._dbg_slot_rear_minus_want = (step_rear_slot_x - recenter_want_r_dbg) if (is_finite(step_rear_slot_x) and is_finite(recenter_want_r_dbg)) else NAN

	# 5R2-J3: Detect swing termination events and set latch so arbitration cannot re-enter SWING until planner acknowledges.
	var front_swing_ended: bool = (prev_front_state == GCTypes.FootPlantState.SWING and o._front_state != GCTypes.FootPlantState.SWING)
	var rear_swing_ended: bool = (prev_rear_state == GCTypes.FootPlantState.SWING and o._rear_state != GCTypes.FootPlantState.SWING)
	if front_swing_ended:
		o._front_swing_ended_latch = true
	if rear_swing_ended:
		o._rear_swing_ended_latch = true

	# 5R2-K: Post-transition ownership COMMIT (authoritative publish).
	# Why:
	# - Arbitration/debug/friction were computed before step_foot_state(), so they can be one frame stale.
	# - We already post-clamp front_ctrl_mode/rear_ctrl_mode using THIS-FRAME exec states.
	# - Publish friction + debug from the FINAL clamped modes so subsystems stop disagreeing in the same frame.
	if step_planner_authority and step_outputs_valid:
		# Recompute slide/friction from FINAL ctrl modes (post step_foot_state + post-clamp).
		match front_ctrl_mode:
			GCTypes.FootControlMode.SWING:
				arb_front_slide = true
				o._allow_recenter_front = false
			GCTypes.FootControlMode.PLANTED_SUPPORT:
				arb_front_slide = false
				o._allow_recenter_front = false
			GCTypes.FootControlMode.PLANTED_CORRECT:
				arb_front_slide = false
				o._allow_recenter_front = arb_recenter_ok and arb_contact_allow_front
			GCTypes.FootControlMode.RECOVERY:
				arb_front_slide = false
				o._allow_recenter_front = false
			_:
				arb_front_slide = false
				o._allow_recenter_front = false

		match rear_ctrl_mode:
			GCTypes.FootControlMode.SWING:
				arb_rear_slide = true
				o._allow_recenter_rear = false
			GCTypes.FootControlMode.PLANTED_SUPPORT:
				arb_rear_slide = false
				o._allow_recenter_rear = false
			GCTypes.FootControlMode.PLANTED_CORRECT:
				arb_rear_slide = false
				o._allow_recenter_rear = arb_recenter_ok and arb_contact_allow_rear
			GCTypes.FootControlMode.RECOVERY:
				arb_rear_slide = false
				o._allow_recenter_rear = false
			_:
				arb_rear_slide = false
				o._allow_recenter_rear = false

		# Final published friction ownership (overwrite any provisional pre-step publish this frame).
		o._dbg_front_slide = arb_front_slide
		o._dbg_rear_slide = arb_rear_slide
		if o._recovery_mod != null:
			o._recovery_mod.apply_foot_friction(arb_front_slide, arb_rear_slide)
		else:
			o._phase6_apply_foot_friction(arb_front_slide, arb_rear_slide)

		# Final committed arbitration publish (authoritative, non-debug).
		o._arb_front_ctrl_mode = front_ctrl_mode
		o._arb_rear_ctrl_mode = rear_ctrl_mode
		o._arb_front_slide = arb_front_slide
		o._arb_rear_slide = arb_rear_slide
		o._arb_ctrl_modes_valid = true

		# Final debug truth must match the final committed modes used by attitude/swing-force paths.
		o._dbg_foot_ctrl_mode_front = front_ctrl_mode
		o._dbg_foot_ctrl_mode_rear = rear_ctrl_mode
		o._dbg_arb_slide_front = 1 if arb_front_slide else 0
		o._dbg_arb_slide_rear = 1 if arb_rear_slide else 0

		# Brief M+: no post-COMMIT recenter executor in STEP_PLANNER.
		# Idle stance correction must happen via planner-owned lift→land corrective sequences, not planted drag.

	# 5R2-L: STEP_PLANNER vertical support must run AFTER final ownership COMMIT.
	# This lets gc_vertical_support read same-frame committed ctrl modes (_arb_*) instead of previous-frame
	# or fallback ownership, which is the body-side leak causing landing/idle jitter and "dragged skates."
	if step_planner_authority and grounded_eff and is_finite(o._support_y_filt) and o._vertical_support_mod != null:
		o._vertical_support_mod.apply_vertical_support(o._support_y_filt, leg_len_cmd, dt, spawn_gate, 1.0)

		# Refresh support saturation debug/flag from the deferred STEP_PLANNER support call.
		# (Phase5 stable-timer earlier in the frame may still see prior support-sat state; acceptable for this ownership pass.)
		var gate_eff_step: float = maxf(o._dbg_vsupport_gate, 0.0001)
		var satY_now_step: float = absf(o._dbg_Fy_cmd) / maxf(1.0, o._dbg_Fy_max * gate_eff_step)
		o._phase5_support_saturated = o._dbg_vsupport_called and (satY_now_step >= o.phase5_sat_ratio)
		o._dbg_phase5_sup_sat = 1 if o._phase5_support_saturated else 0

	# 5R2-I: STEP_PLANNER swing translation/unweight is arbitration-owned and foot-state-gated.
	# Only apply swing translation if:
	# - pipeline arbitration says this foot is SWING
	# - planner swing is active for this foot
	# - foot plant state is still SWING (not CANDIDATE / not PLANTED)
	o._dbg_arb_swing_force_front = 0
	o._dbg_arb_swing_force_rear = 0
	o._dbg_arb_swing_force_blocked = 0

	if step_planner_authority and o._ground_truth_mod != null:
		var step_swing_phase_active_exec: bool = step_outputs_valid and step_has_swing and (
			step_phase == GCTypes.StepPlannerPhase.SINGLE_SUPPORT_SWING
			or step_phase == GCTypes.StepPlannerPhase.TRANSFER
		)

		var step_swing_release_exec: bool = false
		if step_swing_phase_active_exec:
			step_swing_release_exec = step_cmd_release_front if step_swing_is_front else step_cmd_release_rear

		if step_swing_phase_active_exec and step_swing_release_exec and is_finite(step_swing_target_x) and step_swing_force_ready:
			var front_state_now: int = int(o._front_state)
			var rear_state_now: int = int(o._rear_state)

			var front_swing_exec: bool = (front_ctrl_mode == GCTypes.FootControlMode.SWING) and step_swing_is_front and (front_state_now == GCTypes.FootPlantState.SWING)
			var rear_swing_exec: bool = (rear_ctrl_mode == GCTypes.FootControlMode.SWING) and (not step_swing_is_front) and (rear_state_now == GCTypes.FootPlantState.SWING)

			if step_swing_is_front and not front_swing_exec:
				o._dbg_arb_swing_force_blocked += 1
			elif (not step_swing_is_front) and not rear_swing_exec:
				o._dbg_arb_swing_force_blocked += 1

				var exec_is_front: bool = front_swing_exec
				var exec_any: bool = front_swing_exec or rear_swing_exec

				var exec_grounded_now: bool = false
				if exec_any:
					exec_grounded_now = front_g if exec_is_front else rear_g

				var exec_contact_ref_now: bool = false
				if exec_any and o._foot_plant_mod != null:
					exec_contact_ref_now = o._foot_plant_mod.stepplanner_has_ground_contact_ref(exec_is_front)

				# Brief M+: once the swing foot touches ground or a valid contact reference, stop swing
				# translation immediately. From here the foot should be attitude-controlled, not fought by
				# extra below-ankle translation/unweight forces.
				if exec_any and (exec_grounded_now or exec_contact_ref_now):
					o._dbg_arb_swing_force_blocked += 1
					exec_any = false

				if exec_any:
					var swing_target_y_exec: float = step_swing_force_support_y

					# Keep swing clearance through the whole swing/candidate approach window (not only one legacy phase),
					# with a smooth arc so toes don't skim the floor on phase boundaries.
					var swing_nom_sec: float = maxf(0.05, float(o.phase7_stride_time_sec))
					var swing_t01: float = clampf(float(o._step_phase_t) / swing_nom_sec, 0.0, 1.0)
					var swing_lift_mul: float = sin(PI * swing_t01)
					swing_lift_mul = maxf(0.15, swing_lift_mul)
					swing_target_y_exec -= maxf(0.0, o.phase6_swing_lift_px) * swing_lift_mul

					var step_target_w_exec: Vector2 = Vector2(step_swing_target_x, swing_target_y_exec)
					o._ground_truth_mod.apply_swing_force(exec_is_front, step_target_w_exec, dt, spawn_gate)

				if exec_is_front:
					o._dbg_arb_swing_force_front = 1
				else:
					o._dbg_arb_swing_force_rear = 1

				# Brief M+: no swing-unweight direct foot force in STEP_PLANNER.
				# Active swing translation remains via apply_swing_force until touchdown/contact-ref.

	# Pre-landing airborne stance-shape enforcement (ownership pass, not gait tuning).
	# Goal: before touchdown (spawn/landing settle), shape both feet toward planner slots in X so the
	# legs approach ground in authored stance spread instead of collapsing and then fighting post-impact.
	#
	# Hard exclusions:
	# - only in STEP_PLANNER mode
	# - only when planner explicitly owns prelanding shape
	# - only when no planner swing sequence is active
	# - only while both feet are airborne
	# - X-shape only (target Y = current foot Y), so this does not co-author vertical motion
	# Brief M+: disable preland airborne X-shaping direct foot forces in STEP_PLANNER.
	# Preland shape intent remains in planner outputs/slots; foot bodies are not co-authored here.

	if o._foot_plant_mod != null:
		o._foot_plant_mod.update_plant_cache_midpoints()
		o._foot_plant_mod.apply_foot_damping_if_planted(front_g, rear_g)

	# 5R2-G + Brief A: Single foot attitude owner in STEP_PLANNER mode.
	# IMPORTANT:
	# - control ownership (swing/plant/recenter) and attitude ownership are separate channels
	# - feet must still have an attitude owner in preland/settle windows even if ctrl_mode is DISABLED
	# - gc_foot_plant remains the ONLY foot-angle torque writer in STEP_PLANNER mode
	if step_planner_authority:
		# Brief K: attitude mode must follow contact-truth (valid ground normal), not stale plantedish
		# ownership/executor hints. A foot may still be ctrl-owned as planted/recovery for a frame while
		# physically airborne; in that case it must use AIR_PARALLEL attitude behavior, not ground mode.
		var front_ground_ref_att: bool = front_g
		var rear_ground_ref_att: bool = rear_g
		if o._foot_plant_mod != null:
			front_ground_ref_att = o._foot_plant_mod.stepplanner_has_ground_contact_ref(true)
			rear_ground_ref_att = o._foot_plant_mod.stepplanner_has_ground_contact_ref(false)

		match front_ctrl_mode:
			GCTypes.FootControlMode.SWING:
				front_attitude_mode = GCTypes.FootAttitudeMode.AIR_PARALLEL
			GCTypes.FootControlMode.PLANTED_SUPPORT, GCTypes.FootControlMode.PLANTED_CORRECT, GCTypes.FootControlMode.RECOVERY:
				front_attitude_mode = GCTypes.FootAttitudeMode.GROUND_PARALLEL if front_ground_ref_att else GCTypes.FootAttitudeMode.AIR_PARALLEL
			_:
				front_attitude_mode = GCTypes.FootAttitudeMode.GROUND_PARALLEL if front_ground_ref_att else GCTypes.FootAttitudeMode.AIR_PARALLEL

		match rear_ctrl_mode:
			GCTypes.FootControlMode.SWING:
				rear_attitude_mode = GCTypes.FootAttitudeMode.AIR_PARALLEL
			GCTypes.FootControlMode.PLANTED_SUPPORT, GCTypes.FootControlMode.PLANTED_CORRECT, GCTypes.FootControlMode.RECOVERY:
				rear_attitude_mode = GCTypes.FootAttitudeMode.GROUND_PARALLEL if rear_ground_ref_att else GCTypes.FootAttitudeMode.AIR_PARALLEL
			_:
				rear_attitude_mode = GCTypes.FootAttitudeMode.GROUND_PARALLEL if rear_ground_ref_att else GCTypes.FootAttitudeMode.AIR_PARALLEL

		if o._rb_foot_front == null:
			front_attitude_mode = GCTypes.FootAttitudeMode.DISABLED
		if o._rb_foot_rear == null:
			rear_attitude_mode = GCTypes.FootAttitudeMode.DISABLED

		# Publish committed attitude ownership (pipeline-owned, non-debug).
		o._arb_front_attitude_mode = front_attitude_mode
		o._arb_rear_attitude_mode = rear_attitude_mode

		if o._foot_plant_mod != null:
			o._foot_plant_mod.apply_parallel_foot_attitude_by_mode(true, front_ctrl_mode, spawn_gate * muscle_gate, front_attitude_mode)
			o._foot_plant_mod.apply_parallel_foot_attitude_by_mode(false, rear_ctrl_mode, spawn_gate * muscle_gate, rear_attitude_mode)

		# Legacy Phase7 swing-foot attitude helper is intentionally blocked here.
		if o._locomotion_mod != null:
			o._dbg_arb_phase7_helper_blocked += 1
	else:
		var allow_planted_flatness_front: bool = true
		var allow_planted_flatness_rear: bool = true

		if o._foot_plant_mod != null and allow_planted_flatness_front:
			o._foot_plant_mod.apply_planted_foot_flatness(true, spawn_gate * muscle_gate)
		if o._foot_plant_mod != null and allow_planted_flatness_rear:
			o._foot_plant_mod.apply_planted_foot_flatness(false, spawn_gate * muscle_gate)

		if o._locomotion_mod != null:
			o._locomotion_mod.exec_apply_swing_attitude(spawn_gate, front_g, rear_g)

	# Foot rotation debug: current angle (deg) and angular velocity (rad/s) for both feet
	if o._rb_foot_front != null:
		o._dbg_foot_rot_deg_f = rad_to_deg(o._rb_foot_front.global_rotation)
		o._dbg_foot_rot_av_f = o._rb_foot_front.angular_velocity
	if o._rb_foot_rear != null:
		o._dbg_foot_rot_deg_r = rad_to_deg(o._rb_foot_rear.global_rotation)
		o._dbg_foot_rot_av_r = o._rb_foot_rear.angular_velocity

	# Body locomotion execution (body-only) happens AFTER foot execution/arbitration.
	# It must read foot state outcomes, not co-author them.
	if o._locomotion_mod != null:
		var pelvis_x_support_target_x: float = o._stance_center_x
		if step_planner_authority and bool(o._step_slot_center_valid) and is_finite(o._step_slot_center_x):
			# STEP_PLANNER cutover: body support and slot generation must follow the same chassis reference.
			pelvis_x_support_target_x = o._step_slot_center_x

		# Brief I: make root/chassis progression truly foot-led in STEP_PLANNER mode.
		# If command asks for motion but feet are not execution-ready (or settle lock is still active),
		# suppress pelvis x progression this frame instead of only counting it in debug.
		var pelvis_x_support_target_exec_x: float = pelvis_x_support_target_x
		var desired_vx_for_root: float = desired_vx

		if step_planner_authority and o._refs != null and o._refs.rb_pelvis != null and is_finite(pelvis_x_support_target_x):
			var pelvis_x_now: float = o._refs.rb_pelvis.global_position.x
			if is_finite(pelvis_x_now):
				var root_frame_drift_px: float = absf(pelvis_x_now - pelvis_x_support_target_x)
				var drift_trip_px: float = maxf(20.0, o.phase7_min_foot_separation_px)

				var front_st_now: int = int(o._front_state)
				var rear_st_now: int = int(o._rear_state)
				var front_exec_plantedish: bool = (
					front_st_now == GCTypes.FootPlantState.CANDIDATE
					or front_st_now == GCTypes.FootPlantState.PLANTED
					or bool(o._plant_front_active)
				)
				var rear_exec_plantedish: bool = (
					rear_st_now == GCTypes.FootPlantState.CANDIDATE
					or rear_st_now == GCTypes.FootPlantState.PLANTED
					or bool(o._plant_rear_active)
				)
				var dual_support_contact_now: bool = front_g and rear_g
				var dual_support_exec_ready_now: bool = dual_support_contact_now and front_exec_plantedish and rear_exec_plantedish

				var root_progress_hard_block: bool = false
				var idle_correction_root_hold: bool = bool(o.get("_step_idle_step_correction")) and (absf(cmd) <= 0.25)

				if (absf(cmd) > 0.25 or idle_correction_root_hold) and not bool(o._step_preland_shape_active):
					# Brief M+: during no-input corrective stance enforcement, freeze pelvis progression/support target.
					# Feet must resolve slot error; body does not substitute.
					if idle_correction_root_hold:
						root_progress_hard_block = true
					elif not step_has_swing:
						# Do not let pelvis "solve" movement while feet are still not executable supports.
						if dual_support_contact_now and not dual_support_exec_ready_now:
							root_progress_hard_block = true
						# Brief K: if contact has fractured to accidental single support and planner has not yet
						# armed a swing, do not let pelvis progression "solve" the step.
						elif (front_g and not rear_g) or (rear_g and not front_g):
							root_progress_hard_block = true
						# Brief L: if planner says stance needs a corrective step, do not let root progression
						# "solve" it by dragging planted feet.
						elif bool(o.get("_step_idle_step_correction")) and dual_support_exec_ready_now:
							root_progress_hard_block = true
						# During settle lock, hold chassis until planner arms the first real step/correction.
						elif bool(o._step_settle_lock_active) and dual_support_contact_now:
							root_progress_hard_block = true
						# If pelvis has drifted far from planner slot center with no swing active, freeze root and
						# force planner/executor to reestablish foot-led progression first.
						elif root_frame_drift_px > drift_trip_px:
							root_progress_hard_block = true

				if root_progress_hard_block:
					o._dbg_root_progress_blocked_by_foot += 1
					desired_vx_for_root = 0.0
					pelvis_x_support_target_exec_x = pelvis_x_now

		o._locomotion_mod.apply_pelvis_x_support(pelvis_x_support_target_exec_x, desired_vx_for_root, dt, spawn_gate)

	if o._recovery_mod != null:
		o._recovery_mod.update_rest_recapture_gate(front_g, rear_g, stabF, stabR, dt)

	# 5R2-J3: Persist step/phase for next-frame latch clearing.
	if step_outputs_valid:
		o._prev_step_id = o._step_id
		o._prev_step_phase = step_phase

	# Visual follow (allowed; not physics)
	if o._body_root != null and o._rb_pelvis != null:
		o._body_root.global_position = o._rb_pelvis.global_position
		o._body_root.global_rotation = o._rb_pelvis.global_rotation

	if o.debug_draw:
		o.queue_redraw()

	if o.debug_enable:
		o._dbg_accum += dt
		var hz: float = maxf(1.0, o.debug_print_hz)
		if o._dbg_accum >= (1.0 / hz):
			o._dbg_accum = 0.0
			if o._debug_mod != null:
				o._debug_mod.print_debug(front_g, rear_g, stabF, stabR, grounded, run01)
