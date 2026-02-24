# res://scripts/duel/gladiator_controller/gc_step_planner.gd
# 5R2-B: deterministic slot-step planner (shadow mode only).
# Writes _step_* outputs for debug and future authority cutover.
# Does NOT apply forces, pins, friction, or root motion.

class_name GCStepPlanner
extends RefCounted

var _owner: Node = null
var _refs: GCRefs = null

# 5R2-D shadow planner runtime (internal only; not authoritative cutover yet).
const _SEQ_STAGE_NONE := 0
const _SEQ_STAGE_PRIMARY := 1
const _SEQ_STAGE_CORRECTIVE := 2

var _sh_hold_dir_world: int = 0
var _sh_hold_t: float = 0.0
var _sh_prev_dir_world: int = 0

var _sh_seq_active: bool = false
var _sh_seq_dir_world: int = 0
var _sh_seq_hold_mode: bool = false
var _sh_seq_stage: int = _SEQ_STAGE_NONE
var _sh_seq_primary_swing_is_front: bool = true
var _sh_seq_swing_is_front: bool = true
var _sh_seq_stage_t: float = 0.0
var _sh_seq_seen_airborne: bool = false
# Brief 2: planner-side touchdown latch.
# Once a swing foot has valid touchdown contact/stability for the current sequence,
# keep that fact latched until sequence advance/finish so command outputs don't flap.
var _sh_seq_landed_contact: bool = false
var _sh_seq_target_x: float = NAN

# Brief M+: capture tap intent at sequence start so target shaping remains stable after release.
var _sh_seq_cmd_abs_begin: float = 0.0
var _sh_seq_press_t_begin: float = 0.0

var _sh_recover_active: bool = false
var _sh_recover_t: float = 0.0
var _sh_recover_reason: int = GCTypes.StepPlannerRecoverReason.NONE
var _sh_timeout_active: bool = false

# Brief 1: separate "support-loss recover" from "ground settle / step arming".
# Recover marks no-support episodes. A separate settle gate decides when new
# sequences (including idle correction) may arm after spawn/landing.
var _sh_settle_lock_active: bool = true
var _sh_dual_support_stable_t: float = 0.0

# Brief I: one-shot spawn/preland role bootstrap from live foot pose.
# We still lock the neutral axis by facing, but we no longer hardcode lead role every frame.
var _sh_spawn_pose_role_bootstrap_done: bool = false


func setup(owner: Node, refs: GCRefs) -> void:
	_owner = owner
	_refs = refs


func reset() -> void:
	var o := _owner
	if o == null:
		return
	o.set("_step_phase", GCTypes.StepPlannerPhase.DISABLED)
	o.set("_step_phase_t", 0.0)
	o.set("_step_id", 0)
	o.set("_step_support_is_front", true)
	o.set("_step_has_swing", false)
	o.set("_step_swing_is_front", true)
	o.set("_step_front_slot_x", NAN)
	o.set("_step_rear_slot_x", NAN)
	o.set("_step_swing_target_x", NAN)
	o.set("_step_cmd_plant_front", false)
	o.set("_step_cmd_plant_rear", false)
	o.set("_step_cmd_release_front", false)
	o.set("_step_cmd_release_rear", false)
	o.set("_step_timeout_active", false)
	o.set("_step_recover_reason", GCTypes.StepPlannerRecoverReason.NONE)
	# Brief 3: deterministic planner bootstrap at reset/spawn.
	# Start in planner-owned preland/ref-freeze immediately so no downstream module
	# can initialize lead/axis from live airborne foot ordering before planner slots publish.
	o.set("_step_ref_freeze_active", true)
	o.set("_step_preland_shape_active", true)
	o.set("_step_idle_inplace_correction", false)
	o.set("_step_idle_step_correction", false)
	o.set("_step_settle_lock_active", true)

	# Brief I: lock neutral axis by facing, but do NOT hardcode lead role.
	# Lead role gets a one-shot bootstrap from spawn foot pose (if available), then stays stable.
	_seed_neutral_axis_from_facing()
	o.set("_lead_is_front", true) # deterministic fallback only; may be replaced by pose bootstrap below
	o.set("_lead_initialized", true)
	_sh_spawn_pose_role_bootstrap_done = false
	_try_bootstrap_lead_role_from_pose()

	_sh_hold_dir_world = 0
	_sh_hold_t = 0.0
	_sh_prev_dir_world = 0

	_sh_seq_active = false
	_sh_seq_dir_world = 0
	_sh_seq_hold_mode = false
	_sh_seq_stage = _SEQ_STAGE_NONE
	_sh_seq_primary_swing_is_front = true
	_sh_seq_swing_is_front = true
	_sh_seq_stage_t = 0.0
	_sh_seq_seen_airborne = false
	_sh_seq_landed_contact = false
	_sh_seq_target_x = NAN
	_sh_seq_cmd_abs_begin = 0.0
	_sh_seq_press_t_begin = 0.0

	_sh_recover_active = false
	_sh_recover_t = 0.0
	_sh_recover_reason = GCTypes.StepPlannerRecoverReason.NONE
	_sh_timeout_active = false

	# Brief 1 settle-gate reset:
	# start locked so spawn/first landing cannot immediately arm step sequences.
	_sh_settle_lock_active = true
	_sh_dual_support_stable_t = 0.0
	_sh_spawn_pose_role_bootstrap_done = false


func _seed_neutral_axis_from_facing() -> float:
	var o := _owner
	if o == null:
		return 1.0

	var fs_seed: float = 0.0
	if o.has_method("get_facing_sign_x"):
		fs_seed = signf(float(o.get_facing_sign_x()))
	if absf(fs_seed) < 0.5:
		var fs_prop = o.get("facing_sign")
		if typeof(fs_prop) == TYPE_FLOAT or typeof(fs_prop) == TYPE_INT:
			fs_seed = signf(float(fs_prop))
	if absf(fs_seed) < 0.5:
		fs_seed = 1.0

	var axis: float = 1.0 if fs_seed >= 0.0 else -1.0
	o._neutral_axis_sign = axis
	return axis


func _try_bootstrap_lead_role_from_pose() -> bool:
	var o := _owner
	if o == null:
		return false
	if _sh_spawn_pose_role_bootstrap_done:
		return true

	var fx: float = _foot_x(true)
	var rx: float = _foot_x(false)
	if not is_finite(fx) or not is_finite(rx):
		return false

	# Need a meaningful separation to infer role from pose.
	if absf(fx - rx) < 0.5:
		return false

	var axis: float = float(o._neutral_axis_sign)
	if absf(axis) < 0.5:
		axis = _seed_neutral_axis_from_facing()

	# "Lead" = foot that is ahead along the planner neutral axis.
	var front_is_ahead: bool = ((fx - rx) * axis) > 0.0
	o.set("_lead_is_front", front_is_ahead)
	o.set("_lead_initialized", true)
	_sh_spawn_pose_role_bootstrap_done = true
	return true


func tick(_dt: float) -> void:
	# Intentionally unused; controller calls tick_shadow(...) explicitly.
	pass


func tick_shadow(
	cmd_move_x_norm: float,
	dt: float,
	front_g: bool,
	rear_g: bool,
	grounded_eff: bool,
	stabF: float,
	stabR: float
) -> void:
	var o := _owner
	if o == null:
		return

	var old_phase: int = int(o.get("_step_phase"))
	var old_support_is_front: bool = bool(o.get("_step_support_is_front"))
	var old_has_swing: bool = bool(o.get("_step_has_swing"))
	var old_swing_is_front: bool = bool(o.get("_step_swing_is_front"))

	# Brief I: freeze/preland locks axis by facing, but does NOT overwrite lead role every frame.
	# We allow a one-shot pose bootstrap during airborne spawn/preland once foot refs are valid.
	if bool(o.get("_step_ref_freeze_active")) or bool(o.get("_step_preland_shape_active")):
		_seed_neutral_axis_from_facing()

		if not bool(o.get("_lead_initialized")):
			o.set("_lead_is_front", true)
			o.set("_lead_initialized", true)

		if (not _sh_spawn_pose_role_bootstrap_done) and (not _sh_seq_active) and (not front_g) and (not rear_g):
			_try_bootstrap_lead_role_from_pose()

	# Always-on slot invariant: planner always computes valid slots (or deterministic fallback).
	var slots: Vector2 = _compute_slot_targets_x()
	var front_slot_x: float = slots.x
	var rear_slot_x: float = slots.y

	var fx: float = _foot_x(true)
	var rx: float = _foot_x(false)

	# Brief H: planner gates must be able to use executor truth (foot FSM/pin state),
	# not only floating "stability" values.
	var front_state_now: int = int(o.get("_front_state"))
	var rear_state_now: int = int(o.get("_rear_state"))
	var front_exec_plantedish: bool = bool(o.get("_plant_front_active")) or _foot_state_plantedish(front_state_now)
	var rear_exec_plantedish: bool = bool(o.get("_plant_rear_active")) or _foot_state_plantedish(rear_state_now)

	var dir_world: int = 0
	var in_thr: float = float(o.get("input_meaningful_threshold"))
	if absf(cmd_move_x_norm) >= in_thr:
		dir_world = 1 if cmd_move_x_norm > 0.0 else -1

	_update_hold_tracker(dir_world, maxf(0.0, dt))
	var hold_to_walk_sec: float = maxf(0.0, float(o.get("phase7_hold_to_walk_sec")))
	var hold_mode_now: bool = (dir_world != 0) and (_sh_hold_dir_world == dir_world) and (_sh_hold_t >= hold_to_walk_sec)

	var step_done_pos_eps: float = maxf(2.0, float(o.get("phase7_step_done_pos_eps")))
	var slot_sep_now: float = absf(front_slot_x - rear_slot_x) if is_finite(front_slot_x) and is_finite(rear_slot_x) else maxf(0.0, float(o.get("phase7_min_foot_separation_px")))
	var correction_eps: float = maxf(step_done_pos_eps, 3.0)
	# Stance enforcement: threshold scales with slot separation so crouch/tall behaves
	# consistently, but keep it tight so idle feet reliably land into authored slots.
	correction_eps = maxf(correction_eps, clampf(slot_sep_now * 0.12, 2.0, 18.0))

	var front_err: float = absf(fx - front_slot_x) if is_finite(fx) and is_finite(front_slot_x) else 0.0
	var rear_err: float = absf(rx - rear_slot_x) if is_finite(rx) and is_finite(rear_slot_x) else 0.0
	var max_slot_err: float = maxf(front_err, rear_err)

	# Brief M+: no-input stance enforcement is always a lift→land corrective step in STEP_PLANNER.
	# Do NOT use in-place planted recenter as locomotion authority.
	#
	# Keep correction_step_arm_eps for other heuristics (fracture/stall), but idle no-input correction
	# now arms on the base correction threshold.
	var correction_step_arm_eps: float = maxf(correction_eps * 1.4, correction_eps + 4.0)
	correction_step_arm_eps = maxf(correction_step_arm_eps, clampf(slot_sep_now * 0.25, 4.0, 36.0))

	var idle_any_correction_needed: bool = grounded_eff and (dir_world == 0) and (max_slot_err > correction_eps)
	var idle_step_correction_needed: bool = idle_any_correction_needed
	var idle_inplace_correction_needed: bool = false

	# Brief 1: dual-support settle gate (planner-side sequencing ownership).
	# Prevent new step/corrective sequence arming during spawn/landing contact chaos.
	var settle_stab_min: float = 0.45
	if o.has_method("get"):
		var pms_settle = o.get("plant_min_stability")
		if typeof(pms_settle) == TYPE_FLOAT or typeof(pms_settle) == TYPE_INT:
			settle_stab_min = clampf(float(pms_settle) * 0.85, 0.25, 0.90)

	var settle_dual_support_min_sec: float = 0.06
	var dual_support_contact_ready: bool = front_g and rear_g
	# Brief H: "stable" floats can be zero/noisy in some valid planted windows.
	# Use executor truth as an alternate readiness path so settle lock cannot deadlock
	# sequence arming after landing/collapse recovery.
	var dual_support_exec_ready: bool = dual_support_contact_ready and front_exec_plantedish and rear_exec_plantedish
	var dual_support_stable_ready: bool = dual_support_contact_ready and (stabF >= settle_stab_min) and (stabR >= settle_stab_min)
	var dual_support_settle_ready: bool = dual_support_stable_ready or dual_support_exec_ready

	# Any true no-support episode relocks settle.
	if (not grounded_eff) and (not front_g) and (not rear_g):
		_sh_settle_lock_active = true
		_sh_dual_support_stable_t = 0.0

	# Track stable dual-support dwell time.
	if dual_support_settle_ready:
		_sh_dual_support_stable_t += maxf(0.0, dt)
	else:
		_sh_dual_support_stable_t = 0.0

	# Unlock sequence arming only after a short stable dual-support window.
	if _sh_settle_lock_active and (_sh_dual_support_stable_t >= settle_dual_support_min_sec):
		_sh_settle_lock_active = false

	# Idle correction policy after settle unlock:
	# - in-place correction may be active in DOUBLE_SUPPORT after settle unlock
	# - step correction may arm a corrective sequence only after settle unlock
	if idle_any_correction_needed and dual_support_exec_ready:
		idle_step_correction_needed = idle_step_correction_needed and (not _sh_settle_lock_active)
		# Brief M+: keep no-input stance enforcement on the corrective-step path only.
		idle_inplace_correction_needed = false
	else:
		idle_step_correction_needed = false
		idle_inplace_correction_needed = false

	# New sequence arming gate (primary move step + large idle corrective step).
	# Brief H: prevent "feet glued together while holding move" deadlock.
	# If both feet are executor-ready in dual support and slot error is large, movement input may
	# override settle lock and arm a corrective/primary sequence instead of recenter dragging.
	var move_stall_override: bool = (dir_world != 0) and dual_support_exec_ready and (max_slot_err > correction_step_arm_eps)
	var planner_can_start_sequence: bool = (not _sh_seq_active) and (not _sh_recover_active) and dual_support_exec_ready and ((not _sh_settle_lock_active) or move_stall_override)

	# Recover state (shadow semantics only)
	if _sh_recover_active:
		_sh_recover_t += maxf(0.0, dt)

		# Brief 1: RECOVER marks support loss only.
		# As soon as support returns, exit RECOVER and let the settle gate control
		# when sequences may re-arm.
		var support_returned: bool = grounded_eff or front_g or rear_g
		if support_returned:
			_sh_recover_active = false
			_sh_recover_t = 0.0
			_sh_timeout_active = false
			_sh_recover_reason = GCTypes.StepPlannerRecoverReason.NONE

	# Support loss enters recover (planner-level marker only).
	# Settle re-arm is handled separately and is explicitly relocked here.
	if not _sh_recover_active and not grounded_eff and not front_g and not rear_g:
		_sh_settle_lock_active = true
		_sh_dual_support_stable_t = 0.0
		_enter_recover(GCTypes.StepPlannerRecoverReason.SUPPORT_LOSS)

	# Start sequences only after settle unlock.
	# Prevent spawn/landing contact chaos from immediately arming steps.
	if not _sh_recover_active and not _sh_seq_active and planner_can_start_sequence:
		if dir_world != 0:
			var primary_swing_is_front: bool = _pick_initiator_for_dir(dir_world)
			_begin_sequence(dir_world, primary_swing_is_front, _SEQ_STAGE_PRIMARY, false, absf(cmd_move_x_norm))
		elif idle_step_correction_needed:
			var corr_swing_is_front: bool = _pick_idle_correction_swing(front_err, rear_err, front_g, rear_g)
			_begin_sequence(0, corr_swing_is_front, _SEQ_STAGE_CORRECTIVE, false, 0.0)

	# Brief K: support-fracture takeover.
	# If we are not in RECOVER/SEQUENCE and contact fractures into an accidental single-support state,
	# do not remain in "double support on paper" while pelvis motion drags the airborne foot. Promote
	# the missing-contact foot into a planner-owned sequence, overriding settle-lock if needed.
	if not _sh_recover_active and not _sh_seq_active and grounded_eff:
		var accidental_single_support: bool = (front_g and not rear_g) or (rear_g and not front_g)
		if accidental_single_support:
			var support_is_front_fracture: bool = front_g and not rear_g
			var support_exec_ready_fracture: bool = front_exec_plantedish if support_is_front_fracture else rear_exec_plantedish
			var fracture_needs_action: bool = (dir_world != 0) or (max_slot_err > correction_step_arm_eps) or idle_any_correction_needed

			if support_exec_ready_fracture and fracture_needs_action:
				_sh_settle_lock_active = false
				_sh_dual_support_stable_t = 0.0

				var fracture_swing_is_front: bool = not support_is_front_fracture
				var fracture_stage: int = _SEQ_STAGE_PRIMARY if dir_world != 0 else _SEQ_STAGE_CORRECTIVE
				_begin_sequence(dir_world, fracture_swing_is_front, fracture_stage, hold_mode_now and (dir_world != 0), absf(cmd_move_x_norm))

	# Upgrade active sequence to hold mode if input became held in same direction.
	if _sh_seq_active and dir_world != 0 and _sh_seq_dir_world == dir_world and hold_mode_now:
		_sh_seq_hold_mode = true

	# Advance active sequence (shadow planner semantics only)
	if _sh_seq_active and not _sh_recover_active:
		_sh_seq_stage_t += maxf(0.0, dt)

		var swing_is_front_now: bool = _sh_seq_swing_is_front
		var support_is_front_now: bool = not swing_is_front_now
		var support_x: float = front_slot_x if support_is_front_now else rear_slot_x

		var stage_target_x_now: float = _compute_stage_target_x(
			_sh_seq_stage,
			_sh_seq_dir_world,
			_sh_seq_hold_mode,
			swing_is_front_now,
			front_slot_x,
			rear_slot_x,
			support_x,
			cmd_move_x_norm
		)

		# Brief M+: feet must lead body. Freeze target for the current stage once armed.
		# Exception: PRIMARY may extend during same-direction hold, but must never shrink.
		if not is_finite(_sh_seq_target_x):
			_sh_seq_target_x = stage_target_x_now
		elif _sh_seq_stage == _SEQ_STAGE_PRIMARY and (_sh_seq_hold_mode or hold_mode_now) and is_finite(stage_target_x_now):
			if _sh_seq_dir_world > 0:
				_sh_seq_target_x = maxf(_sh_seq_target_x, stage_target_x_now)
			elif _sh_seq_dir_world < 0:
				_sh_seq_target_x = minf(_sh_seq_target_x, stage_target_x_now)
			# dir==0 should not happen for PRIMARY; keep frozen target if it does.
		# else: keep frozen target for corrective / post-release stability

		var swing_g: bool = front_g if swing_is_front_now else rear_g
		var swing_stab: float = stabF if swing_is_front_now else stabR
		if not swing_g:
			_sh_seq_seen_airborne = true

		var landing_min_sec: float = maxf(0.0, float(o.get("phase7_lift_min_swing_sec")))
		var landing_timeout_sec: float = maxf(landing_min_sec + 0.01, float(o.get("phase7_swing_fail_max_sec")))
		var min_stab_for_land: float = 0.30
		if o.has_method("get"):
			# If plant_min_stability exists, use a softened threshold for shadow planner acceptance.
			var pms = o.get("plant_min_stability")
			if typeof(pms) == TYPE_FLOAT or typeof(pms) == TYPE_INT:
				min_stab_for_land = clampf(float(pms) * 0.5, 0.20, 0.80)

		var swing_x: float = fx if swing_is_front_now else rx
		var close_enough: bool = is_finite(swing_x) and is_finite(_sh_seq_target_x) and absf(swing_x - _sh_seq_target_x) <= maxf(step_done_pos_eps, 2.0)

		# Brief 2: separate touchdown confirmation from sequence completion.
		# This lets the planner stop issuing RELEASE for the swing foot as soon as
		# touchdown is valid, even if positional settle/advance happens a bit later.
		var touchdown_valid_now: bool = _sh_seq_seen_airborne and swing_g and (_sh_seq_stage_t >= landing_min_sec) and (swing_stab >= min_stab_for_land)
		if touchdown_valid_now:
			_sh_seq_landed_contact = true

		var stage_done: bool = false
		if _sh_seq_stage == _SEQ_STAGE_CORRECTIVE:
			# Brief L: corrective stage is stance enforcement. Time alone does not mean "done".
			stage_done = _sh_seq_landed_contact and close_enough
		else:
			stage_done = _sh_seq_landed_contact and (_sh_seq_stage_t >= maxf(landing_min_sec, float(o.get("phase7_stride_time_sec")) * 0.35) or close_enough)

		var timed_out: bool = _sh_seq_stage_t >= landing_timeout_sec

		if timed_out:
			if _sh_seq_landed_contact:
				# Touchdown-underperform: foot touched down but didn't reach target.
				# Always advance/finish so the stop sequence can still slot feet
				# into stance width. Never stall in a timed-out stage.
				_advance_or_finish_sequence(dir_world, hold_mode_now)
			elif close_enough and swing_g:
				# Foot arrived at target and is grounded but stability hasn't met
				# the latch threshold yet. Treat as a soft landing and advance.
				_sh_seq_landed_contact = true
				_advance_or_finish_sequence(dir_world, hold_mode_now)
			else:
				# True swing failure = no touchdown by timeout.
				_enter_recover(GCTypes.StepPlannerRecoverReason.LANDING_TIMEOUT)
		elif stage_done:
			_advance_or_finish_sequence(dir_world, hold_mode_now)

	# Output mapping (still shadow-only, but now meaningful)
	var phase: int = GCTypes.StepPlannerPhase.DOUBLE_SUPPORT
	var support_is_front: bool = _support_is_front_from_contacts(front_g, rear_g, stabF, stabR)
	var has_swing: bool = false
	var swing_is_front: bool = bool(o.get("_step_swing_is_front"))
	var swing_target_x: float = NAN
	var recover_reason: int = GCTypes.StepPlannerRecoverReason.NONE

	if _sh_recover_active:
		phase = GCTypes.StepPlannerPhase.RECOVER
		has_swing = false
		recover_reason = _sh_recover_reason
	elif _sh_seq_active:
		has_swing = true
		swing_is_front = _sh_seq_swing_is_front
		support_is_front = not swing_is_front
		swing_target_x = _sh_seq_target_x
		phase = GCTypes.StepPlannerPhase.SINGLE_SUPPORT_SWING if _sh_seq_hold_mode else GCTypes.StepPlannerPhase.TRANSFER
	else:
		phase = GCTypes.StepPlannerPhase.DOUBLE_SUPPORT
		has_swing = false
		# In-place correction path (no sequence armed): bias support toward the lower-error side
		# so the other foot is preferred for correction ownership if arbitration upgrades one foot.
		if idle_inplace_correction_needed:
			support_is_front = rear_err <= front_err

	var cmd_plant_front: bool = false
	var cmd_plant_rear: bool = false
	var cmd_release_front: bool = false
	var cmd_release_rear: bool = false

	match phase:
		GCTypes.StepPlannerPhase.DOUBLE_SUPPORT:
			# Both feet owned as planted/correctable; no release command.
			cmd_plant_front = true
			cmd_plant_rear = true
			cmd_release_front = false
			cmd_release_rear = false

		GCTypes.StepPlannerPhase.SINGLE_SUPPORT_SWING, GCTypes.StepPlannerPhase.TRANSFER:
			# Support foot is always plant-owned.
			# Swing foot is RELEASE-owned only until planner touchdown is confirmed;
			# after touchdown latch, planner hands it back to PLANT ownership even if
			# the sequence remains active for positional settle/advance bookkeeping.
			if support_is_front:
				cmd_plant_front = true
				cmd_release_front = false

				cmd_plant_rear = _sh_seq_landed_contact
				cmd_release_rear = not _sh_seq_landed_contact
			else:
				cmd_plant_rear = true
				cmd_release_rear = false

				cmd_plant_front = _sh_seq_landed_contact
				cmd_release_front = not _sh_seq_landed_contact

		GCTypes.StepPlannerPhase.RECOVER:
			# Recover is not a blanket "release both feet" command.
			# Pipeline arbitration/recovery owns RECOVERY/DISABLED decisions.
			cmd_plant_front = false
			cmd_plant_rear = false
			cmd_release_front = false
			cmd_release_rear = false

		_:
			cmd_plant_front = false
			cmd_plant_rear = false
			cmd_release_front = false
			cmd_release_rear = false

	var signature_changed: bool = (
		phase != old_phase
		or support_is_front != old_support_is_front
		or has_swing != old_has_swing
		or (has_swing and swing_is_front != old_swing_is_front)
	)

	if signature_changed:
		o.set("_step_id", int(o.get("_step_id")) + 1)
		o.set("_step_phase_t", 0.0)
	else:
		var t_prev: float = o.get("_step_phase_t") as float
		o.set("_step_phase_t", maxf(0.0, t_prev) + maxf(0.0, dt))

	o.set("_step_phase", phase)
	o.set("_step_support_is_front", support_is_front)
	o.set("_step_has_swing", has_swing)
	o.set("_step_swing_is_front", swing_is_front)
	o.set("_step_front_slot_x", front_slot_x)
	o.set("_step_rear_slot_x", rear_slot_x)
	o.set("_step_swing_target_x", swing_target_x)
	o.set("_step_cmd_plant_front", cmd_plant_front)
	o.set("_step_cmd_plant_rear", cmd_plant_rear)
	o.set("_step_cmd_release_front", cmd_release_front)
	o.set("_step_cmd_release_rear", cmd_release_rear)
	o.set("_step_timeout_active", _sh_timeout_active)
	o.set("_step_recover_reason", recover_reason)

	# Planner ownership flags for cross-module stability (locomotion/pipeline consumers).
	# preland shape: explicit airborne stance-shape owner before touchdown during settle lock
	var preland_shape_active: bool = _sh_settle_lock_active and (not _sh_seq_active) and (not front_g) and (not rear_g)

	# reference-frame freeze:
	# freeze lead/trail + stance-center re-framing during settle/prelanding and idle in-place correction,
	# so downstream modules do not churn the frame while planner/arbitration are trying to re-shape.
	var ref_freeze_active: bool = false
	if preland_shape_active:
		ref_freeze_active = true
	elif _sh_settle_lock_active:
		ref_freeze_active = true
	elif (dir_world == 0) and (not _sh_seq_active) and idle_inplace_correction_needed:
		ref_freeze_active = true

	o.set("_step_preland_shape_active", preland_shape_active)
	o.set("_step_ref_freeze_active", ref_freeze_active)
	o.set("_step_idle_inplace_correction", idle_inplace_correction_needed and (not _sh_seq_active))
	o.set("_step_idle_step_correction", idle_step_correction_needed)
	o.set("_step_settle_lock_active", _sh_settle_lock_active)

	_sh_prev_dir_world = dir_world


func _update_hold_tracker(dir_world: int, dt: float) -> void:
	if dir_world == 0:
		_sh_hold_dir_world = 0
		_sh_hold_t = 0.0
		return
	if dir_world != _sh_hold_dir_world:
		_sh_hold_dir_world = dir_world
		_sh_hold_t = 0.0
	else:
		_sh_hold_t += maxf(0.0, dt)


func _begin_sequence(dir_world: int, swing_is_front: bool, stage: int, hold_mode: bool, cmd_abs_input: float = 1.0) -> void:
	_sh_seq_active = true
	_sh_seq_dir_world = dir_world
	_sh_seq_hold_mode = hold_mode
	_sh_seq_stage = stage
	_sh_seq_primary_swing_is_front = swing_is_front if stage == _SEQ_STAGE_PRIMARY else _sh_seq_primary_swing_is_front
	_sh_seq_swing_is_front = swing_is_front
	_sh_seq_stage_t = 0.0
	_sh_seq_seen_airborne = false
	_sh_seq_landed_contact = false
	_sh_seq_target_x = NAN
	_sh_seq_cmd_abs_begin = clampf(absf(cmd_abs_input), 0.0, 1.0)
	_sh_seq_press_t_begin = maxf(0.0, _sh_hold_t)
	_sh_timeout_active = false


func _advance_or_finish_sequence(current_dir_world: int, hold_mode_now: bool) -> void:
	var o := _owner
	var stop_correct_enable: bool = bool(o.get("phase7_stop_correct_enable"))
	var step_planner_authority: bool = (int(o.get("movement_authority_mode")) == GCTypes.MovementAuthorityMode.STEP_PLANNER)

	if _sh_seq_stage == _SEQ_STAGE_PRIMARY:
		# Brief M+: in STEP_PLANNER, PRIMARY is never the whole story.
		# Always follow with CORRECTIVE so release/stop re-slots authored stance width.
		var primary_must_correct: bool = stop_correct_enable or step_planner_authority
		if primary_must_correct:
			_begin_sequence(_sh_seq_dir_world, not _sh_seq_swing_is_front, _SEQ_STAGE_CORRECTIVE, _sh_seq_hold_mode or hold_mode_now)
			return
		_finish_sequence()
		return

	if _sh_seq_stage == _SEQ_STAGE_CORRECTIVE:
		# Hold path repeats; tap path ends after corrective step.
		var keep_holding_same_dir: bool = (current_dir_world != 0) and (current_dir_world == _sh_seq_dir_world) and (_sh_seq_hold_mode or hold_mode_now)
		if keep_holding_same_dir:
			_begin_sequence(_sh_seq_dir_world, not _sh_seq_primary_swing_is_front, _SEQ_STAGE_PRIMARY, true)
			return
		_finish_sequence()
		return

	_finish_sequence()


func _finish_sequence() -> void:
	_sh_seq_active = false
	_sh_seq_dir_world = 0
	_sh_seq_hold_mode = false
	_sh_seq_stage = _SEQ_STAGE_NONE
	_sh_seq_stage_t = 0.0
	_sh_seq_seen_airborne = false
	_sh_seq_landed_contact = false
	_sh_seq_target_x = NAN
	_sh_seq_cmd_abs_begin = 0.0
	_sh_seq_press_t_begin = 0.0


func _enter_recover(reason: int) -> void:
	_sh_recover_active = true
	_sh_recover_t = 0.0
	_sh_recover_reason = reason
	_sh_timeout_active = (reason == GCTypes.StepPlannerRecoverReason.LANDING_TIMEOUT)

	# Brief 1 defense-in-depth:
	# any recover entry requires a fresh dual-support stabilization window before
	# the planner may arm another sequence.
	_sh_settle_lock_active = true
	_sh_dual_support_stable_t = 0.0

	_finish_sequence()


func _support_is_front_from_contacts(front_g: bool, rear_g: bool, stabF: float, stabR: float) -> bool:
	var prev: bool = bool(_owner.get("_step_support_is_front"))
	if front_g and not rear_g:
		return true
	if rear_g and not front_g:
		return false
	if front_g and rear_g:
		if absf(stabF - stabR) > 0.001:
			return stabF >= stabR
	return prev


func _foot_state_plantedish(st: int) -> bool:
	return st == GCTypes.FootPlantState.CANDIDATE or st == GCTypes.FootPlantState.PLANTED

func _pick_idle_correction_swing(front_err: float, rear_err: float, front_g: bool, rear_g: bool) -> bool:
	# Swing the foot that is farther from its slot, but prefer keeping a grounded support foot.
	if front_g and not rear_g:
		return false # rear swings
	if rear_g and not front_g:
		return true # front swings
	if absf(front_err - rear_err) <= 0.5:
		# deterministic tie-breaker: move trail first to restore stance shape
		var lead_is_front: bool = bool(_owner.get("_lead_is_front"))
		return lead_is_front
	return front_err >= rear_err


func _foot_x(is_front: bool) -> float:
	if _refs == null:
		return NAN
	var rb: RigidBody2D = _refs.rb_foot_front if is_front else _refs.rb_foot_rear
	return rb.global_position.x if rb != null else NAN


func _compute_stage_target_x(
	stage: int,
	dir_world: int,
	hold_mode: bool,
	swing_is_front: bool,
	front_slot_x: float,
	rear_slot_x: float,
	support_x: float,
	cmd_move_x_norm: float
) -> float:
	var o := _owner
	var slot_x: float = front_slot_x if swing_is_front else rear_slot_x
	if stage == _SEQ_STAGE_CORRECTIVE:
		return slot_x

	# Primary stage: under-body/cross step (tap) and larger stride progression (hold).
	# Prefer planner-owned slot center so stage targets stop chasing live pelvis drift.
	var cx: float = NAN
	if bool(o.get("_step_slot_center_valid")) and is_finite(float(o.get("_step_slot_center_x"))):
		cx = float(o.get("_step_slot_center_x"))
	if not is_finite(cx):
		cx = float(o.get("_stance_center_x"))
	if not is_finite(cx):
		cx = _refs.rb_pelvis.global_position.x if _refs != null and _refs.rb_pelvis != null else 0.0

	var under_body_px: float = maxf(0.0, float(o.get("phase7_under_body_step_px")))
	var stride_t: float = maxf(0.0, float(o.get("phase7_stride_time_sec")))
	var speed_px_s: float = maxf(0.0, float(o.get("phase2_speed_px_s")))
	var run_ramp_sec: float = maxf(0.001, float(o.get("phase7_run_ramp_sec")))
	var run_stride_mult: float = maxf(1.0, float(o.get("phase7_run_stride_px_mult")))
	var cross_allow_px: float = maxf(0.0, float(o.get("phase7_cross_allow_px")))
	var max_reach_px: float = maxf(1.0, float(o.get("phase7_max_foot_reach_px")))

	var run01: float = 0.0
	if hold_mode and _sh_hold_dir_world != 0:
		run01 = clampf((_sh_hold_t - maxf(0.0, float(o.get("phase7_hold_to_walk_sec")))) / run_ramp_sec, 0.0, 1.0)

	# Preserve tap intent after release using sequence-captured input.
	var cmd01_live: float = clampf(absf(cmd_move_x_norm), 0.0, 1.0)
	var cmd01_seq: float = clampf(_sh_seq_cmd_abs_begin, 0.0, 1.0)
	var cmd01: float = cmd01_live if hold_mode else maxf(cmd01_live, cmd01_seq)

	var slot_sep_px: float = NAN
	if is_finite(front_slot_x) and is_finite(rear_slot_x):
		slot_sep_px = absf(front_slot_x - rear_slot_x)

	# Keep geometry continuous across crouch/tall by anchoring floors to authored slot separation.
	if is_finite(slot_sep_px):
		under_body_px = maxf(under_body_px, slot_sep_px * 0.35)
		max_reach_px = maxf(max_reach_px, slot_sep_px * 0.90)

	var tap_hold_ref_sec: float = maxf(0.05, float(o.get("phase7_hold_to_walk_sec")))
	var tap01: float = clampf(_sh_seq_press_t_begin / tap_hold_ref_sec, 0.0, 1.0)

	var cross_scale_t: float = run01 if hold_mode else tap01
	if is_finite(slot_sep_px):
		cross_allow_px = maxf(cross_allow_px, slot_sep_px * lerpf(0.06, 0.22, cross_scale_t))

	var stride_px: float = cmd01 * speed_px_s * stride_t
	stride_px *= lerpf(1.0, run_stride_mult, run01)

	var primary_px: float = under_body_px
	if hold_mode:
		# Hold: feet cross the other, stride grows continuously with run ramp.
		primary_px = maxf(under_body_px, stride_px)
	else:
		# Tap: trail under-body step whose length depends on tap duration.
		# Short tap = small step under body. Longer tap = bigger step, still under-body biased.
		var tap_scale: float = lerpf(0.3, 1.0, tap01)
		primary_px = under_body_px * tap_scale

	var target_x: float = cx + float(dir_world) * primary_px

	# Allow limited crossing relative to the opposite slot before clamping to reach.
	var other_slot_x: float = rear_slot_x if swing_is_front else front_slot_x
	if dir_world > 0:
		target_x = maxf(target_x, other_slot_x - cross_allow_px)
	elif dir_world < 0:
		target_x = minf(target_x, other_slot_x + cross_allow_px)

	# Reach clamp around support slot.
	target_x = clampf(target_x, support_x - max_reach_px, support_x + max_reach_px)

	return target_x


func _pick_initiator_for_dir(dir_world: int) -> bool:
	# forward tap:  trail under-body step (trail foot moves first)
	# backward tap: lead under-body step (lead foot moves first)
	var lead_is_front: bool = bool(_owner.get("_lead_is_front"))
	var axis: float = float(_owner.get("_neutral_axis_sign"))
	if absf(axis) < 0.5:
		axis = 1.0
	var is_forward: bool = (float(dir_world) * axis) > 0.0
	if is_forward:
		return not lead_is_front
	else:
		return lead_is_front


func _compute_slot_targets_x() -> Vector2:
	var o := _owner
	if o == null:
		return Vector2(NAN, NAN)

	# 5R2-C: primary slot source = stance planner authoritative slot service (posture-derived).
	var stance_planner = o.get("_stance_planner_mod")
	if stance_planner != null and stance_planner.has_method("get_authoritative_slot_targets_x"):
		var slots_auth: Vector2 = stance_planner.get_authoritative_slot_targets_x()
		if is_finite(slots_auth.x) and is_finite(slots_auth.y):
			return slots_auth

	var ma: int = int(o.get("movement_authority_mode"))

	# Compatibility fallback (temporary during migration): legacy locomotion wrapper.
	# IMPORTANT: do NOT use this fallback in STEP_PLANNER mode, or we reintroduce the
	# legacy/mixed stance-center leak through locomotion slot wrappers.
	if ma != GCTypes.MovementAuthorityMode.STEP_PLANNER:
		var loc: GCLocomotion = o.get("_locomotion_mod") as GCLocomotion
		if loc != null:
			var slots_live: Vector2 = loc.get_slot_targets_x()
			if is_finite(slots_live.x) and is_finite(slots_live.y):
				return slots_live

	# Deterministic fallback (still authored-width based; no live foot-angle measurement).
	# In STEP_PLANNER mode prefer the planner-owned slot center publish.
	var cx: float = NAN
	if ma == GCTypes.MovementAuthorityMode.STEP_PLANNER and bool(o.get("_step_slot_center_valid")):
		cx = o.get("_step_slot_center_x") as float
	if not is_finite(cx):
		cx = o.get("_stance_center_x") as float
	if not is_finite(cx):
		cx = _refs.rb_pelvis.global_position.x if _refs != null and _refs.rb_pelvis != null else 0.0

	var stance_alpha: float = clampf(o.get("_stance_alpha") as float, 0.0, 1.0)
	var crouch01: float = 1.0 - stance_alpha
	var w_stand: float = o.get("stance_w_stand") as float
	var w_crouch: float = o.get("stance_w_crouch") as float
	var full_width: float = maxf(1.0, lerpf(w_stand, w_crouch, crouch01))
	var half: float = full_width * 0.5

	var lead_is_front: bool = bool(o.get("_lead_is_front"))
	var front_s: float = half if lead_is_front else -half
	var rear_s: float = -half if lead_is_front else half

	var ax: float = float(o.get("_neutral_axis_sign"))
	if ax == 0.0:
		ax = float(o.get("facing_sign"))
	if ax == 0.0:
		ax = 1.0

	return Vector2(cx + front_s * ax, cx + rear_s * ax)
