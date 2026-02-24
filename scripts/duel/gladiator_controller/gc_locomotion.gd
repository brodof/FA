# res://scripts/duel/gladiator_controller/gc_locomotion.gd
# Stepping: planner (TAP/WALK/STOP) + executor (UNPLANT/SWING/TOUCHDOWN). Writes owner _phase7_*, _plan_*; calls owner for helpers.
class_name GCLocomotion
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

func _legacy_phase7_authority_enabled() -> bool:
	if _owner == null:
		return false
	return int(_owner.get("movement_authority_mode")) == int(GCTypes.MovementAuthorityMode.LEGACY_PHASE7)

func update_lead_trail(cmd_move_x_norm: float, dt: float) -> void:
	var o := _owner
	if o == null:
		return
	# Exact controller logic (relocated):
	if dt <= 0.0:
		return

	# STEP_PLANNER ownership pass:
	# when planner freezes the reference frame (spawn/landing settle or idle in-place correction),
	# locomotion must not churn lead/trail identity from transient vx/foot ordering noise.
	var ma_now: int = int(o.get("movement_authority_mode"))
	if ma_now == int(GCTypes.MovementAuthorityMode.STEP_PLANNER) and bool(o.get("_step_ref_freeze_active")):
		o._dbg_lead_is_front = 1 if o._lead_is_front else 0
		return

	var dir: float = 0.0

	# Direction selection (priority): input -> pelvis vx -> 0, with hysteresis.
	if absf(cmd_move_x_norm) >= o.input_meaningful_threshold:
		dir = signf(cmd_move_x_norm)
		o._move_dir_hold_t = o.move_dir_hold_sec
	else:
		# Do NOT infer direction from vx during landing/impact (swaps stance targets mid-collision).
		var allow_vx_dir: bool = (o._ground_grace_t > 0.0) and (o._impact_timer <= 0.0) and o._rest_captured
		if allow_vx_dir and o._rb_pelvis != null:
			var vx: float = o._rb_pelvis.linear_velocity.x
			if absf(vx) >= o.vel_meaningful_threshold:
				dir = signf(vx)
				o._move_dir_hold_t = o.move_dir_hold_sec

		if dir == 0.0:
			o._move_dir_hold_t = maxf(0.0, o._move_dir_hold_t - dt)
			if o._move_dir_hold_t > 0.0:
				dir = o._move_dir_last

	if dir != 0.0:
		o._move_dir_last = dir
		o._neutral_axis_sign = (1 if dir > 0.0 else -1)

	o._move_dir_world = dir
	o._dbg_move_dir_world = dir

	# Lead/trail identity: foot X ordering relative to pelvis. Never depends on facing_sign.
	# If stopped, keep last identity (no jitter).
	if dir == 0.0:
		o._dbg_lead_is_front = 1 if o._lead_is_front else 0
		return

	var cx: float = o._rb_pelvis.global_position.x if o._rb_pelvis != null else NAN
	var fx: float = _foot_best_x(o, true)
	var rx: float = _foot_best_x(o, false)
	if not is_finite(cx) or not is_finite(fx) or not is_finite(rx):
		o._dbg_lead_is_front = 1 if o._lead_is_front else 0
		return

	var want_lead_front: bool = (fx > rx) if (dir > 0.0) else (fx < rx)
	var gap: float = absf(fx - rx)

	if not o._lead_initialized:
		o._lead_is_front = want_lead_front
		o._lead_initialized = true
	elif want_lead_front != o._lead_is_front and gap >= o.lead_switch_margin_px:
		o._lead_is_front = want_lead_front

	o._dbg_lead_is_front = 1 if o._lead_is_front else 0

func update_locomotion_plan(cmd_move_x_norm: float, dt: float, front_g: bool, rear_g: bool, grounded_eff: bool, _stabF: float, _stabR: float) -> void:
	if not _legacy_phase7_authority_enabled():
		_owner.set("_phase7_plan_active", false)
		_owner.set("_dbg_phase7_own", 0)
		return
	_owner.set("_phase7_plan_active", false)
	_owner.set("_dbg_phase7_own", 0)
	_owner.set("_dbg_phase7_target_fx", NAN)
	_owner.set("_dbg_phase7_target_rx", NAN)
	var loc_state: int = _owner.get("_phase7_state")
	var seq_active_is_front: bool = _owner.get("_phase7_seq_active_is_front")
	var seq_step_index: int = _owner.get("_phase7_seq_step_index")
	var seq_dir_world: int = _owner.get("_phase7_seq_dir_world")
	var seq_dir_facing: int = _owner.get("_phase7_seq_dir_facing")
	var seq_gait_lead_is_front: bool = _owner.get("_phase7_seq_gait_lead_is_front")
	var seq_anat_front_is_front: bool = _owner.get("_phase7_seq_anat_front_is_front")
	_owner.set("_dbg_phase7_state", loc_state)
	_owner.set("_dbg_phase7_active_is_front", int(seq_active_is_front))
	_owner.set("_dbg_phase7_seq_index", seq_step_index)
	_owner.set("_dbg_phase7_dir_w", seq_dir_world)
	_owner.set("_dbg_phase7_dir_f", seq_dir_facing)
	_owner.set("_dbg_phase7_tap_lead_is_front", int(seq_gait_lead_is_front))
	_owner.set("_dbg_phase7_tap_anat_front_is_front", int(seq_anat_front_is_front))

	if not _owner.get("phase7_enable"):
		_owner.set("_phase7_state", GCTypes.Phase7LocState.IDLE)
		return

	var exec_done_pulse: bool = _owner.get("_phase7_exec_done_pulse")
	if exec_done_pulse:
		_owner.set("_phase7_exec_done_pulse", false)
		if loc_state == GCTypes.Phase7LocState.TAP_SEQ:
			seq_step_index += 1
			_owner.set("_phase7_seq_step_index", seq_step_index)
			if seq_step_index >= 2:
				_owner.set("_phase7_state", GCTypes.Phase7LocState.IDLE)
			else:
				seq_active_is_front = not seq_active_is_front
				_owner.set("_phase7_seq_active_is_front", seq_active_is_front)
		elif loc_state == GCTypes.Phase7LocState.WALK_SEQ:
			var done_pulse_stop: bool = _owner.get("_phase7_stop_pending")
			if done_pulse_stop:
				_owner.set("_phase7_state", GCTypes.Phase7LocState.STOP_SEQ)
				_owner.set("_phase7_stop_pending", false)
				_owner.set("_phase7_seq_step_index", 0)
				var pick: bool = _phase7_pick_stop_correction_foot()
				_owner.set("_phase7_seq_active_is_front", pick)
			else:
				var dir_done: int = _owner.get("_phase7_seq_dir_world")
				if dir_done == 0:
					dir_done = GCMath.signi(_owner.get("_move_dir_world") as float)
				var last_done_is_front: bool = _owner.get("_phase7_exec_done_is_front")
				var next_pick: bool = _phase7_pick_walk_next_foot(dir_done, last_done_is_front)
				_owner.set("_phase7_seq_gait_lead_is_front", _owner.get("_lead_is_front"))
				_owner.set("_phase7_seq_active_is_front", next_pick)
		elif loc_state == GCTypes.Phase7LocState.STOP_SEQ:
			_owner.set("_phase7_state", GCTypes.Phase7LocState.IDLE)

	var input_meaningful_threshold: float = _owner.get("input_meaningful_threshold") as float
	var cmd_active: bool = absf(cmd_move_x_norm) >= input_meaningful_threshold
	var cmd_hold_t: float = _owner.get("_phase7_hold_t") as float
	if cmd_active:
		cmd_hold_t += dt
	else:
		cmd_hold_t = 0.0
	_owner.set("_phase7_hold_t", cmd_hold_t)
	var prev_cmd_active: bool = _owner.get("_phase7_prev_cmd_active")
	var tap_rise: bool = cmd_active and (not prev_cmd_active)
	_owner.set("_phase7_prev_cmd_active", cmd_active)

	var hold_to_walk_sec: float = _owner.get("phase7_hold_to_walk_sec") as float

	if loc_state == GCTypes.Phase7LocState.IDLE and tap_rise:
		_owner.set("_phase7_state", GCTypes.Phase7LocState.TAP_SEQ)
		_owner.set("_phase7_stop_pending", false)
		_owner.set("_phase7_seq_step_index", 0)
		var dir_sign: int = GCMath.signi(cmd_move_x_norm)
		_owner.set("_phase7_seq_dir_world", dir_sign)
		_owner.set("_phase7_seq_gait_lead_is_front", _owner.get("_lead_is_front"))
		_owner.set("_phase7_seq_dir_facing", 0)
		_owner.set("_phase7_seq_anat_front_is_front", true)
		if dir_sign != 0:
			_owner.set("_phase7_seq_active_is_front", _phase7_pick_initiator_for_dir(dir_sign))
		else:
			_owner.set("_phase7_state", GCTypes.Phase7LocState.IDLE)

	if cmd_active and (cmd_hold_t >= hold_to_walk_sec) and (loc_state == GCTypes.Phase7LocState.TAP_SEQ or loc_state == GCTypes.Phase7LocState.IDLE):
		loc_state = GCTypes.Phase7LocState.WALK_SEQ
		_owner.set("_phase7_state", loc_state)
		_owner.set("_phase7_stop_pending", false)
		_owner.set("_phase7_seq_step_index", 0)
		var dir_sign: int = GCMath.signi(cmd_move_x_norm)
		_owner.set("_phase7_seq_dir_world", dir_sign)
		_owner.set("_phase7_seq_gait_lead_is_front", _owner.get("_lead_is_front"))
		_owner.set("_phase7_seq_dir_facing", 0)
		_owner.set("_phase7_seq_anat_front_is_front", true)
		if dir_sign != 0:
			_owner.set("_phase7_seq_active_is_front", _phase7_pick_initiator_for_dir(dir_sign))
		else:
			_owner.set("_phase7_state", GCTypes.Phase7LocState.IDLE)

	var stop_correct_enable: bool = _owner.get("phase7_stop_correct_enable")
	if loc_state == GCTypes.Phase7LocState.WALK_SEQ and (not cmd_active) and stop_correct_enable:
		_owner.set("_phase7_stop_pending", true)

	var impact_timer: float = _owner.get("_impact_timer") as float
	var touchdown_ramp_t: float = _owner.get("_touchdown_ramp_t") as float
	var plant_touchdown_grace_sec: float = _owner.get("plant_touchdown_grace_sec") as float
	var spawn_brace_active: bool = _owner.get("_spawn_brace_active")
	var stance_changing: bool = _owner.get("_phase6_stance_changing")
	var allow: bool = grounded_eff \
		and (impact_timer <= 0.0) \
		and (touchdown_ramp_t >= plant_touchdown_grace_sec) \
		and (not spawn_brace_active) \
		and (not stance_changing) \
		and (front_g or rear_g)

	if not allow:
		loc_state = _owner.get("_phase7_state")
		seq_active_is_front = _owner.get("_phase7_seq_active_is_front")
		seq_step_index = _owner.get("_phase7_seq_step_index")
		seq_dir_world = _owner.get("_phase7_seq_dir_world")
		seq_dir_facing = _owner.get("_phase7_seq_dir_facing")
		seq_gait_lead_is_front = _owner.get("_phase7_seq_gait_lead_is_front")
		seq_anat_front_is_front = _owner.get("_phase7_seq_anat_front_is_front")
		_owner.set("_dbg_phase7_state", loc_state)
		_owner.set("_dbg_phase7_active_is_front", int(seq_active_is_front))
		_owner.set("_dbg_phase7_seq_index", seq_step_index)
		_owner.set("_dbg_phase7_dir_w", seq_dir_world)
		_owner.set("_dbg_phase7_dir_f", seq_dir_facing)
		_owner.set("_dbg_phase7_tap_lead_is_front", int(seq_gait_lead_is_front))
		_owner.set("_dbg_phase7_tap_anat_front_is_front", int(seq_anat_front_is_front))
		return

	loc_state = _owner.get("_phase7_state")
	if loc_state == GCTypes.Phase7LocState.IDLE:
		return

	seq_dir_world = _owner.get("_phase7_seq_dir_world")
	var dir_world: int = seq_dir_world
	if dir_world == 0:
		dir_world = GCMath.signi(cmd_move_x_norm)
	if dir_world == 0:
		return

	var gy: float = _ground_y()
	_owner.set("_plan_target_front", Vector2(_neutral_target_x_for_foot(true), gy))
	_owner.set("_plan_target_rear", Vector2(_neutral_target_x_for_foot(false), gy))

	seq_active_is_front = _owner.get("_phase7_seq_active_is_front")
	var pelvis_x: float = _refs.rb_pelvis.global_position.x if _refs.rb_pelvis != null else 0.0
	var target_x: float = pelvis_x
	seq_step_index = _owner.get("_phase7_seq_step_index")

	if loc_state == GCTypes.Phase7LocState.TAP_SEQ:
		if seq_step_index == 0:
			var under_body_step_px: float = _owner.get("phase7_under_body_step_px") as float
			target_x = pelvis_x + float(dir_world) * under_body_step_px
		else:
			target_x = _neutral_target_x_for_foot(seq_active_is_front)
	elif loc_state == GCTypes.Phase7LocState.WALK_SEQ:
		target_x = _phase7_compute_walk_target_x(seq_active_is_front, dir_world, cmd_move_x_norm, pelvis_x)
	elif loc_state == GCTypes.Phase7LocState.STOP_SEQ:
		target_x = _neutral_target_x_for_foot(seq_active_is_front)

	target_x = _phase7_clamp_target_x(seq_active_is_front, dir_world, target_x)
	var target_w: Vector2 = Vector2(target_x, gy)

	if seq_active_is_front:
		_owner.set("_plan_front", GCTypes.PlanFoot.SWING)
		_owner.set("_plan_rear", GCTypes.PlanFoot.PLANTED)
		_owner.set("_plan_target_front", target_w)
	else:
		_owner.set("_plan_front", GCTypes.PlanFoot.PLANTED)
		_owner.set("_plan_rear", GCTypes.PlanFoot.SWING)
		_owner.set("_plan_target_rear", target_w)
	_owner.set("_plan_support_is_front", not seq_active_is_front)
	_owner.set("_phase7_plan_active", true)
	_owner.set("_dbg_phase7_own", 1)

	var queue_stop_pending: bool = _owner.get("_phase7_stop_pending")
	var q: int = 0
	if loc_state == GCTypes.Phase7LocState.TAP_SEQ:
		q = max(0, 2 - seq_step_index)
	elif loc_state == GCTypes.Phase7LocState.STOP_SEQ:
		q = 1
	elif loc_state == GCTypes.Phase7LocState.WALK_SEQ:
		q = 2 if queue_stop_pending else 1
	_owner.set("_dbg_phase7_queue_len", q)
	_owner.set("_dbg_phase7_state", loc_state)
	_owner.set("_dbg_phase7_active_is_front", int(seq_active_is_front))
	_owner.set("_dbg_phase7_seq_index", seq_step_index)
	_owner.set("_dbg_phase7_dir_w", seq_dir_world)
	_owner.set("_dbg_phase7_dir_f", seq_dir_facing)
	_owner.set("_dbg_phase7_tap_lead_is_front", int(seq_gait_lead_is_front))
	_owner.set("_dbg_phase7_tap_anat_front_is_front", int(seq_anat_front_is_front))
	var plan_target_front: Vector2 = _owner.get("_plan_target_front")
	var plan_target_rear: Vector2 = _owner.get("_plan_target_rear")
	_owner.set("_dbg_phase7_target_fx", plan_target_front.x)
	_owner.set("_dbg_phase7_target_rx", plan_target_rear.x)

func exec_update_state(dt: float, front_g: bool, rear_g: bool) -> void:
	if not _legacy_phase7_authority_enabled():
		_owner.set("_phase7_force_plant_front", false)
		_owner.set("_phase7_force_plant_rear", false)
		_owner.set("_phase7_slide_front", false)
		_owner.set("_phase7_slide_rear", false)
		_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.IDLE)
		_owner.set("_dbg_phase7_exec_phase", int(GCTypes.Phase7ExecPhase.IDLE))
		return
	_owner.set("_phase7_force_plant_front", false)
	_owner.set("_phase7_force_plant_rear", false)
	_owner.set("_phase7_slide_front", false)
	_owner.set("_phase7_slide_rear", false)
	_owner.set("_dbg_phase7_slide_fail", 0)

	if not _owner.get("phase7_enable") or not _owner.get("_phase7_plan_active"):
		_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.IDLE)
		_owner.set("_dbg_phase7_exec_phase", int(GCTypes.Phase7ExecPhase.IDLE))
		return

	var plan_front: int = _owner.get("_plan_front")
	var plan_rear: int = _owner.get("_plan_rear")
	var plan_target_front: Vector2 = _owner.get("_plan_target_front")
	var plan_target_rear: Vector2 = _owner.get("_plan_target_rear")
	var req_front: bool = (plan_front == GCTypes.PlanFoot.SWING) and is_finite(plan_target_front.x)
	var req_rear: bool = (plan_rear == GCTypes.PlanFoot.SWING) and is_finite(plan_target_rear.x)
	if (not req_front) and (not req_rear):
		_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.IDLE)
		_owner.set("_dbg_phase7_exec_phase", int(GCTypes.Phase7ExecPhase.IDLE))
		return

	var req_is_front: bool = req_front
	if req_front and req_rear:
		req_is_front = true

	var exec_phase: int = _owner.get("_phase7_exec_phase")
	if exec_phase == GCTypes.Phase7ExecPhase.IDLE:
		_owner.set("_phase7_exec_is_front", req_is_front)
		_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.UNPLANT)
		_owner.set("_phase7_exec_unplant_t", _owner.get("phase7_unplant_sec") as float)
		_owner.set("_phase7_exec_swing_t", 0.0)
		_owner.set("_phase7_exec_touchdown_lock_t", 0.0)
		_owner.set("_phase7_exec_left_ground", false)

	exec_phase = _owner.get("_phase7_exec_phase")
	var exec_is_front: bool = _owner.get("_phase7_exec_is_front")
	var exec_target_w: Vector2 = _owner.get("_phase7_exec_target_w")

	# Phase7 invariant: during UNPLANT/SWING, only the non-swing foot may be treated as the support foot.
	if exec_phase == GCTypes.Phase7ExecPhase.UNPLANT or exec_phase == GCTypes.Phase7ExecPhase.SWING:
		_owner.set("support_is_front", not exec_is_front)

	if exec_phase == GCTypes.Phase7ExecPhase.UNPLANT:
		var t: Vector2 = plan_target_front if exec_is_front else plan_target_rear
		if is_finite(t.x):
			var swing_lift_px: float = _owner.get("phase7_swing_lift_px") as float
			var rb_u: RigidBody2D = _refs.rb_foot_front if exec_is_front else _refs.rb_foot_rear
			var gy_u: float = _ground_y()
			if not is_finite(gy_u):
				gy_u = t.y
			if swing_lift_px > 0.0 and rb_u != null and is_finite(gy_u):
				# Lift-first: keep target_x at current foot x while unplanting.
				exec_target_w = Vector2(rb_u.global_position.x, gy_u - swing_lift_px)
			else:
				exec_target_w = t
			_owner.set("_phase7_exec_target_w", exec_target_w)

	var g: bool = front_g if exec_is_front else rear_g
	var exec_unplant_t: float = _owner.get("_phase7_exec_unplant_t") as float

	if exec_phase == GCTypes.Phase7ExecPhase.UNPLANT:
		exec_unplant_t = maxf(0.0, exec_unplant_t - dt)
		_owner.set("_phase7_exec_unplant_t", exec_unplant_t)
		if exec_is_front and g:
			_owner.set("_phase7_slide_front", true)
		if (not exec_is_front) and g:
			_owner.set("_phase7_slide_rear", true)
		if not g:
			_owner.set("_phase7_exec_left_ground", true)
		if (not g) or (exec_unplant_t <= 0.0):
			_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.SWING)

	elif exec_phase == GCTypes.Phase7ExecPhase.SWING:
		if exec_is_front and g:
			_owner.set("_phase7_slide_front", true)
		if (not exec_is_front) and g:
			_owner.set("_phase7_slide_rear", true)
		var exec_swing_t: float = _owner.get("_phase7_exec_swing_t") as float
		exec_swing_t += dt
		_owner.set("_phase7_exec_swing_t", exec_swing_t)
		if not g:
			_owner.set("_phase7_exec_left_ground", true)

		var swing_lift_px: float = _owner.get("phase7_swing_lift_px") as float
		var lift_leave_ground_y_eps: float = _owner.get("phase7_lift_leave_ground_y_eps") as float
		var step_done_pos_eps: float = _owner.get("phase7_step_done_pos_eps") as float
		var step_done_vel_eps: float = _owner.get("phase7_step_done_vel_eps") as float
		var lift_min_swing_sec: float = _owner.get("phase7_lift_min_swing_sec") as float
		var touchdown_lock_sec: float = _owner.get("phase7_touchdown_lock_sec") as float
		exec_target_w = _owner.get("_phase7_exec_target_w")

		if swing_lift_px > 0.0:
			var rb_step: RigidBody2D = _refs.rb_foot_front if exec_is_front else _refs.rb_foot_rear
			var t_step: Vector2 = plan_target_front if exec_is_front else plan_target_rear
			var gy_now: float = _ground_y()
			if not is_finite(gy_now):
				gy_now = t_step.y

			# Airborne detect: either sensor ungrounds, or foot clears ground by eps.
			var exec_left_ground: bool = _owner.get("_phase7_exec_left_ground")
			if rb_step != null and is_finite(gy_now):
				if rb_step.global_position.y <= (gy_now - lift_leave_ground_y_eps):
					exec_left_ground = true
					_owner.set("_phase7_exec_left_ground", true)

			# Lift → travel → drop:
			# - Until airborne: no horizontal chase (prevents ground-slide).
			# - Once airborne (or after max time): chase planned X.
			# - Near target (or after max time): drop target_y back to ground so contact + touchdown can happen.
			if rb_step != null and is_finite(gy_now):
				var drop_dx: float = _owner.get("phase7_swing_drop_dx_px") as float
				var drop_min: float = _owner.get("phase7_swing_drop_min_sec") as float
				var drop_max: float = _owner.get("phase7_swing_drop_max_sec") as float
				var td_clr: float = _owner.get("phase7_touchdown_clearance_px") as float

				var can_chase_x: bool = exec_left_ground or (exec_swing_t >= drop_max)

				var target_x: float = rb_step.global_position.x
				if can_chase_x and is_finite(t_step.x):
					target_x = t_step.x

				var want_drop: bool = false
				if can_chase_x and is_finite(t_step.x):
					var dx_to_plan: float = absf(rb_step.global_position.x - t_step.x)
					if (exec_swing_t >= drop_min) and (dx_to_plan <= drop_dx):
						want_drop = true
					if exec_swing_t >= drop_max:
						want_drop = true

				var target_y: float = (gy_now - td_clr) if want_drop else (gy_now - swing_lift_px)
				exec_target_w = Vector2(target_x, target_y)
				_owner.set("_phase7_exec_target_w", exec_target_w)

			if g and rb_step != null:
				var dx_l: float = absf(rb_step.global_position.x - exec_target_w.x)
				var vx_l: float = absf(rb_step.linear_velocity.x)
				var reached: bool = (dx_l <= step_done_pos_eps) and (vx_l <= step_done_vel_eps)
				var time_ok: bool = (exec_swing_t >= lift_min_swing_sec)
				exec_left_ground = _owner.get("_phase7_exec_left_ground")

				# If we are in the drop stage and we re-contacted, that's touchdown even if X isn't perfect.
				# (TOUCHDOWN stage will handle the final settle/plant gate.)
				var want_drop_now: bool = (exec_target_w.y >= (gy_now - (_owner.get("phase7_touchdown_clearance_px") as float) - 0.001))
				# Touchdown only after foot has been airborne at least once (no slide-touchdown escape).
				if exec_left_ground and time_ok and (reached or want_drop_now):
					_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.TOUCHDOWN)
					_owner.set("_phase7_exec_touchdown_lock_t", touchdown_lock_sec)
					_owner.set("_phase7_exec_touchdown_elapsed", 0.0)
		else:
			var rb: RigidBody2D = _refs.rb_foot_front if exec_is_front else _refs.rb_foot_rear
			if g and rb != null:
				var dx: float = absf(rb.global_position.x - exec_target_w.x)
				var vx: float = absf(rb.linear_velocity.x)
				if (dx <= step_done_pos_eps) and (vx <= step_done_vel_eps):
					_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.TOUCHDOWN)
					_owner.set("_phase7_exec_touchdown_lock_t", touchdown_lock_sec)
					_owner.set("_phase7_exec_touchdown_elapsed", 0.0)

	elif exec_phase == GCTypes.Phase7ExecPhase.TOUCHDOWN:
		var exec_touchdown_lock_t: float = _owner.get("_phase7_exec_touchdown_lock_t") as float
		exec_touchdown_lock_t = maxf(0.0, exec_touchdown_lock_t - dt)
		_owner.set("_phase7_exec_touchdown_lock_t", exec_touchdown_lock_t)
		var elapsed: float = _owner.get("_phase7_exec_touchdown_elapsed") as float
		_owner.set("_phase7_exec_touchdown_elapsed", elapsed + dt)
		# Keep touchdown target on the ground so contact + planting can actually happen.
		var gy_td: float = _ground_y()
		var td_clr: float = _owner.get("phase7_touchdown_clearance_px") as float
		if is_finite(gy_td):
			var tdw: Vector2 = _owner.get("_phase7_exec_target_w")
			if is_finite(tdw.x):
				tdw = Vector2(tdw.x, gy_td - td_clr)
				_owner.set("_phase7_exec_target_w", tdw)
		_owner.set("_phase7_force_plant_front", false)
		_owner.set("_phase7_force_plant_rear", false)
		if phase7_landing_accepts_contact(exec_is_front):
			if exec_is_front:
				_owner.set("_phase7_force_plant_front", true)
			else:
				_owner.set("_phase7_force_plant_rear", true)
		if not g:
			_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.SWING)
			_owner.set("_phase7_exec_touchdown_lock_t", 0.0)
			_owner.set("_phase7_exec_touchdown_elapsed", 0.0)
			_owner.set("_phase7_force_plant_front", false)
			_owner.set("_phase7_force_plant_rear", false)
		else:
			var front_state: int = _owner.get("_front_state")
			var rear_state: int = _owner.get("_rear_state")
			var st: int = front_state if exec_is_front else rear_state
			var rb2: RigidBody2D = _refs.rb_foot_front if exec_is_front else _refs.rb_foot_rear
			exec_touchdown_lock_t = _owner.get("_phase7_exec_touchdown_lock_t") as float
			elapsed = _owner.get("_phase7_exec_touchdown_elapsed") as float
			var pos_eps: float = _owner.get("phase7_step_done_pos_eps") as float
			var vel_eps: float = _owner.get("phase7_step_done_vel_eps") as float
			var settle_max: float = _owner.get("phase7_step_done_settle_max_sec") as float
			if (exec_touchdown_lock_t <= 0.0) and (st == GCTypes.FootPlantState.PLANTED) and (rb2 != null):
				var dx2: float = absf(rb2.global_position.x - exec_target_w.x)
				var vx2: float = absf(rb2.linear_velocity.x)
				var at_target: bool = (dx2 <= pos_eps) and (vx2 <= vel_eps)
				var settled: bool = is_finite(settle_max) and (settle_max > 0.0) and (elapsed >= settle_max) and (vx2 <= vel_eps)
				if at_target or settled:
					_owner.set("_phase7_exec_phase", GCTypes.Phase7ExecPhase.IDLE)
					_owner.set("_phase7_exec_done_pulse", true)
					_owner.set("_phase7_exec_done_is_front", exec_is_front)
					_owner.set("_phase7_force_plant_front", false)
					_owner.set("_phase7_force_plant_rear", false)
					_owner.set("_phase7_exec_touchdown_elapsed", 0.0)

	_owner.set("_phase7_exec_state_front", phase7_foot_exec_state(true))
	_owner.set("_phase7_exec_state_rear", phase7_foot_exec_state(false))
	_owner.set("_dbg_phase7_exec_phase", int(_owner.get("_phase7_exec_phase")))
	_owner.set("_dbg_phase7_slideF", int(_owner.get("_phase7_slide_front")))
	_owner.set("_dbg_phase7_slideR", int(_owner.get("_phase7_slide_rear")))

func phase7_exec_busy() -> bool:
	if not _legacy_phase7_authority_enabled():
		return false
	if not _owner.get("phase7_enable"):
		return false
	var exec_phase: int = _owner.get("_phase7_exec_phase")
	var loc_state: int = _owner.get("_phase7_state")
	return bool(_owner.get("_phase7_plan_active")) \
		or (exec_phase != GCTypes.Phase7ExecPhase.IDLE) \
		or (loc_state != GCTypes.Phase7LocState.IDLE)

func _phase7_truth_grounded(is_front: bool) -> bool:
	return (_owner.get("_truth_front_g") as bool) if is_front else (_owner.get("_truth_rear_g") as bool)

func _phase7_truth_stability(is_front: bool) -> float:
	return (_owner.get("_truth_front_stab") as float) if is_front else (_owner.get("_truth_rear_stab") as float)

func _phase7_active_exec_foot(is_front: bool) -> bool:
	if not _legacy_phase7_authority_enabled():
		return false
	if not _owner.get("phase7_enable"):
		return false
	var exec_phase: int = _owner.get("_phase7_exec_phase")
	if exec_phase == GCTypes.Phase7ExecPhase.IDLE:
		return false
	return (_owner.get("_phase7_exec_is_front") as bool) == is_front

func _phase7_contact_normal_ok_for_landing(is_front: bool) -> bool:
	var sensor: Node = _refs.foot_sensor_front if is_front else _refs.foot_sensor_rear
	if sensor == null:
		return true
	var grounded_sensor: bool = sensor.get("grounded") as bool
	if not grounded_sensor:
		return false
	var n: Vector2 = sensor.get("contact_normal_w") as Vector2
	if not is_finite(n.x) or not is_finite(n.y) or n.length() < 0.2:
		return false
	n = n.normalized()
	var max_slope_deg: float = clampf(_owner.get("foot_flat_max_slope_deg") as float, 0.0, 89.9)
	var min_up_dot: float = cos(deg_to_rad(max_slope_deg))
	return n.dot(Vector2.UP) >= min_up_dot

func phase7_landing_accepts_contact(is_front: bool) -> bool:
	if not _phase7_active_exec_foot(is_front):
		return false
	if not (_owner.get("_phase7_exec_left_ground") as bool):
		return false
	if not _phase7_truth_grounded(is_front):
		return false
	if not _phase7_contact_normal_ok_for_landing(is_front):
		return false

	var min_stab: float = _owner.get("plant_min_stability") as float
	if _phase7_truth_stability(is_front) < min_stab:
		return false

	var plant_pelvis_vy_max: float = _owner.get("plant_pelvis_vy_max") as float
	if _refs.rb_pelvis != null and absf(_refs.rb_pelvis.linear_velocity.y) > plant_pelvis_vy_max:
		return false

	var plant_foot_vy_max: float = _owner.get("plant_foot_vy_max") as float
	var foot: RigidBody2D = _refs.rb_foot_front if is_front else _refs.rb_foot_rear
	if foot != null and absf(foot.linear_velocity.y) > plant_foot_vy_max:
		return false

	return true

func _phase7_compute_foot_exec_state(is_front: bool) -> int:
	var plant_state: int = _owner.get("_front_state") if is_front else _owner.get("_rear_state")
	var planted_default: int = GCTypes.Phase7FootExecState.PLANTED if (plant_state == GCTypes.FootPlantState.PLANTED) else GCTypes.Phase7FootExecState.SWING

	if not _owner.get("phase7_enable"):
		return planted_default

	if not _phase7_active_exec_foot(is_front):
		return planted_default

	var exec_phase: int = _owner.get("_phase7_exec_phase")
	match exec_phase:
		GCTypes.Phase7ExecPhase.UNPLANT:
			return GCTypes.Phase7FootExecState.RELEASE
		GCTypes.Phase7ExecPhase.SWING:
			if phase7_landing_accepts_contact(is_front):
				return GCTypes.Phase7FootExecState.LANDING_CANDIDATE
			return GCTypes.Phase7FootExecState.SWING
		GCTypes.Phase7ExecPhase.TOUCHDOWN:
			if plant_state == GCTypes.FootPlantState.PLANTED:
				return GCTypes.Phase7FootExecState.PLANTED_CONFIRMED
			if phase7_landing_accepts_contact(is_front):
				return GCTypes.Phase7FootExecState.LANDING_CANDIDATE
			return GCTypes.Phase7FootExecState.SWING
		_:
			return planted_default

func phase7_foot_exec_state(is_front: bool) -> int:
	var s: int = _phase7_compute_foot_exec_state(is_front)
	if is_front:
		_owner.set("_phase7_exec_state_front", s)
	else:
		_owner.set("_phase7_exec_state_rear", s)
	return s

func phase7_foot_owns_swing_orientation(is_front: bool) -> bool:
	if not _phase7_active_exec_foot(is_front):
		return false
	var s: int = phase7_foot_exec_state(is_front)
	return s == GCTypes.Phase7FootExecState.RELEASE \
		or s == GCTypes.Phase7FootExecState.SWING \
		or s == GCTypes.Phase7FootExecState.LANDING_CANDIDATE

func phase7_foot_owns_swing_position(is_front: bool) -> bool:
	# Brief 4 contract: same owner as swing attitude until plant is confirmed.
	return phase7_foot_owns_swing_orientation(is_front)

func phase7_allow_generic_air_pd(is_front: bool) -> bool:
	# Brief 5 fail-safe:
	# In STEP_PLANNER mode, generic airborne foot PD must be globally inert.
	# Unified foot attitude ownership lives in gc_foot_plant via pipeline arbitration.
	if _owner != null and int(_owner.get("movement_authority_mode")) == int(GCTypes.MovementAuthorityMode.STEP_PLANNER):
		return false
	# Legacy Phase7 mode: generic air PD is allowed only when Phase7 swing/landing does not own this foot.
	return not phase7_foot_owns_swing_orientation(is_front)

func phase7_allow_recenter(_is_front: bool) -> bool:
	if not _legacy_phase7_authority_enabled():
		return true
	if not _owner.get("phase7_enable"):
		return true
	# Keep support stance stable during a live step; active foot is always disallowed.
	if _owner.get("_phase7_plan_active") or (_owner.get("_phase7_exec_phase") != GCTypes.Phase7ExecPhase.IDLE):
		return false
	return true

func phase7_allow_planted_flatness(is_front: bool) -> bool:
	# Planted flatness is disabled only while Phase7 swing/landing attitude owns this foot.
	return not phase7_foot_owns_swing_orientation(is_front)

func phase7_allow_plant_glue(is_front: bool) -> bool:
	if not _owner.get("phase7_enable"):
		return true
	# Never allow plant glue on the active exec foot while a Phase7 step is active.
	if _phase7_active_exec_foot(is_front):
		return false
	return true

func phase7_force_release_plant_state(is_front: bool) -> bool:
	if not _phase7_active_exec_foot(is_front):
		return false
	var s: int = phase7_foot_exec_state(is_front)
	return s == GCTypes.Phase7FootExecState.RELEASE or s == GCTypes.Phase7FootExecState.SWING

func phase7_allow_plant_candidate_enter(is_front: bool) -> bool:
	# CANDIDATE/PLANTED entry for the active exec foot is only legal in landing states.
	if not _phase7_active_exec_foot(is_front):
		return true
	var s: int = phase7_foot_exec_state(is_front)
	return s == GCTypes.Phase7FootExecState.LANDING_CANDIDATE \
		or s == GCTypes.Phase7FootExecState.PLANTED_CONFIRMED

func phase7_is_active_swing_foot(is_front: bool) -> bool:
	# Backward-compatible helper used by pipeline; now backed by Brief 4 ownership contract.
	return phase7_foot_owns_swing_orientation(is_front)

func phase7_apply_recenter_gates() -> void:
	if not _legacy_phase7_authority_enabled():
		return
	if not _owner.get("phase7_enable"):
		return

	var allow_f: bool = (_owner.get("_allow_recenter_front") as bool) and phase7_allow_recenter(true)
	var allow_r: bool = (_owner.get("_allow_recenter_rear") as bool) and phase7_allow_recenter(false)
	_owner.set("_allow_recenter_front", allow_f)
	_owner.set("_allow_recenter_rear", allow_r)

func exec_apply_swing_attitude(spawn_gate: float, front_g: bool, rear_g: bool) -> void:
	if not _legacy_phase7_authority_enabled():
		return
	if not _owner.get("phase7_enable"):
		return
	if _owner._posture_mod == null:
		return

	var swing_is_front: bool = _owner.get("_phase7_exec_is_front")
	if not phase7_foot_owns_swing_orientation(swing_is_front):
		return

	var foot_state: int = phase7_foot_exec_state(swing_is_front)
	var swing_grounded: bool = front_g if swing_is_front else rear_g
	# In LANDING_CANDIDATE the foot may be grounded, and Phase7 still owns orientation.
	if swing_grounded and (foot_state != GCTypes.Phase7FootExecState.LANDING_CANDIDATE):
		return

	var foot_rb: RigidBody2D = _refs.rb_foot_front if swing_is_front else _refs.rb_foot_rear
	if foot_rb == null:
		return

	var ref_ang: float = _owner._swing_flat_ref_angle_f if swing_is_front else _owner._swing_flat_ref_angle_r
	if not is_finite(ref_ang):
		ref_ang = 0.0

	var k: float = _owner.get("swing_foot_flat_k") as float
	var d: float = _owner.get("swing_foot_flat_d") as float
	var tau_max: float = _owner.get("swing_foot_flat_tau_max") as float
	var strength: float = _owner.get("swing_foot_flat_strength") as float
	if is_finite(k) and is_finite(d) and is_finite(tau_max) and strength > 0.0:
		_owner._posture_mod.apply_angle_pd(foot_rb, ref_ang, k, d, tau_max, strength * spawn_gate)

func exec_apply_forces(dt: float, spawn_gate: float, front_g: bool, rear_g: bool) -> void:
	if not _legacy_phase7_authority_enabled():
		return
	if not _owner.get("phase7_enable") or not _owner.get("_phase7_plan_active"):
		return
	var exec_is_front: bool = _owner.get("_phase7_exec_is_front")
	if not phase7_foot_owns_swing_position(exec_is_front):
		return
	var rb: RigidBody2D = _refs.rb_foot_front if exec_is_front else _refs.rb_foot_rear
	if rb == null:
		return
	var g: bool = front_g if exec_is_front else rear_g
	var unweight_mult: float = _owner.get("phase7_unweight_mult") as float
	var g_val: float = _refs.g
	if g and (unweight_mult > 0.0):
		var uw: float = clampf(unweight_mult, 0.0, 2.5)
		rb.apply_central_force(Vector2(0.0, -rb.mass * g_val * uw) * spawn_gate)
	var exec_target_w: Vector2 = _owner.get("_phase7_exec_target_w")

	var swing_force_mult: float = _owner.get("phase7_swing_force_mult") as float
	var run_ramp_sec: float = _owner.get("phase7_run_ramp_sec") as float
	var run_swing_force_mult: float = _owner.get("phase7_run_swing_force_mult") as float
	var hold_t: float = _owner.get("_phase7_hold_t") as float
	var run01: float = clampf(hold_t / maxf(0.001, run_ramp_sec), 0.0, 1.0)
	swing_force_mult *= lerpf(1.0, maxf(1.0, run_swing_force_mult), run01)

	if is_finite(exec_target_w.x):
		if _owner._ground_truth_mod != null:
			_owner._ground_truth_mod.apply_swing_force(exec_is_front, exec_target_w, dt, spawn_gate * swing_force_mult)

# --- Pelvis X support (PD toward target_x; was _apply_pelvis_x_support on controller) ---
func apply_pelvis_x_support(target_x: float, desired_vx: float, _dt: float, spawn01: float) -> void:
	var rb: RigidBody2D = _refs.rb_pelvis
	if rb == null or spawn01 <= 0.0:
		return
	if not is_finite(target_x):
		return
	_owner.set("_dbg_px_called", true)
	var px: float = rb.global_position.x
	var vx: float = rb.linear_velocity.x
	var err: float = target_x - px
	var deadband: float = _owner.get("phase2_pelvis_x_deadband_px") as float
	if absf(err) < deadband:
		err = 0.0
	var m_eff: float = _refs.total_mass
	var freq_hz: float = maxf(0.1, _owner.get("phase2_pelvis_x_freq_hz") as float)
	var w: float = TAU * freq_hz
	var k: float = m_eff * w * w
	var zeta: float = maxf(0.0, _owner.get("phase2_pelvis_x_zeta") as float)
	var d: float = 2.0 * m_eff * w * zeta
	var Fx: float = (k * err) - (d * (vx - desired_vx))
	var force_mult: float = maxf(0.0, _owner.get("phase2_pelvis_x_force_mult") as float)
	var Fx_max: float = (_refs.total_mass * _refs.g) * force_mult
	var Fx_cmd: float = clampf(Fx, -Fx_max, Fx_max)
	_owner.set("_dbg_pelvis_target_x", target_x)
	_owner.set("_dbg_Fx_pre", Fx)
	_owner.set("_dbg_Fx_cmd", Fx_cmd)
	_owner.set("_dbg_Fx_max", maxf(1.0, Fx_max))
	rb.apply_central_force(Vector2(Fx_cmd * spawn01, 0.0))

func _foot_best_x(o: Node, is_front: bool) -> float:
	var mod = o.get("_foot_plant_mod")
	if mod == null:
		return NAN
	return (mod as GCFootPlant).foot_best_x(is_front)

func _ground_y() -> float:
	var vs = _owner.get("_vertical_support_mod")
	if vs != null:
		return (vs as GCVerticalSupport).ground_y()
	var rb: RigidBody2D = _refs.rb_pelvis
	return rb.global_position.y if rb != null else 0.0

func get_slot_targets_x() -> Vector2:
	# 5R2-C compatibility wrapper:
	# prefer stance planner authoritative slot service, keep legacy local computation as fallback only.
	var sp_mod = _owner.get("_stance_planner_mod")
	if sp_mod != null and sp_mod.has_method("get_authoritative_slot_targets_x"):
		var slots_auth: Vector2 = sp_mod.get_authoritative_slot_targets_x()
		if is_finite(slots_auth.x) and is_finite(slots_auth.y):
			return slots_auth

	# Legacy local slot geometry fallback (temporary during migration).
	# Single source of slot geometry (Brief 3 rebuilt):
	# 1) derive asymmetry template from authored posture targets + calibrated rig geometry
	# 2) normalize to configured stance width
	# 3) enforce role-side sanity + minimum separation
	# Returns Vector2(front_slot_x, rear_slot_x) in world X.
	var cx: float = _owner.get("_stance_center_x") as float
	if not is_finite(cx):
		cx = _refs.rb_pelvis.global_position.x if _refs.rb_pelvis != null else 0.0

	var stance_alpha: float = clampf(_owner.get("_stance_alpha") as float, 0.0, 1.0)
	var crouch01: float = 1.0 - stance_alpha
	var w_stand: float = _owner.get("stance_w_stand") as float
	var w_crouch: float = _owner.get("stance_w_crouch") as float
	var full_width: float = lerpf(w_stand, w_crouch, crouch01)
	full_width = maxf(1.0, full_width)

	var ax: float = float(_owner.get("_neutral_axis_sign"))
	if ax == 0.0:
		ax = signf(_owner.get("facing_sign") as float)
	if ax == 0.0:
		ax = 1.0

	# 5R2-C anti-leak: slot geometry fallback must not depend on Phase7 planner role state.
	var lead_is_front: bool = _owner.get("_lead_is_front")

	var front_s: float = NAN
	var rear_s: float = NAN

	# Primary source: posture-derived template (authored, continuous, deterministic).
	if _owner._posture_mod != null:
		var tpl: Vector2 = _owner._posture_mod.get_slot_template_axis_offsets(lead_is_front)
		front_s = tpl.x
		rear_s = tpl.y

	# Fallback: old symmetric layout only if posture template is unavailable.
	if not is_finite(front_s) or not is_finite(rear_s):
		var half_fb: float = full_width * 0.5
		front_s = half_fb if lead_is_front else -half_fb
		rear_s = -half_fb if lead_is_front else half_fb
		_owner.set("_dbg_slot_tpl_fallback", 1)
	else:
		_owner.set("_dbg_slot_tpl_fallback", 0)

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

	# Role-side sanity: front/rear must stay on expected sides in axis space.
	var front_expected_sign: float = 1.0 if lead_is_front else -1.0
	var rear_expected_sign: float = -front_expected_sign
	var sign_bad: bool = (front_s * front_expected_sign <= 0.0) or (rear_s * rear_expected_sign <= 0.0)
	if sign_bad:
		var half_fix: float = full_width * 0.5
		front_s = half_fix if lead_is_front else -half_fix
		rear_s = -half_fix if lead_is_front else half_fix
		_owner.set("_dbg_slot_tpl_signfix", 1)
	else:
		_owner.set("_dbg_slot_tpl_signfix", 0)

	# Minimum slot separation invariant (planner-level anti-collapse).
	var min_sep_cfg: float = maxf(0.0, _owner.get("phase7_min_foot_separation_px") as float)
	var min_sep: float = maxf(min_sep_cfg, full_width * 0.5)
	if absf(front_s - rear_s) < min_sep:
		var half_sep: float = 0.5 * min_sep
		front_s = front_expected_sign * half_sep
		rear_s = rear_expected_sign * half_sep
		_owner.set("_dbg_slot_tpl_sepfix", 1)
	else:
		_owner.set("_dbg_slot_tpl_sepfix", 0)

	var front_x: float = cx + (front_s * ax)
	var rear_x: float = cx + (rear_s * ax)

	_owner.set("_dbg_slot_front_x", front_x)
	_owner.set("_dbg_slot_rear_x", rear_x)
	_owner.set("_dbg_slot_sep_x", absf(front_x - rear_x))

	return Vector2(front_x, rear_x)

func slot_target_x_for_foot(is_front: bool) -> float:
	var slots: Vector2 = get_slot_targets_x()
	return slots.x if is_front else slots.y

func _neutral_target_x_for_foot(is_front: bool) -> float:
	# Backward-compatible wrapper; callers should migrate to slot_target_x_for_foot().
	return slot_target_x_for_foot(is_front)

func _phase7_pick_initiator_for_dir(dir_world: int) -> bool:
	# Real intended semantics:
	# forward tap -> trail moves first
	# backward tap -> lead moves first
	var lead_is_front: bool = _owner.get("_lead_is_front")
	if dir_world > 0:
		return not lead_is_front
	if dir_world < 0:
		return lead_is_front
	return _owner.get("_phase7_seq_active_is_front") as bool

func _phase7_pick_walk_next_foot(dir_world: int, last_done_is_front: bool) -> bool:
	# Prefer live role-based pick. If live role has not flipped yet this frame, fall back to alternating.
	var want: bool = _phase7_pick_initiator_for_dir(dir_world)
	if want == last_done_is_front:
		want = not last_done_is_front
	return want

func _phase7_compute_walk_target_x(active_is_front: bool, dir_world: int, cmd_move_x_norm: float, pelvis_x: float) -> float:
	var min_foot_separation_px: float = _owner.get("phase7_min_foot_separation_px") as float
	var under_body_step_px: float = _owner.get("phase7_under_body_step_px") as float
	var max_foot_reach_px: float = _owner.get("phase7_max_foot_reach_px") as float
	var speed_px_s: float = _owner.get("phase2_speed_px_s") as float
	var stride_time_sec: float = _owner.get("phase7_stride_time_sec") as float

	var run_ramp_sec: float = _owner.get("phase7_run_ramp_sec") as float
	var run_stride_px_mult: float = _owner.get("phase7_run_stride_px_mult") as float
	var hold_t_walk: float = _owner.get("_phase7_hold_t") as float
	var run01: float = clampf(hold_t_walk / maxf(0.001, run_ramp_sec), 0.0, 1.0)

	var stance_alpha: float = clampf(_owner.get("_stance_alpha") as float, 0.0, 1.0)
	var w_stand: float = _owner.get("stance_w_stand") as float
	var w_crouch: float = _owner.get("stance_w_crouch") as float
	var full_width: float = lerpf(w_stand, w_crouch, 1.0 - stance_alpha)

	var min_stride: float = maxf(min_foot_separation_px, under_body_step_px)
	min_stride = maxf(min_stride, full_width)

	var stride_px: float = clampf(
		absf(cmd_move_x_norm) * speed_px_s * stride_time_sec * lerpf(1.0, maxf(1.0, run_stride_px_mult), run01),
		min_stride,
		max_foot_reach_px
	)
	_owner.set("_phase7_stride_px", stride_px)

	var support_foot_is_front: bool = not active_is_front
	var support_x: float = _foot_best_x(_owner, support_foot_is_front)
	if not is_finite(support_x):
		support_x = pelvis_x

	# Early hold behavior: cross the other foot realistically (tap-like semantics).
	var other_x: float = _foot_best_x(_owner, support_foot_is_front)
	if not is_finite(other_x):
		other_x = support_x
	var cross_allow_px: float = maxf(0.0, _owner.get("phase7_cross_allow_px") as float)
	var cross_target_x: float = other_x + float(dir_world) * cross_allow_px

	# Longer hold behavior: blend toward stride/run target.
	var stride_target_x: float = support_x + float(dir_world) * stride_px

	return lerpf(cross_target_x, stride_target_x, run01)

func _phase7_pick_stop_correction_foot() -> bool:
	var nxF: float = _neutral_target_x_for_foot(true)
	var nxR: float = _neutral_target_x_for_foot(false)
	var fx: float = _foot_best_x(_owner, true)
	var rx: float = _foot_best_x(_owner, false)
	var errF: float = absf(fx - nxF) if is_finite(fx) else 0.0
	var errR: float = absf(rx - nxR) if is_finite(rx) else 0.0
	return errF >= errR

func _phase7_clamp_target_x(active_is_front: bool, dir_world: int, target_x: float) -> float:
	var cx: float = _refs.rb_pelvis.global_position.x if _refs.rb_pelvis != null else target_x
	var max_reach: float = _owner.get("phase7_max_foot_reach_px") as float
	var out: float = clampf(target_x, cx - max_reach, cx + max_reach)
	if out != target_x:
		_owner.set("_phase7_clamp_reach_count", (_owner.get("_phase7_clamp_reach_count") as int) + 1)

	var loc_state: int = _owner.get("_phase7_state")
	var other_live_x: float = _foot_best_x(_owner, not active_is_front)
	var other_slot_x: float = slot_target_x_for_foot(not active_is_front)

	# WALK semantics: allow/require crossing target on the commanded side of the other foot.
	# Do NOT apply the old anti-collapse min-separation clamp here (that blocked realistic crossing).
	if loc_state == GCTypes.Phase7LocState.WALK_SEQ and dir_world != 0 and is_finite(other_live_x):
		var cross_allow_px: float = maxf(0.0, _owner.get("phase7_cross_allow_px") as float)
		if dir_world > 0:
			var min_walk_x: float = other_live_x + cross_allow_px
			if out < min_walk_x:
				out = min_walk_x
				_owner.set("_phase7_clamp_cross_count", (_owner.get("_phase7_clamp_cross_count") as int) + 1)
		else:
			var max_walk_x: float = other_live_x - cross_allow_px
			if out > max_walk_x:
				out = max_walk_x
				_owner.set("_phase7_clamp_cross_count", (_owner.get("_phase7_clamp_cross_count") as int) + 1)
		return out

	# TAP/STOP/correction semantics: preserve anti-collapse slot separation.
	var other_x: float = other_slot_x if is_finite(other_slot_x) else other_live_x
	if is_finite(other_x):
		var min_sep: float = maxf(0.0, _owner.get("phase7_min_foot_separation_px") as float)
		var dx: float = out - other_x
		if absf(dx) < min_sep:
			var push_sign: float = signf(dx)
			if push_sign == 0.0:
				push_sign = 1.0 if (out >= other_x) else -1.0
			out = other_x + (min_sep * push_sign)
			_owner.set("_phase7_clamp_cross_count", (_owner.get("_phase7_clamp_cross_count") as int) + 1)
	return out
