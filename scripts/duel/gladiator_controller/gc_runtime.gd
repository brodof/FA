# res://scripts/duel/gladiator_controller/gc_runtime.gd
# Godot 4.5.1
# Orchestrator for per-frame controller tick (introduced as wiring-only in Pass B1).

extends RefCounted
class_name GCRuntime

const _preload_gc_pipeline := preload("res://scripts/duel/gladiator_controller/gc_pipeline.gd")

var _host: FA_GladiatorPhysicsController = null
var _refs: GCRefs = null
var _pipeline: RefCounted = null

func setup(host: FA_GladiatorPhysicsController, refs: GCRefs) -> void:
	_host = host
	_refs = refs
	if _pipeline == null:
		_pipeline = _preload_gc_pipeline.new()
		_pipeline.setup(host, refs)

func tick(dt: float) -> void:
	if _host == null:
		return

	_tick_preamble(dt)

	var pre: Array = _tick_frame_pre(dt)
	var spawn01: float = float(pre[0])
	var dragging: bool = bool(pre[1])
	var drag_ended: bool = bool(pre[2])

	if _pipeline != null:
		_pipeline.tick(dt, spawn01, dragging, drag_ended)

func _tick_preamble(dt: float) -> void:
	var o := _host
	if o == null:
		return
	o._plant_tick_id += 1

	o.facing_sign = (1 if o.facing_sign >= 0 else -1)
	o._facing_mod.tick(dt)
	o._rb_pelvis_opponent = o._refs.opponent_pelvis
	o.facing_sign = (1 if o.facing_sign >= 0 else -1)

	o.knee_flex_sign = (1 if o.knee_flex_sign >= 0 else -1)

	var ksF: int = o.knee_flex_sign_front
	if ksF == 0:
		ksF = o.knee_flex_sign
	ksF = (1 if ksF >= 0 else -1)

	var ksR: int = o.knee_flex_sign_rear
	if ksR == 0:
		ksR = o.knee_flex_sign
	ksR = (1 if ksR >= 0 else -1)

	o._knee_flex_sign_F = ksF
	o._knee_flex_sign_R = ksR

func _tick_frame_pre(dt: float) -> Array:
	var o := _host
	if o == null:
		return [1.0, false, false]
	# Returns [spawn01: float, dragging: bool, drag_ended: bool]
	o._t += dt

	# --- Per-frame debug latch reset (prevents stale forces showing up in logs) ---
	o._dbg_target_y = NAN
	o._dbg_err_y = 0.0
	o._dbg_vy_for_pd = 0.0
	o._dbg_Fy_pre = 0.0
	o._dbg_Fy_cmd = 0.0
	o._dbg_Fy_max = 1.0
	o._dbg_pelvis_target_x = NAN
	o._dbg_Fx_pre = 0.0
	o._dbg_Fx_cmd = 0.0
	o._dbg_Fx_max = 1.0
	o._dbg_vsupport_called = false
	o._dbg_px_called = false

	o._dbg_front_slide = false
	o._dbg_rear_slide = false

	# Phase 5: reset solver health flags for this tick
	o._phase5_upright_saturated = false
	o._phase5_limit_saturated = false
	o._phase5_support_saturated = false

	# Phase 5: reset per-frame debug latches
	o._dbg_phase5_upr_sat = 0
	o._dbg_phase5_lim_sat = 0
	o._dbg_phase5_sup_sat = 0
	o._dbg_phase5_spine_on = 0

	# Prevent stale SPINE torque showing up when SPINE isn't evaluated this frame.
	if o.debug_enable and o._dbg_lim.has("SPINE"):
		o._dbg_lim.erase("SPINE")

	o._input_mod.tick(dt)

	# Normalize world move command + derive facing-relative forward/back (for planner/debug).
	var dz_cmd: float = o.input_deadzone if o.input_deadzone > 0.0 else o.cmd_move_deadzone
	var cmdx_norm: float = clampf(o.cmd_move_x, -1.0, 1.0)
	if absf(cmdx_norm) < dz_cmd:
		cmdx_norm = 0.0
	o.cmd_move_x = cmdx_norm

	var cmd_facing: float = cmdx_norm * float(o.facing_sign)
	o.cmd_forward01 = maxf(0.0, cmd_facing)
	o.cmd_backward01 = maxf(0.0, -cmd_facing)

	var dragging := o._debug_apply_mouse_drag(dt)
	var drag_ended := (o._drag_prev_applied and not dragging)
	o._drag_prev_applied = dragging
	if dragging:
		# Treat like a big disturbance so planting doesn't fight you
		o._impact_timer = maxf(o._impact_timer, o.impact_ignore_d_sec)

	# Stance command: smoothly move stance_height01 (alpha) with full travel in ~stance_shift_time_sec.
	# +cmd_stance_y raises alpha toward 1 (upright). -cmd_stance_y lowers alpha toward 0 (crouch).
	o._stance_alpha_prev = o._stance_alpha
	var sy := o.cmd_stance_y
	if absf(sy) < o.cmd_stance_deadzone:
		sy = 0.0
	sy = clampf(sy, -1.0, 1.0)

	var a_prev: float = o.stance_height01
	if sy != 0.0 and o.stance_shift_time_sec > 0.001:
		var a_next: float = clampf(a_prev + (sy * dt / o.stance_shift_time_sec), 0.0, 1.0)

		# If we hit an end-stop, treat this as "no active shift intent"
		# (prevents extra downforce / extra posture gain while already clamped).
		if absf(a_next - a_prev) <= 0.000001:
			sy = 0.0
		else:
			o.stance_height01 = a_next

	o._cmd_stance_y_eff = sy

	# Spawn-only BRACE: prepared landing crouch cap (don't start tall/locked).
	if o._spawn_brace_active:
		o.stance_height01 = minf(o.stance_height01, o.spawn_brace_stance_height01)

	# Stance alpha is the single runtime stance signal used everywhere (leg length, templates, toe-only overrides).
	o._stance_alpha = o.stance_height01

	# Stance-change must be detectable even when alpha changes smoothly (small per-frame deltas).
	# Treat active stance intent as "changing".
	var _stance_changing: bool = (sy != 0.0) or (absf(o._stance_alpha - o._stance_alpha_prev) >= o.phase6_stance_change_eps)
	o._phase6_stance_changing = _stance_changing

	# Spawn ramp
	var spawn01: float = 1.0
	if o.spawn_ramp_sec > 0.001:
		spawn01 = clampf(o._t / o.spawn_ramp_sec, 0.0, 1.0)

	if o._spawn_brace_active:
		o._spawn_brace_elapsed += dt

	return [spawn01, dragging, drag_ended]
