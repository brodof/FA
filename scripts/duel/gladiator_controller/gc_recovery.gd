# res://scripts/duel/gladiator_controller/gc_recovery.gd
# Phase 6: per-foot friction (slide material when SWING/sliding). Refresh + apply. Uses refs for feet; owner for materials.
class_name GCRecovery
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

func capture_rest_snapshot(set_captured_flag: bool) -> void:
	var o := _owner
	if o == null:
		return
	# Exact controller logic (relocated):
	if o._rb_pelvis == null:
		return

	# Knee extension reference must NOT be redefined by rest recapture.
	# Capture once (seed time) and then keep it forever.
	var capture_knee_zero: bool = (not o._knee_zero_valid)

	# Legs
	if o._rb_thigh_front != null:
		o.rest_pelvis_thigh_F = wrapf(o._rb_thigh_front.global_rotation - o._rb_pelvis.global_rotation, -PI, PI)
	if o._rb_shin_front != null and o._rb_thigh_front != null:
		o.rest_thigh_shin_F = wrapf(o._rb_shin_front.global_rotation - o._rb_thigh_front.global_rotation, -PI, PI)
		if capture_knee_zero:
			o._knee_zero_F = o.rest_thigh_shin_F
	# Debug: record nominal thigh<->shin center distance once (detects "knee separation" after impact)
	if o._dbg_rest_center_dist_knee_F < 0.0 and o._rb_thigh_front != null and o._rb_shin_front != null:
		o._dbg_rest_center_dist_knee_F = o._rb_thigh_front.global_position.distance_to(o._rb_shin_front.global_position)
	if o._rb_foot_front != null and o._rb_shin_front != null:
		o.rest_shin_foot_F = wrapf(o._rb_foot_front.global_rotation - o._rb_shin_front.global_rotation, -PI, PI)

	if o._rb_thigh_rear != null:
		o.rest_pelvis_thigh_R = wrapf(o._rb_thigh_rear.global_rotation - o._rb_pelvis.global_rotation, -PI, PI)
	if o._rb_shin_rear != null and o._rb_thigh_rear != null:
		o.rest_thigh_shin_R = wrapf(o._rb_shin_rear.global_rotation - o._rb_thigh_rear.global_rotation, -PI, PI)
		if capture_knee_zero:
			o._knee_zero_R = o.rest_thigh_shin_R
	# Debug: record nominal thigh<->shin center distance once (detects "knee separation" after impact)
	if o._dbg_rest_center_dist_knee_R < 0.0 and o._rb_thigh_rear != null and o._rb_shin_rear != null:
		o._dbg_rest_center_dist_knee_R = o._rb_thigh_rear.global_position.distance_to(o._rb_shin_rear.global_position)
	if o._rb_foot_rear != null and o._rb_shin_rear != null:
		o.rest_shin_foot_R = wrapf(o._rb_foot_rear.global_rotation - o._rb_shin_rear.global_rotation, -PI, PI)

	# Spine
	if o._rb_torso != null:
		o.rest_pelvis_torso = wrapf(o._rb_torso.global_rotation - o._rb_pelvis.global_rotation, -PI, PI)
	if o._rb_head != null and o._rb_torso != null:
		o.rest_torso_head = wrapf(o._rb_head.global_rotation - o._rb_torso.global_rotation, -PI, PI)

	if capture_knee_zero:
		o._knee_zero_valid = (o._rb_shin_front != null and o._rb_thigh_front != null and o._rb_shin_rear != null and o._rb_thigh_rear != null)

	if set_captured_flag:
		o._rest_captured = true
		o._rest_capture_time_since = 0.0

func overwrite_rest_snapshot() -> void:
	capture_rest_snapshot(false)
	if _owner != null:
		_owner.set("_rest_capture_time_since", 0.0)

func update_rest_capture_gate(grounded_eff: bool, _front_g: bool, _rear_g: bool, stabF: float, stabR: float, dt: float) -> void:
	var o := _owner
	if o == null:
		return
	o._rest_capture_elapsed += dt
	if o._rest_captured:
		o._rest_capture_time_since += dt
		return
	o._rest_capture_time_since = 0.0
	if not grounded_eff or o._rb_pelvis == null:
		o._rest_capture_timer = 0.0
		return
	var support_is_front: bool = o.support_is_front
	var sup_stab: float = stabF if support_is_front else stabR
	var pel_deg_abs: float = absf(rad_to_deg(wrapf(o._rb_pelvis.global_rotation, -PI, PI)))
	var pel_w: float = absf(o._rb_pelvis.angular_velocity)
	var tor_w: float = absf(o._rb_torso.angular_velocity) if o._rb_torso != null else 0.0
	var ok: bool = true
	ok = ok and (sup_stab >= o.rest_capture_min_stability)
	ok = ok and (pel_deg_abs <= o.rest_capture_max_pelvis_deg)
	ok = ok and (pel_w <= o.rest_capture_max_angvel)
	ok = ok and (tor_w <= o.rest_capture_max_angvel)
	if ok:
		o._rest_capture_timer += dt
	else:
		o._rest_capture_timer = 0.0
	if o._rest_capture_timer >= o.rest_capture_min_sec:
		capture_rest_snapshot(true)
		return
	if o._rest_capture_elapsed >= o.rest_capture_timeout_sec:
		o._rest_captured = true
		o._rest_capture_time_since = 0.0

func update_rest_recapture_gate(front_g: bool, rear_g: bool, stabF: float, stabR: float, dt: float) -> void:
	var o := _owner
	if o == null:
		return
	# Exact controller logic (relocated):
	if not o.rest_recapture_enable:
		return
	if not o._rest_seeded or not o._rest_captured:
		return
	if o._drag_active:
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return
	if o._impact_timer > 0.0:
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return

	o._rest_recapture_time_since += dt
	if o._rest_recapture_time_since < o.rest_recapture_min_interval_sec:
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return

	# Need both feet grounded + truly planted (pins-connected truth).
	if not (front_g and rear_g):
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return
	if not (o._plant_front_active and o._plant_rear_active):
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return
	if stabF < o.rest_recapture_min_stability or stabR < o.rest_recapture_min_stability:
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return

	# Low angular motion gate
	var pel_w: float = absf(o._rb_pelvis.angular_velocity) if o._rb_pelvis != null else 0.0
	var tor_w: float = absf(o._rb_torso.angular_velocity) if o._rb_torso != null else 0.0
	var ff_w: float = absf(o._rb_foot_front.angular_velocity) if o._rb_foot_front != null else 0.0
	var fr_w: float = absf(o._rb_foot_rear.angular_velocity) if o._rb_foot_rear != null else 0.0
	var max_w: float = maxf(maxf(pel_w, tor_w), maxf(ff_w, fr_w))
	if max_w > o.rest_recapture_max_angvel:
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return

	# Contact chaos gate: points/normals must be stable across frames
	if o._foot_sensor_front == null or o._foot_sensor_rear == null:
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return
	if not (o._foot_sensor_front.grounded and o._foot_sensor_rear.grounded):
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false
		return

	var cpF: Vector2 = o._foot_sensor_front.contact_point_w
	var cpR: Vector2 = o._foot_sensor_rear.contact_point_w
	var nF: Vector2 = o._foot_sensor_front.contact_normal_w.normalized()
	var nR: Vector2 = o._foot_sensor_rear.contact_normal_w.normalized()

	if not o._rest_prev_valid:
		o._rest_prev_valid = true
		o._rest_prev_cp_front = cpF
		o._rest_prev_cp_rear = cpR
		o._rest_prev_n_front = nF
		o._rest_prev_n_rear = nR
		o._rest_recapture_timer = 0.0
		return

	var dpos: float = maxf(cpF.distance_to(o._rest_prev_cp_front), cpR.distance_to(o._rest_prev_cp_rear))
	var dnorm: float = maxf((nF - o._rest_prev_n_front).length(), (nR - o._rest_prev_n_rear).length())

	o._rest_prev_cp_front = cpF
	o._rest_prev_cp_rear = cpR
	o._rest_prev_n_front = nF
	o._rest_prev_n_rear = nR

	if dpos > o.rest_recapture_max_contact_dpos_px or dnorm > o.rest_recapture_max_contact_dnorm:
		o._rest_recapture_timer = 0.0
		return

	o._rest_recapture_timer += dt
	if o._rest_recapture_timer >= o.rest_recapture_min_stable_sec:
		overwrite_rest_snapshot()
		o._rest_recapture_time_since = 0.0
		o._rest_recapture_timer = 0.0
		o._rest_prev_valid = false

func refresh_foot_materials() -> void:
	var pm_base: PhysicsMaterial = _owner.get("_pm_feet_base")
	var pm_slide: PhysicsMaterial = _owner.get("_pm_feet_slide")
	if pm_base == null:
		pm_base = PhysicsMaterial.new()
		_owner.set("_pm_feet_base", pm_base)
	if pm_slide == null:
		pm_slide = PhysicsMaterial.new()
		_owner.set("_pm_feet_slide", pm_slide)

	var base_f: float = maxf(0.0, _owner.get("feet_friction") as float)
	var slide_f: float = maxf(0.0, _owner.get("phase6_slide_friction") as float)
	var b: float = _owner.get("feet_bounce") as float

	var pm_base_fric: float = _owner.get("_pm_feet_base_fric") as float
	var pm_base_bounce: float = _owner.get("_pm_feet_base_bounce") as float
	var pm_slide_fric: float = _owner.get("_pm_feet_slide_fric") as float
	var pm_slide_bounce: float = _owner.get("_pm_feet_slide_bounce") as float

	if pm_base_fric != base_f:
		pm_base.friction = base_f
		_owner.set("_pm_feet_base_fric", base_f)
	if pm_base_bounce != b:
		pm_base.bounce = b
		_owner.set("_pm_feet_base_bounce", b)
	if pm_slide_fric != slide_f:
		pm_slide.friction = slide_f
		_owner.set("_pm_feet_slide_fric", slide_f)
	if pm_slide_bounce != b:
		pm_slide.bounce = b
		_owner.set("_pm_feet_slide_bounce", b)

func apply_foot_friction(front_slide: bool, rear_slide: bool) -> void:
	refresh_foot_materials()
	var pm_base: PhysicsMaterial = _owner.get("_pm_feet_base")
	var pm_slide: PhysicsMaterial = _owner.get("_pm_feet_slide")
	if _refs.rb_foot_front != null:
		var want_f: PhysicsMaterial = pm_slide if front_slide else pm_base
		if _refs.rb_foot_front.physics_material_override != want_f:
			_refs.rb_foot_front.physics_material_override = want_f
	if _refs.rb_foot_rear != null:
		var want_r: PhysicsMaterial = pm_slide if rear_slide else pm_base
		if _refs.rb_foot_rear.physics_material_override != want_r:
			_refs.rb_foot_rear.physics_material_override = want_r
