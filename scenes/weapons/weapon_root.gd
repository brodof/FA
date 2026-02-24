# res://scripts/duel/weapon_root.gd
extends RigidBody2D
class_name WeaponRoot

signal contact_impulse(J: Vector2, world_pos: Vector2, world_n: Vector2)
signal reaction_impulse(J: Vector2)
signal ground_contact(velocity: Vector2, normal: Vector2, position: Vector2)

# -----------------------------------------------------------------------------
# Servo tuning (impulse constraint; stable with contacts)
# -----------------------------------------------------------------------------
@export_group("One-Hand Mode")
@export var one_hand_beta: float = 0.18
@export var one_hand_damp: float = 0.78
@export var one_hand_ang_beta: float = 0.12
@export var one_hand_ang_damp: float = 0.75
@export var one_hand_max_force: float = 6000.0
@export var one_hand_max_torque: float = 4000.0

@export_group("Two-Hand Mode")
@export var two_hand_beta: float = 0.28
@export var two_hand_damp: float = 0.92
@export var two_hand_ang_beta: float = 0.18
@export var two_hand_ang_damp: float = 0.88
@export var two_hand_max_force: float = 14000.0
@export var two_hand_max_torque: float = 10000.0

@export_group("Impulse Transfer")
@export var one_hand_transmission: float = 0.12
@export var two_hand_transmission: float = 0.55
@export var reaction_impulse_max: float = 1200.0

# -----------------------------------------------------------------------------
# Ground feel
# -----------------------------------------------------------------------------
@export_group("One-Hand Ground Feel")
@export var bounce_one_hand: float = 0.45
@export var friction_one_hand: float = 0.30

@export_group("Two-Hand Ground Feel")
@export var bounce_two_hand: float = 0.02
@export var friction_two_hand: float = 1.50
@export var two_hand_ground_lin_damp: float = 12.0
@export var two_hand_ground_ang_damp: float = 15.0

@export_group("Pogo Effect (Two-Hand)")
@export var pogo_efficiency: float = 0.75
@export var pogo_velocity_threshold: float = 100.0
@export var pogo_max_impulse: float = 600.0
@export var pogo_stick_time: float = 0.08

# -----------------------------------------------------------------------------
# Safety / stability clamps
# -----------------------------------------------------------------------------
@export_group("Safety Limits")
@export var max_linear_speed: float = 1400.0
@export var max_angular_speed: float = 25.0
@export var max_target_step_per_frame: float = 160.0

# -----------------------------------------------------------------------------
# Contact->character shove (for satisfying clashes)
# -----------------------------------------------------------------------------
@export_group("Contact Shove")
@export var shove_gain_one_hand: float = 0.15
@export var shove_gain_two_hand: float = 0.60
@export var shove_max: float = 1000.0

# -----------------------------------------------------------------------------
# Internal drive state
# -----------------------------------------------------------------------------
var _two_hand: bool = false
var _drive_enabled: bool = false

var _butt_target_w: Vector2 = Vector2.ZERO
var _grip_target_w: Vector2 = Vector2.ZERO
var _angle_target: float = 0.0

var _targets_inited: bool = false
var _butt_target_prev: Vector2 = Vector2.ZERO
var _grip_target_prev: Vector2 = Vector2.ZERO

# Local attachment points. Convention:
# - Node origin = ATTACH (blade/guard attach)
# - +X points ATTACH -> BUTT (toward hands)
var butt_local: Vector2 = Vector2.ZERO
var grip_local: Vector2 = Vector2(-28.0, 0.0)

# Profile for shove strength (set by character)
var _two_hand_profile: bool = false
var _weapon_mass_for_shove: float = 2.2

# Ground contact state
var _is_ground_contact: bool = false
var _ground_contact_normal: Vector2 = Vector2.UP
var _ground_contact_point: Vector2 = Vector2.ZERO
var _pogo_stick_timer: float = 0.0
var _last_pogo_impulse: Vector2 = Vector2.ZERO

# Visual nodes
@export var blade_visual_path: NodePath = NodePath("BladeVisual")
@export var handle_visual_path: NodePath = NodePath("HandleVisual")
var _blade_visual: Polygon2D = null
var _handle_visual: Polygon2D = null

# Collision nodes
var _convex_nodes: Array[CollisionPolygon2D] = []

func _ready() -> void:
	# Critical: we rely on _integrate_forces.
	custom_integrator = false

	continuous_cd = RigidBody2D.CCD_MODE_CAST_SHAPE
	contact_monitor = true
	max_contacts_reported = 16
	can_sleep = false

	linear_damp = 1.4
	angular_damp = 1.8

	_ensure_visual_nodes()
	if _convex_nodes.is_empty():
		_make_fallback_box()
	_set_fallback_visuals()
	_apply_mode_material()

func set_drive_enabled(enabled: bool) -> void:
	_drive_enabled = enabled
	if not _drive_enabled:
		_angle_target = global_rotation
		_butt_target_w = global_transform * butt_local
		if _two_hand:
			_grip_target_w = global_transform * grip_local

func set_collision_from_character(ch: CollisionObject2D) -> void:
	if ch == null:
		return
	collision_layer = ch.collision_layer
	collision_mask = ch.collision_mask

func set_physics_properties(new_mass: float, new_inertia: float) -> void:
	mass = maxf(0.05, new_mass)
	if new_inertia > 0.0:
		inertia = new_inertia

func set_shove_profile(is_two_hand: bool, w_mass: float) -> void:
	_two_hand_profile = is_two_hand
	_weapon_mass_for_shove = maxf(0.1, w_mass)

func set_attachment_points_local(butt: Vector2, grip: Vector2) -> void:
	butt_local = butt
	grip_local = grip

func set_one_hand_target(butt_world: Vector2, angle_target: float, _has_input: bool, _input_dir: Vector2) -> void:
	_two_hand = false
	_butt_target_w = butt_world
	_angle_target = angle_target

func set_two_hand_targets(butt_world: Vector2, grip_world: Vector2, angle_target: float, _has_input: bool, _input_dir: Vector2) -> void:
	_two_hand = true
	_butt_target_w = butt_world
	_grip_target_w = grip_world
	_angle_target = angle_target

func _integrate_forces(state: PhysicsDirectBodyState2D) -> void:
	var dt: float = maxf(0.000001, state.step)

	_apply_mode_material()
	_clamp_body_speeds(state)
	_detect_ground_contacts(state)

	if _pogo_stick_timer > 0.0:
		_pogo_stick_timer = maxf(0.0, _pogo_stick_timer - dt)

	if not _drive_enabled:
		_emit_contact_impulses(state)
		return

	_limit_target_step()
	_guard_targets_against_contacts(state, dt)

	if _is_ground_contact:
		if _two_hand:
			_apply_two_hand_ground_physics(state, dt)
		else:
			_apply_one_hand_ground_physics(state)

	# Servo constraint (impulses)
	var maxF: float = two_hand_max_force if _two_hand else one_hand_max_force
	var maxT: float = two_hand_max_torque if _two_hand else one_hand_max_torque
	var beta: float = two_hand_beta if _two_hand else one_hand_beta
	var damp: float = two_hand_damp if _two_hand else one_hand_damp
	var ang_beta: float = two_hand_ang_beta if _two_hand else one_hand_ang_beta
	var ang_damp: float = two_hand_ang_damp if _two_hand else one_hand_ang_damp
	var transmission: float = two_hand_transmission if _two_hand else one_hand_transmission

	var max_impulse: float = maxf(0.0, maxF) * dt
	var max_torque_imp: float = maxf(0.0, maxT) * dt

	var J_sum: Vector2 = Vector2.ZERO
	var iters: int = 3 if _two_hand else 2

	for _k in range(iters):
		J_sum += _solve_point_impulse(state, butt_local, _butt_target_w, beta, damp, max_impulse, dt)
		if _two_hand:
			J_sum += _solve_point_impulse(state, grip_local, _grip_target_w, beta, damp, max_impulse, dt)
		_solve_angle_impulse(state, _angle_target, ang_beta, ang_damp, max_torque_imp, dt)

	# Reaction impulse to character (what the hands/body "feel" from weapon constraint)
	if J_sum.length_squared() > 0.000001 and transmission > 0.0:
		var J_char: Vector2 = (-J_sum) * transmission
		J_char = J_char.limit_length(reaction_impulse_max)
		emit_signal("reaction_impulse", J_char)

	_emit_contact_impulses(state)

func _detect_ground_contacts(state: PhysicsDirectBodyState2D) -> void:
	_is_ground_contact = false
	_ground_contact_normal = Vector2.UP
	_ground_contact_point = Vector2.ZERO

	var ccount: int = state.get_contact_count()
	for i in range(ccount):
		var n_local: Vector2 = state.get_contact_local_normal(i)
		if n_local.length_squared() < 0.0001:
			continue

		var n_world: Vector2 = state.transform.basis_xform(n_local).normalized()
		var p_world: Vector2 = state.transform * state.get_contact_local_position(i)

		# "Ground-like" contact = normal points up.
		if n_world.dot(Vector2.UP) > 0.5:
			_is_ground_contact = true
			_ground_contact_normal = n_world
			_ground_contact_point = p_world
			emit_signal("ground_contact", state.linear_velocity, n_world, p_world)
			break

func _apply_one_hand_ground_physics(state: PhysicsDirectBodyState2D) -> void:
	# One-hand: lively bounce. Add a small spin kick when slapping the floor.
	var v_down: float = state.linear_velocity.dot(Vector2.DOWN)
	if v_down > 60.0:
		var spin_dir: float = signf(state.linear_velocity.x)
		if absf(spin_dir) < 0.1:
			spin_dir = 1.0
		state.angular_velocity += spin_dir * 2.0

func _apply_two_hand_ground_physics(state: PhysicsDirectBodyState2D, dt: float) -> void:
	# Two-hand: heavy thud + pogo if you drive downward into ground.
	var v_down: float = state.linear_velocity.dot(Vector2.DOWN)

	if v_down > 80.0:
		var lin_alpha: float = clampf(two_hand_ground_lin_damp * dt, 0.0, 0.8)
		var ang_alpha: float = clampf(two_hand_ground_ang_damp * dt, 0.0, 0.8)
		state.linear_velocity = state.linear_velocity.lerp(Vector2.ZERO, lin_alpha)
		state.angular_velocity = lerpf(state.angular_velocity, 0.0, ang_alpha)

	if v_down > pogo_velocity_threshold and _pogo_stick_timer <= 0.0:
		var pogo_impulse: Vector2 = Vector2.UP * (v_down * pogo_efficiency)
		pogo_impulse = pogo_impulse.limit_length(pogo_max_impulse)
		_last_pogo_impulse = pogo_impulse
		_pogo_stick_timer = pogo_stick_time
		emit_signal("reaction_impulse", pogo_impulse)

func _apply_mode_material() -> void:
	var pm: PhysicsMaterial = physics_material_override as PhysicsMaterial
	if pm == null:
		pm = PhysicsMaterial.new()
		physics_material_override = pm

	if _two_hand:
		pm.bounce = bounce_two_hand
		pm.friction = friction_two_hand
		linear_damp = 2.5
		angular_damp = 3.5
	else:
		pm.bounce = bounce_one_hand
		pm.friction = friction_one_hand
		linear_damp = 1.2
		angular_damp = 1.5

func _clamp_body_speeds(state: PhysicsDirectBodyState2D) -> void:
	var v: Vector2 = state.linear_velocity
	var sp: float = v.length()
	if sp > max_linear_speed and sp > 0.0001:
		state.linear_velocity = v * (max_linear_speed / sp)

	var w: float = state.angular_velocity
	if absf(w) > max_angular_speed:
		state.angular_velocity = signf(w) * max_angular_speed

func _limit_target_step() -> void:
	if not _targets_inited:
		_butt_target_prev = _butt_target_w
		_grip_target_prev = _grip_target_w
		_targets_inited = true
		return

	var max_step: float = max_target_step_per_frame

	var db: Vector2 = _butt_target_w - _butt_target_prev
	if db.length() > max_step:
		_butt_target_w = _butt_target_prev + db.normalized() * max_step

	if _two_hand:
		var dg: Vector2 = _grip_target_w - _grip_target_prev
		if dg.length() > max_step:
			_grip_target_w = _grip_target_prev + dg.normalized() * max_step

	_butt_target_prev = _butt_target_w
	_grip_target_prev = _grip_target_w

func _guard_targets_against_contacts(state: PhysicsDirectBodyState2D, dt: float) -> void:
	var ccount: int = state.get_contact_count()
	if ccount <= 0:
		return

	var center: Vector2 = state.transform.origin
	var butt_w: Vector2 = _butt_target_w
	var grip_w: Vector2 = _grip_target_w

	var margin: float = 2.0
	var strength: float = 60.0

	for i in range(ccount):
		var n_local: Vector2 = state.get_contact_local_normal(i)
		if n_local.length_squared() < 0.0001:
			continue

		var n_world: Vector2 = state.transform.basis_xform(n_local).normalized()
		var p_world: Vector2 = state.transform * state.get_contact_local_position(i)

		# Ensure normal points "toward" our center (so projection pushes targets out).
		if (center - p_world).dot(n_world) < 0.0:
			n_world = -n_world

		butt_w = _project_point_out(butt_w, p_world, n_world, margin, strength, dt)
		if _two_hand:
			grip_w = _project_point_out(grip_w, p_world, n_world, margin, strength, dt)

	_butt_target_w = butt_w
	if _two_hand:
		_grip_target_w = grip_w

func _project_point_out(pt: Vector2, plane_p: Vector2, plane_n: Vector2, margin: float, strength: float, dt: float) -> Vector2:
	var d: float = (pt - plane_p).dot(plane_n)
	if d >= margin:
		return pt
	var push: float = margin - d
	var alpha: float = clampf(strength * dt, 0.0, 1.0)
	return pt + plane_n * (push * alpha)

func _solve_point_impulse(
	state: PhysicsDirectBodyState2D,
	local_point: Vector2,
	target_world: Vector2,
	beta: float,
	damp: float,
	max_impulse: float,
	dt: float
) -> Vector2:
	var origin: Vector2 = state.transform.origin
	var x_world: Vector2 = state.transform * local_point
	var r: Vector2 = x_world - origin # offset from body origin in GLOBAL coordinates

	# Velocity at point
	var v_point: Vector2 = state.linear_velocity + Vector2(-state.angular_velocity * r.y, state.angular_velocity * r.x)

	var err: Vector2 = target_world - x_world
	var inv_dt: float = 1.0 / maxf(0.000001, dt)

	# Desired point velocity
	var v_des: Vector2 = err * (beta * inv_dt)
	var v_err: Vector2 = (v_des - v_point) * damp

	var m: float = maxf(0.00001, mass)
	var I: float = maxf(0.00001, inertia)
	var inv_m: float = 1.0 / m
	var inv_I: float = 1.0 / I

	# Effective mass matrix inverse for point impulse
	var rpx: float = -r.y
	var rpy: float = r.x

	var k00: float = inv_m + inv_I * (rpx * rpx)
	var k01: float = inv_I * (rpx * rpy)
	var k10: float = k01
	var k11: float = inv_m + inv_I * (rpy * rpy)

	var det: float = k00 * k11 - k01 * k10
	if absf(det) < 0.0000001:
		return Vector2.ZERO

	var inv_det: float = 1.0 / det
	var Jx: float = (k11 * v_err.x - k01 * v_err.y) * inv_det
	var Jy: float = (-k10 * v_err.x + k00 * v_err.y) * inv_det

	var J: Vector2 = Vector2(Jx, Jy)
	if max_impulse > 0.0 and J.length() > max_impulse:
		J = J.normalized() * max_impulse

	# IMPORTANT: position is an OFFSET from body origin in GLOBAL coords.
	state.apply_impulse(J, r)
	return J

func _solve_angle_impulse(
	state: PhysicsDirectBodyState2D,
	target_angle: float,
	beta: float,
	damp: float,
	max_torque_impulse: float,
	dt: float
) -> void:
	var cur: float = state.transform.get_rotation()
	var ang_err: float = wrapf(target_angle - cur, -PI, PI)

	var inv_dt: float = 1.0 / maxf(0.000001, dt)
	var omega_des: float = ang_err * (beta * inv_dt)
	var omega_err: float = (omega_des - state.angular_velocity) * damp

	var I: float = maxf(0.00001, inertia)
	var torque_imp: float = omega_err * I

	if max_torque_impulse > 0.0:
		torque_imp = clampf(torque_imp, -max_torque_impulse, max_torque_impulse)

	state.apply_torque_impulse(torque_imp)

func _emit_contact_impulses(state: PhysicsDirectBodyState2D) -> void:
	var ccount: int = state.get_contact_count()
	if ccount <= 0:
		return

	var gain: float = shove_gain_two_hand if _two_hand_profile else shove_gain_one_hand
	var m_use: float = maxf(0.1, _weapon_mass_for_shove)
	var I_use: float = maxf(0.00001, inertia)

	for i in range(ccount):
		var n_local: Vector2 = state.get_contact_local_normal(i)
		if n_local.length_squared() < 0.0001:
			continue
		n_local = n_local.normalized()

		var v_local: Vector2 = state.get_contact_local_velocity_at_position(i)
		var v_n: float = v_local.dot(n_local)
		if v_n >= 0.0:
			continue

		var p_local: Vector2 = state.get_contact_local_position(i)

		# Approximate effective mass along normal at contact point.
		var r: Vector2 = p_local
		var r_perp: Vector2 = Vector2(-r.y, r.x)
		var rp_dot_n: float = r_perp.dot(n_local)
		var m_eff_n: float = 1.0 / ((1.0 / m_use) + (rp_dot_n * rp_dot_n) / I_use)

		var e: float = 0.0
		var pm: PhysicsMaterial = physics_material_override as PhysicsMaterial
		if pm != null:
			e = clampf(pm.bounce, 0.0, 1.0)

		var j_mag: float = (-(1.0 + e) * v_n) * m_eff_n * gain
		j_mag = minf(j_mag, shove_max)

		var J_local: Vector2 = n_local * j_mag
		var p_world: Vector2 = state.transform * p_local
		var n_world: Vector2 = state.transform.basis_xform(n_local).normalized()
		var J_world: Vector2 = state.transform.basis_xform(J_local)

		emit_signal("contact_impulse", J_world, p_world, n_world)

# -----------------------------------------------------------------------------
# Visuals
# -----------------------------------------------------------------------------
func _ensure_visual_nodes() -> void:
	_blade_visual = get_node_or_null(blade_visual_path) as Polygon2D
	_handle_visual = get_node_or_null(handle_visual_path) as Polygon2D

	if _blade_visual == null:
		_blade_visual = Polygon2D.new()
		_blade_visual.name = "BladeVisual"
		add_child(_blade_visual)

	if _handle_visual == null:
		_handle_visual = Polygon2D.new()
		_handle_visual.name = "HandleVisual"
		add_child(_handle_visual)

func set_visuals(
	blade_poly_local: PackedVector2Array,
	handle_len_world: float,
	handle_thickness: float,
	blade_color: Color,
	handle_color: Color
) -> void:
	_ensure_visual_nodes()

	if _blade_visual != null and blade_poly_local.size() >= 3:
		_blade_visual.polygon = blade_poly_local
		_blade_visual.color = blade_color

	if _handle_visual != null:
		var half_t: float = handle_thickness * 0.5
		var L: float = maxf(1.0, handle_len_world)
		_handle_visual.polygon = PackedVector2Array([
			Vector2(0.0, -half_t),
			Vector2(L, -half_t),
			Vector2(L, half_t),
			Vector2(0.0, half_t),
		])
		_handle_visual.color = handle_color

func _set_fallback_visuals() -> void:
	_ensure_visual_nodes()

	if _blade_visual != null and _blade_visual.polygon.size() < 3:
		_blade_visual.polygon = PackedVector2Array([
			Vector2(-32.0, -4.0),
			Vector2(32.0, -4.0),
			Vector2(32.0, 4.0),
			Vector2(-32.0, 4.0),
		])
		_blade_visual.color = Color(0.9, 0.9, 0.9, 1.0)

	if _handle_visual != null and _handle_visual.polygon.size() < 3:
		_handle_visual.polygon = PackedVector2Array([
			Vector2(0.0, -2.5),
			Vector2(64.0, -2.5),
			Vector2(64.0, 2.5),
			Vector2(0.0, 2.5),
		])
		_handle_visual.color = Color(0.582, 0.496, 0.457, 1.0)

# -----------------------------------------------------------------------------
# Collision geometry
# -----------------------------------------------------------------------------
func set_convex_parts(parts: Array[PackedVector2Array]) -> void:
	for n in _convex_nodes:
		if is_instance_valid(n):
			n.queue_free()
	_convex_nodes.clear()

	var idx: int = 0
	for poly in parts:
		if poly.size() < 3:
			continue
		var cp: CollisionPolygon2D = CollisionPolygon2D.new()
		cp.name = "Convex_%d" % idx
		cp.build_mode = CollisionPolygon2D.BUILD_SOLIDS
		cp.polygon = poly
		add_child(cp)
		_convex_nodes.append(cp)
		idx += 1

	if _convex_nodes.is_empty():
		_make_fallback_box()

func _make_fallback_box() -> void:
	var cp: CollisionPolygon2D = CollisionPolygon2D.new()
	cp.name = "Convex_Fallback"
	cp.build_mode = CollisionPolygon2D.BUILD_SOLIDS

	var w: float = 18.0
	var h: float = 60.0
	cp.polygon = PackedVector2Array([
		Vector2(-w * 0.5, -h * 0.5),
		Vector2(w * 0.5, -h * 0.5),
		Vector2(w * 0.5, h * 0.5),
		Vector2(-w * 0.5, h * 0.5),
	])
	add_child(cp)
	_convex_nodes.append(cp)

# -----------------------------------------------------------------------------
# Utility
# -----------------------------------------------------------------------------
func get_last_pogo_impulse() -> Vector2:
	return _last_pogo_impulse

func is_in_ground_contact() -> bool:
	return _is_ground_contact

func get_ground_contact_info() -> Dictionary:
	return {
		"is_contact": _is_ground_contact,
		"normal": _ground_contact_normal,
		"point": _ground_contact_point
	}
