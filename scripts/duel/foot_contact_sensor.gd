# res://scripts/duel/foot_contact_sensor.gd
# Godot 4.5.1
# Foot ground contact sensor:
# 1) Primary: contact manifold in _integrate_forces()
# 2) Fallback: short sole rays when manifold has 0 contacts
#    - can REQUIRE collision handshake (mask/layer) to avoid “fake grounded”

extends RigidBody2D
class_name FootContactSensor

const SRC_NONE: int = 0
const SRC_MANIFOLD: int = 1
const SRC_FALLBACK: int = 2

@export var world_layer_index: int = 1
@export var accept_any_static: bool = true
@export var ground_normal_min_up: float = 0.18
@export var ignore_root: Node = null: set = _set_ignore_root

@export_group("Fallback Rays (only when manifold contacts = 0)")
@export var fallback_enable: bool = true
@export var fallback_ray_len: float = 10.0
@export var fallback_max_gap_px: float = 2.5
@export var fallback_ray_local_origin: Vector2 = Vector2.ZERO
@export var fallback_require_mask_handshake: bool = true
@export var fallback_require_static: bool = true
@export var fallback_vy_max: float = 140.0
@export var fallback_vx_max: float = 220.0
@export var fallback_w_max: float = 7.0

@export_group("Fallback Confidence (anti-jitter)")
@export var fallback_confidence_tau_sec: float = 0.08
@export var fallback_confidence_min: float = 0.35
@export var fallback_confidence_velocity_ref_vx: float = 160.0
@export var fallback_confidence_velocity_ref_vy: float = 120.0
@export var fallback_confidence_velocity_ref_w: float = 6.0

@export_group("Honest Grounded")
@export var max_grounded_clearance_px: float = 3.0  # If foot center is more than this many px above contact, do not accept as grounded.

@export_group("Debug (read-only)")
@export var dbg_contact_count: int = 0
@export var dbg_fallback_hits: int = 0
@export var dbg_fallback_used: bool = false
var dbg_handshake_ok: bool = false
@export var dbg_rej_above: int = 0
@export var dbg_rej_gap: int = 0
@export var dbg_rej_layer: int = 0
@export var dbg_rej_static: int = 0
@export var dbg_rej_vmax: int = 0
@export var dbg_rej_hysteresis: int = 0

# Manifold rejection breakdown (primary contacts)
@export var dbg_man_valid_count: int = 0
@export var dbg_man_rej_ignored: int = 0
@export var dbg_man_rej_updot: int = 0
@export var dbg_man_rej_layer: int = 0
@export var dbg_man_rej_static: int = 0
@export var dbg_rej_clearance: int = 0  # Grounding rejected: foot too far above contact.

# Controller-facing source classification
@export var contact_valid: bool = false
@export var contact_source: int = SRC_NONE

# -----------------------------------------------------------------------------
# Controller-facing API fields (must exist; controller reads these by name)
# -----------------------------------------------------------------------------
@export var contact_count: int = 0 # raw manifold count
@export var accepted_count: int = 0 # 1 if grounded accepted this tick else 0
var contact_normal_w: Vector2 = Vector2.UP # alias for controller debug (world normal)

# -----------------------------------------------------------------------------
# Public state (read by controller)
# -----------------------------------------------------------------------------
var grounded: bool = false
var ground_normal_world: Vector2 = Vector2.UP
var ground_point_world: Vector2 = Vector2.ZERO
var ground_collider: Object = null

# Controller compatibility fields (expected by gladiator_physics_controller.gd)
var contact_point_w: Vector2 = Vector2.ZERO
var contact_y_raw: float = NAN
var contact_y: float = NAN
var stability01: float = 0.0

# Manifold stats
var contact_point_world: Vector2 = Vector2.ZERO
var contact_normal_world: Vector2 = Vector2.UP
var contact_collider: Object = null
var contact_has_valid: bool = false

# Fallback confidence
var _fallback_confidence: float = 0.0
var _fallback_grounded: bool = false
var _fallback_point: Vector2 = Vector2.ZERO
var _fallback_normal: Vector2 = Vector2.UP
var _fallback_collider: Object = null

var _exclude_rids: Array[RID] = []
var _exclude_dirty: bool = true

func _enter_tree() -> void:
	# Ensure contact reporting is on.
	contact_monitor = true
	max_contacts_reported = 8
	_rebuild_exclude_rids()

func _integrate_forces(state: PhysicsDirectBodyState2D) -> void:
	# Reset per-step debug counters.
	dbg_contact_count = 0
	dbg_fallback_hits = 0
	dbg_fallback_used = false
	dbg_handshake_ok = false
	dbg_rej_above = 0
	dbg_rej_gap = 0
	dbg_rej_layer = 0
	dbg_rej_static = 0
	dbg_rej_vmax = 0
	dbg_rej_hysteresis = 0
	dbg_man_valid_count = 0
	dbg_man_rej_ignored = 0
	dbg_man_rej_updot = 0
	dbg_man_rej_layer = 0
	dbg_man_rej_static = 0
	dbg_rej_clearance = 0

	# Read manifold contacts.
	contact_has_valid = false
	contact_point_world = Vector2.ZERO
	contact_normal_world = Vector2.UP
	contact_collider = null

	var ccount := state.get_contact_count()
	dbg_contact_count = ccount

	if ccount > 0:
		# Choose best *valid* contact (manifold): iterate, reject invalid per-contact, then pick best up-dot.
		if _exclude_dirty:
			_rebuild_exclude_rids()

		var best_i := -1
		var best_up := -INF
		var best_point := Vector2.ZERO
		var best_normal := Vector2.UP
		var best_collider: Object = null
		var valid_count := 0

		for i in range(ccount):
			var collider := state.get_contact_collider_object(i)

			if _is_excluded_collider(collider):
				dbg_man_rej_ignored += 1
				continue

			var normal_world := Vector2.UP
			if state.has_method("get_contact_collider_normal"):
				normal_world = state.get_contact_collider_normal(i).normalized()
			else:
				normal_world = global_transform.basis_xform(state.get_contact_local_normal(i)).normalized()

			if not (is_finite(normal_world.x) and is_finite(normal_world.y)):
				normal_world = Vector2.UP

			var up := normal_world.dot(Vector2.UP)
			if up < ground_normal_min_up:
				dbg_man_rej_updot += 1
				continue

			if fallback_require_mask_handshake and not _handshake_ok(collider):
				dbg_man_rej_layer += 1
				continue

			if fallback_require_static and not _is_static_body(collider):
				dbg_man_rej_static += 1
				continue

			valid_count += 1
			if up > best_up:
				best_up = up
				best_i = i
				best_collider = collider
				best_normal = normal_world

				if state.has_method("get_contact_collider_position"):
					best_point = state.get_contact_collider_position(i)
				else:
					best_point = state.get_contact_local_position(i)

		dbg_man_valid_count = valid_count

		if best_i >= 0:
			contact_has_valid = true
			contact_point_world = best_point
			contact_normal_world = best_normal
			contact_collider = best_collider

	if contact_has_valid:
		_set_ground_from_manifold()
		# When manifold is good, decay fallback confidence toward 0 quickly.
		_fallback_confidence = 0.0
	else:
		# If manifold is empty/invalid, attempt fallback rays.
		if fallback_enable:
			_try_fallback_rays(state)
		else:
			_fallback_confidence = 0.0
			_clear_ground()

	# Honest grounded: reject if foot center is too far above contact (stops "grounded" while airborne).
	if grounded:
		var clearance_px: float = ground_point_world.y - global_position.y
		if clearance_px > max_grounded_clearance_px:
			grounded = false
			ground_point_world = Vector2.ZERO
			ground_normal_world = Vector2.UP
			ground_collider = null
			dbg_rej_clearance += 1

	# Publish controller-facing fields every step (avoid early returns).
	if grounded:
		contact_point_w = ground_point_world
		contact_y_raw = ground_point_world.y
		contact_y = contact_y_raw
		# Handshake status is useful to gate rest-capture/soft-limits upstream.
		dbg_handshake_ok = _handshake_ok(ground_collider)
		var vx_n := absf(state.linear_velocity.x) / maxf(1.0, fallback_confidence_velocity_ref_vx)
		var vy_n := absf(state.linear_velocity.y) / maxf(1.0, fallback_confidence_velocity_ref_vy)
		var w_n := absf(state.angular_velocity) / maxf(0.001, fallback_confidence_velocity_ref_w)
		var m := maxf(maxf(vx_n, vy_n), w_n)
		stability01 = clampf(1.0 - m, 0.0, 1.0)
		if dbg_fallback_used:
			stability01 = minf(stability01, _fallback_confidence)
		if not dbg_handshake_ok:
			stability01 *= 0.5
	else:
		contact_point_w = Vector2.ZERO
		contact_y_raw = NAN
		contact_y = NAN
		stability01 = 0.0

	# Update counters/aliases for controller contracts
	contact_count = dbg_contact_count
	accepted_count = (1 if grounded else 0)
	contact_valid = grounded
	contact_source = SRC_NONE
	if grounded:
		contact_source = (SRC_FALLBACK if dbg_fallback_used else SRC_MANIFOLD)
	contact_normal_w = ground_normal_world
func _set_ground_from_manifold() -> void:
	grounded = true
	ground_point_world = contact_point_world
	ground_normal_world = contact_normal_world
	ground_collider = contact_collider

func _clear_ground() -> void:
	grounded = false
	ground_point_world = Vector2.ZERO
	ground_normal_world = Vector2.UP
	ground_collider = null

func _get_sole_params_local() -> Dictionary:
	# derive sole from first enabled CollisionShape2D
	for c in get_children():
		if c is CollisionShape2D:
			var cs := c as CollisionShape2D
			if cs.disabled or cs.shape == null:
				continue
			var sh := cs.shape
			var half_w := 6.0
			var half_h := 0.0
			if sh is RectangleShape2D:
				var r := sh as RectangleShape2D
				half_w = r.size.x * 0.5
				half_h = r.size.y * 0.5
			elif sh is CapsuleShape2D:
				var cap := sh as CapsuleShape2D
				half_w = cap.radius
				half_h = cap.height * 0.5 + cap.radius
			elif sh is CircleShape2D:
				var circ := sh as CircleShape2D
				half_w = circ.radius
				half_h = circ.radius
			return {"sole_y": cs.position.y + half_h, "half_w": half_w}
	return {"sole_y": 0.0, "half_w": 6.0}

func _try_fallback_rays(state: PhysicsDirectBodyState2D) -> void:
	# Velocity influences confidence only (never hard-disables grounding).
	var vy := linear_velocity.y
	var vx := absf(linear_velocity.x)
	var w := absf(angular_velocity)

	# Note high-speed situations for debug, but do not hard-disable grounding.
	var speed_n := maxf(
		maxf(vx / maxf(1.0, fallback_confidence_velocity_ref_vx), absf(vy) / maxf(1.0, fallback_confidence_velocity_ref_vy)),
		w / maxf(0.001, fallback_confidence_velocity_ref_w)
	)
	if speed_n > 1.0:
		dbg_rej_vmax += 1

	# Derive sole geometry (respects CollisionShape2D) and cast rays from the true sole in WORLD space.
	var sp := _get_sole_params_local()
	var sole_y: float = float(sp["sole_y"])
	var half_w: float = float(sp["half_w"])
	if fallback_ray_local_origin != Vector2.ZERO:
		sole_y = fallback_ray_local_origin.y

	var inset := clampf(half_w * 0.80, 2.0, 20.0)

	# local sole points -> world points (respects rotation)
	var origin_l := to_global(Vector2(-inset + fallback_ray_local_origin.x, sole_y))
	var origin_r := to_global(Vector2(+inset + fallback_ray_local_origin.x, sole_y))

	var dir := Vector2.DOWN
	var hit_l := _ray(origin_l, dir, fallback_ray_len)
	var hit_r := _ray(origin_r, dir, fallback_ray_len)

	# Pick the closer valid hit.
	var have_hit := false
	var best_point := Vector2.ZERO
	var best_normal := Vector2.UP
	var best_collider: Object = null
	var best_dist := INF

	if hit_l.has("ok") and hit_l.ok:
		have_hit = true
		best_point = hit_l.point
		best_normal = hit_l.normal
		best_collider = hit_l.collider
		best_dist = hit_l.dist
		dbg_fallback_hits += 1

	if hit_r.has("ok") and hit_r.ok:
		dbg_fallback_hits += 1
		if not have_hit or hit_r.dist < best_dist:
			have_hit = true
			best_point = hit_r.point
			best_normal = hit_r.normal
			best_collider = hit_r.collider
			best_dist = hit_r.dist

	if not have_hit:
		_fallback_confidence = 0.0
		_clear_ground()
		return

	# Gap gate (don’t “ground” if too far below).
	if best_dist > fallback_max_gap_px:
		dbg_rej_gap += 1
		_fallback_confidence = 0.0
		_clear_ground()
		return

	# Normal gate
	if best_normal.dot(Vector2.UP) < ground_normal_min_up:
		dbg_rej_above += 1
		_fallback_confidence = 0.0
		_clear_ground()
		return

	# Static gate
	var is_static_ok := true
	if fallback_require_static:
		is_static_ok = _is_static_body(best_collider)

	if not is_static_ok:
		dbg_rej_static += 1
		_fallback_confidence = 0.0
		_clear_ground()
		return

	# Mask/layer handshake gate (optional).
	if fallback_require_mask_handshake:
		var ok := _handshake_ok(best_collider)
		dbg_handshake_ok = ok
		if not ok:
			dbg_rej_layer += 1
			_fallback_confidence = 0.0
			_clear_ground()
			return
	else:
		dbg_handshake_ok = true

	# Update confidence (anti-jitter): confidence decreases as speed increases.
	var dt := state.get_step()

	var vx_n := clampf(absf(linear_velocity.x) / maxf(0.0001, fallback_confidence_velocity_ref_vx), 0.0, 2.0)
	var vy_n := clampf(absf(linear_velocity.y) / maxf(0.0001, fallback_confidence_velocity_ref_vy), 0.0, 2.0)
	var w_n := clampf(absf(angular_velocity) / maxf(0.0001, fallback_confidence_velocity_ref_w), 0.0, 2.0)

	var worst := maxf(maxf(vx_n, vy_n), w_n)
	var target := clampf(1.0 - worst, 0.0, 1.0)

	# Low-pass filter toward target.
	var tau := maxf(0.0001, fallback_confidence_tau_sec)
	var a := 1.0 - exp(-dt / tau)
	_fallback_confidence = lerp(_fallback_confidence, target, a)

	# Confidence below min is fine: stay grounded, but report low stability.
	if _fallback_confidence < fallback_confidence_min:
		dbg_rej_hysteresis += 1

	# Accept fallback ground.
	dbg_fallback_used = true
	grounded = true
	ground_point_world = best_point
	ground_normal_world = best_normal
	ground_collider = best_collider

func _ray(origin: Vector2, dir: Vector2, length: float) -> Dictionary:
	var space := get_world_2d().direct_space_state
	var to := origin + dir.normalized() * length

	var params := PhysicsRayQueryParameters2D.create(origin, to)
	params.collide_with_areas = false
	params.collide_with_bodies = true
	params.collision_mask = 1 << (world_layer_index - 1)

	# Avoid self and ignore_root subtree (cached; no per-ray tree walk).
	if _exclude_dirty:
		_rebuild_exclude_rids()
	params.exclude = _exclude_rids

	var res := space.intersect_ray(params)
	if res.is_empty():
		return {"ok": false}

	# Expected keys: position/normal/collider
	return {
		"ok": true,
		"point": res.position,
		"normal": res.normal.normalized(),
		"collider": res.collider,
		"dist": origin.distance_to(res.position),
	}

func _is_excluded_collider(collider_obj: Object) -> bool:
	if collider_obj == null:
		return false
	if _exclude_dirty:
		_rebuild_exclude_rids()
	if collider_obj is PhysicsBody2D:
		return _exclude_rids.has((collider_obj as PhysicsBody2D).get_rid())
	if collider_obj is CollisionObject2D:
		return _exclude_rids.has((collider_obj as CollisionObject2D).get_rid())
	return false

func _set_ignore_root(n: Node) -> void:
	ignore_root = n
	_exclude_dirty = true

func _rebuild_exclude_rids() -> void:
	_exclude_rids.clear()
	_exclude_rids.append(get_rid())

	if ignore_root != null:
		_gather_exclude_rids(ignore_root, _exclude_rids)

	_exclude_dirty = false

func _gather_exclude_rids(node: Node, out: Array[RID]) -> void:
	if node is CollisionObject2D:
		out.append((node as CollisionObject2D).get_rid())
	for c in node.get_children():
		if c is Node:
			_gather_exclude_rids(c, out)

func _is_static_body(collider_obj: Object) -> bool:
	if collider_obj == null:
		return false
	if accept_any_static and collider_obj is StaticBody2D:
		return true
	if collider_obj is RigidBody2D:
		var rb := collider_obj as RigidBody2D
		return rb.freeze or rb.mass <= 0.0 # heuristic: frozen is effectively static
	return false

func _handshake_ok(collider_obj: Object) -> bool:
	# Collision handshake means: my mask hits their layer AND their mask hits my layer.
	# If collider is not a CollisionObject2D, fail closed.
	if collider_obj == null:
		return false
	if not (collider_obj is CollisionObject2D):
		return false

	var other := collider_obj as CollisionObject2D
	var my_layer := collision_layer
	var my_mask := collision_mask
	var other_layer := other.collision_layer
	var other_mask := other.collision_mask

	# We require each to “see” the other.
	var i_see_other := (my_mask & other_layer) != 0
	var other_sees_me := (other_mask & my_layer) != 0
	return i_see_other and other_sees_me
