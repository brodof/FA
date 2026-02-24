# res://scripts/duel/gladiator_controller/gc_math.gd
# Godot 4.5.1
# Shared math / PD helpers used by gladiator controller modules.

extends RefCounted
class_name GCMath

static func signi(v: float) -> int:
	if v > 0.0:
		return 1
	if v < 0.0:
		return -1
	return 0

static func wrap_angle(a: float) -> float:
	return wrapf(a, -PI, PI)

static func compute_com_world(rb_list: Array, rb_mass_sum: float) -> Vector2:
	if rb_list.is_empty() or rb_mass_sum <= 0.0:
		return Vector2.ZERO
	var sx: float = 0.0
	var sy: float = 0.0
	for rb in rb_list:
		if rb is RigidBody2D:
			var m: float = (rb as RigidBody2D).mass
			var p: Vector2 = (rb as RigidBody2D).global_position
			sx += m * p.x
			sy += m * p.y
	return Vector2(sx / rb_mass_sum, sy / rb_mass_sum)

## Neutral stance X for one foot: cx + axis_sign * half_dx (lead) or cx - axis_sign * half_dx (trail).
static func neutral_target_x(cx: float, half_dx: float, axis_sign: float, lead_is_front: bool, is_front: bool) -> float:
	var x_lead: float = cx + axis_sign * half_dx
	var x_trail: float = cx - axis_sign * half_dx
	return x_lead if (is_front == lead_is_front) else x_trail

## Stance center X such that the support foot is already at its neutral (no net pull on support).
## lead foot: support_x = cx + axis_sign * half_dx => cx = support_x - axis_sign * half_dx
## trail foot: support_x = cx - axis_sign * half_dx => cx = support_x + axis_sign * half_dx
static func center_x_for_neutral_at(support_x: float, half_dx: float, axis_sign: float, lead_is_front: bool, support_is_front: bool) -> float:
	var sign_mult: float = 1.0 if (lead_is_front == support_is_front) else -1.0
	return support_x - axis_sign * half_dx * sign_mult


static func rel_angle(parent: RigidBody2D, child: RigidBody2D) -> float:
	if parent == null or child == null:
		return 0.0
	return wrap_angle(child.global_rotation - parent.global_rotation)


static func apply_angle_pd(rb: RigidBody2D, target_angle: float, k: float, d: float, max_tau: float, tau_scale: float) -> void:
	if rb == null or tau_scale <= 0.0:
		return
	var err: float = wrapf(target_angle - rb.global_rotation, -PI, PI)
	var tau: float = (k * err) - (d * rb.angular_velocity)
	tau = clampf(tau, -max_tau, max_tau)
	rb.apply_torque(tau * tau_scale)

