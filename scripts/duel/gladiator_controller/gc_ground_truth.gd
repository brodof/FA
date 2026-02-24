# res://scripts/duel/gladiator_controller/gc_ground_truth.gd
# Phase 2 fused ground truth: SENSOR(valid) -> PROBE(tight) -> HOLD_LAST(tight).
# Updates owner _truth_front_g, _truth_rear_g, _truth_front_stab, _truth_rear_stab,
# _truth_front_allow_t, _truth_rear_allow_t from foot sensors.
class_name GCGroundTruth
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

# --- Fused ground truth (call once per physics tick before using truth_* vars) ---
func update_fused_truth(dt: float) -> void:
	var grace_sec: float = _owner.get("grounded_grace_sec") as float
	if not is_finite(grace_sec) or grace_sec <= 0.0:
		grace_sec = 0.08

	var front_g: bool = _foot_grounded(_refs.foot_sensor_front)
	var rear_g: bool = _foot_grounded(_refs.foot_sensor_rear)
	var stabF: float = _foot_stability(_refs.foot_sensor_front)
	var stabR: float = _foot_stability(_refs.foot_sensor_rear)

	var allow_front: float = _owner.get("_truth_front_allow_t") as float
	var allow_rear: float = _owner.get("_truth_rear_allow_t") as float
	if front_g:
		allow_front = grace_sec
	else:
		allow_front = maxf(0.0, allow_front - dt)
	if rear_g:
		allow_rear = grace_sec
	else:
		allow_rear = maxf(0.0, allow_rear - dt)

	_owner.set("_truth_front_g", front_g)
	_owner.set("_truth_rear_g", rear_g)
	_owner.set("_truth_front_stab", stabF)
	_owner.set("_truth_rear_stab", stabR)
	_owner.set("_truth_front_allow_t", allow_front)
	_owner.set("_truth_rear_allow_t", allow_rear)

func _foot_grounded(sensor: Node) -> bool:
	if sensor == null:
		return false
	if sensor.get("grounded"):
		return true
	var cnt: int = sensor.get("contact_count") if sensor.get("contact_count") != null else 0
	return cnt > 0

func _foot_stability(sensor: Node) -> float:
	if sensor == null:
		return 0.0
	var s: Variant = sensor.get("stability01")
	if s != null and typeof(s) == TYPE_FLOAT:
		return clampf(float(s), 0.0, 1.0)
	return 0.0

# --- Phase2 swing force (PD toward target_w, applied to SHIN not foot) ---
# The foot body should only receive attitude (ground-parallel) torque.
# All swing translation force goes through the shin so the ankle joint
# transmits motion naturally and the foot hangs without jitter.
func apply_swing_force(is_front: bool, target_w: Vector2, _dt: float, spawn01: float) -> void:
	var shin: RigidBody2D = _refs.rb_shin_front if is_front else _refs.rb_shin_rear
	var foot: RigidBody2D = _refs.rb_foot_front if is_front else _refs.rb_foot_rear
	if shin == null or foot == null or spawn01 <= 0.0:
		return

	var m := maxf(0.001, shin.mass + foot.mass)
	var freq: float = _owner.get("phase2_swing_freq_hz") as float
	var zeta: float = _owner.get("phase2_swing_zeta") as float
	var mult: float = _owner.get("phase2_swing_force_mult") as float
	if not is_finite(freq) or freq <= 0.0:
		freq = 1.0
	if not is_finite(zeta):
		zeta = 0.5
	if not is_finite(mult):
		mult = 1.0

	var w := TAU * maxf(0.1, freq)
	var k := m * w * w
	var d := 2.0 * m * w * maxf(0.0, zeta)

	var err := target_w - foot.global_position
	var vel := shin.linear_velocity

	var F := (k * err) - (d * vel)

	var Fmax := (m * _refs.g) * maxf(0.0, mult)
	F.x = clampf(F.x, -Fmax, Fmax)
	F.y = clampf(F.y, -Fmax, Fmax)

	shin.apply_central_force(F * spawn01)
