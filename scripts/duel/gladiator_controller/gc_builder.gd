# res://scripts/duel/gladiator_controller/gc_builder.gd
# Godot 4.5.1
# Builder / init / reset helper for FA_GladiatorPhysicsController.
#
# Owns the heavy-weight setup and reset routines so the controller can keep
# a thin, readable _ready() and delegate implementation details here.

extends RefCounted
class_name GCBuilder

var _host: FA_GladiatorPhysicsController = null


func setup_host(host: FA_GladiatorPhysicsController) -> void:
	_host = host


func post_ready_init() -> void:
	var o := _host
	if o == null:
		return

	_impl_configure_layers_and_masks()

	if o.enable_visual_reparent_to_ragdoll:
		_impl_reparent_visuals_to_ragdoll()

	if o.disable_characterbody_collisions:
		o.collision_layer = 0
		o.collision_mask = 0

	o._g = float(ProjectSettings.get_setting("physics/2d/default_gravity"))
	o._total_mass = _impl_compute_total_ragdoll_mass()
	_impl_build_rb_list()

	_impl_enable_foot_contact_monitor(o._rb_foot_front)
	_impl_enable_foot_contact_monitor(o._rb_foot_rear)

	o._foot_sensor_front = o._get_sensor_from_foot(o._rb_foot_front)
	o._foot_sensor_rear = o._get_sensor_from_foot(o._rb_foot_rear)

	# HARD FAIL: sensor must be the Script on the actual colliding foot RB.
	if o._rb_foot_front != null:
		if o._foot_sensor_front == null:
			o.push_error("Foot sensor FRONT missing. Attach res://scripts/duel/foot_contact_sensor.gd directly to RB_Foot_Front.")
		elif o._foot_sensor_front != o._rb_foot_front:
			o.push_error("Foot sensor FRONT is not on RB_Foot_Front. Remove child sensor RB and attach script directly to RB_Foot_Front. Found: " + str(o._foot_sensor_front.get_path()))
			o._foot_sensor_front = null

	if o._rb_foot_rear != null:
		if o._foot_sensor_rear == null:
			o.push_error("Foot sensor REAR missing. Attach res://scripts/duel/foot_contact_sensor.gd directly to RB_Foot_Rear.")
		elif o._foot_sensor_rear != o._rb_foot_rear:
			o.push_error("Foot sensor REAR is not on RB_Foot_Rear. Remove child sensor RB and attach script directly to RB_Foot_Rear. Found: " + str(o._foot_sensor_rear.get_path()))
			o._foot_sensor_rear = null

	_impl_validate_foot_shapes(o._rb_foot_front, "RB_Foot_Front")
	_impl_validate_foot_shapes(o._rb_foot_rear, "RB_Foot_Rear")

	o._configure_sensors()

	# Sanity: if legs are null, posture/limits can't act.
	if o._rb_thigh_front == null or o._rb_shin_front == null or o._rb_thigh_rear == null or o._rb_shin_rear == null:
		o.push_error("Missing leg RB refs: check rb_thigh_* and rb_shin_* NodePaths in inspector.")
	if o._rb_foot_front == null or o._rb_foot_rear == null:
		o.push_error("Missing foot RB refs: check rb_foot_* NodePaths in inspector.")

	_impl_validate_and_wire_foot_plant_points()

	# Shared refs for modules (node refs + per-frame state)
	o._refs = GCRefs.new()
	o._refs.g = o._g
	o._refs.total_mass = o._total_mass
	o._refs.rb_pelvis = o._rb_pelvis
	o._refs.rb_torso = o._rb_torso
	o._refs.rb_head = o._rb_head
	o._refs.rb_thigh_front = o._rb_thigh_front
	o._refs.rb_shin_front = o._rb_shin_front
	o._refs.rb_foot_front = o._rb_foot_front
	o._refs.rb_thigh_rear = o._rb_thigh_rear
	o._refs.rb_shin_rear = o._rb_shin_rear
	o._refs.rb_foot_rear = o._rb_foot_rear
	o._refs.ragdoll_root = o._ragdoll_root
	o._refs.body_root = o._body_root
	o._refs.foot_sensor_front = o._foot_sensor_front
	o._refs.foot_sensor_rear = o._foot_sensor_rear
	o._refs.rb_list = o._rb_list
	o._refs.rb_mass_sum = o._rb_mass_sum

	o._input_mod = GCInput.new()
	o._input_mod.setup(o, o._refs)
	o._facing_mod = GCFacing.new()
	o._facing_mod.setup(o, o._refs)
	o._facing_mod.tick(0.0)
	o._rb_pelvis_opponent = o._refs.opponent_pelvis

	o._debug_mod = GCDebug.new()
	o._debug_mod.setup(o, o._refs)
	o._ground_truth_mod = o._preload_gc_ground_truth.new()
	o._ground_truth_mod.setup(o, o._refs)
	o._vertical_support_mod = GCVerticalSupport.new()
	o._vertical_support_mod.setup(o, o._refs)
	o._posture_mod = GCPostureLimits.new()
	o._posture_mod.setup(o, o._refs)
	var _stance_script: GDScript = load(o._path_gc_stance_planner) as GDScript
	o._stance_planner_mod = _stance_script.new() if _stance_script != null else null
	if o._stance_planner_mod != null:
		o._stance_planner_mod.setup(o, o._refs)
	var _step_script: GDScript = load(o._path_gc_step_planner) as GDScript
	o._step_planner_mod = _step_script.new() if _step_script != null else null
	if o._step_planner_mod != null:
		o._step_planner_mod.setup(o, o._refs)
	o._foot_plant_mod = GCFootPlant.new()
	o._foot_plant_mod.setup(o, o._refs)
	o._recovery_mod = GCRecovery.new()
	o._recovery_mod.setup(o, o._refs)
	o._locomotion_mod = GCLocomotion.new()
	o._locomotion_mod.setup(o, o._refs)

	o._runtime_mod = o._preload_gc_runtime.new()
	o._runtime_mod.setup(o, o._refs)

	# All controller modules are mandatory. If any failed to construct or set up, hard-disable the controller.
	if o._input_mod == null \
			or o._facing_mod == null \
			or o._debug_mod == null \
			or o._ground_truth_mod == null \
			or o._vertical_support_mod == null \
			or o._posture_mod == null \
			or o._stance_planner_mod == null \
			or o._step_planner_mod == null \
			or o._foot_plant_mod == null \
			or o._recovery_mod == null \
			or o._locomotion_mod == null \
			or o._runtime_mod == null:
		o.push_error("GladiatorPhysicsController: one or more controller modules failed to initialize; disabling controller.")
		o.enable_controller = false
		return

	if o.configure_ragdoll_defaults_on_start:
		_impl_apply_ragdoll_defaults()
	if o.enforce_feet_material:
		_impl_apply_feet_material()

	reset_runtime_state()

	# --- Per-instance InputMap auto-bind (prevents both gladiators sharing ui_* defaults) ---
	if o.input_actions_enable:
		if o.is_player2:
			if o.action_move_left == &"ui_left" and InputMap.has_action(&"p2_left"): o.action_move_left = &"p2_left"
			if o.action_move_right == &"ui_right" and InputMap.has_action(&"p2_right"): o.action_move_right = &"p2_right"
			if o.action_stance_up == &"ui_up" and InputMap.has_action(&"p2_up"): o.action_stance_up = &"p2_up"
			if o.action_stance_down == &"ui_down" and InputMap.has_action(&"p2_down"): o.action_stance_down = &"p2_down"
		else:
			if o.action_move_left == &"ui_left" and InputMap.has_action(&"p1_left"): o.action_move_left = &"p1_left"
			if o.action_move_right == &"ui_right" and InputMap.has_action(&"p1_right"): o.action_move_right = &"p1_right"
			if o.action_stance_up == &"ui_up" and InputMap.has_action(&"p1_up"): o.action_stance_up = &"p1_up"
			if o.action_stance_down == &"ui_down" and InputMap.has_action(&"p1_down"): o.action_stance_down = &"p1_down"

		if o.debug_enable:
			var still_ui := (
				o.action_move_left == &"ui_left"
				or o.action_move_right == &"ui_right"
				or o.action_stance_up == &"ui_up"
				or o.action_stance_down == &"ui_down"
			)
			if still_ui:
				o.push_warning((o._debug_mod.dbg_prefix() if o._debug_mod != null else "") + " using ui_* actions (both instances will respond unless you bind per-player actions).")

	# Seed "rest" from authored pose (baseline for limits even before a stable ground capture).
	if o._recovery_mod != null:
		o._recovery_mod.capture_rest_snapshot(false)
	o._rest_seeded = true

	if o.debug_collision_sanity_on_start and o._debug_mod != null:
		o._debug_mod.dbg_collision_sanity_start()

	o.set_process_input(true)

	o._neutral_axis_sign = (1 if o.facing_sign >= 0 else -1)
	o._init_done = true


func reset_runtime_state() -> void:
	var o := _host
	if o == null:
		return

	if o._input_mod != null:
		o._input_mod.reset()
	if o._facing_mod != null:
		o._facing_mod.reset()
	if o._debug_mod != null:
		o._debug_mod.reset()
	if o._vertical_support_mod != null:
		o._vertical_support_mod.reset()
	if o._posture_mod != null:
		o._posture_mod.reset()
	if o._stance_planner_mod != null:
		o._stance_planner_mod.reset()
	if o._step_planner_mod != null:
		o._step_planner_mod.reset()
	if o._foot_plant_mod != null:
		o._foot_plant_mod.reset()
	if o._recovery_mod != null:
		o._recovery_mod.reset()
	if o._locomotion_mod != null:
		o._locomotion_mod.reset()

	o._t = 0.0
	o._dbg_accum = 0.0

	o._support_blend = 0.0
	o._pelvis_upright_blend = 0.0
	o._muscle_blend = 0.0
	o._posture_blend = 0.0

	o._impact_timer = 0.0
	o._grounded_prev = false
	o._ground_grace_t = 0.0
	o._support_y_last_valid = NAN
	o._support_hold_t = 0.0

	# Phase 5
	o._phase5_upright_saturated = false
	o._phase5_limit_saturated = false
	o._phase5_support_saturated = false
	o._phase5_spine_stable_t = 0.0
	o._dbg_phase5_upr_sat = 0
	o._dbg_phase5_lim_sat = 0
	o._dbg_phase5_sup_sat = 0
	o._dbg_phase5_spine_on = 0

	o._vsupport_Fy_prev = 0.0
	o._dbg_sink_allow = 0.0
	o._dbg_vsupport_gate = 0.0

	o._plant_tick_id = 0
	o._allow_plant_forces_this_frame = true
	o._dbg_anchor_moved_front = false
	o._dbg_anchor_moved_rear = false
	o._dbg_recenter_skip_f = -1
	o._dbg_recenter_skip_r = -1
	o._dbg_recenter_want_f = NAN
	o._dbg_recenter_want_r = NAN
	o._dbg_recenter_now_f = NAN
	o._dbg_recenter_now_r = NAN
	o._dbg_recenter_dx_f = NAN
	o._dbg_recenter_dx_r = NAN
	o._dbg_foot_x_f = NAN
	o._dbg_foot_x_r = NAN
	o._dbg_phantom_planted_front = false
	o._dbg_phantom_planted_rear = false
	o._planted_heel_front = Vector2.ZERO
	o._planted_toe_front = Vector2.ZERO
	o._planted_heel_rear = Vector2.ZERO
	o._planted_toe_rear = Vector2.ZERO


# -----------------------------------------------------------------------------
# Init/validation implementations (moved from controller; use _host only)
# -----------------------------------------------------------------------------
static func _bit(idx: int) -> int:
	return 1 << (maxi(1, idx) - 1)

func _impl_configure_layers_and_masks() -> void:
	var o := _host
	if o == null:
		return
	var world_bit: int = _bit(o.L_WORLD)
	var body_bit: int = _bit(o.L_P2_BODY) if o.is_player2 else _bit(o.L_P1_BODY)
	var enemy_body_bit: int = _bit(o.L_P1_BODY) if o.is_player2 else _bit(o.L_P2_BODY)
	var weapon_bit: int = _bit(o.L_P2_WEAPON) if o.is_player2 else _bit(o.L_P1_WEAPON)
	var enemy_weapon_bit: int = _bit(o.L_P1_WEAPON) if o.is_player2 else _bit(o.L_P2_WEAPON)
	var body_mask: int = world_bit | enemy_body_bit
	if o.weapons_collide_with_weapons:
		body_mask |= enemy_weapon_bit
	if o.bodies_collide_with_owner_weapon:
		body_mask |= weapon_bit
	_impl_apply_to_ragdoll_bodies(o, body_bit, body_mask)

func _impl_apply_to_ragdoll_bodies(o: FA_GladiatorPhysicsController, body_layer_mask: int, body_mask: int) -> void:
	if o == null or o._ragdoll_root == null:
		return
	var stack: Array[Node] = [o._ragdoll_root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is CollisionObject2D:
			var co: CollisionObject2D = n as CollisionObject2D
			co.collision_layer = body_layer_mask
			co.collision_mask = body_mask
		for c: Node in n.get_children():
			stack.append(c)

func _impl_reparent_visuals_to_ragdoll() -> void:
	var o := _host
	if o == null:
		return
	_impl_reparent_keep_global(o._vis_torso, o._rb_torso)
	_impl_reparent_keep_global(o._vis_head, o._rb_head)
	_impl_reparent_keep_global(o._vis_upper_front, o.get_node_or_null(o.rb_upperarm_front_path))
	_impl_reparent_keep_global(o._vis_lower_front, o.get_node_or_null(o.rb_lowerarm_front_path))
	_impl_reparent_keep_global(o._vis_upper_rear, o.get_node_or_null(o.rb_upperarm_rear_path))
	_impl_reparent_keep_global(o._vis_lower_rear, o.get_node_or_null(o.rb_lowerarm_rear_path))
	_impl_reparent_keep_global(o._vis_upper_leg_front, o._rb_thigh_front)
	_impl_reparent_keep_global(o._vis_lower_leg_front, o._rb_shin_front)
	_impl_reparent_keep_global(o._vis_upper_leg_rear, o._rb_thigh_rear)
	_impl_reparent_keep_global(o._vis_lower_leg_rear, o._rb_shin_rear)

static func _impl_reparent_keep_global(child: Node, new_parent: Node) -> void:
	if child == null or new_parent == null:
		return
	if child.get_parent() == new_parent:
		return
	child.reparent(new_parent, true)

func _impl_enable_foot_contact_monitor(foot: RigidBody2D) -> void:
	if foot == null:
		return
	foot.contact_monitor = true
	foot.max_contacts_reported = maxi(8, foot.max_contacts_reported)

func _impl_count_direct_collision_shapes(rb: RigidBody2D) -> int:
	if rb == null:
		return 0
	var n: int = 0
	for c: Node in rb.get_children():
		if c is CollisionShape2D:
			var cs := c as CollisionShape2D
			if cs != null and (not cs.disabled) and cs.shape != null:
				n += 1
	return n

func _impl_count_nested_collision_shapes(rb: RigidBody2D) -> int:
	if rb == null:
		return 0
	var found: Array[Node] = rb.find_children("*", "CollisionShape2D", true, false)
	var n: int = 0
	for node in found:
		var cs := node as CollisionShape2D
		if cs != null and (not cs.disabled) and cs.shape != null:
			n += 1
	return n

func _impl_dbg_list_collision_shapes(rb: RigidBody2D) -> String:
	if rb == null:
		return "null"
	var parts: Array[String] = []
	var found: Array[Node] = rb.find_children("*", "CollisionShape2D", true, false)
	for node in found:
		var cs := node as CollisionShape2D
		if cs == null:
			continue
		var sh: Shape2D = cs.shape
		parts.append(
			str(cs.get_path())
			+ "(direct=" + str(int(cs.get_parent() == rb))
			+ " disabled=" + str(int(cs.disabled))
			+ " shape=" + (sh.get_class() if sh != null else "null") + ")"
		)
	return ", ".join(parts)

func _impl_validate_foot_shapes(rb: RigidBody2D, label: String) -> void:
	var o := _host
	if o == null or rb == null:
		return
	var direct_ok: int = _impl_count_direct_collision_shapes(rb)
	if direct_ok > 0:
		return
	var nested_ok: int = _impl_count_nested_collision_shapes(rb)
	if nested_ok > 0:
		o.push_error(label + " has CollisionShape2D nodes, but NONE are valid DIRECT children of the foot body. Godot will ignore nested shapes; you will get zero contacts. Found: " + _impl_dbg_list_collision_shapes(rb))
	else:
		o.push_error(label + " has no enabled CollisionShape2D with a Shape2D. It will never report contacts. Found: " + _impl_dbg_list_collision_shapes(rb))

func _impl_compute_total_ragdoll_mass() -> float:
	var o := _host
	if o == null or o._ragdoll_root == null:
		return 1.0
	var m: float = 0.0
	var stack: Array[Node] = [o._ragdoll_root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is RigidBody2D:
			m += (n as RigidBody2D).mass
		for c: Node in n.get_children():
			stack.append(c)
	return maxf(0.1, m)

func _impl_build_rb_list() -> void:
	var o := _host
	if o == null:
		return
	o._rb_list.clear()
	o._rb_mass_sum = 0.0
	if o._ragdoll_root == null:
		return
	var stack: Array[Node] = [o._ragdoll_root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is RigidBody2D:
			var rb := n as RigidBody2D
			o._rb_list.append(rb)
			o._rb_mass_sum += rb.mass
		for c: Node in n.get_children():
			stack.append(c)
	o._rb_mass_sum = maxf(0.1, o._rb_mass_sum)
	o._ray_exclude.clear()
	for rb in o._rb_list:
		o._ray_exclude.append(rb.get_rid())

static func _impl_cs_sole_local_point(cs: CollisionShape2D) -> Vector2:
	if cs == null:
		return Vector2.ZERO
	var p: Vector2 = cs.position
	var sh: Shape2D = cs.shape
	var yoff: float = 0.0
	if sh is RectangleShape2D:
		var r: RectangleShape2D = sh as RectangleShape2D
		yoff = r.size.y * 0.5
	elif sh is CapsuleShape2D:
		var c: CapsuleShape2D = sh as CapsuleShape2D
		yoff = c.height * 0.5 + c.radius
	elif sh is CircleShape2D:
		var cir: CircleShape2D = sh as CircleShape2D
		yoff = cir.radius
	return Vector2(p.x, p.y + yoff)

func _impl_validate_and_wire_foot_plant_points() -> void:
	var o := _host
	if o == null:
		return
	o._plant_points_wired_ok = false
	o._cs_mid_f = o.get_node_or_null(o.foot_cs_mid_front_path) as CollisionShape2D
	o._cs_heel_f = o.get_node_or_null(o.foot_cs_heel_front_path) as CollisionShape2D
	o._cs_toe_f = o.get_node_or_null(o.foot_cs_toe_front_path) as CollisionShape2D
	o._cs_mid_r = o.get_node_or_null(o.foot_cs_mid_rear_path) as CollisionShape2D
	o._cs_heel_r = o.get_node_or_null(o.foot_cs_heel_rear_path) as CollisionShape2D
	o._cs_toe_r = o.get_node_or_null(o.foot_cs_toe_rear_path) as CollisionShape2D
	var missing: PackedStringArray = PackedStringArray()
	if o._cs_mid_f == null: missing.append(str(o.foot_cs_mid_front_path))
	if o._cs_heel_f == null: missing.append(str(o.foot_cs_heel_front_path))
	if o._cs_toe_f == null: missing.append(str(o.foot_cs_toe_front_path))
	if o._cs_mid_r == null: missing.append(str(o.foot_cs_mid_rear_path))
	if o._cs_heel_r == null: missing.append(str(o.foot_cs_heel_rear_path))
	if o._cs_toe_r == null: missing.append(str(o.foot_cs_toe_rear_path))
	if missing.size() > 0:
		o.push_error("FootPlant CS points unresolved (expected CS_FootMid/Heel/Toe):\n - " + "\n - ".join(missing))
		return
	o._pt_mid_f = _impl_cs_sole_local_point(o._cs_mid_f)
	o._pt_heel_f = _impl_cs_sole_local_point(o._cs_heel_f)
	o._pt_toe_f = _impl_cs_sole_local_point(o._cs_toe_f)
	o._pt_mid_r = _impl_cs_sole_local_point(o._cs_mid_r)
	o._pt_heel_r = _impl_cs_sole_local_point(o._cs_heel_r)
	o._pt_toe_r = _impl_cs_sole_local_point(o._cs_toe_r)
	o._plant_points_wired_ok = true

func _impl_apply_ragdoll_defaults() -> void:
	var o := _host
	if o == null:
		return
	_impl_apply_rigid_damping_defaults()
	_impl_apply_joint_solver_defaults(o)

func _impl_apply_rigid_damping_defaults() -> void:
	var o := _host
	if o == null or o._ragdoll_root == null:
		return
	var stack: Array[Node] = [o._ragdoll_root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is RigidBody2D:
			var rb: RigidBody2D = n as RigidBody2D
			rb.linear_damp = o.rb_default_linear_damp
			rb.angular_damp = o.rb_default_angular_damp
			if rb == o._rb_pelvis:
				rb.linear_damp = o.pelvis_linear_damp
				rb.angular_damp = o.pelvis_angular_damp
			elif rb == o._rb_torso:
				rb.linear_damp = o.torso_linear_damp
				rb.angular_damp = o.torso_angular_damp
			elif rb == o._rb_head:
				rb.linear_damp = o.head_linear_damp
				rb.angular_damp = o.head_angular_damp
			elif rb == o._rb_foot_front or rb == o._rb_foot_rear:
				rb.linear_damp = o.foot_linear_damp_air
				rb.angular_damp = o.foot_angular_damp_air
		for c: Node in n.get_children():
			stack.append(c)

func _impl_apply_joint_solver_defaults(o: FA_GladiatorPhysicsController) -> void:
	if o == null or o._ragdoll_root == null:
		return
	var stack: Array[Node] = [o._ragdoll_root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is Joint2D:
			var j: Joint2D = n as Joint2D
			_impl_set_obj_prop_if_exists(j, &"disable_collision", o.joint_disable_collision)
			_impl_set_obj_prop_if_exists(j, &"bias", o.joint_bias)
			_impl_set_obj_prop_if_exists(j, &"max_bias", o.joint_max_bias)
			_impl_set_obj_prop_if_exists(j, &"max_force", o.joint_max_force)
			_impl_set_obj_prop_if_exists(j, &"softness", o.joint_softness)
		for c: Node in n.get_children():
			stack.append(c)

static func _impl_set_obj_prop_if_exists(obj: Object, prop: StringName, value: Variant) -> void:
	if obj == null:
		return
	var plist: Array = obj.get_property_list()
	for i in range(plist.size()):
		var dct: Dictionary = plist[i]
		var name_v: Variant = dct.get("name")
		if name_v != null and StringName(str(name_v)) == prop:
			obj.set(prop, value)
			return

func _impl_apply_feet_material() -> void:
	var o := _host
	if o == null:
		return
	if o._recovery_mod != null:
		o._recovery_mod.apply_foot_friction(false, false)
	else:
		o._phase6_refresh_foot_materials()
		if o._rb_foot_front != null:
			o._rb_foot_front.physics_material_override = o._pm_feet_base
		if o._rb_foot_rear != null:
			o._rb_foot_rear.physics_material_override = o._pm_feet_base
