# res://scripts/duel/gladiator_physics_controller.gd
# Godot 4.5.1
# Baseline Standing Lab Controller (no kinematic posing, no teleporting rigid bodies)
# - RB_Pelvis remains locomotion authority (forces/torques only)
# - Plant anchors may be repositioned (non-colliding, shape-free static helpers)

extends CharacterBody2D
class_name FA_GladiatorPhysicsController

# Preload controller modules so class_name types (GCRefs, GCVerticalSupport, etc.) resolve.
const _preload_gc_refs = preload("res://scripts/duel/gladiator_controller/gc_refs.gd")
const _preload_gc_types = preload("res://scripts/duel/gladiator_controller/gc_types.gd")
const _preload_gc_input = preload("res://scripts/duel/gladiator_controller/gc_input.gd")
const _preload_gc_facing = preload("res://scripts/duel/gladiator_controller/gc_facing.gd")
const _preload_gc_debug = preload("res://scripts/duel/gladiator_controller/gc_debug.gd")
const _preload_gc_ground_truth = preload("res://scripts/duel/gladiator_controller/gc_ground_truth.gd")
const _preload_gc_vertical_support = preload("res://scripts/duel/gladiator_controller/gc_vertical_support.gd")
const _preload_gc_posture_limits = preload("res://scripts/duel/gladiator_controller/gc_posture_limits.gd")
const _preload_gc_math = preload("res://scripts/duel/gladiator_controller/gc_math.gd")
const _path_gc_stance_planner = "res://scripts/duel/gladiator_controller/gc_stance_planner.gd"
const _path_gc_step_planner = "res://scripts/duel/gladiator_controller/gc_step_planner.gd"
const _preload_gc_foot_plant = preload("res://scripts/duel/gladiator_controller/gc_foot_plant.gd")
const _preload_gc_recovery = preload("res://scripts/duel/gladiator_controller/gc_recovery.gd")
const _preload_gc_locomotion = preload("res://scripts/duel/gladiator_controller/gc_locomotion.gd")
const _preload_gc_runtime = preload("res://scripts/duel/gladiator_controller/gc_runtime.gd")
const _preload_gc_builder = preload("res://scripts/duel/gladiator_controller/gc_builder.gd")

@export var is_player2: bool = false
@export var disable_characterbody_collisions: bool = true

@export_group("Authority")
@export var facing_sign: int = 1 # must be either -1 or +1
@export var facing_deadzone_x: float = 10.0 # px. If |dx_to_enemy| < this, keep last facing_sign.
@export var role_swap_confirm_sec: float = 0.10
@export var support_swap_confirm_sec: float = 0.10
@export var air_strength_floor: float = 0.25 # 0.15..0.35 usable

# 5R2-A/5R2-E: hard locomotion authority switch.
# Migration branch default is STEP_PLANNER so tests actually exercise the new architecture.
@export var movement_authority_mode: int = GCTypes.MovementAuthorityMode.STEP_PLANNER

# =============================================================================
# Collision layer bit indices (1..32). Must match Project Settings.
# =============================================================================
@export_group("Layers")
@export var L_WORLD: int = 1
@export var L_P1_BODY: int = 2
@export var L_P2_BODY: int = 3
@export var L_P1_WEAPON: int = 4
@export var L_P2_WEAPON: int = 5
@export var weapons_collide_with_weapons: bool = true
@export var weapons_collide_with_owner: bool = false
@export var bodies_collide_with_owner_weapon: bool = false
@export var ground_accept_any_static: bool = true

# =============================================================================
# Required node paths (inspector-wired if your scene differs)
# =============================================================================
@export_group("NodePaths")
@export var body_root_path: NodePath = NodePath("BodyRoot")
@export var ragdoll_root_path: NodePath = NodePath("Ragdoll")
# Optional: wire to the OTHER fighter's RB_Pelvis so facing can be opponent-locked.
@export var opponent_rb_pelvis_path: NodePath = NodePath("")

@export var rb_pelvis_path: NodePath = NodePath("Ragdoll/RB_Pelvis")
@export var rb_torso_path: NodePath = NodePath("Ragdoll/RB_Torso")
@export var rb_head_path: NodePath = NodePath("Ragdoll/RB_Head")

@export var rb_thigh_front_path: NodePath = NodePath("Ragdoll/RB_Thigh_Front")
@export var rb_shin_front_path: NodePath = NodePath("Ragdoll/RB_Shin_Front")
@export var rb_foot_front_path: NodePath = NodePath("Ragdoll/RB_Foot_Front")

@export var rb_thigh_rear_path: NodePath = NodePath("Ragdoll/RB_Thigh_Rear")
@export var rb_shin_rear_path: NodePath = NodePath("Ragdoll/RB_Shin_Rear")
@export var rb_foot_rear_path: NodePath = NodePath("Ragdoll/RB_Foot_Rear")

@export var rb_upperarm_front_path: NodePath = NodePath("Ragdoll/RB_UpperArm_Front")
@export var rb_lowerarm_front_path: NodePath = NodePath("Ragdoll/RB_LowerArm_Front")
@export var rb_upperarm_rear_path: NodePath = NodePath("Ragdoll/RB_UpperArm_Rear")
@export var rb_lowerarm_rear_path: NodePath = NodePath("Ragdoll/RB_LowerArm_Rear")

# Optional visual reparent convenience (doesn't affect physics)
@export_group("Visual Follow")
@export var enable_visual_reparent_to_ragdoll: bool = true
@export var torso_node_path: NodePath = NodePath("BodyRoot/Torso")
@export var head_node_path: NodePath = NodePath("BodyRoot/Head")
@export var upper_front_node_path: NodePath = NodePath("BodyRoot/ArmFront/UpperFront")
@export var lower_front_node_path: NodePath = NodePath("BodyRoot/ArmFront/LowerFront")
@export var upper_rear_node_path: NodePath = NodePath("BodyRoot/ArmRear/UpperRear")
@export var lower_rear_node_path: NodePath = NodePath("BodyRoot/ArmRear/LowerRear")
@export var upper_leg_front_node_path: NodePath = NodePath("BodyRoot/FrontLeg/UpperFrontLeg")
@export var lower_leg_front_node_path: NodePath = NodePath("BodyRoot/FrontLeg/LowerFrontLeg")
@export var upper_leg_rear_node_path: NodePath = NodePath("BodyRoot/RearLeg/UpperRearLeg")
@export var lower_leg_rear_node_path: NodePath = NodePath("BodyRoot/RearLeg/LowerRearLeg")

# =============================================================================
# Ragdoll defaults (enforced once on start)
# =============================================================================
@export_group("Ragdoll Defaults")
@export var configure_ragdoll_defaults_on_start: bool = true
@export var rb_default_linear_damp: float = 0.15
@export var rb_default_angular_damp: float = 0.45

@export var pelvis_linear_damp: float = 0.12
@export var pelvis_angular_damp: float = 0.45

@export var torso_linear_damp: float = 0.12
@export var torso_angular_damp: float = 0.60

@export var head_linear_damp: float = 0.12
@export var head_angular_damp: float = 0.70

@export var foot_linear_damp_air: float = 0.08
@export var foot_angular_damp_air: float = 0.20

@export var joint_disable_collision: bool = true
@export var joint_bias: float = 0.2
@export var joint_max_bias: float = 4.0
@export var joint_max_force: float = 1e6
@export var joint_softness: float = 0.0

# Feet material (critical for "can stand" truth)
@export var enforce_feet_material: bool = true
@export var feet_friction: float = 2.2
@export var feet_bounce: float = 0.0

# =============================================================================
# Phase A: Stand + minimal step that is physically possible
# =============================================================================
@export_group("Phase A - Global Gates")
@export var enable_controller: bool = true
@export var spawn_ramp_sec: float = 0.25

@export_group("Phase 1B - Stand Only")
@export var phase1b_stand_only: bool = true
@export var phase1b_allow_pins: bool = true
@export var landing_ramp_sec: float = 0.30 # seconds from touchdown to full muscle (tune)
@export var landing_support_min: float = 0.85 # floor for vertical support force_scale at touchdown (prevents collapse)

# NEW: landing must not start "weak"
@export var landing_muscle_min: float = 0.75 # floor for posture/upright/limits right at touchdown
@export var landing_limit_min: float = 0.55 # floor for NON-knee joint limits at touchdown (hips/ankles/spine)

# NEW: hard touchdowns need extra vertical authority (force cap boost)
@export var landing_impact_vy_ref: float = 520.0 # px/s downward for full boost
@export var landing_impact_cap_boost_g: float = 1.8 # extra g allowance at hard touchdowns (only early in landing)
@export var stand_target_hz: float = 10.0
@export var stand_down_mult: float = 0.35 # "resting" down cap; during active stance shift we temporarily raise it

@export_group("Phase 3 - Standing Planner (no stepping yet)")
@export var phase3_enable: bool = true

# (REMOVED) Phase 2 legacy auto-stepper deleted; no freeze flag needed.

# Semantic state holds (planner-level, not the sensor).
@export var phase3_min_planted_sec: float = 0.16
@export var phase3_min_swing_sec: float = 0.12

# Plant permissiveness around neutral targets (standing only).
@export var phase3_plant_x_window_px: float = 18.0

# Prevent "landing scissor hop": during touchdown/impact, keep ONE support foot planted.
@export var phase3_force_single_support_during_impact: bool = true
@export var phase3_single_support_stab_bias: float = 0.06

@export_group("Phase 6 - Recovery + Spawn BRACE")

# Master enable for Phase 6 RECOVER logic.
@export var phase6_enable: bool = true

# Enter RECOVER if torso upright error exceeds this (degrees).
@export var phase6_enter_torso_err_deg: float = 28.0

# Enter RECOVER if pelvis is below target by this amount (px). (y-down => err_y negative when too low)
@export var phase6_enter_pelvis_below_px: float = 18.0

# Enter RECOVER if either foot is far from its neutral target (px).
@export var phase6_enter_foot_neutral_err_px: float = 48.0

# Exit RECOVER when torso error is below this (degrees) AND feet near neutral for a stable window.
@export var phase6_exit_torso_err_deg: float = 10.0
@export var phase6_exit_foot_neutral_err_px: float = 16.0
@export var phase6_exit_pelvis_err_px: float = 10.0
@export var phase6_exit_stable_sec: float = 0.12

# Minimum time RECOVER stays active once entered (prevents one-frame toggles).
@export var phase6_min_active_sec: float = 0.18

# Swing "lift" during RECOVER (px). (y-down; lift = target_y - lift_px)
@export var phase6_swing_lift_px: float = 10.0


# ------------------------
# Spawn-only BRACE (forced on spawn; behaves like RECOVER but "get under me")
# ------------------------
@export var spawn_brace_enable: bool = true

# BRACE holds at least this long after spawn, then may exit when conditions are met.
@export var spawn_brace_min_sec: float = 0.35

# Safety cap: if grounded, BRACE will not persist beyond this.
@export var spawn_brace_max_sec: float = 0.60

# Absolute safety cap regardless of grounding (prevents inspector overrides trapping BRACE forever).
@export var spawn_brace_hardcap_sec: float = 1.20

# BRACE prepared landing crouch cap for stance alpha (stance_height01).
# BRACE clamps stance_height01 to <= this value (so it crouches rather than locks tall).
@export var spawn_brace_stance_height01: float = 0.55

# BRACE exit gates
@export var spawn_brace_exit_ground_stable_sec: float = 0.10
@export var spawn_brace_exit_pelvis_err_px: float = 10.0
@export var spawn_brace_exit_torso_err_deg: float = 12.0

# BRACE planting window (px). If foot is outside, keep it SWING so it can "get under pelvis".
@export var spawn_brace_plant_x_window_px: float = 28.0

@export var phase6_air_enter_vy: float = 220.0 # if falling faster than this while ungrounded, force RECOVER
@export var phase6_air_enter_torso_err_deg: float = 18.0 # looser mid-air trigger
@export var phase6_swing_force_mult_recover: float = 2.2 # multiply swing force while BRACE/RECOVER
@export var phase6_slide_force_mult: float = 1.4 # extra pull for the sliding (non-support) foot on ground

# When a foot is planned SWING but is still in ground contact (sliding phase), we temporarily lower its friction.
@export var phase6_slide_friction: float = 0.35

# Multiply swing force while stance alpha is changing (crouch/stand transitions),
# so the slide foot can overcome friction (and doesn't feel "stuck").
@export var phase6_swing_force_mult_stance_change: float = 1.8

# While a foot is planned SWING but still grounded (sliding),
# apply an upward force to unweight it so friction doesn't lock it.
# 0 = disabled, 1 = cancel full foot weight.
@export var phase6_swing_unweight_mult: float = 0.55

@export var phase6_single_support_on_stance_change: bool = true
@export var phase6_stance_change_eps: float = 0.0025 # stance_alpha delta per frame considered "changing"

@export_group("Phase 2 - Walk (command driven)")
@export var phase2_enable: bool = true

# If true, controller reads Godot Input actions and overwrites cmd_* every frame.
# If false, you must set cmd_* from your own player code.
@export var input_actions_enable: bool = true
@export var action_move_left: StringName = &"ui_left"
@export var action_move_right: StringName = &"ui_right"
@export var action_stance_up: StringName = &"ui_up"
@export var action_stance_down: StringName = &"ui_down"

# Command: -1..+1 in WORLD X (+ = right, - = left).
@export var cmd_move_x: float = 0.0

# Deadzone applied to cmd_move_x (world). If <= 0, fallback to cmd_move_deadzone.
@export var input_deadzone: float = 0.08

# Legacy inspector field (kept to avoid breaking older scenes). Prefer input_deadzone.
@export var cmd_move_deadzone: float = 0.08

# Derived each frame from (cmd_move_x world) + facing_sign:
# - cmd_forward01: amount of movement input in the facing direction
# - cmd_backward01: amount opposite facing direction
var cmd_forward01: float = 0.0
var cmd_backward01: float = 0.0

# Command: stance shift (alpha). -1=crouch, +1=upright. Hold for ~0.5s full travel.
@export var cmd_stance_y: float = 0.0
@export var cmd_stance_deadzone: float = 0.15
@export var stance_shift_time_sec: float = 0.10

@export var phase2_speed_px_s: float = 130.0

@export var phase2_allow_pins: bool = true

@export_group("Phase 7 - Stepping (planner + executor)")
@export var phase7_enable: bool = true

# Input interpretation
@export var phase7_hold_to_walk_sec: float = 0.22

# Step geometry (world X)
@export var phase7_under_body_step_px: float = 24.0
@export var phase7_stride_time_sec: float = 0.28 # stride_px ~ abs(cmd_move_x_norm) * phase2_speed_px_s * this
@export var phase7_run_ramp_sec: float = 0.85 # sec. Hold duration to ramp walk -> run.
@export var phase7_run_stride_px_mult: float = 1.25 # >=1.0. Multiplies stride_px at full run01.
@export var phase7_run_swing_force_mult: float = 1.60 # >=1.0. Multiplies swing force at full run01.
@export var phase7_min_foot_separation_px: float = 14.0
@export var phase7_cross_allow_px: float = 0.0
@export var phase7_max_foot_reach_px: float = 60.0

# While Phase7 owns locomotion, stance_center_x must stop being a free-integrator.
# Instead, keep it near the planted support foot so the pelvis doesn't "run away" and drag the feet.
@export var phase7_support_center_follow_hz: float = 10.0
@export var phase7_support_center_bias_px: float = 6.0
## When moving, don't pull stance center backward (opposite to step) more than this many px per frame. Reduces torso drag. 0 = disabled.
@export var phase7_stance_center_no_back_px: float = 12.0

# Stop behavior
@export var phase7_stop_correct_enable: bool = true

# Executor (wired in Brief D, declared here to keep one group)
@export var phase7_unplant_sec: float = 0.06
@export var phase7_unweight_mult: float = 0.65
@export var phase7_swing_posture_scale: float = 0.25 # 0..1. During Phase7 UNPLANT/SWING, scale posture PD on the swing leg (prevents "glued foot").
@export var phase7_swing_lift_px: float = 24.0
@export var phase7_lift_leave_ground_y_eps: float = 3.0 # px. Consider "left ground" if foot rises this far above ground_y.
@export var phase7_lift_min_swing_sec: float = 0.08 # sec. Minimum swing time before touchdown can complete (after airborne).
@export var phase7_swing_fail_max_sec: float = 0.45 # sec. If never airborne, allow flagged slide-touchdown after this to avoid deadlock.
@export var phase7_swing_force_mult: float = 1.0
@export var phase7_swing_drop_dx_px: float = 10.0 # px. Start lowering when within this X distance to plan target.
@export var phase7_swing_drop_min_sec: float = 0.10 # sec. Don't drop immediately after lift.
@export var phase7_swing_drop_max_sec: float = 0.32 # sec. Force drop even if still far (prevents dangling).
@export var phase7_touchdown_clearance_px: float = 0.5 # px. Target ground_y - this during drop/touchdown.
@export var phase7_swing_drive_mult: float = 0.35 # 0..1. During UNPLANT/SWING, scale desired_vx (never zero).
@export var phase7_touchdown_lock_sec: float = 0.10
@export var phase7_step_done_pos_eps: float = 2.0
@export var phase7_step_done_vel_eps: float = 20.0
## Max time in TOUCHDOWN before accepting "settled" (planted + low vel) and exiting even if foot missed target. Prevents stuck.
@export var phase7_step_done_settle_max_sec: float = 0.4

# (REMOVED) Phase 2 legacy auto-stepper exports deleted (planner/executor stepping replaces this).

# Swing foot PD (omega/zeta form, force-clamped by (m*g)*mult)
@export var phase2_swing_freq_hz: float = 7.0
@export var phase2_swing_zeta: float = 1.0
@export var phase2_swing_force_mult: float = 2.2

# Pelvis horizontal support (omega/zeta form, force-clamped by (total_mass*g)*mult)
@export var phase2_pelvis_x_freq_hz: float = 4.0
@export var phase2_pelvis_x_zeta: float = 1.0
@export var phase2_pelvis_x_force_mult: float = 0.90
@export var phase2_pelvis_x_deadband_px: float = 1.2
@export var phase2_idle_center_follow_hz: float = 8.0 # 4..12; higher = less snapback memory

# Support blend ramp (slow enough to not spike on touchdown)
@export var support_blend_in_hz: float = 10.0
@export var support_blend_out_hz: float = 18.0

# Touchdown/impact gate (stops damper term becoming a cannon)
@export var impact_ignore_d_sec: float = 0.06
@export var support_damp_vy_max: float = 80.0

# DO NOT set this tiny. Tiny guarantees collapse on touchdown because vy is hundreds of px/s.
@export var support_damp_vy_max_during_impact: float = 650.0

# While actively shifting stance, allow more damping authority than idle.
@export var support_damp_vy_max_shift: float = 220.0

# =============================================================================
# Vertical support (pelvis y)
# =============================================================================
@export_group("Phase A - Vertical Support")
@export var support_freq_hz: float = 3.6
@export var support_zeta: float = 1.25
@export var support_force_mult: float = 1.35
@export var support_down_mult: float = 0.40
@export var support_gravity_comp: float = 1.0

# NEW: make crouch speed tunable (and tied to stance_shift_time_sec via shift_scale)
@export var support_gravity_comp_crouch_mult: float = 0.25 # <1 lets gravity drop you faster while crouching
@export var support_gravity_comp_stand_mult: float = 1.0 # can be >1 if you want extra lift while standing
@export var support_contact_smooth_hz: float = 22.0
@export var support_y_deadband_px: float = 0.35
@export var support_mass_scale: float = 0.45
@export var grounded_grace_sec: float = 0.08
@export var support_hold_last_y_sec: float = 0.10
@export var support_allow_probe_during_spawn: bool = true

@export_group("Phase 2 - Fused Ground Truth")
@export var phase2_truth_hold_sec: float = 0.10 # HOLD_LAST time limit (tight)
@export var phase2_allow_hold_sec: float = 0.22 # slower latch for pins/support eligibility
@export var phase2_hold_max_gap_px: float = 12.0 # HOLD_LAST only if sole stayed near last y

@export var phase2_probe_enable: bool = true # controller emergency probes (tight)
@export var phase2_probe_up_px: float = 12.0
@export var phase2_probe_len_px: float = 40.0
@export var phase2_probe_max_gap_px: float = 8.0 # probe must be close to sole
@export var phase2_probe_min_up_dot: float = 0.35
@export var phase2_probe_stab_cap: float = 0.35 # probes publish low stability by design

@export var touchdown_spring_ramp_sec: float = 0.12
@export var touchdown_spring_min: float = 0.60 # spring_ramp floor at touchdown (prevents "no legs" frame)

# =============================================================================
# Phase 4 - Vertical Support Topology (landing + 0.2s crouch/stand)
# =============================================================================

# Landing sink: temporarily relax the pelvis height target downward after touchdown.
# Prevents "instant full height" demand while legs are wide (hop/scissor trigger).
@export var landing_sink_allow_px: float = 18.0 # base sink allowance right after touchdown (decays over landing_ramp_sec)
@export var landing_sink_extra_px: float = 14.0 # extra sink proportional to touchdown impact speed
@export var landing_sink_vy_ref: float = 520.0 # px/s downward for full extra sink
@export var landing_pelvis_clearance_px: float = 6.0 # never target closer than this to ground plane

# Downward authority boost during stance shifts (so 0.2s crouch/stand is real).
@export var support_down_mult_shift: float = 1.10

# Jerk limiting expressed in "g per second" (slew limit in force space).
@export var support_force_slew_g_per_sec: float = 10.0

# Baseline error clamps (steady standing)
@export var support_err_up_max_px: float = 6.0
@export var support_err_down_max_px: float = 3.0

# While actively shifting stance, allow larger errors so the spring can actually move the body.
# This makes stance_shift_time_sec a real speed knob in BOTH directions.
@export var support_err_up_max_px_shift: float = 80.0
@export var support_err_down_max_px_shift: float = 40.0

# Leg length (auto-calibrated)
@export var leg_len_stand: float = 42.0
@export var leg_len_crouch: float = 32.0
@export var stance_height01: float = 1.0 # alpha: 0=crouch, 1=upright

@export var leg_len_cal_min: float = 26.0
@export var leg_len_cal_max: float = 54.0
@export var leg_len_cal_lerp_hz: float = 6.0
@export var leg_len_cal_after_capture_sec: float = 0.20

# =============================================================================
# Posture / muscles
# =============================================================================
@export_group("Phase A - Upright + Muscles")
@export var pelvis_world_upright_k: float = 42000.0
@export var pelvis_world_upright_d: float = 900.0
@export var pelvis_world_upright_tau_max: float = 120000.0
@export var pelvis_upright_ramp_sec: float = 0.08

# Torso world upright assist: strong before rest capture, small after (anti-drift)
@export var torso_world_upright_k: float = 38000.0
@export var torso_world_upright_d: float = 1800.0
@export var torso_world_upright_tau_max: float = 90000.0
@export var torso_world_upright_pre_mult: float = 1.0
@export var torso_world_upright_post_mult: float = 0.20

@export var foot_world_k: float = 18000.0
@export var foot_world_d: float = 900.0
@export var foot_world_tau_max: float = 25000.0

@export_group("Phase 1B - Torso Upright")
@export var torso_upright_freq_hz: float = 7.0
@export var torso_upright_zeta: float = 1.0
@export var torso_upright_tau_max: float = 160000.0
@export var torso_inertia_radius_px: float = 18.0

# Internal joint PD (legs/spine) - ALWAYS ON (pre-rest uses initial snapshot)
@export var enable_posture_pd: bool = true
@export var k_limb: float = 12000.0
@export var d_limb: float = 650.0
@export var max_limb_tau: float = 38000.0

@export var hip_posture_scale: float = 0.40
@export var knee_posture_scale: float = 0.40
@export var hip_posture_min_gate_pre_support: float = 0.22

@export var stance_transition_gain_mult: float = 2.0
@export var stance_transition_tau_mult: float = 1.5
@export var landing_posture_d_mult: float = 2.0

@export var k_spine_posture: float = 70000.0
@export var d_spine_posture: float = 4200.0
@export var max_spine_posture_tau: float = 220000.0

@export var k_neck: float = 12000.0
@export var d_neck: float = 900.0
@export var max_neck_tau: float = 45000.0

@export var muscle_ramp_sec: float = 0.35
@export var posture_ramp_sec: float = 0.08

@export_group("Phase 1B - Landing Safety")
@export var knee_limit_min_gate_pre_support: float = 0.25 # 0..1 (knee limits never fully off)
@export var knee_posture_min_gate_pre_support: float = 0.10 # 0..1 (light knee pose damping in air)

# NEW: knees must be full-strength right after touchdown (prevents windmill)
@export var knee_touchdown_full_sec: float = 0.10
@export var impact_non_knee_limit_relax_mult: float = 0.35 # 0..1 relax hips/ankles/spine during impacts (knees stay strong)

@export var pre_rest_muscle_mult: float = 0.35
@export var pre_rest_posture_mult: float = 0.50

@export_group("Stance Templates - Tall (deg, facing right)")
@export var stance_tall_lead_hip_deg: float = 20.0
@export var stance_tall_lead_knee_deg: float = -10.0
# ankle ~ clamp(-(hip+knee), +/-ankle_limit) -> -(20 + -10) = -10
@export var stance_tall_lead_ankle_deg: float = -10.0

@export var stance_tall_trail_hip_deg: float = -15.0
@export var stance_tall_trail_knee_deg: float = -25.0
# ankle ~ clamp(-(hip+knee), +/-ankle_limit) -> -(-15 + -25) = +40 -> clamp to +35
@export var stance_tall_trail_ankle_deg: float = 35.0

@export_group("Stance Templates - Crouch (deg, facing right)")
@export var stance_crouch_lead_hip_deg: float = 110.0
@export var stance_crouch_lead_knee_deg: float = -10.0
# ankle ~ clamp(-(hip+knee), +/-ankle_limit) -> -(110 + -10) = -100 -> clamp to -35
@export var stance_crouch_lead_ankle_deg: float = -35.0

@export var stance_crouch_trail_hip_deg: float = -25.0
@export var stance_crouch_trail_knee_deg: float = -90.0
# ankle ~ clamp(-(hip+knee), +/-ankle_limit) -> -(-25 + -90) = +115 -> clamp to +35
@export var stance_crouch_trail_ankle_deg: float = 35.0

@export_group("Lead/Trail Identity")
@export var input_meaningful_threshold: float = 0.20 # cmd_move_x magnitude considered "real"
@export var vel_meaningful_threshold: float = 18.0 # px/s; ignore tiny vx noise
@export var lead_switch_margin_px: float = 6.0 # px; hysteresis vs jitter
@export var move_dir_hold_sec: float = 0.20 # sec; direction memory after release

# =============================================================================
# Rest capture (stability-gated)
# =============================================================================
@export_group("Phase A - Rest Capture Gate")
@export var rest_capture_min_sec: float = 0.25
@export var rest_capture_timeout_sec: float = 3.0
@export var rest_capture_max_pelvis_deg: float = 8.0
@export var rest_capture_max_angvel: float = 2.5
@export var rest_capture_min_stability: float = 0.65

@export_group("Rest Recapture")
@export var rest_recapture_enable: bool = true
@export var rest_recapture_min_interval_sec: float = 0.75
@export var rest_recapture_min_stable_sec: float = 0.20
@export var rest_recapture_min_stability: float = 0.85
@export var rest_recapture_max_angvel: float = 1.25
@export var rest_recapture_max_contact_dpos_px: float = 0.35
@export var rest_recapture_max_contact_dnorm: float = 0.04

# =============================================================================
# Foot sensors + grounding
# =============================================================================
@export_group("Sensors")
@export var ground_normal_min_up: float = 0.18

# =============================================================================
# Foot planting (springs) + anti-skate
# =============================================================================
@export_group("Foot Plant")
@export var enable_foot_plant: bool = true
@export var touchdown_confirm_sec: float = 0.04
@export var plant_ramp_sec: float = 0.12
@export var plant_min_stability: float = 0.35
@export var plant_pelvis_vy_max: float = 90.0
@export var plant_foot_vy_max: float = 120.0
@export var plant_touchdown_grace_sec: float = 0.25
@export var plant_allow_recenter_planted_targets: bool = true

@export var plant_pin_mode: int = GCTypes.PlantPinMode.HEEL_ONLY

@export var heel_x_front: float = 5.0
@export var toe_x_front: float = 5.0
@export var heel_x_rear: float = 5.5
@export var toe_x_rear: float = 5.5

@export var foot_cs_mid_front_path: NodePath = NodePath("Ragdoll/RB_Foot_Front/CS_FootMid")
@export var foot_cs_heel_front_path: NodePath = NodePath("Ragdoll/RB_Foot_Front/CS_FootHeel")
@export var foot_cs_toe_front_path: NodePath = NodePath("Ragdoll/RB_Foot_Front/CS_FootToe")
@export var foot_cs_mid_rear_path: NodePath = NodePath("Ragdoll/RB_Foot_Rear/CS_FootMid")
@export var foot_cs_heel_rear_path: NodePath = NodePath("Ragdoll/RB_Foot_Rear/CS_FootHeel")
@export var foot_cs_toe_rear_path: NodePath = NodePath("Ragdoll/RB_Foot_Rear/CS_FootToe")

@export var plant_freq_hz: float = 18.0
@export var plant_zeta: float = 1.0
@export var plant_pin_slack_px: float = 6.0
@export var plant_pin_slack_planted_px: float = 1.25
@export var crouch_trail_slide_stiff_mult: float = 0.22
@export var crouch_trail_slide_slack_mult: float = 2.5

# Drift glue (bounded)
@export var plant_drift_deadband_px: float = 1.5
@export var plant_drift_vel_gain: float = 18.0
@export_group("Foot Flatness")
@export var enable_foot_flatness: bool = true
@export var foot_flat_force_two_point_pins: bool = true # force HEEL_AND_TOE while planted

@export_group("Crouch Toe-Only (Trail Foot)")
@export var crouch_trail_toe_only_enable: bool = true
@export var crouch_trail_toe_only_alpha: float = 0.55 # stance alpha <= this => treat as crouch

# NEW: toe-only should not resist horizontal slide
@export var crouch_trail_toe_slide_enable: bool = true
@export var crouch_trail_toe_slide_follow_hz: float = 60.0 # dt*hz follow (0 = snap)
@export var foot_flat_omega: float = 14.0
@export var foot_flat_zeta: float = 1.0
@export var foot_flat_tau_max: float = 35000.0
@export var foot_flat_max_slope_deg: float = 50.0
## Smooth contact normal for flatness (0=full smooth, 1=no smooth). Reduces foot jitter from flickering normals.
@export var foot_flat_normal_smooth_alpha: float = 0.35
## Swing-foot attitude: PD gains and cap when foot is in air during Phase 7 step (flat target from last plant).
@export var swing_foot_flat_k: float = 8000.0
@export var swing_foot_flat_d: float = 400.0
@export var swing_foot_flat_tau_max: float = 12000.0
@export var swing_foot_flat_strength: float = 1.0
@export var plant_drift_vx_max: float = 35.0
@export var plant_drift_accel_gain: float = 35.0
@export var plant_drift_force_mult: float = 2.0
## When anchor-foot error (px) is above this, use plant_drift_recenter_force_mult so foot can follow anchor to stance width.
@export var plant_drift_recenter_err_px: float = 6.0
@export var plant_drift_recenter_force_mult: float = 4.0

# Extra damping while planted (force-based)
@export var foot_ground_lin_damp: float = 18.0
@export var foot_ground_lin_force_mult: float = 2.0
@export var foot_ground_ang_damp: float = 1200.0
@export var foot_ground_ang_tau_max: float = 12000.0

@export var slip_vx_warn: float = 30.0

@export_group("Stance Targeting (Anchors)")
@export var stance_anchor_move_hz: float = 8.0
## When feet are converged (narrow), use this higher Hz for anchor movement so stance width is restored faster. 0 = use stance_anchor_move_hz.
@export var stance_anchor_move_hz_when_converged: float = 18.0
@export var stance_recenter_min_stability: float = 0.55
## When anchors are converged (narrow stance), use this lower stability so recenter can run to spread feet. Normal recenter still uses plant_min_stability.
@export var plant_recenter_min_stability_when_converged: float = 0.15
## Cap anchor movement per frame (px). 0 = disabled. Use if anchor runs ahead of foot and glue cannot catch up.
@export var stance_anchor_max_move_px_per_frame: float = 0.0

# =============================================================================
# Soft joint limits (internal torques around rest snapshot)
# =============================================================================
@export_group("Soft Joint Limits")
@export var enable_soft_joint_limits: bool = true
@export var limit_k: float = 50000.0
@export var limit_d: float = 2500.0
@export var knee_limit_tau_mult: float = 3.5
@export var impact_knee_tau_mult: float = 2.5 # boost knee max torque during touchdown/impact to stop windmills
@export var limit_margin_deg: float = 6.0

@export var limit_hip_neg_deg: float = 120.0
@export var limit_hip_pos_deg: float = 120.0

# Knees: one-way hinge around "knee zero".
# limit_knee_neg_deg = max FLEX amount (magnitude)
# limit_knee_pos_deg = max HYPERextension allowed (usually 0)
# knee_flex_sign: set to +1 if your rig's knee flex is positive; -1 if flex is negative.
@export var limit_knee_neg_deg: float = 150.0
@export var limit_knee_pos_deg: float = 0.0

# One-way hinge slack around the 0Â° hyperextension bound (prevents torque spikes when near-straight).
@export var limit_oneway_neg_slack_deg: float = 3.0

@export var knee_flex_sign: int = -1 # must be -1 or +1

# Optional per-knee override (0 = inherit knee_flex_sign).
# "front" = camera/front leg (closer to user). "rear" = back leg.
@export var knee_flex_sign_front: int = 0 # -1 or +1, or 0 to inherit
@export var knee_flex_sign_rear: int = 0 # -1 or +1, or 0 to inherit

# While crouching, keep planted knees slightly bent (prevents near-straight buckle across the 0Â° bound).
@export var knee_planted_min_flex_deg: float = 8.0
@export var knee_minflex_posture_scale_mult: float = 3.0 # multiplies knee posture scale while enforcing min-flex (prevents knee inversion).

@export var limit_ankle_neg_deg: float = 35.0
@export var limit_ankle_pos_deg: float = 35.0

@export var limit_spine_neg_deg: float = 30.0
@export var limit_spine_pos_deg: float = 30.0

# =============================================================================
# Phase 5 - Solver health: spine limit gating + saturation flags
# =============================================================================

# Default spine soft limits OFF until you prove they don't fight upright / landing.
@export var phase5_spine_limits_enable: bool = false

# Re-enable spine limits only after this many seconds of "stable"
# (grounded + no impact + no saturation).
@export var phase5_spine_reenable_sec: float = 0.25

# Saturation threshold (ratio of effective max). Used by upright/support/limit booleans.
@export var phase5_sat_ratio: float = 0.90

@export var limit_neck_neg_deg: float = 35.0
@export var limit_neck_pos_deg: float = 35.0

# =============================================================================
# Stance (stand-only)
# =============================================================================
@export_group("Stance (Stand-only)")
@export var stance_w_stand: float = 16.0
@export var stance_w_crouch: float = 26.0
@export var stance_neutral_dx: float = 20.0
@export var stance_center_follow_hz: float = 1.5
@export var stance_center_margin_px: float = 6.0
@export var stance_anchor_deadband_px: float = 2.0

# =============================================================================
# Debug
# =============================================================================
@export_group("Debug")
@export var debug_enable: bool = true
@export var debug_print_hz: float = 15.0
@export var debug_draw: bool = true
@export var debug_multiline: bool = false
@export var debug_print_setup_on_start: bool = true
@export var debug_include_collision_setup: bool = true
@export var debug_include_limit_torque_est: bool = true

@export var landing_log_window_sec: float = 0.35
@export var landing_log_hz: float = 20.0

@export_group("Debug - Collision Sanity")
@export var debug_collision_sanity_on_start: bool = true
@export var debug_autofix_world_masks_on_start: bool = true
@export var debug_world_probe_len: float = 2000.0

@export_group("Debug - Mouse Drag")
@export var debug_mouse_drag_enable: bool = true
@export var debug_mouse_drag_button: int = 1 # 1=left, 2=right, 3=middle
@export var debug_mouse_drag_pelvis_only: bool = true

# Spring-damper in omega/zeta form (stable across dt)
@export var debug_mouse_drag_omega: float = 18.0 # rad/s (10..30)
@export var debug_mouse_drag_zeta: float = 1.0 # damping ratio (0.7..1.2)
@export var debug_mouse_drag_force_max: float = 250000.0

# While dragging, stop subsystems that fight user intent
@export var debug_mouse_drag_suspend_vertical_support: bool = true
@export var debug_mouse_drag_suspend_pins: bool = true
@export var debug_mouse_drag_suspend_rest_capture: bool = true

# =============================================================================
# Runtime cached physics constants / state
# =============================================================================
var _g: float = 980.0
var _total_mass: float = 1.0

# Phase 6 per-foot friction: cached base vs slide materials (assigned per-foot each frame).
var _pm_feet_base: PhysicsMaterial = null
var _pm_feet_slide: PhysicsMaterial = null

# Cache last applied values to avoid redundant property writes.
var _pm_feet_base_fric: float = NAN
var _pm_feet_slide_fric: float = NAN
var _pm_feet_base_bounce: float = NAN
var _pm_feet_slide_bounce: float = NAN

var _rb_list: Array[RigidBody2D] = []
var _rb_mass_sum: float = 1.0
@warning_ignore("unused_private_class_variable")
var _support_hysteresis_front: float = 0.0

@warning_ignore("unused_private_class_variable")
var _dbg_prev_grounded_eff: bool = false
@warning_ignore("unused_private_class_variable")
var _dbg_prev_front_g: bool = false
@warning_ignore("unused_private_class_variable")
var _dbg_prev_rear_g: bool = false
@warning_ignore("unused_private_class_variable")
var _dbg_prev_pins_f: bool = false
@warning_ignore("unused_private_class_variable")
var _dbg_prev_pins_r: bool = false

var _init_done: bool = false
var _refs: GCRefs = null # Shared refs for controller modules (filled in _post_ready_init, updated each tick)
var _input_mod: GCInput = null
var _facing_mod: GCFacing = null
var _debug_mod: GCDebug = null
var _vertical_support_mod: GCVerticalSupport = null
var _posture_mod: GCPostureLimits = null
var _stance_planner_mod: RefCounted = null # GCStancePlanner, created via load() to avoid preload resolve issues
var _step_planner_mod: RefCounted = null # GCStepPlanner, created via load() to avoid preload resolve issues
var _ground_truth_mod: RefCounted = null # GCGroundTruth
var _foot_plant_mod: GCFootPlant = null
var _recovery_mod: GCRecovery = null
var _locomotion_mod: GCLocomotion = null
var _runtime_mod: GCRuntime = null
var _builder_mod: RefCounted = null
var _math_mod: GCMath = null
var _t: float = 0.0
var _dbg_accum: float = 0.0
@warning_ignore("unused_private_class_variable")
var _dbg_setup_printed: bool = false
@warning_ignore("unused_private_class_variable")
var _dbg_opp_bound_printed: bool = false

@warning_ignore("unused_private_class_variable")
var _dbg_next_sample_t: float = 0.0

# Debug: nominal thigh<->shin center distances captured from rest snapshot (detects "knee separation" after impact)
var _dbg_rest_center_dist_knee_F: float = -1.0
var _dbg_rest_center_dist_knee_R: float = -1.0

# Debug: last-applied soft-limit info per label (KNEE_F/KNEE_R/SPINE/...)
# label -> { rel0_deg, min_deg, max_deg, excess_deg, tau, w }
var _dbg_lim: Dictionary = {}

var _support_blend: float = 0.0
var _pelvis_upright_blend: float = 0.0
var _muscle_blend: float = 0.0
var _posture_blend: float = 0.0

var _impact_timer: float = 0.0
var _grounded_prev: bool = false

var _ground_grace_t: float = 0.0
var _support_y_last_valid: float = NAN
var _support_hold_t: float = 0.0

# ---------------------------------------------------------------------------
# Phase 2 - Fused Ground Truth (SENSOR(valid) -> PROBE(tight) -> HOLD_LAST(tight))
# ---------------------------------------------------------------------------
var _truth_front_g: bool = false
var _truth_rear_g: bool = false
@warning_ignore("unused_private_class_variable")
var _truth_front_y: float = NAN
@warning_ignore("unused_private_class_variable")
var _truth_rear_y: float = NAN
var _truth_front_stab: float = 0.0
var _truth_rear_stab: float = 0.0
@warning_ignore("unused_private_class_variable")
var _truth_front_src: int = GCTypes.GroundTruthSrc.NONE
@warning_ignore("unused_private_class_variable")
var _truth_rear_src: int = GCTypes.GroundTruthSrc.NONE

# Truth latch (tight, short)
@warning_ignore("unused_private_class_variable")
var _truth_front_hold_t: float = 0.0
@warning_ignore("unused_private_class_variable")
var _truth_rear_hold_t: float = 0.0

# Eligibility latch (slower): used for pins/support gating (prevents 1-frame legs-off)
var _truth_front_allow_t: float = 0.0
var _truth_rear_allow_t: float = 0.0

# Debug flags (sensor inconsistency)
@warning_ignore("unused_private_class_variable")
var _dbg_truth_front_y_nan: bool = false
@warning_ignore("unused_private_class_variable")
var _dbg_truth_rear_y_nan: bool = false

var _plant_tick_id: int = 0
var _ray_exclude: Array[RID] = []

var _allow_plant_forces_this_frame: bool = true
var _plant_points_wired_ok: bool = false

var _dbg_anchor_moved_front: bool = false
var _dbg_anchor_moved_rear: bool = false
# Stance-width diagnostic: recenter skip 0=ran 1=no_allow 2=stability 3=deadband; want/now/dx and foot RB x
var _dbg_recenter_skip_f: int = -1
var _dbg_recenter_skip_r: int = -1
var _dbg_recenter_want_f: float = NAN
var _dbg_recenter_want_r: float = NAN
var _dbg_recenter_now_f: float = NAN
var _dbg_recenter_now_r: float = NAN
var _dbg_recenter_dx_f: float = NAN
var _dbg_recenter_dx_r: float = NAN
var _dbg_foot_x_f: float = NAN
var _dbg_foot_x_r: float = NAN
# Foot rotation debug: cur/tgt in deg, av = angular velocity rad/s, flatness normal when applied
var _dbg_foot_rot_deg_f: float = NAN
var _dbg_foot_rot_deg_r: float = NAN
var _dbg_foot_rot_av_f: float = NAN
var _dbg_foot_rot_av_r: float = NAN
var _dbg_foot_flat_tgt_deg_f: float = NAN
var _dbg_foot_flat_tgt_deg_r: float = NAN
var _dbg_foot_flat_err_deg_f: float = NAN
var _dbg_foot_flat_err_deg_r: float = NAN
var _dbg_foot_flat_nx_f: float = NAN
var _dbg_foot_flat_nx_r: float = NAN
var _dbg_foot_flat_ny_f: float = NAN
var _dbg_foot_flat_ny_r: float = NAN
var _dbg_phantom_planted_front: bool = false
var _dbg_phantom_planted_rear: bool = false
var _planted_heel_front: Vector2 = Vector2.ZERO
var _planted_toe_front: Vector2 = Vector2.ZERO
var _planted_heel_rear: Vector2 = Vector2.ZERO
var _planted_toe_rear: Vector2 = Vector2.ZERO
var _foot_flat_n_prev_f: Vector2 = Vector2.ZERO
var _foot_flat_n_prev_r: Vector2 = Vector2.ZERO
var _swing_flat_ref_angle_f: float = NAN
var _swing_flat_ref_angle_r: float = NAN

var _touchdown_ramp_t: float = 999.0
var _landing_t: float = 0.0
var _stand_y_target: float = NAN

var _sy_filt: float = NAN
var _support_y_raw: float = NAN
var _support_y_filt: float = NAN

var _leg_len_touch: float = NAN

var _leg_len_cal: float = 42.0
var _rest_captured: bool = false
var _rest_capture_timer: float = 0.0
var _rest_capture_elapsed: float = 0.0
var _rest_capture_time_since: float = 999.0

# Debug-latched values (so logs reflect actual control signals)
var _dbg_target_y: float = NAN
var _dbg_err_y: float = 0.0
var _dbg_vy_for_pd: float = 0.0
var _dbg_Fy_pre: float = 0.0
var _dbg_Fy_cmd: float = 0.0
var _dbg_Fy_max: float = 1.0

# Phase 4: vertical support slew state (jerk limiting)
var _vsupport_Fy_prev: float = 0.0

# Phase 4 debug (logs must prove sink + gating are behaving)
var _dbg_sink_allow: float = 0.0
var _dbg_vsupport_gate: float = 0.0

var _dbg_pelvis_target_x: float = NAN
var _dbg_Fx_pre: float = 0.0
var _dbg_Fx_cmd: float = 0.0
var _dbg_Fx_max: float = 1.0

var _dbg_vsupport_called: bool = false
var _dbg_px_called: bool = false
var _dbg_last_touch_vy: float = 0.0
@warning_ignore("unused_private_class_variable")
var _dbg_dx_to_enemy: float = NAN
var _dbg_landing_log_t: float = 0.0
var _dbg_landing_log_accum: float = 0.0

# Phase 5 - saturation flags (reset per tick; used for gating + diagnosis)
var _phase5_upright_saturated: bool = false
var _phase5_limit_saturated: bool = false
var _phase5_support_saturated: bool = false

# Phase 5 - spine re-enable stability timer
var _phase5_spine_stable_t: float = 0.0

# Phase 5 - per-frame debug latches (printed)
var _dbg_phase5_upr_sat: int = 0
var _dbg_phase5_lim_sat: int = 0
var _dbg_phase5_sup_sat: int = 0
var _dbg_phase5_spine_on: int = 0

# Phase 6 debug: slide state (set in stand-only block for status print)
var _dbg_front_slide: bool = false
var _dbg_rear_slide: bool = false

# Debug: posture targets (degrees, after facing sign + swing adjustments)
var _dbg_hipF_tgt_deg: float = 0.0
var _dbg_kneeF_tgt_deg: float = 0.0
var _dbg_ankleF_tgt_deg: float = 0.0
var _dbg_hipR_tgt_deg: float = 0.0
var _dbg_kneeR_tgt_deg: float = 0.0
var _dbg_ankleR_tgt_deg: float = 0.0
var _dbg_lead_is_front: int = 1
var _dbg_move_dir_world: float = 0.0

# =============================================================================
# Stand-only stance + support state
# =============================================================================
var support_is_front: bool = true

var _stance_alpha: float = 1.0
var _stance_alpha_prev: float = 1.0
var _stance_center_x: float = NAN

# Dynamic lead/trail semantic identity (mapped onto physical FRONT/REAR legs)
var _lead_is_front: bool = true
var _lead_initialized: bool = false
var _move_dir_world: float = 0.0
var _move_dir_hold_t: float = 0.0
var _move_dir_last: float = 0.0
var _neutral_axis_sign: int = 1 # -1/ +1. Stable axis for neutral targets when move_dir == 0.

var _allow_recenter_front: bool = false
var _allow_recenter_rear: bool = false

# Support candidate (hysteresis) state used by _choose_support_foot()
var _support_candidate_is_front: bool = true
var _support_candidate_t: float = 0.0

# Debug (stance targeting)
var _dbg_neutral_front_x: float = NAN
var _dbg_neutral_rear_x: float = NAN

# Debug (mouse drag)
var _drag_active: bool = false
@warning_ignore("unused_private_class_variable")
var _drag_rb: RigidBody2D = null
@warning_ignore("unused_private_class_variable")
var _drag_offset: Vector2 = Vector2.ZERO
@warning_ignore("unused_private_class_variable")
var _drag_target_world: Vector2 = Vector2.ZERO
@warning_ignore("unused_private_class_variable")
var _drag_was_applied_this_frame: bool = false
var _drag_prev_applied: bool = false

# =============================================================================
# Foot plant state machine
# =============================================================================
var _front_state: int = GCTypes.FootPlantState.SWING
var _rear_state: int = GCTypes.FootPlantState.SWING
var _front_candidate_t: float = 0.0
var _rear_candidate_t: float = 0.0
var _front_plant_blend: float = 0.0
var _rear_plant_blend: float = 0.0

# =============================================================================
# Phase 3 planner (explicit intent outputs; no stepping yet)
# =============================================================================
var _planner_mode: int = GCTypes.PlannerMode.IDLE

# Per-frame planner outputs (what downstream code must consume)
var _plan_front: int = GCTypes.PlanFoot.SWING
var _plan_rear: int = GCTypes.PlanFoot.SWING
var _plan_support_is_front: bool = true
var _plan_target_front: Vector2 = Vector2.ZERO
var _plan_target_rear: Vector2 = Vector2.ZERO

# Planner-level hysteresis (independent of sensor flicker)
var _plan_front_age: float = 0.0
var _plan_rear_age: float = 0.0

# =============================================================================
# Phase 7 - Stepping (planner + executor)
# =============================================================================

# Planner ownership (this frame)
var _phase7_plan_active: bool = false

# Planner state
var _phase7_state: int = GCTypes.Phase7LocState.IDLE
var _phase7_prev_cmd_active: bool = false
var _phase7_hold_t: float = 0.0
var _phase7_stop_pending: bool = false

# Frozen-at-sequence-start snapshots (for stability)
var _phase7_seq_dir_world: int = 0 # sign(cmd_move_x) at start
var _phase7_seq_dir_facing: int = 0 # sign(cmd_move_x * facing_sign) at start
var _phase7_seq_anat_front_is_front: bool = true
var _phase7_seq_gait_lead_is_front: bool = true

# Sequence progress
var _phase7_seq_step_index: int = 0 # TAP: 0..1, STOP: 0, WALK: 0..
var _phase7_seq_active_is_front: bool = true # which physical foot is the step foot
@warning_ignore("unused_private_class_variable")
var _phase7_stride_px: float = 0.0 # computed stride for current Phase7 step (planner)

# Clamp counters
var _phase7_clamp_cross_count: int = 0
var _phase7_clamp_reach_count: int = 0

# Executor handshake + overrides (driven in Brief D)
var _phase7_exec_phase: int = GCTypes.Phase7ExecPhase.IDLE
var _phase7_exec_is_front: bool = true
var _phase7_exec_target_w: Vector2 = Vector2.ZERO
var _phase7_exec_unplant_t: float = 0.0
@warning_ignore("unused_private_class_variable")
var _phase7_exec_swing_t: float = 0.0
var _phase7_exec_touchdown_lock_t: float = 0.0
var _phase7_exec_touchdown_elapsed: float = 0.0
var _phase7_exec_left_ground: bool = false
var _phase7_exec_done_pulse: bool = false
var _phase7_exec_done_is_front: bool = true

var _phase7_force_plant_front: bool = false
var _phase7_force_plant_rear: bool = false
var _phase7_slide_front: bool = false
var _phase7_slide_rear: bool = false
# Brief 4: per-foot Phase7 execution ownership contract (for subsystem gating / debug visibility).
var _phase7_exec_state_front: int = GCTypes.Phase7FootExecState.PLANTED
var _phase7_exec_state_rear: int = GCTypes.Phase7FootExecState.PLANTED

# Debug (printed in Brief D)
var _dbg_phase7_state: int = 0
var _dbg_phase7_exec_phase: int = 0
var _dbg_phase7_active_is_front: int = 0
var _dbg_phase7_seq_index: int = 0
@warning_ignore("unused_private_class_variable")
var _dbg_phase7_queue_len: int = 0
var _dbg_phase7_dir_w: int = 0
var _dbg_phase7_dir_f: int = 0
var _dbg_phase7_tap_lead_is_front: int = 0
var _dbg_phase7_tap_anat_front_is_front: int = 0
var _dbg_phase7_target_fx: float = NAN
var _dbg_phase7_target_rx: float = NAN
var _dbg_phase7_own: int = 0
var _dbg_phase7_slideF: int = 0
var _dbg_phase7_slideR: int = 0
var _dbg_phase7_dvx_clamped: int = 0
var _dbg_phase7_slide_fail: int = 0

# =============================================================================
# Step planner scaffold (5R2-A only; no behavior migration yet)
# =============================================================================
var _step_phase: int = GCTypes.StepPlannerPhase.DISABLED
var _step_phase_t: float = 0.0
var _step_id: int = 0

var _step_support_is_front: bool = true
var _step_has_swing: bool = false
var _step_swing_is_front: bool = true

var _step_front_slot_x: float = NAN
var _step_rear_slot_x: float = NAN
var _step_swing_target_x: float = NAN

# STEP_PLANNER slot-frame center publish (pipeline-owned).
# This is the chassis/reference-frame center consumed by stance slot generation.
# It must come from planner-owned center state (not raw pelvis body x) to avoid stale slot targets.
var _step_slot_center_x: float = NAN
var _step_slot_center_valid: bool = false

var _step_cmd_plant_front: bool = false
var _step_cmd_plant_rear: bool = false

# Brief 2: explicit planner command bus (separate release intent from plant intent).
# Pipeline/executor must consume release-vs-plant ownership from these commands,
# not infer everything from phase labels.
var _step_cmd_release_front: bool = false
var _step_cmd_release_rear: bool = false

var _step_timeout_active: bool = false
var _step_recover_reason: int = GCTypes.StepPlannerRecoverReason.NONE

# Planner-published ownership flags (reference-frame / correction policy / pre-landing shaping).
# These are planner outputs consumed by locomotion/pipeline to prevent cross-module churn.
var _step_ref_freeze_active: bool = false
var _step_preland_shape_active: bool = false
var _step_idle_inplace_correction: bool = false
var _step_idle_step_correction: bool = false
var _step_settle_lock_active: bool = false

# Debug / anti-leak counters (5R2-A)
var _dbg_movement_authority_mode: int = GCTypes.MovementAuthorityMode.LEGACY_PHASE7
var _dbg_step_planner_active: int = 0
var _dbg_legacy_phase7_planner_called: int = 0
var _dbg_dual_writer_step_phase: int = 0
var _dbg_dual_writer_swing_target: int = 0
var _dbg_root_progress_blocked_by_foot: int = 0

# 5R2-F: arbitration / exclusion debug (pipeline-owned)
var _dbg_foot_ctrl_mode_front: int = GCTypes.FootControlMode.DISABLED
var _dbg_foot_ctrl_mode_rear: int = GCTypes.FootControlMode.DISABLED
var _dbg_arb_slide_front: int = 0
var _dbg_arb_slide_rear: int = 0
var _dbg_arb_support_owner_step: int = 0
var _dbg_arb_excl_violations: int = 0
var _dbg_arb_phase7_helper_blocked: int = 0

# 5R2-J: authoritative per-frame arbitration publish (cross-module ownership contract).
# These are written by pipeline AFTER post-transition COMMIT, so downstream modules/debug can
# read the actual final modes used that frame (not provisional pre-step selections).
var _arb_front_ctrl_mode: int = GCTypes.FootControlMode.DISABLED
var _arb_rear_ctrl_mode: int = GCTypes.FootControlMode.DISABLED
var _arb_front_slide: bool = false
var _arb_rear_slide: bool = false
var _arb_ctrl_modes_valid: bool = false

# Brief B+: provisional pre-step arbitration publish (pipeline-owned migration bridge).
# step_foot_state() runs before post-transition COMMIT, so gc_foot_plant needs a same-frame,
# non-debug ctrl ownership channel. This is TEMPORARY and should be removed once pipeline/executor
# frame ordering no longer requires a provisional bridge.
var _arb_pre_front_ctrl_mode: int = GCTypes.FootControlMode.DISABLED
var _arb_pre_rear_ctrl_mode: int = GCTypes.FootControlMode.DISABLED
var _arb_pre_ctrl_modes_valid: bool = false

# Brief D: pipeline-published consumed slot authority (same-frame, non-debug).
# These are the FINAL slot targets consumed by arbitration/executor that frame after:
# - stance service selection
# - planner override
# - pipeline no-cross/prelanding clamps
# Executors must prefer this over legacy locomotion/Phase7 slot sources.
var _arb_slot_front_x: float = NAN
var _arb_slot_rear_x: float = NAN
var _arb_slot_targets_valid: bool = false

# Brief B+: STEP_PLANNER executor diagnostics (per-frame).
# ctrl source: 0=none/not-stepplanner, 1=committed, 2=provisional, 3=debug fallback
var _dbg_step_ctrl_src_f: int = 0
var _dbg_step_ctrl_src_r: int = 0
var _dbg_step_ctrl_src_debug_fallback_hits: int = 0

# Candidate entry denial bitmask (0 means can_enter=true):
# 1 plant_allowed
# 2 owner_wants_plant
# 4 grounded
# 8 stable_enough_or_override
# 16 pelvis_vy_ok
# 32 foot_vy_ok
# 64 can_plant_now
# 128 candidate_enter_allowed
# 256 stepplanner_land_gate_ok
var _dbg_step_can_enter_deny_mask_f: int = 0
var _dbg_step_can_enter_deny_mask_r: int = 0

# Arbitration-owned plant intent still ending in SWING (the exact mismatch we are cutting over).
var _dbg_step_owned_plant_but_swing_f: int = 0
var _dbg_step_owned_plant_but_swing_r: int = 0
var _dbg_step_owned_plant_but_swing_hits: int = 0

# 5R2-H: planner-slot shape invariant debug (STEP_PLANNER arbitration)
var _dbg_arb_shape_dual: int = 0
var _dbg_arb_shape_need_front: int = 0
var _dbg_arb_shape_need_rear: int = 0
var _dbg_step_slot_err_front: float = NAN
var _dbg_step_slot_err_rear: float = NAN
var _dbg_step_slot_sep: float = NAN

# Brief C: slot service / role-map diagnostics (service + pipeline-consumed slot truth).
# _dbg_slot_mapping_mode_dbg:
#   0 = none/invalid
#   1 = stance service posture-template path
#   2 = stance service width fallback path
#   3 = pipeline role-aware pelvis fallback (service invalid)
#   4 = planner override (_step_front/_step_rear_slot_x)
# _dbg_slot_center_src_dbg (set by stance planner service):
#   0 = unset/invalid
#   1 = _step_slot_center_x publish
#   2 = pelvis fallback inside STEP_PLANNER center selection
#   3 = legacy _stance_center_x
#   4 = final pelvis fallback
var _dbg_slot_src_valid: int = 0
var _dbg_slot_mapping_mode_dbg: int = 0
var _dbg_slot_center_src_dbg: int = 0
var _dbg_slot_role_map_valid: int = 0
var _dbg_slot_cross_blocked_count: int = 0
var _dbg_slot_preland_no_cross_applied: int = 0
var _dbg_slot_world_front_x_dbg: float = NAN
var _dbg_slot_world_rear_x_dbg: float = NAN
var _dbg_slot_sep_dbg: float = NAN
var _dbg_slot_sep_min_dbg: float = NAN
var _dbg_slot_front_minus_want: float = NAN
var _dbg_slot_rear_minus_want: float = NAN

# 5R2-I: swing translation ownership + landing gate diagnostics
var _dbg_arb_swing_force_front: int = 0
var _dbg_arb_swing_force_rear: int = 0
var _dbg_arb_swing_force_blocked: int = 0
var _dbg_land_gate_front: int = 1
var _dbg_land_gate_rear: int = 1
var _dbg_land_target_dx_front: float = NAN
var _dbg_land_target_dx_rear: float = NAN

# Brief A: final committed attitude ownership outputs (pipeline-owned).
var _arb_front_attitude_mode: int = GCTypes.FootAttitudeMode.DISABLED
var _arb_rear_attitude_mode: int = GCTypes.FootAttitudeMode.DISABLED
var _dbg_foot_att_owner_f: int = 0
var _dbg_foot_att_owner_r: int = 0

# 5R2-J3: swing lifetime latch (planner-consumed, exec-confirmed)
var _front_swing_ended_latch: bool = false
var _rear_swing_ended_latch: bool = false
var _prev_step_id: int = -1
var _prev_step_phase: int = -1

# Phase 6 / Spawn BRACE persistent state
var _phase6_recover_hold_t: float = 0.0
var _phase6_recover_exit_t: float = 0.0

var _spawn_brace_active: bool = false
var _spawn_brace_elapsed: float = 0.0
var _spawn_brace_ground_stable_t: float = 0.0

# Cached stance-change flag (set each _physics_process) so helpers can use it.
var _phase6_stance_changing: bool = false

var _cmd_stance_y_eff: float = 0.0

# (REMOVED) Phase 2 legacy auto-stepper state deleted.

var _plant_front_active: bool = false
var _plant_rear_active: bool = false
var _plant_front_x: float = 0.0
var _plant_rear_x: float = 0.0

# =============================================================================
# Rest snapshot (radians) - pre-rest uses initial snapshot; post-rest overwrites
# =============================================================================
var rest_pelvis_thigh_F: float = 0.0
var rest_thigh_shin_F: float = 0.0
var rest_shin_foot_F: float = 0.0
var rest_pelvis_thigh_R: float = 0.0
var rest_thigh_shin_R: float = 0.0
var rest_shin_foot_R: float = 0.0
var rest_pelvis_torso: float = 0.0
var rest_torso_head: float = 0.0

var _rest_seeded: bool = false

var _knee_zero_F: float = 0.0
var _knee_zero_R: float = 0.0
var _knee_zero_valid: bool = false

var _knee_flex_sign_F: int = -1
var _knee_flex_sign_R: int = -1

var _rest_recapture_timer: float = 0.0
var _rest_recapture_time_since: float = 999.0
var _rest_prev_cp_front: Vector2 = Vector2.ZERO
var _rest_prev_cp_rear: Vector2 = Vector2.ZERO
var _rest_prev_n_front: Vector2 = Vector2.UP
var _rest_prev_n_rear: Vector2 = Vector2.UP
var _rest_prev_valid: bool = false

# =============================================================================
# Node refs
# =============================================================================
@onready var _body_root: Node2D = get_node_or_null(body_root_path) as Node2D
@onready var _ragdoll_root: Node2D = get_node_or_null(ragdoll_root_path) as Node2D

@onready var _rb_pelvis: RigidBody2D = get_node_or_null(rb_pelvis_path) as RigidBody2D
@onready var _rb_pelvis_opponent: RigidBody2D = get_node_or_null(opponent_rb_pelvis_path) as RigidBody2D
@onready var _rb_torso: RigidBody2D = get_node_or_null(rb_torso_path) as RigidBody2D
@onready var _rb_head: RigidBody2D = get_node_or_null(rb_head_path) as RigidBody2D

@onready var _rb_thigh_front: RigidBody2D = get_node_or_null(rb_thigh_front_path) as RigidBody2D
@onready var _rb_shin_front: RigidBody2D = get_node_or_null(rb_shin_front_path) as RigidBody2D
@onready var _rb_foot_front: RigidBody2D = get_node_or_null(rb_foot_front_path) as RigidBody2D

@onready var _rb_thigh_rear: RigidBody2D = get_node_or_null(rb_thigh_rear_path) as RigidBody2D
@onready var _rb_shin_rear: RigidBody2D = get_node_or_null(rb_shin_rear_path) as RigidBody2D
@onready var _rb_foot_rear: RigidBody2D = get_node_or_null(rb_foot_rear_path) as RigidBody2D

@onready var _vis_torso: Node2D = get_node_or_null(torso_node_path) as Node2D
@onready var _vis_head: Node2D = get_node_or_null(head_node_path) as Node2D
@onready var _vis_upper_front: Node2D = get_node_or_null(upper_front_node_path) as Node2D
@onready var _vis_lower_front: Node2D = get_node_or_null(lower_front_node_path) as Node2D
@onready var _vis_upper_rear: Node2D = get_node_or_null(upper_rear_node_path) as Node2D
@onready var _vis_lower_rear: Node2D = get_node_or_null(lower_rear_node_path) as Node2D
@onready var _vis_upper_leg_front: Node2D = get_node_or_null(upper_leg_front_node_path) as Node2D
@onready var _vis_lower_leg_front: Node2D = get_node_or_null(lower_leg_front_node_path) as Node2D
@onready var _vis_upper_leg_rear: Node2D = get_node_or_null(upper_leg_rear_node_path) as Node2D
@onready var _vis_lower_leg_rear: Node2D = get_node_or_null(lower_leg_rear_node_path) as Node2D

# Foot sensors (script attached on foot RBs)
var _foot_sensor_front: FootContactSensor = null
var _foot_sensor_rear: FootContactSensor = null

# Foot plant point nodes + cached local points (NO PlantAnchors / NO PlantPins)
var _cs_mid_f: CollisionShape2D = null
var _cs_heel_f: CollisionShape2D = null
var _cs_toe_f: CollisionShape2D = null
var _cs_mid_r: CollisionShape2D = null
var _cs_heel_r: CollisionShape2D = null
var _cs_toe_r: CollisionShape2D = null

var _pt_mid_f: Vector2 = Vector2.ZERO
var _pt_heel_f: Vector2 = Vector2.ZERO
var _pt_toe_f: Vector2 = Vector2.ZERO
var _pt_mid_r: Vector2 = Vector2.ZERO
var _pt_heel_r: Vector2 = Vector2.ZERO
var _pt_toe_r: Vector2 = Vector2.ZERO

# =============================================================================
# READY / INIT
# =============================================================================
func _ready() -> void:
	if _ragdoll_root == null or _rb_pelvis == null:
		push_error("Missing ragdoll root or RB_Pelvis. Check NodePaths.")
		return
	call_deferred("_post_ready_init")

func _post_ready_init() -> void:
	if _builder_mod == null:
		_builder_mod = _preload_gc_builder.new()
		_builder_mod.setup_host(self)
	if _math_mod == null:
		_math_mod = _preload_gc_math.new()
	_builder_mod.post_ready_init()

func _reset_runtime_state() -> void:
	if _builder_mod != null:
		_builder_mod.reset_runtime_state()
	_touchdown_ramp_t = 999.0
	_landing_t = 0.0
	_stand_y_target = NAN

	_sy_filt = NAN
	_support_y_raw = NAN
	_support_y_filt = NAN
	_leg_len_touch = NAN

	_leg_len_cal = leg_len_stand

	_rest_captured = false
	_rest_seeded = false
	_rest_recapture_timer = 0.0
	_rest_recapture_time_since = 999.0
	_rest_prev_cp_front = Vector2.ZERO
	_rest_prev_cp_rear = Vector2.ZERO
	_rest_prev_n_front = Vector2.UP
	_rest_prev_n_rear = Vector2.UP
	_rest_prev_valid = false
	_rest_capture_timer = 0.0
	_rest_capture_elapsed = 0.0
	_rest_capture_time_since = 999.0

	# Stand-only state
	support_is_front = true
	_support_candidate_is_front = support_is_front
	_support_candidate_t = 0.0

	_stance_alpha = 1.0
	_stance_center_x = NAN

	# Lead/trail is dynamic (movement dir + foot X ordering). Initialize unknown until first update.
	_lead_is_front = true
	_lead_initialized = false
	_move_dir_world = 0.0
	_move_dir_hold_t = 0.0
	_move_dir_last = 0.0
	_neutral_axis_sign = 1

	# phase 3
	_planner_mode = GCTypes.PlannerMode.BRACE if spawn_brace_enable else GCTypes.PlannerMode.IDLE
	_plan_front = GCTypes.PlanFoot.SWING
	_plan_rear = GCTypes.PlanFoot.SWING
	_plan_support_is_front = true
	_plan_target_front = Vector2.ZERO
	_plan_target_rear = Vector2.ZERO

	# initialize ages so first transition is not artificially blocked
	_plan_front_age = phase3_min_swing_sec
	_plan_rear_age = phase3_min_swing_sec

	# Phase 7 reset
	_phase7_plan_active = false
	_phase7_state = GCTypes.Phase7LocState.IDLE
	_phase7_prev_cmd_active = false
	_phase7_hold_t = 0.0
	_phase7_stop_pending = false

	_phase7_seq_dir_world = 0
	_phase7_seq_dir_facing = 0
	_phase7_seq_anat_front_is_front = true
	_phase7_seq_gait_lead_is_front = true
	_phase7_seq_step_index = 0
	_phase7_seq_active_is_front = true

	_phase7_clamp_cross_count = 0
	_phase7_clamp_reach_count = 0

	_phase7_exec_phase = GCTypes.Phase7ExecPhase.IDLE
	_phase7_exec_is_front = true
	_phase7_exec_target_w = Vector2.ZERO
	_phase7_exec_unplant_t = 0.0
	_phase7_exec_touchdown_lock_t = 0.0
	_phase7_exec_touchdown_elapsed = 0.0
	_phase7_exec_left_ground = false
	_phase7_exec_done_pulse = false
	_phase7_exec_done_is_front = true

	_phase7_force_plant_front = false
	_phase7_force_plant_rear = false
	_phase7_exec_state_front = GCTypes.Phase7FootExecState.SWING
	_phase7_exec_state_rear = GCTypes.Phase7FootExecState.SWING
	_phase7_slide_front = false
	_phase7_slide_rear = false

	_dbg_phase7_state = 0
	_dbg_phase7_exec_phase = 0
	_dbg_phase7_active_is_front = 0
	_dbg_phase7_seq_index = 0
	_dbg_phase7_dir_w = 0
	_dbg_phase7_dir_f = 0
	_dbg_phase7_tap_lead_is_front = 0
	_dbg_phase7_tap_anat_front_is_front = 0
	_dbg_phase7_target_fx = NAN
	_dbg_phase7_target_rx = NAN
	_dbg_phase7_own = 0
	_dbg_phase7_slideF = 0
	_dbg_phase7_slideR = 0
	_dbg_phase7_dvx_clamped = 0
	_dbg_phase7_slide_fail = 0

	# 5R2-A step-planner scaffold reset (no behavior yet)
	_step_phase = GCTypes.StepPlannerPhase.DISABLED
	_step_phase_t = 0.0
	_step_id = 0
	_step_support_is_front = true
	_step_has_swing = false
	_step_swing_is_front = true
	_step_front_slot_x = NAN
	_step_rear_slot_x = NAN
	_step_swing_target_x = NAN
	_step_slot_center_x = NAN
	_step_slot_center_valid = false
	_step_cmd_plant_front = false
	_step_cmd_plant_rear = false
	_step_cmd_release_front = false
	_step_cmd_release_rear = false
	_step_timeout_active = false
	_step_recover_reason = GCTypes.StepPlannerRecoverReason.NONE
	_step_ref_freeze_active = false
	_step_preland_shape_active = false
	_step_idle_inplace_correction = false
	_step_idle_step_correction = false
	_step_settle_lock_active = false

	_dbg_movement_authority_mode = movement_authority_mode
	_dbg_step_planner_active = 0
	_dbg_legacy_phase7_planner_called = 0
	_dbg_dual_writer_step_phase = 0
	_dbg_dual_writer_swing_target = 0
	_dbg_root_progress_blocked_by_foot = 0

	_dbg_arb_slide_front = 0
	_dbg_arb_slide_rear = 0
	_dbg_arb_support_owner_step = 0
	_dbg_arb_excl_violations = 0
	_dbg_arb_phase7_helper_blocked = 0

	_arb_front_ctrl_mode = GCTypes.FootControlMode.DISABLED
	_arb_rear_ctrl_mode = GCTypes.FootControlMode.DISABLED
	_arb_front_slide = false
	_arb_rear_slide = false
	_arb_ctrl_modes_valid = false

	_arb_pre_front_ctrl_mode = GCTypes.FootControlMode.DISABLED
	_arb_pre_rear_ctrl_mode = GCTypes.FootControlMode.DISABLED
	_arb_pre_ctrl_modes_valid = false

	_arb_slot_front_x = NAN
	_arb_slot_rear_x = NAN
	_arb_slot_targets_valid = false

	_dbg_step_ctrl_src_f = 0
	_dbg_step_ctrl_src_r = 0
	_dbg_step_ctrl_src_debug_fallback_hits = 0
	_dbg_step_can_enter_deny_mask_f = 0
	_dbg_step_can_enter_deny_mask_r = 0
	_dbg_step_owned_plant_but_swing_f = 0
	_dbg_step_owned_plant_but_swing_r = 0
	_dbg_step_owned_plant_but_swing_hits = 0

	_dbg_arb_shape_dual = 0
	_dbg_arb_shape_need_front = 0
	_dbg_arb_shape_need_rear = 0
	_dbg_step_slot_err_front = NAN
	_dbg_step_slot_err_rear = NAN
	_dbg_step_slot_sep = NAN

	_dbg_slot_src_valid = 0
	_dbg_slot_mapping_mode_dbg = 0
	_dbg_slot_center_src_dbg = 0
	_dbg_slot_role_map_valid = 0
	_dbg_slot_cross_blocked_count = 0
	_dbg_slot_preland_no_cross_applied = 0
	_dbg_slot_world_front_x_dbg = NAN
	_dbg_slot_world_rear_x_dbg = NAN
	_dbg_slot_sep_dbg = NAN
	_dbg_slot_sep_min_dbg = NAN
	_dbg_slot_front_minus_want = NAN
	_dbg_slot_rear_minus_want = NAN

	# 5R2-I
	_dbg_arb_swing_force_front = 0
	_dbg_arb_swing_force_rear = 0
	_dbg_arb_swing_force_blocked = 0
	_dbg_land_gate_front = 1
	_dbg_land_gate_rear = 1
	_dbg_land_target_dx_front = NAN
	_dbg_land_target_dx_rear = NAN

	_arb_front_attitude_mode = GCTypes.FootAttitudeMode.DISABLED
	_arb_rear_attitude_mode = GCTypes.FootAttitudeMode.DISABLED
	_dbg_foot_att_owner_f = 0
	_dbg_foot_att_owner_r = 0

	_front_swing_ended_latch = false
	_rear_swing_ended_latch = false
	_prev_step_id = -1
	_prev_step_phase = -1

	# Phase 6 / Spawn BRACE state
	_phase6_recover_hold_t = 0.0
	_phase6_recover_exit_t = 0.0

	_spawn_brace_active = spawn_brace_enable
	_spawn_brace_elapsed = 0.0
	_spawn_brace_ground_stable_t = 0.0

	_phase6_stance_changing = false

	_allow_recenter_front = false
	_allow_recenter_rear = false

	_dbg_neutral_front_x = NAN
	_dbg_neutral_rear_x = NAN

	_dbg_hipF_tgt_deg = 0.0
	_dbg_kneeF_tgt_deg = 0.0
	_dbg_ankleF_tgt_deg = 0.0
	_dbg_hipR_tgt_deg = 0.0
	_dbg_kneeR_tgt_deg = 0.0
	_dbg_ankleR_tgt_deg = 0.0
	_dbg_lead_is_front = 1
	_dbg_move_dir_world = 0.0

	_front_state = GCTypes.FootPlantState.SWING
	_rear_state = GCTypes.FootPlantState.SWING
	_front_candidate_t = 0.0
	_rear_candidate_t = 0.0
	_front_plant_blend = 0.0
	_rear_plant_blend = 0.0
	_plant_front_active = false
	_plant_rear_active = false
	_plant_front_x = 0.0
	_plant_rear_x = 0.0

# =============================================================================
# Phase 7 - Stepping (planner)
# =============================================================================
func _movement_authority_uses_step_planner() -> bool:
	return movement_authority_mode == GCTypes.MovementAuthorityMode.STEP_PLANNER

func _legacy_phase7_force_inert() -> void:
	_phase7_plan_active = false
	_phase7_exec_phase = GCTypes.Phase7ExecPhase.IDLE
	_phase7_exec_done_pulse = false
	_phase7_exec_left_ground = false
	_phase7_force_plant_front = false
	_phase7_force_plant_rear = false
	_phase7_slide_front = false
	_phase7_slide_rear = false
	_dbg_phase7_own = 0

func _step_planner_tick_shadow(cmd_move_x_norm: float, dt: float, front_g: bool, rear_g: bool, grounded_eff: bool, stabF: float, stabR: float) -> void:
	if _step_planner_mod == null:
		return
	_step_planner_mod.tick_shadow(cmd_move_x_norm, dt, front_g, rear_g, grounded_eff, stabF, stabR)

func _phase7_update_locomotion_plan(cmd_move_x_norm: float, dt: float, front_g: bool, rear_g: bool, grounded_eff: bool, _stabF: float, _stabR: float) -> void:
	if _movement_authority_uses_step_planner():
		_dbg_legacy_phase7_planner_called += 1
		_legacy_phase7_force_inert()
		return
	if _locomotion_mod == null:
		return
	_locomotion_mod.update_locomotion_plan(cmd_move_x_norm, dt, front_g, rear_g, grounded_eff, _stabF, _stabR)

# =============================================================================
# Phase 7 - Stepping (executor)
# =============================================================================

func _phase7_exec_update_state(dt: float, front_g: bool, rear_g: bool) -> void:
	if _movement_authority_uses_step_planner():
		_legacy_phase7_force_inert()
		return
	if _locomotion_mod == null:
		return
	_locomotion_mod.exec_update_state(dt, front_g, rear_g)

func _phase7_exec_apply_forces(dt: float, spawn_gate: float, front_g: bool, rear_g: bool) -> void:
	if _movement_authority_uses_step_planner():
		_legacy_phase7_force_inert()
		return
	if _locomotion_mod == null:
		return
	_locomotion_mod.exec_apply_forces(dt, spawn_gate, front_g, rear_g)

func _support_y_from_support_foot(front_g: bool, rear_g: bool) -> float:
	return _vertical_support_mod.support_y_from_support_foot(front_g, rear_g) if _vertical_support_mod != null else NAN

func _physics_process(delta: float) -> void:
	# Dispatcher. Old logic moved behind _physics_process_impl (same body as before).
	if not _init_done or not enable_controller:
		return

	# Match the old dt clamp semantics exactly (previously done inside _physics_process_impl).
	var dt: float = maxf(0.00001, float(delta))

	# Runtime module owns tick ordering from this point forward.
	if _runtime_mod != null:
		_runtime_mod.tick(dt)
	else:
		return


func _input(event: InputEvent) -> void:
	if _input_mod != null:
		_input_mod.handle_input(event)

func _debug_apply_mouse_drag(dt: float) -> bool:
	return _debug_mod.debug_apply_mouse_drag(dt) if _debug_mod != null else false

func _can_plant_now(foot: RigidBody2D) -> bool:
	if foot == null or _rb_pelvis == null:
		return false

	# During touchdown/impact, ALLOW planting to prevent windmills.
	# The spring stiffness is ramped, so this does not hard-snap.
	if _impact_timer > 0.0 or _touchdown_ramp_t < plant_touchdown_grace_sec:
		return true

	if absf(_rb_pelvis.linear_velocity.y) > plant_pelvis_vy_max:
		return false
	if absf(foot.linear_velocity.y) > plant_foot_vy_max:
		return false
	return true

# Returns local Y coordinate of the sole (bottom) of the first direct CollisionShape2D under this foot RB.
# Planting must pin the sole, not the foot's midline (midline pins force the foot into the floor and jitter).
func _foot_sole_local_y(foot: RigidBody2D) -> float:
	if foot == null:
		return 0.0
	var cs: CollisionShape2D = null
	for c in foot.get_children():
		if c is CollisionShape2D:
			var t := c as CollisionShape2D
			if t.disabled or t.shape == null:
				continue
			cs = t
			break
	if cs == null:
		return 0.0
	var sh: Shape2D = cs.shape
	var half_h: float = 0.0
	if sh is RectangleShape2D:
		half_h = (sh as RectangleShape2D).size.y * 0.5
	elif sh is CapsuleShape2D:
		var cap := sh as CapsuleShape2D
		half_h = cap.radius + cap.height * 0.5
	elif sh is CircleShape2D:
		half_h = (sh as CircleShape2D).radius
	else:
		# Fallback: bounding rect (may be imperfect, but better than midline).
		half_h = sh.get_rect().size.y * 0.5
	return cs.position.y + half_h

# =============================================================================
# Leg posture + soft limits
# =============================================================================
func _limit_deltas_from_rest(rest_rel: float, neg_deg: float, pos_deg: float) -> Vector2:
	if _posture_mod != null:
		return _posture_mod.limit_deltas_from_rest(rest_rel, neg_deg, pos_deg)
	var min_rel := -deg_to_rad(neg_deg)
	var max_rel := deg_to_rad(pos_deg)
	var a := min_rel - rest_rel
	var b := max_rel - rest_rel
	if a <= b:
		return Vector2(a, b)
	return Vector2(b, a)

func _apply_rel_limit_soft(
	parent_rb: RigidBody2D,
	child_rb: RigidBody2D,
	rest_rel: float,
	min_delta: float,
	max_delta: float,
	k: float,
	d: float,
	max_tau: float,
	scale_val: float,
	parent_share: float = 0.10
) -> void:
	if _posture_mod != null:
		_posture_mod.apply_rel_limit_soft(parent_rb, child_rb, rest_rel, min_delta, max_delta, k, d, max_tau, scale_val, parent_share)

func _apply_rel_limit_soft_abs0(
	label: String,
	child_rb: RigidBody2D,
	parent_rb: RigidBody2D,
	zero_rel: float,
	min_abs0: float,
	max_abs0: float,
	k: float,
	d: float,
	max_tau: float,
	limit_scale: float
) -> void:
	if _posture_mod != null:
		_posture_mod.apply_rel_limit_soft_abs0(label, child_rb, parent_rb, zero_rel, min_abs0, max_abs0, k, d, max_tau, limit_scale)

# =============================================================================
# PD helpers
# =============================================================================
func _apply_joint_pd_child_only(
	parent_rb: RigidBody2D,
	child_rb: RigidBody2D,
	target_rel: float,
	k: float,
	d: float,
	max_tau: float,
	scale_val: float,
	parent_share: float = 0.10
) -> void:
	if _posture_mod != null:
		_posture_mod.apply_joint_pd_child_only(parent_rb, child_rb, target_rel, k, d, max_tau, scale_val, parent_share)

# =============================================================================
# =============================================================================
# Draw debug: targets + support lines + posture cue
# =============================================================================
func _draw() -> void:
	if _debug_mod == null:
		return
	_debug_mod.draw_overlay()

# =============================================================================
# Sensor wiring
# =============================================================================
func _get_sensor_from_foot(foot_rb: RigidBody2D) -> FootContactSensor:
	if foot_rb == null:
		return null
	if foot_rb is FootContactSensor:
		return foot_rb as FootContactSensor
	for c: Node in foot_rb.get_children():
		if c is FootContactSensor:
			return c as FootContactSensor
	return null

func _configure_sensors() -> void:
	# Ignore only ragdoll subtree, not the whole controller tree (safer).
	var ignore_root: Node = _ragdoll_root

	if _foot_sensor_front != null:
		_foot_sensor_front.world_layer_index = L_WORLD
		_foot_sensor_front.accept_any_static = ground_accept_any_static
		_foot_sensor_front.ground_normal_min_up = ground_normal_min_up
		_foot_sensor_front.ignore_root = ignore_root
	if _foot_sensor_rear != null:
		_foot_sensor_rear.world_layer_index = L_WORLD
		_foot_sensor_rear.accept_any_static = ground_accept_any_static
		_foot_sensor_rear.ground_normal_min_up = ground_normal_min_up
		_foot_sensor_rear.ignore_root = ignore_root

func _phase6_refresh_foot_materials() -> void:
	if _recovery_mod != null:
		_recovery_mod.refresh_foot_materials()
		return
	if _pm_feet_base == null:
		_pm_feet_base = PhysicsMaterial.new()
	if _pm_feet_slide == null:
		_pm_feet_slide = PhysicsMaterial.new()
	var base_f: float = maxf(0.0, feet_friction)
	var slide_f: float = maxf(0.0, phase6_slide_friction)
	var b: float = feet_bounce
	if _pm_feet_base_fric != base_f:
		_pm_feet_base.friction = base_f
		_pm_feet_base_fric = base_f
	if _pm_feet_base_bounce != b:
		_pm_feet_base.bounce = b
		_pm_feet_base_bounce = b
	if _pm_feet_slide_fric != slide_f:
		_pm_feet_slide.friction = slide_f
		_pm_feet_slide_fric = slide_f
	if _pm_feet_slide_bounce != b:
		_pm_feet_slide.bounce = b
		_pm_feet_slide_bounce = b

func _phase6_apply_foot_friction(front_slide: bool, rear_slide: bool) -> void:
	if _recovery_mod != null:
		_recovery_mod.apply_foot_friction(front_slide, rear_slide)
		return
	_phase6_refresh_foot_materials()
	if _rb_foot_front != null:
		var want_f: PhysicsMaterial = _pm_feet_slide if front_slide else _pm_feet_base
		if _rb_foot_front.physics_material_override != want_f:
			_rb_foot_front.physics_material_override = want_f
	if _rb_foot_rear != null:
		var want_r: PhysicsMaterial = _pm_feet_slide if rear_slide else _pm_feet_base
		if _rb_foot_rear.physics_material_override != want_r:
			_rb_foot_rear.physics_material_override = want_r
