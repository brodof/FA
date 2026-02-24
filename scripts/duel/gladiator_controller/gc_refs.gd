# res://scripts/duel/gladiator_controller/gc_refs.gd
# Shared refs + per-frame state. Filled by main controller each tick.
# Modules read from refs; they do not hold node paths.
class_name GCRefs
extends RefCounted

# --- Node refs (resolved in main _ready) ---
var rb_pelvis: RigidBody2D = null
var rb_torso: RigidBody2D = null
var rb_head: RigidBody2D = null
var rb_thigh_front: RigidBody2D = null
var rb_shin_front: RigidBody2D = null
var rb_foot_front: RigidBody2D = null
var rb_thigh_rear: RigidBody2D = null
var rb_shin_rear: RigidBody2D = null
var rb_foot_rear: RigidBody2D = null
var opponent_pelvis: RigidBody2D = null
var ragdoll_root: Node2D = null
var body_root: Node2D = null

# Foot contact sensors (script on foot RBs)
var foot_sensor_front: Node = null  # FootContactSensor
var foot_sensor_rear: Node = null   # FootContactSensor

# --- Per-frame inputs (set by main each _physics_process) ---
var dt: float = 0.0
var front_g: bool = false
var rear_g: bool = false
var grounded_eff: bool = false
var stabF: float = 0.0
var stabR: float = 0.0
var support_y_raw: float = NAN
var support_y_filt: float = NAN
# Plant support Y per foot (written by foot plant module; read by vertical support)
var plant_support_y_front: float = NAN
var plant_support_y_rear: float = NAN
var stance_alpha: float = 0.0
var stance_height01: float = 0.0
var cmd_move_x_norm: float = 0.0
var cmd_stance_y_eff: float = 0.0
var planner_mode: int = 0  # GCTypes.PlannerMode

# Gravity (set once in main _ready)
var g: float = 980.0

# Total ragdoll mass (set once)
var total_mass: float = 0.0

# RB list + mass sum (for COM; set after build_rb_list)
var rb_list: Array = []  # Array[RigidBody2D]
var rb_mass_sum: float = 0.0
