# res://scripts/duel/gladiator_controller/gc_types.gd
# Shared enums for gladiator controller modules. No behavior; types only.
class_name GCTypes

enum PlantPinMode { HEEL_ONLY, TOE_ONLY, HEEL_AND_TOE }
enum FootPlantState { SWING, CANDIDATE, PLANTED }
enum GroundTruthSrc { NONE, SENSOR, PROBE, HOLD }
enum PlannerMode { IDLE, BRACE, RECOVER }
enum PlanFoot { SWING, PLANTED }
enum Phase7LocState { IDLE, TAP_SEQ, WALK_SEQ, STOP_SEQ }
enum Phase7ExecPhase { IDLE, UNPLANT, SWING, TOUCHDOWN }
# Brief 4: per-foot execution ownership contract states (derived from Phase7 exec + contact truth).
enum Phase7FootExecState {
	PLANTED,
	RELEASE,
	SWING,
	LANDING_CANDIDATE,
	PLANTED_CONFIRMED
}

# 5R2-A: explicit locomotion authority switch (legacy Phase7 vs new step planner path).
enum MovementAuthorityMode { LEGACY_PHASE7, STEP_PLANNER }

# 5R2-A scaffold only (no behavior yet).
enum StepPlannerPhase {
	DISABLED,
	DOUBLE_SUPPORT,
	SINGLE_SUPPORT_SWING,
	TRANSFER,
	RECOVER
}

# 5R2-A scaffold only (debug + future recover routing).
enum StepPlannerRecoverReason {
	NONE,
	LANDING_TIMEOUT,
	SLOT_COLLAPSE,
	SUPPORT_LOSS,
	EMERGENCY_REPLANT
}

# 5R2-F: pipeline arbitration output (single owner per foot per frame).
enum FootControlMode {
	DISABLED,         # no normal foot control authority this frame
	PLANTED_SUPPORT,  # planted support foot hold (no recenter)
	PLANTED_CORRECT,  # planted corrective foot (recenter allowed)
	SWING,            # swing/release/landing transit (no plant/recenter/plant-flatness)
	RECOVERY          # recovery override owns foot behavior this frame
}

# Brief A: foot attitude torque ownership is a separate channel from foot control ownership.
# This prevents "ctrl_mode == DISABLED" from creating no-attitude-owner gaps in STEP_PLANNER mode.
enum FootAttitudeMode {
	DISABLED,         # no attitude torque owner (avoid in STEP_PLANNER except fail-safe)
	AIR_PARALLEL,     # keep ankle-down parallel using airborne/held reference (last valid plane / neutral)
	GROUND_PARALLEL   # keep ankle-down parallel to current ground/contact plane reference
}
