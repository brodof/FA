extends Node

func _ready() -> void:
	# P1 movement
	_ensure_action("p1_left", KEY_A)
	_ensure_action("p1_right", KEY_D)
	_ensure_action("p1_up", KEY_W)
	_ensure_action("p1_down", KEY_S)
	# P1 arm
	_ensure_action("p1_arm_left", KEY_F)
	_ensure_action("p1_arm_right", KEY_H)
	_ensure_action("p1_arm_up", KEY_T)
	_ensure_action("p1_arm_down", KEY_G)

	# P2 movement
	_ensure_action("p2_left", KEY_LEFT)
	_ensure_action("p2_right", KEY_RIGHT)
	_ensure_action("p2_up", KEY_UP)
	_ensure_action("p2_down", KEY_DOWN)
	# P2 arm (J=left, B=up, N=down, M=right)
	_ensure_action("p2_arm_left", KEY_J)
	_ensure_action("p2_arm_right", KEY_M)
	_ensure_action("p2_arm_up", KEY_B)
	_ensure_action("p2_arm_down", KEY_N)

	_ensure_action("start_duel", KEY_ENTER)

func _ensure_action(action_name: String, keycode: int) -> void:
	if not InputMap.has_action(action_name):
		InputMap.add_action(action_name)
	for ev in InputMap.action_get_events(action_name):
		if ev is InputEventKey and ev.physical_keycode == (keycode as Key):
			return
	var e := InputEventKey.new()
	e.physical_keycode = (keycode as Key) # cast int → Key enum
	InputMap.action_add_event(action_name, e)
