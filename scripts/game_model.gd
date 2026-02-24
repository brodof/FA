extends Node
# Autoload name: GameModel

var inventory: Array[Weapon] = []
var current_weapon_p1: Weapon = null
var current_weapon_p2: Weapon = null

# weapon being worked on (forge -> handle mount)
var pending_weapon: Weapon = null

func _ready() -> void:
	_load_inventory()

func _load_inventory() -> void:
	var s: Node = get_node_or_null("/root/Storage")
	if s != null:
		var loaded: Array = s.load_all_weapons()
		print("GameModel: loaded weapons from storage: ", loaded.size())
		for item in loaded:
			if item is Weapon:
				var w: Weapon = item
				inventory.append(w)
	else:
		push_error("GameModel: Storage autoload not found at /root/Storage")

	# If nothing loaded, create a starter blade only in memory
	if inventory.is_empty():
		var w: Weapon = Weapon.new()
		w.name_ = "Starter Blade"
		inventory.append(w)

func add_weapon(w: Weapon) -> void:
	if w != null:
		inventory.append(w)

func find_weapon_by_uuid(u: String) -> Weapon:
	for w in inventory:
		if w.uuid == u:
			return w
	return null
