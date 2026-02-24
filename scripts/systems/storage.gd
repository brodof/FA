extends Node

const WEAPON_DIR: String = "user://weapons"

func _ready() -> void:
	_ensure_weapon_dir()

func _ensure_weapon_dir() -> void:
	var root: DirAccess = DirAccess.open("user://")
	if root == null:
		push_error("Storage: Could not open user://")
		return

	if not root.dir_exists("weapons"):
		var err: int = root.make_dir("weapons")
		if err != OK:
			push_error("Storage: Failed to create 'weapons' directory (code %d)" % err)

func _make_safe_name(raw_name: String) -> String:
	var base: String = raw_name.strip_edges()
	if base == "":
		base = "blade"
	base = base.to_lower()

	var sb := ""
	for i in range(base.length()):
		var ch: String = base.substr(i, 1)
		var code: int = ch.unicode_at(0)
		var is_digit: bool = (code >= 48 and code <= 57)   # '0'..'9'
		var is_lower: bool = (code >= 97 and code <= 122)  # 'a'..'z'
		if is_digit or is_lower or ch == "_":
			sb += ch
		else:
			sb += "_"

	var suffix: String = "%d" % int(Time.get_unix_time_from_system())
	return "%s_%s" % [sb, suffix]

func save_weapon(w: Weapon, raw_name: String) -> String:
	_ensure_weapon_dir()

	var safe_name: String = _make_safe_name(raw_name)
	var path: String = "%s/%s.tres" % [WEAPON_DIR, safe_name]

	# Godot 4: ResourceSaver.save(resource, path)
	var err: int = ResourceSaver.save(w, path)
	if err != OK:
		push_error("Storage: Failed to save weapon to %s (code %d)" % [path, err])
		return ""

	print("Storage: saved weapon to ", path)
	return path

func load_all_weapons() -> Array:
	_ensure_weapon_dir()

	var out: Array = []
	var dir: DirAccess = DirAccess.open(WEAPON_DIR)
	if dir == null:
		push_error("Storage: Could not open %s" % WEAPON_DIR)
		return out

	var files: PackedStringArray = dir.get_files()
	print("Storage: files in weapons dir: ", files)

	for f in files:
		if not (f.ends_with(".tres") or f.ends_with(".res")):
			continue
		var full: String = "%s/%s" % [WEAPON_DIR, f]
		var res: Resource = ResourceLoader.load(full)
		if res == null:
			push_error("Storage: Failed to load %s" % full)
			continue
		if res is Weapon:
			var w: Weapon = res
			w.saved_path = full
			out.append(w)

	print("Storage: loaded weapons count: ", out.size())
	return out
