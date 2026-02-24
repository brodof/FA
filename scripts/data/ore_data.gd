extends Resource
class_name OreData

# Each ore: stat modifiers, display color, initial bar proportions
# width_frac/height_frac are relative to the anvil grid (0..1)
var ores := {
	"Iron": {
		"mods": {"damage":0.00, "weight":0.00, "control":0.00, "speed":0.00, "edge":0.00},
		"color": Color8(200, 205, 215),
		"width_frac": 0.60, "height_frac": 0.15, "thickness": 0.70
	},
	"Steel": {
		"mods": {"damage":0.06, "weight":0.04, "control":0.04, "speed":-0.04, "edge":0.02},
		"color": Color8(185, 190, 210),
		"width_frac": 0.62, "height_frac": 0.14, "thickness": 0.72
	},
	"Bronze": {
		"mods": {"damage":-0.04, "weight":-0.02, "control":0.02, "speed":0.03, "edge":0.00},
		"color": Color8(198, 140, 72),
		"width_frac": 0.56, "height_frac": 0.18, "thickness": 0.68
	},
	"Mithril": {
		"mods": {"damage":0.03, "weight":-0.10, "control":0.08, "speed":0.08, "edge":0.06},
		"color": Color8(160, 230, 255),
		"width_frac": 0.58, "height_frac": 0.12, "thickness": 0.62
	},
	"Obsidian": {
		"mods": {"damage":0.10, "weight":0.02, "control":-0.06, "speed":-0.02, "edge":0.10},
		"color": Color8(40, 40, 48),
		"width_frac": 0.52, "height_frac": 0.16, "thickness": 0.66
	}
}

func get_modifiers(name: String) -> Dictionary:
	if ores.has(name):
		return ores[name]["mods"]
	return {"damage":0.0,"weight":0.0,"control":0.0,"speed":0.0,"edge":0.0}

func get_color(name: String) -> Color:
	return ores[name]["color"] if ores.has(name) else Color(0.85,0.85,0.90)

func get_bar_preset(name: String) -> Dictionary:
	if ores.has(name):
		return {
			"width_frac": float(ores[name]["width_frac"]),
			"height_frac": float(ores[name]["height_frac"]),
			"thickness": float(ores[name]["thickness"])
		}
	return {"width_frac":0.60, "height_frac":0.15, "thickness":0.70}
