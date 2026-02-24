# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

Forge Arena is a skill-based mobile multiplayer dueling and forging game built with **Godot 4.5.1** (GDScript). The main scenes are:

- **Hub** (`scenes/hub/Hub.tscn`) — inventory, weapon selection, entry point (main scene)
- **Forge** (`scenes/forge/Forge.tscn`) — weapon crafting
- **Duel** (`scenes/duel/Duel.tscn`) — 2-player combat arena
- **HandleMount** (`scenes/handle/HandleMount.tscn`) — weapon assembly

### Running the game

```
godot --rendering-driver opengl3
```

In headless/CI environments (no GPU):
```
godot --headless
```

The game starts at the Hub scene. Click **Quick Start** to assign the Starter Blade to both players and transition into the Duel scene.

### GDScript validation (lint)

Validate a single script:
```
godot --headless --check-only --script scripts/game_model.gd
```

Validate all scripts:
```
find scripts scenes -name "*.gd" -type f -exec godot --headless --check-only --script {} \;
```

### Re-importing assets

After pulling new art assets, re-import before running:
```
godot --headless --import
```

### Key autoloads

Defined in `project.godot`:
- `Storage` — persists weapons to `user://weapons/`
- `GameModel` — inventory, weapon selection state
- `InputSetup` — keyboard bindings for P1 (WASD + FTGH) and P2 (arrows + JBNM)

### Gotchas

- The `project.godot` file was not originally committed. If it's missing, the project cannot open in Godot. It must define the three autoloads above and set `run/main_scene` to `res://scenes/hub/Hub.tscn`.
- Art assets live in `art/` and have corresponding `.import` files. Missing art causes non-fatal errors at runtime (UI renders without textures).
- No audio hardware in cloud VMs — Godot falls back to the dummy audio driver automatically.
- Use `--rendering-driver opengl3` for software-rendered (llvmpipe) environments; Vulkan is not available.
