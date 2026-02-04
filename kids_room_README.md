# Kids Room MuJoCo Simulation Scene

> A realistic children's room simulation environment for robot manipulation research and algorithm testing

![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## Project Overview

This project provides a high-quality, physically accurate MuJoCo simulation scene of a children's room, specifically designed for robot manipulation research and algorithm development. The scene features a realistic room layout with multiple functional areas, various interactive objects, and dual robotic arms for performing organization tasks.

### Core Features

- **Physically Accurate**: High-precision physics simulation based on the MuJoCo engine
- **Visually Realistic**: Carefully designed materials, colors, and lighting system
- **Modular Design**: Structured scene with distinct functional areas
- **Robot Compatible**: Equipped with dual robotic arms for manipulation tasks
- **Rich Objects**: 140+ interactive toys and objects for testing
- **Ready to Use**: Complete scene file with test scripts and documentation

---

## Scene Specifications

### Room Dimensions

| Parameter | Value |
|-----------|-------|
| Length | 10m |
| Width | 12m |
| Height | 3.2m |
| Total Area | 80m² |

### Functional Areas

The room is divided into four distinct functional areas with specific area ratios:

```
┌─────────────────────────────────────────────────────────┐ North Wall
│  ┌───────────┐  ┌───────────────────────┐  ┌─────────┐  │
│  │ Storage   │  │      Rest Area        │  │ Study   │  │
│  │   15%     │  │       25%             │  │  20%    │  │
│  │  (3x4m)   │  │      (5x4m)           │  │ (4x4m)  │  │
│  └───────────┘  └───────────────────────┘  └─────────┘  │
├─────────────────────────────────────────────────────────┤ Transition
│                    Play Area (20%)                      │
│                      (5x3.2m)                           │
└─────────────────────────────────────────────────────────┘ South Wall
                    12m (Width)
```

| Area | Ratio | Dimensions | Location |
|------|-------|------------|----------|
| Rest Area | 25% | 5m × 4m | Upper center |
| Storage Area | 15% | 3m × 4m | Upper left |
| Study Area | 20% | 4m × 4m | Upper right |
| Play Area | 20% | 5m × 3.2m | Lower section |
| Transition | 10% | - | Between areas |

---

## Functional Areas Detail

### 1. Rest Area (Upper Center)

A comfortable sleeping area with a complete bed setup and storage furniture.

**Furniture:**
| Item | Dimensions | Material | Position |
|------|------------|----------|----------|
| Bed | 2.4m × 1.2m × 0.5m | Wood | Area center |
| Headboard | 1.3m × 0.05m × 0.4m | Wood | Bed head |
| Mattress | 1.1m × 1.7m × 0.15m | Foam | On bed frame |
| Pillow | 0.5m × 0.6m × 0.08m | Fabric | On mattress |
| Blanket | 1.1m × 1.7m × 0.02m | Fabric | On mattress |
| Bedside Cabinets | 0.35m × 0.35m × 0.6m | Wood | Both sides of bed |
| Wardrobe | 1m × 0.5m × 1.8m | Wood | Area right |
| Chest of Drawers | 0.5m × 0.4m × 1.0m | Wood | Area corner |

**Features:**
- Warm color scheme (cream, light pink)
- Two bedside lamps for ambient lighting
- Four-drawer chest with metal handles
- Wardrobe with internal shelves and doors

### 2. Storage Area (Upper Left)

A dedicated storage space with wardrobes, toy racks, and organized storage boxes. Initial state is intentionally messy.

**Furniture:**
| Item | Dimensions | Material | Position |
|------|------------|----------|----------|
| Large Wardrobe | 1m × 0.3m × 1.1m | Wood (dark brown) | Area corner |
| Toy Rack 1 | 0.75m × 0.2m × 0.9m | Metal/Plastic | Area center |
| Toy Rack 2 | 0.75m × 0.2m × 0.9m | Metal/Plastic | Area center |
| Storage Box 1 | 0.3m × 0.2m × 0.15m | Plastic (blue) | Floor |
| Storage Box 2 | 0.3m × 0.2m × 0.15m | Plastic (green) | Floor |
| Storage Box 3 | 0.3m × 0.2m × 0.15m | Plastic (pink) | Floor |
| Decorative Plant | 0.3m × 0.3m × 0.8m | Ceramic/Leaves | Area corner |

**Initial State:** Messy
- 60 toys scattered on the floor
- Storage boxes with lids
- Toys in various colors (red, orange, blue, green, purple, etc.)

### 3. Study Area (Upper Right)

A tidy learning space with desk, chair, bookshelf, and study materials.

**Furniture:**
| Item | Dimensions | Material | Position |
|------|------------|----------|----------|
| Desk | 0.7m × 0.35m × 0.375m | Wood | Area center |
| Chair | 0.225m × 0.225m × 0.4m | Plastic | Desk front |
| Bookshelf | 0.9m × 0.2m × 1.0m | Wood | Area left |
| Bookshelf (Main) | 0.8m × 0.3m × 2.0m | Wood | Area corner |
| Reading Chair | 0.3m × 0.3m × 0.4m | Plastic | Next to bookshelf |
| Side Table | 0.2m × 0.2m × 0.3m | Wood | Next to chair |

**Initial State:** Tidy
- 45 books neatly arranged on study bookshelf
- 36 books on main bookshelf (9 per shelf, 4 shelves)
- Desk lamp on desk
- Decorative items on bookshelf

### 4. Play Area (Lower Section)

A dedicated play space with a play mat, slide, blocks table, and numerous toys. Initial state is intentionally messy.

**Furniture:**
| Item | Dimensions | Material | Position |
|------|------------|----------|----------|
| Play Mat | 1.5m × 1.2m × 0.025m | Foam (tan) | Area center |
| Slide Base | 1m × 0.5m × 0.6m | Plastic (red) | Area left |
| Blocks Table | 0.4m × 0.4m × 0.25m | Wood | Area center |
| Building Blocks | 0.06m × 0.06m × 0.06m | Plastic (4 colors) | On table |
| Kids Sofa | 0.4m × 0.3m × 0.25m | Plastic (lavender) | Area corner |

**Initial State:** Messy
- 80 toys scattered on the play mat and floor
- Various toy colors and types
- Building blocks on table

---

## Interactive Objects

### Toys Collection

The scene contains **140 interactive toys** for robot manipulation tasks:

| Category | Count | Size | Material | Colors |
|----------|-------|------|----------|--------|
| Storage Toys | 60 | 0.08m (radius) | Plastic | Multi-colored |
| Play Toys | 80 | 0.08m (radius) | Plastic | Multi-colored |
| Building Blocks | 4 | 0.06m cube | Plastic | Red, Green, Blue, Yellow |

### Books Collection

The scene contains **81 books** for testing object manipulation:

| Location | Count | Dimensions | Colors |
|----------|-------|------------|--------|
| Study Bookshelf | 45 | 0.1m × 0.075m × 0.01m | Multi-colored |
| Main Bookshelf | 36 | 0.1m × 0.2m × 0.25m | Multi-colored |

---

## Robotic Arms

The scene is equipped with **two UR5-style robotic arms** designed for performing organization tasks in storage and play areas.

### Robot Specifications

| Parameter | Value |
|-----------|-------|
| Type | UR5-style (simplified) |
| Degrees of Freedom | 6 |
| Mounting | Ceiling-mounted |
| Control Type | Position control |

### Robot Configuration

| Robot | Mounting Position | Working Area | Tasks |
|-------|-------------------|--------------|-------|
| Robot 1 | Storage area ceiling | Storage + Rest areas | Organize toys, sort objects |
| Robot 2 | Play area ceiling | Play area | Clean up toys, arrange blocks |

**Note:** Actuator configuration is prepared for custom control implementation.

---

## Lighting System

The scene features a comprehensive lighting setup with 5 light sources:

| Light | Position | Direction | Diffuse | Specular | Purpose |
|-------|----------|-----------|---------|----------|---------|
| Ambient Top | (5, 6, 3.5) | Down | 0.8, 0.8, 0.8 | 0.3, 0.3, 0.3 | General illumination |
| North Soft | (5, 0, 3.5) | Down | 0.6, 0.6, 0.6 | 0.2, 0.2, 0.2 | Wall lighting |
| South Soft | (5, 12, 3.5) | Down | 0.6, 0.6, 0.6 | 0.2, 0.2, 0.2 | Wall lighting |
| West Soft | (0, 6, 3.5) | Down | 0.6, 0.6, 0.6 | 0.2, 0.2, 0.2 | Wall lighting |
| East Soft | (10, 6, 3.5) | Down | 0.6, 0.6, 0.6 | 0.2, 0.2, 0.2 | Wall lighting |
| Ceiling Light | (5, 6, 3.5) | Down | 1.0, 1.0, 1.0 | 0.6, 0.6, 0.6 | Main light source |

---

## Materials and Textures

The scene uses 10 material classes for different object types:

| Class | Color | Friction | Applications |
|-------|-------|----------|--------------|
| floor | Light gray/cream | 1.0, 0.005, 0.0001 | Room floor, floor decorations |
| wall | Light pink | - | All walls |
| ceiling | Off-white | - | Ceiling |
| furniture | Gray | 0.8, 0.03, 0.001 | General furniture |
| wood | Cream/light brown | 0.8, 0.03, 0.001 | Beds, cabinets, shelves |
| metal | Silver-gray | 0.9, 0.02, 0.001 | Handles, frames, robot arms |
| plastic | Light gray-blue | 0.6, 0.02, 0.001 | Toys, storage boxes |
| toy | Multi-colored | 0.6, 0.01, 0.001 | All interactive toys |
| lamp | Warm yellow | - | Lamp fixtures |
| arm | Steel blue | 0.9, 0.02, 0.001 | Robot arm components |

---

## Room Decorations

### Floor Design

The entire room floor features a decorative 9×11 checkerboard pattern with soft pastel colors:
- Color palette: Light blue, light pink, light green, cream
- Each tile: 1m × 1m × 0.001m
- Creates a warm, child-friendly atmosphere

### Wall Features

| Feature | Position | Dimensions | Description |
|---------|----------|------------|-------------|
| North Window | North wall | 1.5m × 0.8m | Blue-tinted glass with metal frames |
| Paintings (4) | All walls | 0.6m × 0.4m | Colorful wall decorations |
| Shelves (4) | All walls | 0.4m × 0.2m | Decorative wall shelves |

---

## Task Scenarios

### Recommended Tasks

1. **Toy Sorting**: Robot picks up scattered toys and places them in storage boxes or on toy racks
2. **Book Organization**: Robot arranges books on shelves in proper order
3. **Play Area Cleanup**: Robot clears the play mat and organizes blocks
4. **General Tidying**: Robot moves objects between areas based on type
5. **Object Retrieval**: Robot fetches specific toys or books on command

### Task Complexity Levels

| Level | Description | Example |
|-------|-------------|---------|
| Simple | Single object pickup and placement | Pick up one toy, place in box |
| Medium | Multiple objects, same type | Sort all red toys to one area |
| Complex | Multiple objects, classification | Sort toys by color, size, and location |

---

## Physics Settings

### Simulation Parameters

| Parameter | Value |
|-----------|-------|
| Gravity | 0, 0, -9.81 m/s² |
| Integrator | RK4 |
| Timestep | Default (0.002s) |

### Friction Coefficients

| Surface | Static | Dynamic | Viscous |
|---------|--------|---------|---------|
| Floor | 1.0 | 0.005 | 0.0001 |
| Wood | 0.8 | 0.03 | 0.001 |
| Metal | 0.9 | 0.02 | 0.001 |
| Plastic | 0.6 | 0.02 | 0.001 |
| Toys | 0.6 | 0.01 | 0.001 |

---

## Usage Instructions

### Requirements

- **Python**: 3.8 or higher
- **MuJoCo**: 3.0 or higher
- **Dependencies**: `mujoco`, `numpy`

### Installation

```bash
pip install mujoco numpy
```

### Running the Scene

```python
import mujoco
import mujoco.viewer

# Load the scene
model = mujoco.MjModel.from_xml_path("scenes/kids_room_mujoco.1.22.xml")
data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### Using Test Scripts

```bash
# Test with default scene
python scripts/test_scene.py -f kids_room_mujoco.1.22.xml

# Validate scene syntax
python scripts/validate_scene.py kids_room_mujoco.1.22.xml
```

---

## Scene Statistics

| Category | Count |
|----------|-------|
| Total Bodies | ~200+ |
| Geometries | ~300+ |
| Materials | 10 classes |
| Lights | 6 |
| Toys | 140 |
| Books | 81 |
| Furniture Pieces | ~30 |

---

## File Structure

```
01/
├── scenes/
│   └── kids_room_mujoco.1.22.xml    # Main scene file
├── scripts/
│   ├── test_scene.py                # Scene testing script
│   └── validate_scene.py            # Scene validation script
├── docs/
│   └── kids_room_documentation.md   # Detailed documentation (optional)
└── README.md                        # This file
```

---

## Customization Guide

### Adding New Objects

To add new interactive objects to the scene:

```xml
<body name="new_object" pos="x y z">
  <geom class="plastic" type="box" size="0.1 0.1 0.1" rgba="0.9 0.5 0.5 1"/>
</body>
```

### Modifying Materials

Edit the `<default>` section to change material properties:

```xml
<default>
  <default class="plastic">
    <geom friction="0.7 0.03 0.001" rgba="0.8 0.8 0.9 1"/>
  </default>
</default>
```

### Adjusting Lighting

Modify light parameters in the `<worldbody>` section:

```xml
<light name="new_light" pos="x y z" dir="0 0 -1"
       diffuse="0.9 0.9 0.9" specular="0.5 0.5 0.5"/>
```

---

## Development Notes

### Scene Version

- **Version**: 1.22
- **Last Updated**: 2026-01-22
- **Status**: Production ready

### Known Limitations

1. Robot actuators require custom control implementation
2. No collision groups configured (all objects collide by default)
3. Static scene (no dynamic elements like doors that open)

### Future Enhancements

- Add door/window opening mechanisms
- Implement robot control interfaces
- Add more interactive furniture
- Include collision groups for selective collision
- Add sensors (touch, force, vision)

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- [MuJoCo](https://mujoco.org/) - Physics engine by Google DeepMind
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - Robot model inspiration

---

**Last Updated**: 2026-01-22
**Scene Version**: 1.22
