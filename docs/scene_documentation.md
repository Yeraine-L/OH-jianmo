# Home Complete Scene - Enhanced Multi-Room Environment

## Overview

`home_complete.xml` is a comprehensive, high-fidelity home simulation scene designed for embodied AI research. It features multiple rooms with interactive elements, realistic physics, and extensive support for robotic manipulation tasks.

## Scene Composition

### Rooms (5 Total)

1. **Entrance (Foyer)**
   - Location: Front-left corner (-5, -4)
   - Features: Shoe cabinet, coat rack, decorative vase
   - Floor: Ceramic tile

2. **Living Room**
   - Location: Central area (-1.5, -3)
   - Features: Sofa with cushions, coffee table with objects, TV stand with interactive doors, bookshelf
   - Floor: Wood flooring with carpet
   - Interactive: TV cabinet doors (left/right), moveable books, remote control

3. **Kitchen**
   - Location: Right side (2.5, -3.5)
   - Features: Counter with sink, stove, refrigerator, dining table with chairs
   - Floor: Ceramic tile
   - Interactive: Cabinet doors (left/right), moveable utensils (bowl, cup, plate, glass, fruits)

4. **Bedroom**
   - Location: Upper-left (-2, 0.5)
   - Features: Bed with pillows, two nightstands with drawers, wardrobe, dresser with mirror
   - Floor: Wood flooring
   - Interactive: Nightstand drawers (slide-out), alarm clock

5. **Bathroom** ⭐ NEW
   - Location: Left of bedroom (-5.2, 0.5)
   - Features: Toilet with lift seat, sink with cabinet, shower area with glass enclosure, towel rack, storage shelf
   - Floor: Non-slip ceramic tile
   - Interactive: Toilet seat, sink cabinet door, moveable items (toothbrush holder, soap, shampoo, body wash, towel)

### Doors (4 Total, All Interactive)

All doors feature realistic hinge physics and can be controlled by robots:

1. **Main Entrance Door** - Heavy dark wood door with handle
2. **Bedroom Door** - Light wood door
3. **Kitchen Door** - Light wood door
4. **Bathroom Door** ⭐ NEW - Translucent frosted glass door

### Windows (4 Total)

1. **Living Room Window** - Large window with curtains (decorative)
2. **Bedroom Window** ⭐ NEW - Openable window with hinge mechanism
3. **Kitchen Window** - Small fixed window
4. **Bathroom Window** ⭐ NEW - Frosted glass window for privacy

## Interactive Elements

### Doors and Cabinets (13 Actuators Total)

- 4 Room doors (hinged, 90° range)
- 2 TV stand doors (hinged)
- 2 Kitchen cabinet doors (hinged)
- 2 Nightstand drawers (slide-out, 15cm range)
- 1 Bathroom sink cabinet door (hinged)
- 1 Toilet seat (hinged, 100° range)
- 1 Bedroom window (hinged, 90° range)

### Moveable Objects (20+ Items)

Perfect for robotic manipulation tasks:

**Living Room:**
- Teapot (mesh model)
- 2 Books
- Tea cup
- Cushion
- TV remote

**Kitchen:**
- Bowl
- Cup
- Plate
- Glass
- Apple
- Banana

**Bedroom:**
- 2 Pillows
- Alarm clock

**Bathroom:**
- Toothbrush holder
- Soap bar
- Shampoo bottle
- Body wash bottle
- Towel

## Robot Task Support

### Navigation Points

- 3 mobile robot navigation waypoints in living room
- Clear pathways between all rooms
- Door passages for navigation training

### Manipulation Task Sites (16 Markers)

**Room-specific task sites:**
- Kitchen: counter, sink, stove, dining table
- Living room: coffee table, TV stand
- Bedroom: 2 nightstands, bed
- Bathroom: sink, toilet

**Grasp target markers:**
- Teapot, cup, plate, bowl (for pick-and-place tasks)

### Robot Spawn Areas

- **Humanoid robot spawn**: Entrance area
- **Robotic arm workspace**: Living room (platform marker)
- **Mobile robot**: Multiple navigation points

## Lighting System

5 strategically placed lights for optimal visibility:

1. **Main ceiling light** (0, -1, 2.8) - Central illumination
2. **Living room light** (-1, -3, 2.5) - Ambient fill
3. **Kitchen light** (2.5, -3.5, 2.5) - Task lighting
4. **Bedroom light** (-2, 0.5, 2.3) - Warm tone
5. **Bathroom light** ⭐ NEW (-5.2, 0.5, 2.3) - Bright white light

## Camera Views (7 Angles)

- `main_view`: Overall scene perspective
- `top_view`: Bird's eye view
- `living_room_view`: Living room focus
- `kitchen_view`: Kitchen area
- `bedroom_view`: Bedroom corner
- `entrance_view`: Front door area
- `bathroom_view`: ⭐ NEW Bathroom interior

## Materials and Textures

Rich visual appearance with 9 distinct materials:

- Wood flooring (checker pattern with edge marks)
- Ceramic tile (smooth, reflective)
- Carpet (soft, high friction)
- Wall paint (matte finish)
- Wallpaper (gradient)
- Dark wood (furniture, high specular)
- Light wood (furniture, medium specular)
- Fabric (sofa, low reflectance)
- Metal (appliances, high reflectance)
- Ceramic (bathroom fixtures)

## Physical Properties

- **Gravity**: Standard Earth gravity (9.81 m/s²)
- **Timestep**: 0.002s (500 Hz)
- **Collision detection**: Optimized friction coefficients for different surfaces
- **Object masses**: Realistic (e.g., teapot 0.5kg, sofa 48kg, bed 60kg)

## Usage

### Quick Start

```bash
# Validate scene syntax
python scripts/validate_scene.py home_complete.xml

# Launch interactive viewer (requires MuJoCo)
python scripts/test_scene.py -s home_complete
```

### Integration with Robots

To add a robot to the scene, include it in the `worldbody` at one of the spawn sites:

```xml
<include file="path/to/robot.xml"/>
<body name="robot" pos="-5 -4.5 0">
  <!-- Robot definition or reference -->
</body>
```

## Statistics

- **Total bodies**: 65
- **Geometries**: 181
- **Joints**: 14 (all actuated)
- **Assets**: 21 (textures + materials + meshes)
- **Sites**: 20+ (task markers)
- **Actuators**: 13

## Scene Dimensions

- **Total floor area**: ~14m × 12m
- **Ceiling height**: 2.5m
- **Room sizes**:
  - Living room: 5m × 4m
  - Kitchen: 3m × 3m
  - Bedroom: 4m × 3m
  - Bathroom: 2.4m × 3m
  - Entrance: 2m × 2m

## Recommended Use Cases

1. **Navigation Tasks**: Multi-room path planning, door opening, obstacle avoidance
2. **Manipulation Tasks**: Pick-and-place, drawer opening, object grasping
3. **Mobile Manipulation**: Combined navigation and manipulation
4. **Human-Robot Interaction**: Household task simulation
5. **Perception Training**: Object detection, semantic segmentation, scene understanding

## Dependencies

- MuJoCo 2.3.0 or higher
- Python 3.7+
- Required assets: `assets/meshes/teapot_simple.obj`

## Notes

- All interactive elements have properly configured actuators
- Physics parameters are tuned for stable simulation
- Scene is optimized for real-time performance
- Compatible with OH ROBOSIM simulator import

## Version History

- **v2.0** (2026-01-16): Added bathroom, windows, enhanced interactivity, 20+ moveable objects
- **v1.0** (2026-01-15): Initial release with 4 rooms

---

For technical support or questions, please refer to the project documentation or open an issue on GitHub.
