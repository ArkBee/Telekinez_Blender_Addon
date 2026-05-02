# Telekinez SpaceMouse Object Control for Blender

A powerful Blender add-on that allows you to control 3D objects directly using a 3Dconnexion SpaceMouse (NDOF device). By default, Blender only allows the SpaceMouse to navigate the viewport camera. This add-on unlocks the true potential of your SpaceMouse by granting precise 6-DOF (Degrees of Freedom) translation and rotation over active objects.

## Features
- **Object Manipulation:** Seamlessly switch between navigating the Camera and moving/rotating the active Object.
- **View Relative Movement (View Space):** Push objects relative to your current camera viewing angle, or toggle it off to move them strictly along Global World Axes.
- **Input Isolation:** Automatically isolates SpaceMouse inputs from standard mouse orbiting. You can Orbit with your regular mouse (Alt+LMB/MMB) without the object jumping.
- **On-the-fly Hotkeys:** Rebind the standard toggle shortcut directly from the N-Panel UI. Map it to one of your physical SpaceMouse buttons for lightning-fast swapping!
- **Custom Sensitivity:** Real-time adjustable slider for scaling movement and rotation sensitivity.
- **Trackball Auto-Toggle:** Intelligently switches Blender's navigation to Trackball mode when moving objects to ensure the `Roll` axis is properly captured, and restores your preferred settings when returning to Camera mode.
- **Zero-Reset Transform:** Instantly snap your object's location and rotation back to World Origin.

## Installation
1. Go to the [Releases](https://github.com/ArkBee/Telekinez_Blender_Addon/releases) page or download the repository as a ZIP.
2. Or simply download the `spacemouse_control.py` file.
3. Open Blender.
4. Go to `Edit` -> `Preferences` -> `Add-ons`.
5. Click `Install...` and select the downloaded `spacemouse_control.py` (or the `.zip` file).
6. Check the box to enable **3D View: SpaceMouse Object Control**.

## Usage
1. Open the **N-Panel** in the 3D Viewport (press `N`).
2. Go to the new **SpaceMouse** tab.
3. Click **Start NDOF Listener** to activate the modal operator.
4. Toggle the **Control Mode** between `Camera` and `Object`. 
5. Start moving your SpaceMouse!

## Compatibility
Developed and tested for **Blender 4.0+** and **Blender 5.1.1**. Requires a 3Dconnexion SpaceMouse or compatible NDOF device. 
No third-party Python modules required (uses pure `bpy` and `mathutils`).
