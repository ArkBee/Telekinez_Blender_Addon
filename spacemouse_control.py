bl_info = {
    "name": "Telekinez SpaceMouse",
    "author": "Telekinez",
    "version": (3, 0),
    "blender": (4, 5, 0),
    "location": "View3D > N-Panel > Telekinez",
    "description": "Control viewport / objects / pose bones from raw SpaceMouse input",
    "category": "3D View",
}

import bpy
import rna_keymap_ui
from bpy.props import EnumProperty, FloatProperty, StringProperty, BoolProperty
from mathutils import Vector, Matrix, Quaternion


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

class SpaceMouseSettings(bpy.types.PropertyGroup):
    mode: EnumProperty(
        name="Mode",
        items=[
            ('CAMERA', "Camera", "Fly the viewport with the SpaceMouse"),
            ('OBJECT', "Object", "Move/rotate selected objects (camera-relative)"),
            ('POSE',   "Pose",   "Move/rotate selected pose bones (camera-relative)"),
        ],
        default='CAMERA',
    )
    move_speed: FloatProperty(
        name="Move", default=0.05, min=0.01, max=2.0, soft_min=0.01, soft_max=2.0,
        description="Translation speed (units per NDOF tick at full deflection)",
    )
    rot_speed: FloatProperty(
        name="Rotate", default=0.03, min=0.01, max=2.0, soft_min=0.01, soft_max=2.0,
        description="Rotation speed (radians per NDOF tick at full deflection)",
    )
    invert_tx: BoolProperty(name="Invert X", default=False)
    invert_ty: BoolProperty(name="Invert Y", default=False)
    invert_tz: BoolProperty(name="Invert Z", default=False)
    invert_rx: BoolProperty(name="Invert Pitch", default=False)
    invert_ry: BoolProperty(name="Invert Yaw", default=False)
    invert_rz: BoolProperty(name="Invert Roll", default=False)
    lock_roll: BoolProperty(
        name="Lock Roll",
        default=False,
        description=("CAMERA only: when ON, roll input is ignored so the "
                     "camera can't accidentally tilt sideways. Yaw and "
                     "pitch stay fully free"),
    )
    camera_style: EnumProperty(
        name="Camera Style",
        items=[
            ('WALK',  "Walk",  "First-person flight: camera IS the eye, "
                               "translation moves where you're looking"),
            ('ORBIT', "Orbit", "Move/rotate the orbit pivot; camera stays "
                               "at view_distance from it (Blender default)"),
        ],
        default='WALK',
        description="How CAMERA mode interprets translation",
    )
    show_inverts: BoolProperty(
        name="Show Invert Axes", default=False,
        description="Expand/collapse the per-axis invert toggles",
    )
    chord_action: EnumProperty(
        name="Chord Action",
        items=[
            ('POSE',       "Switch to POSE",  "Set mode to POSE"),
            ('LEVEL',      "Level View",      "Zero out roll, keep look direction"),
            ('LOCK_ROLL',  "Toggle Lock Roll","Flip the Lock Roll toggle"),
            ('RESET',      "Reset Target",    "Zero location/rotation of selection"),
            ('NONE',       "Disabled",        "Ignore the chord"),
        ],
        default='POSE',
        description="Action triggered by pressing both mode hotkeys within 0.25s",
    )
    debug_info: StringProperty(default="")


# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------

class _ObjectTarget:
    __slots__ = ("obj",)
    def __init__(self, obj): self.obj = obj
    def world_pivot(self): return self.obj.matrix_world.translation.copy()
    def world_matrix(self): return self.obj.matrix_world.copy()
    def set_world_matrix(self, m): self.obj.matrix_world = m


class _PoseBoneTarget:
    __slots__ = ("arm", "pbone")
    def __init__(self, arm, pbone): self.arm, self.pbone = arm, pbone
    def world_pivot(self):
        return (self.arm.matrix_world @ self.pbone.matrix).translation.copy()
    def world_matrix(self):
        return self.arm.matrix_world @ self.pbone.matrix
    def set_world_matrix(self, m):
        self.pbone.matrix = self.arm.matrix_world.inverted() @ m


def _collect_targets(context):
    sm = context.scene.spacemouse_settings
    if sm.mode == 'POSE':
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE' or arm.mode != 'POSE':
            return []
        bones = list(context.selected_pose_bones or [])
        if not bones and arm.data.bones.active:
            pb = arm.pose.bones.get(arm.data.bones.active.name)
            if pb:
                bones = [pb]
        return [_PoseBoneTarget(arm, b) for b in bones]
    if sm.mode == 'OBJECT':
        sel = list(context.selected_objects)
        if not sel and context.active_object:
            sel = [context.active_object]
        return [_ObjectTarget(o) for o in sel]
    return []


# ---------------------------------------------------------------------------
# Viewport helpers
# ---------------------------------------------------------------------------

def _find_rv3d_from_context(context):
    sd = getattr(context, "space_data", None)
    if sd and sd.type == 'VIEW_3D':
        return sd.region_3d
    for win in context.window_manager.windows:
        for area in win.screen.areas:
            if area.type == 'VIEW_3D':
                return area.spaces.active.region_3d
    return None


def _redraw_view3d(context):
    for win in context.window_manager.windows:
        for area in win.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()


# ---------------------------------------------------------------------------
# Raw NDOF reader
# ---------------------------------------------------------------------------

# Blender 5.0 flipped the sign of the raw NDOF translation Z axis relative
# to 4.x — pushing the cap forward used to report -Z, now reports +Z. (Roll
# rotation sign is unchanged, so only the tz baseline needs branching.)
_NDOF_TZ_FLIPPED = bpy.app.version >= (5, 0, 0)


def _read_ndof(event, sm):
    """Pull raw translation+rotation from an NDOF_MOTION event and apply
    per-axis inversion + speed scaling. Returns (t_local, r_local) in the
    SpaceMouse's own frame: X=right, Y=up, Z=toward-user. Rotation is the
    same axes as Euler pitch/yaw/roll."""
    nm = event.ndof_motion
    if nm is None:
        return None, None

    # Blender's raw NDOF translation needs three axis flips to match the
    # "object follows joystick, viewport-relative" convention used downstream
    # (+X=screen-right, +Y=screen-up, +Z=into-screen). These flips are baked
    # in here, NOT exposed as inversions — the Invert toggles below sit on
    # top of the already-correct baseline.
    t = nm.translation
    tx = t.x if sm.invert_tx else -t.x
    ty = t.y if sm.invert_ty else -t.y
    if _NDOF_TZ_FLIPPED:
        tz = -t.z if sm.invert_tz else t.z
    else:
        tz = t.z if sm.invert_tz else -t.z
    t_local = Vector((tx, ty, tz)) * sm.move_speed

    # Same idea for rotation: roll needs an extra flip to feel natural, baked
    # into the baseline rather than surfaced as a default-on inversion. (Same
    # on 4.x and 5.x — only translation Z sign changed between versions.)
    r = nm.rotation
    rx = -r.x if sm.invert_rx else r.x
    ry = -r.y if sm.invert_ry else r.y
    rz = r.z if sm.invert_rz else -r.z
    r_local = Vector((rx, ry, rz)) * sm.rot_speed

    return t_local, r_local


# ---------------------------------------------------------------------------
# Mode handlers
# ---------------------------------------------------------------------------

def _apply_camera(context, rv3d, t_local, r_local):
    """Drive the viewport. Two styles:
    - WALK: collapse view_distance into view_location so the camera sits at
      the eye position, then translation moves the eye directly. Matches
      Blender's Ctrl+Shift+` Walk Navigation feel.
    - ORBIT: leave view_distance alone; translation moves the orbit pivot
      and the camera follows at distance. Standard Blender NDOF orbit feel.
    Rotation works the same in both styles — around local camera axes."""
    sm = context.scene.spacemouse_settings
    view_rot = rv3d.view_rotation

    cam_right   = view_rot @ Vector((1, 0, 0))
    cam_up      = view_rot @ Vector((0, 1, 0))
    cam_forward = view_rot @ Vector((0, 0, -1))

    pitch = Quaternion(cam_right,   r_local.x)
    yaw   = Quaternion(cam_up,      r_local.y)
    roll  = Quaternion(cam_forward, 0.0 if sm.lock_roll else r_local.z)
    delta = yaw @ pitch @ roll
    new_view_rot = (delta @ view_rot).normalized()
    world_move = cam_right * t_local.x + cam_up * t_local.y + cam_forward * t_local.z

    if sm.camera_style == 'WALK' and rv3d.is_perspective:
        # Walk feel without touching view_distance (so mouse wheel zoom still
        # works). Pivot sits view_distance ahead of the eye; we anchor the eye,
        # apply translation in eye space, then place the new pivot ahead of
        # the post-rotation eye by the same view_distance. Net effect: rotation
        # spins the camera in place (not orbits around an anchor), translation
        # walks the eye, and view_distance is left alone for the user / wheel.
        new_cam_forward = new_view_rot @ Vector((0, 0, -1))
        eye = rv3d.view_location - cam_forward * rv3d.view_distance
        eye_after = eye + world_move
        rv3d.view_location = eye_after + new_cam_forward * rv3d.view_distance
    else:
        # ORBIT (and ortho fallback handled upstream): translation slides the
        # pivot, rotation orbits the camera around it. Blender NDOF default.
        rv3d.view_location = rv3d.view_location + world_move

    rv3d.view_rotation = new_view_rot
    _redraw_view3d(context)


def _apply_to_targets(context, rv3d, t_local, r_local):
    """OBJECT / POSE: convert camera-frame deltas into world deltas and apply
    them around the group's median pivot. Viewport stays still."""
    targets = _collect_targets(context)
    if not targets:
        context.scene.spacemouse_settings.debug_info = (
            f"[{context.scene.spacemouse_settings.mode}] no selection")
        return

    view_rot = rv3d.view_rotation
    cam_right   = view_rot @ Vector((1, 0, 0))
    cam_up      = view_rot @ Vector((0, 1, 0))
    cam_forward = view_rot @ Vector((0, 0, -1))

    # Translation in world space, derived from camera axes.
    world_move = cam_right * t_local.x + cam_up * t_local.y + cam_forward * t_local.z

    # Rotation: build axis-angle in world space using the camera basis.
    # pitch around cam_right, yaw around cam_up, roll around cam_forward.
    pitch = Quaternion(cam_right,   r_local.x)
    yaw   = Quaternion(cam_up,      r_local.y)
    roll  = Quaternion(cam_forward, r_local.z)
    world_rot = (yaw @ pitch @ roll).normalized()

    # Pivot at group median, rotate around it, then translate.
    pivot = Vector((0, 0, 0))
    for t in targets:
        pivot += t.world_pivot()
    pivot /= len(targets)
    rot4 = world_rot.to_matrix().to_4x4()
    base = Matrix.Translation(pivot) @ rot4 @ Matrix.Translation(-pivot)
    for tgt in targets:
        m = base @ tgt.world_matrix()
        m.translation += world_move
        tgt.set_world_matrix(m)

    sm = context.scene.spacemouse_settings
    sm.debug_info = (
        f"[{sm.mode}] x{len(targets)} "
        f"T({t_local.x:+.2f},{t_local.y:+.2f},{t_local.z:+.2f}) "
        f"R({r_local.x:+.2f},{r_local.y:+.2f},{r_local.z:+.2f})"
    )


# ---------------------------------------------------------------------------
# Modal listener — single source of truth, consumes raw NDOF
# ---------------------------------------------------------------------------

class NDOF_OT_listener(bpy.types.Operator):
    """Consume raw NDOF events. Drive viewport (CAMERA) or targets (OBJECT/POSE)."""
    bl_idname = "view3d.ndof_listener"
    bl_label = "Telekinez SpaceMouse Listener"

    def modal(self, context, event):
        wm = context.window_manager
        if not wm.get("ndof_modal_running", False):
            return self._cleanup(context, {'FINISHED'})

        if event.type == 'NDOF_MOTION':
            sm = context.scene.spacemouse_settings
            rv3d = _find_rv3d_from_context(context)
            if rv3d is None:
                return {'PASS_THROUGH'}

            t_local, r_local = _read_ndof(event, sm)
            if t_local is None or r_local is None:
                return {'PASS_THROUGH'}

            # Idle-zone: don't apply tiny noise.
            if t_local.length < 1e-4 and r_local.length < 1e-4:
                return {'RUNNING_MODAL'}  # consume, but no-op

            if sm.mode == 'CAMERA':
                # Ortho views have no meaningful eye position and our WALK/ORBIT
                # math distorts them (view_distance is a zoom scale there, not
                # a distance). Hand the event off to Blender's native NDOF nav
                # which knows how to pan/zoom an ortho viewport correctly.
                if not rv3d.is_perspective:
                    sm.debug_info = "[CAMERA-ORTHO] passthrough → native NDOF"
                    return {'PASS_THROUGH'}
                _apply_camera(context, rv3d, t_local, r_local)
                sm.debug_info = (
                    f"[CAMERA] "
                    f"T({t_local.x:+.2f},{t_local.y:+.2f},{t_local.z:+.2f}) "
                    f"R({r_local.x:+.2f},{r_local.y:+.2f},{r_local.z:+.2f})"
                )
            else:
                _apply_to_targets(context, rv3d, t_local, r_local)

            # We consume NDOF in all modes so Blender's built-in NDOF nav
            # doesn't fight us. This is the whole point of the rewrite.
            return {'RUNNING_MODAL'}

        # Pass everything else (mouse, keyboard) through unmodified.
        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        wm["ndof_modal_running"] = True
        wm.modal_handler_add(self)
        context.scene.spacemouse_settings.debug_info = "Running"
        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        return self.execute(context)

    def _cleanup(self, context, ret):
        context.window_manager["ndof_modal_running"] = False
        context.scene.spacemouse_settings.debug_info = "Stopped"
        return ret


class NDOF_OT_stop(bpy.types.Operator):
    bl_idname = "view3d.ndof_stop"
    bl_label = "Stop Listener"
    def execute(self, context):
        context.window_manager["ndof_modal_running"] = False
        return {'FINISHED'}


class NDOF_OT_reset_target(bpy.types.Operator):
    """Zero out location and rotation of selected object(s) or pose bone(s)."""
    bl_idname = "view3d.ndof_reset_target"
    bl_label = "Reset Target to Origin"

    def execute(self, context):
        targets = _collect_targets(context)
        n = 0
        for t in targets:
            if isinstance(t, _ObjectTarget):
                t.obj.location = (0, 0, 0)
                t.obj.rotation_euler = (0, 0, 0)
                if t.obj.rotation_mode == 'QUATERNION':
                    t.obj.rotation_quaternion = (1, 0, 0, 0)
            else:
                t.pbone.location = (0, 0, 0)
                t.pbone.rotation_euler = (0, 0, 0)
                t.pbone.rotation_quaternion = (1, 0, 0, 0)
            n += 1
        self.report({'INFO'}, f"Reset {n} target(s)")
        return {'FINISHED'}


class NDOF_OT_level_view(bpy.types.Operator):
    """Zero out roll: keep look direction, snap horizon to world Z."""
    bl_idname = "view3d.ndof_level_view"
    bl_label = "Level View (Zero Roll)"

    def execute(self, context):
        n = 0
        for win in context.window_manager.windows:
            for area in win.screen.areas:
                if area.type != 'VIEW_3D':
                    continue
                rv3d = area.spaces.active.region_3d
                forward = (rv3d.view_rotation @ Vector((0, 0, -1))).normalized()
                # Looking straight up/down — yaw is undefined, leave it alone.
                if abs(forward.z) > 0.9999:
                    continue
                rv3d.view_rotation = forward.to_track_quat('-Z', 'Y')
                area.tag_redraw()
                n += 1
        self.report({'INFO'}, f"Leveled {n} viewport(s)")
        return {'FINISHED'}


# Chord state: when two different mode hotkeys fire within CHORD_WINDOW
# seconds, the second press promotes the result to POSE. The first press
# tentatively switches to its own target; if no second press arrives in time,
# that switch stands. If a second press IS the chord, we override to POSE.
import time as _time
_chord_state = {'last_time': 0.0, 'last_target': None, 'pre_chord_mode': None}
CHORD_WINDOW = 0.25


class NDOF_OT_set_mode(bpy.types.Operator):
    """Switch SpaceMouse mode. Two different hotkeys within 0.25s → POSE."""
    bl_idname = "view3d.ndof_set_mode"
    bl_label = "Set SpaceMouse Mode"
    target: EnumProperty(
        items=[('CAMERA', "Camera", ""),
               ('OBJECT', "Object", ""),
               ('POSE',   "Pose",   "")],
    )

    def execute(self, context):
        sm = context.scene.spacemouse_settings
        now = _time.time()

        # Chord check: a different target within the window → run chord_action.
        if (_chord_state['last_target'] is not None
                and _chord_state['last_target'] != self.target
                and now - _chord_state['last_time'] < CHORD_WINDOW):
            action = sm.chord_action
            # Roll back the tentative first-press mode switch so the chord
            # doesn't leave the user with an unwanted intermediate state.
            if _chord_state['pre_chord_mode'] is not None:
                sm.mode = _chord_state['pre_chord_mode']
            _chord_state['last_target'] = None
            _chord_state['pre_chord_mode'] = None

            if action == 'POSE':
                sm.mode = 'POSE'
                msg = "POSE"
            elif action == 'LEVEL':
                bpy.ops.view3d.ndof_level_view()
                msg = "Level View"
            elif action == 'LOCK_ROLL':
                sm.lock_roll = not sm.lock_roll
                msg = f"Lock Roll {'ON' if sm.lock_roll else 'OFF'}"
            elif action == 'RESET':
                bpy.ops.view3d.ndof_reset_target()
                msg = "Reset Target"
            else:  # 'NONE'
                msg = "ignored"

            self.report({'INFO'}, f"Chord → {msg}")
            print(f"[Telekinez] Chord fired → {msg}", flush=True)
            return {'FINISHED'}

        # First press of a fresh chord — remember pre-chord mode and switch.
        if (_chord_state['last_target'] is None
                or now - _chord_state['last_time'] >= CHORD_WINDOW):
            _chord_state['pre_chord_mode'] = sm.mode
        _chord_state['last_target'] = self.target
        _chord_state['last_time'] = now
        sm.mode = self.target
        self.report({'INFO'}, f"Mode: {self.target}")
        print(f"[Telekinez] Hotkey fired → mode={self.target}", flush=True)
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class VIEW3D_PT_spacemouse(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Telekinez"
    bl_label = "Telekinez SpaceMouse"

    def draw(self, context):
        layout = self.layout
        sm = context.scene.spacemouse_settings

        layout.prop(sm, "mode", expand=True)

        col = layout.column(align=True)
        col.prop(sm, "move_speed", slider=True)
        col.prop(sm, "rot_speed",  slider=True)

        box = layout.box()
        box.prop(sm, "show_inverts",
                 icon='TRIA_DOWN' if sm.show_inverts else 'TRIA_RIGHT',
                 text="Invert Axes", emboss=False)
        if sm.show_inverts:
            row = box.row(align=True)
            row.prop(sm, "invert_tx", toggle=True, text="X")
            row.prop(sm, "invert_ty", toggle=True, text="Y")
            row.prop(sm, "invert_tz", toggle=True, text="Z")
            row = box.row(align=True)
            row.prop(sm, "invert_rx", toggle=True, text="Pitch")
            row.prop(sm, "invert_ry", toggle=True, text="Yaw")
            row.prop(sm, "invert_rz", toggle=True, text="Roll")

        row = layout.row(align=True)
        row.prop(sm, "lock_roll", toggle=True,
                 icon='LOCKVIEW_ON' if sm.lock_roll else 'LOCKVIEW_OFF')
        row.operator("view3d.ndof_level_view", text="-|-")

        if sm.mode == 'CAMERA':
            row = layout.row(align=True)
            row.label(text="Style:")
            row.prop(sm, "camera_style", expand=True)

        running = context.window_manager.get("ndof_modal_running", False)
        row = layout.row(align=True)
        if running:
            row.operator("view3d.ndof_stop", text="Stop", icon='PAUSE')
        else:
            row.operator("view3d.ndof_listener", text="Start", icon='PLAY')
        layout.operator("view3d.ndof_reset_target",
                        text="Reset Target (0,0,0)", icon='LOOP_BACK')
        layout.label(text=sm.debug_info or ("Running" if running else "Idle"),
                     icon='INFO')

        layout.separator()
        layout.prop(sm, "chord_action", text="Chord")
        layout.label(text="Mode Hotkeys:")
        wm = context.window_manager
        # Hotkeys live in the Window keymap (see register()). Show whichever
        # keyconfig actually has them — user overrides take priority.
        found = False
        for kc in (wm.keyconfigs.user, wm.keyconfigs.addon):
            if not kc or found:
                continue
            for km_name in ('Window', '3D View'):
                km = kc.keymaps.get(km_name)
                if not km:
                    continue
                kmis = [i for i in km.keymap_items
                        if i.idname == "view3d.ndof_set_mode"]
                if kmis:
                    col = layout.column()
                    col.context_pointer_set("keymap", km)
                    for kmi in kmis:
                        box = col.box()
                        box.label(text=f"Hotkey -> {kmi.properties.target}")
                        rna_keymap_ui.draw_kmi([], kc, km, kmi, box, 0)
                    found = True
                    break


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

classes = (
    SpaceMouseSettings,
    NDOF_OT_listener,
    NDOF_OT_stop,
    NDOF_OT_reset_target,
    NDOF_OT_level_view,
    NDOF_OT_set_mode,
    VIEW3D_PT_spacemouse,
)

addon_keymaps = []


def _autostart_listener():
    wm = bpy.context.window_manager
    if wm.get("ndof_modal_running", False):
        return None
    for win in wm.windows:
        for area in win.screen.areas:
            if area.type == 'VIEW_3D':
                for region in area.regions:
                    if region.type == 'WINDOW':
                        with bpy.context.temp_override(window=win, area=area, region=region):
                            bpy.ops.view3d.ndof_listener('INVOKE_DEFAULT')
                        return None
    return None


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.spacemouse_settings = bpy.props.PointerProperty(type=SpaceMouseSettings)

    try:
        bpy.context.window_manager["ndof_modal_running"] = False
    except Exception:
        pass

    bpy.app.timers.register(_autostart_listener, first_interval=0.3)

    # Sweep stale keymap entries from prior versions of the addon.
    wm = bpy.context.window_manager
    STALE_IDNAMES = {
        "view3d.ndof_set_mode",
        "view3d.ndof_toggle_mode_1",
        "view3d.ndof_toggle_mode_2",
        "view3d.ndof_toggle_lock_horizon",
        "view3d.ndof_cycle_mode",
    }
    for kc in (wm.keyconfigs.addon, wm.keyconfigs.user):
        if not kc:
            continue
        for km_name in ('3D View', 'Window'):
            km = kc.keymaps.get(km_name)
            if not km:
                continue
            for stale in [i for i in km.keymap_items if i.idname in STALE_IDNAMES]:
                try: km.keymap_items.remove(stale)
                except Exception: pass

    # Register hotkeys in BOTH addon and user keyconfigs to guarantee they
    # fire regardless of resolution order. Window keymap so they work from
    # anywhere in Blender, not just over the 3D viewport.
    # Physical keys (US-layout codes; map to the same physical keys on any
    # OS layout): Shift+; (Russian "Shift+э") → Camera, Shift+' (Russian
    # "Shift+ж") → Object. POSE has no dedicated key — pressing both within
    # 0.25s acts as a chord (handled in NDOF_OT_set_mode.execute).
    BINDINGS = (('SEMI_COLON', 'CAMERA'),
                ('QUOTE',      'OBJECT'))
    registered = 0
    for kc in (wm.keyconfigs.addon, wm.keyconfigs.user):
        if not kc:
            continue
        km = kc.keymaps.get('Window') or kc.keymaps.new(
            name='Window', space_type='EMPTY', region_type='WINDOW')
        for key, target in BINDINGS:
            kmi = km.keymap_items.new(NDOF_OT_set_mode.bl_idname,
                                       type=key, value='PRESS', shift=True)
            kmi.properties.target = target
            addon_keymaps.append((km, kmi))
            registered += 1
    print(f"[Telekinez] Registered {registered} hotkey bindings: "
          f"Shift+э → Camera, Shift+ж → Object, both within 0.25s → Pose. "
          f"Watch System Console for '[Telekinez] Hotkey fired' on press.",
          flush=True)


def unregister():
    try:
        bpy.context.window_manager["ndof_modal_running"] = False
    except Exception:
        pass
    for km, kmi in addon_keymaps:
        try: km.keymap_items.remove(kmi)
        except Exception: pass
    addon_keymaps.clear()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.spacemouse_settings


if __name__ == "__main__":
    register()
