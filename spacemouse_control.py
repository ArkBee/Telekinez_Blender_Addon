bl_info = {
    "name": "Telekinez SpaceMouse",
    "author": "Telekinez",
    "version": (1, 1),
    "blender": (4, 0, 0),
    "location": "View3D > N-Panel > Telekinez",
    "description": "Control objects / pose bones instead of camera using SpaceMouse",
    "category": "3D View",
}

import time
import mathutils
import rna_keymap_ui

import bpy
from bpy.props import EnumProperty, StringProperty, FloatProperty, BoolProperty
from mathutils import Vector, Matrix, Quaternion, Euler


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

def update_lock_horizon(self, context):
    prefs = context.preferences.inputs
    if self.lock_horizon:
        self.saved_rotate_method = prefs.view_rotate_method
        prefs.view_rotate_method = 'TURNTABLE'
    else:
        if self.saved_rotate_method != "":
            prefs.view_rotate_method = self.saved_rotate_method
        else:
            prefs.view_rotate_method = 'TRACKBALL'


def update_mode(self, context):
    prefs = context.preferences.inputs
    has_orbit = hasattr(prefs, "ndof_orbit")
    has_lock = hasattr(prefs, "use_ndof_lock_horizon")

    if self.mode == 'CAMERA':
        if has_orbit and self.saved_orbit != "":
            prefs.ndof_orbit = self.saved_orbit
        if has_lock:
            prefs.use_ndof_lock_horizon = self.saved_lock_horizon
    else:
        # OBJECT or POSE — save user prefs and switch NDOF to free trackball
        if has_orbit:
            self.saved_orbit = prefs.ndof_orbit
            prefs.ndof_orbit = 'TRACKBALL'
        if has_lock:
            self.saved_lock_horizon = prefs.use_ndof_lock_horizon
            prefs.use_ndof_lock_horizon = False

    # Center NDOF orbit on the active target's world pivot
    pivot = _active_pivot(context)
    if pivot is not None and context.space_data and context.space_data.type == 'VIEW_3D':
        context.space_data.region_3d.view_location = pivot


class SpaceMouseSettings(bpy.types.PropertyGroup):
    mode: EnumProperty(
        name="Control Mode",
        description="Select what SpaceMouse controls",
        items=[
            ('CAMERA', "Camera", "Standard viewport camera control"),
            ('OBJECT', "Object", "Control selected objects"),
            ('POSE',   "Pose",   "Control selected pose bones"),
        ],
        default='CAMERA',
        update=update_mode,
    )
    pivot_mode: EnumProperty(
        name="Pivot",
        description="How to pivot multiple targets",
        items=[
            ('MEDIAN',     "Median",     "Treat selection as one group around its median point"),
            ('INDIVIDUAL', "Individual", "Each target moves/rotates around its own origin"),
        ],
        default='MEDIAN',
    )
    saved_orbit: StringProperty(default="")
    saved_lock_horizon: BoolProperty(default=True)
    saved_rotate_method: StringProperty(default="")

    use_view_space: BoolProperty(
        name="View Relative",
        description="Move and rotate relative to camera view",
        default=True,
    )
    sensitivity: FloatProperty(
        name="Sensitivity", default=0.1, min=0.001, max=5.0, step=1,
    )
    debug_info: StringProperty(default="Ready...")

    show_inversion_settings: BoolProperty(name="Invert Axes Settings", default=False)
    inv_tx: BoolProperty(name="Invert Move X (Left/Right)", default=False)
    inv_ty: BoolProperty(name="Invert Move Y (Forward/Back)", default=False)
    inv_tz: BoolProperty(name="Invert Move Z (Up/Down)", default=False)
    inv_rx: BoolProperty(name="Invert Pitch (Tilt F/B)", default=False)
    inv_ry: BoolProperty(name="Invert Roll (Tilt L/R)", default=False)
    inv_rz: BoolProperty(name="Invert Yaw (Twist)", default=False)

    lock_horizon: BoolProperty(
        name="Lock Horizon (Turntable)",
        description="Switch viewport orbit to Turntable. Off = Trackball",
        default=False,
        update=update_lock_horizon,
    )


# ---------------------------------------------------------------------------
# Targets — abstraction over objects and pose bones
# ---------------------------------------------------------------------------

class _ObjectTarget:
    __slots__ = ("obj",)
    def __init__(self, obj): self.obj = obj
    def world_pivot(self): return self.obj.matrix_world.translation.copy()
    def world_matrix(self): return self.obj.matrix_world.copy()
    def set_world_matrix(self, m):
        # Preserve scale by composing with parent inverse if any
        self.obj.matrix_world = m


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
        bones = context.selected_pose_bones or ([arm.pose.bones[arm.data.bones.active.name]]
                                                if arm.data.bones.active else [])
        return [_PoseBoneTarget(arm, b) for b in bones if b]
    if sm.mode == 'OBJECT':
        sel = list(context.selected_objects)
        if not sel and context.active_object:
            sel = [context.active_object]
        return [_ObjectTarget(o) for o in sel]
    return []


def _active_pivot(context):
    targets = _collect_targets(context)
    if not targets:
        return None
    sm = context.scene.spacemouse_settings
    if sm.pivot_mode == 'MEDIAN' and len(targets) > 1:
        acc = Vector((0, 0, 0))
        for t in targets:
            acc += t.world_pivot()
        return acc / len(targets)
    return targets[0].world_pivot()


# ---------------------------------------------------------------------------
# Apply a world-space delta (translation + rotation) to targets
# ---------------------------------------------------------------------------

def _apply_delta(targets, pivot_mode, delta_loc_world, delta_rot_world):
    """delta_rot_world: Quaternion. delta_loc_world: Vector. Both in world space."""
    if not targets:
        return

    if pivot_mode == 'INDIVIDUAL' or len(targets) == 1:
        for t in targets:
            m = t.world_matrix()
            origin = m.translation.copy()
            # Rotate around own origin, then translate
            rot = delta_rot_world.to_matrix().to_4x4()
            new_m = (Matrix.Translation(origin)
                     @ rot
                     @ Matrix.Translation(-origin)
                     @ m)
            new_m.translation += delta_loc_world
            t.set_world_matrix(new_m)
        return

    # MEDIAN
    pivot = Vector((0, 0, 0))
    for t in targets:
        pivot += t.world_pivot()
    pivot /= len(targets)

    rot4 = delta_rot_world.to_matrix().to_4x4()
    base = Matrix.Translation(pivot) @ rot4 @ Matrix.Translation(-pivot)
    for t in targets:
        m = t.world_matrix()
        new_m = base @ m
        new_m.translation += delta_loc_world
        t.set_world_matrix(new_m)


def _redraw_view3d(context):
    for win in context.window_manager.windows:
        for area in win.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()


def _find_rv3d(context):
    """Return any VIEW_3D region_3d, scanning all open windows."""
    sd = getattr(context, "space_data", None)
    if sd and sd.type == 'VIEW_3D':
        return sd.region_3d
    for win in context.window_manager.windows:
        for area in win.screen.areas:
            if area.type == 'VIEW_3D':
                for sp in area.spaces:
                    if sp.type == 'VIEW_3D':
                        return sp.region_3d
    return None


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

class NDOF_OT_reset_object(bpy.types.Operator):
    """Reset transform of selected target(s)"""
    bl_idname = "view3d.ndof_reset_object"
    bl_label = "Reset Transform"

    def execute(self, context):
        targets = _collect_targets(context)
        n = 0
        for t in targets:
            if isinstance(t, _ObjectTarget):
                t.obj.location = (0, 0, 0)
                t.obj.rotation_euler = (0, 0, 0)
            else:
                t.pbone.location = (0, 0, 0)
                if t.pbone.rotation_mode == 'QUATERNION':
                    t.pbone.rotation_quaternion = (1, 0, 0, 0)
                else:
                    t.pbone.rotation_euler = (0, 0, 0)
            n += 1
        if n and context.space_data and context.space_data.type == 'VIEW_3D':
            p = _active_pivot(context)
            if p is not None:
                context.space_data.region_3d.view_location = p
        self.report({'INFO'}, f"Reset {n} target(s)")
        return {'FINISHED'}


class NDOF_OT_toggle_mode(bpy.types.Operator):
    """Cycle SpaceMouse control mode (Camera / Object / Pose)"""
    bl_idname = "view3d.ndof_toggle_mode"
    bl_label = "Toggle SpaceMouse Mode"

    def execute(self, context):
        sm = context.scene.spacemouse_settings
        order = ['CAMERA', 'OBJECT', 'POSE']
        sm.mode = order[(order.index(sm.mode) + 1) % len(order)]
        self.report({'INFO'}, f"SpaceMouse Mode: {sm.mode}")
        return {'FINISHED'}


class NDOF_OT_toggle_lock_horizon(bpy.types.Operator):
    bl_idname = "view3d.ndof_toggle_lock_horizon"
    bl_label = "Toggle Lock Horizon"

    def execute(self, context):
        sm = context.scene.spacemouse_settings
        sm.lock_horizon = not sm.lock_horizon
        self.report({'INFO'}, f"Lock Horizon: {'ON' if sm.lock_horizon else 'OFF'}")
        return {'FINISHED'}


class NDOF_OT_object_control(bpy.types.Operator):
    """SpaceMouse modal listener"""
    bl_idname = "view3d.ndof_object_control"
    bl_label = "NDOF Object Control Modal"

    def modal(self, context, event):
        sm = context.scene.spacemouse_settings

        if not context.window_manager.get("ndof_modal_running", False):
            self.cancel(context)
            sm.debug_info = "Stopped."
            return {'FINISHED'}

        rv3d = _find_rv3d(context)

        if event.type in {'LEFTMOUSE', 'MIDDLEMOUSE', 'RIGHTMOUSE'}:
            if event.value == 'PRESS':
                self.mouse_held = True
            elif event.value == 'RELEASE':
                self.mouse_held = False

        if event.type == 'TIMER':
            self.tick_count += 1
            if rv3d is None:
                sm.debug_info = f"[{sm.mode}] (no VIEW_3D) ticks={self.tick_count}"
                _redraw_view3d(context)
            else:
                self._tick(context, rv3d, sm)

        if event.type == 'NDOF_MOTION':
            self.last_ndof_time = time.time()
            return {'PASS_THROUGH'}

        return {'PASS_THROUGH'}

    def _tick(self, context, rv3d, sm):
        current_loc = rv3d.view_location.copy()
        current_rot = rv3d.view_rotation.copy()
        current_dist = rv3d.view_distance

        if self.saved_loc is None:
            self.saved_loc = current_loc
            self.saved_rot = current_rot
            self.saved_dist = current_dist
            return

        # Active when in OBJECT/POSE mode and the user is NOT manually orbiting with the mouse.
        # We don't require NDOF_MOTION events: in some Blender builds they don't reach modals.
        active = sm.mode in {'OBJECT', 'POSE'} and not self.mouse_held

        if not active:
            self.saved_loc = current_loc
            self.saved_rot = current_rot
            self.saved_dist = current_dist
            if context.area:
                context.area.tag_redraw()
            return

        moved = ((current_loc - self.saved_loc).length > 0.0001
                 or current_rot.rotation_difference(self.saved_rot).angle > 0.0001
                 or abs(current_dist - self.saved_dist) > 0.0001)
        if not moved:
            return

        targets = _collect_targets(context)
        if not targets:
            sm.debug_info = f"[{sm.mode}] no targets selected"
            self.saved_loc = current_loc
            self.saved_rot = current_rot
            self.saved_dist = current_dist
            _redraw_view3d(context)
            return

        sens = sm.sensitivity

        # Extract raw (hardware) joystick axes from viewport delta in view-space
        q_view = self.saved_rot.copy()
        local_diff = q_view.inverted() @ (current_loc - self.saved_loc)
        delta_dist = current_dist - self.saved_dist

        sm_tx = local_diff.x * (-1 if sm.inv_tx else 1)
        sm_ty = (-delta_dist - local_diff.z) * (-1 if sm.inv_ty else 1)
        sm_tz = local_diff.y * (-1 if sm.inv_tz else 1)

        q_diff_local = q_view.inverted() @ current_rot
        e = q_diff_local.to_euler('XYZ')
        sm_rx =  e.x * (-1 if sm.inv_rx else 1)
        sm_ry = -e.z * (-1 if sm.inv_ry else 1)
        sm_rz = -e.y * (-1 if sm.inv_rz else 1)

        if sm.use_view_space:
            view_move = Vector((sm_tx, sm_tz, -sm_ty)) * sens
            delta_loc_world = q_view @ view_move
            view_rot_euler = Euler((sm_rx * sens, sm_rz * sens, -sm_ry * sens), 'XYZ')
            view_rot = view_rot_euler.to_quaternion()
            delta_rot_world = q_view @ view_rot @ q_view.inverted()
        else:
            delta_loc_world = Vector((sm_tx, sm_ty, sm_tz)) * sens
            delta_rot_world = Euler((sm_rx * sens, sm_ry * sens, sm_rz * sens),
                                    'XYZ').to_quaternion()

        _apply_delta(targets, sm.pivot_mode, delta_loc_world, delta_rot_world)

        sm.debug_info = (
            f"[{sm.mode}/{sm.pivot_mode}] x{len(targets)}  "
            f"T({sm_tx:+.2f},{sm_ty:+.2f},{sm_tz:+.2f}) "
            f"R({sm_rx:+.2f},{sm_ry:+.2f},{sm_rz:+.2f})"
        )

        # Snap viewport back so it feels locked
        rv3d.view_location = self.saved_loc
        rv3d.view_rotation = self.saved_rot
        rv3d.view_distance = self.saved_dist

        _redraw_view3d(context)

    def execute(self, context):
        wm = context.window_manager
        wm["ndof_modal_running"] = True
        self._timer = None
        self.saved_loc = None
        self.saved_rot = None
        self.saved_dist = None
        self.last_ndof_time = 0.0
        self.mouse_held = False
        self.tick_count = 0
        self._timer = wm.event_timer_add(0.01, window=context.window)
        wm.modal_handler_add(self)
        context.scene.spacemouse_settings.debug_info = "Listener started..."
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm["ndof_modal_running"] = False
        if self._timer is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None


class NDOF_OT_cancel_control(bpy.types.Operator):
    bl_idname = "view3d.ndof_cancel_control"
    bl_label = "Stop NDOF Modal"

    def execute(self, context):
        context.window_manager["ndof_modal_running"] = False
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class VIEW3D_PT_spacemouse_control(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Telekinez"
    bl_label = "Telekinez SpaceMouse"

    def draw(self, context):
        layout = self.layout
        sm = context.scene.spacemouse_settings

        layout.prop(sm, "mode", expand=True)
        row = layout.row(align=True)
        row.prop(sm, "pivot_mode", expand=True)
        layout.prop(sm, "use_view_space", toggle=True, icon='TRACKING')
        layout.prop(sm, "sensitivity", slider=True)
        layout.operator("view3d.ndof_reset_object", text="Reset Target(s)", icon='LOOP_BACK')

        layout.separator()
        box = layout.box()
        box.prop(sm, "show_inversion_settings",
                 icon='TRIA_DOWN' if sm.show_inversion_settings else 'TRIA_RIGHT',
                 emboss=False)
        if sm.show_inversion_settings:
            col = box.column(align=True)
            col.label(text="Translation:")
            col.prop(sm, "inv_tx"); col.prop(sm, "inv_ty"); col.prop(sm, "inv_tz")
            col.separator()
            col.label(text="Rotation:")
            col.prop(sm, "inv_rx"); col.prop(sm, "inv_ry"); col.prop(sm, "inv_rz")

        layout.separator()
        layout.label(text="Shortcuts:")
        self._draw_kmi(layout, context, sm, "view3d.ndof_toggle_mode", "Toggle Mode")
        row = layout.row(align=True)
        row.prop(sm, "lock_horizon", toggle=True,
                 icon='LOCKVIEW_ON' if sm.lock_horizon else 'LOCKVIEW_OFF')
        self._draw_kmi(layout, context, sm, "view3d.ndof_toggle_lock_horizon", "Toggle Lock Horizon")

        layout.separator()
        running = context.window_manager.get("ndof_modal_running", False)
        layout.label(text=f"Listener: {'RUNNING' if running else 'STOPPED'}",
                     icon='REC' if running else 'PAUSE')
        if not running:
            layout.operator("view3d.ndof_object_control", text="Start NDOF Listener", icon='PLAY')
        else:
            layout.operator("view3d.ndof_cancel_control", text="Stop NDOF Listener", icon='CANCEL')

        layout.separator()
        layout.label(text="Debug Info:")
        layout.label(text=sm.debug_info, icon='INFO')

    @staticmethod
    def _draw_kmi(layout, context, sm, idname, label):
        wm = context.window_manager
        for kc in (wm.keyconfigs.user, wm.keyconfigs.addon):
            if not kc:
                continue
            km = kc.keymaps.get('3D View')
            if not km:
                continue
            kmi = next((i for i in km.keymap_items if i.idname == idname), None)
            if not kmi:
                continue
            col = layout.column()
            col.context_pointer_set("keymap", km)
            col.label(text=f"{label}:")
            rna_keymap_ui.draw_kmi([], kc, km, kmi, col, 0)
            return
        layout.label(text=f"{label}: (no keymap entry)", icon='ERROR')


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

classes = (
    SpaceMouseSettings,
    VIEW3D_PT_spacemouse_control,
    NDOF_OT_object_control,
    NDOF_OT_cancel_control,
    NDOF_OT_reset_object,
    NDOF_OT_toggle_mode,
    NDOF_OT_toggle_lock_horizon,
)

addon_keymaps = []


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.spacemouse_settings = bpy.props.PointerProperty(type=SpaceMouseSettings)

    def _sync_lock_horizon():
        try:
            current = bpy.context.preferences.inputs.view_rotate_method
            for scene in bpy.data.scenes:
                scene.spacemouse_settings.lock_horizon = (current == 'TURNTABLE')
        except Exception:
            pass
        return None  # one-shot
    bpy.app.timers.register(_sync_lock_horizon, first_interval=0.1)

    def _autostart_listener():
        try:
            wm = bpy.context.window_manager
            if wm.get("ndof_modal_running", False):
                return None
            for win in wm.windows:
                for area in win.screen.areas:
                    if area.type == 'VIEW_3D':
                        for region in area.regions:
                            if region.type == 'WINDOW':
                                with bpy.context.temp_override(window=win, area=area, region=region):
                                    bpy.ops.view3d.ndof_object_control('INVOKE_DEFAULT')
                                return None
        except Exception as e:
            print("[Telekinez] autostart failed:", e)
        return None
    bpy.app.timers.register(_autostart_listener, first_interval=0.5)

    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc:
        km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
        kmi = km.keymap_items.new(NDOF_OT_toggle_mode.bl_idname,
                                  type='M', value='PRESS', ctrl=True, shift=True)
        addon_keymaps.append((km, kmi))
        kmi = km.keymap_items.new(NDOF_OT_toggle_lock_horizon.bl_idname,
                                  type='H', value='PRESS', ctrl=True, shift=True)
        addon_keymaps.append((km, kmi))


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.spacemouse_settings
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()


if __name__ == "__main__":
    register()
