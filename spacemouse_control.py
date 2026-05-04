bl_info = {
    "name": "Telekinez SpaceMouse",
    "author": "Telekinez",
    "version": (1, 0),
    "blender": (4, 0, 0),
    "location": "View3D > N-Panel > Telekinez",
    "description": "Control object instead of camera using SpaceMouse",
    "category": "3D View",
}

import time
import mathutils
import rna_keymap_ui

import bpy
from bpy.props import EnumProperty, StringProperty, FloatProperty, BoolProperty

def update_mode(self, context):
    prefs = context.preferences.inputs
    if self.mode == 'OBJECT':
        # Сохраняем пользовательские настройки камеры
        self.saved_orbit = prefs.ndof_orbit
        self.saved_lock_horizon = prefs.use_ndof_lock_horizon
        
        # Для объекта включаем Trackball и выключаем Lock Horizon
        prefs.ndof_orbit = 'TRACKBALL'
        prefs.use_ndof_lock_horizon = False
        
        # Центрируем ось вращения NDOF на объекте (помещаем джойстик "внутрь" объекта)
        if context.active_object and context.space_data and context.space_data.type == 'VIEW_3D':
            rv3d = context.space_data.region_3d
            rv3d.view_location = context.active_object.location
    else:
        # Восстанавливаем настройки камеры
        if self.saved_orbit != "":
            prefs.ndof_orbit = self.saved_orbit
            prefs.use_ndof_lock_horizon = self.saved_lock_horizon

class SpaceMouseSettings(bpy.types.PropertyGroup):
    mode: EnumProperty(
        name="Control Mode",
        description="Select what SpaceMouse controls",
        items=[
            ('CAMERA', "Camera (Default)", "Standard viewport camera control"),
            ('OBJECT', "Object", "Control the active object"),
        ],
        default='CAMERA',
        update=update_mode
    )
    saved_orbit: StringProperty(default="")
    saved_lock_horizon: BoolProperty(default=True)

    use_view_space: BoolProperty(
        name="View Relative",
        description="Move and rotate relative to camera view",
        default=True
    )

    sensitivity: FloatProperty(
        name="Sensitivity",
        description="Multiplier for movement and rotation",
        default=0.1,
        min=0.001,
        max=5.0,
        step=1
    )
    debug_info: StringProperty(
        name="Debug",
        description="Last event Info",
        default="Ready..."
    )

    show_inversion_settings: BoolProperty(
        name="Invert Axes Settings",
        description="Show separate axis inversion settings",
        default=False
    )
    inv_tx: BoolProperty(name="Invert Move X (Left/Right)", default=False)
    inv_ty: BoolProperty(name="Invert Move Y (Forward/Back)", default=False)
    inv_tz: BoolProperty(name="Invert Move Z (Up/Down)", default=False)

    inv_rx: BoolProperty(name="Invert Pitch (Tilt F/B)", default=False)
    inv_ry: BoolProperty(name="Invert Roll (Tilt L/R)", default=False)
    inv_rz: BoolProperty(name="Invert Yaw (Twist)", default=False)

class NDOF_OT_reset_object(bpy.types.Operator):
    """Сбросить координаты объекта по нулям"""
    bl_idname = "view3d.ndof_reset_object"
    bl_label = "Reset Object Transform"
    
    def execute(self, context):
        obj = context.active_object
        if obj:
            obj.location = (0, 0, 0)
            obj.rotation_euler = (0, 0, 0)
            
            # Если нужно сфокусироваться
            if context.space_data.type == 'VIEW_3D':
                context.space_data.region_3d.view_location = obj.location
                
            self.report({'INFO'}, f"Reset {obj.name} to origin")
        return {'FINISHED'}

class NDOF_OT_toggle_mode(bpy.types.Operator):
    """Переключить режим (Camera / Object) с клавиатуры/джойстика"""
    bl_idname = "view3d.ndof_toggle_mode"
    bl_label = "Toggle SpaceMouse Mode"
    
    def execute(self, context):
        settings = context.scene.spacemouse_settings
        if settings.mode == 'CAMERA':
            settings.mode = 'OBJECT'
        else:
            settings.mode = 'CAMERA'
        
        self.report({'INFO'}, f"SpaceMouse Mode: {settings.mode}")
        return {'FINISHED'}

class VIEW3D_PT_spacemouse_control(bpy.types.Panel):
    """Creates a Panel in the 3D-Viewport N-Menu"""
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Telekinez"
    bl_label = "Telekinez SpaceMouse"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        sm_settings = scene.spacemouse_settings

        layout.prop(sm_settings, "mode", expand=True)
        layout.prop(sm_settings, "use_view_space", toggle=True, icon='TRACKING')
        layout.prop(sm_settings, "sensitivity", slider=True)
        
        layout.operator("view3d.ndof_reset_object", text="Reset Object to Zero", icon='LOOP_BACK')
        
        layout.separator()
        
        # Выпадающий список (Box) инверсии осей
        box = layout.box()
        box.prop(sm_settings, "show_inversion_settings", 
                 icon='TRIA_DOWN' if sm_settings.show_inversion_settings else 'TRIA_RIGHT', 
                 emboss=False)
        
        if sm_settings.show_inversion_settings:
            col = box.column(align=True)
            col.label(text="Translation:")
            col.prop(sm_settings, "inv_tx")
            col.prop(sm_settings, "inv_ty")
            col.prop(sm_settings, "inv_tz")
            col.separator()
            col.label(text="Rotation:")
            col.prop(sm_settings, "inv_rx")
            col.prop(sm_settings, "inv_ry")
            col.prop(sm_settings, "inv_rz")
            col.separator()
        
        layout.separator()
        
        # Получаем горячую клавишу для переключения и рисуем ее здесь же в панели
        wm = context.window_manager
        kc = wm.keyconfigs.user
        if kc:
            km = kc.keymaps.get('3D View')
            if km:
                kmi = next((item for item in km.keymap_items if item.idname == "view3d.ndof_toggle_mode"), None)
                if kmi:
                    col = layout.column()
                    col.label(text="Toggle Mode Shortcut:")
                    
                    col.context_pointer_set("keymap", km)
                    rna_keymap_ui.draw_kmi([], kc, km, kmi, col, 0)
        
        layout.separator()
        
        # Оператор для запуска модального слушателя, если он еще не запущен
        if not context.window_manager.get("ndof_modal_running", False):
            layout.operator("view3d.ndof_object_control", text="Start NDOF Listener", icon='PLAY')
        else:
            layout.operator("view3d.ndof_cancel_control", text="Stop NDOF Listener", icon='CANCEL')
            
        layout.separator()
        layout.label(text="Debug Info:")
        layout.label(text=sm_settings.debug_info, icon='INFO')


class NDOF_OT_object_control(bpy.types.Operator):
    """Слушатель SpaceMouse событий (Modal)"""
    bl_idname = "view3d.ndof_object_control"
    bl_label = "NDOF Object Control Modal"

    def modal(self, context, event):
        sm_settings = context.scene.spacemouse_settings

        # Если выключили слушатель через кнопку стоп
        if not context.window_manager.get("ndof_modal_running", False):
            self.cancel(context)
            sm_settings.debug_info = "Stopped."
            if context.area:
                context.area.tag_redraw()
            return {'FINISHED'}

        # Наш хак: Blender не отдает сырые координаты осей напрямую в Python.
        # Поэтому мы пропускаем событие (PASS_THROUGH), позволяя Blender "сдвинуть" камеру окна (вьюпорт),
        # а затем в событии таймера высчитываем это смещение, применяем к объекту и возвращаем камеру назад!
        
        rv3d = None
        if context.space_data and context.space_data.type == 'VIEW_3D':
            rv3d = context.space_data.region_3d

        if event.type == 'TIMER' and rv3d:
            current_loc = rv3d.view_location.copy()
            current_rot = rv3d.view_rotation.copy()
            current_dist = rv3d.view_distance

            # Инициализация первого кадра
            if self.saved_loc is None:
                self.saved_loc = current_loc
                self.saved_rot = current_rot
                self.saved_dist = current_dist

            is_ndof_moving = (time.time() - self.last_ndof_time < 0.2)

            if sm_settings.mode == 'OBJECT' and is_ndof_moving:
                # Проверяем, сдвинулся ли вьюпорт
                if (current_loc - self.saved_loc).length > 0.0001 or \
                   current_rot.rotation_difference(self.saved_rot).angle > 0.0001 or \
                   abs(current_dist - self.saved_dist) > 0.0001:
                    
                    obj = context.active_object
                    if obj:
                        sens = sm_settings.sensitivity

                        # 1. Извлекаем "чистые" (аппаратные) движения джойстика из смещения камеры во View-Space
                        # Это позволит нам отвязать джойстик от угла обзора камеры и получить сырые X/Y/Z
                        q_view = self.saved_rot.copy()
                        local_diff = q_view.inverted() @ (current_loc - self.saved_loc)
                        delta_dist = current_dist - self.saved_dist
                        
                        # Определение осей для перемещения (с учетом инверсии пользовательских настроек):
                        sm_tx = local_diff.x * (-1 if sm_settings.inv_tx else 1)
                        sm_ty = (-delta_dist - local_diff.z) * (-1 if sm_settings.inv_ty else 1)
                        sm_tz = local_diff.y * (-1 if sm_settings.inv_tz else 1)
                        
                        # 2. Извлекаем вращение
                        q_diff_local = q_view.inverted() @ current_rot.copy()
                        euler_diff = q_diff_local.to_euler('XYZ')
                        
                        # Применение инверсии вращения
                        sm_rx = euler_diff.x * (-1 if sm_settings.inv_rx else 1)
                        sm_ry = -euler_diff.z * (-1 if sm_settings.inv_ry else 1)
                        sm_rz = -euler_diff.y * (-1 if sm_settings.inv_rz else 1)
                        
                        if sm_settings.use_view_space:
                            # Перемещение относительно камеры (View Space)
                            # Конвертируем аппаратные оси обратно в оси камеры (X-вправо, Y-вверх, Z-к нам)
                            view_move = mathutils.Vector((sm_tx, sm_tz, -sm_ty)) * sens
                            obj.location += q_view @ view_move
                            
                            # Вращение относительно камеры
                            view_rot_euler = mathutils.Euler((sm_rx * sens, sm_rz * sens, -sm_ry * sens), 'XYZ')
                            delta_rot_mat = q_view.to_matrix() @ view_rot_euler.to_matrix() @ q_view.to_matrix().inverted()
                            obj.rotation_euler = (delta_rot_mat @ obj.rotation_euler.to_matrix()).to_euler(obj.rotation_mode)
                        else:
                            # Глобальное перемещение и вращение
                            obj.location.x += sm_tx * sens
                            obj.location.y += sm_ty * sens
                            obj.location.z += sm_tz * sens
                            
                            delta_eu = mathutils.Euler((sm_rx * sens, sm_ry * sens, sm_rz * sens), 'XYZ')
                            delta_rot_mat = delta_eu.to_matrix()
                            obj.rotation_euler = (delta_rot_mat @ obj.rotation_euler.to_matrix()).to_euler(obj.rotation_mode)
                        
                        sm_settings.debug_info = (
                            f"T({round(sm_tx,3)}, {round(sm_ty,3)}, {round(sm_tz,3)}) "
                            f"R({round(sm_rx,3)}, {round(sm_ry,3)}, {round(sm_rz,3)})"
                        )
                    
                        # ОЧЕНЬ ВАЖНО: Возвращаем камеру назад, чтобы было чувство, что она заблокирована
                        rv3d.view_location = self.saved_loc
                        rv3d.view_rotation = self.saved_rot
                        rv3d.view_distance = self.saved_dist
            else:
                # В режиме CAMERA камера двигается штатно, просто обновляем сохраненные данные
                self.saved_loc = current_loc
                self.saved_rot = current_rot
                self.saved_dist = current_dist
                
            # Перерисовываем UI панель
            if context.area:
                context.area.tag_redraw()

        if event.type == 'NDOF_MOTION':
            self.last_ndof_time = time.time()
            return {'PASS_THROUGH'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        wm["ndof_modal_running"] = True

        self._timer = None
        self.saved_loc = None
        self.saved_rot = None
        self.saved_dist = None
        self.last_ndof_time = 0.0

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
    """Остановить слушатель SpaceMouse"""
    bl_idname = "view3d.ndof_cancel_control"
    bl_label = "Stop NDOF Modal"
    
    def execute(self, context):
        context.window_manager["ndof_modal_running"] = False
        return {'FINISHED'}


classes = (
    SpaceMouseSettings,
    VIEW3D_PT_spacemouse_control,
    NDOF_OT_object_control,
    NDOF_OT_cancel_control,
    NDOF_OT_reset_object,
    NDOF_OT_toggle_mode,
)

addon_keymaps = []

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.spacemouse_settings = bpy.props.PointerProperty(type=SpaceMouseSettings)
    
    # Добавляем горячую клавишу (по умолчанию Ctrl+Shift+M), вы сможете назначить кнопку джойстика
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc:
        km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
        # Создаем шорткат - Ctrl + Shift + M
        kmi = km.keymap_items.new(NDOF_OT_toggle_mode.bl_idname, type='M', value='PRESS', ctrl=True, shift=True)
        addon_keymaps.append((km, kmi))

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.spacemouse_settings
    
    # Удаляем горячие клавиши
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()

if __name__ == "__main__":
    register()
