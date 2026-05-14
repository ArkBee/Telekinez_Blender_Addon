# Telekinez SpaceMouse — Техническая документация

**Файл:** `spacemouse_control.py`
**Версия:** 3.0
**Целевая платформа:** Blender 4.5+

---

## 1. Назначение

Аддон превращает 3D-манипулятор (SpaceMouse / NDOF-устройство) в универсальный контроллер для Blender'а с тремя независимыми режимами работы:

- **CAMERA** — управление полётом viewport'а (drone-style).
- **OBJECT** — перемещение и вращение выделенных объектов относительно камеры; viewport при этом стоит.
- **POSE** — то же самое для выделенных pose-bones в режиме Pose Mode.

Главное архитектурное отличие от стандартного NDOF-управления Blender'а: аддон **полностью перехватывает сырые NDOF-события** и сам решает, во что их превращать. Blender'овская встроенная NDOF-навигация (Fly/Orbit, Lock Horizon, Turntable/Trackball) при этом обходится — она не фильтрует, не подавляет и не перенаправляет ни одну ось.

---

## 2. Архитектура

### 2.1. Высокоуровневая схема

```
SpaceMouse физическое устройство
        │
        ▼
Blender NDOF driver
        │  генерирует событие event.type == 'NDOF_MOTION'
        │  с атрибутом event.ndof_motion
        ▼
NDOF_OT_listener.modal()     ◄── модальный оператор, постоянно работает
        │
        ▼  читает event.ndof_motion.translation / .rotation
_read_ndof()
        │  применяет invert-флаги и speed-множители
        ▼
        ├─► _apply_camera()      (mode == CAMERA)
        │       └─► rv3d.view_location, rv3d.view_rotation
        │
        └─► _apply_to_targets()  (mode == OBJECT / POSE)
                └─► obj.matrix_world  или  pbone.matrix
```

### 2.2. Ключевая идея

В предыдущих версиях аддон **наблюдал** за тем, как Blender изменил viewport под действием NDOF, и вычислял дельту через `cur_rot @ saved_rot.inverted()`. Этот подход хрупок: Blender в зависимости от режима навигации направляет одни и те же физические оси устройства в разные стороны (translation → pan, zoom → distance, yaw → orbit без изменения rotation), и часть осей просто пропадает.

В версии 3.0 аддон **сам** читает сырые `event.ndof_motion.translation` и `.rotation` и применяет их так, как считает нужным, минуя viewport-навигацию Blender'а. Событие `NDOF_MOTION` всегда возвращается со статусом `{'RUNNING_MODAL'}` — это **потребляет** событие, и встроенная NDOF-навигация Blender'а его не получает.

### 2.3. Модальный оператор

`NDOF_OT_listener` — единственный долгоживущий оператор. Запускается:

- Автоматически при регистрации аддона через `bpy.app.timers.register(_autostart_listener, first_interval=0.3)`.
- Вручную кнопкой Start в N-панели.

Останавливается:

- Кнопкой Stop в N-панели.
- Программно через `wm["ndof_modal_running"] = False`.

Проверка `wm["ndof_modal_running"]` происходит в начале каждого вызова `modal()`. Если флаг сброшен — оператор завершается через `_cleanup()`.

---

## 3. Структуры данных

### 3.1. `SpaceMouseSettings` (PropertyGroup)

Хранится на `Scene.spacemouse_settings`, сохраняется в `.blend` файле.

| Свойство | Тип | По умолчанию | Назначение |
|---|---|---|---|
| `mode` | EnumProperty | `'CAMERA'` | Активный режим: CAMERA / OBJECT / POSE |
| `move_speed` | FloatProperty | 0.05 | Множитель translation (мир-единиц на один NDOF-тик при полном отклонении) |
| `rot_speed` | FloatProperty | 0.03 | Множитель rotation (радиан на один NDOF-тик при полном отклонении) |
| `invert_tx/ty/tz` | BoolProperty | True/True/True | Инверсия осей translation в локальной системе устройства |
| `invert_rx/ry/rz` | BoolProperty | False/False/True | Инверсия Pitch / Yaw / Roll |
| `lock_horizon` | BoolProperty | True | В CAMERA-режиме yaw вокруг мирового Z вместо camera_up; roll подавляется |
| `debug_info` | StringProperty | "" | Текст в N-панели для диагностики |

### 3.2. Target-обёртки

Унификация работы с объектами и pose-bones через duck typing:

```python
class _ObjectTarget:    # обёртка над bpy.types.Object
class _PoseBoneTarget:  # обёртка над bpy.types.PoseBone + Armature
```

Оба класса предоставляют интерфейс:

- `world_pivot() → Vector` — мировая позиция точки вращения.
- `world_matrix() → Matrix` — текущая мировая матрица.
- `set_world_matrix(m)` — установка новой мировой матрицы.

Для pose-bone `set_world_matrix` пересчитывает через `arm.matrix_world.inverted() @ m`, потому что `pbone.matrix` хранится в armature-space.

### 3.3. `_collect_targets(context) → list`

Собирает список целей в зависимости от текущего режима:

- **POSE**: `context.selected_pose_bones`, иначе активная кость armature, если активный объект — Armature в Pose Mode.
- **OBJECT**: `context.selected_objects`, иначе активный объект.
- **CAMERA**: пустой список (в CAMERA-режиме функция не используется).

---

## 4. Поток обработки NDOF-события

### 4.1. Получение сырых данных — `_read_ndof(event, sm)`

```python
nm = event.ndof_motion
t = nm.translation   # Vector(x, y, z), компоненты ~[-1, +1]
r = nm.rotation      # Vector(x, y, z), axis*angle в координатах устройства
```

**Конвенция осей SpaceMouse в Blender:**

- `t.x` = right (+) / left (−)
- `t.y` = up (+) / down (−)
- `t.z` = **toward user** (+) / away from user (−)

Аддон сразу переворачивает Z, чтобы дальше работать с `+Z = "вглубь сцены"`:

```python
t_local = Vector((tx, ty, -tz)) * move_speed
```

Также применяется каждый из шести `invert_*` флагов.

**Конвенция осей вращения:**

- `r.x` = pitch (наклон вверх/вниз)
- `r.y` = yaw (поворот влево/вправо вокруг вертикали устройства)
- `r.z` = roll (поворот вокруг продольной оси устройства)

### 4.2. Idle-фильтр

```python
if t_local.length < 1e-4 and r_local.length < 1e-4:
    return {'RUNNING_MODAL'}  # потребляем событие, но ничего не делаем
```

Это нужно, потому что устройство в покое всё равно посылает мелкий шум. Без фильтра объект бы тихо дрейфовал.

Важная деталь: возвращается `'RUNNING_MODAL'`, а не `'PASS_THROUGH'`. Шумные события **поглощаются**, чтобы они не дошли до встроенной NDOF-навигации Blender'а и не сдвинули viewport.

---

## 5. Режим CAMERA — `_apply_camera()`

### 5.1. Translation

Строится базис камеры из `rv3d.view_rotation`:

```python
cam_right   = view_rot @ Vector(( 1,  0,  0))   # вправо на экране
cam_up      = view_rot @ Vector(( 0,  1,  0))   # вверх на экране
cam_forward = view_rot @ Vector(( 0,  0, -1))   # куда смотрит камера
```

(В Blender'е `view_rotation` поворачивает мир в координаты камеры, и взгляд камеры идёт по `-Z` в собственной системе, поэтому `forward = view_rot @ (0,0,-1)`.)

Translation в мировом пространстве:

```python
world_move = cam_right * t_local.x  +  cam_up * t_local.y  +  cam_forward * t_local.z
rv3d.view_location += world_move
```

### 5.2. Rotation

Три отдельных кватерниона, каждый вокруг своей оси:

```python
pitch    = Quaternion(cam_right,   r_local.x)
yaw_axis = Vector((0,0,1)) if lock_horizon else cam_up
yaw      = Quaternion(yaw_axis,    r_local.y)
roll     = Quaternion(cam_forward, 0.0 if lock_horizon else r_local.z)
```

**Lock Horizon** делает две вещи:

1. Yaw происходит вокруг мирового Z, а не вокруг локального camera-up. Это даёт turntable-навигацию: горизонт остаётся горизонтальным даже после серии наклонов.
2. Roll **полностью подавляется** (`angle = 0.0`). Без этого крен от устройства аккумулируется и горизонт всё-таки кривится.

Композиция и применение:

```python
delta = yaw @ pitch @ roll
rv3d.view_rotation = (delta @ view_rot).normalized()
```

Порядок умножения: сначала к view_rotation применяется delta слева — это означает поворот в **мировой** системе (потому что оси `cam_*` уже были получены в мире). `normalized()` страхует от накопления численной погрешности кватерниона.

### 5.3. Перерисовка

`_redraw_view3d(context)` помечает все VIEW_3D области грязными через `area.tag_redraw()`. Без этого viewport обновляется только когда мышь двигается над ним.

---

## 6. Режим OBJECT / POSE — `_apply_to_targets()`

### 6.1. Translation

Та же логика, что в CAMERA, но применяется к объекту:

```python
world_move = cam_right*t.x + cam_up*t.y + cam_forward*t.z
```

Это даёт **camera-relative** управление: толкнул джойстик вправо — объект едет вправо относительно того, как ты сейчас смотришь, независимо от мировой ориентации.

### 6.2. Rotation

```python
pitch = Quaternion(cam_right,   r_local.x)
yaw   = Quaternion(cam_up,      r_local.y)
roll  = Quaternion(cam_forward, r_local.z)
world_rot = (yaw @ pitch @ roll).normalized()
```

В отличие от CAMERA-режима, `lock_horizon` здесь **не учитывается**: при манипуляции объектом естественно использовать все три оси экрана. Если нужно ограничить вращение объекта вертикалью — есть отдельный invert-флаг на Yaw, либо можно отключить ось руками в Edit > Preferences > NDOF.

### 6.3. Pivot и применение

Точка вращения — медиана мировых позиций всех целей:

```python
pivot = sum(t.world_pivot() for t in targets) / len(targets)
```

Матрица трансформации в мировом пространстве:

```python
rot4 = world_rot.to_matrix().to_4x4()
base = Matrix.Translation(pivot) @ rot4 @ Matrix.Translation(-pivot)
```

Это стандартный "rotate around pivot" — сдвинуть pivot в ноль, повернуть, вернуть на место. К каждому target'у применяется:

```python
m = base @ tgt.world_matrix()
m.translation += world_move
tgt.set_world_matrix(m)
```

Translation добавляется **после** поворота — иначе translation сам повернулся бы вместе с объектом, что неинтуитивно для пользователя.

### 6.4. Viewport не трогается

В этом режиме `rv3d` используется **только для чтения** базиса камеры. Никаких `rv3d.view_location = ...`. Это и означает, что в OBJECT/POSE камера стоит, а объект двигается — независимое управление.

---

## 7. UI — `VIEW3D_PT_spacemouse`

Панель размещается в N-панели 3D viewport'а, вкладка "Telekinez".

Состав сверху вниз:

1. **Mode** — три кнопки `expand=True` для переключения CAMERA / OBJECT / POSE.
2. **Move / Rotate** — слайдеры скоростей.
3. **Invert Axes box** — две строки по три toggle-кнопки: X/Y/Z и Pitch/Yaw/Roll.
4. **Lock Horizon** — toggle с иконкой LOCKVIEW_ON/OFF.
5. **Start / Stop** — кнопка запуска/остановки модального оператора.
6. **Reset Target (0,0,0)** — сброс позиции и поворота выделенных целей.
7. **Debug info** — последняя строка диагностики из `_apply_*()`.
8. **Mode Hotkeys** — список биндингов через `rna_keymap_ui.draw_kmi`, позволяющий редактировать клавиши прямо из панели.

---

## 8. Операторы

| `bl_idname` | Назначение |
|---|---|
| `view3d.ndof_listener` | Главный модальный оператор. Запускает прослушку NDOF-событий. |
| `view3d.ndof_stop` | Останавливает прослушку (сбрасывает флаг `ndof_modal_running`). |
| `view3d.ndof_reset_target` | Обнуляет `location` и `rotation` выделенных объектов/костей. |
| `view3d.ndof_set_mode` | Переключает `sm.mode`. Привязан к хоткеям. |

### 8.1. `view3d.ndof_set_mode`

Принимает Enum-параметр `target` (CAMERA / OBJECT / POSE). По умолчанию забинжен на:

- **Shift+,** (COMMA) → CAMERA
- **Shift+.** (PERIOD) → OBJECT

Биндинги создаются в addon-keyconfig в `register()` и удаляются в `unregister()`. Пользователь может изменить их через UI N-панели или Edit > Preferences > Keymap.

---

## 9. Регистрация и жизненный цикл

### 9.1. `register()`

1. Регистрирует все классы из `classes`.
2. Создаёт `Scene.spacemouse_settings` через `PointerProperty`.
3. Сбрасывает `wm["ndof_modal_running"]` в False.
4. Откладывает автозапуск слушателя на 0.3 секунды через `bpy.app.timers` — нужна задержка, чтобы Blender успел создать VIEW_3D areas.
5. **Sweep stale keymap entries**: проходит по обоим keyconfig (addon и user) и удаляет старые биндинги для устаревших `idname` (toggle_mode_1, toggle_lock_horizon, cycle_mode и т.п.). Это нужно, потому что Blender сохраняет user keymap в пользовательских настройках — после обновления аддона мёртвые биндинги остались бы перехватывать клавиши, и новые биндинги не работали бы.
6. Создаёт свежие биндинги в addon-keyconfig.

### 9.2. `_autostart_listener()`

Callback для `bpy.app.timers`. Ищет первый VIEW_3D area, делает временный override контекста через `bpy.context.temp_override(window=..., area=..., region=...)` и вызывает оператор через `INVOKE_DEFAULT`. Возвращает `None` — это говорит таймеру, что повторять не нужно.

Если в момент срабатывания таймера VIEW_3D ещё не существует (например, Blender запущен в headless-режиме), функция тихо ничего не делает.

### 9.3. `unregister()`

1. Сбрасывает `ndof_modal_running` (модальный оператор увидит это на следующем тике и сам завершится).
2. Удаляет все биндинги из `addon_keymaps`.
3. Удаляет все классы и `Scene.spacemouse_settings`.

---

## 10. Известные ограничения и компромиссы

### 10.1. Захват событий в режиме CAMERA

Модальный оператор возвращает `{'RUNNING_MODAL'}` для NDOF-событий **во всех режимах**, включая CAMERA. Это означает: встроенная Blender'овская NDOF-навигация в нашем CAMERA-режиме **не работает**. Мы навигируем viewport сами через `_apply_camera()`. Преимущество — единообразное поведение по всем шести осям; недостаток — настройки `Edit > Preferences > Input > NDOF` влияют только на нашу логику в той части, которую мы сами не дублируем.

### 10.2. `event.ndof_motion` может быть `None`

В редких случаях Blender присылает событие `NDOF_MOTION` с `event.ndof_motion is None` (документация это допускает). `_read_ndof()` возвращает `(None, None)`, и обработчик пропускает такой тик через `PASS_THROUGH`.

### 10.3. Множественное выделение и pivot

Когда выбрано N объектов, точка вращения — их **медиана**. Это похоже на стандартный pivot "Median Point" в Blender. Альтернативные стратегии (individual origins, 3D cursor, active element) не поддерживаются — все объекты вращаются как жёсткая группа вокруг общей точки.

### 10.4. Idle-threshold

`1e-4` — компромиссное значение. Если SpaceMouse имеет особенно шумную центральную зону, объект может слегка дёргаться. Если порог увеличить — теряется чувствительность при медленных деликатных движениях. При проблемах правится в `NDOF_OT_listener.modal()`.

### 10.5. Pose Mode и зависимости bone'ов

При вращении нескольких выделенных bone'ов формула pivot-rotation применяется к каждому **независимо**. Если кости связаны иерархией parent-child, результат может удивить: родительская кость потащит дочерние своим поворотом, и наш дополнительный поворот ляжет поверх. Для типичного rigging-сценария это ожидаемое поведение, но при сложных иерархиях имеет смысл выделять одну кость за раз.

---

## 11. Расширение

### 11.1. Добавить новый режим

1. Добавить значение в Enum `mode` в `SpaceMouseSettings`.
2. Если есть новый тип target — создать класс с интерфейсом `world_pivot/world_matrix/set_world_matrix` и добавить ветку в `_collect_targets()`.
3. Если логика принципиально другая — добавить ветку в `modal()` рядом с `_apply_camera` / `_apply_to_targets`.

### 11.2. Per-mode настройки скорости

Сейчас `move_speed` и `rot_speed` общие для всех режимов. Чтобы развести — добавить отдельные `FloatProperty` (например, `move_speed_obj`, `move_speed_cam`) и выбирать нужное по `sm.mode` внутри `_read_ndof()`.

### 11.3. Захват NDOF-кнопок

`event.type` принимает значения `NDOF_BUTTON_1`, `NDOF_BUTTON_2` и т.д. (полный список — в Blender enum `event_type_items`). Достаточно добавить ветку в `modal()`:

```python
if event.type == 'NDOF_BUTTON_1' and event.value == 'PRESS':
    sm.mode = 'OBJECT'
    return {'RUNNING_MODAL'}
```
