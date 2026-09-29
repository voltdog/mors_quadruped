# HeightMapBuilder

Текущая реализация покрывает шаги 1-10 из `plan.md`:

1. Получение входных данных из LCM:
   - `DEPTH_IMAGE` (`depth_image_msg`)
   - `ROBOT_STATE` (`robot_state_msg`)
2. Предобработка depth:
   - отбрасывание невалидных значений
   - фильтрация по диапазону глубины
   - опциональный downsampling
   - подавление выбросов по локальной медиане
3. Восстановление 3D-точек в системе камеры по intrinsics.
4. Преобразование `camera -> body -> world` по `T_body_camera` и позе робота из `ROBOT_STATE`.
5. Построение внутренней 2.5D карты высот (регулярная XY-сетка в world frame, rolling-map буфер).
6. Выделение локального окна вокруг робота.
7. Морфологическая фильтрация `opening` (`erosion -> dilation`).
8. Вычисление `gradient map` оператором Sobel на отфильтрованной карте.
9. Классификация `traversability map`:
   - `STEPPABLE` при `gradient <= grad_thr_steppable`
   - `UNSTEPPABLE` при `grad_thr_steppable < gradient <= grad_thr_unsteppable`
   - `IMPASSABLE` при `gradient > grad_thr_unsteppable`
10. Публикация в `HEIGHTMAP` packed `heightmap_msg`.

Результаты публикуются:
- в `POINCLOUD` сообщением `pointcloud_msg` (точки в world frame),
- в `HEIGHTMAP` сообщением `heightmap_msg` (локальное окно 2.5D карты, row-major packed cells).

Для каждой packed-ячейки `HEIGHTMAP` используется схема:
- `bit 15`: `valid`
- `bit 14..13`: `traversability_class` (`00` steppable, `01` unsteppable, `10` impassable)
- `bit 12..0`: квантованная высота `height_q`

Параметры положения и наклона камеры берутся из `config/heightmap_builder.yaml`:
- `transforms.camera_frame.position_body_m`
- `transforms.camera_frame.euler_deg`
- `transforms.depth_camera.position_camera_frame_m`
- `transforms.depth_camera.euler_deg`

Параметры 2.5D карты и окна:
- `map.cell_size`
- `map.global_size_x`, `map.global_size_y`
- `map.rolling_enabled`
- `map.rolling_margin_cells_x`, `map.rolling_margin_cells_y`
- `map.local_window_cells_x`, `map.local_window_cells_y`
- `map.height_min`, `map.height_max`, `map.height_resolution`
- имя канала карты (`heightmap`) берётся из `config/channels.yaml`

### Rolling global map

- Глобальная карта хранится в фиксированном буфере размера `global_size_x/global_size_y`.
- При `map.rolling_enabled: true` буфер автоматически сдвигается, когда робот подходит к краю безопасной зоны.
- Безопасная зона задаётся `rolling_margin_cells_x/y` (значение `0` = auto: `max(local_window/2, 1)` с последующим clamp).
- При сдвиге новые полосы карты очищаются в `unknown`, поэтому память ограничена фиксированным размером, а дальняя история отбрасывается.
- Для больших телепортов (сдвиг >= размера буфера по любой оси) карта полностью сбрасывается вокруг текущей позиции робота.

Параметры градиента и проходимости:
- `gradient.method` (сейчас поддерживается `sobel`)
- `gradient.sobel_scale`
- `traversability.grad_thr_steppable`
- `traversability.grad_thr_unsteppable`
- `traversability.unknown_is_impassable`

## Сборка

```bash
cd HeightMapBuilder
mkdir -p build
cd build
cmake ..
make -j
```

## Запуск

```bash
./height_map_builder
```

По умолчанию конфиг читается из:
- `$CONFIGPATH/heightmap_builder.yaml`, либо
- `../config/heightmap_builder.yaml` (fallback).
