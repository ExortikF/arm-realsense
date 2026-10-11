# Управление SO-ARM100 по камере Intel RealSense

Папка самодостаточная: библиотека серв, модель MediaPipe и весь код лежат
здесь. Все команды ниже запускаются из этой папки.

```bash
cd /Users/macbook/Desktop/projects/arm_realsense
```

## 0. Про venv

Своё окружение в этой папке, ни от чего снаружи не зависит.
Стоят: mediapipe, opencv-contrib-python, numpy, pyserial (нужен для серв)
и собранный вручную pyrealsense2.

Если понадобится собрать заново:

```bash
python3.12 -m venv venv && venv/bin/pip install "mediapipe==0.10.35" opencv-contrib-python numpy pyserial
```

## 1. Главная засада: pyrealsense2 на Mac нет в PyPI

`pip install pyrealsense2` на arm64 macOS не сработает — колёс под Mac Intel не
публикует. Формула `brew install librealsense` ставит только C++ библиотеку,
**без** Python-биндингов (в формуле нет `-DBUILD_PYTHON_BINDINGS`).

Значит, собираем из исходников. 10–15 минут на M-процессоре.

```bash
brew install cmake libusb pkgconf
```

```bash
cd ~/Desktop/projects && git clone --depth 1 https://github.com/IntelRealSense/librealsense.git && cd librealsense
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=/Users/macbook/Desktop/projects/arm_realsense/venv/bin/python -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF
```

```bash
cmake --build build -j8
```

`-DFORCE_RSUSB_BACKEND=ON` обязателен: на macOS нет V4L2, работа идёт через
libusb. `-DPYTHON_EXECUTABLE` должен указывать ровно на тот python, из которого
запускаются скрипты, иначе модуль соберётся под другую версию ABI.

После копирования модуль ищет `librealsense2.dylib` по абсолютному пути к
папке сборки. Чтобы исходники можно было удалить (а это 300 МБ) и чтобы
работало под `sudo` (SIP вырезает переменные `DYLD_*`), путь надо
перевести на «рядом с собой», а потом переподписать — правка бинарника
ломает подпись, и на arm64 он просто не загрузится:

```bash
cd /Users/macbook/Desktop/projects/arm_realsense/venv/lib/python3.12/site-packages && install_name_tool -add_rpath @loader_path pyrealsense2.2.58.3.cpython-312-darwin.so && codesign --force -s - pyrealsense2.2.58.3.cpython-312-darwin.so librealsense2.2.58.3.dylib
```

Положить собранный модуль в venv:

```bash
cp -a build/Release/pyrealsense2*.so build/Release/librealsense2*.dylib /Users/macbook/Desktop/projects/arm_realsense/venv/lib/python3.12/site-packages/
```

Проверка:

```bash
cd /Users/macbook/Desktop/projects/arm_realsense && venv/bin/python -c "import pyrealsense2 as rs; print(rs.__version__); print([d.get_info(rs.camera_info.name) for d in rs.context().devices])"
```

Должно напечатать версию. Если список устройств пуст — смотри
следующий раздел, там все три причины разобраны.

## 2. Запуск на macOS: три обязательных условия

Разбирались с этим долго, поэтому по пунктам. Не выполнено любое —
камера «не находится», причём с невнятными сообщениями.

### Кабель Type-C ↔ Type-C

Проверено на практике: с кабелем USB-A камера поднимается как USB 2
(480 Mb/s), и тогда либо не хватает полосы на depth+color, либо она
вовсе подвисает. Дополнительная причина — питание: D455 просит 496 мА
при лимите порта 500, то есть работает на грани.

Проверить, что всё хорошо:

```bash
system_profiler SPUSBDataType | grep -A 8 -i realsense
```

`Speed: Up to 5 Gb/s` — норма. `480 Mb/s` — кабель или порт не тот.
Пропала строка `Speed` и появилось `IOCreatePlugInInterfaceForService
failed` — камера подвисла, помогает только передёрнуть кабель.

### Разрешение «Камера» приложению

Системные настройки → Конфиденциальность и безопасность → Камера →
включить Терминал (или из чего запускаете). **После этого приложение
надо полностью закрыть через Cmd+Q и открыть заново** — macOS проверяет
доступ при старте процесса, на живом приложении переключатель не
действует.

Без этого libusb получает `RS2_USB_STATUS_ACCESS` при попытке забрать
интерфейс, а SDK показывает это как `No device connected`.

### Запуск от root

```bash
sudo venv/bin/python rs_check.py
```

На macOS 15 UVC-интерфейсы держит системное расширение UVCAssistant,
и libusb, в отличие от Linux, не умеет отцеплять системный драйвер —
он отбирает устройство целиком, а на это нужны права root. SIP при
этом не даёт выгрузить само расширение, так что обойтись без sudo
не выйдет. Демона VDCAssistant скрипты сгоняют сами.

## 3. Грабли, на которые уже наступили

**Не ставить visual_preset.** Вызов `set_option(rs.option.visual_preset, ...)`
роняет процесс segfault'ом: смена пресета дёргает питание сенсора, это
запускает переподключение устройства через libusb, а параллельный поток
в этот момент шлёт запрос на уже освобождённый хендл. Работу пресета
мы делаем сами — медианой по окну и отбраковкой по MAD в `patch_depth()`.

**Режим проверять кадром, а не стартом.** `pipeline.start()` спокойно
соглашается на режим, которому не хватает полосы, и падает только через
5 секунд на `wait_for_frames`. Поэтому `open_best_camera()` считает режим
годным лишь тогда, когда реально пришёл кадр.

**mediapipe строго 0.10.35, не новее.** В 1.0.1 калькулятор
`TensorsToDetections` безусловно создаёт Metal-помощник, и на macOS это
кончается `Check failed: service_ Service is unavailable` ещё при создании
HandLandmarker — до всякой камеры и независимо от прав. `delegate=CPU`
не спасает, проверено. Версия закреплена в команде установки выше.

**Не опрашивать сенсоры без нужды.** Обращение к `sensor.profiles`
включает питание сенсора; если RGB кем-то занят, прилетает
`failed to set power state`. Опрос обёрнут в try, есть запасной список.

## 4. Порядок запуска

```bash
cd /Users/macbook/Desktop/projects/arm_realsense && sudo venv/bin/python rs_check.py
```

Проверка камеры без робота. Кликни по картинке — увидишь, во сколько раз
окно 9×9 стабильнее одиночного пикселя. Обычно 5–15×.

```bash
cd /Users/macbook/Desktop/projects/arm_realsense && venv/bin/python calibrate_arm.py
```

Калибровка нулей суставов. **Без неё обратная кинематика работать не будет** —
она считает физические углы, а какому тику соответствует угол 0, знает только
твоя сборка. Инструкция — в докстринге файла.

Перед этим замерь линейкой длины звеньев и впиши в `ArmGeometry`
в [manip/kinematics.py](manip/kinematics.py). Ошибка 5 мм в длине предплечья —
это сантиметр промаха на краю зоны.

Порт серв задан в `manip/arm.py` (`/dev/tty.usbmodem5AE60853301`). Проверить
свой:

```bash
ls /dev/tty.usbmodem*
```

```bash
cd /Users/macbook/Desktop/projects/arm_realsense && sudo venv/bin/python rs_control.py
```

Основной скрипт. Запускается на паузе, команды в сервы идут только после
пробела.

## 5. Что где лежит

| файл | что делает |
|---|---|
| `manip/depth.py` | камера, выравнивание depth→color, **устойчивый замер глубины** |
| `manip/filters.py` | медиана против выбросов, IIR и One Euro против шума |
| `manip/kinematics.py` | прямая и обратная кинематика, рабочая зона |
| `manip/arm.py` | радианы → тики, ограничение скорости, мёртвая зона |
| `rs_control.py` | главный цикл |
| `rs_check.py` | диагностика камеры |
| `calibrate_arm.py` | калибровка нулей и направлений |
| `python_st3215/` | библиотека серв, скопирована как есть |
| `hand_landmarker.task` | модель MediaPipe, ищется в текущей папке |

## 6. Настройка «на глаз», если что-то не так

**Рука дрожит в покое** → в `rs_control.py` уменьшить `min_cutoff` у
`pos_filter` (0.7 → 0.4).

**Рука запаздывает за твоей** → увеличить `beta` (0.6 → 1.2). One Euro тем и
хорош, что эти две ручки почти независимы: `min_cutoff` управляет покоем,
`beta` — динамикой. Крутить именно в таком порядке.

**Рука изредка дёргается** → увеличить окно медианы (`median_size=5` → 7) и
уменьшить `max_step_ticks` в `Arm`.

**Глубина часто теряется** → встань в 0.6–1.0 м (у D455 минимум ~0.5 м), убери из кадра блестящее
и чёрное, добавь света. Процент дырок видно в `rs_check.py`.

**Рука упирается в предел** → надпись `[ПРЕДЕЛ]` на экране. Сузь `ROBOT_BOX`
или расширь `CAM_BOX` в `rs_control.py`.

## 7. Куда двигаться дальше

`CAM_BOX → ROBOT_BOX` — это линейное натягивание коробки на коробку, замена
настоящей hand-eye калибровки. Для управления жестами хватает: человек за
секунду подстраивается под масштаб. Если понадобится, чтобы робот брал
предмет, который видит камера (а не тот, на который ты показываешь), —
понадобится честная калибровка: шахматная доска, `cv2.calibrateHandEye`,
и матрица 4×4 вместо `cam_to_robot()`.
