"""
Работа с Intel RealSense: получение выровненных кадров и —
самое важное — УСТОЙЧИВОЕ измерение глубины.

Почему нельзя просто взять depth[y][x] в одной точке
-----------------------------------------------------
Стереодальномер D4xx считает глубину, сопоставляя патчи левой и правой
ИК-картинок. Из этого следуют три типа мусора, и все три ломают
управление рукой:

1. ДЫРКИ (значение ровно 0). Там, где сопоставить не удалось:
   блик, чёрная/глянцевая поверхность, тень от проектора, слишком
   близко (<~0.2 м у D435, <~0.5 м у D455). Ноль — это НЕ «расстояние 0 метров»,
   это «не знаю». Если такой ноль скормить в map_value, рука дёрнется
   в край диапазона. Нули надо ВЫБРАСЫВАТЬ, а не усреднять.

2. «ЛЕТАЮЩИЕ ПИКСЕЛИ» на границах объекта. На краю ладони патч
   захватывает и руку, и стену за ней, и алгоритм выдаёт нечто среднее —
   точку, висящую в пустоте между ними. Ключевых точек MediaPipe это
   касается напрямую: кончик пальца — это ровно граница объекта,
   худшее место для замера. Поэтому меряем по ЛАДОНИ, а не по кончикам.

3. ШУМ. Среднеквадратичное отклонение растёт как квадрат дистанции:
   на 0.5 м это ~1 мм, на 2 м — уже ~1.5 см, на 3 м — 3.5 см.
   Отсюда простое правило: держите руку в 0.6-1.0 м от камеры
   (для D455; у D435 ближе — 0.4-0.8 м),
   и половина проблем исчезнет сама.

Что делаем
----------
* берём не одну точку, а ОКНО пикселей вокруг каждой точки;
* выкидываем нули;
* берём МЕДИАНУ, а не среднее (среднее один летающий пиксель утащит
  за собой, медиана — нет);
* дополнительно режем выбросы по MAD;
* повторяем для НЕСКОЛЬКИХ точек ладони и берём медиану уже по ним.

Три уровня медианы подряд — звучит избыточно, но именно это и убирает
дрожь, из-за которой «отдельные точки на камере глубины неточны».
"""

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as e:  # чтобы файл можно было импортировать без камеры
    rs = None
    _IMPORT_ERROR = e


# Точки ладони в разметке MediaPipe Hands.
# Намеренно БЕЗ кончиков пальцев (4, 8, 12, 16, 20): они тонкие,
# на них стереоалгоритму не за что зацепиться, и именно там живут
# летающие пиксели. Ладонь — крупная и плоская, идеальная мишень.
PALM_LANDMARKS = (0, 1, 5, 9, 13, 17)


def require_root(script):
    """
    На macOS 15 libusb не может забрать интерфейс камеры у системного
    UVC-драйвера без прав рута — SIP при этом не даёт выгрузить сам
    драйвер. Поэтому запуск от обычного пользователя обречён, и лучше
    сказать это сразу, чем показывать невнятное 'No device connected'.
    """
    import os
    import sys
    if os.geteuid() == 0:
        return
    print(
        "Нужны права root: без них macOS не отдаёт камеру библиотеке.\n"
        "\nЗапускай так:\n"
        f"  sudo killall VDCAssistant 2>/dev/null; sudo venv/bin/python {script}\n"
        "\nVDCAssistant — системный демон, который держит RGB-интерфейс.\n"
        "Убивать его безопасно, он поднимется сам при следующем обращении\n"
        "к камере со стороны системы.",
        file=sys.stderr,
    )
    sys.exit(1)


def explain_start_failure(err, width, height, fps):
    """
    Превратить сухую ошибку SDK в понятную причину.

    Раньше здесь на любую ошибку выдавалась версия про USB 2, и это
    сбивало с толку: «No device connected» к полосе отношения не имеет.
    Причины разные, и лечатся они по-разному, поэтому разделяем.
    """
    msg = str(err)
    head = f"не удалось запустить {width}x{height}@{fps}: {msg}\n"

    if "power state" in msg or "No device connected" in msg:
        return head + (
            "\nПочти наверняка это НЕ камера и НЕ USB, а разрешение macOS.\n"
            "librealsense забирает камеру через libusb, а система не отдаёт\n"
            "интерфейс процессу без доступа к камере — в логах SDK это видно\n"
            "как 'failed to claim usb interface: RS2_USB_STATUS_ACCESS'.\n"
            "\n"
            "Лечится так:\n"
            "  1. Системные настройки → Конфиденциальность и безопасность →\n"
            "     Камера → включить приложение, из которого запускается\n"
            "     python (Терминал, iTerm, VS Code, Claude).\n"
            "  2. Полностью выйти из этого приложения (Cmd+Q) и открыть\n"
            "     заново — разрешение подхватывается только при старте.\n"
            "  3. Если приложения нет в списке — оно там появится после\n"
            "     первой попытки доступа; запустите rs_check.py ещё раз.\n"
            "\n"
            "Проверить, видит ли камеру сама система:\n"
            "  system_profiler SPUSBDataType | grep -i realsense\n"
            "Если и там пусто — тогда уже кабель, переходник или хаб."
        )

    if "Couldn't resolve requests" in msg or "not supported" in msg.lower():
        return head + (
            "\nКамера видна, но такого режима у неё нет. Обычно это значит,\n"
            "что она сидит на USB 2 (хаб, переходник или кабель-зарядка) —\n"
            "тогда часть режимов пропадает. Воткните напрямую кабелем USB 3\n"
            "либо снизьте до RealSenseCamera(424, 240, 15).\n"
            "Посмотреть, что камера вообще умеет:\n"
            "  python -c \"import pyrealsense2 as rs; d=list(rs.context().devices)[0];\n"
            "  [print(p) for s in d.sensors for p in s.profiles]\""
        )

    return head + "\nСмотрите подробности в логах SDK: rs.log_to_console(rs.log_severity.debug)"


def describe_usb(device):
    """
    В каком режиме шины реально работает камера.

    Нужно потому, что D435 на USB 2 стартует молча, без единой ошибки,
    а потом либо теряет кадры, либо не отдаёт нужное разрешение.
    Камера сама знает, как договорилась, и об этом можно спросить —
    гадать по симптомам не надо.

    Возвращает {'name', 'usb', 'is_usb3'}. usb — строка вида '3.2' или '2.1'.
    """
    info = {"name": "unknown", "usb": "?", "is_usb3": True}
    try:
        info["name"] = device.get_info(rs.camera_info.name)
    except Exception:
        pass
    try:
        usb = device.get_info(rs.camera_info.usb_type_descriptor)
        info["usb"] = usb
        info["is_usb3"] = not usb.startswith("2")
    except Exception:
        pass  # старые прошивки этого поля не отдают
    return info


def release_camera_daemon():
    """
    Согнать VDCAssistant с камеры.

    Это системный демон, который захватывает RGB-интерфейс, как только
    камерой интересуется что-нибудь через AVFoundation. Пока он держит
    интерфейс, librealsense получает 'failed to set power state'.
    Убивать безопасно — он поднимается сам при следующем обращении.
    Работает только из-под root, а нам он и так нужен.
    """
    import os
    import subprocess
    if os.geteuid() != 0:
        return
    subprocess.run(["killall", "VDCAssistant"],
                   capture_output=True, check=False)


# Запасной список на случай, если опросить камеру не удалось.
# Порядок: сперва то, что реально пролезает по USB 2, потом жирнее.
# Разрешения взяты общие для стерео-модуля и RGB у D4xx.
FALLBACK_MODES = [
    (424, 240, 15), (424, 240, 30),
    (640, 360, 15), (640, 480, 15),
    (640, 360, 30), (640, 480, 30),
    (848, 480, 15), (424, 240, 6),
]


def common_modes(device):
    """
    Режимы, которые поддерживают ОБА сенсора — глубина и цвет.

    Гадать бессмысленно: набор зависит от модели, прошивки и сильнее
    всего — от того, USB 2 сейчас или USB 3. Список знает сама камера.

    Но спрашивать её опасно: обращение к s.profiles включает питание
    сенсора, а если RGB-интерфейс кем-то занят, прилетает
    'failed to set power state'. Поэтому каждый сенсор опрашиваем
    отдельно и под try — что удалось, то и берём, иначе запасной список.
    """
    depth_modes, color_modes = set(), set()
    for sensor in device.sensors:
        try:
            name = sensor.get_info(rs.camera_info.name).lower()
            profiles = list(sensor.profiles)
        except Exception:
            continue          # сенсор занят — пропускаем, не роняя всё
        for pr in profiles:
            try:
                v = pr.as_video_stream_profile()
            except Exception:
                continue
            st = str(pr.stream_type())
            if "depth" in st:
                depth_modes.add((v.width(), v.height(), pr.fps()))
            elif "color" in st:
                color_modes.add((v.width(), v.height(), pr.fps()))

    # Объединяем, а не выбираем одно из двух: опрос часто выходит
    # неполным (занятый сенсор молчит), и тогда из списка пропадают
    # как раз лёгкие режимы, которые на USB 2 единственные и работают.
    common = (depth_modes & color_modes) | set(FALLBACK_MODES)

    # Для отслеживания руки частота важнее числа пикселей, а на USB 2
    # за всё сразу не заплатить. Целимся в ~640x480 при 15-30 fps.
    def rank(m):
        w, h, f = m
        return (0 if 15 <= f <= 30 else 1, abs(w * h - 640 * 480), -f)

    return sorted(common, key=rank)


MODE_CACHE = "camera_mode.json"


def _load_cached_mode():
    """Режим, который сработал в прошлый раз."""
    import json
    import os
    if os.path.exists(MODE_CACHE):
        try:
            with open(MODE_CACHE, encoding="utf-8") as f:
                m = json.load(f)
            return (int(m["w"]), int(m["h"]), int(m["fps"]))
        except Exception:
            pass
    return None


def _save_cached_mode(w, h, f):
    import json
    try:
        with open(MODE_CACHE, "w", encoding="utf-8") as fh:
            json.dump({"w": w, "h": h, "fps": f}, fh)
    except Exception:
        pass


def open_best_camera(timeout_ms=2500, **kwargs):
    """
    Открыть камеру на первом режиме, который РЕАЛЬНО отдаёт кадры.

    Ключевое слово — реально. pipeline.start() на USB 2 спокойно
    соглашается на режим, которому не хватает полосы, и падает уже
    на wait_for_frames через пять секунд. Поэтому проверка здесь —
    не «запустилось», а «пришёл кадр».
    """
    if rs is None:
        raise ImportError("pyrealsense2 не установлен, см. README_realsense.md")

    release_camera_daemon()

    devs = list(rs.context().query_devices())
    if not devs:
        raise RuntimeError(explain_start_failure(
            RuntimeError("No device connected"), 0, 0, 0))

    modes = common_modes(devs[0])

    # Удачный режим прошлого запуска — вперёд списка. Перебор с проверкой
    # кадром честный, но медленный: каждый негодный режим съедает секунды,
    # и со стороны это выглядит зависшим скриптом.
    cached = _load_cached_mode()
    if cached and cached in modes:
        modes = [cached] + [m for m in modes if m != cached]
    elif cached:
        modes = [cached] + modes

    print(f"[realsense] подбираю режим, вариантов: {len(modes)}")
    if not modes:
        raise RuntimeError("у камеры нет ни одного режима, общего для "
                           "глубины и цвета — это очень странно")

    errors = []
    for n, (w, h, f) in enumerate(modes, 1):
        print(f"[realsense]   {n}/{len(modes)}: пробую {w}x{h}@{f} ...",
              flush=True)
        try:
            cam = RealSenseCamera(w, h, f, **kwargs)
        except Exception as e:
            errors.append(f"  {w}x{h}@{f}: не стартовал — {e}")
            continue
        try:
            cam.pipeline.wait_for_frames(timeout_ms)   # вот настоящая проверка
            print(f"[realsense] рабочий режим: {w}x{h} @ {f}")
            _save_cached_mode(w, h, f)
            return cam
        except Exception:
            errors.append(f"  {w}x{h}@{f}: стартовал, но кадры не идут")
            try:
                cam.stop()
            except Exception:
                pass

    stuck = sum("power state" in e or "No device" in e for e in errors)
    if stuck and stuck == len(errors):
        raise RuntimeError(
            "камера не отвечает ни на одном режиме — она подвисла.\n"
            "\n"
            "Так бывает после того, как libusb несколько раз отбирал\n"
            "устройство у системного драйвера и переподключал обратно.\n"
            "Питание при этом не сбрасывается, и камера остаётся в\n"
            "промежуточном состоянии: на шине висит, а отвечать не хочет.\n"
            "\n"
            "Лечится только физически: выньте кабель камеры, подождите\n"
            "пару секунд и воткните обратно. Затем запустите снова.\n"
            "\n"
            "Признак того же самого со стороны системы — в выводе\n"
            "system_profiler SPUSBDataType пропадает строка Speed\n"
            "и появляется IOCreatePlugInInterfaceForService failed."
        )

    raise RuntimeError(
        "ни один режим не отдал кадры. Проверено:\n" + "\n".join(errors) +
        "\n\nПочти наверняка не хватает полосы USB. Камера сейчас на USB 2 —\n"
        "нужен кабель и порт USB 3 (проверить: system_profiler SPUSBDataType\n"
        "| grep -A6 -i realsense, строка Speed должна быть 5 Gb/s и выше)."
    )


class RealSenseCamera:
    """
    Обёртка над pyrealsense2: старт потоков, выравнивание depth->color,
    встроенные фильтры SDK.

    Выравнивание обязательно: физически ИК-камера и цветная разнесены
    на пару сантиметров, поэтому пиксель (x, y) цветного кадра и
    пиксель (x, y) карты глубины — это РАЗНЫЕ точки пространства.
    MediaPipe работает по цветному кадру, значит глубину надо привести
    к его системе координат.
    """

    def __init__(self, width=640, height=480, fps=30, enable_sdk_filters=True,
                 visual_preset=None):
        if rs is None:
            raise ImportError(
                "pyrealsense2 не установлен. Смотри инструкцию по сборке "
                "librealsense для macOS в README_realsense.md"
            ) from _IMPORT_ERROR

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        try:
            self.profile = self.pipeline.start(cfg)
        except RuntimeError as e:
            raise RuntimeError(explain_start_failure(e, width, height, fps)) from e

        self.usb = describe_usb(self.profile.get_device())
        print(f"[realsense] {self.usb['name']}  шина USB {self.usb['usb']}")
        if not self.usb["is_usb3"]:
            print(
                "[realsense] ВНИМАНИЕ: камера на USB 2.\n"
                "            Причина почти всегда в кабеле, переходнике или\n"
                "            хабе, а не в самой камере. Будут пропуски кадров\n"
                "            и урезанный список режимов. Воткните напрямую\n"
                "            кабелем USB 3, либо снизьте до "
                "RealSenseCamera(424, 240, 15)."
            )

        # depth_scale — сколько метров в одной «единице» из z16.
        # У D435 это 0.001 (миллиметры), но хардкодить нельзя: у разных
        # моделей и прошивок значение разное.
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # ВНИМАНИЕ: здесь НЕ ставим visual_preset (High Accuracy и прочие).
        #
        # На macOS этот вызов роняет процесс segfault'ом, и вот почему.
        # Под root libusb ОТБИРАЕТ устройство у системного UVC-драйвера,
        # а при освобождении интерфейса переподключает его обратно
        # (darwin_reenumerate_device). Смена пресета внутри себя дёргает
        # питание сенсора -> release_power -> начинается переподключение,
        # и параллельный поток в этот момент шлёт control transfer на уже
        # освобождённый хендл. Итог — обращение по нулевому адресу.
        # На Linux этой ветки нет вовсе, потому там пресет безопасен.
        #
        # Потеря невелика: High Accuracy давал меньше мусорных точек ценой
        # дырок, а мусор у нас и так отсеивается медианой по окну и MAD
        # в patch_depth(). Ради него ронять процесс смысла нет.
        if visual_preset and rs is not None:
            depth_sensor.set_option(rs.option.visual_preset, visual_preset)

        self.align = rs.align(rs.stream.color)

        self.enable_sdk_filters = enable_sdk_filters
        # Фильтры SDK. temporal — это тоже IIR, только по каждому пикселю
        # отдельно, ещё ДО того как мы что-то посчитали.
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 2)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 20)
        self.temporal = rs.temporal_filter()
        self.temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        self.temporal.set_option(rs.option.filter_smooth_delta, 20)
        self.hole = rs.hole_filling_filter(1)  # 1 = взять ближайшего соседа

        self.intrinsics = None

    def read(self):
        """
        Возвращает (color_bgr: np.ndarray, depth_m: np.ndarray в МЕТРАХ)
        либо (None, None), если кадр не пришёл.

        depth_m — float32, где 0.0 означает «нет данных».
        """
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None

        if self.enable_sdk_filters:
            depth_frame = self.spatial.process(depth_frame)
            depth_frame = self.temporal.process(depth_frame)
            depth_frame = self.hole.process(depth_frame)

        # Интринсики берём у ЦВЕТНОГО потока — глубина к нему выровнена.
        if self.intrinsics is None:
            self.intrinsics = (
                color_frame.profile.as_video_stream_profile().intrinsics
            )

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth *= self.depth_scale

        return color, depth

    def deproject(self, u, v, z):
        """Пиксель (u, v) + глубина z (м) -> точка (X, Y, Z) в метрах.

        Система координат RealSense: X вправо, Y ВНИЗ, Z от камеры вперёд.
        """
        p = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [float(u), float(v)], float(z)
        )
        return np.array(p, dtype=np.float32)

    def stop(self):
        self.pipeline.stop()


# ----------------------------------------------------------------------
#  Устойчивый замер глубины
# ----------------------------------------------------------------------

def patch_depth(depth_m, u, v, half=4, min_valid=6, mad_k=3.0):
    """
    Глубина в точке (u, v), посчитанная по окну (2*half+1)^2 пикселей.

    Возвращает float (метры) или None, если данных не хватило.
    None — это ВАЖНО: «не знаю» должно распространяться дальше как
    «не знаю», а не превращаться в ноль. Ноль — это команда рукой в стену.

    half=4 -> окно 9x9. На 640x480 и дистанции 0.5 м это примерно
    сантиметр ладони: достаточно много, чтобы усреднить шум, и
    достаточно мало, чтобы не залезть на фон.
    """
    h, w = depth_m.shape
    u = int(round(u))
    v = int(round(v))
    if not (0 <= u < w and 0 <= v < h):
        return None

    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    patch = depth_m[v0:v1, u0:u1].ravel()

    # 1) выкидываем дырки и заведомую чушь
    valid = patch[(patch > 0.1) & (patch < 4.0)]
    if valid.size < min_valid:
        return None

    # 2) режем выбросы по MAD (median absolute deviation).
    #    MAD устойчив к выбросам, в отличие от обычного СКО, которое
    #    сами выбросы и раздувают. Так отсекается фон, если край окна
    #    свесился с ладони на стену.
    med = float(np.median(valid))
    mad = float(np.median(np.abs(valid - med)))
    if mad > 1e-6:
        keep = valid[np.abs(valid - med) <= mad_k * 1.4826 * mad]
        if keep.size >= min_valid:
            valid = keep

    return float(np.median(valid))


def hand_point_3d(cam, depth_m, landmarks, img_w, img_h,
                  indices=PALM_LANDMARKS, half=4, min_points=3):
    """
    Главная функция: 3D-точка ладони в метрах, устойчивая к глюкам глубины.

    Алгоритм:
      1. для каждой точки ладони меряем глубину окном (patch_depth);
      2. отбрасываем те, где не получилось;
      3. режем по MAD уже между точками — если один landmark «улетел»
         за руку, он вылетит здесь;
      4. каждую выжившую точку разворачиваем в 3D (deproject);
      5. берём медиану по X, Y, Z отдельно.

    Возвращает np.array([X, Y, Z]) в метрах или None.
    Именно это «среднее по всем точкам», о котором речь, — только
    медиана вместо среднего и с отсевом на каждом шаге.
    """
    pts_px = []
    depths = []

    for i in indices:
        lm = landmarks[i]
        u = lm.x * img_w
        v = lm.y * img_h
        z = patch_depth(depth_m, u, v, half=half)
        if z is not None:
            pts_px.append((u, v))
            depths.append(z)

    if len(depths) < min_points:
        return None

    depths = np.array(depths, dtype=np.float32)

    # Отсев «улетевших» landmark'ов. Ладонь плоская: разброс глубины
    # по ней — сантиметры. Всё, что дальше — артефакт.
    med = float(np.median(depths))
    mad = float(np.median(np.abs(depths - med)))
    tol = max(0.03, 3.0 * 1.4826 * mad)   # но не строже 3 см
    keep = np.abs(depths - med) <= tol
    if keep.sum() < min_points:
        keep = np.ones_like(depths, dtype=bool)

    xyz = [cam.deproject(u, v, z)
           for (u, v), z, k in zip(pts_px, depths, keep) if k]

    return np.median(np.stack(xyz, axis=0), axis=0)


def grip_distance_3d(cam, depth_m, landmarks, img_w, img_h):
    """
    Раскрытие захвата — расстояние большой палец / указательный,
    но в МЕТРАХ, а не в долях кадра.

    В исходном коде считалось sqrt(dx^2 + dy^2) по нормализованным
    координатам, из-за чего одно и то же положение пальцев давало
    разное значение в зависимости от того, далеко рука или близко:
    отошёл на полметра — захват сам собой закрылся. С реальной
    глубиной этой зависимости нет.

    Тонкость: кончики пальцев (4 и 8) — плохие точки для глубины
    (см. коммент про летающие пиксели). Поэтому глубину берём общую,
    от ладони, а сами кончики только разворачиваем в 3D по ней.
    Пальцы и ладонь лежат примерно в одной плоскости, ошибка мала.
    """
    palm_z = None
    for i in PALM_LANDMARKS:
        z = patch_depth(depth_m, landmarks[i].x * img_w,
                        landmarks[i].y * img_h, half=4)
        if z is not None:
            palm_z = z if palm_z is None else min(palm_z, z)
    if palm_z is None:
        return None

    a = cam.deproject(landmarks[4].x * img_w, landmarks[4].y * img_h, palm_z)
    b = cam.deproject(landmarks[8].x * img_w, landmarks[8].y * img_h, palm_z)
    return float(np.linalg.norm(a - b))
