"""
Управление манипулятором SO-ARM100 по 3D-положению руки (Intel RealSense).

Отличие от cv_control.py: там координаты картинки мапились прямо на тики
серв («сустав 1 = x руки»). Здесь честная цепочка:

    рука в кадре -> 3D-точка в МЕТРАХ -> фильтр -> точка в системе робота
                 -> обратная кинематика -> углы -> тики

Что это даёт:
  * рука робота идёт ТУДА, куда показывает ваша рука, а не «примерно туда»;
  * захват реагирует на реальное расстояние между пальцами, а не на
    его проекцию (раньше стоило отойти от камеры — и захват сам закрывался);
  * пределы рабочей зоны считаются физически, а не подбором min/max.

Клавиши:
    ПРОБЕЛ — включить/выключить передачу команд в сервы
    h      — вернуться в нулевую позу
    r      — снять момент (рука обмякает)
    d      — показать/скрыть карту глубины
    ESC    — выход
"""

import math
import os
import time

import cv2
import numpy as np

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from manip.depth import open_best_camera, hand_point_3d, grip_distance_3d, require_root
from manip.filters import VectorFilter, OneEuro, MedianSpikeFilter
from manip.kinematics import ArmGeometry, inverse, clamp_to_workspace, forward
from manip.arm import Arm

MODEL_PATH = "hand_landmarker.task"

# ----------------------------------------------------------------------
#  Рабочие объёмы
# ----------------------------------------------------------------------
# Вместо честной hand-eye калибровки (нужна доска и полчаса времени)
# просто линейно натягиваем коробку в пространстве перед камерой на
# коробку в рабочей зоне робота. Для управления жестами этого хватает:
# человек за секунду подстраивается под масштаб.
#
# Камера RealSense: X вправо, Y ВНИЗ, Z вперёд от объектива. Метры.
# ВАЖНО: значения подобраны под D455. У неё минимальная дистанция ~0.5 м —
# заметно больше, чем у D435 (~0.2 м), потому что база между ИК-камерами
# шире (95 мм против 50 мм). Широкая база — это плюс на расстоянии
# (точность падает как квадрат дистанции, и у D455 на 4 м ошибка вдвое
# меньше) и минус вблизи: ближе ~0.5 м зоны обзора двух ИК-камер почти
# не пересекаются, и сопоставлять становится нечего — глубина сыплется
# дырками. Поэтому рабочая зона по Z отодвинута.
# Если подключишь D435 — верни (0.35, 0.70) и сядь ближе.
# Куда едет робот, когда рука идёт вправо по экрану.
# Кадр уже отзеркален (см. cv2.flip после cam.read()), поэтому
# "вправо на экране" = "вправо у вас". Если направление всё равно
# ощущается наоборот — поменяйте здесь знак, и развернутся сразу
# оба режима, и прямой, и координатный.
X_SIGN = +1

CAM_BOX = {
    "x": (-0.30, 0.30),      # влево / вправо (у D455 обзор шире: 87° против 69°)
    "y": (-0.24, 0.24),      # вверх / вниз (Y растёт ВНИЗ!)
    "z": (0.60, 1.05),       # ближе / дальше. Ближе 0.5 м D455 слепнет
}

# Робот: X вперёд, Y влево, Z вверх. Метры от оси основания.
ROBOT_BOX = {
    "x": (0.12, 0.28),
    "y": (-0.14, 0.14),
    "z": (0.02, 0.26),
}

GRIP_RANGE = (0.02, 0.11)    # расстояние палец-палец в метрах -> захват


def lerp_box(value, src, dst):
    t = (value - src[0]) / (src[1] - src[0])
    t = min(max(t, 0.0), 1.0)
    return dst[0] + t * (dst[1] - dst[0])


def cam_to_robot(p_cam):
    """
    3D-точка в системе камеры -> цель в системе робота.

    Оси переставлены так, чтобы движения были интуитивными при камере,
    стоящей перед оператором:
      рука вправо  -> робот вправо   (cam +X -> robot -Y)
      рука вверх   -> робот вверх    (cam -Y -> robot +Z)
      рука от себя -> робот вперёд   (cam +Z -> robot +X)
    """
    x = lerp_box(p_cam[2], CAM_BOX["z"], ROBOT_BOX["x"])
    y = lerp_box(X_SIGN * p_cam[0],
                 (CAM_BOX["x"][0], CAM_BOX["x"][1]), ROBOT_BOX["y"])
    z = lerp_box(-p_cam[1], (-CAM_BOX["y"][1], -CAM_BOX["y"][0]), ROBOT_BOX["z"])
    return np.array([x, y, z])



def direct_joints(p_cam, pitch, geo):
    """
    ПРЯМОЙ режим: положение ладони -> углы суставов, один к одному.

    Зачем он рядом с обратной кинематикой. ОК управляет ТОЧКОЙ схвата:
    вы задаёте, где должен оказаться кончик, а решатель сам выбирает
    комбинацию углов. Это правильно, когда надо попасть в предмет, но
    непредсказуемо на ощупь: одна и та же поза руки может дать разные
    конфигурации, и рука «живёт своей жизнью».

    Прямой режим ведёт себя как в старом cv_control.py: каждая ось
    ладони жёстко привязана к своему суставу. Подняли руку — плечо
    поднялось, и никак иначе. Точности по координатам нет, зато
    полная предсказуемость.

    Возвращает [q1, q2, q3, q4] в радианах.
    """
    x, y, z = p_cam                      # метры в системе камеры

    def span(v, lo, hi, a, b):
        t = (v - lo) / (hi - lo)
        return a + (b - a) * float(np.clip(t, 0.0, 1.0))

    # влево-вправо -> поворот базы. Кадр уже отзеркален, так что
    # дополнительно переворачивать ось не надо — за это отвечает X_SIGN.
    q1 = span(X_SIGN * x, CAM_BOX["x"][0], CAM_BOX["x"][1], -1.2, 1.2)

    # вверх-вниз -> плечо. Y в кадре растёт ВНИЗ, отсюда минус.
    q2 = span(-y, -CAM_BOX["y"][1], -CAM_BOX["y"][0], -0.5, 1.2)

    # ближе-дальше -> локоть. Тянете руку к себе — локоть складывается.
    q3 = span(z, CAM_BOX["z"][0], CAM_BOX["z"][1], -1.8, 0.2)

    # наклон ладони -> кисть, напрямую
    q4 = float(np.clip(pitch if pitch is not None else -0.5, -1.4, 1.0))

    return [q1, q2, q3, q4]


def hand_pitch(cam, depth_m, lm, w, h):
    """
    Наклон ладони в 3D — им задаём наклон схвата.

    Считаем по вектору «запястье (0) -> основание среднего пальца (9)».
    Оба — крупные точки на ладони, глубина по ним меряется устойчиво,
    в отличие от кончиков пальцев.
    """
    from manip.depth import patch_depth

    z0 = patch_depth(depth_m, lm[0].x * w, lm[0].y * h)
    z9 = patch_depth(depth_m, lm[9].x * w, lm[9].y * h)
    if z0 is None or z9 is None:
        return None

    a = cam.deproject(lm[0].x * w, lm[0].y * h, z0)
    b = cam.deproject(lm[9].x * w, lm[9].y * h, z9)
    v = b - a
    n = np.linalg.norm(v)
    if n < 1e-4:
        return None

    # -v[1] потому что Y камеры смотрит вниз; горизонталь берём по XZ
    return math.atan2(-v[1], math.hypot(v[0], v[2]))


def draw_hud(frame, lines, color=(0, 255, 0)):
    for i, t in enumerate(lines):
        cv2.putText(frame, t, (10, 26 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, t, (10, 26 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 1, cv2.LINE_AA)


def main():
    require_root("rs_control.py")
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"{MODEL_PATH} не найден")

    options = vision.HandLandmarkerOptions(
        # delegate=CPU — осознанно. Процесс работает под sudo (иначе
        # macOS не отдаёт камеру), а рут не привязан к графической
        # сессии, и обращаться к GPU ему незачем. Ладонь на CPU
        # считается быстро, разницы в отзывчивости нет.
        base_options=python.BaseOptions(
            model_asset_path=MODEL_PATH,
            delegate=python.BaseOptions.Delegate.CPU,
        ),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.6,
        min_tracking_confidence=0.6,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)

    cam = open_best_camera()
    geo = ArmGeometry()          # !!! подставьте свои замеры
    arm = Arm(dry_run=False)

    # Фильтруем ЦЕЛЬ (метры), а не углы.
    # Почему так: вблизи вытянутого положения ОК нелинейна — миллиметр
    # цели превращается в десяток градусов сустава. Сгладив углы, вы
    # сгладите уже испорченный сигнал. Сглаживать надо до нелинейности.
    pos_filter = VectorFilter(3, median_size=5, use_one_euro=True,
                              min_cutoff=0.7, beta=0.6)
    direct_mode = True     # начинаем с понятного режима; m — переключить
    grip_med = MedianSpikeFilter(5)
    grip_flt = OneEuro(min_cutoff=1.0, beta=0.3)
    pitch_med = MedianSpikeFilter(5)
    pitch_flt = OneEuro(min_cutoff=0.6, beta=0.2)

    enabled = False
    show_depth = False
    last_t = time.time()
    fps = 0.0
    lost_frames = 0

    cv2.namedWindow("RealSense arm control", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("RealSense arm control", 1280, 720)

    try:
        while True:
            color, depth = cam.read()

            # ЗЕРКАЛО. Отражаем ОБА кадра сразу после чтения, до всего
            # остального — тогда MediaPipe, замер глубины по пикселям и
            # отрисовка живут в одной, уже отзеркаленной системе, и
            # рассинхрона между картинкой и координатами не возникает.
            # Отражать только для показа нельзя: точки поедут относительно
            # глубины. Смысл — как в зеркале: двинули руку вправо, и на
            # экране (и у робота) ушло вправо.
            if color is not None:
                color = cv2.flip(color, 1)
                depth = cv2.flip(depth, 1)
            if color is None:
                continue

            now = time.time()
            dt = max(now - last_t, 1e-3)
            last_t = now
            fps = 0.9 * fps + 0.1 / dt

            frame = color.copy()
            rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]

            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            # Метка времени в МИЛЛИСЕКУНДАХ реального времени.
            # В исходнике был счётчик +1 на кадр — трекер при этом думает,
            # что между кадрами прошла миллисекунда, и его внутреннее
            # сглаживание работает не так, как задумано.
            result = landmarker.detect_for_video(mp_image, int(now * 1000))

            target_cam = None
            grip_m = None
            pitch = None

            if result.hand_landmarks:
                lm = result.hand_landmarks[0]

                for p in lm:
                    cv2.circle(frame, (int(p.x * w), int(p.y * h)), 3,
                               (0, 255, 0), -1)

                target_cam = hand_point_3d(cam, depth, lm, w, h)
                grip_m = grip_distance_3d(cam, depth, lm, w, h)
                pitch = hand_pitch(cam, depth, lm, w, h)

            if target_cam is None:
                # Рука пропала или глубина невалидна — ДЕРЖИМ последнюю
                # позу, ничего не двигаем. Ноль в такой ситуации отправил
                # бы руку в упор.
                lost_frames += 1
                if lost_frames > 30:      # ~1 с — считаем, что оператор ушёл
                    pos_filter.reset()
                    grip_flt.reset()
                    pitch_flt.reset()
                draw_hud(frame, [f"FPS {fps:4.1f}", "рука не найдена"],
                         (0, 165, 255))
            else:
                lost_frames = 0

                p = pos_filter.update(list(target_cam), dt)
                target = cam_to_robot(p)
                target, in_ws = clamp_to_workspace(geo, target)

                if pitch is not None:
                    pitch = pitch_flt.update(pitch_med.update(pitch), dt)
                    # ограничим наклон разумным диапазоном:
                    # от «горизонтально вперёд» до «вертикально вниз»
                    pitch = float(np.clip(pitch, -math.pi / 2, 0.2))
                else:
                    pitch = -math.pi / 2

                if direct_mode:
                    q = direct_joints(p, pitch, geo)
                    ok, in_ws = True, True   # в прямом режиме зоны нет
                else:
                    q, ok = inverse(geo, target, pitch=pitch)

                if grip_m is not None:
                    g = grip_flt.update(grip_med.update(grip_m), dt)
                    t = (g - GRIP_RANGE[0]) / (GRIP_RANGE[1] - GRIP_RANGE[0])
                    # 0 = пальцы вместе (схват закрыт), 1 = разведены
                    grip_rad = float(np.clip(t, 0, 1))
                else:
                    grip_rad = None

                if enabled:
                    arm.move_rad(q, gripper_rad=grip_rad)

                # Проверка себя: куда по нашей же модели уехал схват
                tcp = forward(geo, q)
                err_mm = np.linalg.norm(tcp - target) * 1000

                draw_hud(frame, [
                    f"FPS {fps:4.1f}   {'АКТИВНО' if enabled else 'пауза (пробел)'}"
                    f"   {'ПРЯМОЙ' if direct_mode else 'КООРДИНАТЫ'} (m)",
                    f"камера XYZ  {target_cam[0]:+.3f} {target_cam[1]:+.3f} {target_cam[2]:.3f} м",
                    f"цель робота {target[0]:+.3f} {target[1]:+.3f} {target[2]:+.3f} м"
                    + ("" if in_ws and ok else "  [ПРЕДЕЛ]"),
                    f"углы  {np.degrees(q).round(1)}  pitch {math.degrees(pitch):+.0f}°",
                    f"захват {grip_m*1000:.0f} мм" if grip_m else "захват —",
                    f"невязка ОК {err_mm:.1f} мм",
                ], (0, 255, 0) if (enabled and in_ws and ok) else (0, 200, 255))

            if show_depth:
                d8 = cv2.convertScaleAbs(depth, alpha=255.0 / 2.0)
                dmap = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
                dmap[depth <= 0] = (0, 0, 0)     # дырки — чёрным, чтобы их видеть
                frame = np.hstack([frame, dmap])

            cv2.imshow("RealSense arm control", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord(" "):
                enabled = not enabled
                pos_filter.reset()
            elif key == ord("h"):
                enabled = False
                arm.go_home()
            elif key == ord("r"):
                enabled = False
                arm.relax()
            elif key == ord("m"):
                direct_mode = not direct_mode
                print("режим:", "ПРЯМОЙ (рука -> суставы)" if direct_mode
                      else "КООРДИНАТЫ (обратная кинематика)")

            elif key == ord("d"):
                show_depth = not show_depth

    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
