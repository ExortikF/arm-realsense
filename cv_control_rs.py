"""
Управление манипулятором рукой. Глубина берётся с RealSense.

Первая рука в кадре: база, плечо, локоть, хват.
Вторая: поворот клешни и кисть.

Сервы слушаются только когда в кадре ОБЕ руки — это защита от того,
чтобы рука не поехала от случайного человека или жеста в кадре.

Клавиши: пробел - пауза, d - показать координаты точек, Esc - выход.

Запуск:  sudo launchctl asuser $(id -u) venv/bin/python cv_control_rs.py
"""

import os
import time

import cv2
import numpy as np

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from manip.depth import open_best_camera, patch_depth, require_root
from python_st3215.st3215 import ST3215


# Пределы серв в тиках. mid — куда встаём, пока руки нет в кадре.
servo_config = {
    1: {"min": 1044, "max": 3038, "mid": (1044 + 3038) // 2},   # база
    2: {"min":  958, "max": 3142, "mid": (958 + 3142) // 2},    # плечо
    3: {"min":  981, "max": 3488, "mid": (981 + 3488) // 2},    # локоть
    4: {"min": 1525, "max": 2962, "mid": (1525 + 2962) // 2},   # кисть
    5: {"min":    0, "max": 4095, "mid": 2047},                 # поворот
    6: {"min": 1850, "max": 3165, "mid": (1850 + 3165) // 2},   # хват
}

SERIAL_PORT = "/dev/tty.usbmodem5AE60853301"
MODEL_PATH = "hand_landmarker.task"

# Рабочая дистанция руки от камеры, метры. Ближе 0.5 м D455 не видит.
DEPTH_NEAR = 0.55
DEPTH_FAR = 1.00

require_root("cv_control_rs.py")

try:
    servo = ST3215(SERIAL_PORT)
    ids = servo.ListServos()
    print(f"Connected servos: {ids}")
except Exception as e:
    print("Servo connection failed:", e)
    servo = None


def send_servo(angles):
    """Разослать позиции по сервам, обрезав по пределам."""
    if servo:
        for i, angle in enumerate(angles, start=1):
            angle = int(np.clip(angle, servo_config[i]["min"], servo_config[i]["max"]))
            servo.MoveTo(i, angle)


if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError("hand_landmarker.task not found")

BaseOptions = python.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH,
                             delegate=BaseOptions.Delegate.CPU),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=2,
)

print("загружаю модель ...")
landmarker = HandLandmarker.create_from_options(options)
print("модель готова, открываю камеру ...")


def map_value(v, in_min, in_max, out_min, out_max):
    """Линейно перегнать значение из одного диапазона в другой."""
    return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def draw_landmarks(frame, hand_landmarks):
    """Точки руки поверх кадра."""
    h, w, _ = frame.shape
    for lm in hand_landmarks:
        x = int(lm.x * w)
        y = int(lm.y * h)
        cv2.circle(frame, (x, y), 4, (0, 255, 0), -1)


def draw_palm_coords(frame, hand, depth, w, h, label):
    """Одна подпись на руку — в центре ладони, чтобы не загораживать кадр."""
    ids = (0, 5, 9, 13, 17)
    x = int(sum(hand[i].x for i in ids) / len(ids) * w)
    y = int(sum(hand[i].y for i in ids) / len(ids) * h)

    z = palm_depth(depth, hand, w, h)
    txt = f"{label}  x{x} y{y}" + (f" z{z*100:.0f}cm" if z else "")

    cv2.circle(frame, (x, y), 7, (255, 200, 0), 2)
    cv2.putText(frame, txt, (x + 12, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 0, 0), 4)
    cv2.putText(frame, txt, (x + 12, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 200, 0), 1)


def draw_pinch(frame, hand, depth, w, h):
    """Линия между большим и указательным пальцем с расстоянием в мм."""
    x1, y1 = int(hand[4].x * w), int(hand[4].y * h)
    x2, y2 = int(hand[8].x * w), int(hand[8].y * h)
    cv2.line(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)

    # Глубину берём по ладони: на кончиках пальцев она врёт.
    z = palm_depth(depth, hand, w, h)
    if z is None:
        return

    a = cam.deproject(x1, y1, z)
    b = cam.deproject(x2, y2, z)
    mm = float(np.linalg.norm(np.array(a) - np.array(b))) * 1000

    mx, my = (x1 + x2) // 2, (y1 + y2) // 2
    txt = f"{mm:.0f} mm"
    cv2.putText(frame, txt, (mx + 8, my), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 4)
    cv2.putText(frame, txt, (mx + 8, my), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 200, 255), 2)


def smooth_angles(prev, curr, alpha=0.15):
    """IIR-сглаживание: тянем старое значение к новому на alpha."""
    return [prev[i] * (1 - alpha) + curr[i] * alpha for i in range(len(prev))]


def palm_depth(depth, hand, w, h):
    """Расстояние до ладони в метрах. Меряем по ладони, не по пальцам."""
    zs = []
    for i in (0, 1, 5, 9, 13, 17):
        z = patch_depth(depth, int(hand[i].x * w), int(hand[i].y * h), half=4)
        if z:
            zs.append(z)
    return float(np.median(zs)) if zs else None


def calculate_grip(hand):
    """Хват: чем ближе большой палец к указательному, тем сильнее сжат."""
    index_tip = hand[8]
    thumb_tip = hand[4]

    dist = np.sqrt(
        (index_tip.x - thumb_tip.x) ** 2 +
        (index_tip.y - thumb_tip.y) ** 2
    )

    return map_value(dist, 0.02, 0.25,
                     servo_config[6]["min"], servo_config[6]["max"])


def calculate_rotation(hand):
    """Поворот клешни — тем же жестом, но у второй руки."""
    index_tip = hand[8]
    thumb_tip = hand[4]

    dist = np.sqrt(
        (index_tip.x - thumb_tip.x) ** 2 +
        (index_tip.y - thumb_tip.y) ** 2
    )

    return map_value(dist, 0.02, 0.25,
                     servo_config[5]["min"], servo_config[5]["max"])


def calculate_wrist_pitch(hand):
    """Кисть вверх-вниз по высоте запястья второй руки."""
    wrist = hand[0]
    return map_value(wrist.y, 0.2, 0.8,
                     servo_config[4]["min"], servo_config[4]["max"])


cam = open_best_camera()
print("камера открыта, окно должно появиться")

timestamp = 0
paused = False        # пробел
show_coords = False   # d

# Стартуем из середины диапазонов и от них же сглаживаем.
prev_angles = [servo_config[i]["mid"] for i in range(1, 7)]

cv2.namedWindow("Manipulator Control", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Manipulator Control", 1280, 720)

while True:
    color, depth = cam.read()
    if color is None:
        continue

    # Зеркалим оба кадра сразу, иначе точки разъедутся с глубиной.
    frame = cv2.flip(color, 1)
    depth = cv2.flip(depth, 1)
    h, w = frame.shape[:2]

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    result = landmarker.detect_for_video(mp_image, timestamp)
    timestamp += 33

    angles = prev_angles.copy()
    info = "рук нет"

    if len(result.hand_landmarks) > 0:

        for idx, hand_landmarks in enumerate(result.hand_landmarks):

            draw_landmarks(frame, hand_landmarks)

            if idx == 0:
                # Первая рука ведёт базу, плечо, локоть и хват.

                angles[0] = map_value(hand_landmarks[0].x, 0.2, 0.8,
                                      servo_config[1]["min"],
                                      servo_config[1]["max"])

                angles[1] = map_value(hand_landmarks[0].y, 0.2, 0.8,
                                      servo_config[2]["min"],
                                      servo_config[2]["max"])

                # Локоть — от реальной глубины, а не от landmark.z.
                z = palm_depth(depth, hand_landmarks, w, h)
                if z is not None:
                    angles[2] = map_value(z, DEPTH_NEAR, DEPTH_FAR,
                                          servo_config[3]["min"],
                                          servo_config[3]["max"])
                    info = f"глубина {z * 100:.1f} см"
                else:
                    info = "глубина - (подойдите на 55-100 см)"

                angles[5] = calculate_grip(hand_landmarks)

            elif idx == 1:
                # Вторая рука — только поворот клешни и кисть.
                angles[4] = calculate_rotation(hand_landmarks)
                angles[3] = calculate_wrist_pitch(hand_landmarks)

    hands = result.hand_landmarks
    two_hands = len(hands) >= 2

    # По клавише d: расстояние между пальцами у каждой руки и одна
    # подпись с координатами в центре ладони.
    if show_coords:
        for i, hand in enumerate(hands):
            draw_pinch(frame, hand, depth, w, h)
            draw_palm_coords(frame, hand, depth, w, h, f"рука {i + 1}")

    angles = smooth_angles(prev_angles, angles)
    prev_angles = angles.copy()

    # Главная защита: без двух рук в кадре сервы не трогаем вообще.
    # Углы при этом продолжают считаться, поэтому при возврате рук
    # манипулятор не дёрнется из старой позиции.
    if two_hands and not paused:
        send_servo(angles)
        state = "РАБОТА"
    elif paused:
        state = "ПАУЗА (пробел)"
    else:
        state = "ЖДУ ДВЕ РУКИ"

    txt = (f"{state}   {info}   рук: {len(hands)}"
           f"   d - координаты, пробел - пауза, Esc - выход")
    cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 0), 4)
    cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 1)

    cv2.imshow("Manipulator Control", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    elif key == ord(" "):
        paused = not paused
        print("пауза" if paused else "работа")
    elif key == ord("d"):
        show_coords = not show_coords

cam.stop()
cv2.destroyAllWindows()
