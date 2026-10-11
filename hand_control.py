"""
Прямое управление манипулятором рукой — как в cv_control.py, но с
настоящей глубиной от RealSense.

ЧЕМ ОТЛИЧАЕТСЯ ОТ rs_control.py
--------------------------------
Здесь НЕТ обратной кинематики. Каждая ось руки жёстко привязана к
своей серве, счёт идёт сразу в тиках. Что видите — то и получаете,
никакой решатель за спиной ничего не переигрывает.

ЧЕМ ОТЛИЧАЕТСЯ ОТ cv_control.py
--------------------------------
1. Глубина берётся с камеры, а не из landmark.z. У MediaPipe z — это
   грубая оценка «на глаз» в условных единицах относительно запястья;
   она пляшет и зависит от того, как повёрнута ладонь. Здесь глубина
   меряется по-настоящему, в метрах, медианой по площадке ладони.
2. Хватает ОДНОЙ руки. В старом файле стояло `if len(...) > 1`, то есть
   пока в кадре не появятся две руки, не работало вообще ничего.
3. Хват считается по РАЗМЕРУ ЛАДОНИ, а не по абсолютному расстоянию
   между пальцами — иначе отодвинулись от камеры, и схват сам поехал.
4. Фильтры: медиана против выбросов + IIR против дрожи.

УПРАВЛЕНИЕ (одна рука, любая)
------------------------------
    рука влево-вправо   -> база          (серва 1)
    рука вверх-вниз     -> плечо         (серва 2)
    рука ближе-дальше   -> локоть        (серва 3)
    пальцы вместе/врозь -> схват         (серва 6)

Вторая рука, если попадёт в кадр:
    вверх-вниз          -> кисть         (серва 4)
    пальцы вместе/врозь -> поворот кисти (серва 5)

КЛАВИШИ
-------
    пробел  пауза / работа (сервы слушаются только в режиме РАБОТА)
    c       запомнить ТЕКУЩИЙ жест как «схват закрыт»
    o       запомнить ТЕКУЩИЙ жест как «схват открыт»
    z       увести все сервы в середину диапазона
    i       инвертировать ось, которая крутится не туда (по цифре 1..6)
    Esc     выход

Если какая-то ось едет наоборот — не надо лезть в формулы: нажмите i,
затем цифру сустава. Знак запомнится в axis_config.json.
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from manip.arm import JOINTS, TICKS_PER_RAD, rad_to_ticks
from manip.depth import open_best_camera, patch_depth, require_root
from python_st3215.st3215 import ST3215

# =====================================================================
# НАСТРОЙКИ
# =====================================================================
SERIAL_PORT = "/dev/tty.usbmodem5AE60853301"
MODEL_PATH = "hand_landmarker.task"
AXIS_FILE = "axis_config.json"

# Пределы серв — из вашего cv_control.py, они уже проверены железом.
SERVOS = {
    1: {"name": "база",     "min": 1044, "max": 3038},
    2: {"name": "плечо",    "min":  958, "max": 3142},
    3: {"name": "локоть",   "min":  981, "max": 3488},
    4: {"name": "кисть",    "min": 1525, "max": 2962},
    5: {"name": "поворот",  "min":    0, "max": 4095},
    # min здесь — «полностью закрыто». Значение 2026 досталось от
    # старого конфига и до упора губки не доводило. Реальный упор
    # ищется клавишами [ и ] прямо на ходу, результат сохраняется.
    6: {"name": "схват",    "min": 1850, "max": 3165},
}
for s in SERVOS.values():
    s["mid"] = (s["min"] + s["max"]) // 2

# Рабочая зона в метрах, система камеры: X вправо, Y вниз, Z от камеры.
# Под D455: ближе ~0.5 м она слепнет, поэтому Z начинается с 0.55.
ZONE = {
    "x": (-0.28, 0.28),
    "y": (-0.22, 0.22),
    "z": (0.55, 1.00),
}

# Хват: отношение «расстояние между пальцами / размер ладони».
# Безразмерное, поэтому не зависит от того, далеко вы или близко.
# Уточняется клавишами c и o прямо на ходу.
GRIP = {"closed": 0.25, "open": 1.40}

# Длины звеньев в метрах — ЗАМЕРЬТЕ ЛИНЕЙКОЙ между осями серв.
# L2 — плечо (ось сервы 2 до оси 3), L3 — предплечье (ось 3 до оси 4).
# От них зависит, попадёт ли схват туда, куда вы показываете.
L2, L3 = 0.130, 0.130

# Куда дотягивается схват, в метрах от оси базы.
# REACH — вперёд, HEIGHT — вверх от стола.
# Дальше, чем L2+L3, звенья не дотянутся физически — верхнюю границу
# держим чуть ниже этой суммы, иначе схват упрётся в вытянутую прямую
# и перестанет реагировать на последние сантиметры движения руки.
REACH = (0.07, 0.235)

# Вниз уходим НИЖЕ оси плеча (отрицательные значения) — иначе схват
# не достаёт до стола, и брать со стола нечего.
HEIGHT = (-0.06, 0.26)

SMOOTH = 0.25          # IIR: больше — резче и дёрганее, меньше — плавнее и ленивее
MAX_STEP = 40          # максимум тиков за кадр, страховка от рывков
DEADBAND = 6           # мельче — не дёргаем серву впустую


def load_axes():
    """Знаки осей. Лежат в файле, чтобы не терялись между запусками."""
    default = {str(i): 1 for i in SERVOS}
    if os.path.exists(AXIS_FILE):
        try:
            with open(AXIS_FILE, encoding="utf-8") as f:
                default.update(json.load(f))
        except Exception:
            pass
    return {int(k): int(v) for k, v in default.items()}


def save_axes(axes):
    with open(AXIS_FILE, "w", encoding="utf-8") as f:
        json.dump({str(k): v for k, v in axes.items()}, f)


def to_ticks(sid, t, axes):
    """
    Доля 0..1 -> тики сервы, с учётом знака оси.

    Знак применяем ЗДЕСЬ, в одном месте. В старом коде направление
    задавалось порядком аргументов в map_value, и чтобы развернуть
    ось, приходилось искать нужную формулу и менять местами числа.
    """
    t = float(np.clip(t, 0.0, 1.0))
    if axes.get(sid, 1) < 0:
        t = 1.0 - t
    s = SERVOS[sid]
    return int(round(s["min"] + (s["max"] - s["min"]) * t))


def frac(v, lo, hi):
    """Положение v внутри [lo, hi] как доля 0..1."""
    if hi == lo:
        return 0.5
    return float(np.clip((v - lo) / (hi - lo), 0.0, 1.0))


def palm_scale(lm):
    """
    Размер ладони в долях кадра: запястье (0) -> основание среднего (9).

    Нужен как мерка для хвата. Абсолютное расстояние между пальцами
    меняется, когда вы просто отходите от камеры; отношение к размеру
    ладони — нет.
    """
    dx = lm[9].x - lm[0].x
    dy = lm[9].y - lm[0].y
    return max(math.hypot(dx, dy), 1e-6)


def pinch_ratio(lm):
    """Насколько разведены большой (4) и указательный (8) пальцы."""
    dx = lm[8].x - lm[4].x
    dy = lm[8].y - lm[4].y
    return math.hypot(dx, dy) / palm_scale(lm)



def reach_to_joints(reach, height):
    """
    Схват в точку (reach вперёд, height вверх) -> углы плеча и локтя.

    Это и есть «интуитивно»: рука человека и схват робота оказываются
    в одном месте, а как для этого сложатся плечо с локтем — забота
    математики, а не оператора.

    Почему это НЕ та обратная кинематика, что ощущалась хаотично:
      * плоскость одна, задача сводится к двум звеньям — решение
        находится формулой, без итераций и без «выбора из вариантов»;
      * ветвь локтя жёстко зафиксирована (локоть всегда вниз), поэтому
        одна и та же поза руки ВСЕГДА даёт одну и ту же конфигурацию.
        Именно перескок между ветвями и создаёт ощущение, что рука
        живёт своей жизнью;
      * кисть довёрнута так, чтобы схват был параллелен столу — что бы
        ни делали плечо с локтем.

    Возвращает (q2, q3, q4) в радианах или None, если не дотянуться.
    """
    d = math.hypot(reach, height)
    d = min(d, (L2 + L3) * 0.999)      # до предела вытягивания, но не в него
    d = max(d, abs(L2 - L3) * 1.001)   # и не ближе, чем звенья позволяют

    # теорема косинусов для двух звеньев
    cos_e = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    q3 = -math.acos(float(np.clip(cos_e, -1.0, 1.0)))   # минус = локоть вниз

    cos_s = (d * d + L2 * L2 - L3 * L3) / (2 * L2 * d)
    q2 = math.atan2(height, reach) + math.acos(float(np.clip(cos_s, -1.0, 1.0)))

    q4 = -(q2 + q3)      # схват параллелен столу

    return q2, q3, q4


class IIR:
    """
    Экспоненциальное сглаживание, тот же smooth_angles из вашего файла,
    но помнит своё состояние само и умеет мгновенно перезапускаться,
    когда рука потерялась и появилась снова (иначе серва поедет из
    старой точки в новую через весь диапазон).
    """

    def __init__(self, alpha):
        self.a = alpha
        self.v = None

    def update(self, x):
        if x is None:
            return self.v
        self.v = x if self.v is None else self.v * (1 - self.a) + x * self.a
        return self.v

    def reset(self):
        self.v = None


class Median:
    """Медиана последних n значений — против одиночных выбросов глубины."""

    def __init__(self, n=5):
        self.n = n
        self.buf = []

    def update(self, x):
        if x is None:
            return None
        self.buf.append(x)
        if len(self.buf) > self.n:
            self.buf.pop(0)
        return float(np.median(self.buf))

    def reset(self):
        self.buf.clear()


def main():
    require_root("hand_control.py")

    if not os.path.exists(MODEL_PATH):
        sys.exit(f"нет файла модели {MODEL_PATH}")

    axes = load_axes()
    if "grip_min" in axes:
        SERVOS[6]["min"] = axes["grip_min"]
        print("лимит схвата из файла:", SERVOS[6]["min"])

    try:
        servo = ST3215(SERIAL_PORT)
        ids = servo.ListServos()
        print("сервы на связи:", ids)
        for i in SERVOS:
            servo.SetMode(i, 0)      # режим позиции, один раз
    except Exception as e:
        print("нет связи с сервами:", e, "— работаем вхолостую")
        servo = None

    cam = open_best_camera()

    options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(
            model_asset_path=MODEL_PATH,
            delegate=python.BaseOptions.Delegate.CPU),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=2,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)

    cur = {i: SERVOS[i]["mid"] for i in SERVOS}
    flt = {i: IIR(SMOOTH) for i in SERVOS}
    depth_med = Median(5)
    grip_med = Median(5)

    enabled = False
    await_axis = False
    ts = 0
    fps = 0.0
    last = time.time()
    zone_z = None          # последняя измеренная глубина, для подсказки

    cv2.namedWindow("Manipulator", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Manipulator", 1280, 720)

    while True:
        color, depth = cam.read()
        if color is None:
            continue

        # Зеркалим оба кадра сразу: и картинку, и глубину. Тогда
        # «вправо на экране» = «вправо у вас», а замер глубины
        # по-прежнему совпадает с точками MediaPipe.
        color = cv2.flip(color, 1)
        depth = cv2.flip(depth, 1)
        h, w = color.shape[:2]

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
        last = now

        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        res = landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
        ts += 33

        targets = {}     # доли 0..1 — для базы, схвата, поворота
        ticks = {}       # готовые тики — то, что посчитала кинематика
        info = []

        hands = res.hand_landmarks or []
        if not hands:
            depth_med.reset()
            grip_med.reset()

        for idx, lm in enumerate(hands):
            for p in lm:
                cv2.circle(color, (int(p.x * w), int(p.y * h)), 4, (0, 255, 0), -1)

            if idx == 0:
                # --- ОСНОВНАЯ РУКА: база, плечо, локоть, схват ---
                targets[1] = frac(lm[0].x, 0.15, 0.85)          # влево-вправо

                # Глубина ладони, в метрах. Меряем по нескольким точкам
                # ЛАДОНИ, а не по кончикам пальцев: на кончиках живут
                # «летающие пиксели» — там граница объекта, и стерео
                # выдаёт нечто среднее между рукой и стеной за ней.
                zs = []
                for i in (0, 1, 5, 9, 13, 17):
                    z = patch_depth(depth, int(lm[i].x * w), int(lm[i].y * h), half=4)
                    if z:
                        zs.append(z)
                if zs:
                    zone_z = depth_med.update(float(np.median(zs)))
                else:
                    zone_z = depth_med.update(None)

                if zone_z is not None:
                    # Рука в кадре -> точка в рабочей зоне робота.
                    # Ближе к камере = дальше вперёд: тянетесь к камере,
                    # и схват тянется от робота.
                    tz = 1.0 - frac(zone_z, ZONE["z"][0], ZONE["z"][1])
                    ty = frac(1.0 - lm[0].y, 0.15, 0.85)

                    reach = REACH[0] + (REACH[1] - REACH[0]) * tz
                    hgt = HEIGHT[0] + (HEIGHT[1] - HEIGHT[0]) * ty

                    q2, q3, q4 = reach_to_joints(reach, hgt)
                    ticks[2] = rad_to_ticks(2, q2)
                    ticks[3] = rad_to_ticks(3, q3)
                    ticks[4] = rad_to_ticks(4, q4)

                    info.append(f"рука {zone_z*100:5.1f} см  ->  "
                                f"вперёд {reach*100:4.1f} см, "
                                f"высота {hgt*100:4.1f} см")
                else:
                    info.append("глубина — (подойдите на 55-100 см)")

                r = grip_med.update(pinch_ratio(lm))
                if r is not None:
                    targets[6] = frac(r, GRIP["closed"], GRIP["open"])
                    info.append(f"хват {r:.2f}"
                                f"  (закрыт {GRIP['closed']:.2f} / "
                                f"открыт {GRIP['open']:.2f})")

            elif idx == 1:
                # --- ВТОРАЯ РУКА: кисть и поворот ---
                # Кисть держит кинематика (схват параллелен столу),
                # поэтому второй руке остаётся только поворот.
                targets[5] = frac(pinch_ratio(lm), GRIP["closed"], GRIP["open"])

        # --- фильтрация и отправка ---
        goals = {sid: to_ticks(sid, t, axes) for sid, t in targets.items()}
        goals.update(ticks)          # кинематика уже дала тики

        moves = {}
        for sid, goal in goals.items():
            sm = flt[sid].update(goal)
            step = int(np.clip(sm - cur[sid], -MAX_STEP, MAX_STEP))
            new = int(np.clip(cur[sid] + step, SERVOS[sid]["min"], SERVOS[sid]["max"]))
            if abs(new - cur[sid]) >= DEADBAND:
                cur[sid] = new
                moves[sid] = new

        if enabled and servo and moves:
            # Одним пакетом на все сервы: MoveTo делает пять обменов
            # по шине на каждую серву, и на 30 кадрах шина захлёбывается.
            try:
                lo, hi = servo.sts_lobyte, servo.sts_hibyte
                servo.groupSyncWrite.clearParam()
                for sid, pos in moves.items():
                    servo.groupSyncWrite.addParam(sid, [
                        50, lo(pos), hi(pos), lo(0), hi(0), lo(2400), hi(2400)])
                servo.groupSyncWrite.txPacket()
                servo.groupSyncWrite.clearParam()
            except Exception as e:
                print("групповая запись не прошла:", e)
                for sid, pos in moves.items():
                    servo.MoveTo(sid, pos)

        # --- экран ---
        lines = [
            f"FPS {fps:4.1f}   {'РАБОТА' if enabled else 'ПАУЗА (пробел)'}"
            f"   рук в кадре: {len(hands)}",
        ] + info + [
            "  ".join(f"{SERVOS[i]['name']}={cur[i]}"
                      f"{'-' if axes.get(i, 1) < 0 else ''}" for i in sorted(SERVOS)),
        ]
        if await_axis:
            lines.append("какую ось развернуть? нажмите 1..6")

        for i, t in enumerate(lines):
            cv2.putText(color, t, (10, 26 + i * 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 4)
            cv2.putText(color, t, (10, 26 + i * 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1)

        cv2.imshow("Manipulator", color)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        elif k == ord(" "):
            enabled = not enabled
            print("режим:", "РАБОТА" if enabled else "ПАУЗА")
        elif k == ord("i"):
            await_axis = True
        elif await_axis and ord("1") <= k <= ord("6"):
            sid = k - ord("0")
            axes[sid] = -axes.get(sid, 1)
            save_axes(axes)
            flt[sid].reset()
            print(f"ось {sid} ({SERVOS[sid]['name']}) развёрнута:", axes[sid])
            await_axis = False
        elif k == ord("c") and hands:
            GRIP["closed"] = pinch_ratio(hands[0])
            print("схват закрыт при", round(GRIP["closed"], 3))
        elif k == ord("o") and hands:
            GRIP["open"] = pinch_ratio(hands[0])
            print("схват открыт при", round(GRIP["open"], 3))
        elif k in (ord("["), ord("]")):
            # Двигаем «полностью закрыто». Серва при упоре в механику
            # начинает греться и жрать ток, поэтому подбираем на слух:
            # доводим до касания губок и останавливаемся.
            step = -20 if k == ord("[") else 20
            SERVOS[6]["min"] = int(np.clip(SERVOS[6]["min"] + step,
                                           1500, SERVOS[6]["max"] - 100))
            axes["grip_min"] = SERVOS[6]["min"]
            save_axes(axes)
            print(f"схват закрыт при {SERVOS[6]['min']} тиках"
                  f"  ([ ещё, ] назад)")
            if servo:
                servo.MoveTo(6, SERVOS[6]["min"])

        elif k == ord("z"):
            for i in SERVOS:
                cur[i] = SERVOS[i]["mid"]
                flt[i].reset()
                if servo:
                    servo.MoveTo(i, cur[i])
            print("сервы в середине диапазона")

    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
