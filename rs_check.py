"""
Проверка камеры RealSense + наглядная демонстрация, почему одну точку
глубины брать нельзя.

Запустить ПЕРВЫМ, до всего остального. Робот не нужен.

Кликните мышью по картинке — скрипт 60 кадров подряд меряет глубину
в этой точке двумя способами и печатает статистику:

    сырой пиксель : 0.612 м   разброс 34.1 мм   дырок 12/60
    окно 9x9      : 0.607 м   разброс  2.3 мм   дырок  0/60

Второе число — это то, ради чего всё затевалось. Три сантиметра дрожи
на суставе — это дёрганая рука; два миллиметра — это плавная.
Если разброс по окну у вас всё равно большой, значит:
  - точка попала на границу объекта (сдвиньтесь к центру),
  - или объект слишком далеко (подойдите к 0.4-0.6 м),
  - или блик / чёрная поверхность (стереокамера их не видит).
"""

import time

import cv2
import numpy as np

from manip.depth import open_best_camera, require_root, patch_depth

click_pt = None


def on_mouse(event, x, y, flags, param):
    global click_pt
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pt = (x, y)


def main():
    global click_pt

    require_root("rs_check.py")

    # Режим не хардкодим: подбираем тот, что реально отдаёт кадры
    # (на USB 2 старт проходит, а кадры могут не пойти).
    cam = open_best_camera()

    print(f"depth_scale = {cam.depth_scale} м/единицу")

    cv2.namedWindow("realsense check", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("realsense check", on_mouse)

    probe = None          # (x, y, [сырые], [по окну])
    fps = 0.0
    last_t = time.time()

    while True:
        color, depth = cam.read()
        if color is None:
            continue

        # Реальный FPS. Если он заметно ниже запрошенного (30) —
        # камере не хватает полосы: хаб, переходник или кабель USB 2.
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - last_t, 1e-3)
        last_t = now

        if click_pt is not None:
            probe = (click_pt[0], click_pt[1], [], [])
            click_pt = None
            print(f"\nзамер в точке {probe[0]}, {probe[1]} ...")

        if probe is not None and len(probe[2]) < 60:
            x, y, raw, med = probe
            raw.append(float(depth[y, x]))
            med.append(patch_depth(depth, x, y, half=4))

            if len(raw) == 60:
                r = np.array(raw)
                m = np.array([v for v in med if v is not None])
                r_ok = r[r > 0]
                print(f"  сырой пиксель : "
                      f"{r_ok.mean() if r_ok.size else float('nan'):.3f} м   "
                      f"разброс {r_ok.std()*1000 if r_ok.size else 0:5.1f} мм   "
                      f"дырок {60 - r_ok.size}/60")
                print(f"  окно 9x9      : "
                      f"{m.mean() if m.size else float('nan'):.3f} м   "
                      f"разброс {m.std()*1000 if m.size else 0:5.1f} мм   "
                      f"дырок {60 - m.size}/60")
                if m.size and r_ok.size:
                    print(f"  выигрыш по стабильности: "
                          f"{r_ok.std()/max(m.std(), 1e-9):.1f}x")

        vis = color.copy()
        if probe is not None:
            cv2.drawMarker(vis, (probe[0], probe[1]), (0, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)
            cv2.rectangle(vis, (probe[0] - 4, probe[1] - 4),
                          (probe[0] + 4, probe[1] + 4), (0, 255, 255), 1)

        # Карта глубины: чёрное = дырка (нет данных).
        d8 = cv2.convertScaleAbs(depth, alpha=255.0 / 2.0)
        dmap = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
        dmap[depth <= 0] = (0, 0, 0)

        holes = float((depth <= 0).mean()) * 100
        cv2.putText(dmap, f"holes {holes:.1f}%   fps {fps:4.1f}   "
                          f"USB {cam.usb['usb']}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        cv2.imshow("realsense check", np.hstack([vis, dmap]))
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
