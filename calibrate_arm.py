"""
Калибровка нулей суставов — обязательный шаг перед обратной кинематикой.

ОК считает физические углы. Чтобы перевести их в тики, надо знать,
какому тику соответствует угол 0 у КАЖДОГО сустава именно вашей сборки.
Рог сервы надевается с шагом ~1.4°, так что чужие числа не подойдут.

НУЛЕВАЯ ПОЗА (та, для которой углы = 0 в manip/kinematics.py):

    рука вытянута ГОРИЗОНТАЛЬНО ВПЕРЁД, вся в одну линию,
    схват смотрит вперёд, параллельно столу:

        ==o====o====o====o--->     все звенья на одной прямой
          q1   q2   q3   q4

    q1 = 0: плечо смотрит прямо вперёд (не вбок)
    q2 = 0: плечо горизонтально
    q3 = 0: предплечье продолжает плечо, локоть прямой
    q4 = 0: схват продолжает предплечье

Как пользоваться:
    1. python calibrate_arm.py
    2. нажать 'r' — сервы обмякнут
    3. руками выставить нулевую позу (лучше по линейке/уровню)
    4. нажать 'z' — скрипт напечатает готовый блок JOINTS
    5. вставить напечатанное в manip/arm.py
    6. проверить: нажать 'h' — рука должна вернуться ровно в эту позу

Про dir (+1/-1): нажмите 't' — каждый сустав по очереди повернётся
на +15° по модели. Если сустав пошёл НЕ в ту сторону, поменяйте знак
dir у него в manip/arm.py.

Направления «правильных» сторон:
    q1 +  -> влево (против часовой, если смотреть сверху)
    q2 +  -> плечо вверх
    q3 +  -> предплечье вверх относительно плеча
    q4 +  -> схват вверх относительно предплечья
"""

import sys
import time

from manip.arm import Arm, JOINTS, TICKS_PER_RAD

try:
    import numpy as np
except ImportError:
    sys.exit("нужен numpy")


def getch():
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    arm = Arm(max_step_ticks=25)
    if arm.servo is None:
        sys.exit("сервы не подключены — калибровать нечего")

    print(__doc__)
    print("\nклавиши: r=обмякнуть  z=записать нули  t=тест направлений  "
          "h=в нули  p=позиции  q=выход\n")

    while True:
        c = getch()

        if c == "q":
            break

        elif c == "r":
            arm.relax()
            print("момент снят, выставляйте позу руками")

        elif c == "p":
            pos = arm.read_positions()
            print("  " + "  ".join(
                f"{JOINTS[i]['name']}="
                + ("НЕ ОТВЕЧАЕТ" if pos[i] is None else str(pos[i]))
                for i in sorted(pos)))

        elif c == "z":
            pos = arm.read_positions()

            # Молчащая серва даёт None. Записать её как ноль нельзя:
            # ошибка уедет в конфиг и потом будет искажать все углы,
            # причём незаметно. Лучше честно отказаться.
            mute = [i for i in sorted(pos) if pos[i] is None]
            if mute:
                print("\nне ответили: " + ", ".join(
                    f"{i} ({JOINTS[i]['name'].strip()})" for i in mute))
                print("Нули не записаны — сначала разберитесь со связью.")
                print("Проверьте питание манипулятора и шлейф; список\n"
                      "отзывающихся серв покажет ListServos().\n")
                continue

            print("\n--- вставить в manip/arm.py вместо JOINTS ---")
            print("JOINTS = {")
            for i in sorted(JOINTS):
                j = JOINTS[i]
                print(f'    {i}: {{"name": "{j["name"]:<13}", '
                      f'"zero": {pos[i]:4d}, "dir": {j["dir"]:2d}, '
                      f'"min": {j["min"]:4d}, "max": {j["max"]:4d}}},')
            print("}")
            print("--- конец ---\n")

        elif c == "t":
            print("тест направлений: каждый сустав на +15°")
            for i in sorted(JOINTS):
                if i == 6:
                    continue
                base = arm.current[i]
                delta = int(np.radians(15) * TICKS_PER_RAD * JOINTS[i]["dir"])
                print(f"  {JOINTS[i]['name']}: +15° ...")
                for _ in range(40):
                    arm.move_ticks({i: base + delta})
                    time.sleep(0.02)
                time.sleep(0.6)
                for _ in range(40):
                    arm.move_ticks({i: base})
                    time.sleep(0.02)

        elif c == "h":
            arm.go_home()
            print("в нулевой позе")

    arm.relax()


if __name__ == "__main__":
    main()
