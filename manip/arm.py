"""
Драйвер руки: углы в радианах -> тики сервоприводов ST3215.

Отдельный слой нужен затем, чтобы кинематика ничего не знала про тики,
а тики — про кинематику. Кинематика оперирует физическими углами,
здесь они переводятся в «попугаи» конкретного железа.
"""

import time

import numpy as np

TICKS_PER_TURN = 4096          # STS3215: 12 бит на полный оборот
TICKS_PER_RAD = TICKS_PER_TURN / (2 * np.pi)


# Калибровка каждого сустава.
#   zero  — тик, при котором ФИЗИЧЕСКИЙ угол сустава равен нулю
#           (нулевые позы описаны в calibrate_arm.py)
#   dir   — +1 или -1: совпадает ли направление роста тиков
#           с направлением роста угла в модели кинематики
#   min/max — механические пределы из my_awesome_follower_arm.json
#
# zero и dir ОБЯЗАТЕЛЬНО откалибровать под свою сборку:
# рог сервы можно надеть с шагом в один зуб, и у каждой руки нули свои.
JOINTS = {
    1: {"name": "shoulder_pan ", "zero": 2036, "dir": -1, "min": 1044, "max": 3038},
    2: {"name": "shoulder_lift", "zero": 1027, "dir":  1, "min":  958, "max": 3142},
    3: {"name": "elbow_flex   ", "zero": 2939, "dir":  1, "min":  981, "max": 3488},
    4: {"name": "wrist_flex   ", "zero": 2825, "dir":  1, "min": 1525, "max": 2962},
    5: {"name": "wrist_roll   ", "zero": 2042, "dir":  1, "min":    0, "max": 4095},
    # У схвата "zero" раньше был 2021 — НИЖЕ, чем "min" 2026, поэтому
    # закрыться до конца он не мог: команда упиралась в ограничение.
    # И мерить его в радианах бессмысленно — это не сустав, а губки.
    # Задаём прямо: сколько тиков = закрыто, сколько = открыто.
    6: {"name": "gripper      ", "zero": 2026, "dir":  1, "min": 2026, "max": 3165,
        "closed": 2026, "open": 3165},
}



def rad_to_ticks(joint_id, angle_rad):
    j = JOINTS[joint_id]
    t = j["zero"] + j["dir"] * angle_rad * TICKS_PER_RAD
    return int(np.clip(round(t), j["min"], j["max"]))


def ticks_to_rad(joint_id, ticks):
    j = JOINTS[joint_id]
    return (ticks - j["zero"]) / TICKS_PER_RAD * j["dir"]


class Arm:
    """
    Обёртка над ST3215 с двумя защитами, которых не было в исходнике.

    1. ОГРАНИЧЕНИЕ СКОРОСТИ (max_step_ticks). Даже с хорошей фильтрацией
       рано или поздно прилетит скачок цели: рука пропала из кадра и
       появилась в другом месте, или ОК переключилась на другое решение.
       Без ограничения серва отработает это на полном моменте — это
       и есть те самые «дёрганья», от которых гнутся кронштейны.
       Ограничение превращает любой скачок в плавный проезд.

    2. МЁРТВАЯ ЗОНА (deadband). Не слать команду, если изменение меньше
       нескольких тиков. Разгружает шину (6 серв * 30 Гц — это уже
       заметный трафик на 1 Мбит) и убирает гудение серв, которые
       дёргаются вокруг цели на уровне шума.
    """

    def __init__(self, port="/dev/tty.usbmodem5AE60853301",
                 max_step_ticks=40, deadband=3, speed=1200, acc=30,
                 dry_run=False):
        self.max_step = max_step_ticks
        self.deadband = deadband
        self.speed = speed
        self.acc = acc
        self.servo = None
        self._sync_ok = True      # групповая запись; выключится при сбое
        self.current = {i: JOINTS[i]["zero"] for i in JOINTS}

        if dry_run:
            print("[arm] dry-run: команды в сервы не уходят")
            return

        try:
            from python_st3215.st3215 import ST3215
            self.servo = ST3215(port)
            ids = self.servo.ListServos()
            print(f"[arm] подключены сервы: {ids}")

            missing = [i for i in JOINTS if i not in ids]
            if missing:
                print(f"[arm] ВНИМАНИЕ: не отозвались {missing} — "
                      "проверьте питание и шлейф")

            for i in JOINTS:
                if i in ids:
                    pos = self.servo.ReadPosition(i)
                    if pos is not None:
                        self.current[i] = pos
                    # Режим позиционирования ставим ОДИН раз здесь.
                    # В цикле его трогать незачем, а MoveTo делала это
                    # на каждом кадре — лишний обмен по шине.
                    self.servo.SetMode(i, 0)
        except Exception as e:
            print(f"[arm] нет связи с сервами ({e}) — работаем вхолостую")
            self.servo = None

    def read_positions(self, retries=3):
        """
        Текущие позиции всех суставов в тиках.

        Значение может быть None: шина полудуплексная, и ответ иногда
        теряется — особенно сразу после команды движения. Поэтому
        переспрашиваем несколько раз, прежде чем признать серву молчащей.
        Вызывающий код обязан быть готов к None (см. calibrate_arm.py).
        """
        if not self.servo:
            return dict(self.current)

        out = {}
        for i in JOINTS:
            for _ in range(retries):
                v = self.servo.ReadPosition(i)
                if v is not None:
                    break
                time.sleep(0.02)
            out[i] = v
        return out

    def grip_ticks(self, t):
        """
        t: 0 = полностью закрыт, 1 = полностью открыт.

        Отдельно от суставов намеренно: у губок нет физического угла,
        который что-то значил бы для кинематики, зато есть два крайних
        положения. Линейная интонация между ними понятнее и, главное,
        реально доходит до упоров — в отличие от пересчёта через
        радианы, где крайние значения упирались в clip.
        """
        j = JOINTS[6]
        lo, hi = j.get("closed", j["min"]), j.get("open", j["max"])
        t = float(np.clip(t, 0.0, 1.0))
        return int(round(lo + (hi - lo) * t))

    def move_rad(self, angles_rad, gripper_rad=None):
        """
        angles_rad — [q1, q2, q3, q4] (+ опционально q5 для wrist_roll).
        gripper_rad — угол захвата, либо None чтобы не трогать.
        """
        targets = {}
        for idx, ang in enumerate(angles_rad, start=1):
            targets[idx] = rad_to_ticks(idx, ang)
        if gripper_rad is not None:
            targets[6] = self.grip_ticks(gripper_rad)
        self.move_ticks(targets)

    def move_ticks(self, targets):
        moves = {}
        for jid, target in targets.items():
            cur = self.current[jid]
            step = int(np.clip(target - cur, -self.max_step, self.max_step))
            new = int(np.clip(cur + step, JOINTS[jid]["min"], JOINTS[jid]["max"]))

            if abs(new - cur) < self.deadband:
                continue

            self.current[jid] = new
            moves[jid] = new

        if self.servo and moves:
            self._send(moves)

    def _send(self, moves):
        """
        Отправить целевые позиции всем суставам ОДНИМ пакетом.

        Почему не MoveTo: она делает пять обменов по шине на каждую серву
        (SetMode, SetAcceleration, SetSpeed, ReadPosition, WritePosition),
        причём режим со скоростью переустанавливает каждый раз заново,
        а ReadPosition нужен ей только для wait=True, которого мы не
        используем. Шесть серв на 30 кадрах — это 900 транзакций в
        секунду на полудуплексной линии, столько она не пропустит,
        и цикл упрётся в шину вместо камеры.

        Групповая запись шлёт одну широковещательную посылку без ответов.
        Регистры 41..47 идут подряд (ACC, POSITION, TIME, SPEED), ровно
        семь байт — под них groupSyncWrite и настроен в конструкторе
        библиотеки. Режим выставлен один раз при старте, в цикле он
        не меняется.
        """
        if not self._sync_ok:
            for jid, pos in moves.items():
                self.servo.MoveTo(jid, pos, speed=self.speed, acc=self.acc)
            return

        try:
            lo, hi = self.servo.sts_lobyte, self.servo.sts_hibyte
            self.servo.groupSyncWrite.clearParam()
            for jid, pos in moves.items():
                self.servo.groupSyncWrite.addParam(jid, [
                    self.acc,
                    lo(pos), hi(pos),          # цель
                    lo(0), hi(0),              # время хода: 0 = идти по скорости
                    lo(self.speed), hi(self.speed),
                ])
            self.servo.groupSyncWrite.txPacket()
            self.servo.groupSyncWrite.clearParam()
        except Exception as e:
            # Не роняем управление из-за оптимизации: откатываемся на
            # медленный, но проверенный путь и больше не пробуем.
            print(f"[arm] групповая запись не удалась ({e}), перехожу на MoveTo")
            self._sync_ok = False
            for jid, pos in moves.items():
                self.servo.MoveTo(jid, pos, speed=self.speed, acc=self.acc)

    def go_home(self):
        """Плавно в нулевую позу. Вызывать при старте и на выходе."""
        home = {i: JOINTS[i]["zero"] for i in JOINTS}
        for _ in range(120):          # шагами по max_step, пока не доедет
            self.move_ticks(home)
            if all(abs(self.current[i] - home[i]) < self.deadband for i in home):
                break

    def relax(self):
        """Снять момент — рука обмякает, можно двигать руками."""
        if self.servo:
            for i in JOINTS:
                try:
                    self.servo.StopServo(i)
                except Exception:
                    pass
