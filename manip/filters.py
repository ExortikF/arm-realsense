"""
Фильтры для сглаживания сигналов с камеры.

Главная мысль: у нас два разных врага, и их надо бить разным оружием.

  1. ВЫБРОСЫ (spikes) — глубина на один кадр прыгнула на метр,
     потому что пиксель попал на край руки. Это НЕ шум, это мусор.
     Линейный фильтр (IIR) на выброс отреагирует — «размажет» его
     на десяток кадров. Поэтому выбросы режем НЕЛИНЕЙНЫМ фильтром
     (медиана) ДО того, как отдать значение в IIR.

  2. ШУМ (jitter) — значение дрожит на ±2 мм вокруг правды.
     Вот это как раз задача IIR.

Правильный порядок: медиана -> IIR.  Не наоборот.
"""

import math
from collections import deque


class IIR:
    """
    Однополюсный IIR (экспоненциальное сглаживание):

        y[n] = a * x[n] + (1 - a) * y[n-1]

    Это то же самое, что было в исходном smooth_angles(), но с двумя
    доработками, без которых оно врёт:

    * alpha считается от ЧАСТОТЫ СРЕЗА и реального dt, а не подбирается
      на глаз. Иначе при просадке FPS с 30 до 15 фильтр незаметно
      становится вдвое «тяжелее» и рука начинает запаздывать.

    * есть reset(): при потере руки нельзя тянуть фильтр к нулю,
      его надо просто заморозить, а при возврате — перезапустить
      с нового значения (иначе рука рванёт от старой позиции к новой).

    cutoff_hz — частота среза:
        0.5 ... 1.0 Гц — очень плавно, заметное запаздывание (~0.2-0.3 с)
        2.0 ... 4.0 Гц — компромисс, для управления рукой обычно то, что нужно
        8.0 +          — почти не фильтрует
    """

    def __init__(self, cutoff_hz=3.0):
        self.cutoff_hz = cutoff_hz
        self.y = None

    def reset(self, value=None):
        self.y = value

    def update(self, x, dt):
        if x is None:
            return self.y

        if self.y is None:
            self.y = x
            return self.y

        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        a = dt / (tau + dt)          # dt в знаменателе — вот эта поправка на FPS
        self.y = a * x + (1.0 - a) * self.y
        return self.y


class MedianSpikeFilter:
    """
    Скользящая медиана по последним N кадрам.

    Убивает одиночные выбросы полностью (медиана из 5 значений не сдвинется,
    даже если два из них — полный мусор), при этом не размазывает фронт,
    когда рука реально двинулась.

    size обязательно нечётный. 5 — хорошее значение для 30 fps:
    задержка ~2 кадра (66 мс), а одиночные и двойные глюки глубины уходят.
    """

    def __init__(self, size=5):
        assert size % 2 == 1, "размер окна медианы должен быть нечётным"
        self.buf = deque(maxlen=size)

    def reset(self):
        self.buf.clear()

    def update(self, x):
        if x is None:
            return None
        self.buf.append(x)
        s = sorted(self.buf)
        return s[len(s) // 2]


class OneEuro:
    """
    One Euro Filter — адаптивный IIR.

    Проблема обычного IIR: одна и та же alpha и для медленного движения,
    и для быстрого. Хочется-то разного:
      - рука стоит на месте -> фильтруем сильно, чтобы не дрожала
      - рука быстро летит    -> фильтруем слабо, чтобы не запаздывала

    One Euro именно это и делает: он оценивает скорость сигнала и
    поднимает частоту среза пропорционально ей.

        cutoff = min_cutoff + beta * |скорость|

    Внутри — те же самые однополюсные IIR, просто alpha пересчитывается
    каждый кадр. Для управления рукой по камере это заметно приятнее
    обычного IIR: в статике нет дрожи, в динамике нет «резины».

    Настройка (в таком порядке!):
      1. beta = 0, крутим min_cutoff, пока дрожь в покое не исчезнет
      2. потом поднимаем beta, пока не уйдёт запаздывание на резких движениях
    """

    def __init__(self, min_cutoff=0.8, beta=0.05, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0

    def reset(self, value=None):
        self.x_prev = value
        self.dx_prev = 0.0

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return dt / (tau + dt)

    def update(self, x, dt):
        if x is None:
            return self.x_prev

        if self.x_prev is None:
            self.x_prev = x
            return x

        # 1) оценка производной, сама по себе сглаженная
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx_prev = a_d * dx + (1.0 - a_d) * self.dx_prev

        # 2) частота среза растёт со скоростью
        cutoff = self.min_cutoff + self.beta * abs(self.dx_prev)
        a = self._alpha(cutoff, dt)

        self.x_prev = a * x + (1.0 - a) * self.x_prev
        return self.x_prev


class VectorFilter:
    """
    Обёртка: тот же фильтр, применённый покомпонентно к вектору
    (точке XYZ или списку углов). Плюс медиана перед ним.

    use_one_euro=True  -> OneEuro (рекомендую для точки руки в 3D)
    use_one_euro=False -> обычный IIR (для углов сервоприводов хватает)
    """

    def __init__(self, n, median_size=5, use_one_euro=True, **kwargs):
        self.med = [MedianSpikeFilter(median_size) for _ in range(n)]
        if use_one_euro:
            self.flt = [OneEuro(**kwargs) for _ in range(n)]
        else:
            self.flt = [IIR(**kwargs) for _ in range(n)]

    def reset(self):
        for m in self.med:
            m.reset()
        for f in self.flt:
            f.reset()

    def update(self, vec, dt):
        if vec is None:
            return None
        out = []
        for i, v in enumerate(vec):
            out.append(self.flt[i].update(self.med[i].update(v), dt))
        return out
