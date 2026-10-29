#!/usr/bin/env python3
"""
GPIO Pulse Generator — TELEOFIS LT70
Генератор импульсов для УСПД RTU202 (режим сухой контакт)

Полностью интерактивный: просто запустите без аргументов —
программа спросит все параметры по шагам (Enter = значение по умолчанию).

  python3 gpio_pulse_gen.py

Использует sysfs GPIO iolines LT70 (ссылки создаёт /etc/init.d/iolines boot):
  /dev/pu{N} → /sys/class/gpio/gpio{384+N}  — pull-up
  /dev/pd{N} → /sys/class/gpio/gpio{393+N}  — pull-down (наш выход OC)
Линии висят на I2C-расширителе (gpiochip3), запись в value — синхронная и
небыстрая, поэтому value держим открытым и пишем только его.

Режимы пина (iolines):
  mode1: pu=low, pd=low  → АЦП (пассивный)
  mode2: pu=high,pd=low  → Сухой контакт (ВХОД)
  mode3: pu=low, pd=high → Открытый коллектор (ВЫХОД) ← используем

Подключение к УСПД RTU202:
  IOx роутера (mode3) → I{x}+ УСПД
  GND роутера         → I{x}- УСПД

Ограничения УСПД RTU202 (касаются и импульса, и паузы):
  Частота опроса 2 Гц  (по умолч.) → минимум 500 мс → max 1 Гц при 50 %
  Частота опроса 20 Гц             → минимум  50 мс → max 10 Гц при 50 %
"""

import sys
import os
import json
import time
import signal
import threading
from datetime import datetime

__version__ = "1.4.0"

# ─── ANSI коды ────────────────────────────────────────────────────────────────
RESET    = "\033[0m"
BOLD     = "\033[1m"
DIM      = "\033[2m"
CLEAR    = "\033[2J\033[H"
HOME     = "\033[H"
EOL      = "\033[K"     # стереть до конца строки
EOS      = "\033[J"     # стереть до конца экрана

FG_WHITE  = "\033[97m"
FG_CYAN   = "\033[96m"
FG_GREEN  = "\033[92m"
FG_YELLOW = "\033[93m"
FG_GRAY   = "\033[90m"
FG_BLACK  = "\033[30m"
FG_RED    = "\033[91m"

BG_GREEN  = "\033[42m"
BG_GRAY   = "\033[100m"
BG_WHITE  = "\033[107m"
BG_RED    = "\033[101m"

# ─── Тайминги ─────────────────────────────────────────────────────────────────
MIN_LEVEL_FAST = 0.050   # мин. длительность уровня при опросе УСПД 20 Гц, с
MIN_LEVEL_SLOW = 0.500   # мин. длительность уровня при опросе УСПД 2 Гц, с
FINISH_MAX     = 2.0     # при остановке дожидаемся конца импульса, если осталось ≤ стольких секунд

# ─── Сохранение показаний ─────────────────────────────────────────────────────
# Рядом со скриптом (/root на флеше роутера) — переживает перезагрузку.
STATE_FILE          = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "gpio_pulse_gen_state.json")
STATE_SAVE_INTERVAL = 300   # с; плюс сохранение при остановке. Чаще не надо — износ флеша

# ─── Sysfs GPIO (iolines LT70) ────────────────────────────────────────────────

def io_index(io_num: int) -> int:
    """IO1 → 0, IO2 → 1, ..., IO9 → 8"""
    return io_num - 1

def pd_path(io_num: int) -> str:
    return f"/dev/pd{io_index(io_num)}"

def pu_path(io_num: int) -> str:
    return f"/dev/pu{io_index(io_num)}"

class GpioError(Exception):
    pass

def _read(path: str) -> str:
    with open(path) as f:
        return f.read().strip()

class Line:
    """Одна sysfs-линия GPIO (/dev/puN или /dev/pdN). Уровни — физические."""
    def __init__(self, path: str):
        self.path   = path
        self.invert = False
        self.orig   = None
        self._fd    = None

    def open(self):
        try:
            if _read(f"{self.path}/direction") != "out":
                # "low" переводит линию в выход с низким уровнем (так же делает iolines)
                with open(f"{self.path}/direction", "w") as f:
                    f.write("low")
            self.invert = _read(f"{self.path}/active_low") == "1"
            self.orig   = (_read(f"{self.path}/value") == "1") != self.invert
            self._fd    = os.open(f"{self.path}/value", os.O_WRONLY)
        except OSError as e:
            raise GpioError(f"{self.path}: {e.strerror or e}") from e

    def set(self, level: bool):
        try:
            os.lseek(self._fd, 0, os.SEEK_SET)
            os.write(self._fd, b"1" if level != self.invert else b"0")
        except OSError as e:
            raise GpioError(f"{self.path}/value: {e.strerror or e}") from e

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

class Pin:
    """Пара линий pu/pd одного IO. В симуляции ничего не пишет."""
    def __init__(self, io_num: int, dry_run: bool):
        self.io_num  = io_num
        self.dry_run = dry_run
        self.pu      = Line(pu_path(io_num))
        self.pd      = Line(pd_path(io_num))
        self._opened = False

    def open(self):
        """Запоминаем исходное состояние и переводим в mode3 с отпущенной линией."""
        if self.dry_run:
            return
        self.pu.open()
        self.pd.open()
        self._opened = True
        # pd отпускаем первым, чтобы ни на миг не получить pu=high + pd=high
        self.pd.set(False)
        self.pu.set(False)

    def set(self, value: bool):
        """
        Управляем только pd:
          True  → pd=high → линия притянута к GND → УСПД видит замкнутый контакт
          False → pd=low  → линия свободна         → УСПД видит разомкнутый контакт
        """
        if self._opened:
            self.pd.set(value)

    def release(self):
        """Возвращаем режим, который был до запуска (обычно mode1). Идемпотентно."""
        if not self._opened:
            return
        self._opened = False
        try:
            self.pd.set(False)
            self.pu.set(self.pu.orig)
            self.pd.set(self.pd.orig)
        finally:
            self.pu.close()
            self.pd.close()

    def orig_mode(self) -> str:
        if self.dry_run or self.pu.orig is None:
            return "—"
        return {(False, False): "mode1", (True, False): "mode2",
                (False, True): "mode3"}.get((self.pu.orig, self.pd.orig), "pu+pd")

# ─── Канал ────────────────────────────────────────────────────────────────────
class Channel:
    def __init__(self, idx: int, io_num: int, freq: float, duty: float,
                 dry_run: bool, lpp: float = 1.0, start_liters: float = 0.0):
        self.idx          = idx
        self.io_num       = io_num
        self.freq         = freq
        self.duty         = duty
        self.pin          = Pin(io_num, dry_run)
        self.lpp          = lpp            # литров на импульс
        self.start_liters = start_liters   # начальные показания счётчика, литры

        self.count   = 0
        self.state   = False
        self.error   = None                # текст ошибки GPIO, если поток упал
        self.note    = None                # что случилось с последним импульсом при остановке
        self._thread = None
        self._stop   = threading.Event()
        self.history = [False] * 32

    @property
    def period(self):
        return 1.0 / self.freq

    @property
    def t_on(self):
        return self.period * self.duty

    @property
    def t_off(self):
        return self.period * (1.0 - self.duty)

    @property
    def liters(self) -> float:
        """Показания счётчика, литры (начальные показания + импульсы сессии)"""
        return self.start_liters + self.count * self.lpp

    @property
    def m3(self) -> float:
        """Показания счётчика, кубометры"""
        return self.liters / 1000.0

    @property
    def flow_lpm(self) -> float:
        """Текущий расход, л/мин (freq имп/с × лит/имп × 60)"""
        return self.freq * self.lpp * 60.0

    def _set(self, value: bool):
        self.pin.set(value)
        self.state = value
        # новый список вместо мутации — draw() читает history из другого потока
        self.history = self.history[1:] + [value]

    def _wait_until(self, deadline: float) -> bool:
        """Ждёт до deadline (monotonic). True — пришёл запрос остановки."""
        return self._stop.wait(timeout=max(0.0, deadline - time.monotonic()))

    def _finish_pulse(self, on_start: float, on_end: float):
        """
        Остановка пришла посреди импульса. Короткий остаток дожидаемся, чтобы
        счёт скрипта совпал с УСПД; длинный обрываем и засчитываем, только если
        импульс уже длился ≥ 500 мс (его гарантированно увидел УСПД).
        """
        remaining = on_end - time.monotonic()
        if remaining <= FINISH_MAX:
            time.sleep(max(0.0, remaining))
            self._set(False)
            self.count += 1
            return
        held = time.monotonic() - on_start
        self._set(False)
        if held >= MIN_LEVEL_SLOW:
            self.count += 1
            self.note = f"последний импульс прерван через {held*1000:.0f} мс — засчитан"
        else:
            self.note = f"последний импульс прерван через {held*1000:.0f} мс — не засчитан"

    def _run(self):
        try:
            t = time.monotonic()
            while not self._stop.is_set():
                # ON
                self._set(True)
                if self._wait_until(t + self.t_on):
                    self._finish_pulse(t, t + self.t_on)
                    break
                # OFF
                self._set(False)
                self.count += 1
                # шаг по абсолютному времени — задержки записи в I2C не копятся
                t += self.period
                if time.monotonic() - t > self.period:
                    # сильно отстали (подвис I2C) — не догоняем пачкой коротких импульсов
                    t = time.monotonic()
                self._wait_until(t)
        except GpioError as e:
            self.error = str(e)
        finally:
            self.state = False
            try:
                self.pin.release()
            except GpioError as e:
                self.error = self.error or str(e)

    def start(self, phase_delay: float = 0.0):
        def _delayed():
            if phase_delay > 0:
                self._stop.wait(timeout=phase_delay)
            if self._stop.is_set():
                self._release_quietly()
                return
            self._run()
        self._stop.clear()
        self._thread = threading.Thread(target=_delayed, daemon=True)
        self._thread.start()

    def _release_quietly(self):
        try:
            self.pin.release()
        except GpioError as e:
            self.error = self.error or str(e)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=FINISH_MAX + 3)
        else:
            self._release_quietly()

# ─── UI ───────────────────────────────────────────────────────────────────────
def box(title: str, width: int) -> list:
    """Рамка с заголовком; width — внутренняя ширина."""
    text = f"  {title}"
    return [
        f"{BOLD}{FG_CYAN}╔{'═'*width}╗{RESET}",
        f"{BOLD}{FG_CYAN}║{text}{' '*(width-len(text))}║{RESET}",
        f"{BOLD}{FG_CYAN}╚{'═'*width}╝{RESET}",
    ]

def fmt_l(v: float) -> str:
    """Литры без хвостовых нулей: 51 → '51', 12.5 → '12.5'"""
    return f"{v:.3f}".rstrip("0").rstrip(".")

def sparkline(history: list) -> str:
    out = []
    for v in history:
        out.append(f"{FG_GREEN}▄{RESET}" if v else f"{FG_GRAY}·{RESET}")
    return "".join(out)

def led(state: bool) -> str:
    if state:
        return f"{BG_GREEN}{FG_BLACK}{BOLD} ON  {RESET}"
    else:
        return f"{BG_GRAY}{FG_BLACK} OFF {RESET}"

def bar(state: bool) -> str:
    if state:
        return f"{FG_GREEN}{BOLD}██████{RESET}"
    else:
        return f"{FG_GRAY}░░░░░░{RESET}"

def odometer(ch) -> str:
    """
    Табло водосчётчика: 5 чёрных барабанов = м³, 3 красных = литры.
    1 импульс = ch.lpp литров.  Пример: 51 л → 00000 . 051  м³
    """
    total_l = round(ch.liters, 3)
    m3_int  = int(total_l // 1000)
    rem_l   = total_l - m3_int * 1000          # 0..999.xxx литры в текущем м³
    int_str = f"{m3_int:05d}"[-5:]
    frac_str = f"{int(rem_l):03d}"

    black = "".join(f"{BG_WHITE}{FG_BLACK}{BOLD} {d} {RESET}" for d in int_str)
    red   = "".join(f"{BG_RED}{FG_WHITE}{BOLD} {d} {RESET}" for d in frac_str)
    return f"{black}{FG_RED}{BOLD}.{RESET}{red}"


def draw(channels: list, dry_run: bool, start_time: float):
    elapsed = time.monotonic() - start_time
    hh = int(elapsed // 3600)
    mm = int((elapsed % 3600) // 60)
    ss = int(elapsed % 60)
    ts = datetime.now().strftime("%H:%M:%S")

    W = 60

    out = box("GPIO Pulse Generator — TELEOFIS LT70", W)

    mode = f"{FG_YELLOW}СИМУЛЯЦИЯ{RESET}" if dry_run else f"{FG_GREEN}РЕАЛЬНЫЙ GPIO{RESET}"
    out.append(f"  Режим: {mode}   {FG_GRAY}{ts}   uptime {hh:02d}:{mm:02d}:{ss:02d}{RESET}")
    out.append("")

    for ch in channels:
        idx  = io_index(ch.io_num)
        out.append(
            f"  {BOLD}{FG_CYAN}Выход {ch.idx}{RESET}  "
            f"IO{ch.io_num}  "
            f"{FG_GRAY}/dev/pd{idx}/value   (до запуска: {ch.pin.orig_mode()}){RESET}"
        )
        out.append(
            f"  {FG_GRAY}freq={ch.freq:.3f} Гц   "
            f"T={ch.period*1000:.0f} мс   "
            f"ON={ch.t_on*1000:.0f} мс   "
            f"OFF={ch.t_off*1000:.0f} мс{RESET}"
        )
        out.append(
            f"  {led(ch.state)}  {bar(ch.state)}  "
            f"Импульсов: {BOLD}{FG_WHITE}{ch.count:>6}{RESET}  "
            f"{FG_GRAY}({ch.lpp:g} л/имп){RESET}"
        )
        out.append(
            f"  {BOLD}{FG_CYAN}ВОДОСЧЁТЧИК{RESET}  {odometer(ch)} {BOLD}м³{RESET}"
        )
        out.append(
            f"  {FG_GRAY}объём: {RESET}{BOLD}{FG_WHITE}{ch.m3:.3f} м³{RESET}"
            f"{FG_GRAY}  ({fmt_l(ch.liters)} л)   расход: {RESET}"
            f"{BOLD}{FG_WHITE}{ch.flow_lpm:.2f} л/мин{RESET}"
        )
        out.append(f"  {FG_GRAY}{'─'*32}{RESET}  {sparkline(ch.history)}")
        out.append("")

    total   = sum(c.count for c in channels)
    total_l = sum(c.liters for c in channels)
    out.append(f"  {FG_GRAY}{'─'*W}{RESET}")
    out.append(
        f"  {DIM}Всего импульсов: {total}   "
        f"Всего: {total_l/1000.0:.3f} м³ ({fmt_l(total_l)} л)   "
        f"Ctrl+C — выход{RESET}"
    )

    # без полной очистки экрана — табло не мерцает
    print(HOME + "".join(line + EOL + "\n" for line in out) + EOS, end="", flush=True)

def summary(channels: list) -> list:
    """Итог по каналам после остановки — для сверки с показаниями УСПД."""
    out = []
    for ch in channels:
        line = (f"  Выход {ch.idx} (IO{ch.io_num}): {ch.count} имп., "
                f"{ch.m3:.3f} м³ ({fmt_l(ch.liters)} л)")
        if ch.note:
            line += f"  {FG_GRAY}— {ch.note}{RESET}"
        out.append(line)
        if ch.error:
            out.append(f"  {FG_RED}  ошибка GPIO: {ch.error}{RESET}")
    return out

# ─── Проверки ─────────────────────────────────────────────────────────────────
def check_freq(freq: float, duty: float):
    t_on  = (1.0 / freq) * duty
    t_off = (1.0 / freq) * (1.0 - duty)
    shortest, what = min((t_on, "ON"), (t_off, "OFF"))
    ms = shortest * 1000
    if shortest < MIN_LEVEL_FAST:
        print(f"\n{FG_YELLOW}⚠  {what} = {ms:.1f} мс < 50 мс — импульсы будут теряться!")
        print(f"   Минимум для УСПД RTU202 при опросе 20 Гц = 50 мс (и для импульса, и для паузы){RESET}\n")
        time.sleep(3)
    elif shortest < MIN_LEVEL_SLOW:
        print(f"\n{FG_YELLOW}⚠  {what} = {ms:.1f} мс < 500 мс")
        print("   При опросе УСПД 2 Гц (по умолч.) минимум = 500 мс (и для импульса, и для паузы)")
        print(f"   Переключите УСПД на 20 Гц, уменьшите частоту или выровняйте скважность{RESET}\n")
        time.sleep(3)

def load_state() -> dict:
    """Сохранённые показания: {номер IO: литры}. Нет файла или он битый — пусто."""
    try:
        with open(STATE_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return {int(io): float(v["liters"]) for io, v in data["counters"].items()}
    except FileNotFoundError:
        return {}
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as e:
        print(f"{FG_YELLOW}⚠  Не удалось прочитать {STATE_FILE}: {e} — начинаем с нуля{RESET}")
        return {}

def save_state(channels: list, saved: dict) -> str:
    """
    Записывает показания каналов (выходы, не участвующие в сессии, сохраняются как были).
    Пишем во временный файл и подменяем — при пропадании питания файл не побьётся.
    Возвращает текст ошибки или None.
    """
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        with open(STATE_FILE, encoding="utf-8") as f:
            counters = dict(json.load(f)["counters"])
    except (OSError, ValueError, TypeError, KeyError):
        counters = {str(io): {"liters": liters} for io, liters in sorted(saved.items())}
    for ch in channels:
        counters[str(ch.io_num)] = {"liters": round(ch.liters, 3), "updated": now}
    tmp = STATE_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"updated": now, "counters": counters}, f, ensure_ascii=False, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, STATE_FILE)
        return None
    except OSError as e:
        return str(e)

def gpio_hint() -> list:
    return [
        f"{FG_YELLOW}   Проверьте, что скрипт запущен от root и iolines инициализирован:",
        "     opkg install iolines-lt70",
        f"     /etc/init.d/iolines boot{RESET}",
    ]

# ─── Интерактивный ввод ───────────────────────────────────────────────────────
def _read_line(prompt: str, default_repr: str) -> str:
    """input() с поддержкой неинтерактивного запуска (EOF → значение по умолчанию)."""
    try:
        return input(prompt).strip()
    except EOFError:
        print(default_repr)
        return ""

def ask_float(label: str, default: float,
              gt: float = None, lo: float = None, hi: float = None) -> float:
    while True:
        raw = _read_line(f"  {FG_CYAN}{label}{RESET} [{fmt_l(default)}]: ", fmt_l(default))
        if raw == "":
            return default
        raw = raw.replace(",", ".")
        try:
            v = float(raw)
        except ValueError:
            print(f"{FG_YELLOW}   Введите число, напр. {default:g}{RESET}")
            continue
        if gt is not None and v <= gt:
            print(f"{FG_YELLOW}   Должно быть больше {gt:g}{RESET}")
            continue
        if lo is not None and v < lo:
            print(f"{FG_YELLOW}   Минимум {lo:g}{RESET}")
            continue
        if hi is not None and v > hi:
            print(f"{FG_YELLOW}   Максимум {hi:g}{RESET}")
            continue
        return v

def ask_yesno(label: str, default: bool = False) -> bool:
    opts = "[Д/н]" if default else "[д/Н]"
    while True:
        raw = _read_line(f"  {FG_CYAN}{label}{RESET} {opts}: ",
                         "д" if default else "н").lower()
        if raw == "":
            return default
        if raw[0] in ("y", "д", "1", "t", "+"):
            return True
        if raw[0] in ("n", "н", "0", "f", "-"):
            return False
        print(f"{FG_YELLOW}   Ответьте «д» или «н»{RESET}")

def ask_outputs(default: str = "1") -> list:
    while True:
        raw = _read_line(
            f"  {FG_CYAN}Выходы IO (1–9 через запятую){RESET} [{default}]: ", default)
        if raw == "":
            raw = default
        try:
            pins = [int(x.strip()) for x in raw.split(",") if x.strip() != ""]
        except ValueError:
            print(f"{FG_YELLOW}   Список чисел через запятую, напр. 1,2,3{RESET}")
            continue
        if not pins:
            print(f"{FG_YELLOW}   Нужен хотя бы один выход{RESET}")
            continue
        if any(not (1 <= p <= 9) for p in pins):
            print(f"{FG_YELLOW}   Допустимы пины 1–9{RESET}")
            continue
        if len(pins) != len(set(pins)):
            print(f"{FG_YELLOW}   Пины не должны повторяться{RESET}")
            continue
        return pins

def configure() -> dict:
    """Интерактивный мастер настройки. Возвращает словарь параметров."""
    print("\n".join(box("GPIO Pulse Generator — TELEOFIS LT70 → УСПД RTU202", 56)))
    print(f"{FG_GRAY}  Настройка по шагам. Enter — значение по умолчанию. Ctrl+C — выход.{RESET}\n")

    # не роутер (например, ПК) — реальный режим невозможен, включаем симуляцию без вопроса
    if os.path.exists(pd_path(1)):
        dry_run = ask_yesno("Режим симуляции (без записи в GPIO)?", default=False)
    else:
        dry_run = True
        print(f"  {FG_YELLOW}GPIO iolines не найдены ({pd_path(1)}) — режим симуляции.{RESET}")
    io_pins = ask_outputs("1")
    freq    = ask_float("Частота, Гц", 0.5, gt=0)
    duty    = ask_float("Скважность, %", 50.0, lo=1, hi=99) / 100.0
    lpp     = ask_float("Литров на импульс", 1.0, gt=0)

    saved = load_state()
    if any(p in saved for p in io_pins):
        print(f"\n  {FG_GRAY}Начальные показания, м³ (Enter — продолжить с сохранённых, 0 — сброс):{RESET}")
    else:
        print(f"\n  {FG_GRAY}Начальные показания счётчиков, м³ (Enter — 0):{RESET}")
    start_liters = [
        round(ask_float(f"Выход {i+1} (IO{p}), м³", saved.get(p, 0.0) / 1000.0, lo=0) * 1000.0, 3)
        for i, p in enumerate(io_pins)
    ]
    if dry_run:
        print(f"  {FG_GRAY}В симуляции показания не сохраняются.{RESET}")

    sync = False
    if len(io_pins) > 1:
        sync = ask_yesno("Синхронный старт всех выходов (без сдвига фазы)?", default=False)

    print()
    return {
        "dry_run": dry_run, "io_pins": io_pins, "freq": freq, "duty": duty,
        "lpp": lpp, "start_liters": start_liters, "sync": sync, "refresh": 0.15,
        "saved": saved,
    }

# ─── main ─────────────────────────────────────────────────────────────────────
class _Interrupted(Exception):
    pass

class _GpioInitFailed(Exception):
    pass

def _safe_print(*lines):
    """Печать, которая не мешает остановке, если терминал уже отвалился (обрыв SSH)."""
    try:
        print("\n".join(lines), flush=True)
    except (OSError, ValueError):
        pass

def _enable_windows_ansi():
    """Консоль Windows (PowerShell 5.1, cmd) по умолчанию не понимает ANSI — включаем VT-режим."""
    if os.name != "nt":
        return
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)          # STD_OUTPUT_HANDLE
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)   # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except (OSError, AttributeError):
        pass

def main():
    _enable_windows_ansi()
    if len(sys.argv) > 1 and sys.argv[1] in ("-v", "--version"):
        print(f"gpio_pulse_gen {__version__}")
        return

    try:
        cfg = configure()
    except KeyboardInterrupt:
        print(f"\n{FG_YELLOW}Отменено.{RESET}")
        return

    channels = [
        Channel(i + 1, pin, cfg["freq"], cfg["duty"], cfg["dry_run"],
                cfg["lpp"], cfg["start_liters"][i])
        for i, pin in enumerate(cfg["io_pins"])
    ]

    # SIGHUP — обрыв SSH: без обработчика процесс умер бы, оставив пин замкнутым
    stop_signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, "SIGHUP"):
        stop_signals.append(signal.SIGHUP)

    def on_signal(sig, frame):
        raise _Interrupted()

    for s in stop_signals:
        signal.signal(s, on_signal)

    exit_code = 0
    started = False
    try:
        check_freq(cfg["freq"], cfg["duty"])

        for ch in channels:
            try:
                ch.pin.open()
            except GpioError as e:
                _safe_print(f"{FG_YELLOW}⚠  IO{ch.io_num}: {e}{RESET}", *gpio_hint())
                raise _GpioInitFailed()

        # равномерный сдвиг фазы между выходами: при N каналах i-й сдвинут на i/N периода
        period = channels[0].period
        n = len(channels)
        print(CLEAR + "\033[?25l", end="", flush=True)
        start_time = time.monotonic()
        started = True
        for i, ch in enumerate(channels):
            delay = 0.0 if cfg["sync"] else (period / n * i)
            ch.start(phase_delay=delay)

        last_save = time.monotonic()
        while not any(ch.error for ch in channels):
            draw(channels, cfg["dry_run"], start_time)
            time.sleep(cfg["refresh"])
            if not cfg["dry_run"] and time.monotonic() - last_save >= STATE_SAVE_INTERVAL:
                save_state(channels, cfg["saved"])
                last_save = time.monotonic()
        exit_code = 1
    except _Interrupted:
        pass
    except _GpioInitFailed:
        exit_code = 1
    except OSError:
        # терминал недоступен (обрыв SSH) — просто останавливаемся
        pass
    finally:
        # повторный Ctrl+C/обрыв связи не должен прервать сброс пинов
        for s in stop_signals:
            signal.signal(s, signal.SIG_IGN)
        if started:
            _safe_print("\033[?25h", f"\n{FG_YELLOW}Остановка...{RESET}")
        for ch in channels:
            ch.stop()
        if started:
            lines = summary(channels)
            if not cfg["dry_run"]:
                err = save_state(channels, cfg["saved"])
                lines.append(f"  {FG_GRAY}Показания сохранены: {STATE_FILE}{RESET}" if err is None
                             else f"{FG_RED}  Не удалось сохранить показания: {err}{RESET}")
            done = (f"{FG_GREEN}Пины возвращены в исходный режим. Готово.{RESET}"
                    if not cfg["dry_run"] else f"{FG_GREEN}Готово.{RESET}")
            _safe_print(*lines, done)
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
