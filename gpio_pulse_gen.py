#!/usr/bin/env python3
"""
GPIO Pulse Generator — TELEOFIS LT70
Генератор импульсов для УСПД RTU202 (режим сухой контакт)

Использует нативный sysfs-интерфейс iolines LT70:
  /dev/pu{N}/direction  — pull-up
  /dev/pd{N}/direction  — pull-down (наш выход OC)

Режимы пина (iolines):
  mode1: pu=low, pd=low  → АЦП (пассивный)
  mode2: pu=high,pd=low  → Сухой контакт (ВХОД)
  mode3: pu=low, pd=high → Открытый коллектор (ВЫХОД) ← используем

Подключение к УСПД RTU202:
  IOx роутера (mode3) → I{x}+ УСПД
  GND роутера         → I{x}- УСПД

Использование:
  python3 gpio_pulse_gen.py [--freq FREQ] [--duty DUTY] [--outputs 1,2,...]

Примеры:
  python3 gpio_pulse_gen.py --dry-run                    # симуляция без железа, IO1
  python3 gpio_pulse_gen.py --freq 0.5                   # IO1 (по умолчанию)
  python3 gpio_pulse_gen.py --freq 0.5 --outputs 1,2,3,4
  python3 gpio_pulse_gen.py --freq 1 --outputs 1,5,7
  python3 gpio_pulse_gen.py --freq 0.5 --outputs 1,2 --sync

Ограничения УСПД RTU202:
  Частота опроса 2 Гц  (по умолч.) → минимальный импульс 500 мс → max 1 Гц
  Частота опроса 20 Гц             → минимальный импульс  50 мс → max 10 Гц
"""

import sys
import os
import time
import signal
import argparse
import threading
from datetime import datetime

__version__ = "1.1.0"

# ─── ANSI коды ────────────────────────────────────────────────────────────────
RESET    = "\033[0m"
BOLD     = "\033[1m"
DIM      = "\033[2m"
CLEAR    = "\033[2J\033[H"

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

# ─── Sysfs GPIO (iolines LT70) ────────────────────────────────────────────────

def io_index(io_num: int) -> int:
    """IO1 → 0, IO2 → 1, ..., IO9 → 8"""
    return io_num - 1

def pd_path(io_num: int) -> str:
    return f"/dev/pd{io_index(io_num)}/direction"

def pu_path(io_num: int) -> str:
    return f"/dev/pu{io_index(io_num)}/direction"

_warnings = []
def _warn(msg: str):
    _warnings.append(msg)
    if len(_warnings) > 5:
        _warnings.pop(0)

def _write(path: str, val: str):
    try:
        with open(path, 'w') as f:
            f.write(val)
    except OSError as e:
        _warn(f"{path}: {e}")

def io_init_oc(io_num: int, dry_run: bool):
    """mode3: pu=low, pd=high — выход открытый коллектор"""
    if dry_run:
        return
    _write(pu_path(io_num), "low")
    _write(pd_path(io_num), "high")

def io_set(io_num: int, value: bool, dry_run: bool):
    """
    Управляем только pd:
      True  → pd=high → линия притянута к GND → УСПД видит замкнутый контакт
      False → pd=low  → линия свободна         → УСПД видит разомкнутый контакт
    """
    if dry_run:
        return
    _write(pd_path(io_num), "high" if value else "low")

def io_reset(io_num: int, dry_run: bool):
    """Сбрасываем в mode1 (пассивный) при выходе"""
    if dry_run:
        return
    _write(pu_path(io_num), "low")
    _write(pd_path(io_num), "low")

# ─── Канал ────────────────────────────────────────────────────────────────────
class Channel:
    def __init__(self, idx: int, io_num: int, freq: float, duty: float,
                 dry_run: bool, lpp: float = 1.0):
        self.idx     = idx
        self.io_num  = io_num
        self.freq    = freq
        self.duty    = duty
        self.dry_run = dry_run
        self.lpp     = lpp          # литров на импульс

        self.count   = 0
        self.state   = False
        self.running = False
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
        """Накопленный объём, литры"""
        return self.count * self.lpp

    @property
    def m3(self) -> float:
        """Накопленный объём, кубометры"""
        return self.liters / 1000.0

    @property
    def flow_lpm(self) -> float:
        """Текущий расход, л/мин (freq имп/с × лит/имп × 60)"""
        return self.freq * self.lpp * 60.0

    def _run(self):
        self.running = True
        io_init_oc(self.io_num, self.dry_run)

        while not self._stop.is_set():
            # ON
            self.state = True
            self.history.append(True)
            self.history.pop(0)
            io_set(self.io_num, True, self.dry_run)
            self._stop.wait(timeout=self.t_on)
            if self._stop.is_set():
                break

            # OFF
            self.state = False
            self.count += 1
            self.history.append(False)
            self.history.pop(0)
            io_set(self.io_num, False, self.dry_run)
            self._stop.wait(timeout=self.t_off)

        self.running = False
        self.state = False
        io_reset(self.io_num, self.dry_run)

    def start(self, phase_delay: float = 0.0):
        def _delayed():
            if phase_delay > 0:
                self._stop.wait(timeout=phase_delay)
            if self._stop.is_set():
                return
            self._run()
        self._stop.clear()
        self._thread = threading.Thread(target=_delayed, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

# ─── UI ───────────────────────────────────────────────────────────────────────
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
    now = time.time()
    elapsed = now - start_time
    hh = int(elapsed // 3600)
    mm = int((elapsed % 3600) // 60)
    ss = int(elapsed % 60)
    ts = datetime.now().strftime("%H:%M:%S")

    W = 60

    out = [CLEAR]
    out.append(f"{BOLD}{FG_CYAN}╔{'═'*W}╗{RESET}")
    out.append(f"{BOLD}{FG_CYAN}║  GPIO Pulse Generator — TELEOFIS LT70{' '*(W-38)}║{RESET}")
    out.append(f"{BOLD}{FG_CYAN}╚{'═'*W}╝{RESET}")

    mode = f"{FG_YELLOW}СИМУЛЯЦИЯ{RESET}" if dry_run else f"{FG_GREEN}РЕАЛЬНЫЙ GPIO{RESET}"
    out.append(f"  Режим: {mode}   {FG_GRAY}{ts}   uptime {hh:02d}:{mm:02d}:{ss:02d}{RESET}")
    out.append("")

    for ch in channels:
        idx  = io_index(ch.io_num)
        out.append(
            f"  {BOLD}{FG_CYAN}Выход {ch.idx}{RESET}  "
            f"IO{ch.io_num}  "
            f"{FG_GRAY}/dev/pd{idx}/direction{RESET}"
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
            f"{FG_GRAY}  ({ch.liters:.0f} л)   расход: {RESET}"
            f"{BOLD}{FG_WHITE}{ch.flow_lpm:.2f} л/мин{RESET}"
        )
        out.append(f"  {FG_GRAY}{'─'*32}{RESET}  {sparkline(ch.history)}")
        out.append("")

    if _warnings:
        out.append(f"  {FG_YELLOW}⚠  " + "   ".join(_warnings[-2:]) + RESET)
        out.append("")

    total   = sum(c.count for c in channels)
    total_l = sum(c.liters for c in channels)
    out.append(f"  {FG_GRAY}{'─'*W}{RESET}")
    out.append(
        f"  {DIM}Всего импульсов: {total}   "
        f"Всего: {total_l/1000.0:.3f} м³ ({total_l:.0f} л)   "
        f"Ctrl+C — выход{RESET}"
    )

    print("\n".join(out), end="", flush=True)

# ─── Проверки ─────────────────────────────────────────────────────────────────
def check_freq(freq: float, duty: float):
    t_on_ms = (1.0 / freq) * duty * 1000
    if t_on_ms < 50:
        print(f"\n{FG_YELLOW}⚠  ON = {t_on_ms:.1f} мс < 50 мс — импульсы будут теряться!")
        print(f"   Минимум для УСПД RTU202 при опросе 20 Гц = 50 мс{RESET}\n")
        time.sleep(3)
    elif t_on_ms < 500:
        print(f"\n{FG_YELLOW}⚠  ON = {t_on_ms:.1f} мс < 500 мс")
        print("   При опросе УСПД 2 Гц (по умолч.) минимум = 500 мс")
        print(f"   Переключите УСПД на 20 Гц или уменьшите --freq{RESET}\n")
        time.sleep(3)

def check_sysfs(io_num: int, dry_run: bool) -> bool:
    if dry_run:
        return True
    for path in [pu_path(io_num), pd_path(io_num)]:
        if not os.path.exists(path):
            print(f"{FG_YELLOW}⚠  Не найден: {path}")
            print("   Установите пакет и инициализируйте:")
            print("     opkg install iolines-lt70")
            print(f"     /etc/init.d/iolines boot{RESET}")
            return False
    return True

# ─── main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="GPIO Pulse Generator — TELEOFIS LT70 → УСПД RTU202",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--version", action="version", version=f"gpio_pulse_gen {__version__}")
    parser.add_argument("--freq",    type=float, default=0.5,
                        help="Частота Гц (по умолч. 0.5 = 1 имп/2с)")
    parser.add_argument("--duty",    type=float, default=50.0,
                        help="Скважность %% (по умолч. 50)")
    parser.add_argument("--liters-per-pulse", type=float, default=1.0,
                        dest="lpp",
                        help="Вес импульса счётчика воды, литров (по умолч. 1.0)")
    parser.add_argument("--outputs", type=str,   default="1",
                        help="IO пины роутера через запятую (1–9, по умолч. 1). Пример: 1,2,3,4")
    parser.add_argument("--sync",    action="store_true",
                        help="Синхронный старт всех выходов (без сдвига фазы)")
    parser.add_argument("--refresh", type=float, default=0.15,
                        help="Частота обновления экрана сек (по умолч. 0.15)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Симуляция без записи в GPIO")
    args = parser.parse_args()

    if args.freq <= 0:
        sys.exit("Ошибка: --freq должен быть > 0")
    if not (1 <= args.duty <= 99):
        sys.exit("Ошибка: --duty от 1 до 99")
    if args.lpp <= 0:
        sys.exit("Ошибка: --liters-per-pulse должен быть > 0")

    # парсим список пинов
    try:
        io_pins = [int(x.strip()) for x in args.outputs.split(",")]
    except ValueError:
        sys.exit("Ошибка: --outputs должен быть списком чисел через запятую, например: 1,2,3,4")

    if not io_pins:
        sys.exit("Ошибка: --outputs не может быть пустым")
    for p in io_pins:
        if not (1 <= p <= 9):
            sys.exit(f"Ошибка: пин {p} вне диапазона 1–9")
    if len(io_pins) != len(set(io_pins)):
        sys.exit("Ошибка: в --outputs не должно быть повторяющихся пинов")

    duty = args.duty / 100.0

    # Каналы создаём заранее — до возможной блокирующей паузы в check_freq,
    # чтобы обработчики сигналов уже могли корректно их остановить.
    channels = [Channel(i + 1, pin, args.freq, duty, args.dry_run, args.lpp)
                for i, pin in enumerate(io_pins)]

    _shutting_down = threading.Event()

    def shutdown(sig=None, frame=None):
        # Идемпотентно: при Ctrl+C обработчик и finally не должны дублироваться.
        if _shutting_down.is_set():
            return
        _shutting_down.set()
        print("\033[?25h", end="", flush=True)
        print(f"\n{FG_YELLOW}Остановка...{RESET}", flush=True)
        for ch in channels:
            ch.stop()
        print(f"{FG_GREEN}Пины сброшены в mode1. Готово.{RESET}")
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    check_freq(args.freq, duty)

    for io_num in io_pins:
        if not check_sysfs(io_num, args.dry_run):
            sys.exit(1)

    # равномерный сдвиг фазы между выходами: при N каналах i-й сдвинут на i/N периода
    period = channels[0].period
    n = len(channels)
    start_time = time.time()
    for i, ch in enumerate(channels):
        delay = 0.0 if args.sync else (period / n * i)
        ch.start(phase_delay=delay)

    print("\033[?25l", end="", flush=True)

    try:
        while True:
            draw(channels, args.dry_run, start_time)
            time.sleep(args.refresh)
    finally:
        print("\033[?25h", end="", flush=True)
        shutdown()

if __name__ == "__main__":
    main()
