"""Read back the FPGA parameter registers and compare with the expected set."""
from __future__ import annotations

import sys
import time

import serial

PORT = "COM13"
EXPECT = {
    "kp": 600, "zl": 2038, "zh": 2040, "pkp": 200, "pkd": 2000,
    "swp": 100, "mlim": 100, "vlim": 25, "pos": 1, "enc": 0, "dir": 0,
}


def main() -> int:
    sp = serial.Serial(PORT, 115200, timeout=0.15)
    sp.reset_input_buffer()
    buf = b""

    def drain() -> list[list[int]]:
        nonlocal buf
        buf += sp.read(2048)
        rows = []
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            parts = line.decode("ascii", "replace").strip().split(",")
            if len(parts) != 8:
                continue
            try:
                rows.append([int(p) for p in parts])
            except ValueError:
                pass
        return rows

    sp.write(b"tel=1;")
    sp.flush()
    time.sleep(0.5)
    drain()
    rows = drain()
    for _ in range(5):
        if rows:
            break
        time.sleep(0.2)
        rows = drain()
    if not rows:
        print("没有收到遥测数据")
        sp.close()
        return 1

    r = rows[-1]
    got = {
        "kp": r[0], "zl": r[1], "zh": r[2], "pkp": r[3], "pkd": r[4],
        "swp": r[5], "mlim": r[6] // 256, "vlim": r[6] % 256,
        "pos": (r[7] >> 3) & 1, "enc": (r[7] >> 2) & 1,
        "dir": (r[7] >> 1) & 1, "en": r[7] & 1,
    }

    print("=" * 56)
    print("  FPGA 内部寄存器回读   vs   期望值")
    print("=" * 56)
    ok = True
    for k, exp in EXPECT.items():
        val = got[k]
        mark = "OK " if val == exp else "!! "
        if val != exp:
            ok = False
        print(f"  {mark}{k:5s} 内部 = {val:6d}   期望 = {exp:6d}")
    print("-" * 56)
    print(f"  en = {got['en']}   (1 = 正在运行)")
    print("=" * 56)
    print("  全部一致" if ok else "  有项目不一致（见 !! 标记）")

    sp.write(b"tel=0;")
    sp.flush()
    time.sleep(0.2)
    sp.close()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
