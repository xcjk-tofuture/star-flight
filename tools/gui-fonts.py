"""Build compact 12px/16px Chinese font assets from a pinned U8g2 checkout.

The generated glyphs retain WenQuanYi's GPLv2 font-embedding exception.
Only ASCII and non-ASCII characters present in the GUI source/charset are kept.
"""
from pathlib import Path
import argparse
import subprocess
import shutil

p = argparse.ArgumentParser()
p.add_argument("upstream", type=Path, help="U8g2 source checkout")
p.add_argument("--gcc", default=shutil.which("gcc") or "C:/MinGW/bin/gcc.exe")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
destination = root / "firmware/gui/fonts"
destination.mkdir(parents=True, exist_ok=True)
work = root / "build/font-tool"
work.mkdir(parents=True, exist_ok=True)
tool = work / "bdfconv.exe"
source = a.upstream / "tools/font/bdfconv"
subprocess.run([a.gcc, "-std=gnu99", "-O2", *map(str, sorted(source.glob("*.c"))), "-o", str(tool)], check=True)
characters = set(range(32, 127))
for file in [*sorted((root / "firmware/gui").glob("*.c")), *sorted((root / "firmware/gui").glob("*.h")),
             destination / "characters.txt"]:
    if file.exists():
        characters.update(ord(c) for c in file.read_text(encoding="utf-8") if ord(c) >= 128)
(destination / "codepoints.txt").write_text("\n".join(f"U+{c:04X} {chr(c)}" for c in sorted(characters)), encoding="utf-8", newline="\n")

def subset(source, target):
    header, blocks = [], {}
    current = None
    encoding = None
    before_chars = True
    with source.open(encoding="utf-8") as file:
        for line in file:
            if before_chars:
                if line.startswith("CHARS "): before_chars = False
                else: header.append(line)
            elif line.startswith("STARTCHAR "):
                current, encoding = [line], None
            elif current is not None:
                current.append(line)
                if line.startswith("ENCODING "): encoding = int(line.split()[1])
                if line.startswith("ENDCHAR"):
                    if encoding in characters: blocks[encoding] = current
                    current = None
    missing = characters - blocks.keys()
    if missing:
        raise RuntimeError(f"Missing glyphs in {source.name}: " + ", ".join(f"U+{v:04X}" for v in sorted(missing)))
    with target.open("w", encoding="utf-8", newline="\n") as out:
        out.writelines(line.rstrip()+"\n" for line in header)
        out.write(f"CHARS {len(blocks)}\n")
        for codepoint in sorted(blocks): out.writelines(line.rstrip()+"\n" for line in blocks[codepoint])
        out.write("ENDFONT\n")

for size, bdf in [(12, "wenquanyi_9pt.bdf"), (16, "wenquanyi_12pt.bdf")]:
    font_source = destination / f"wqy{size}-subset.bdf"
    subset(a.upstream / "tools/font/bdf" / bdf, font_source)
    output = destination / f"gui_font_cn{size}.c"
    symbol = f"gui_font_cn{size}"
    mapping = ",".join(str(c) for c in sorted(characters))
    subprocess.run([str(tool), "-f", "1", "-b", "0", "-m", mapping,
                    str(font_source), "-n", symbol, "-o", str(output)], check=True)
    notice = """/* Generated WenQuanYi Bitmap Song subset for StarFlight.
 * Copyright (C) 2004-2010 WenQuanYi Project, Board of Trustees and Qianqian Fang.
 * License: GPL v2 with font embedding exception. See README.md and source BDF.
 * Derived from U8g2 d6c8499c5f2707cac8eccd09fd8f677d12b17977.
 */
"""
    generated="\n".join(line.rstrip() for line in output.read_text(encoding="utf-8").splitlines())+"\n"
    output.write_text(notice + generated, encoding="utf-8", newline="\n")
print(f"Built 12px and 16px font subsets with {len(characters)} glyphs: {destination}")
