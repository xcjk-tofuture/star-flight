"""Convert the UI exporter's PBM output to OLED previews (requires Pillow)."""
from pathlib import Path
import argparse
from PIL import Image, ImageOps, ImageDraw, ImageFont

parser = argparse.ArgumentParser()
parser.add_argument("input", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

def screen(path, scale=4):
    # PBM's one bits are black; OLED's set pixels emit light.
    img = ImageOps.invert(Image.open(path).convert("L"))
    return img.resize((128 * scale, 64 * scale), Image.Resampling.NEAREST).convert("RGB")

for path in args.input.glob("*.pbm"):
    screen(path).save(args.output / (path.stem + ".png"))

try:
    font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)
    title = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 26)
except OSError:
    font = title = ImageFont.load_default()

panels = [("menu-main", "Main menu / Chinese 12px / 3 rows"), ("menu-main-next", "Highlight selection"),
          ("overview", "01 Overview"), ("attitude-3d", "02 3D attitude"),
          ("trend-attitude", "03 Trend chart"), ("sensors", "04 Sensors"),
          ("menu-calibration", "Calibration menu"), ("menu-flash-warning", "Readable status message")]
montage = Image.new("RGB", (1112, 1320), "#101318")
draw = ImageDraw.Draw(montage)
draw.text((28, 14), "STAR FLIGHT / 128 x 64 OLED", fill="#f2f4f7", font=title)
draw.text((28, 51), "Actual firmware renderer / simulated data / pixels enlarged 4x", fill="#adb4bd", font=font)
for i, (name, caption) in enumerate(panels):
    x = 28 + (i % 2) * 548
    y = 95 + (i // 2) * 303
    draw.text((x, y), caption, fill="#c3cbd5", font=font)
    draw.rectangle((x - 1, y + 29, x + 513, y + 287), outline="#3b4350")
    montage.paste(screen(args.input / (name + ".pbm")), (x, y + 30))
montage.save(args.output / "pages.png")
frames = [screen(args.input / f"pose-{i:02d}.pbm") for i in range(48)]
frames[0].save(args.output / "attitude.gif", save_all=True, append_images=frames[1:],
               duration=100, loop=0, disposal=2)
print(f"OLED gallery and pose animation: {args.output}")
