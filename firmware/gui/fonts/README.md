# Chinese UI font subsets

12px and 16px glyphs derive from WenQuanYi Bitmap Song 0.9.9.8 in the pinned
U8g2 checkout `d6c8499c5f2707cac8eccd09fd8f677d12b17977`:

- `tools/font/bdf/wenquanyi_9pt.bdf` (12px)
- `tools/font/bdf/wenquanyi_12pt.bdf` (16px)

Copyright (C) 2004-2010 WenQuanYi Project, Board of Trustees and Qianqian Fang.
The font's stated license is GPL v2 **with font embedding exception**.
This applies to these font glyphs, distinct from U8g2's BSD graphics core.
Original font metadata and notices are preserved in each subset BDF.

Source and font-license reference:
https://github.com/olikraus/u8g2/wiki/fntgrpwqy
https://wenq.org/wqy2/index.cgi?BitmapSong

The modified subsets and generated C arrays are supplied together. Rebuild with:

```powershell
python tools/gui-fonts.py PATH_TO_PINNED_U8G2_CHECKOUT
```

`characters.txt` supplies shared UI vocabulary. The generator also scans GUI
sources for non-ASCII characters; `codepoints.txt` records the complete subset.
Adding labels requires regenerating both fonts before building the firmware.
