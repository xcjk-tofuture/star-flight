# U8g2 graphics subset

Source: https://github.com/olikraus/u8g2
Pinned commit: `d6c8499c5f2707cac8eccd09fd8f677d12b17977`.

The selected C graphics sources and generated fonts are copied unchanged.
`LICENSE` retains upstream license text; selected Misc Fixed fonts carry their
public domain notices in each font source. No Arduino, HAL, RTOS or upstream
display drivers are used. The application supplies display geometry, a static
frame buffer and its own transport. Fonts are compiled through gui_font_assets.c.
