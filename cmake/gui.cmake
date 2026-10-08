list(APPEND FIRMWARE_SOURCES
  firmware/gui/gui_tuning.c
  firmware/gui/gui_canvas.c
  firmware/gui/gui_menu.c
  firmware/gui/gui_font_assets.c
  firmware/gui/gui_plot.c
  firmware/gui/gui_scene.c
  firmware/gui/gui_dashboard.c
  firmware/platform/stm32/gui_display_port.c
  firmware/third_party/u8g2/csrc/u8g2_setup.c
  firmware/third_party/u8g2/csrc/u8g2_buffer.c
  firmware/third_party/u8g2/csrc/u8g2_hvline.c
  firmware/third_party/u8g2/csrc/u8g2_line.c
  firmware/third_party/u8g2/csrc/u8g2_box.c
  firmware/third_party/u8g2/csrc/u8g2_circle.c
  firmware/third_party/u8g2/csrc/u8g2_polygon.c
  firmware/third_party/u8g2/csrc/u8g2_font.c
  firmware/third_party/u8g2/csrc/u8g2_intersection.c
  firmware/third_party/u8g2/csrc/u8g2_ll_hvline.c
  firmware/third_party/u8g2/csrc/u8g2_bitmap.c
  firmware/third_party/u8g2/csrc/u8x8_8x8.c
)
list(APPEND FIRMWARE_INCLUDES firmware/gui firmware/third_party/u8g2/csrc)
