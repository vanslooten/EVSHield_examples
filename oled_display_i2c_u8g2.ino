/*
 * https://home.et.utwente.nl/slootenvanf/2019/04/04/displays/
 * 
 * Connections (in combi with EVShield)
 */
#include <U8g2lib.h>

// software i2c:
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(7, 8, U8X8_PIN_NONE);
// hardware i2c:
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

void setup() {
  u8x8.begin();
  u8x8.setPowerSave(0);

  u8x8.setFont(u8x8_font_pxplusibmcgathin_f);
  u8x8.drawString(0,0,"Hello World v1!");
  u8x8.drawString(0,3,"1234567890123456");
  u8x8.drawString(0,7,"Hello World 209!");
}

void loop() {
}
