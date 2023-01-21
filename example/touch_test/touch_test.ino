/* Please make sure your touch IC model. */
#define TOUCH_MODULES_CST_MUTUAL
// #define TOUCH_MODULES_CST_SELF

#include "Arduino.h"
#include "OneButton.h"
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "TouchLib.h"
#include "Wire.h"
#include "img_logo.h"
#include "pin_config.h"

/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the
 * switch macro. */
#define LCD_MODULE_CMD_1

#warning Please confirm that you have purchased a display screen with a touch chip, otherwise the touch routine cannot be implemented.
#if defined(TOUCH_MODULES_CST_MUTUAL)
TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, CTS328_SLAVE_ADDRESS, PIN_TOUCH_RES);
#elif defined(TOUCH_MODULES_CST_SELF)
TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, CTS820_SLAVE_ADDRESS, PIN_TOUCH_RES);
#endif

#define TOUCH_GET_FORM_INT 0

TFT_eSPI tft = TFT_eSPI();
bool flags_sleep = false;
#if TOUCH_GET_FORM_INT
bool get_int = false;
#endif
OneButton button(PIN_BUTTON_1, true);
#if defined(LCD_MODULE_CMD_1)
typedef struct {
  uint8_t cmd;
  uint8_t data[14];
  uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    // {0x3A, {0X06}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif
void setup() {
  gpio_hold_dis((gpio_num_t)PIN_TOUCH_RES);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  pinMode(PIN_TOUCH_RES, OUTPUT);
  digitalWrite(PIN_TOUCH_RES, LOW);
  delay(500);
  digitalWrite(PIN_TOUCH_RES, HIGH);

  Serial.begin(115200);
  Serial.println("Hello T-Display-S3");

  tft.begin();
#if defined(LCD_MODULE_CMD_1)
  for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
    tft.writecommand(lcd_st7789v[i].cmd);
    for (int j = 0; j < lcd_st7789v[i].len & 0x7f; j++) {
      tft.writedata(lcd_st7789v[i].data[j]);
    }

    if (lcd_st7789v[i].len & 0x80) {
      delay(120);
    }
  }
#endif
  tft.setRotation(3);
  tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);

  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  ledcWrite(0, 255);

  delay(2000);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
  if (!touch.init()) {
    Serial.println("Touch IC not found");
  }

  button.attachClick([] { flags_sleep = 1; });
  tft.drawString("Press the BOT button to go", 0, 15);
  tft.drawString(" to sleep", 0, 40);
#if TOUCH_GET_FORM_INT
  attachInterrupt(
      PIN_TOUCH_INT, [] { get_int = true; }, FALLING);
#endif
}

void loop() {
  char str_buf[100];
  static uint8_t last_finger;
#if TOUCH_GET_FORM_INT
  if (get_int) {
    get_int = 0;
    touch.read();
#else
  if (touch.read()) {
#endif
    uint8_t n = touch.getPointNum();
    sprintf(str_buf, "finger num :%d \r\n", n);
    tft.drawString(str_buf, 0, 15);
    for (uint8_t i = 0; i < n; i++) {
      TP_Point t = touch.getPoint(i);
      sprintf(str_buf, "x:%04d y:%04d p:%04d \r\n", t.x, t.y, t.pressure);
      tft.drawString(str_buf, 0, 35 + (20 * i));
    }
    if (last_finger > n) {
      tft.fillRect(0, 55, 320, 100, TFT_BLACK);
    }
    last_finger = n;
  } else {
    tft.fillScreen(TFT_BLACK);
  }

  button.tick();
  if (flags_sleep) {
    flags_sleep = false;
    tft.fillScreen(TFT_BLACK);
    tft.drawString("about to go to sleep", 0, 15);
    touch.enableSleep();
    delay(2000);
    digitalWrite(PIN_POWER_ON, LOW);
    gpio_hold_en((gpio_num_t)PIN_TOUCH_RES);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_TOUCH_INT, 0);
    esp_deep_sleep_start();
  }
  delay(1);
}