// Weather icons by Zlatko Najdenovski, https://www.iconfinder.com/zlaten .

#define temperature_width 32
#define temperature_height 32
static const unsigned char temperature_bits[] PROGMEM = {
   0x00, 0xf8, 0x07, 0x00, 0x00, 0x3c, 0x07, 0x00, 0x00, 0x0c, 0x0e, 0x00,
   0x00, 0x0e, 0x0e, 0x00, 0x00, 0x0e, 0x0e, 0x00, 0x00, 0x0e, 0x0e, 0x00,
   0x00, 0x0e, 0x0e, 0x00, 0x00, 0x0e, 0x0e, 0x00, 0x00, 0x0e, 0x0e, 0x00,
   0x00, 0x0e, 0x0e, 0x00, 0x00, 0x0e, 0x0e, 0x00, 0x00, 0x0e, 0x0e, 0x00,
   0x00, 0xee, 0x0e, 0x00, 0x00, 0xee, 0x0e, 0x00, 0x00, 0xee, 0x0e, 0x00,
   0x00, 0xee, 0x0e, 0x00, 0x00, 0xee, 0x0e, 0x00, 0x00, 0xee, 0x1e, 0x00,
   0x00, 0xe7, 0x1c, 0x00, 0x80, 0xf3, 0x39, 0x00, 0x80, 0xfb, 0x3b, 0x00,
   0x80, 0xf9, 0x73, 0x00, 0x80, 0xf9, 0x77, 0x00, 0x80, 0xf9, 0x77, 0x00,
   0x80, 0xfb, 0x33, 0x00, 0x80, 0xf3, 0x39, 0x00, 0x00, 0xc7, 0x3c, 0x00,
   0x00, 0x0f, 0x1e, 0x00, 0x00, 0xfe, 0x0f, 0x00, 0x00, 0xf8, 0x07, 0x00,
   0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define humidity_32_width 32
#define humidity_32_height 32
static const unsigned char humidity_32_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xc0, 0x03,
   0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 0x78, 0x0f, 0x00, 0x00, 0x38, 0x1c,
   0x00, 0x00, 0x1c, 0x3c, 0x00, 0x00, 0x1e, 0x38, 0x00, 0x00, 0x0e, 0x70,
   0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0,
   0x00, 0x00, 0x07, 0xe8, 0x80, 0x01, 0x07, 0xec, 0xc0, 0x03, 0x07, 0xe6,
   0xf0, 0x07, 0x0e, 0x73, 0x78, 0x0f, 0x1e, 0x78, 0x38, 0x1c, 0xfc, 0x3f,
   0x1c, 0x3c, 0xf8, 0x1f, 0x1e, 0x38, 0xe0, 0x07, 0x0e, 0x70, 0x00, 0x00,
   0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00,
   0x07, 0xe8, 0x00, 0x00, 0x07, 0xec, 0x00, 0x00, 0x07, 0xe6, 0x00, 0x00,
   0x0e, 0x73, 0x00, 0x00, 0x1e, 0x78, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00,
   0xf8, 0x1f, 0x00, 0x00, 0xe0, 0x07, 0x00, 0x00 };

#define humidity_abs_32_width 32
#define humidity_abs_32_height 32
static const unsigned char humidity_abs_32_bits[] PROGMEM = {
   0x00, 0x80, 0x01, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0xe0, 0x07, 0x00,
   0x00, 0xf0, 0x0f, 0x00, 0x00, 0x78, 0x1e, 0x00, 0x00, 0x3c, 0x3c, 0x00,
   0x00, 0x1e, 0x78, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x80, 0x07, 0xe0, 0x01,
   0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x01, 0x80, 0x03, 0xe0, 0x00, 0x00, 0x07,
   0xf0, 0x00, 0x40, 0x0f, 0x70, 0x00, 0xc0, 0x0e, 0x38, 0x00, 0x80, 0x1c,
   0x38, 0x00, 0x80, 0x1d, 0x38, 0x00, 0x00, 0x1d, 0x18, 0x00, 0x00, 0x19,
   0x18, 0x00, 0x00, 0x19, 0x18, 0x00, 0x00, 0x19, 0x18, 0x00, 0x00, 0x1d,
   0x38, 0x00, 0x00, 0x1d, 0x38, 0x00, 0x80, 0x1d, 0x70, 0x00, 0xc0, 0x0e,
   0x70, 0x00, 0x40, 0x0e, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x01, 0x80, 0x07,
   0xc0, 0x03, 0xc0, 0x03, 0x80, 0x0f, 0xf0, 0x01, 0x00, 0xff, 0xff, 0x00,
   0x00, 0xfc, 0x3f, 0x00, 0x00, 0xf0, 0x0f, 0x00 };

#define pressure_32_width 32
#define pressure_32_height 32
static const unsigned char pressure_32_bits[] PROGMEM = {
   0x00, 0xf0, 0x0f, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x80, 0xff, 0xff, 0x01,
   0xc0, 0x87, 0xf1, 0x03, 0xe0, 0x81, 0x81, 0x07, 0xf0, 0x00, 0x00, 0x0f,
   0x78, 0x00, 0x00, 0x1e, 0x3c, 0x00, 0x00, 0x3c, 0x1c, 0x00, 0x00, 0x38,
   0x0e, 0x00, 0x00, 0x70, 0x0e, 0x00, 0x30, 0x70, 0x06, 0x00, 0x3c, 0x70,
   0x07, 0x00, 0x1f, 0xe0, 0x07, 0xc0, 0x19, 0xe0, 0x07, 0x70, 0x0c, 0xe0,
   0x1f, 0x1c, 0x0c, 0xf8, 0x1f, 0x07, 0x06, 0xf8, 0x07, 0x3e, 0x06, 0xe0,
   0x07, 0x30, 0x02, 0xe0, 0x07, 0x60, 0x03, 0xe0, 0x06, 0xc0, 0x01, 0x70,
   0x0e, 0xc0, 0x01, 0x70, 0x0e, 0xc0, 0x00, 0x70, 0x1c, 0x80, 0x00, 0x38,
   0x3c, 0x00, 0x00, 0x3c, 0x78, 0x00, 0x00, 0x1e, 0xf0, 0x00, 0x00, 0x0f,
   0xe0, 0x81, 0x81, 0x07, 0xc0, 0x8f, 0xf1, 0x03, 0x80, 0xff, 0xff, 0x01,
   0x00, 0xfe, 0x7f, 0x00, 0x00, 0xf0, 0x0f, 0x00 };

#define home_inside_width 16
#define home_inside_height 16
static const unsigned char home_inside_bits[] PROGMEM = {
   0x00, 0x00, 0x80, 0x01, 0x40, 0x02, 0x20, 0x04, 0x10, 0x08, 0x08, 0x10,
   0x04, 0x20, 0x06, 0x60, 0x04, 0x20, 0x04, 0x20, 0xe4, 0x20, 0xe4, 0x20,
   0xe4, 0x20, 0x04, 0x20, 0x04, 0x20, 0xfc, 0x3f };

#define home_outside_width 16
#define home_outside_height 16
static const unsigned char home_outside_bits[] PROGMEM = {
   0x00, 0xe0, 0x80, 0xe1, 0x40, 0xe2, 0x20, 0x04, 0x10, 0x08, 0x08, 0x10,
   0x04, 0x20, 0x06, 0x60, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20,
   0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0xfc, 0x3f };

#define fan_8_width 8
#define fan_8_height 8
static const unsigned char fan_8_bits[] PROGMEM = {
   0x0e, 0x8c, 0xd8, 0xe4, 0x27, 0x1b, 0x31, 0x70 };

#define arrow_inside_8_width 8
#define arrow_inside_8_height 8
static const unsigned char arrow_inside_8_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x78, 0x18, 0x28, 0x48, 0x80 };

#define arrow_outside_8_width 8
#define arrow_outside_8_height 8
static const unsigned char arrow_outside_8_bits[] PROGMEM = {
   0xf0, 0xc0, 0xa0, 0x90, 0x08, 0x00, 0x00, 0x00 };

#define blow_width 16
#define blow_height 16
static const unsigned char blow_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x1e, 0x00, 0x32, 0x00, 0x22, 0x00, 0x30, 0xff, 0x1f,
   0x00, 0x00, 0xff, 0x7f, 0x00, 0xc0, 0xff, 0x89, 0x00, 0xdb, 0x20, 0x72,
   0x60, 0x03, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00 };

#define arrow_receive_8_width 8
#define arrow_receive_8_height 8
static const unsigned char arrow_receive_8_bits[] PROGMEM = {
   0x08, 0x08, 0x08, 0x08, 0x08, 0x3e, 0x1c, 0x08 };