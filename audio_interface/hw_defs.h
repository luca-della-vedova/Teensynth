// Defines for VS1053B board
#define VS1053_RESET  41      // VS1053 reset pin (output)
#define VS1053_CS     10     // VS1053 chip select pin (output)
#define VS1053_DCS    93      // VS1053 Data/command select pin (output)

#define VS1053_CARDCS 40     // Card chip select pin
#define VS1053_DREQ 14       // VS1053 Data request, ideally an Interrupt pin

#define VS1053_MIDI Serial4

// LEDS
#define LED_R 32
#define LED_G 37
#define LED_B 38

#define BEAT_LED LED_B
#define READY_LED LED_G
#define SD_ERROR_LED LED_R
#define SMF_ERROR_LED LED_R

// Buttons
#define SW_SELECT 36
#define SW_CANCEL 35
#define SW_NEXT 34
#define SW_PREV 33

// Pitch bend pot
#define POT_X 9
#define POT_Y 8

// Grove LCD I2C, just default I2C
#define LCD_I2C Wire
