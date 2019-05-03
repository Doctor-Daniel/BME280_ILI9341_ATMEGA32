/*
 * ILI9341.h
 *
 * Created: 2018-11-06 22:21:06
 * Author : Daniel
 * 
 * Podstawowe deklaracje pinow wyswietlacza TFT ILI9341
 * Calosc oparta o sprzetowa szyne SPI
 */ 

#include <stdbool.h>

#ifndef ILI9341_H_
#define ILI9341_H_

#define	TFT_WIDTH	320
#define	TFT_HEIGHT	240

#define USE_TFT_CS	1

#define	TFT_SCK		PB7												// SCK	-  Assigment dla ATMEGA 32
#define TFT_MISO	PB6												// MISO
#define TFT_MOSI	PB5												// MOSI
#define TFT_RST		PB4												// RESET  - ten w sumie moze byc na stale do 3,3 V
#define TFT_DC		PB3												// D/C	-	Data / Command
#define TFT_CS		PB2												// CS	- Chip Select

// Definicje portow - Wyswietlacz TDT na porcie B
#define TFT_SCK_PORT	PORTB
#define TFT_SCK_DDR		DDRB
#define TFT_MOSI_PORT	PORTB
#define TFT_MOSI_DDR	DDRB
#define TFT_RST_PORT	PORTB
#define TFT_RST_DDR		DDRB
#define TFT_DC_PORT		PORTB
#define TFT_DC_DDR		DDRB
#define TFT_CS_PORT		PORTB
#define TFT_CS_DDR		DDRB
#define TFT_MISO_PORT	PORTB
#define TFT_MISO_DDR	DDRB
#define TFT_MISO_PIN	PINB	


// Makra
#define TFT_SCK_LO	TFT_SCK_PORT &= ~(_BV(TFT_SCK))
#define TFT_SCK_HI	TFT_SCK_PORT |= _BV(TFT_SCK)
#define TFT_MOSI_LO	TFT_MOSI_PORT &= ~(_BV(TFT_MOSI))
#define TFT_MOSI_HI	TFT_MOSI_PORT |= _BV(TFT_MOSI)
#define TFT_RST_LO	TFT_RST_PORT &= ~(_BV(TFT_RST))
#define TFT_RST_HI	TFT_RST_PORT |= _BV(TFT_RST)
#define TFT_DC_LO	TFT_DC_PORT &= ~(_BV(TFT_DC))
#define TFT_DC_HI	TFT_DC_PORT |= _BV(TFT_DC)
#define TFT_CS_LO	TFT_CS_PORT &= ~(_BV(TFT_CS))
#define TFT_CS_HI	TFT_CS_PORT |= _BV(TFT_CS)
#define	TFT_MISO_X	TFT_MISO_PIN & _BV(TFT_MISO)

enum direction_t {HORIZONTAL, VERTICAL, UP_SLOPE, DN_SLOPE};
enum position_t {PORTRAIT = 0, LANDSCAPE = 1, PORTRAIT_REV = 2, LANDSCAPE_REV = 3};

// Stan wyswietlacza TFT
typedef struct
{
	uint16_t width;
	uint16_t height;
	uint8_t inversion;
	uint8_t idle;
} tft_t;

// Font description
typedef struct
{
    const uint8_t *font;
    uint8_t x_size;
    uint8_t y_size;
    uint16_t fg_color;
    uint16_t bg_color;
} font_t;

uint8_t rotation;
font_t current_font;
tft_t tft_state;

// Funkcje
void ILI9341_init(void);
void ILI9341_reset(void);
void ILI9341_select(void);
void ILI9341_deselect(void);
uint16_t rgb_color565(uint8_t r, uint8_t g, uint8_t b);
uint8_t rgb_color8(uint8_t r, uint8_t g, uint8_t b);
uint16_t ILI9341_rd_ram(uint16_t x, uint16_t y);
void ILI9341_rd_dummy(void);
uint32_t ILI9341_rd_id(void);
void ILI9341_push_color(uint16_t color);
void ILI9341_set_8colors(void);
void ILI9341_set_64kcolors(void);
void ILI9341_negative(void);
void ILI9341_scroll(uint16_t shift);
void ILI9341_set_rotation(uint8_t orientation);
void ILI9341_set_xy(uint16_t x, uint16_t y);
void ILI9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9341_set_font(font_t font);
void ILI9341_cls(uint16_t color);
void ILI9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ILI9341_draw_fast_line(uint16_t x, uint16_t y, uint16_t lenght, uint16_t color, enum direction_t line_direction);
void ILI9341_draw_fast_rect(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, bool fill, uint16_t color);
void ILI9341_draw_polygon(uint16_t nodes[], uint8_t count, uint16_t color, uint8_t close);
void ILI9341_draw_triangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ILI9341_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ILI9341_draw_circle(uint16_t x, uint16_t y, uint8_t radius, bool fill, uint16_t color);
void ILI9341_draw_mono_bmp(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap, uint16_t fg, uint16_t bg);
void ILI9341_draw_mono_trans_bmp(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap, uint16_t color);
void ILI9341_draw_color_bmp(uint16_t x, uint16_t y, uint16_t height, uint16_t width, const uint16_t *bitmap);
void ILI9341_chr(uint16_t x, uint16_t y, char c);
void ILI9341_txt(uint16_t x, uint16_t y, char *string);
void ILI9341_txt_P(uint16_t x, uint16_t y, const char *string);

// Z Datashheta: adresy poszczegolnych komend
#define ILI9341_NOP                             0x00
#define ILI9341_SOFTWARE_RESET                  0x01
#define ILI9341_READ_DISP_ID                    0x04
#define ILI9341_READ_DISP_STATUS                0x09
#define ILI9341_READ_DISP_MADCTRL               0x0B
#define ILI9341_READ_DISP_PIXEL_FORMAT          0x0C
#define ILI9341_READ_DISP_IMAGE_FORMAT          0x0D
#define ILI9341_READ_DISP_SIGNAL_MODE           0x0E
#define ILI9341_READ_DISP_SELF_DIAGNOSTIC       0x0F
#define ILI9341_ENTER_SLEEP_MODE                0x10
#define ILI9341_SLEEP_OUT                       0x11
#define ILI9341_PARTIAL_MODE_ON                 0x12
#define ILI9341_NORMAL_DISP_MODE_ON             0x13
#define ILI9341_DISP_INVERSION_OFF              0x20
#define ILI9341_DISP_INVERSION_ON               0x21
#define ILI9341_GAMMA_SET                       0x26
#define ILI9341_DISPLAY_OFF                     0x28
#define ILI9341_DISPLAY_ON                      0x29
#define ILI9341_COLUMN_ADDRESS_SET              0x2A
#define ILI9341_PAGE_ADDRESS_SET                0x2B
#define ILI9341_MEMORY_WRITE                    0x2C
#define ILI9341_COLOR_SET                       0x2D
#define ILI9341_MEMORY_READ                     0x2E
#define ILI9341_PARTIAL_AREA                    0x30
#define ILI9341_VERT_SCROLL_DEFINITION          0x33
#define ILI9341_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9341_TEARING_EFFECT_LINE_ON          0x35
#define ILI9341_MEMORY_ACCESS_CONTROL           0x36
#define ILI9341_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9341_IDLE_MODE_OFF                   0x38
#define ILI9341_IDLE_MODE_ON                    0x39
#define ILI9341_COLMOD_PIXEL_FORMAT_SET         0x3A
#define ILI9341_WRITE_MEMORY_CONTINUE           0x3C
#define ILI9341_READ_MEMORY_CONTINUE            0x3E
#define ILI9341_SET_TEAR_SCANLINE               0x44
#define ILI9341_GET_SCANLINE                    0x45
#define ILI9341_WRITE_DISPLAY_BRIGHTNESS        0x51
#define ILI9341_READ_DISPLAY_BRIGHTNESS         0x52
#define ILI9341_WRITE_CTRL_DISPLAY              0x53
#define ILI9341_READ_CTRL_DISPLAY               0x54
#define ILI9341_WRITE_CONTENT_ADAPT_BRIGHTNESS  0x55
#define ILI9341_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define ILI9341_WRITE_MIN_CAB_LEVEL             0x5E
#define ILI9341_READ_MIN_CAB_LEVEL              0x5F
#define ILI9341_READ_ID1                        0xDA
#define ILI9341_READ_ID2                        0xDB
#define ILI9341_READ_ID3                        0xDC
#define ILI9341_RGB_SIGNAL_CONTROL              0xB0
#define ILI9341_FRAME_RATE_CONTROL_NORMAL       0xB1
#define ILI9341_FRAME_RATE_CONTROL_IDLE_8COLOR  0xB2
#define ILI9341_FRAME_RATE_CONTROL_PARTIAL      0xB3
#define ILI9341_DISPLAY_INVERSION_CONTROL       0xB4
#define ILI9341_BLANKING_PORCH_CONTROL          0xB5
#define ILI9341_DISPLAY_FUNCTION_CONTROL        0xB6
#define ILI9341_ENTRY_MODE_SET                  0xB7
#define ILI9341_BACKLIGHT_CONTROL_1             0xB8
#define ILI9341_BACKLIGHT_CONTROL_2             0xB9
#define ILI9341_BACKLIGHT_CONTROL_3             0xBA
#define ILI9341_BACKLIGHT_CONTROL_4             0xBB
#define ILI9341_BACKLIGHT_CONTROL_5             0xBC
#define ILI9341_BACKLIGHT_CONTROL_6             0xBD
#define ILI9341_BACKLIGHT_CONTROL_7             0xBE
#define ILI9341_BACKLIGHT_CONTROL_8             0xBF
#define ILI9341_POWER_CONTROL_1                 0xC0
#define ILI9341_POWER_CONTROL_2                 0xC1
#define ILI9341_VCOM_CONTROL_1                  0xC5
#define ILI9341_VCOM_CONTROL_2                  0xC7
#define ILI9341_POWER_ON_SEQ_CONTROL            0xCB
#define ILI9341_POWER_CONTROL_A                 0xCD
#define ILI9341_POWER_CONTROL_B                 0xCF
#define ILI9341_NVMEM_WRITE                     0xD0
#define ILI9341_NVMEM_PROTECTION_KEY            0xD1
#define ILI9341_NVMEM_STATUS_READ               0xD2
#define ILI9341_READ_ID4                        0xD3
#define ILI9341_POSITIVE_GAMMA_CORRECTION       0xE0
#define ILI9341_NEGATIVE_GAMMA_CORRECTION       0xE1
#define ILI9341_DIGITAL_GAMMA_CONTROL_1         0xE2
#define ILI9341_DIGITAL_GAMMA_CONTROL_2         0xE3
#define ILI9341_DRIVER_TIMING_CONTROL_A         0xE8
#define ILI9341_DRIVER_TIMING_CONTROL_B         0xEA
#define ILI9341_ENABLE_3_GAMMA_CONTROL          0xF2
#define ILI9341_INTERFACE_CONTROL               0xF6
#define ILI9341_PUMP_RATIO_CONTROL              0xF7
#define ILI9341_SOMETHING						0xED

#endif /* ILI9341_H_ */