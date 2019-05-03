/*
 * ILI9341.c
 *
 *	Definicje funkcji do wyswietlacza TFT ILI9341 
 *
 */ 

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include "SPI.h"
#include "ILI9341.h"
#include "colors.h"


static uint8_t ILI9341_rd_cmd(uint8_t addr, uint8_t parameter);
static uint8_t ILI9341_xchg_data(uint8_t data);
static void ILI9341_wr_cmd(uint8_t cmd);


void ILI9341_reset()				// Reset wyswietlacza
{
    TFT_RST_LO;						
    _delay_ms(10);
    TFT_RST_HI;
}


void ILI9341_select()		// Aktywacja wyswietlacza
{
    TFT_CS_LO;
}

void ILI9341_deselect()		// Dezaktywacja wyswietlacza
{
    TFT_CS_HI;
}

static void ILI9341_init_io()				// Inicjalizacja szyny SPI do wyswietlacza TFT
{
    TFT_MOSI_DDR |= _BV(TFT_MOSI);			// Kierunek dla pinow MOSI, SCK, D/C jako wyjscie
    TFT_SCK_DDR |= _BV(TFT_SCK);			
    TFT_DC_DDR |= _BV(TFT_DC);				
	TFT_MISO_DDR &= ~(_BV(TFT_MISO));		// Kierunek pinu MISO jako wejscie
    
	TFT_MOSI_PORT |= _BV(TFT_MOSI);			// Stan wysoki na MOSI i SCK
    TFT_SCK_PORT |= _BV(TFT_SCK);			 
    TFT_MISO_PORT |= _BV(TFT_MISO);			// na MISO musi byc pull-up

#if USE_TFT_CS == 1							// Jesli uzywany pin CS:
    TFT_CS_DDR |= _BV(TFT_CS);				// Kierunek dla pinu CS jako wyjscie
    TFT_CS_PORT |= _BV(TFT_CS);				
#endif
    TFT_RST_DDR |= _BV(TFT_RST);			// TFT_RST pin as output
    TFT_RST_PORT |= _BV(TFT_RST);			// Hi state

    SPI_init();

}

static void ILI9341_wr_cmd(uint8_t command)	// wyslij komende
{
    TFT_DC_LO;								// TFT_DC LOW - komenda
#if USE_TFT_CS == 1							// ustawiany w ILI9341.h
    TFT_CS_LO;
#endif
    SPI_rxtx(command, TFT);					// Wymiana bajtow po SPI
#if USE_TFT_CS == 1							
    TFT_CS_HI;
#endif
}

static uint8_t ILI9341_xchg_data(uint8_t data)	// wymiana danych
{
	uint8_t rx_data;
	TFT_DC_HI;								// TFT_DC HIGH - dane
	#if USE_TFT_CS == 1			
	TFT_CS_LO;
	#endif
	rx_data = SPI_rxtx(data, TFT);			// Odbior bajtow po SPI
	#if USE_TFT_CS == 1
	TFT_CS_HI;
	#endif
	return rx_data;
}

void ILI9341_init()		// Inicjalizacja wyswietlacza ILI9341
{
    ILI9341_init_io();
    TFT_RST_LO;
    _delay_ms(50);
    TFT_RST_HI;
    _delay_ms(50);
    ILI9341_wr_cmd(ILI9341_SLEEP_OUT);										//sleep out
    ILI9341_wr_cmd(ILI9341_POWER_CONTROL_B);								//power control b
    ILI9341_xchg_data(0x00);
    ILI9341_xchg_data(0x83);
    ILI9341_xchg_data(0x30);
    ILI9341_wr_cmd(ILI9341_SOMETHING);										//power on seq control
    ILI9341_xchg_data(0x64);
    ILI9341_xchg_data(0x03);
    ILI9341_xchg_data(0x12);
    ILI9341_xchg_data(0x81);
    ILI9341_wr_cmd(ILI9341_DRIVER_TIMING_CONTROL_A);						//timing control a
    ILI9341_xchg_data(0x85);
    ILI9341_xchg_data(0x01);
    ILI9341_xchg_data(0x79);
    ILI9341_wr_cmd(ILI9341_POWER_ON_SEQ_CONTROL);							//power control a
    ILI9341_xchg_data(0x39);
    ILI9341_xchg_data(0X2C);
    ILI9341_xchg_data(0x00);
    ILI9341_xchg_data(0x34);
    ILI9341_xchg_data(0x02);
    ILI9341_wr_cmd(ILI9341_PUMP_RATIO_CONTROL);								//pump ratio control
    ILI9341_xchg_data(0x20);
    ILI9341_wr_cmd(ILI9341_DRIVER_TIMING_CONTROL_B);						//timing control b
    ILI9341_xchg_data(0x00);
    ILI9341_xchg_data(0x00);
    ILI9341_wr_cmd(ILI9341_POWER_CONTROL_1);								//power control 1
    ILI9341_xchg_data(0x26);
    ILI9341_wr_cmd(ILI9341_POWER_CONTROL_2);								//power control 2
    ILI9341_xchg_data(0x11);
    ILI9341_wr_cmd(ILI9341_VCOM_CONTROL_1);									//vcom control 1
    ILI9341_xchg_data(0x35);
    ILI9341_xchg_data(0x3E);
    ILI9341_wr_cmd(ILI9341_VCOM_CONTROL_2);									//vcom control 2
    ILI9341_xchg_data(0xBE);
    ILI9341_wr_cmd(ILI9341_FRAME_RATE_CONTROL_NORMAL);						//frame control
    ILI9341_xchg_data(0x00);
    ILI9341_xchg_data(0x1B);
    ILI9341_wr_cmd(ILI9341_DISPLAY_FUNCTION_CONTROL);						//display control
    ILI9341_xchg_data(0x0A);
    ILI9341_xchg_data(0x82);
    ILI9341_xchg_data(0x27);
    ILI9341_xchg_data(0x00);
    ILI9341_wr_cmd(ILI9341_ENTRY_MODE_SET);									//emtry mode
    ILI9341_xchg_data(0x07);
    ILI9341_wr_cmd(ILI9341_COLMOD_PIXEL_FORMAT_SET);						//pixel format
    ILI9341_xchg_data(0x55);
    ILI9341_wr_cmd(ILI9341_DISPLAY_ON);										//display on
}

/* Read command */
static uint8_t ILI9341_rd_cmd(uint8_t addr, uint8_t parameter)
{
	TFT_DC_LO;																// TFT_DC LOW - command
	#if USE_TFT_CS == 1													
	TFT_CS_LO;
	#endif
	SPI_rxtx(0xD9, TFT);													// Magic command
	TFT_DC_HI;
	SPI_rxtx(0x10 + parameter, TFT);										// Magic data
	TFT_DC_LO;
	SPI_rxtx(addr, TFT);
	uint8_t data = SPI_rxtx(0, TFT);										// Send dummy byte and receive answer
	#if USE_TFT_CS == 1													
	TFT_CS_HI;
	#endif
	return data;
}

uint32_t ILI9341_rd_id(void)
{
    uint32_t data = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        data <<= 8;
        data |= ILI9341_rd_cmd(ILI9341_READ_ID4, i + 1);
    }

    return data;
}

void ILI9341_rd_dummy(void)
{
	volatile uint16_t data;
	data = ILI9341_xchg_data(100);											// Dummy read
	data <<= 8;
	data |= ILI9341_xchg_data(100);											// Dummy read
}

/* Set display rotation -> paragraph 8.2.29
 * 0 - up
 * 1 - right
 * 2 - down
 * 3 - left
 * */
void ILI9341_set_rotation(uint8_t orientation)		// Pozycja obrazu
{
    rotation = orientation;

    switch (orientation)
    {
        case PORTRAIT:														// up
            tft_state.width = TFT_HEIGHT;
            tft_state.height = TFT_WIDTH;
            ILI9341_wr_cmd(ILI9341_MEMORY_ACCESS_CONTROL);
            ILI9341_xchg_data(0x40 | 0x08);
            break;

        case LANDSCAPE:														// right
            tft_state.width = TFT_WIDTH;
            tft_state.height = TFT_HEIGHT;
            ILI9341_wr_cmd(ILI9341_MEMORY_ACCESS_CONTROL);
            ILI9341_xchg_data(0x20 | 0x08);
            break;

        case PORTRAIT_REV:													// down
            tft_state.width = TFT_HEIGHT;
            tft_state.height = TFT_WIDTH;
            ILI9341_wr_cmd(ILI9341_MEMORY_ACCESS_CONTROL);
            ILI9341_xchg_data(0x80 | 0x08);
            break;

        case LANDSCAPE_REV:													//  left
            tft_state.width = TFT_WIDTH;
            tft_state.height = TFT_HEIGHT;
            ILI9341_wr_cmd(ILI9341_MEMORY_ACCESS_CONTROL);
            ILI9341_xchg_data(0x40 | 0x80 | 0x20 | 0x08);
            break;
    }
}

/* Set current drawing coordinates */
void ILI9341_set_xy(uint16_t x, uint16_t y)
{
    ILI9341_wr_cmd(ILI9341_COLUMN_ADDRESS_SET);
    ILI9341_xchg_data(x >> 8);
    ILI9341_xchg_data(x);
    ILI9341_xchg_data(tft_state.width >> 8);
    ILI9341_xchg_data(tft_state.width);
    ILI9341_wr_cmd(ILI9341_PAGE_ADDRESS_SET);
    ILI9341_xchg_data(y >> 8);
    ILI9341_xchg_data(y);
    ILI9341_xchg_data(tft_state.height >> 8);
    ILI9341_xchg_data(tft_state.height);
    ILI9341_wr_cmd(ILI9341_MEMORY_WRITE);
}

/* Set current drawing window */
void ILI9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    if ((x0 > tft_state.width) || (y0 > tft_state.height) || (x1 > tft_state.width) || (y1 > tft_state.height)) return;	// Nothing if out of the screen

    ILI9341_wr_cmd(ILI9341_COLUMN_ADDRESS_SET);
    ILI9341_xchg_data(x0 >> 8);
    ILI9341_xchg_data(x0);
    ILI9341_xchg_data(x1 >> 8);
    ILI9341_xchg_data(x1);
    ILI9341_wr_cmd(ILI9341_PAGE_ADDRESS_SET);
    ILI9341_xchg_data(y0 >> 8);
    ILI9341_xchg_data(y0);
    ILI9341_xchg_data(y1 >> 8);
    ILI9341_xchg_data(y1);
    ILI9341_wr_cmd(ILI9341_MEMORY_WRITE);
}

/* Draw single pixel at given coordinates with a color */
void ILI9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x > tft_state.width) || (y > tft_state.height)) return;			// Nothing if out of the screen

	ILI9341_set_xy(x, y);
    ILI9341_push_color(color);
}

/* Write color to LCD (at current coordinates) */
void ILI9341_push_color(uint16_t color)
{
    ILI9341_xchg_data(color >> 8);
    ILI9341_xchg_data(color);
}

/* 8 colors on display */
void ILI9341_set_8colors()
{
    ILI9341_wr_cmd(ILI9341_IDLE_MODE_ON);
    tft_state.idle = 1;
}

/* 64k colors on display */
void ILI9341_set_64kcolors()
{
    ILI9341_wr_cmd(ILI9341_IDLE_MODE_OFF);
    tft_state.idle = 0;
}

/* Convert RGB to 8bit */
uint8_t rgb_color8(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t color8 = ((r * 7 / 255) << 5) + ((g * 7 / 255) << 2) + (b * 3 / 255);
    return color8;
}

/* Convert RGB to 16bit */
uint16_t rgb_color565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t color;
    color = r >> 3;															// 00000000_000rrrrr
    color <<= 6;															// 00000rrr_rr000000
    color |= g >> 2;														// 00000rrr_rrgggggg
    color <<= 5;															// rrrrrggg_ggg00000
    color |= b >> 3;														// rrrrrggg_gggbbbbb
    return color;
}

/* Draw character at given coordinates and color */
void ILI9341_chr(uint16_t x, uint16_t y, char c)
{
    if (current_font.x_size == 5)
    {
        ILI9341_set_window(x, y, x + current_font.x_size, y + current_font.y_size);

        for (uint8_t cols = 0; cols < 5; cols++ )								// 5 columns
        {
            uint8_t line = pgm_read_byte(current_font.font + ( (c - ' ') * 5) + cols);		// Characters start with code 32 (space)

            for (uint8_t rows = 0; rows < 8; rows++)							// 8 rows
            {
                if (line & 0x1)													// If bit in pattern set
                {
                    ILI9341_draw_pixel(x + cols, y + rows, current_font.fg_color);
                }
                else
                {
                    if (current_font.bg_color != TRANSPARENT)
                    {
                        ILI9341_draw_pixel(x + cols, y + rows, current_font.bg_color);
                    }
                }

                line >>= 1;														// Next bit in pattern
            }
        }
    }
    else
    {
        const uint8_t *adr = current_font.font + ((c - ' ') * ((current_font.x_size + 7) / 8) * current_font.y_size);

        if (current_font.bg_color != TRANSPARENT) ILI9341_draw_mono_bmp(x, y, current_font.x_size, current_font.y_size, adr, current_font.fg_color, current_font.bg_color);
        else ILI9341_draw_mono_trans_bmp(x, y, current_font.x_size, current_font.y_size, adr, current_font.fg_color);
    }
}

/* Draw text at given coordinates */
void ILI9341_txt(uint16_t x, uint16_t y, char *string)
{
    while (*string)
    {
        ILI9341_chr(x, y, *string);

        if (current_font.x_size == 5) x += 6;
        else x += current_font.x_size;

        string++;
    }
}

/* Draw text from program memory at given coordinates */
void ILI9341_txt_P(uint16_t x, uint16_t y, const char *string)
{
    while (pgm_read_byte(string))
    {
        ILI9341_chr(x, y, pgm_read_byte(string));

        if (current_font.x_size == 5) x += 6;
        else x += current_font.x_size;

        string++;
    }
}

/* Set parameters for current text */
void ILI9341_set_font(font_t font)
{
    current_font = font;
}

/* Fill screen with color */
void ILI9341_cls(uint16_t color)
{
    ILI9341_set_window(0, 0, tft_state.width - 1, tft_state.height - 1);
    uint32_t i = 76800;									// 320x240=76800

    while (i--)
    {
        ILI9341_push_color(color);
    }
}

/* Draw any line on the display */
void ILI9341_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int16_t d, dx, dy, ai, bi;												// aux variables
    int8_t xi, yi;
    int16_t ax = x0, ay = y0;

    if (x0 < x1)															// drawing direction
    {
        xi = 1;
        dx = x1 - x0;
    }
    else
    {
        xi = -1;
        dx = x0 - x1;
    }

    if (y0 < y1)															// drawing direction
    {
        yi = 1;
        dy = y1 - y0;
    }
    else
    {
        yi = -1;
        dy = y0 - y1;
    }

    ILI9341_draw_pixel(ax, ay, color);							// First pixel

    if (dx > dy)															// Leading axis X
    {
        ai = (dy - dx) * 2;
        bi = dy * 2;
        d = bi - dx;

        while (ax != x1)													// Loop x
        {
            if (d >= 0)														// Test coefficient
            {
                ax += xi;
                ay += yi;
                d += ai;
            }
            else
            {
                d += bi;
                ax += xi;
            }

            ILI9341_draw_pixel(ax, ay, color);
        }
    }
    else																	// Leading axle Y
    {
        ai = ( dx - dy ) * 2;
        bi = dx * 2;
        d = bi - dy;

        while (ay != y1)													// Loop y
        {
            if (d >= 0)														// Test coefficient
            {
                ax += xi;
                ay += yi;
                d += ai;
            }
            else
            {
                d += bi;
                ay += yi;
            }

            ILI9341_draw_pixel(ax, ay, color);
        }
    }
}

/* Draw fast line
 * - HORIZONTAL
 * | VERTICAL
 * / UP_SLOPE
 * \ DN_SLOPE */
void ILI9341_draw_fast_line(uint16_t x, uint16_t y, uint16_t lenght, uint16_t color, enum direction_t line_direction)
{
    switch (line_direction)
    {
        case HORIZONTAL:
            ILI9341_set_window(x, y, x + lenght, y + 1);

            for (uint16_t i = 0; i < lenght; i++) ILI9341_push_color(color);

            break;

        case VERTICAL:
            ILI9341_set_window(x, y, x, y + lenght);

            for (uint16_t i = 0; i < lenght; i++) ILI9341_push_color(color);

            break;

        case UP_SLOPE:
            ILI9341_set_window(x, y - lenght, x + lenght, y);

            for (uint16_t i = 0; i < lenght; i++)
            {
                ILI9341_set_xy(x + i, y - i);
                ILI9341_push_color(color);
            }

            break;

        case DN_SLOPE:
            ILI9341_set_window(x, y, x + lenght, y + lenght);

            for (uint16_t i = 0; i < lenght; i++)
            {
                ILI9341_set_xy(x + i, y + i);
                ILI9341_push_color(color);
            }

            break;
    }
}

void ILI9341_draw_fast_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bool fill, uint16_t color)
{
    if (fill)
    {
        ILI9341_set_window(x, y, x + width - 1, y + height - 1);
        uint32_t i = (uint32_t) width * (uint32_t)height;

        while (i--)
        {
            ILI9341_push_color(color);
        }
    }
    else
    {
        ILI9341_draw_fast_line(x, y, width, color, HORIZONTAL);
        ILI9341_draw_fast_line(x, y + height - 1, width, color, HORIZONTAL);
        ILI9341_draw_fast_line(x, y, height, color, VERTICAL);
        ILI9341_draw_fast_line(x + width, y, height, color, VERTICAL);
    }
}

/* Draw polygon on the display
 * nodes[]: table of nodes
 * count: number of lines
 * color: color
 * close: 0 - don't close, 1 - close polygon */
void ILI9341_draw_polygon(uint16_t nodes[], uint8_t count, uint16_t color, uint8_t close)
{
    uint8_t node_id = 0;
    uint16_t x0, y0, x1 = 0, y1 = 0;										// Only to avoid warning

    while ((node_id / 2) < count)
    {
        x0 = nodes[node_id];
        y0 = nodes[node_id + 1];
        node_id += 2;
        x1 = nodes[node_id];
        y1 = nodes[node_id + 1];
        ILI9341_draw_line(x0, y0, x1, y1, color);
    }

    if (close) ILI9341_draw_line(x1, y1, nodes[0], nodes[1], color);		// Close polygon
}

/* Draw a circle with given center, radius, fill flag and color */
void ILI9341_draw_circle(uint16_t x, uint16_t y, uint8_t radius, bool fill, uint16_t color)
{
    uint8_t repeat = radius;

    do
    {
        int16_t t = 0;
        int16_t err = 0;

        while (radius >= t)
        {
            ILI9341_draw_pixel(x + radius, y + t, color);
            ILI9341_draw_pixel(x + t, y + radius, color);
            ILI9341_draw_pixel(x - t, y + radius, color);
            ILI9341_draw_pixel(x - radius, y + t, color);
            ILI9341_draw_pixel(x - radius, y - t, color);
            ILI9341_draw_pixel(x - t, y - radius, color);
            ILI9341_draw_pixel(x + t, y - radius, color);
            ILI9341_draw_pixel(x + radius, y - t, color);
            t += 1;
            err += 1 + 2 * t;

            if (2 * (err - radius) + 1 > 0)
            {
                radius -= 1;
                err += 1 - 2 * radius;
            }
        }

        repeat--;
        radius = repeat;
    }
    while ((radius > 0) && fill);
}

/* Draw triangle with given nodes and colors*/
void ILI9341_draw_triangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t triangle[3][2] =
    {
        {x0, y0},
        {x1, y1},
        {x2, y2}
    };
    ILI9341_draw_polygon(*triangle, 2, color, 1);
}

/* Draw rectangle with given nodes and colors*/
void ILI9341_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    uint16_t rectangle[4][2] =
    {
        {x0, y0},
        {x1, y0},
        {x1, y1},
        {x0, y1}
    };
    ILI9341_draw_polygon(*rectangle, 3, color, 1);
}

/* Draw 1-bit bitmap with transparent background */
void ILI9341_draw_mono_trans_bmp(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap, uint16_t color)
{
    uint16_t byte_width = (width + 7) / 8;
    uint8_t pattern = pgm_read_byte(bitmap);

    for (uint16_t rows = 0; rows < height; rows++)							// For every row in bitmap pattern
    {
        for (uint16_t cols = 0; cols < width; cols++)						// For every column in bitmap pattern
        {
            if (pattern & 0x80) ILI9341_draw_pixel(x + cols, y + rows, color);	// If bit in pattern is set, pixel in foreground color

            if (cols & 7) pattern <<= 1;									// Eight bits to draw
            else pattern = pgm_read_byte(bitmap + rows * byte_width + cols / 8); // Next byte of pattern
        }
    }
}

/* Draw 1-bit bitmap with given foreground and background color */
void ILI9341_draw_mono_bmp(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap, uint16_t fg, uint16_t bg)
{
    ILI9341_set_window(x, y, x + width, y + height);
    uint16_t byte_width = (width + 7) / 8;
    uint8_t pattern = pgm_read_byte(bitmap);

    for (uint16_t rows = 0; rows < height; rows++)							// For every row in bitmap pattern
    {
        for (uint16_t cols = 0; cols < width + 1; cols++)					// For every column in bitmap pattern
        {
            if (pattern & 0x80) ILI9341_push_color(fg);						// If bit in pattern is set, pixel in foreground color
            else
                ILI9341_push_color(bg);

            if (cols & 7) pattern <<= 1;									// Eight bits to draw
            else pattern = pgm_read_byte(bitmap + rows * byte_width + cols / 8); // Next byte of pattern
        }
    }
}

/* Draw 16-bit color bitmap
 * (overlaps if on the window edge) */
void ILI9341_draw_color_bmp(uint16_t x, uint16_t y, uint16_t height, uint16_t width, const uint16_t *bitmap)
{
    ILI9341_set_window(x, y, x + width - 1, y + height - 1);

    for (uint16_t rows = 0; rows < height; rows++)
    {
        for (uint16_t cols = 0; cols < width; cols++)
        {
            ILI9341_push_color(pgm_read_word(bitmap + (width * rows) + cols));
        }
    }
}

/* Reverse all colors on the display */
void ILI9341_negative()
{
    if (tft_state.inversion == 0)
    {
        ILI9341_wr_cmd(ILI9341_DISP_INVERSION_ON);
        tft_state.inversion = 1;
    }
    else
    {
        ILI9341_wr_cmd(ILI9341_DISP_INVERSION_OFF);
        tft_state.inversion = 0;
    }
}
