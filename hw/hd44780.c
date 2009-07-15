/*
 * HD44780 LCD Controller
 *
 * Copyright (c) 2009 Filip Navara
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "sysbus.h"
#include "console.h"
#include "pixel_ops.h"

/* pin mapping:
   8x Data
   Register Select (input only)
   Read/Write (input only)
   Enable (input only)
   Backlight (input only) */

#define D(x) x

#define PIN_RS              8
#define PIN_RW              9
#define PIN_E               10
#define PIN_BL              11

#define LCD_CLR             1  /* DB0: clear display */
#define LCD_HOME            2  /* DB1: return to home position */
#define LCD_ENTRY_MODE      4  /* DB2: set entry mode */
#define LCD_ENTRY_INC       2  /*   DB1: increment */
#define LCD_ENTRY_SHIFT     1  /*   DB2: shift */
#define LCD_ON_CTRL         8  /* DB3: turn lcd/cursor on */
#define LCD_ON_DISPLAY      4  /*   DB2: turn display on */
#define LCD_ON_CURSOR       2  /*   DB1: turn cursor on */
#define LCD_ON_BLINK        1  /*   DB0: blinking cursor */
#define LCD_MOVE            16 /* DB4: move cursor/display */
#define LCD_MOVE_DISP       8  /*   DB3: move display (0-> move cursor) */
#define LCD_MOVE_RIGHT      4  /*   DB2: move right (0-> left) */
#define LCD_FUNCTION        32 /* DB5: function set */
#define LCD_FUNCTION_8BIT   16 /*  DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_2LINES 8  /*  DB3: two lines (0->one line) */
#define LCD_FUNCTION_10DOTS 4  /*  DB2: 5x10 font (0->5x7 font) */
#define LCD_CGRAM           64 /* DB6: set CG RAM address */
#define LCD_DDRAM           128 /* DB7: set DD RAM address */
#define LCD_BUSY            64 /* DB7: LCD is busy */

#define WIDTH 20
#define HEIGHT 4
#define CZOOM 3

uint8_t font5x7[][7]={
/*   */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* ! */ {0x04,0x04,0x04,0x04,0x00,0x00,0x04},
/* " */ {0x0A,0x0A,0x0A,0x00,0x00,0x00,0x00},
/* # */ {0x0A,0x0A,0x1F,0x0A,0x1F,0x0A,0x0A},
/* $ */ {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
/* % */ {0x18,0x19,0x02,0x04,0x08,0x13,0x03},
/* & */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* ' */ {0x0C,0x04,0x08,0x00,0x00,0x00,0x00},
/* ( */ {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
/* ) */ {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
/* * */ {0x00,0x04,0x15,0x0E,0x15,0x04,0x00},
/* + */ {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
/* , */ {0x00,0x00,0x00,0x00,0x0C,0x04,0x08},
/* - */ {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
/* . */ {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
/* / */ {0x00,0x01,0x02,0x04,0x08,0x10,0x00},
/* 0 */ {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
/* 1 */ {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
/* 2 */ {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
/* 3 */ {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},
/* 4 */ {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
/* 5 */ {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
/* 6 */ {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
/* 7 */ {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
/* 8 */ {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
/* 9 */ {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
/* : */ {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00},
/* ; */ {0x00,0x0C,0x0C,0x00,0x0C,0x04,0x08},
/* < */ {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
/* = */ {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
/* > */ {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
/* ? */ {0x0E,0x11,0x01,0x02,0x04,0x00,0x04},
/* @ */ {0x0E,0x11,0x01,0x0D,0x15,0x15,0x0E},
/* A */ {0x0E,0x11,0x11,0x11,0x1F,0x11,0x11},
/* B */ {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
/* C */ {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
/* D */ {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C},
/* E */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
/* F */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
/* G */ {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
/* H */ {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
/* I */ {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
/* J */ {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
/* K */ {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
/* L */ {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
/* M */ {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
/* N */ {0x11,0x11,0x19,0x15,0x13,0x11,0x11},
/* O */ {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
/* P */ {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
/* Q */ {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
/* R */ {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
/* S */ {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
/* T */ {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
/* U */ {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
/* V */ {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
/* W */ {0x11,0x11,0x11,0x15,0x15,0x15,0x0A},
/* X */ {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
/* Y */ {0x11,0x11,0x11,0x0A,0x04,0x04,0x04},
/* Z */ {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
/* [ */ {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E},
/* \ */ {0x00,0x10,0x08,0x04,0x02,0x01,0x00},
/* ] */ {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E},
/* ^ */ {0x04,0x0A,0x11,0x00,0x00,0x00,0x00},
/* _ */ {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
/* ` */ {0x08,0x04,0x02,0x00,0x00,0x00,0x00},
/* a */ {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F},
/* b */ {0x10,0x10,0x10,0x16,0x19,0x11,0x1E},
/* c */ {0x00,0x00,0x0E,0x10,0x10,0x11,0x0E},
/* d */ {0x01,0x01,0x01,0x0D,0x13,0x11,0x0F},
/* e */ {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E},
/* f */ {0x06,0x09,0x08,0x1C,0x08,0x08,0x08},
/* g */ {0x00,0x0F,0x11,0x11,0x0F,0x01,0x0E},
/* h */ {0x10,0x10,0x16,0x19,0x11,0x11,0x11},
/* i */ {0x00,0x04,0x00,0x04,0x04,0x04,0x04},
/* j */ {0x02,0x00,0x06,0x02,0x02,0x12,0x0C},
/* k */ {0x10,0x10,0x12,0x14,0x18,0x14,0x12},
/* l */ {0x04,0x04,0x04,0x04,0x04,0x04,0x0F},
/* m */ {0x00,0x00,0x1A,0x15,0x15,0x11,0x11},
/* n */ {0x00,0x00,0x16,0x19,0x11,0x11,0x11},
/* o */ {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E},
/* p */ {0x00,0x00,0x1E,0x11,0x1E,0x10,0x10},
/* q */ {0x00,0x00,0x0D,0x13,0x0F,0x01,0x01},
/* r */ {0x00,0x00,0x16,0x19,0x10,0x10,0x10},
/* s */ {0x00,0x00,0x0E,0x10,0x0E,0x01,0x1E},
/* t */ {0x08,0x08,0x1C,0x08,0x08,0x09,0x06},
/* u */ {0x00,0x00,0x11,0x11,0x11,0x13,0x0D},
/* v */ {0x00,0x00,0x11,0x11,0x11,0x0A,0x04},
/* w */ {0x00,0x00,0x11,0x11,0x15,0x15,0x0A},
/* x */ {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11},
/* y */ {0x00,0x00,0x11,0x11,0x0F,0x01,0x0E},
/* z */ {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F},
/* { */ {0x02,0x04,0x04,0x08,0x04,0x04,0x02},
/* | */ {0x04,0x04,0x04,0x04,0x04,0x04,0x04},
/* } */ {0x08,0x04,0x04,0x02,0x04,0x04,0x08},
};

typedef struct LCDState {
    SysBusDevice busdev;
    qemu_irq out[8];
    DisplayState *ds;

    uint8_t input;
    uint32_t control : 1;
    uint32_t rw : 1;
    uint32_t backlight : 1;

    uint8_t mode8bit : 1;
    uint8_t write_low : 1;

    uint8_t ac; /* address counter */
    uint8_t dispcol; /* first visible column (display shift!) */
    uint8_t id : 1; /* cursor move increase(1)/decrease(0) */
    uint8_t sh : 1; /* shift display(1) */
    uint8_t ddram : 1; /* access ddram(1)/cgram(0) */

    uint8_t display : 1;
    uint8_t cursor : 1;
    uint8_t blink : 1;

    uint8_t two_lines : 1;
    uint8_t font5x10 : 1;

    uint8_t need_update : 1;
    char data[20 * 4];
} LCDState;

static void draw_char(DisplayState *ds, int x, int y, char ch, uint32_t color, uint32_t backcolor)
{
    uint8_t *d;
    uint8_t cdata;
    int i, bpp, line;

    bpp = (ds_get_bits_per_pixel(ds) + 7) >> 3;
    for (line = 0; line < 7 * CZOOM; line++) {
        d = ds_get_data(ds) + ds_get_linesize(ds) * y + bpp * x;
        if (ch >= ' ' && (ch - ' ') < sizeof(font5x7)/sizeof(font5x7[0]))
            cdata = font5x7[(int)(ch - ' ')][line / CZOOM];
        else
            cdata = font5x7[0][line / CZOOM];
        switch(bpp) {
        case 1:
            d += 5 * CZOOM;
            for (i = 0; i < 5 * CZOOM; i++) {
                *((uint8_t *)d) = (cdata & (1 << (i / CZOOM))) ? color : backcolor;
                d--;
            }
            break;
        case 2:
            d += 10 * CZOOM;
            for (i = 0; i < 5 * CZOOM; i++) {
                *((uint16_t *)d) = (cdata & (1 << (i / CZOOM))) ? color : backcolor;
                d -= 2;
            }
            break;
        case 4:
            d += 20 * CZOOM;
            for (i = 0; i < 5 * CZOOM; i++) {
                *((uint32_t *)d) = (cdata & (1 << (i / CZOOM))) ? color : backcolor;
                d -= 4;
            }
            break;
        }
        y++;
    }
}

static void hd44780_enable(LCDState *s)
{
    if (s->control) {
        //D(printf("CONTROL: %x\n", s->input));
        if (s->input & LCD_DDRAM) {
            int ddram_addr = s->input & ~LCD_DDRAM;
            if (ddram_addr >= 0 && ddram_addr < WIDTH)
                s->ac = ddram_addr;
            else if (ddram_addr >= 64 && ddram_addr < 64 + WIDTH)
                s->ac = (ddram_addr - 64) + WIDTH;
            else if (ddram_addr >= 20 && ddram_addr < 20 + WIDTH)
                s->ac = (ddram_addr - 20) + WIDTH * 2;
            else if (ddram_addr >= 84 && ddram_addr < 84 + WIDTH)
                s->ac = (ddram_addr - 84) + WIDTH * 3;
            s->ddram = 1;
        } else if (s->input & LCD_CGRAM) {
            s->ac = s->input & ~LCD_CGRAM;
            s->ddram = 0;
        } else if (s->input & LCD_FUNCTION) {
            s->mode8bit = !!(s->input & LCD_FUNCTION_8BIT);
            s->two_lines = !!(s->input & LCD_FUNCTION_2LINES);
            s->font5x10 = !!(s->input & LCD_FUNCTION_10DOTS);
        } else if (s->input & LCD_MOVE) {
            if (s->input & LCD_MOVE_DISP) {
                if (s->input & LCD_MOVE_RIGHT) {
                    s->dispcol--;
                } else {
                    s->dispcol++;
                }
                s->dispcol %= sizeof(s->data);
            } else {
                // ...
            }
        } else if (s->input & LCD_ON_CTRL) {
            s->display = !!(s->input & LCD_ON_DISPLAY);
            s->cursor = !!(s->input & LCD_ON_CURSOR);
            s->blink = !!(s->input & LCD_ON_BLINK);
        } else if (s->input & LCD_ENTRY_MODE) {
            s->id = !!(s->input & LCD_ENTRY_INC);
            s->sh = !!(s->input & LCD_ENTRY_SHIFT);
        } else if (s->input & LCD_HOME) {
            s->ac = 0;
            s->dispcol = 0;
            s->ddram = 1;
        } else if (s->input & LCD_CLR) {
            memset(s->data, 32, sizeof(s->data));
            s->ac = 0;
            s->dispcol = 0;
            s->id = 1;
            s->ddram = 1;
        }
    } else {
        if (s->ddram) {
            s->data[s->ac] = s->input;
            s->ac++;
            s->ac %= sizeof(s->data);
            if (s->sh) {
                if (s->id) {
                    s->dispcol++;
                } else {
                    s->dispcol--;
                }
                s->dispcol %= sizeof(s->data);
            }
            s->need_update = 1;
        } else {
            // ...
        }
    }
}

static void hd44780_set_pin(void *opaque, int pin, int level)
{
    LCDState *s = opaque;

    if (pin >= 0 && pin <= 7) {
        if (!s->rw) {
            if (!s->mode8bit && s->write_low)
                pin -= 4;
            if (level)
                s->input |= 1 << pin;
            else
                s->input &= ~(1 << pin);
        }
    } else if (pin == PIN_RS) {
        s->control = !level;
    } else if (pin == PIN_RW) {
        s->rw = level;
    } else if (pin == PIN_E) {
        if (!level && !s->rw) {
            if (!s->mode8bit) {
                s->write_low = !s->write_low;
                if (!s->write_low)
                    hd44780_enable(s);
            } else {
                hd44780_enable(s);
            }
        }
    } else if (pin == PIN_BL) {
        s->backlight = level;
        s->need_update = 1;
    }
}

static void hd44780_update_display(void *opaque)
{
    LCDState *s = opaque;
    uint32_t color_segment, color_led;
    int y, x, r, g, b;

    if (s->need_update) {
        if (s->backlight) {
            r = 0;
            g = 0xff;
            b = 0x80;
        } else {
            r = 0xf0;
            g = 0xe0;
            b = 0xb0;
        }

        switch (ds_get_bits_per_pixel(s->ds)) {
        case 8:
            color_segment = rgb_to_pixel8(0, 0, 0);
            color_led = rgb_to_pixel8(r, g, b);
            break;
        case 15:
            color_segment = rgb_to_pixel15(0, 0, 0);
            color_led = rgb_to_pixel15(r, g, b);
            break;
        case 16:
            color_segment = rgb_to_pixel16(0, 0, 0);
            color_led = rgb_to_pixel16(r, g, b);
            break;
        case 24:
            color_segment = rgb_to_pixel24(0, 0, 0);
            color_led = rgb_to_pixel24(r, g, b);
            break;
        case 32:
            color_segment = rgb_to_pixel32(0, 0, 0);
            color_led = rgb_to_pixel32(r, g, b);
            break;
        default:
            return;
        }

        if (s->display) {
            for (y = 0; y < HEIGHT; y++) {
                for (x = 0; x < WIDTH; x++) {
                    draw_char(s->ds, x * 5 * CZOOM, y * 7 * CZOOM, s->data[y * WIDTH + x], color_segment, color_led);
                }
            }
        }

        dpy_update(s->ds, 0, 0, WIDTH * CZOOM * 5, HEIGHT * CZOOM * 7);
    }
}

static void hd44780_invalidate_display(void * opaque)
{
    LCDState *s = opaque;
    s->need_update = 1;
}

static void hd44780_init(SysBusDevice *dev)
{
    LCDState *s = FROM_SYSBUS(LCDState, dev);

    s->need_update = 1;

    qdev_init_gpio_in(&dev->qdev, hd44780_set_pin, 12);
    qdev_init_gpio_out(&dev->qdev, s->out, 8);

    s->ds = graphic_console_init(hd44780_update_display,
                                 hd44780_invalidate_display,
                                 NULL, NULL, s);
    qemu_console_resize(s->ds, WIDTH * CZOOM * 5, HEIGHT * CZOOM * 7);
}

static void hd44780_register(void)
{
    sysbus_register_dev("gpio,hd44780", sizeof(LCDState),
                        hd44780_init);
}

device_init(hd44780_register)
