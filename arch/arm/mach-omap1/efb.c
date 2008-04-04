/*
** efb.c for efb in /home/nico/work/tornado/linux-tornado
** 
** Made by nico
** Login   <nico@chac.le-poulpe.net>
** 
** Started on  Wed Apr  5 18:55:08 2006 nico
** Last update Thu May 11 13:50:52 2006 nico
*/

/*
 *
 * early frame buffer debug routines.
 * Copyright (C) 2006 Nicolas Schichan
 *
 * This  program is  free  software; you  can  redistribute it  and/or
 * modify  it under the  terms of  the GNU  General Public  License as
 * published by the Free Software  Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT  ANY  WARRANTY;  without   even  the  implied  warranty  of
 * MERCHANTABILITY or  FITNESS FOR A PARTICULAR PURPOSE.   See the GNU
 * General Public License for more details.
 * 
 * You should have  received a copy of the  GNU General Public License
 * along  with  this program;  if  not,  write  to the  Free  Software
 * Foundation,  Inc.,  51 Franklin  Street,  Fifth  Floor, Boston,  MA
 * 02110-1301, USA. 
 *
 */

#include <asm/mach-types.h>
#include <asm/arch/efb.h>

static void efb_move_screen_up(void);
static void blit_char(unsigned int x, unsigned int y, char c);

#define PIX16(R, G, B) (((R >> 3) << 11) | ((G >> 2) << 5) | ((B >> 3)))


/* 0xD0000000 is where OMAP730_SRAM is mapped by head.S */
/*
 * Windows CE seems to have physical frame buffer mapped to
 * 0x20001020. we try to keep this space available at virtual address
 * 0xD0001020 during kernel boot. This is strange as this is not page
 * aligned. However even if I don't know how to do it, there must be a
 * way to configure the framebuffer location inside the sram.
 *
 * TODO: see omap730 TRM.
 */
static unsigned short __initdata *vram = (unsigned short *)0xD0001020;

/*
 * no  attribute since it is used by efb_putstr which has no
 * attribute ...
 */
static int enabled = 0;

unsigned int efb_width;
unsigned int efb_height;

void __init efb_init(void)
{
	unsigned int i;

	/*
	 * only htc typhoon is known to have 176x220 screen. all other
	 * machines have 320x240 screens.
	 */
	if (machine_is_typhoon()) {
		efb_width = 176;
		efb_height = 220;
	} else {
		efb_width = 240;
		efb_height = 320;
	}

	for (i = 0; i < efb_width * efb_height; ++i) {
		vram[i] = 0;
	}

	efb_enable();
}


/*
 * Write a pixel with color (r,g,b) at screen coordinate (x,y)
 */
static void __init dbg_pix(int x, int y, int r, int g, int b)
{
	unsigned short pix = PIX16(r, g, b);

	vram[y * efb_width + x] = pix;
}


static __initdata int x = 1;
static __initdata int  y = 0;


/*
 * put string s to the frame buffer using the 5x5 font.
 * 
 * this functions has no __init attribute since it is can be called in printk.
 */
void efb_putstr(const char *s)
{
	if (enabled == 0)
		return ;

	while (*s) {
		if (*s == '\n')
			goto newline;
		blit_char(x, y, *s);
		x += 6;

		if (x + 6 > efb_width) {
		newline:
			if (y + 6 > efb_height - 6)
				efb_move_screen_up();
			else
				y += 6;
			x = 1;
		}
		++s;
	}
}

#define NR_LINE (efb_height / 6)

#define FONT_WIDTH 5
#define FONT_HEIGHT 5

/*
 * blit a char on the screen at position (x,y)
 */
static __init void blit_char(unsigned int x, unsigned int y, char c)
{
	const char *cur_font;
	int i, j;

	/*
	 * do not blit glyph if some part of it are ofscreen.
	 */
	if (x + FONT_WIDTH > efb_width)
		return;
	if (y + FONT_HEIGHT > efb_height)
		return;

	cur_font = efb_font[(unsigned int)c];
	for (j= 0; j < FONT_HEIGHT; ++j) {
		for (i = 0; i < FONT_WIDTH; ++i) {
			if (cur_font[j * FONT_WIDTH + i])
				dbg_pix(x + i, y + j, 255, 255, 255);
			else
				dbg_pix(x + i, y + j, 0, 0, 0);
		}
	}
}

/*
 * perform some basic one way scrolling.
 *
 * for each line l do
 *   clear line l
 *   break if l is the last line
 *   copy next line over l
 * done
 *
 * It is not possible to scroll back.
 */
static void __init efb_move_screen_up(void)
{
	int i, k;
	int start1, start2;

	for (k = 0; 1; ++k) {
		start1 = efb_width * 6 * k;
		start2 = efb_width * 6 * (k + 1);
		/* clear the n current line */
		for (i = 0; i < 6 * efb_width; ++i)
			vram[start1 + i] = 0;
		if (k == NR_LINE - 1)
			break;
		/* copy the next line to the current line */
		for (i = 0; i < 6 * efb_width; ++i)
			vram[start1 + i] = vram[start2 +i];
	}
}

void __init efb_disable(void)
{
	enabled = 0;
}

void __init efb_enable(void)
{
	enabled = 1;
}
