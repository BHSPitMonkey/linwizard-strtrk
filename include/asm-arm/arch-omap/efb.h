/*
** efb.h for efb in /home/nico/work/tornado/linux-tornado
** 
** Made by nico
** Login   <nico@chac.le-poulpe.net>
** 
** Started on  Wed Apr  5 18:58:04 2006 nico
** Last update Thu May 11 13:58:28 2006 nico
*/

/*
 *
 * 
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

#ifndef _EFB_H
# define _EFB_H

# include <linux/init.h>

/*
 * TODO: add __init and __initdata where applicable.
 */

void efb_init(void);
void efb_putstr(const char *str);

void efb_disable(void);
void efb_enable(void);

/* #define EFB_WIDTH 240 */
/* #define EFB_HEIGHT 320 */

extern const char efb_font[255][25];

extern unsigned int efb_width;
extern unsigned int efb_height;

#endif /* !_EFB_H */
