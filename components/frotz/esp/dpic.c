/*
 * dpic.c - Dumb interface, picture outline functions
 *
 * This file is part of Frotz.
 *
 * Frotz is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Frotz is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 * Or visit http://www.fsf.org/
 */

#include "dfrotz.h"
#include "dblorb.h"

extern f_setup_t f_setup;
extern z_header_t z_header;



bool dumb_init_pictures (void)
{
	return FALSE;
} /* dumb_init_pictures */



bool os_picture_data(int num, int *height, int *width)
{
	return TRUE;
} /* os_picture_data */


void os_draw_picture (int num, int row, int col)
{
} /* os_draw_picture */


int os_peek_colour (void) {return BLACK_COLOUR; }
