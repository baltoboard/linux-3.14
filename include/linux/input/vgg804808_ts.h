/*
 * Evervision VGG804808 touchscreen driver.
 *
 * Copyright (C) 2014, Ketron Srl. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

#ifndef __VGG804808_TS_H__
#define __VGG804808_TS_H__

#define VGG804808_TS_INVERT_X	(1 << 0)
#define VGG804808_TS_INVERT_Y	(1 << 1)
#define VGG804808_TS_SWAP_XY	(1 << 2)

struct vgg804808_ts_platform_data {
	u32 reset_pin;
	u32 flags;
};

#endif /* __VGG804808_TS_H__ */
