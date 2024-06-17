/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _SCREEN_INFO_H
#define _SCREEN_INFO_H

#include <uapi/linux/screen_info.h>

#include <linux/bits.h>

extern struct screen_info screen_info;

static inline bool __screen_info_vbe_mode_nonvga(const struct screen_info *si)
{
	/*
	 * VESA modes typically run on VGA hardware. Set bit 5 signals that this
	 * is not the case. Drivers can then not make use of VGA resources. See
	 * Sec 4.4 of the VBE 2.0 spec.
	 */
	return si->vesa_attributes & BIT(5);
}

#endif /* _SCREEN_INFO_H */
