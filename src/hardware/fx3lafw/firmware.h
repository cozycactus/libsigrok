/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 Ruslan Migirov <trapi78@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_FX3LAFW_FIRMWARE_H
#define LIBSIGROK_HARDWARE_FX3LAFW_FIRMWARE_H

#include <stddef.h>
#include <stdint.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

typedef int (*fx3lafw_firmware_write_cb)(uint32_t addr,
		const unsigned char *data, size_t length, void *cb_data);

static inline int fx3lafw_parse_firmware_image(const unsigned char *firmware,
		size_t length, fx3lafw_firmware_write_cb write_cb,
		void *cb_data, uint32_t *entry_addr)
{
	size_t offset, i, segment_bytes;
	uint32_t checksum, expected_checksum;
	int ret;

	if (length < 16 || firmware[0] != 'C' || firmware[1] != 'Y' ||
			(firmware[2] & 0x01) || firmware[3] != 0xb0)
		return SR_ERR;

	checksum = 0;
	offset = 4;
	while (offset < length) {
		uint32_t words, addr;
		uint64_t words64;

		if (length - offset < 8)
			break;

		words = RL32(firmware + offset);
		addr = RL32(firmware + offset + 4);
		offset += 8;

		if (words == 0) {
			if (length - offset < 4)
				break;
			expected_checksum = RL32(firmware + offset);
			if (checksum != expected_checksum)
				return SR_ERR;
			*entry_addr = addr;
			return SR_OK;
		}

		words64 = words;
		if (words64 > SIZE_MAX / 4)
			break;

		segment_bytes = (size_t)words * 4;
		if (segment_bytes > length - offset)
			break;

		for (i = 0; i < segment_bytes; i += 4)
			checksum += RL32(firmware + offset + i);

		if (write_cb) {
			ret = write_cb(addr, firmware + offset, segment_bytes,
				cb_data);
			if (ret != SR_OK)
				return ret;
		}

		offset += segment_bytes;
	}

	return SR_ERR;
}

#endif
