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

#include <config.h>
#include <string.h>
#include <check.h>
#include "lib.h"

#include "../src/hardware/fx3lafw/firmware.h"
#include "../src/hardware/fx3lafw/protocol.h"

struct fx3lafw_test_write {
	uint32_t addr;
	unsigned char data[16];
	size_t length;
	unsigned int calls;
};

static void append_u32(unsigned char *image, size_t *offset, uint32_t value)
{
	WL32(image + *offset, value);
	*offset += sizeof(uint32_t);
}

static size_t build_valid_image(unsigned char *image)
{
	size_t offset;
	uint32_t word0, word1;

	offset = 0;
	image[offset++] = 'C';
	image[offset++] = 'Y';
	image[offset++] = 0x1c;
	image[offset++] = 0xb0;

	word0 = 0x11223344;
	word1 = 0xaabbccdd;

	append_u32(image, &offset, 2);
	append_u32(image, &offset, 0x40000000);
	append_u32(image, &offset, word0);
	append_u32(image, &offset, word1);

	append_u32(image, &offset, 0);
	append_u32(image, &offset, 0x40004e14);
	append_u32(image, &offset, word0 + word1);

	return offset;
}

static int capture_write(uint32_t addr, const unsigned char *data,
		size_t length, void *cb_data)
{
	struct fx3lafw_test_write *write;

	write = cb_data;
	fail_unless(length <= sizeof(write->data));

	write->addr = addr;
	write->length = length;
	write->calls++;
	memcpy(write->data, data, length);

	return SR_OK;
}

START_TEST(test_fx3_firmware_parse_valid)
{
	struct fx3lafw_test_write write;
	unsigned char image[64];
	uint32_t entry_addr;
	size_t length;
	int ret;

	memset(&write, 0, sizeof(write));
	memset(image, 0, sizeof(image));
	length = build_valid_image(image);

	ret = fx3lafw_parse_firmware_image(image, length, capture_write,
		&write, &entry_addr);

	fail_unless(ret == SR_OK);
	fail_unless(entry_addr == 0x40004e14);
	fail_unless(write.calls == 1);
	fail_unless(write.addr == 0x40000000);
	fail_unless(write.length == 8);
	fail_unless(!memcmp(write.data, image + 12, write.length));
}
END_TEST

START_TEST(test_fx3_firmware_parse_bad_header)
{
	unsigned char image[64];
	uint32_t entry_addr;
	size_t length;
	int ret;

	memset(image, 0, sizeof(image));
	length = build_valid_image(image);
	image[0] = 'B';

	ret = fx3lafw_parse_firmware_image(image, length, NULL, NULL,
		&entry_addr);

	fail_unless(ret == SR_ERR);
}
END_TEST

START_TEST(test_fx3_firmware_parse_bad_checksum)
{
	unsigned char image[64];
	uint32_t entry_addr;
	size_t length;
	int ret;

	memset(image, 0, sizeof(image));
	length = build_valid_image(image);
	image[length - 1] ^= 0x80;

	ret = fx3lafw_parse_firmware_image(image, length, NULL, NULL,
		&entry_addr);

	fail_unless(ret == SR_ERR);
}
END_TEST

START_TEST(test_fx3_firmware_parse_truncated)
{
	unsigned char image[64];
	uint32_t entry_addr;
	size_t length;
	int ret;

	memset(image, 0, sizeof(image));
	length = build_valid_image(image);

	ret = fx3lafw_parse_firmware_image(image, length - 3, NULL, NULL,
		&entry_addr);

	fail_unless(ret == SR_ERR);
}
END_TEST

START_TEST(test_fx3_samplerate_params)
{
	uint16_t sample_delay;
	uint8_t clock_flag;
	int ret;

	ret = fx3lafw_get_samplerate_params(SR_MHZ(192), &clock_flag,
		&sample_delay);
	fail_unless(ret == SR_OK);
	fail_unless(clock_flag == CMD_START_FLAGS_CLK_192MHZ);
	fail_unless(sample_delay == 0);

	ret = fx3lafw_get_samplerate_params(SR_MHZ(64), &clock_flag,
		&sample_delay);
	fail_unless(ret == SR_OK);
	fail_unless(clock_flag == CMD_START_FLAGS_CLK_192MHZ);
	fail_unless(sample_delay == 2);

	ret = fx3lafw_get_samplerate_params(SR_KHZ(123), &clock_flag,
		&sample_delay);
	fail_unless(ret == SR_ERR_SAMPLERATE);
}
END_TEST

Suite *suite_fx3lafw(void)
{
	Suite *s;
	TCase *tc;

	s = suite_create("fx3lafw");

	tc = tcase_create("firmware");
	tcase_add_test(tc, test_fx3_firmware_parse_valid);
	tcase_add_test(tc, test_fx3_firmware_parse_bad_header);
	tcase_add_test(tc, test_fx3_firmware_parse_bad_checksum);
	tcase_add_test(tc, test_fx3_firmware_parse_truncated);
	suite_add_tcase(s, tc);

	tc = tcase_create("protocol");
	tcase_add_test(tc, test_fx3_samplerate_params);
	suite_add_tcase(s, tc);

	return s;
}
