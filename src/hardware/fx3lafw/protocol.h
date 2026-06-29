/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2022 Ruslan Migirov <trapi78@gmail.com>
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

#ifndef LIBSIGROK_HARDWARE_FX3LAFW_PROTOCOL_H
#define LIBSIGROK_HARDWARE_FX3LAFW_PROTOCOL_H

#include <glib.h>
#include <stdint.h>
#include <string.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "fx3lafw"

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1
#define USB_EP_DATA_IN		(2 | LIBUSB_ENDPOINT_IN)

#define MAX_RENUM_DELAY_MS	3000
#define NUM_SIMUL_TRANSFERS	128
#define MAX_EMPTY_TRANSFERS	(NUM_SIMUL_TRANSFERS * 2)
#define TRANSFER_SIZE		(256 * 1024)
#define TRANSFER_TIMEOUT_MS	500

#define FX3LAFW_REQUIRED_VERSION_MAJOR	1

#define DEV_CAPS_16BIT_POS	0
#define DEV_CAPS_AX_ANALOG_POS	1
#define DEV_CAPS_FX3_POS	2
#define DEV_CAPS_24BIT_POS	3
#define DEV_CAPS_32BIT_POS	4

#define DEV_CAPS_16BIT		(1 << DEV_CAPS_16BIT_POS)
#define DEV_CAPS_AX_ANALOG	(1 << DEV_CAPS_AX_ANALOG_POS)
#define DEV_CAPS_FX3		(1 << DEV_CAPS_FX3_POS)
#define DEV_CAPS_24BIT		(1 << DEV_CAPS_24BIT_POS)
#define DEV_CAPS_32BIT		(1 << DEV_CAPS_32BIT_POS)

/* Protocol commands. Keep in sync with fx3lafw/command.h. */
#define CMD_GET_FW_VERSION		0xb0
#define CMD_START			0xb1
#define CMD_GET_REVID_VERSION		0xb2
#define CMD_STOP			0xb3
#define CMD_GET_ACQ_STATUS		0xb4

#define CMD_START_FLAGS_SUPERWIDE_POS	3
#define CMD_START_FLAGS_CLK_CTL2_POS	4
#define CMD_START_FLAGS_WIDE_POS	5
#define CMD_START_FLAGS_CLK_SRC_POS	6

#define CMD_START_FLAGS_EXT_CLOCK	(1 << 0)
#define CMD_START_FLAGS_CLK_INVERT	(1 << 1)
#define CMD_START_FLAGS_CLK_CTL2	(1 << CMD_START_FLAGS_CLK_CTL2_POS)
#define CMD_START_FLAGS_SAMPLE_8BIT	(0 << CMD_START_FLAGS_WIDE_POS)
#define CMD_START_FLAGS_SAMPLE_16BIT	(1 << CMD_START_FLAGS_WIDE_POS)
#define CMD_START_FLAGS_SAMPLE_24BIT	((0 << CMD_START_FLAGS_WIDE_POS) | \
					 (1 << CMD_START_FLAGS_SUPERWIDE_POS))
#define CMD_START_FLAGS_SAMPLE_32BIT	((1 << CMD_START_FLAGS_WIDE_POS) | \
					 (1 << CMD_START_FLAGS_SUPERWIDE_POS))

#define CMD_START_FLAGS_CLK_30MHZ	(0 << CMD_START_FLAGS_CLK_SRC_POS)
#define CMD_START_FLAGS_CLK_48MHZ	(1 << CMD_START_FLAGS_CLK_SRC_POS)
#define CMD_START_FLAGS_CLK_192MHZ	(2 << CMD_START_FLAGS_CLK_SRC_POS)
#define CMD_START_FLAGS_CLK_80MHZ	(3 << CMD_START_FLAGS_CLK_SRC_POS)

#define FX3LAFW_MAX_SAMPLE_BYTES_PER_SEC	SR_MHZ(320)

enum fx3lafw_clock_edge {
	FX3LAFW_CLOCK_EDGE_RISING,
	FX3LAFW_CLOCK_EDGE_FALLING,
};

static inline int fx3lafw_get_samplerate_params(uint64_t samplerate,
		uint8_t *clock_flag, uint16_t *sample_delay)
{
	static const struct {
		uint64_t clock;
		uint8_t flag;
	} clocks[] = {
		{ SR_MHZ(48), CMD_START_FLAGS_CLK_48MHZ },
		{ SR_MHZ(30), CMD_START_FLAGS_CLK_30MHZ },
		{ SR_MHZ(80), CMD_START_FLAGS_CLK_80MHZ },
		{ SR_MHZ(192), CMD_START_FLAGS_CLK_192MHZ },
	};
	uint64_t divisor;
	size_t i;

	if (!samplerate)
		return SR_ERR_SAMPLERATE;

	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		if (clocks[i].clock % samplerate)
			continue;
		divisor = clocks[i].clock / samplerate;
		if (divisor == 0 || divisor - 1 > UINT16_MAX)
			continue;
		*clock_flag = clocks[i].flag;
		*sample_delay = divisor - 1;
		return SR_OK;
	}

	return SR_ERR_SAMPLERATE;
}

static inline uint64_t fx3lafw_max_samplerate_for_unitsize(uint8_t unitsize)
{
	switch (unitsize) {
	case 1:
	case 2:
		return FX3LAFW_MAX_SAMPLE_BYTES_PER_SEC / unitsize;
	case 3:
	case 4:
		return SR_MHZ(80);
	default:
		return 0;
	}
}

static inline gboolean fx3lafw_samplerate_supported_for_unitsize(
		uint64_t samplerate, uint8_t unitsize)
{
	uint64_t max_samplerate;

	max_samplerate = fx3lafw_max_samplerate_for_unitsize(unitsize);
	return max_samplerate && samplerate <= max_samplerate;
}

static inline int fx3lafw_channel_unitsize(const struct sr_dev_inst *sdi,
		uint8_t *unitsize)
{
	const GSList *l;
	struct sr_channel *ch;
	int max_enabled_logic;

	if (!sdi || !unitsize)
		return SR_ERR_ARG;

	max_enabled_logic = -1;

	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled)
			continue;
		if (ch->index > max_enabled_logic)
			max_enabled_logic = ch->index;
	}

	if (max_enabled_logic < 0)
		return SR_ERR;

	if (max_enabled_logic > 23)
		*unitsize = 4;
	else if (max_enabled_logic > 15)
		*unitsize = 3;
	else if (max_enabled_logic > 7)
		*unitsize = 2;
	else
		*unitsize = 1;

	return SR_OK;
}

struct fx3lafw_profile {
	uint16_t vid;
	uint16_t pid;

	const char *vendor;
	const char *model;
	const char *model_version;

	const char *firmware;

	uint32_t dev_caps;

	const char *usb_manufacturer;
	const char *usb_product;
};

struct dev_context {
	const struct fx3lafw_profile *profile;
	/*
	 * Since we can't keep track of an fx3lafw device after upgrading
	 * the firmware (it renumerates into a different device address
	 * after the upgrade), this timestamp drives a bounded wait in open().
	 */
	int64_t fw_updated;

	const uint64_t *samplerates;
	int num_samplerates;

	uint64_t cur_samplerate;
	uint64_t limit_frames;
	uint64_t limit_samples;
	uint64_t capture_ratio;
	gboolean external_clock;
	enum fx3lafw_clock_edge clock_edge;

	gboolean trigger_fired;
	gboolean acq_aborted;
	uint8_t unitsize;
	struct soft_trigger_logic *stl;

	uint64_t num_frames;
	uint64_t sent_samples;
	int submitted_transfers;
	int empty_transfer_count;

	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	struct libusb_transfer *status_transfer;
	gboolean status_requested;
	struct sr_context *ctx;
};

SR_PRIV int fx3lafw_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di);
SR_PRIV struct dev_context *fx3lafw_dev_new(void);
SR_PRIV int fx3lafw_start_acquisition(const struct sr_dev_inst *sdi);
SR_PRIV int fx3lafw_stop_acquisition(const struct sr_dev_inst *sdi);
SR_PRIV void fx3lafw_abort_acquisition(struct dev_context *devc);
SR_PRIV int fx3_upload_firmware(struct sr_context *ctx, libusb_device *dev,
		int configuration, const char *name);

#endif
