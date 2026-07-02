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

#include <config.h>
#include <inttypes.h>
#include "protocol.h"

static const struct fx3lafw_profile supported_fx3[] = {
	/*
	 * Cypress SuperSpeed Explorer Kit (CYUSB3KIT-003)
	 */
	{ 0x04b4, 0x00f3, "Cypress", "SuperSpeed Explorer Kit", NULL,
		"fx3lafw-cypress-fx3.fw",
		DEV_CAPS_FX3 | DEV_CAPS_32BIT, NULL, NULL },

	ALL_ZERO
};

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_EXTERNAL_CLOCK | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CLOCK_EDGE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const char *clock_edges[] = {
	[FX3LAFW_CLOCK_EDGE_RISING] = "rising",
	[FX3LAFW_CLOCK_EDGE_FALLING] = "falling",
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

static const uint64_t samplerates[] = {
	SR_KHZ(200),
	SR_KHZ(250),
	SR_KHZ(500),
	SR_MHZ(1),
	SR_MHZ(2),
	SR_MHZ(3),
	SR_MHZ(4),
	SR_MHZ(6),
	SR_MHZ(8),
	SR_MHZ(12),
	SR_MHZ(16),
	SR_MHZ(24),
	SR_MHZ(32),
	SR_MHZ(48),
	SR_MHZ(64),
	SR_MHZ(80),
	FX3LAFW_89MHZ_SAMPLERATE,
	SR_MHZ(96),
	SR_MHZ(192),
};

static gboolean is_plausible(const struct libusb_device_descriptor *desc)
{
	size_t i;

	for (i = 0; supported_fx3[i].vid; i++) {
		if (desc->idVendor == supported_fx3[i].vid &&
				desc->idProduct == supported_fx3[i].pid)
			return TRUE;
	}

	return FALSE;
}

static int get_string_descriptor(libusb_device_handle *hdl, uint8_t index,
		char *buf, size_t size, const char *name)
{
	int ret;

	if (index == 0) {
		buf[0] = '\0';
		return SR_OK;
	}

	ret = libusb_get_string_descriptor_ascii(hdl, index,
		(unsigned char *)buf, size);
	if (ret < 0) {
		sr_warn("Failed to get %s string descriptor: %s.",
			name, libusb_error_name(ret));
		buf[0] = '\0';
	}

	return SR_OK;
}

static size_t num_logic_channels(const struct fx3lafw_profile *profile)
{
	if (profile->dev_caps & DEV_CAPS_32BIT)
		return 32;
	if (profile->dev_caps & DEV_CAPS_24BIT)
		return 24;
	if (profile->dev_caps & DEV_CAPS_16BIT)
		return 16;
	return 8;
}

static const struct fx3lafw_profile *find_profile(
		const struct libusb_device_descriptor *desc,
		const char *manufacturer, const char *product)
{
	size_t i;

	for (i = 0; supported_fx3[i].vid; i++) {
		if (desc->idVendor != supported_fx3[i].vid ||
				desc->idProduct != supported_fx3[i].pid)
			continue;
		if (supported_fx3[i].usb_manufacturer &&
				strcmp(manufacturer,
				supported_fx3[i].usb_manufacturer))
			continue;
		if (supported_fx3[i].usb_product &&
				strcmp(product, supported_fx3[i].usb_product))
			continue;
		return &supported_fx3[i];
	}

	return NULL;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	struct sr_config *src;
	const struct fx3lafw_profile *profile;
	GSList *l, *devices, *conn_devices;
	gboolean has_firmware;
	struct libusb_device_descriptor desc;
	libusb_device **devlist;
	libusb_device_handle *hdl;
	int ret, i, device_count;
	size_t j, channels;
	const char *conn;
	char manufacturer[64], product[64], serial_num[64], connection_id[64];
	char channel_name[16];

	drvc = di->context;
	conn = NULL;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}

	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;

	devices = NULL;
	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
			libusb_error_name(device_count));
		g_slist_free_full(conn_devices,
			(GDestroyNotify)sr_usb_dev_inst_free);
		return NULL;
	}

	for (i = 0; i < device_count; i++) {
		if (conn) {
			usb = NULL;
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i])
						&& usb->address ==
						libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				continue;
		}

		if (libusb_get_device_descriptor(devlist[i], &desc) < 0)
			continue;
		if (!is_plausible(&desc))
			continue;

		ret = libusb_open(devlist[i], &hdl);
		if (ret < 0) {
			sr_warn("Failed to open potential device with VID:PID "
				"%04x:%04x: %s.", desc.idVendor,
				desc.idProduct, libusb_error_name(ret));
			continue;
		}

		ret = get_string_descriptor(hdl, desc.iManufacturer,
			manufacturer, sizeof(manufacturer), "manufacturer");
		if (ret == SR_OK)
			ret = get_string_descriptor(hdl, desc.iProduct,
				product, sizeof(product), "product");
		if (ret == SR_OK)
			ret = get_string_descriptor(hdl, desc.iSerialNumber,
				serial_num, sizeof(serial_num), "serial number");
		libusb_close(hdl);
		if (ret != SR_OK)
			continue;

		if (usb_get_port_path(devlist[i], connection_id,
				sizeof(connection_id)) < 0)
			continue;

		profile = find_profile(&desc, manufacturer, product);
		if (!profile)
			continue;

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup(profile->vendor);
		sdi->model = g_strdup(profile->model);
		sdi->version = g_strdup(profile->model_version);
		sdi->serial_num = g_strdup(serial_num);
		sdi->connection_id = g_strdup(connection_id);

		devc = fx3lafw_dev_new();
		devc->profile = profile;
		devc->samplerates = samplerates;
		devc->num_samplerates = ARRAY_SIZE(samplerates);
		devc->usb_speed = libusb_get_device_speed(devlist[i]);
		sdi->priv = devc;
		devices = g_slist_append(devices, sdi);

		cg = sr_channel_group_new(sdi, "Logic", NULL);
		channels = num_logic_channels(profile);
		for (j = 0; j < channels; j++) {
			snprintf(channel_name, sizeof(channel_name), "D%zu", j);
			ch = sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE,
				channel_name);
			cg->channels = g_slist_append(cg->channels, ch);
		}

		has_firmware = usb_match_manuf_prod(devlist[i],
			"sigrok", "fx3lafw");
		if (has_firmware) {
			sr_dbg("Found an fx3lafw device.");
			sdi->status = SR_ST_INACTIVE;
			sdi->inst_type = SR_INST_USB;
			sdi->conn = sr_usb_dev_inst_new(
				libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i]), NULL);
		} else {
			ret = fx3_upload_firmware(drvc->sr_ctx, devlist[i],
				USB_CONFIGURATION, profile->firmware);
			if (ret == SR_OK) {
				devc->fw_updated = g_get_monotonic_time();
			} else {
				sr_err("Firmware upload failed for device %d.%d "
					"(logical), name %s.",
					libusb_get_bus_number(devlist[i]),
					libusb_get_device_address(devlist[i]),
					profile->firmware);
			}
			sdi->inst_type = SR_INST_USB;
			sdi->conn = sr_usb_dev_inst_new(
				libusb_get_bus_number(devlist[i]), 0xff, NULL);
		}
	}

	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	int ret;
	int64_t timediff_us, timediff_ms;

	if (!sdi)
		return SR_ERR_ARG;

	di = sdi->driver;
	devc = sdi->priv;
	usb = sdi->conn;
	if (!devc || !usb)
		return SR_ERR_ARG;

	ret = SR_ERR;
	if (devc->fw_updated > 0) {
		sr_info("Waiting for device to reset.");
		g_usleep(300 * 1000);
		timediff_ms = 0;
		while (timediff_ms < MAX_RENUM_DELAY_MS) {
			ret = fx3lafw_dev_open(sdi, di);
			if (ret == SR_OK)
				break;
			g_usleep(100 * 1000);

			timediff_us = g_get_monotonic_time() - devc->fw_updated;
			timediff_ms = timediff_us / 1000;
			sr_spew("Waited %" PRIi64 "ms.", timediff_ms);
		}
		if (ret != SR_OK) {
			sr_err("Device failed to renumerate.");
			return SR_ERR;
		}
		sr_info("Device came back after %" PRIi64 "ms.", timediff_ms);
	} else {
		sr_info("Firmware upload was not needed.");
		ret = fx3lafw_dev_open(sdi, di);
	}

	if (ret != SR_OK) {
		sr_err("Unable to open device.");
		return ret;
	}

	ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
	if (ret != 0) {
		switch (ret) {
		case LIBUSB_ERROR_BUSY:
			sr_err("Unable to claim USB interface. Another program "
				"or driver has already claimed it.");
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			break;
		default:
			sr_err("Unable to claim interface: %s.",
				libusb_error_name(ret));
			break;
		}
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
		return SR_ERR;
	}

	if (devc->cur_samplerate == 0)
		devc->cur_samplerate = devc->samplerates[0];

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	if (!sdi || !sdi->conn)
		return SR_ERR_ARG;

	usb = sdi->conn;
	if (!usb->devhdl)
		return SR_ERR_BUG;

	sr_info("Closing device on %d.%d (logical) / %s (physical) "
		"interface %d.", usb->bus, usb->address,
		sdi->connection_id, USB_INTERFACE);
	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	if (!devc)
		return SR_ERR_ARG;

	switch (key) {
	case SR_CONF_CONN:
		if (!sdi->conn)
			return SR_ERR_ARG;
		usb = sdi->conn;
		if (usb->address == 0xff)
			return SR_ERR;
		*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
		break;
	case SR_CONF_LIMIT_FRAMES:
		*data = g_variant_new_uint64(devc->limit_frames);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_EXTERNAL_CLOCK:
		*data = g_variant_new_boolean(devc->external_clock);
		break;
	case SR_CONF_CLOCK_EDGE:
		if (devc->clock_edge >= ARRAY_SIZE(clock_edges))
			return SR_ERR_BUG;
		*data = g_variant_new_string(clock_edges[devc->clock_edge]);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	int idx;
	uint64_t value;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	if (!devc)
		return SR_ERR_ARG;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		value = g_variant_get_uint64(data);
		if (!value)
			return SR_ERR_SAMPLERATE;
		idx = std_u64_idx(data, devc->samplerates,
			devc->num_samplerates);
		if (idx >= 0) {
			devc->cur_samplerate = devc->samplerates[idx];
			break;
		}
		devc->cur_samplerate = value;
		break;
	case SR_CONF_LIMIT_FRAMES:
		devc->limit_frames = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		value = g_variant_get_uint64(data);
		if (value > 100)
			return SR_ERR_ARG;
		devc->capture_ratio = value;
		break;
	case SR_CONF_EXTERNAL_CLOCK:
		devc->external_clock = g_variant_get_boolean(data);
		break;
	case SR_CONF_CLOCK_EDGE:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(clock_edges))) < 0)
			return SR_ERR_ARG;
		devc->clock_edge = idx;
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t *filtered_samplerates;
	uint64_t samplerate_steps[3];
	uint64_t max_samplerate;
	unsigned int filtered_count;
	uint8_t unitsize;
	int i;

	devc = (sdi) ? sdi->priv : NULL;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		if (cg)
			return SR_ERR_NA;
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts,
			drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		if (!devc)
			return SR_ERR_NA;
		if (devc->external_clock) {
			if (fx3lafw_channel_unitsize(sdi, &unitsize) != SR_OK)
				unitsize = 1;
			max_samplerate = fx3lafw_max_samplerate_for_usb_speed(
				unitsize, devc->usb_speed);
			samplerate_steps[0] = 1;
			samplerate_steps[1] = max_samplerate;
			samplerate_steps[2] = 1;
			*data = std_gvar_samplerates_steps(
				ARRAY_AND_SIZE(samplerate_steps));
			break;
		}
		if (fx3lafw_channel_unitsize(sdi, &unitsize) != SR_OK) {
			*data = std_gvar_samplerates(devc->samplerates,
				devc->num_samplerates);
			break;
		}

		filtered_samplerates = g_new(uint64_t, devc->num_samplerates);
		filtered_count = 0;
		for (i = 0; i < devc->num_samplerates; i++) {
			if (fx3lafw_samplerate_supported_for_usb_speed(
					devc->samplerates[i], unitsize,
					devc->usb_speed))
				filtered_samplerates[filtered_count++] =
					devc->samplerates[i];
		}
		*data = std_gvar_samplerates(filtered_samplerates,
			filtered_count);
		g_free(filtered_samplerates);
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	case SR_CONF_CLOCK_EDGE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(clock_edges));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	return fx3lafw_stop_acquisition(sdi);
}

static struct sr_dev_driver fx3lafw_driver_info = {
	.name = "fx3lafw",
	.longname = "fx3lafw (FX3 based logic analyzer)",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = fx3lafw_start_acquisition,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(fx3lafw_driver_info);
