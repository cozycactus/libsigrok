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
#include "firmware.h"
#include "protocol.h"

#pragma pack(push, 1)

struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};

struct acquisition_status {
	uint8_t gpif_stat;
	uint8_t gpif_state;
	uint16_t reserved;
	uint32_t gpif_status;
	uint32_t gpif_intr;
	uint32_t pib_intr;
	uint32_t pib_error;
	uint32_t pib_sck0_status;
	uint32_t pib_sck0_intr;
	uint32_t pib_sck0_dscr;
	uint32_t pib_sck0_count;
	uint32_t pib_sck1_status;
	uint32_t pib_sck1_intr;
	uint32_t pib_sck1_dscr;
	uint32_t pib_sck1_count;
	uint32_t uib_sck2_status;
	uint32_t uib_sck2_intr;
	uint32_t uib_sck2_dscr;
	uint32_t uib_sck2_count;
	uint32_t pause_count;
	uint8_t pause_gpif_stat;
	uint8_t pause_gpif_state;
	uint16_t pause_reserved;
	uint32_t pause_gpif_status;
	uint32_t pause_pib_sck0_status;
	uint32_t pause_pib_sck0_dscr;
	uint32_t pause_pib_sck0_count;
	uint32_t pause_pib_sck1_status;
	uint32_t pause_pib_sck1_dscr;
	uint32_t pause_pib_sck1_count;
	uint32_t pause_uib_sck2_status;
	uint32_t pause_uib_sck2_dscr;
	uint32_t pause_uib_sck2_count;
};

#pragma pack(pop)

#define USB_TIMEOUT		100
#define FX3_UPLOAD_TIMEOUT	5000
#define FX3_MAX_WRITE_SIZE	(2 * 1024)

static int command_get_fw_version(libusb_device_handle *devhdl,
		struct version_info *vi)
{
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_FW_VERSION, 0x0000, 0x0000,
		(unsigned char *)vi, sizeof(struct version_info), USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get firmware version: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}
	if (ret != sizeof(struct version_info)) {
		sr_err("Short firmware version response: %d/%zu bytes.",
			ret, sizeof(struct version_info));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_get_revid_version(const struct sr_dev_inst *sdi,
		uint8_t *revid)
{
	struct sr_usb_dev_inst *usb;
	int ret;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_REVID_VERSION, 0x0000, 0x0000,
		revid, 1, USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get REVID: %s.", libusb_error_name(ret));
		return SR_ERR;
	}
	if (ret != 1) {
		sr_err("Short REVID response: %d/1 bytes.", ret);
		return SR_ERR;
	}

	return SR_OK;
}

static int command_stop_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	int ret;

	if (!sdi || !sdi->conn)
		return SR_ERR_ARG;

	usb = sdi->conn;
	if (!usb->devhdl)
		return SR_ERR_ARG;

	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, CMD_STOP, 0x0000, 0x0000, NULL, 0,
		USB_TIMEOUT);
	if (ret < 0) {
		if (ret == LIBUSB_ERROR_PIPE) {
			sr_dbg("Firmware does not support stop command.");
			return SR_OK;
		}
		sr_warn("Unable to send stop command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}
	if (ret != 0) {
		sr_warn("Unexpected stop command response: %d bytes.", ret);
		return SR_ERR;
	}

	return SR_OK;
}

static int command_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct cmd_start_acquisition cmd;
	uint16_t sample_delay;
	uint8_t clock_flag;
	int ret;

	devc = sdi->priv;
	usb = sdi->conn;

	if (devc->external_clock) {
		clock_flag = CMD_START_FLAGS_CLK_192MHZ;
		sample_delay = 0;
	} else {
		if ((ret = fx3lafw_get_samplerate_params(devc->cur_samplerate,
				&clock_flag, &sample_delay)) != SR_OK) {
			sr_err("Unable to sample at %" PRIu64 "Hz.",
				devc->cur_samplerate);
			return ret;
		}
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.flags = clock_flag;
	cmd.sample_delay_h = (sample_delay >> 8) & 0xff;
	cmd.sample_delay_l = sample_delay & 0xff;

	switch (devc->unitsize) {
	case 1:
		cmd.flags |= CMD_START_FLAGS_SAMPLE_8BIT;
		break;
	case 2:
		cmd.flags |= CMD_START_FLAGS_SAMPLE_16BIT;
		break;
	case 3:
		cmd.flags |= CMD_START_FLAGS_SAMPLE_24BIT;
		break;
	case 4:
		cmd.flags |= CMD_START_FLAGS_SAMPLE_32BIT;
		break;
	default:
		return SR_ERR_BUG;
	}

	if (devc->external_clock) {
		cmd.flags |= CMD_START_FLAGS_EXT_CLOCK;
		if (devc->clock_edge == FX3LAFW_CLOCK_EDGE_FALLING)
			cmd.flags |= CMD_START_FLAGS_CLK_INVERT;
	}

	sr_dbg("Starting acquisition: samplerate=%" PRIu64
		", unitsize=%" PRIu8 ", delay=%" PRIu16
		", external_clock=%d, clock_edge=%s, flags=0x%02x.",
		devc->cur_samplerate, devc->unitsize, sample_delay,
		devc->external_clock,
		devc->clock_edge == FX3LAFW_CLOCK_EDGE_FALLING ?
			"falling" : "rising",
		cmd.flags);

	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, CMD_START, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send start command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}
	if (ret != sizeof(cmd)) {
		sr_err("Short start command transfer: %d/%zu bytes.",
			ret, sizeof(cmd));
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int fx3lafw_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor desc;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct version_info vi;
	int ret, i, device_count;
	uint8_t revid;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
			libusb_error_name(device_count));
		return SR_ERR;
	}

	ret = SR_ERR;
	for (i = 0; i < device_count; i++) {
		if (libusb_get_device_descriptor(devlist[i], &desc) < 0)
			continue;

		if (desc.idVendor != devc->profile->vid ||
				desc.idProduct != devc->profile->pid)
			continue;

		if (usb_get_port_path(devlist[i], connection_id,
				sizeof(connection_id)) < 0)
			continue;

		if (sdi->connection_id && strcmp(sdi->connection_id,
				connection_id))
			continue;

		ret = libusb_open(devlist[i], &usb->devhdl);
		if (ret < 0) {
			sr_err("Failed to open device: %s.",
				libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (usb->address == 0xff)
			usb->address = libusb_get_device_address(devlist[i]);

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl,
					USB_INTERFACE) == 1) {
				ret = libusb_detach_kernel_driver(usb->devhdl,
					USB_INTERFACE);
				if (ret < 0) {
					sr_err("Failed to detach kernel driver: %s.",
						libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		ret = command_get_fw_version(usb->devhdl, &vi);
		if (ret != SR_OK) {
			sr_err("Failed to get firmware version.");
			break;
		}

		ret = command_get_revid_version(sdi, &revid);
		if (ret != SR_OK) {
			sr_err("Failed to get REVID.");
			break;
		}

		if (vi.major != FX3LAFW_REQUIRED_VERSION_MAJOR) {
			sr_err("Expected firmware version %d.x, got %d.%d.",
				FX3LAFW_REQUIRED_VERSION_MAJOR, vi.major,
				vi.minor);
			ret = SR_ERR;
			break;
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), "
			"interface %d, firmware %d.%d, REVID=%d.",
			usb->bus, usb->address, connection_id, USB_INTERFACE,
			vi.major, vi.minor, revid);
		ret = SR_OK;
		break;
	}

	if (ret != SR_OK && usb && usb->devhdl) {
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV struct dev_context *fx3lafw_dev_new(void)
{
	struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->profile = NULL;
	devc->fw_updated = 0;
	devc->cur_samplerate = 0;
	devc->limit_frames = 1;
	devc->limit_samples = 0;
	devc->capture_ratio = 0;
	devc->external_clock = FALSE;
	devc->clock_edge = FX3LAFW_CLOCK_EDGE_RISING;
	devc->unitsize = 1;
	devc->stl = NULL;

	return devc;
}

SR_PRIV void fx3lafw_abort_acquisition(struct dev_context *devc)
{
	unsigned int i;

	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers; i > 0; i--) {
		if (devc->transfers[i - 1])
			libusb_cancel_transfer(devc->transfers[i - 1]);
	}
}

static void clear_acquisition_allocations(struct dev_context *devc)
{
	devc->num_transfers = 0;
	g_free(devc->transfers);
	devc->transfers = NULL;

	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);
	usb_source_remove(sdi->session, devc->ctx);
	clear_acquisition_allocations(devc);
}

static void maybe_finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;
	if (devc->submitted_transfers == 0 && !devc->status_transfer)
		finish_acquisition(sdi);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	maybe_finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	ret = libusb_submit_transfer(transfer);
	if (ret == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s.", __func__, libusb_error_name(ret));
	free_transfer(transfer);
}

static const char *transfer_status_name(enum libusb_transfer_status status)
{
	switch (status) {
	case LIBUSB_TRANSFER_COMPLETED:
		return "COMPLETED";
	case LIBUSB_TRANSFER_ERROR:
		return "ERROR";
	case LIBUSB_TRANSFER_TIMED_OUT:
		return "TIMED_OUT";
	case LIBUSB_TRANSFER_CANCELLED:
		return "CANCELLED";
	case LIBUSB_TRANSFER_STALL:
		return "STALL";
	case LIBUSB_TRANSFER_NO_DEVICE:
		return "NO_DEVICE";
	case LIBUSB_TRANSFER_OVERFLOW:
		return "OVERFLOW";
	default:
		return "UNKNOWN";
	}
}

static void log_acquisition_status(const struct acquisition_status *status)
{
	sr_err("Acquisition status: gpif_stat=%u gpif_state=%u "
		"gpif_status=0x%08" PRIx32 " gpif_intr=0x%08" PRIx32
		" pib_intr=0x%08" PRIx32 " pib_error=0x%08" PRIx32 ".",
		status->gpif_stat, status->gpif_state, status->gpif_status,
		status->gpif_intr, status->pib_intr, status->pib_error);
	sr_err("DMA status: pib0 status=0x%08" PRIx32
		" intr=0x%08" PRIx32 " dscr=0x%08" PRIx32
		" count=0x%08" PRIx32 " pib1 status=0x%08" PRIx32
		" intr=0x%08" PRIx32 " dscr=0x%08" PRIx32
		" count=0x%08" PRIx32 " uib2 status=0x%08" PRIx32
		" intr=0x%08" PRIx32 " dscr=0x%08" PRIx32
		" count=0x%08" PRIx32 ".",
		status->pib_sck0_status, status->pib_sck0_intr,
		status->pib_sck0_dscr, status->pib_sck0_count,
		status->pib_sck1_status, status->pib_sck1_intr,
		status->pib_sck1_dscr, status->pib_sck1_count,
		status->uib_sck2_status, status->uib_sck2_intr,
		status->uib_sck2_dscr, status->uib_sck2_count);
	sr_err("Pause snapshot: count=%" PRIu32 " gpif_stat=%u gpif_state=%u"
		" gpif_status=0x%08" PRIx32
		" pib0 status=0x%08" PRIx32 " dscr=0x%08" PRIx32
		" count=0x%08" PRIx32 " pib1 status=0x%08" PRIx32
		" dscr=0x%08" PRIx32 " count=0x%08" PRIx32
		" uib2 status=0x%08" PRIx32 " dscr=0x%08" PRIx32
		" count=0x%08" PRIx32 ".",
		status->pause_count, status->pause_gpif_stat,
		status->pause_gpif_state, status->pause_gpif_status,
		status->pause_pib_sck0_status, status->pause_pib_sck0_dscr,
		status->pause_pib_sck0_count,
		status->pause_pib_sck1_status, status->pause_pib_sck1_dscr,
		status->pause_pib_sck1_count,
		status->pause_uib_sck2_status, status->pause_uib_sck2_dscr,
		status->pause_uib_sck2_count);
}

static void LIBUSB_CALL receive_status_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct acquisition_status *status;

	sdi = transfer->user_data;
	devc = sdi->priv;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
			transfer->actual_length == sizeof(*status)) {
		status = (struct acquisition_status *)
			libusb_control_transfer_get_data(transfer);
		log_acquisition_status(status);
	} else {
		sr_err("Acquisition status transfer failed: status %s, "
			"actual_length %d.",
			transfer_status_name(transfer->status),
			transfer->actual_length);
	}

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);
	devc->status_transfer = NULL;
	maybe_finish_acquisition(sdi);
}

static void submit_status_transfer(struct sr_dev_inst *sdi,
	libusb_transfer_cb_fn callback)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	unsigned char *buf;
	int ret;

	devc = sdi->priv;
	if (devc->status_transfer)
		return;

	usb = sdi->conn;
	if (!usb || !usb->devhdl)
		return;

	buf = g_try_malloc0(LIBUSB_CONTROL_SETUP_SIZE +
		sizeof(struct acquisition_status));
	if (!buf) {
		sr_err("USB acquisition status buffer malloc failed.");
		return;
	}

	transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		g_free(buf);
		return;
	}

	libusb_fill_control_setup(buf, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_ACQ_STATUS, 0x0000, 0x0000,
		sizeof(struct acquisition_status));
	libusb_fill_control_transfer(transfer, usb->devhdl, buf,
		callback, (void *)sdi, USB_TIMEOUT);
	ret = libusb_submit_transfer(transfer);
	if (ret != LIBUSB_SUCCESS) {
		sr_err("Unable to submit acquisition status transfer: %s.",
			libusb_error_name(ret));
		libusb_free_transfer(transfer);
		g_free(buf);
		return;
	}

	devc->status_transfer = transfer;
}

static void request_acquisition_status(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;
	if (devc->status_requested)
		return;
	devc->status_requested = TRUE;

	submit_status_transfer(sdi, receive_status_transfer);
}

static void LIBUSB_CALL receive_error_check_transfer(
	struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct acquisition_status *status;

	sdi = transfer->user_data;
	devc = sdi->priv;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
			transfer->actual_length == sizeof(*status)) {
		status = (struct acquisition_status *)
			libusb_control_transfer_get_data(transfer);
		if (status->pib_error) {
			sr_err("Device reported an acquisition fault "
				"(PIB_ERROR=0x%08" PRIx32 "); aborting instead "
				"of waiting out the remaining USB timeouts.",
				status->pib_error);
			log_acquisition_status(status);
			fx3lafw_abort_acquisition(devc);
		}
	} else {
		sr_dbg("Early error check transfer failed: status %s, "
			"actual_length %d.",
			transfer_status_name(transfer->status),
			transfer->actual_length);
	}

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);
	devc->status_transfer = NULL;
	maybe_finish_acquisition(sdi);
}

static void send_logic_data(struct sr_dev_inst *sdi, uint8_t *data,
		size_t length, size_t sample_width)
{
	const struct sr_datafeed_logic logic = {
		.length = length,
		.unitsize = sample_width,
		.data = data
	};
	const struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic
	};

	sr_session_send(sdi, &packet);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	gboolean packet_has_error;
	unsigned int num_samples;
	int trigger_offset, cur_sample_count, unitsize, processed_samples;
	int pre_trigger_samples;
	gboolean frame_ended, final_frame;

	sdi = transfer->user_data;
	devc = sdi->priv;
	packet_has_error = FALSE;

	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	sr_dbg("receive_transfer(): status %s received %d bytes.",
		transfer_status_name(transfer->status), transfer->actual_length);

	unitsize = devc->unitsize;
	cur_sample_count = transfer->actual_length / unitsize;
	processed_samples = 0;

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		fx3lafw_abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT:
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count >=
				EARLY_ERROR_CHECK_EMPTY_TRANSFERS &&
				!devc->error_check_done) {
			devc->error_check_done = TRUE;
			submit_status_transfer(sdi,
				receive_error_check_transfer);
		}
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			sr_err("Aborting acquisition after %d empty/error USB "
				"transfers; last transfer status %s, "
				"actual_length %d; requested data was not fully "
				"received.", devc->empty_transfer_count,
				transfer_status_name(transfer->status),
				transfer->actual_length);
			request_acquisition_status(sdi);
			fx3lafw_abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	}
	devc->empty_transfer_count = 0;
	devc->error_check_done = FALSE;

check_trigger:
	if (devc->trigger_fired) {
		if (!devc->limit_samples ||
				devc->sent_samples < devc->limit_samples) {
			num_samples = cur_sample_count - processed_samples;
			if (devc->limit_samples &&
					devc->sent_samples + num_samples >
					devc->limit_samples)
				num_samples = devc->limit_samples -
					devc->sent_samples;

			send_logic_data(sdi, transfer->buffer +
				processed_samples * unitsize,
				num_samples * unitsize, unitsize);
			devc->sent_samples += num_samples;
			processed_samples += num_samples;
		}
	} else {
		trigger_offset = soft_trigger_logic_check(devc->stl,
			transfer->buffer + processed_samples * unitsize,
			transfer->actual_length - processed_samples * unitsize,
			&pre_trigger_samples);
		if (trigger_offset > -1) {
			std_session_send_df_frame_begin(sdi);
			devc->sent_samples += pre_trigger_samples;
			num_samples = cur_sample_count - processed_samples -
				trigger_offset;
			if (devc->limit_samples &&
					devc->sent_samples + num_samples >
					devc->limit_samples)
				num_samples = devc->limit_samples -
					devc->sent_samples;

			send_logic_data(sdi, transfer->buffer +
				(processed_samples + trigger_offset) * unitsize,
				num_samples * unitsize, unitsize);
			devc->sent_samples += num_samples;
			processed_samples += trigger_offset + num_samples;
			devc->trigger_fired = TRUE;
		}
	}

	frame_ended = devc->limit_samples &&
		devc->sent_samples >= devc->limit_samples;
	final_frame = devc->limit_frames &&
		devc->num_frames >= devc->limit_frames - 1;

	if (frame_ended) {
		devc->num_frames++;
		devc->sent_samples = 0;
		devc->trigger_fired = FALSE;
		std_session_send_df_frame_end(sdi);

		if (processed_samples < cur_sample_count && !final_frame) {
			if (devc->stl)
				devc->stl->cur_stage = 0;
			else {
				std_session_send_df_frame_begin(sdi);
				devc->trigger_fired = TRUE;
			}
			goto check_trigger;
		}
	}

	if (frame_ended && final_frame) {
		fx3lafw_abort_acquisition(devc);
		free_transfer(transfer);
	} else {
		resubmit_transfer(transfer);
	}
}

static int configure_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	uint8_t unitsize;
	int ret;

	devc = sdi->priv;
	if ((ret = fx3lafw_channel_unitsize(sdi, &unitsize)) != SR_OK) {
		sr_err("Need at least one enabled logic channel.");
		return ret;
	}
	devc->unitsize = unitsize;

	return SR_OK;
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	drvc = cb_data;
	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
}

static int start_transfers(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_trigger *trigger;
	struct libusb_transfer *transfer;
	unsigned char *buf;
	unsigned int i;
	int ret;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;
	devc->submitted_transfers = 0;
	devc->num_transfers = NUM_SIMUL_TRANSFERS;
	devc->status_transfer = NULL;
	devc->status_requested = FALSE;
	devc->error_check_done = FALSE;

	if ((trigger = sr_session_trigger_get(sdi->session))) {
		int pre_trigger_samples;

		pre_trigger_samples = 0;
		if (devc->limit_samples > 0)
			pre_trigger_samples = (devc->capture_ratio *
				devc->limit_samples) / 100;
		devc->stl = soft_trigger_logic_new_with_unitsize(sdi, trigger,
			pre_trigger_samples, devc->unitsize);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		devc->trigger_fired = FALSE;
	} else {
		std_session_send_df_frame_begin(sdi);
		devc->trigger_fired = TRUE;
	}

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) *
		devc->num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		ret = SR_ERR_MALLOC;
		goto err;
	}

	/*
	 * Allocate every transfer and its buffer before submitting any of
	 * them. The session core does not pump the USB event loop when an
	 * acquisition fails to start, so transfers already handed to libusb
	 * could not be reaped. Allocating up front means the common failure
	 * (out of memory) leaves nothing in flight and can be freed
	 * synchronously below.
	 */
	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc(TRANSFER_SIZE);
		if (!buf) {
			sr_err("USB transfer buffer malloc failed.");
			ret = SR_ERR_MALLOC;
			goto err;
		}

		transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			g_free(buf);
			ret = SR_ERR_MALLOC;
			goto err;
		}

		libusb_fill_bulk_transfer(transfer, usb->devhdl,
			USB_EP_DATA_IN, buf, TRANSFER_SIZE,
			receive_transfer, (void *)sdi, TRANSFER_TIMEOUT_MS);
		devc->transfers[i] = transfer;
	}

	for (i = 0; i < devc->num_transfers; i++) {
		ret = libusb_submit_transfer(devc->transfers[i]);
		if (ret != 0) {
			sr_err("Failed to submit transfer: %s.",
				libusb_error_name(ret));
			ret = SR_ERR;
			goto err;
		}
		devc->submitted_transfers++;
	}

	std_session_send_df_header(sdi);

	return SR_OK;

err:
	/*
	 * Transfers at and beyond submitted_transfers were allocated but
	 * never submitted, so free them directly. Lower indices are already
	 * in flight; cancel them and let their completion callbacks free them
	 * through finish_acquisition() if the event loop is still running.
	 */
	if (devc->transfers) {
		for (i = devc->submitted_transfers; i < devc->num_transfers; i++) {
			if (!devc->transfers[i])
				continue;
			g_free(devc->transfers[i]->buffer);
			libusb_free_transfer(devc->transfers[i]);
			devc->transfers[i] = NULL;
		}
	}

	if (devc->submitted_transfers)
		fx3lafw_abort_acquisition(devc);
	else
		clear_acquisition_allocations(devc);

	return ret;
}

SR_PRIV int fx3lafw_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	int ret;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;

	devc->ctx = drvc->sr_ctx;
	devc->num_frames = 0;
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;

	if ((ret = configure_channels(sdi)) != SR_OK)
		return ret;

	if (!fx3lafw_samplerate_supported_for_unitsize(devc->cur_samplerate,
			devc->unitsize)) {
		sr_err("%" PRIu64 "Hz exceeds the sustained limit for "
			"%" PRIu8 "-byte samples; maximum is %" PRIu64 "Hz.",
			devc->cur_samplerate, devc->unitsize,
			fx3lafw_max_samplerate_for_unitsize(devc->unitsize));
		return SR_ERR_SAMPLERATE;
	}

	(void)command_stop_acquisition(sdi);

	usb_source_add(sdi->session, devc->ctx, TRANSFER_TIMEOUT_MS,
		receive_data, drvc);

	ret = start_transfers(sdi);
	if (ret != SR_OK) {
		usb_source_remove(sdi->session, devc->ctx);
		return ret;
	}

	ret = command_start_acquisition(sdi);
	if (ret != SR_OK) {
		fx3lafw_abort_acquisition(devc);
		return ret;
	}

	return SR_OK;
}

SR_PRIV int fx3lafw_stop_acquisition(const struct sr_dev_inst *sdi)
{
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;

	(void)command_stop_acquisition(sdi);
	fx3lafw_abort_acquisition(sdi->priv);

	return SR_OK;
}

static int fx3_ram_write(libusb_device_handle *hdl, uint32_t addr,
		const unsigned char *data, size_t length)
{
	size_t offset, chunksize;
	int ret;

	offset = 0;
	while (offset < length) {
		chunksize = MIN(length - offset, FX3_MAX_WRITE_SIZE);
		ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, 0xa0, (addr + offset) & 0xffff,
			(addr + offset) >> 16, (unsigned char *)data + offset,
			chunksize, FX3_UPLOAD_TIMEOUT);
		if (ret < 0) {
			sr_err("Unable to send firmware to device: %s.",
				libusb_error_name(ret));
			return SR_ERR;
		}
		if ((size_t)ret != chunksize) {
			sr_err("Short firmware write at 0x%08" PRIx32
				": %d/%zu bytes.", addr + (uint32_t)offset,
				ret, chunksize);
			return SR_ERR;
		}
		offset += chunksize;
	}

	return SR_OK;
}

static int fx3_ram_write_cb(uint32_t addr, const unsigned char *data,
		size_t length, void *cb_data)
{
	return fx3_ram_write(cb_data, addr, data, length);
}

static int fx3_install_firmware(struct sr_context *ctx,
		libusb_device_handle *hdl, const char *name)
{
	unsigned char *firmware;
	size_t length;
	uint32_t entry_addr;
	int ret;

	firmware = sr_resource_load(ctx, SR_RESOURCE_FIRMWARE,
		name, &length, (512 << 10));
	if (!firmware)
		return SR_ERR;

	sr_info("Uploading firmware '%s'.", name);

	ret = fx3lafw_parse_firmware_image(firmware, length, fx3_ram_write_cb,
		hdl, &entry_addr);
	if (ret != SR_OK) {
		sr_err("Invalid FX3 firmware image.");
		g_free(firmware);
		return ret;
	}

	ret = libusb_control_transfer(hdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		0xa0, entry_addr & 0xffff, entry_addr >> 16, NULL, 0,
		FX3_UPLOAD_TIMEOUT);
	if (ret < 0)
		sr_dbg("Ignored FX3 launch transfer status: %s.",
			libusb_error_name(ret));
	g_free(firmware);
	sr_info("Firmware upload done.");

	return SR_OK;
}

SR_PRIV int fx3_upload_firmware(struct sr_context *ctx, libusb_device *dev,
		int configuration, const char *name)
{
	struct libusb_device_handle *hdl;
	int ret;

	sr_info("Uploading firmware to device on %d.%d.",
		libusb_get_bus_number(dev), libusb_get_device_address(dev));

	ret = libusb_open(dev, &hdl);
	if (ret < 0) {
		sr_err("Failed to open device: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	ret = libusb_set_configuration(hdl, configuration);
	if (ret < 0 && ret != LIBUSB_ERROR_BUSY) {
		sr_warn("Unable to set configuration: %s, continuing.",
			libusb_error_name(ret));
	}

	ret = fx3_install_firmware(ctx, hdl, name);

	libusb_close(hdl);

	return ret;
}
