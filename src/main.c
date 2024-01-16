/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <bluetooth/services/dfu_smp.h>
#include <zephyr/sys/byteorder.h>
#include <bluetooth/services/dfu_smp.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zcbor_encode.h>
#include <zcbor_decode.h>
#include <zcbor_common.h>


#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);



/* Mimimal number of ZCBOR encoder states to provide full encoder functionality. */
#define CBOR_ENCODER_STATE_NUM 2

/* Number of ZCBOR decoder states required to provide full decoder functionality plus one
 * more level for decoding nested map received in response to SMP echo command.
 */
#define CBOR_DECODER_STATE_NUM 3

#define CBOR_MAP_MAX_ELEMENT_CNT 2
#define CBOR_BUFFER_SIZE 128

#define SMP_ECHO_MAP_KEY_MAX_LEN 2
#define SMP_ECHO_MAP_VALUE_MAX_LEN 30

#define KEY_LIST_MASK  DK_BTN1_MSK
#define KEY_UPLOAD_MASK  DK_BTN2_MSK
#define KEY_TEST_MASK  DK_BTN3_MSK
#define KEY_CONFIRM_MASK  DK_BTN4_MSK

static struct bt_conn *default_conn;
static struct bt_dfu_smp dfu_smp;
static struct bt_gatt_exchange_params exchange_params;

struct k_work upload_work_item;
struct k_sem upload_sem;
K_SEM_DEFINE(upload_sem, 1, 1);

/* Buffer for response */
struct smp_buffer {
	struct bt_dfu_smp_header header;
	uint8_t payload[2500];
};
static struct smp_buffer smp_rsp_buff;

static char hash_value_secondary_slot[33];//={0xA0,0x72,0xA6,0x85,0x86,0x7B,0x88,0xE2,0xFC,0xBD,0x7F,0x47,0xEC,0x45,0x3D,0x3F,\
										   //0x16,0x6A,0x1A,0xA6,0xAC,0x4D,0x6F,0x78,0x51,0xB9,0x4C,0xCA,0xDE,0x00,0x68,0xB7,'\0'};
static char hash_value_primary_slot[33];


/* UART payload buffer element size. */
#define UART_BUF_SIZE 20

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_work_delayable uart_work;

K_SEM_DEFINE(nus_write_sem, 0, 1);

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
					const uint8_t *const data, uint16_t len)
{
	ARG_UNUSED(nus);

	struct uart_data_t *buf;

	/* Retrieve buffer context. */
	buf = CONTAINER_OF(data, struct uart_data_t, data);
	k_free(buf);

	k_sem_give(&nus_write_sem);

	if (err) {
		LOG_WRN("ATT error code: 0x%02X", err);
	}
}

static uint8_t ble_data_received(struct bt_nus_client *nus,
						const uint8_t *data, uint16_t len)
{
	ARG_UNUSED(nus);

	int err;

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return BT_GATT_ITER_CONTINUE;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf,
					   struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_RX_TIMEOUT);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
}

static int uart_init(void)
{
	int err;
	struct uart_data_t *rx;

	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data),
			      UART_RX_TIMEOUT);
}

static void discovery_complete(struct bt_gatt_dm *dm,
			       void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("Service discovery completed for nus");

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);

	bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn,
					void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
			    int err,
			    void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	if (conn != default_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn,
			       BT_UUID_NUS_SERVICE,
			       &discovery_cb,
			       &nus_client);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error "
			"code: %d", err);
	}
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done");
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void discovery_completed_cb(struct bt_gatt_dm *dm,
				   void *context)
{
	int err;

	LOG_INF("The discovery smp succeeded\n");

	bt_gatt_dm_data_print(dm);

	err = bt_dfu_smp_handles_assign(dm, &dfu_smp);
	if (err) {
		LOG_INF("Could not init DFU SMP client object, error: %d\n",
		       err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_INF("Could not release the discovery data, error "
		       "code: %d\n", err);
	}
}

static void discovery_service_not_found_cb(struct bt_conn *conn,
					   void *context)
{
	LOG_INF("The service could not be found during the discovery\n");
}

static void discovery_error_found_cb(struct bt_conn *conn,
				     int err,
				     void *context)
{
	LOG_INF("The discovery procedure failed with %d\n", err);
}

struct bt_gatt_dm_cb discovery_cb_smp = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};


#define INTERVAL_MIN	0x6	/* 320 units, 400 ms */
#define INTERVAL_MAX	0x6	/* 320 units, 400 ms */
static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);

		if (default_conn == conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)",
					err);
			}
		}

		return;
	}

	LOG_INF("Connected: %s", addr);

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	//err = bt_conn_set_security(conn, BT_SECURITY_L2);
	// if (err) {
	// 	LOG_WRN("Failed to set security: %d", err);

	// 	gatt_discover(conn);
	// }

	if (conn == default_conn) {
		err = bt_gatt_dm_start(conn, BT_UUID_DFU_SMP_SERVICE,
				       &discovery_cb_smp, NULL);
		if (err) {
			LOG_INF("Could not start the discovery procedure "
			       "(err %d)\n", err);
		}
	}

	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)",
			err);
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}

	gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d",
		addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

static int nus_client_init(void)
{
	int err;
	struct bt_nus_client_init_param init = {
		.cb = {
			.received = ble_data_received,
			.sent = ble_data_sent,
		}
	};

	err = bt_nus_client_init(&nus_client, &init);
	if (err) {
		LOG_ERR("NUS Client initialization failed (err %d)", err);
		return err;
	}

	LOG_INF("NUS Client module initialized");
	return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);



static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
		.conn_param = conn_param
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	// err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
	// if (err) {
	// 	LOG_ERR("Scanning filters cannot be set (err %d)", err);
	// 	return err;
	// }
	
	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID,
				 BT_UUID_DFU_SMP_SERVICE);
	if (err) {
		LOG_INF("Scanning filters cannot be set (err %d)\n", err);

		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	LOG_INF("Scan module initialized");
	return err;
}

static void dfu_smp_on_error(struct bt_dfu_smp *dfu_smp, int err)
{
	LOG_INF("DFU SMP generic error: %d\n", err);
}
static void smp_reset_rsp_proc(struct bt_dfu_smp *dfu_smp)
{
	LOG_INF("RESET RESPONSE CB. Doing nothing\n");
}

static const struct bt_dfu_smp_init_params init_params = {
	.error_cb = dfu_smp_on_error
};

static void smp_upload_rsp_proc(struct bt_dfu_smp *dfu_smp)
{
	uint8_t *p_outdata = (uint8_t *)(&smp_rsp_buff);
	const struct bt_dfu_smp_rsp_state *rsp_state;

	rsp_state = bt_dfu_smp_rsp_state(dfu_smp);

	if (rsp_state->offset + rsp_state->chunk_size > sizeof(smp_rsp_buff)) {
		LOG_INF("Response size buffer overflow\n");
	} else {
		p_outdata += rsp_state->offset;
		memcpy(p_outdata,
		       rsp_state->data,
		       rsp_state->chunk_size);
	}

	if (bt_dfu_smp_rsp_total_check(dfu_smp)) {
		if (smp_rsp_buff.header.op != 3 /* WRITE RSP*/) {
			LOG_INF("Unexpected operation code (%u)!\n",
			       smp_rsp_buff.header.op);
			return;
		}
		uint16_t group = ((uint16_t)smp_rsp_buff.header.group_h8) << 8 |
				      smp_rsp_buff.header.group_l8;
		if (group != 1 /* Application/software image management group */) {
			LOG_INF("Unexpected command group (%u)!\n", group);
			return;
		}
		if (smp_rsp_buff.header.id != 1 /* UPLOAD */) {
			LOG_INF("Unexpected command (%u)",
			       smp_rsp_buff.header.id);
			return;
		}
		size_t payload_len = ((uint16_t)smp_rsp_buff.header.len_h8) << 8 |
				      smp_rsp_buff.header.len_l8;

		zcbor_state_t zsd[CBOR_DECODER_STATE_NUM];
		struct zcbor_string value = {0};
		bool ok;
		zcbor_new_decode_state(zsd, ARRAY_SIZE(zsd), smp_rsp_buff.payload, payload_len, 1);

		/* Stop decoding on the error. */
		zsd->constant_state->stop_on_error = true;

		ok = zcbor_map_start_decode(zsd);
		if (!ok) {
			LOG_INF("Decoding error, start_decode (err: %d)\n", zcbor_pop_error(zsd));
			return;
		} 
		
		/**
		 * 
		 * Decoding rc (status error code)
		 * 
		*/
///customer need disable this rc check in ncs2.4.0
		//Decoding rc key
		char rc_key[5];
		ok = zcbor_tstr_decode(zsd, &value);
		if (!ok) {
			LOG_INF("Decoding error, rc key (err: %d)\n", zcbor_pop_error(zsd));
			return;
		}  else if (value.len != 2) {
			LOG_INF("Invalid data received (rc key). Length %d is not equal 2\n", value.len);
			return;
		} else if(!strncmp(value.value, 'rc', 2)){
			LOG_INF("Invalid data received (rc key). String '%.2s' is not equal to 'rc'\n", value.value);
			return;
		}
		memcpy(rc_key, value.value, value.len);
		rc_key[value.len] = '\0';

		//Decoding rc value
		int32_t rc_value;
		ok = zcbor_int32_decode(zsd, &rc_value);
		if (!ok) {
			LOG_INF("Decoding error, rc value (err: %d)\n", zcbor_pop_error(zsd));
			return;
		};
		if(rc_value){
			LOG_INF("Error in image upload response: %d\n", rc_value);
			return;
		}
///
		//decoding offset key
		char off_key[5];
		ok = zcbor_tstr_decode(zsd, &value);
		if (!ok) {
			LOG_INF("Decoding error, offset key (err: %d)\n", zcbor_pop_error(zsd));
			return;
		} else if ((value.len != 3)) {
			LOG_INF("Invalid data received (rc key). Length %d is not equal 3\n", value.len);
			return;
		}

		memcpy(off_key, value.value, value.len);
		off_key[value.len] = '\0';

		//Decoding offset value
		int32_t off_val;
		ok = zcbor_int32_decode(zsd, &off_val);
		if (!ok) {
			LOG_INF("Decoding error, offset value (err: %d)\n", zcbor_pop_error(zsd));
			return;
		}
		zcbor_map_end_decode(zsd);
		if (zcbor_check_error(zsd)) {
			//LOG_INF("%s: %d\n", off_key, off_val);
			//do nothing
		} else {
			LOG_INF("Cannot print received image upload CBOR stream (err: %d)\n",
					zcbor_pop_error(zsd));
		}
		k_sem_give(&upload_sem);
	}
}


static void smp_list_rsp_proc(struct bt_dfu_smp *dfu_smp)
{
	uint8_t *p_outdata = (uint8_t *)(&smp_rsp_buff);
	const struct bt_dfu_smp_rsp_state *rsp_state;

	rsp_state = bt_dfu_smp_rsp_state(dfu_smp);

	if (rsp_state->offset + rsp_state->chunk_size > sizeof(smp_rsp_buff)) {
		LOG_INF("Response size buffer overflow\n");
	} else {
		p_outdata += rsp_state->offset;
		memcpy(p_outdata,
		       rsp_state->data,
		       rsp_state->chunk_size);
	}
	if (bt_dfu_smp_rsp_total_check(dfu_smp)) {
		if (smp_rsp_buff.header.op != 1 && smp_rsp_buff.header.op != 3 ) {
			LOG_INF("Unexpected operation code (%u)!\n",
			       smp_rsp_buff.header.op);
			return;
		}
		uint16_t group = ((uint16_t)smp_rsp_buff.header.group_h8) << 8 |
				      smp_rsp_buff.header.group_l8;
		if (group != 1 /* Application/software image management group */) {
			LOG_INF("Unexpected command group (%u)!\n", group);
			return;
		}
		if (smp_rsp_buff.header.id != 0 /* STATE */) {
			LOG_INF("Unexpected command (%u)",
			       smp_rsp_buff.header.id);
			return;
		}
		size_t payload_len = ((uint16_t)smp_rsp_buff.header.len_h8) << 8 |
				      smp_rsp_buff.header.len_l8;
		zcbor_state_t zsd[10];
		struct zcbor_string value = {0};
		bool ok;
		zcbor_new_decode_state(zsd, ARRAY_SIZE(zsd), smp_rsp_buff.payload, payload_len, 5);
		/* Stop decoding on the error. */
		zsd->constant_state->stop_on_error = true;
		ok = zcbor_map_start_decode(zsd);
		if (!ok) {
			LOG_INF("Decoding error 1, start_decode (err: %d)\n", zcbor_pop_error(zsd));
			return;
		} 
		
		//Decoding images key
		char images_key[10];
		ok = zcbor_tstr_decode(zsd, &value);
		if (!ok) {
			LOG_INF("Decoding error 2, images key (err: %d)\n", zcbor_pop_error(zsd));
			return;
		}  
		memcpy(images_key, value.value, value.len);
		images_key[value.len] = '\0';
		//LOG_INF("Images key: %s\n",images_key);
		ok = zcbor_list_start_decode(zsd);
		if (!ok) {
			LOG_INF("Decoding error, start_decode images->list  (err: %d)\n", zcbor_pop_error(zsd));
			return;
		} 

		
		for(int slot=0; slot<2;slot++){
			ok = zcbor_map_start_decode(zsd);
			if (!ok) {
				if(slot == 0){
					LOG_INF("Error decoding slot 0. Err: %d", zcbor_pop_error(zsd));
				}else if(slot == 1){
					//LOG_INF("No secondary image present\n");
					break;
				}
			} 
			if(slot==0){
				LOG_INF("\n-----------PRIMARY IMAGE-----------\n");
			}else if(slot == 1){
				LOG_INF("\n-----------SECONDARY IMAGE-----------\n");
			}
			
			//Decoding slot key 
			char slot_key[5];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, slot key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			} 
			memcpy(slot_key, value.value, value.len);
			slot_key[value.len] = '\0';
			//LOG_INF("Slot key: %s\n",slot_key);

			//Decoding slot value
			int32_t slot_value;
			ok = zcbor_int32_decode(zsd, &slot_value);
			if (!ok) {
				LOG_INF("Decoding error, slot value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			LOG_INF("      %s: %d\n",slot_key,slot_value);

			//Decoding version key
			char version_key[5];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, version key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}  
			memcpy(version_key, value.value, value.len);
			version_key[value.len] = '\0';
			//LOG_INF("version key: %s\n",version_key);

			//decoding version value
			char version_value[5];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, version value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			} 
			memcpy(version_value, value.value, value.len);
			version_value[value.len] = '\0';
			LOG_INF("      %s: %s\n",version_key,version_value);

			//Decoding hash key
			char hash_key[5];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, hash key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			} 
			memcpy(hash_key, value.value, value.len);
			hash_key[value.len] = '\0';
			//LOG_INF("hash key: %s\n",hash_key);

			//decoding hash value
			char hash_value[40];
			ok = zcbor_bstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, hash value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			} 
			memcpy(hash_value, value.value, value.len);
			if(slot == 0){
				memcpy(hash_value_primary_slot, value.value, value.len);
			}
			else if(slot == 1){
				memcpy(hash_value_secondary_slot, value.value, value.len);
			}
			hash_value[value.len] = '\0';
			LOG_HEXDUMP_INF(hash_value,value.len,"hash is");

			//Decoding bootable key
			char bootable_key[10];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, hash key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}  
			memcpy(bootable_key, value.value, value.len);
			bootable_key[value.len] = '\0';

			//Decoding bootable value
			bool bootable_value;
			bootable_value = zcbor_bool_expect(zsd, true);
			if (!zcbor_check_error(zsd)) {
				LOG_INF("Decoding error, bootable value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			LOG_INF("      %s: %s\n",bootable_key, bootable_value?"true":"false");

			//Decoding pending key
			char pending_key[10];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, pending key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}  
			memcpy(pending_key, value.value, value.len);
			pending_key[value.len] = '\0';

			//Decoding pending value
			bool pending_value;
			pending_value = zcbor_bool_expect(zsd, false);
			if (!zcbor_check_error(zsd)) {
				LOG_INF("Decoding error, pending value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			LOG_INF("      %s: %s\n",pending_key, pending_value?"false":"true");
			// zcbor_bool_decode(zsd,&pending_value);
			// LOG_INF("      %s: %d\n",pending_key, pending_value);

			//Decoding confirmed key
			char confirmed_key[10];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, confirmed key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}  
			memcpy(confirmed_key, value.value, value.len);
			confirmed_key[value.len] = '\0';
			//LOG_INF("Confirmed key: %s\n",confirmed_key);

			//Decoding confirmed value
			bool confirmed_value;
			confirmed_value = zcbor_bool_expect(zsd, true);
			if (!zcbor_check_error(zsd)) {
				LOG_INF("Decoding error, confirmed value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			LOG_INF("      %s: %s\n",confirmed_key, confirmed_value?"true":"false");


			//Decoding active key
			char active_key[10];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, active key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			memcpy(active_key, value.value, value.len);
			active_key[value.len] = '\0';

			//Decoding active value
			bool active_value;
			active_value = zcbor_bool_expect(zsd, true);
			if (!zcbor_check_error(zsd)) {
				LOG_INF("Decoding error, active value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			LOG_INF("      %s: %s\n",active_key, active_value?"true":"false");

			//Decoding permanent key
			char permanent_key[10];
			ok = zcbor_tstr_decode(zsd, &value);
			if (!ok) {
				LOG_INF("Decoding error, permanent key (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}  
			memcpy(permanent_key, value.value, value.len);
			permanent_key[value.len] = '\0';

			//Decoding permanent value
			bool permanent_value;
			permanent_value = zcbor_bool_expect(zsd, true);
			if (!zcbor_check_error(zsd)) {
				LOG_INF("Decoding error, permanent value (err: %d)\n", zcbor_pop_error(zsd));
				return;
			}
			LOG_INF("      %s: %s\n",permanent_key, permanent_value?"true":"false");
			// zcbor_bool_decode(zsd,&permanent_value);
			// LOG_INF("      %s: %d\n",permanent_key, permanent_value);

			zcbor_map_end_decode(zsd);

		}
		zcbor_list_end_decode(zsd);
		zcbor_map_end_decode(zsd);


	}
}

static void smp_echo_rsp_proc(struct bt_dfu_smp *dfu_smp)
{
	uint8_t *p_outdata = (uint8_t *)(&smp_rsp_buff);
	const struct bt_dfu_smp_rsp_state *rsp_state;

	rsp_state = bt_dfu_smp_rsp_state(dfu_smp);
	LOG_INF("Echo response part received, size: %zu.\n",
	       rsp_state->chunk_size);

	if (rsp_state->offset + rsp_state->chunk_size > sizeof(smp_rsp_buff)) {
		LOG_INF("Response size buffer overflow\n");
	} else {
		p_outdata += rsp_state->offset;
		memcpy(p_outdata,
		       rsp_state->data,
		       rsp_state->chunk_size);
	}

	if (bt_dfu_smp_rsp_total_check(dfu_smp)) {
		LOG_INF("Total response received - decoding\n");
		if (smp_rsp_buff.header.op != 3 /* WRITE RSP*/) {
			LOG_INF("Unexpected operation code (%u)!\n",
			       smp_rsp_buff.header.op);
			return;
		}
		uint16_t group = ((uint16_t)smp_rsp_buff.header.group_h8) << 8 |
				      smp_rsp_buff.header.group_l8;
		if (group != 0 /* OS */) {
			LOG_INF("Unexpected command group (%u)!\n", group);
			return;
		}
		if (smp_rsp_buff.header.id != 0 /* ECHO */) {
			LOG_INF("Unexpected command (%u)",
			       smp_rsp_buff.header.id);
			return;
		}
		size_t payload_len = ((uint16_t)smp_rsp_buff.header.len_h8) << 8 |
				      smp_rsp_buff.header.len_l8;

		zcbor_state_t zsd[CBOR_DECODER_STATE_NUM];
		struct zcbor_string value = {0};
		char map_key[SMP_ECHO_MAP_KEY_MAX_LEN];
		char map_value[SMP_ECHO_MAP_VALUE_MAX_LEN];
		bool ok;

		zcbor_new_decode_state(zsd, ARRAY_SIZE(zsd), smp_rsp_buff.payload, payload_len, 1);

		/* Stop decoding on the error. */
		zsd->constant_state->stop_on_error = true;

		zcbor_map_start_decode(zsd);
		
		ok = zcbor_tstr_decode(zsd, &value);

		if (!ok) {
			LOG_INF("Decoding error (err: %d)\n", zcbor_pop_error(zsd));
			return;
		} else if ((value.len != 1) || (*value.value != 'r')) {
			LOG_INF("Invalid data received.\n");
			return;
		} else {
			/* Do nothing */
		}

		map_key[0] = value.value[0];

		/* Add string NULL terminator */
		map_key[1] = '\0';

		ok = zcbor_tstr_decode(zsd, &value);

		if (!ok) {
			LOG_INF("Decoding error (err: %d)\n", zcbor_pop_error(zsd));
			return;
		} else if (value.len > (sizeof(map_value) - 1)) {
			LOG_INF("To small buffer for received data.\n");
			return;
		} else {
			/* Do nothing */
		}

		memcpy(map_value, value.value, value.len);

		/* Add string NULL terminator */
		map_value[value.len] = '\0';

		zcbor_map_end_decode(zsd);

		if (zcbor_check_error(zsd)) {
			/* Print textual representation of the received CBOR map. */
			LOG_INF("{_\"%s\": \"%s\"}\n", map_key, map_value);
		} else {
			LOG_INF("Cannot print received CBOR stream (err: %d)\n",
			       zcbor_pop_error(zsd));
		}
	}

}


#define PROGRESS_WIDTH 50
static void progress_print(size_t downloaded, size_t file_size)
{
	const int percent = (downloaded * 100) / file_size;
	size_t lpad = (percent * PROGRESS_WIDTH) / 100;
	size_t rpad = PROGRESS_WIDTH - lpad;

	printk("\r[ %3d%% ] |", percent);
	for (size_t i = 0; i < lpad; i++) {
		printk("=");
	}
	for (size_t i = 0; i < rpad; i++) {
		printk(" ");
	}
	printk("| (%d/%d bytes)", downloaded, file_size);
}
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#define UPLOAD_CHUNK		2400 //This has to be at least 32 bytes, since first it has to send the whole header (which is 32 bytes)
uint8_t data[UPLOAD_CHUNK+1]; // One more byte, to store '/0'
uint8_t load_seq=0;
struct smp_buffer smp_cmd;
void send_upload2(struct k_work *item)
{
   	zcbor_state_t zse[2];
	size_t payload_len;
	const struct device *flash_dev = NULL;
	flash_dev = FIXED_PARTITION_DEVICE(custom_storage);
	int start_addr = 0x80000;
	int last_addr = 0;//(start_addr+0x26C04); //more smart way to get filesize
	
	int curr_addr = 0x80000;
	int upload_chunk = UPLOAD_CHUNK;
	int err;
	bool update_complete = false;
	uint64_t off = 0;
    uint8_t encode_len;
	load_seq = 0;

	flash_read(flash_dev, 0xF0000, &last_addr,4);
	LOG_INF("button2 pressed. starting uploading images, file size is %d",last_addr);	
	last_addr += start_addr;//from start_addr to last_addr

	while(!update_complete){ 
		
		zcbor_new_encode_state(zse, ARRAY_SIZE(zse), smp_cmd.payload,
			       sizeof(smp_cmd.payload), 0);
		//Wait until response is received until sending the next chunk
		k_sem_take(&upload_sem, K_FOREVER);
		if(curr_addr+UPLOAD_CHUNK > last_addr){
			upload_chunk = last_addr - curr_addr;
			update_complete = true;
		}
		LOG_INF("upload %d of %d bytes, at addr:%d\n", upload_chunk,last_addr - start_addr,curr_addr-start_addr);

		progress_print(curr_addr-start_addr, last_addr-start_addr);

		err = flash_read(flash_dev, curr_addr, data, upload_chunk);
		if (err != 0) {
			LOG_INF("flash_read failed with error: %d\n", err);
			return;
		}

		off = curr_addr - start_addr;
        if (off == 0){
            encode_len = 8;
        } else {
            encode_len = 6;
        }

		data[upload_chunk] = '\0';
		zse->constant_state->stop_on_error = true;
		if(!zcbor_map_start_encode(zse, encode_len))
			LOG_INF("zcbor_map_start_encode(zse, encode_len) fail");
		zcbor_tstr_put_lit(zse, "data");
		if(!zcbor_bstr_encode_ptr(zse, data, upload_chunk))
			LOG_INF("zcbor_bstr_encode_ptr(zse, data, upload_chunk) fail");
		zcbor_tstr_put_lit(zse, "len");
		if(!zcbor_uint64_put(zse, (uint64_t)(last_addr-start_addr)))
			LOG_INF("zcbor_uint64_put(zse, (uint64_t)(last_addr-start_addr)) fail");

		zcbor_tstr_put_lit(zse, "sha");
		zcbor_bstr_put_lit(zse, "123");

		zcbor_tstr_put_lit(zse, "off");
		if(!zcbor_uint64_put(zse, curr_addr - start_addr))
		LOG_INF("zcbor_uint64_put(zse, curr_addr - start_addr) fail");
		
		// zcbor_tstr_put_lit(zse, "upgrade");
		// zcbor_bool_put(zse, false);
		zcbor_map_end_encode(zse, encode_len);

		if (!zcbor_check_error(zse)) {
			LOG_INF("Failed to encode SMP test packet, err: %d\n", zcbor_pop_error(zse));
			return;
		}
		
		curr_addr+=upload_chunk;

		payload_len = (size_t)(zse->payload - smp_cmd.payload);

		smp_cmd.header.op = 2; /* write request */
		smp_cmd.header.flags = 0;
		smp_cmd.header.len_h8 = (uint8_t)((payload_len >> 8) & 0xFF);
		smp_cmd.header.len_l8 = (uint8_t)((payload_len >> 0) & 0xFF);
		smp_cmd.header.group_h8 = 0;
		smp_cmd.header.group_l8 = 1; /* IMAGE */
		smp_cmd.header.seq = load_seq;//-------
		load_seq++;
		smp_cmd.header.id  = 1; /* UPLOAD */
		
		err = bt_dfu_smp_command(&dfu_smp, smp_upload_rsp_proc,
					sizeof(smp_cmd.header) + payload_len,
					&smp_cmd);
		if(err){
			LOG_INF("bt_dfu_smp_command failed with %d\n", err);
			return;
		}
	}
	if(update_complete)
	{	LOG_INF("bt_dfu_smp_command completed \n");
		progress_print(curr_addr-start_addr, last_addr-start_addr);
	}
}

static int send_smp_list(struct bt_dfu_smp *dfu_smp)
{
	static struct smp_buffer smp_cmd;
	zcbor_state_t zse[CBOR_ENCODER_STATE_NUM];
	size_t payload_len;

	payload_len = (size_t)(zse->payload);
	smp_cmd.header.op = 0; /* read request */
	smp_cmd.header.flags = 0;
	smp_cmd.header.len_h8 = 0;
	smp_cmd.header.len_l8 = 0;
	smp_cmd.header.group_h8 = 0;
	smp_cmd.header.group_l8 = 1; /* IMAGE */
	smp_cmd.header.seq = 0;
	smp_cmd.header.id  = 0; /* LIST */
	return bt_dfu_smp_command(dfu_smp, smp_list_rsp_proc,
				  sizeof(smp_cmd.header),
				  &smp_cmd);
}


static int send_smp_reset(struct bt_dfu_smp *dfu_smp)
{
	static struct smp_buffer smp_cmd;
	zcbor_state_t zse[CBOR_ENCODER_STATE_NUM];
	size_t payload_len;

	zcbor_new_encode_state(zse, ARRAY_SIZE(zse), smp_cmd.payload,
			       sizeof(smp_cmd.payload), 0);

	/* Stop encoding on the error. */
	zse->constant_state->stop_on_error = true;

	zcbor_map_start_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);

	zcbor_map_end_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);

	if (!zcbor_check_error(zse)) {
		LOG_INF("Failed to encode SMP reset packet, err: %d\n", zcbor_pop_error(zse));
		return -EFAULT;
	}

	payload_len = (size_t)(zse->payload - smp_cmd.payload);

	smp_cmd.header.op = 2; /* Write */
	smp_cmd.header.flags = 0;
	smp_cmd.header.len_h8 = (uint8_t)((payload_len >> 8) & 0xFF);
	smp_cmd.header.len_l8 = (uint8_t)((payload_len >> 0) & 0xFF);
	smp_cmd.header.group_h8 = 0;
	smp_cmd.header.group_l8 = 0; /* OS */
	smp_cmd.header.seq = load_seq;//-------
		load_seq++;
	smp_cmd.header.id  = 5; /* RESET */

	return bt_dfu_smp_command(dfu_smp, smp_reset_rsp_proc,
				  sizeof(smp_cmd.header) + payload_len,
				  &smp_cmd);
}

static int send_smp_test(struct bt_dfu_smp *dfu_smp)
{
	static struct smp_buffer smp_cmd;
	zcbor_state_t zse[CBOR_ENCODER_STATE_NUM];
	size_t payload_len;

	zcbor_new_encode_state(zse, ARRAY_SIZE(zse), smp_cmd.payload,
			       sizeof(smp_cmd.payload), 0);

	/* Stop encoding on the error. */
	zse->constant_state->stop_on_error = true;

	zcbor_map_start_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);

	zcbor_map_end_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);

	if (!zcbor_check_error(zse)) {
		LOG_INF("Failed to encode SMP reset packet, err: %d\n", zcbor_pop_error(zse));
		return -EFAULT;
	}

	payload_len = (size_t)(zse->payload - smp_cmd.payload);

	smp_cmd.header.op = 2; /* Write */
	smp_cmd.header.flags = 0;
	smp_cmd.header.len_h8 = (uint8_t)((payload_len >> 8) & 0xFF);
	smp_cmd.header.len_l8 = (uint8_t)((payload_len >> 0) & 0xFF);
	smp_cmd.header.group_h8 = 0;
	smp_cmd.header.group_l8 = 1; /* OS */
	smp_cmd.header.seq = load_seq;
	load_seq++;
	smp_cmd.header.id  = 0; /* RESET */

	return bt_dfu_smp_command(dfu_smp, smp_reset_rsp_proc,
				  sizeof(smp_cmd.header) + payload_len,
				  &smp_cmd);
}

static int send_smp_confirm(struct bt_dfu_smp *dfu_smp)
{
	static struct smp_buffer smp_cmd;
	zcbor_state_t zse[CBOR_ENCODER_STATE_NUM];
	size_t payload_len;

	zcbor_new_encode_state(zse, ARRAY_SIZE(zse), smp_cmd.payload,
			       sizeof(smp_cmd.payload), 0);

	/* Stop encoding on the error. */
	zse->constant_state->stop_on_error = true;

	zcbor_map_start_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);
	zcbor_tstr_put_lit(zse, "confirm");
	zcbor_bool_put(zse, false);
	zcbor_tstr_put_lit(zse, "hash");
	zcbor_bstr_put_lit(zse, hash_value_secondary_slot);
	
	
	zcbor_map_end_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);

	if (!zcbor_check_error(zse)) {
		LOG_INF("Failed to encode SMP confirm packet, err: %d\n", zcbor_pop_error(zse));
		return -EFAULT;
	}

	payload_len = (size_t)(zse->payload - smp_cmd.payload);

	smp_cmd.header.op = 2; /* Write */
	smp_cmd.header.flags = 0;
	smp_cmd.header.len_h8 = (uint8_t)((payload_len >> 8) & 0xFF);
	smp_cmd.header.len_l8 = (uint8_t)((payload_len >> 0) & 0xFF);
	smp_cmd.header.group_h8 = 0;
	smp_cmd.header.group_l8 = 1; /* app/image */
	smp_cmd.header.seq = load_seq;
	load_seq++;
	smp_cmd.header.id  = 0; 

	// confirm has same response as list command
	return bt_dfu_smp_command(dfu_smp, smp_list_rsp_proc,
				  sizeof(smp_cmd.header) + payload_len,
				  &smp_cmd);
}



static int send_smp_echo(struct bt_dfu_smp *dfu_smp,
			 const char *string)
{
	static struct smp_buffer smp_cmd;
	zcbor_state_t zse[CBOR_ENCODER_STATE_NUM];
	size_t payload_len;

	zcbor_new_encode_state(zse, ARRAY_SIZE(zse), smp_cmd.payload,
			       sizeof(smp_cmd.payload), 0);

	/* Stop encoding on the error. */
	zse->constant_state->stop_on_error = true;

	zcbor_map_start_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);
	zcbor_tstr_put_lit(zse, "d");
	zcbor_tstr_put_term(zse, string);
	zcbor_map_end_encode(zse, CBOR_MAP_MAX_ELEMENT_CNT);

	if (!zcbor_check_error(zse)) {
		LOG_INF("Failed to encode SMP echo packet, err: %d\n", zcbor_pop_error(zse));
		return -EFAULT;
	}

	payload_len = (size_t)(zse->payload - smp_cmd.payload);

	smp_cmd.header.op = 2; /* Write */
	smp_cmd.header.flags = 0;
	smp_cmd.header.len_h8 = (uint8_t)((payload_len >> 8) & 0xFF);
	smp_cmd.header.len_l8 = (uint8_t)((payload_len >> 0) & 0xFF);
	smp_cmd.header.group_h8 = 0;
	smp_cmd.header.group_l8 = 0; /* OS */
	smp_cmd.header.seq = 0;
	smp_cmd.header.id  = 0; /* ECHO */

	return bt_dfu_smp_command(dfu_smp, smp_echo_rsp_proc,
				  sizeof(smp_cmd.header) + payload_len,
				  &smp_cmd);
}

static void button_upload(bool state)
{
	
	if (state) {
		int ret;

		k_work_submit(&upload_work_item);

	}
}


static void button_confirm(bool state)
{
	
	if (state) {
		int ret;
		ret = send_smp_confirm(&dfu_smp);
		LOG_INF("button4 pressed. send hash to test image\r\n");
		if (ret) {
			LOG_INF("Confirm command send error (err: %d)\n", ret);
		}
		

	}
}

static void button_test(bool state)
{
	
	if (state) {
		int ret;
		ret = send_smp_reset(&dfu_smp);
		LOG_INF("button3 pressed. send reset command\r\n");
		if (ret) {
			LOG_INF("Test command send error (err: %d)\n", ret);
		}
		

	}
}


static void button_echo(bool state)
{
	if (state) {
		static unsigned int echo_cnt;
		char buffer[32];
		int ret;

		++echo_cnt;
		LOG_INF("Echo test: %d\n", echo_cnt);
		ret = send_smp_echo(&dfu_smp, buffer);
		if (ret) {
			LOG_INF("Echo command send error (err: %d)\n", ret);
		}
	}
}

static void button_image_list(bool state)
{
	if (state) {
		int ret;

		ret = send_smp_list(&dfu_smp);
		LOG_INF("button1 pressed. list image for slot0 and slot1\r\n");
		if (ret) {
			LOG_INF("Image list command send error (err: %d)\n", ret);
		}
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & KEY_LIST_MASK) {
		button_image_list(button_state & KEY_LIST_MASK);
	}
	if(has_changed & KEY_UPLOAD_MASK){
		button_upload(button_state & KEY_UPLOAD_MASK);
	}
	if(has_changed & KEY_TEST_MASK){
		button_test(button_state & KEY_TEST_MASK);
	}
	if(has_changed & KEY_CONFIRM_MASK){
		button_confirm(button_state & KEY_CONFIRM_MASK);
	}
	//No more buttons for reset
}

void main(void)
{
	int err;

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_INF("Failed to register authorization info callbacks.\n");
		return;
	}

	LOG_INF("Starting Bluetooth Central SMP Client example\n");
	
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_INF("Failed to initialize buttons (err %d)\n", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");

	k_work_init(&upload_work_item, send_upload2);
	
	bt_dfu_smp_init(&dfu_smp, &init_params);


	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	int (*module_init[])(void) = {uart_init, scan_init, nus_client_init};
	for (size_t i = 0; i < ARRAY_SIZE(module_init); i++) {
		err = (*module_init[i])();
		if (err) {
			return;
		}
	}

	LOG_INF("Starting Bluetooth Central UART example\n");



	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}

	LOG_INF("Scanning successfully started");

	for (;;) {
		/* Wait indefinitely for data to be sent over Bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		err = bt_nus_client_send(&nus_client, buf->data, buf->len);
		if (err) {
			LOG_WRN("Failed to send data over BLE connection"
				"(err %d)", err);
		}

		err = k_sem_take(&nus_write_sem, NUS_WRITE_TIMEOUT);
		if (err) {
			LOG_WRN("NUS send timeout");
		}
	}
}
