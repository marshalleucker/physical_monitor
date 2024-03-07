/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

const struct device *uart_console = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct device *uart_mx7 = DEVICE_DT_GET(DT_NODELABEL(uart1));

struct uart_config uart_cfg = {
    .baudrate = 9600,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    .data_bits = UART_CFG_DATA_BITS_8,
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL))
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
	static uint8_t heartrate = 90U;

	/* Heartrate measurements simulation */
	heartrate++;
	if (heartrate == 160U) {
		heartrate = 90U;
	}

	bt_hrs_notify(heartrate);
}

void send_str(const struct device *uart, char *str, int msg_len)
{
    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart, str[i]);
    }
}


void recv_str(const struct device *uart, char *str)
{
    char *head = str;
    char c;
    while (!uart_poll_in(uart, &c)) {
        *head++ = c;
    }
    *head = '\0';
}

void get_board_serial(char *str){
    unsigned char mssg[5] = {0xA1, 0x01, 0x70, 0x70, 0xAF};
    char recv_buff[64];
    send_str(uart_mx7, mssg, 5);
    k_sleep(K_MSEC(100));
    recv_str(uart_mx7, recv_buff);
    for (int i=0; i<4; i++)
        str[i] = recv_buff[i+3];
}

void unlock_board(char *mfg){
    unsigned char mssg[16];
    unsigned char serial[4];
    char recv_buff[64];
    int CSUM = 0;

    get_board_serial(serial);

    mssg[0] = 0xA1;     //SOM
    mssg[1] = 0x0B;     //LEN = 11
    mssg[2] = 0x71;     //d1=0x71 Unlock Board
    // d2..d5 = Board_Serial_Number XOR Manufacturer_ID
    for (int i=0; i<4; i++)
        mssg[i+3] = serial[i] ^ mfg[i];
    mssg[7] = 0x00;     // d6
    mssg[8] = 0x00;     // d7
    mssg[9] = 0x00;     // d8
    mssg[10] = 0x03;    // d9
    mssg[11] = 0x00;    // d10
    mssg[12] = 0x00;    // d11
    for (int j=2; j<=12; j++)
        CSUM += mssg[j];
    mssg[13] = CSUM & 0xFF; // CSUM 
    mssg[14] = 0xAF;    // EOM

    send_str(uart_mx7, mssg, 15);
    k_sleep(K_MSEC(100));
    recv_str(uart_mx7, recv_buff);

    if (!((recv_buff[1] == 0x01) && (recv_buff[2] == 0x01))){
        unlock_board(mfg);
    }

}

int request_pulse(void){
    char recv_buff[64];
    unsigned char mssg[6] = {0xA1, 0x02, 0x10, 0x02, 0x12, 0xAF}; // SOM LEN d1 d2 CSUM EOM: d1 = request parameter, d2 = pulse rate

    send_str(uart_mx7, mssg, 6);
    k_sleep(K_MSEC(100));       /* Wait for 100 ms or check uart available. */
    recv_str(uart_mx7, recv_buff);
    return((int)recv_buff[5]);    
}

uint16_t request_SpO2(void){
    char recv_buff[64];
    unsigned char mssg[6] = {0xA1, 0x02, 0x10, 0x01, 0x11, 0xAF}; // SOM LEN d1 d2 CSUM EOM: d1 = request parameter, d2 = SpO2

    send_str(uart_mx7, mssg, 6);
    k_sleep(K_MSEC(100));       /* Wait for 100 ms or check uart available. */
    recv_str(uart_mx7, recv_buff);
    return((int)(recv_buff[4] << 8 | recv_buff[5]));
}



int main(void)
{
    int err;
	int rc;
    unsigned char Mfg_ID[4] = {0x19, 0xF9, 0x56, 0x14};
    unsigned int Pulse_Rate;
    uint16_t SpO2;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    rc = uart_configure(uart_mx7, &uart_cfg);
    if (rc) {
        printk("Could not configure device %s", uart_mx7->name);
		return -1;
    }

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);

    k_sleep(K_MSEC(1500));

    unlock_board(Mfg_ID);

   
    while(1) {
        Pulse_Rate = request_pulse();
		bt_hrs_notify(Pulse_Rate);
        printk("Pulse: %d\n", Pulse_Rate);

        SpO2 = request_SpO2();
        printk("Pulse: %d, SpO2: %d\n", Pulse_Rate, SpO2);
    }


    return 0;
}
