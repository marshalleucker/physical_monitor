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

const struct device *uart_console = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct device *uart_mx7 = DEVICE_DT_GET(DT_NODELABEL(uart1));

struct uart_config uart_cfg = {
    .baudrate = 9600,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    .data_bits = UART_CFG_DATA_BITS_8,
};

#define PDM_BT_UUID_VAL BT_UUID_128_ENCODE(0xba3fc78e, 0x12ad, 0x4cc6, 0x8ee2, 0x4ffc42f0757d)
#define PDM_BT_UUID BT_UUID_DECLARE_128(PDM_BT_UUID_VAL)

#define HR_BT_UUID_VAL BT_UUID_128_ENCODE(0xd3c40f85, 0x0ada, 0x4072, 0xa155, 0xfc78ce198193)
#define HR_BT_UUID BT_UUID_DECLARE_128(HR_BT_UUID_VAL)

#define SP_BT_UUID_VAL BT_UUID_128_ENCODE(0x5297a449, 0x5b81, 0x4629, 0x8a74, 0x6fbd8f7a2d26)
#define SP_BT_UUID BT_UUID_DECLARE_128(SP_BT_UUID_VAL)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, PDM_BT_UUID_VAL)};

int16_t Pulse = 0;
int16_t SpO2 = 0;

volatile bool ble_ready=false;

ssize_t read_pulse(struct bt_conn *conn,
                   const struct bt_gatt_attr *attr, void *buf,
                   uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &Pulse, sizeof(Pulse));
}

BT_GATT_SERVICE_DEFINE(custom_srv,
                       BT_GATT_PRIMARY_SERVICE(PDM_BT_UUID),
                       BT_GATT_CHARACTERISTIC(HR_BT_UUID, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, NULL, NULL, NULL),
                       BT_GATT_CHARACTERISTIC(SP_BT_UUID, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, NULL, NULL, NULL));


void bt_ready(int err)
{

    if (err)
    {
        printk("err %d\n", err);
        return;
    }

    printk("Advertising successfully started\n");
    ble_ready=true;
}
int init_ble(void){
    int err;

    err = bt_enable(bt_ready);
    if(err){
        return err;
    }
    return 0;
}


void send_str(const struct device *uart, char *str, int msg_len)
{
    for (int i = 0; i < msg_len; i++)
    {
        uart_poll_out(uart, str[i]);
    }
}

void recv_str(const struct device *uart, char *str)
{
    char *head = str;
    char c;
    while (!uart_poll_in(uart, &c))
    {
        *head++ = c;
    }
    *head = '\0';
}

void get_board_serial(char *str)
{
    unsigned char mssg[5] = {0xA1, 0x01, 0x70, 0x70, 0xAF};
    char recv_buff[64];
    send_str(uart_mx7, mssg, 5);
    k_sleep(K_MSEC(100));
    recv_str(uart_mx7, recv_buff);
    for (int i = 0; i < 4; i++)
        str[i] = recv_buff[i + 3];
}

void unlock_board(char *mfg)
{
    unsigned char mssg[16];
    unsigned char serial[4];
    char recv_buff[64];
    int CSUM = 0;

    get_board_serial(serial);

    mssg[0] = 0xA1; // SOM
    mssg[1] = 0x0B; // LEN = 11
    mssg[2] = 0x71; // d1=0x71 Unlock Board
    // d2..d5 = Board_Serial_Number XOR Manufacturer_ID
    for (int i = 0; i < 4; i++)
        mssg[i + 3] = serial[i] ^ mfg[i];
    mssg[7] = 0x00;  // d6
    mssg[8] = 0x00;  // d7
    mssg[9] = 0x00;  // d8
    mssg[10] = 0x03; // d9
    mssg[11] = 0x00; // d10
    mssg[12] = 0x00; // d11
    for (int j = 2; j <= 12; j++)
        CSUM += mssg[j];
    mssg[13] = CSUM & 0xFF; // CSUM
    mssg[14] = 0xAF;        // EOM

    send_str(uart_mx7, mssg, 15);
    k_sleep(K_MSEC(100));
    recv_str(uart_mx7, recv_buff);

    if (!((recv_buff[1] == 0x01) && (recv_buff[2] == 0x01)))
    {
        unlock_board(mfg);
    }
}

uint16_t request_pulse(void)
{
    char recv_buff[64];
    unsigned char mssg[6] = {0xA1, 0x02, 0x10, 0x02, 0x12, 0xAF}; // SOM LEN d1 d2 CSUM EOM: d1 = request parameter, d2 = pulse rate

    send_str(uart_mx7, mssg, 6);
    k_sleep(K_MSEC(100)); /* Wait for 100 ms or check uart available. */
    recv_str(uart_mx7, recv_buff);
    return ((int)recv_buff[5]);
}

uint16_t request_SpO2(void)
{
    char recv_buff[64];
    unsigned char mssg[6] = {0xA1, 0x02, 0x10, 0x01, 0x11, 0xAF}; // SOM LEN d1 d2 CSUM EOM: d1 = request parameter, d2 = SpO2

    send_str(uart_mx7, mssg, 6);
    k_sleep(K_MSEC(100)); /* Wait for 100 ms or check uart available. */
    recv_str(uart_mx7, recv_buff);
    return ((int)(recv_buff[4] << 8 | recv_buff[5]));
}

int main(void)
{
    int rc;
    unsigned char Mfg_ID[4] = {0x19, 0xF9, 0x56, 0x14};

    init_ble();
    while(!ble_ready){
        k_msleep(100);
    }

    int err;
    err=bt_le_adv_start(BT_LE_ADV_CONN_NAME,ad,ARRAY_SIZE(ad),NULL,0);
    if(err){
        printk("err %d\n", err);
        return 0;
    }


    rc = uart_configure(uart_mx7, &uart_cfg);
    if (rc)
    {
        printk("Could not configure device %s", uart_mx7->name);
        return 0;
    }

    k_sleep(K_MSEC(1500));

   // unlock_board(Mfg_ID);

    while (1)
    {
        //Pulse = request_pulse();

        //SpO2 = request_SpO2();
        Pulse++;
        k_sleep(K_MSEC(1000));

        if(Pulse==10){
            Pulse=0;
        }
        printk("Pulse: %d, SpO2: %d\n", Pulse, SpO2);
    }

    return 0;
}
