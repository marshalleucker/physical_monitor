#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>


#include <stdio.h>
#include <string.h>


// Temporary position for flash stuff
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/devicetree.h>

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
#define TEST_PARTITION	slot1_ns_partition
#else
#define TEST_PARTITION	slot1_partition
#endif

#define TEST_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(TEST_PARTITION)
#define TEST_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(TEST_PARTITION)

#define FLASH_PAGE_SIZE   4096



const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));




struct uart_config uart_cfg = {
    .baudrate = 9600,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    .data_bits = UART_CFG_DATA_BITS_8,
};




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




int main(void)
{
    int rc, i, CSUM=0;
    char recv_buf[64];
    unsigned char Board_Request_info[5] = {0xA1, 0x01, 0x70, 0x70, 0xAF};
    unsigned char Mfg_ID[4] = {0x19, 0xF9, 0x56, 0x14};
    unsigned char Board_Serial_Num[4];
//    unsigned char Board_Serial_Num[4] = {0x89, 0xD1, 0x8D, 0xBD};
    unsigned char Unlock_Board_Msg[16];
    unsigned char Request_Pulse_Rate[6] = {0xA1, 0x02, 0x10, 0x02, 0x12, 0xAF};
    unsigned int Pulse_Rate;
    unsigned char Request_Respiration_Rate[6] = {0xA1, 0x02, 0x10, 0x35, 0x45, 0xAF};
    unsigned int Respiration_Rate;

    // Flash variables
    const struct device *flash_dev = TEST_PARTITION_DEVICE;
    uint32_t buf_word = 0U;
	uint32_t offset;
    uint32_t add = 0U;

    rc = uart_configure(uart0, &uart_cfg);
    if (rc) {
        printk("Could not configure device %s", uart0->name);
        return -1;
    }

    // Check for flash errors
    if (!device_is_ready(flash_dev)) {
		printf("Flash device not ready\n");
		return -1;
	}

	// Returns 0 on success
	if (flash_erase(flash_dev, TEST_PARTITION_OFFSET, FLASH_PAGE_SIZE) != 0) {
		printf("   Flash erase failed!\n");
	} else {
		printf("   Flash erase succeeded!\n");
	}

    k_sleep(K_MSEC(1000));
    send_str(uart0, Board_Request_info, 5);
    k_sleep(K_MSEC(100));       /* Wait for 100 ms or check uart available. */
    recv_str(uart0, recv_buf);
    for (i=0; i<4; i++)
        Board_Serial_Num[i] = recv_buf[i+3];


    // Build the unlock message
    //  Ali send this {0xA1, 0x0B, 0x71, 0x90, 0x28, 0xDB, 0xA9, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0xB0, 0xAF}
    // I send this    {0xA1, 0x0B, 0x71, 0x90, 0x28, 0xDB, 0xA9, 0x00, 0x00, 0x00, 0x02, 0xB0, 0xAF}


    Unlock_Board_Msg[0] = 0xA1;     //SOM
    Unlock_Board_Msg[1] = 0x0B;     //LEN = 11
    Unlock_Board_Msg[2] = 0x71;     //d1=0x71 Unlock Board
    // d2..d5 = Board_Serial_Number XOR Manufacturer_ID
    for (i=0; i<4; i++)
        Unlock_Board_Msg[i+3] = Board_Serial_Num[i] ^ Mfg_ID[i];
    Unlock_Board_Msg[7] = 0x00;     // d6
    Unlock_Board_Msg[8] = 0x00;     // d7
    Unlock_Board_Msg[9] = 0x00;     // d8
    Unlock_Board_Msg[10] = 0x02;    // d9
    Unlock_Board_Msg[11] = 0x00;    // d10
    Unlock_Board_Msg[12] = 0x01;    // d11
    for (i=2; i<=12; i++)
        CSUM += Unlock_Board_Msg[i];
    Unlock_Board_Msg[13] = CSUM & 0xFF; // CSUM 0x36 not 0xB0
    Unlock_Board_Msg[14] = 0xAF;    // EOM




   
    send_str(uart0, Unlock_Board_Msg, 15);
    memset(recv_buf, 0, 64);
    k_sleep(K_MSEC(100));       /* Wait for 100 ms or check uart available. */
    recv_str(uart0, recv_buf);
   
    while(1) {
        // To request Pulse Rate:
        // SOM LEN d1 d2 CSUM EOM
        // 0xA1 0x02 0x10 0x02 0x12 0xAF
        send_str(uart0, Request_Pulse_Rate, 6);
        memset(recv_buf, 0, 64);
        k_sleep(K_MSEC(100));       /* Wait for 100 ms or check uart available. */
        recv_str(uart0, recv_buf);
        Pulse_Rate = (int)recv_buf[5];

        // Write into flash on page 130
        offset = TEST_PARTITION_OFFSET + (add << 2);
        printf("   Attempted to write %x at 0x%x\n", Pulse_Rate,
				offset);
		if (flash_write(flash_dev, offset, &Pulse_Rate,
					sizeof(uint32_t)) != 0) {
			printf("   Flash write failed!\n");
			return 0;
		}
		printf("   Attempted to read 0x%x\n", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(uint32_t)) != 0) {
			printf("   Flash read failed!\n");
			return 0;
		}
		printf("   Data read: %x\n", buf_word);
		if (Pulse_Rate == buf_word) {
			printf("   Data read matches data written. Good!\n");
		} else {
			printf("   Data read does not match data written!\n");
		}

        // To request Respiration Rate:
        // SOM LEN d1 d2 CSUM EOM
        // 0xA1 0x02 0x10 0x35 0x45 0xAF
        send_str(uart0, Request_Respiration_Rate, 6);
        memset(recv_buf, 0, 64);
        k_sleep(K_MSEC(100));       /* Wait for 100 ms or check uart available. */
        recv_str(uart0, recv_buf);
        Respiration_Rate = (int)recv_buf[5];

        // Write into flash on page 131
        offset = TEST_PARTITION_OFFSET + (add << 2) + 8000;
        printf("   Attempted to write %x at 0x%x\n", Respiration_Rate,
				offset);
		if (flash_write(flash_dev, offset, &Respiration_Rate,
					sizeof(uint32_t)) != 0) {
			printf("   Flash write failed!\n");
			return 0;
		}
		printf("   Attempted to read 0x%x\n", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(uint32_t)) != 0) {
			printf("   Flash read failed!\n");
			return 0;
		}
		printf("   Data read: %x\n", buf_word);
		if (Respiration_Rate == buf_word) {
			printf("   Data read matches data written. Good!\n");
		} else {
			printf("   Data read does not match data written!\n");
		}

        if (add >= 1999) {
            printf("Exceeded memory");
            return -1;
        } else {
            add++;
        }
    }




    return 0;
}
