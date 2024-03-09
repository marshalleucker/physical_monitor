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


const struct device *uart_console = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct device *uart_mx7 = DEVICE_DT_GET(DT_NODELABEL(uart1));


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
    int rc;
    unsigned char Mfg_ID[4] = {0x19, 0xF9, 0x56, 0x14};
    unsigned int Pulse_Rate, SpO2, GSR;
    //uint16_t SpO2, GSR;

    // Flash variables
    const struct device *flash_dev = TEST_PARTITION_DEVICE;
    uint32_t buf_word = 0U;
	uint32_t offset;
    uint32_t add = 0U;

    rc = uart_configure(uart_mx7, &uart_cfg);
    if (rc) {
        printk("Could not configure device %s", uart_mx7->name);
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
    // Returns 0 on success
	if (flash_erase(flash_dev, TEST_PARTITION_OFFSET + (2 << 12), FLASH_PAGE_SIZE) != 0) {
		printf("   Flash erase failed!\n");
	} else {
		printf("   Flash erase succeeded!\n");
	}
    // Returns 0 on success
	if (flash_erase(flash_dev, TEST_PARTITION_OFFSET + (2 << 13), FLASH_PAGE_SIZE) != 0) {
		printf("   Flash erase failed!\n");
	} else {
		printf("   Flash erase succeeded!\n");
	}

    k_sleep(K_MSEC(1500));

    unlock_board(Mfg_ID);
   
    while(1) {
        Pulse_Rate = request_pulse();

        // Write into flash on page 130
        offset = TEST_PARTITION_OFFSET + (add << 2);
        printf("   Attempted to write %x at 0x%x", Pulse_Rate,
				offset);
		if (flash_write(flash_dev, offset, &Pulse_Rate,
					sizeof(uint32_t)) != 0) {
			printf("   Flash write failed!");
			return 0;
		}
		printf("   Attempted to read 0x%x", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(uint32_t)) != 0) {
			printf("   Flash read failed!");
			return 0;
		}
		printf("   Data read: %x\n", buf_word);
		if (Pulse_Rate == buf_word) {
			printf("   Data read matches data written. Good!\n");
		} else {
			printf("   Data read does not match data written!\n");
		}

        SpO2 = request_SpO2();
        // SpO2 /= 100;

        // Write into flash on page 131
        offset = TEST_PARTITION_OFFSET + (add << 2) + (2 << 12);
        printf("   Attempted to write %x at 0x%x", SpO2,
				offset);
		if (flash_write(flash_dev, offset, &SpO2,
					sizeof(uint32_t)) != 0) {
			printf("   Flash write failed!");
			return 0;
		}
		printf("   Attempted to read 0x%x", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(uint32_t)) != 0) {
			printf("   Flash read failed!");
			return 0;
		}
		// printf("   Data read: %x", buf_word - 0xf9190000);
		// if (SpO2 == buf_word - 0xf9190000) {
		// 	printf("   Data read matches data written. Good!\n");
		// } else {
		// 	printf("   Data read does not match data written!\n");
		// }

        // Issue is because Sp02 is uint16 instead of 32
        printf("   Data read: %x", buf_word);
		if (SpO2 == buf_word) {
			printf("   Data read matches data written. Good!\n");
		} else {
			printf("   Data read does not match data written!\n");
		}

        GSR = 313U;

        // Write into flash on page 132
        offset = TEST_PARTITION_OFFSET + (add << 2) + (2 << 13);
        printf("   Attempted to write %x at 0x%x", GSR,
				offset);
		if (flash_write(flash_dev, offset, &GSR,
					sizeof(uint32_t)) != 0) {
			printf("   Flash write failed!");
			return 0;
		}
		printf("   Attempted to read 0x%x", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(uint32_t)) != 0) {
			printf("   Flash read failed!");
			return 0;
		}
		printf("   Data read: %x\n", buf_word);
		if (GSR == buf_word) {
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

        printk("Pulse: %d, SpO2: %d, GSR: %d\n", Pulse_Rate, SpO2, GSR);

        k_sleep(K_MSEC(2000));
    }


    return 0;
}
