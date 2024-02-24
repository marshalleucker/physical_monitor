#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
#define TEST_PARTITION	slot1_ns_partition
#else
#define TEST_PARTITION	slot1_partition
#endif

#define TEST_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(TEST_PARTITION)
#define TEST_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(TEST_PARTITION)

#define FLASH_PAGE_SIZE   4096
//#define TEST_DATA_WORD_0  0x1122
//#define TEST_DATA_WORD_1  0xaabb
//#define TEST_DATA_WORD_2  0xabcd
//#define TEST_DATA_WORD_3  0x1234

#define FLASH_TEST_OFFSET2 0x41234
#define FLASH_TEST_PAGE_IDX 37

const struct device *flash_dev = TEST_PARTITION_DEVICE;
uint32_t buf_word = 0U;
uint32_t i, offset;
