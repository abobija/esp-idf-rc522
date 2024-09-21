#include "unity.h"

#include "picc/rc522_mifare.c"

TEST_CASE("test_Sector_desc_contains_correct_info", "[mifare]")
{
    rc522_mifare_sector_desc_t desc;

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_desc(0, &desc));
    TEST_ASSERT_EQUAL(0, desc.index);
    TEST_ASSERT_EQUAL(0, desc.block_0_address);
    TEST_ASSERT_EQUAL(4, desc.number_of_blocks);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_desc(31, &desc));
    TEST_ASSERT_EQUAL(31, desc.index);
    TEST_ASSERT_EQUAL(124, desc.block_0_address);
    TEST_ASSERT_EQUAL(4, desc.number_of_blocks);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_desc(32, &desc));
    TEST_ASSERT_EQUAL(32, desc.index);
    TEST_ASSERT_EQUAL(128, desc.block_0_address);
    TEST_ASSERT_EQUAL(16, desc.number_of_blocks);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_desc(33, &desc));
    TEST_ASSERT_EQUAL(33, desc.index);
    TEST_ASSERT_EQUAL(144, desc.block_0_address);
    TEST_ASSERT_EQUAL(16, desc.number_of_blocks);
}

TEST_CASE("test_Sector_block_0_address", "[mifare]")
{
    uint8_t block_address = 0;

    for (uint8_t sector_index = 0; sector_index < 31; sector_index++) {
        TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_0_address(sector_index, &block_address));
        TEST_ASSERT_EQUAL(sector_index * 4, block_address);
    }

    for (uint8_t sector_index = 32; sector_index < 40; sector_index++) {
        TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_0_address(sector_index, &block_address));
        TEST_ASSERT_EQUAL(128 + (sector_index - 32) * 16, block_address);
    }
}

TEST_CASE("test_Sector_number_of_blocks", "[mifare]")
{
    uint8_t number_of_blocks = 0;

    for (uint8_t sector_index = 0; sector_index < 31; sector_index++) {
        TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_number_of_blocks(sector_index, &number_of_blocks));
        TEST_ASSERT_EQUAL(4, number_of_blocks);
    }

    for (uint8_t sector_index = 32; sector_index < 40; sector_index++) {
        TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_number_of_blocks(sector_index, &number_of_blocks));
        TEST_ASSERT_EQUAL(16, number_of_blocks);
    }
}

TEST_CASE("test_Sector_number_of_blocks_returns_error_on_invalid_sector_index", "[mifare]")
{
    uint8_t number_of_blocks;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, rc522_mifare_get_sector_number_of_blocks(40, &number_of_blocks));
}

TEST_CASE("test_Sector_number_by_block_address", "[mifare]")
{
    uint8_t addr = 0;
    TEST_ASSERT_EQUAL(0, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (3);
    TEST_ASSERT_EQUAL(0, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (1);
    TEST_ASSERT_EQUAL(1, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(2, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(3, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (7 * 4);
    TEST_ASSERT_EQUAL(10, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (20 * 4);
    TEST_ASSERT_EQUAL(30, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(31, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(32, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(32, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(32, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(32, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (4);
    TEST_ASSERT_EQUAL(33, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (16);
    TEST_ASSERT_EQUAL(34, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (5 * 16);
    TEST_ASSERT_EQUAL(39, rc522_mifare_get_sector_index_by_block_address(addr));
    addr += (15);
    TEST_ASSERT_EQUAL(39, rc522_mifare_get_sector_index_by_block_address(addr));
}

TEST_CASE("test_MIFAREMini_has_5_sectors", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_number_of_sectors(RC522_PICC_TYPE_MIFARE_MINI, &result));
    TEST_ASSERT_EQUAL_UINT8(5, result);
}

TEST_CASE("test_MIFARE1K_has_16_sectors", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_number_of_sectors(RC522_PICC_TYPE_MIFARE_1K, &result));
    TEST_ASSERT_EQUAL_UINT8(16, result);
}

TEST_CASE("test_MIFARE4K_has_40_sectors", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_number_of_sectors(RC522_PICC_TYPE_MIFARE_4K, &result));
    TEST_ASSERT_EQUAL_UINT8(40, result);
}

TEST_CASE("test_Error_sector_numbers_for_no_MIFARE_picc", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, rc522_mifare_get_number_of_sectors(RC522_PICC_TYPE_UNKNOWN, &result));
}

TEST_CASE("test_MIFARE1K_is_classic_compatible", "[mifare]")
{
    TEST_ASSERT_TRUE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_MIFARE_1K));
}

TEST_CASE("test_MIFARE4K_is_classic_compatible", "[mifare]")
{
    TEST_ASSERT_TRUE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_MIFARE_4K));
}

TEST_CASE("test_MIFAREMini_is_classic_compatible", "[mifare]")
{
    TEST_ASSERT_TRUE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_MIFARE_MINI));
}

TEST_CASE("test_NoMIFAREClassic_is_not_classic_compatible", "[mifare]")
{
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_MIFARE_UL));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_MIFARE_DESFIRE));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_MIFARE_PLUS));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_ISO_14443_4));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_ISO_18092));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_TNP3XXX));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_UNDEFINED));
    TEST_ASSERT_FALSE(rc522_mifare_type_is_classic_compatible(RC522_PICC_TYPE_UNKNOWN));
}
