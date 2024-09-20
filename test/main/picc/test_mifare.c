#include "unity.h"

#include "picc/rc522_mifare.c"

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
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_number_of_sectors(RC522_PICC_TYPE_MIFARE_MINI, &result));
    TEST_ASSERT_EQUAL_UINT8(5, result);
}

TEST_CASE("test_MIFARE1K_has_16_sectors", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_number_of_sectors(RC522_PICC_TYPE_MIFARE_1K, &result));
    TEST_ASSERT_EQUAL_UINT8(16, result);
}

TEST_CASE("test_MIFARE4K_has_40_sectors", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_number_of_sectors(RC522_PICC_TYPE_MIFARE_4K, &result));
    TEST_ASSERT_EQUAL_UINT8(40, result);
}

TEST_CASE("test_Error_sector_numbers_for_no_MIFARE_picc", "[mifare]")
{
    uint8_t result;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, rc522_mifare_number_of_sectors(RC522_PICC_TYPE_UNKNOWN, &result));
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
