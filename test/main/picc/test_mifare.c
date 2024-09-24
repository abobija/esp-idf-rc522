#include "unity.h"

#include "picc/rc522_mifare.c"

TEST_CASE("test_Block_at_address_is_sector_trailer", "[mifare]")
{
    bool result = true;

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_block_at_address_is_sector_trailer(0, &result));
    TEST_ASSERT_FALSE(result);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_block_at_address_is_sector_trailer(3, &result));
    TEST_ASSERT_TRUE(result);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_block_at_address_is_sector_trailer(4, &result));
    TEST_ASSERT_FALSE(result);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_block_at_address_is_sector_trailer(5, &result));
    TEST_ASSERT_FALSE(result);

    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_block_at_address_is_sector_trailer(7, &result));
    TEST_ASSERT_TRUE(result);
}

TEST_CASE("test_Sector_block_group_index", "[mifare]")
{
    rc522_mifare_sector_desc_t sector;
    uint8_t group = 0;

    sector.number_of_blocks = 4;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 3, &group));
    TEST_ASSERT_EQUAL(3, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 2, &group));
    TEST_ASSERT_EQUAL(2, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 1, &group));
    TEST_ASSERT_EQUAL(1, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 0, &group));
    TEST_ASSERT_EQUAL(0, group);

    sector.number_of_blocks = 16;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 15, &group));
    TEST_ASSERT_EQUAL(3, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 14, &group));
    TEST_ASSERT_EQUAL(2, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 10, &group));
    TEST_ASSERT_EQUAL(2, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 9, &group));
    TEST_ASSERT_EQUAL(1, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 5, &group));
    TEST_ASSERT_EQUAL(1, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 4, &group));
    TEST_ASSERT_EQUAL(0, group);
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_sector_block_group_index(&sector, 0, &group));
    TEST_ASSERT_EQUAL(0, group);

    sector.number_of_blocks = 4;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rc522_mifare_get_sector_block_group_index(&sector, 5, &group));
}

TEST_CASE("test_Block_is_value_based_on_access_bits", "[mifare]")
{
    rc522_mifare_access_bits_t access_bits;

    access_bits.c1 = 0;
    access_bits.c2 = 0;
    access_bits.c3 = 0;
    TEST_ASSERT_FALSE(rc522_mifare_block_is_value(access_bits));

    access_bits.c3 = 1;
    TEST_ASSERT_TRUE(rc522_mifare_block_is_value(access_bits));

    access_bits.c1 = 1;
    access_bits.c2 = 1;
    access_bits.c3 = 0;
    TEST_ASSERT_TRUE(rc522_mifare_block_is_value(access_bits));

    access_bits.c1 = 0;
    TEST_ASSERT_FALSE(rc522_mifare_block_is_value(access_bits));
}

TEST_CASE("test_Access_bits_after_parsing_sector_trailer", "[mifare]")
{
    // +-----+----------------+----+----+----+
    // |     |                | c1 | c2 | c3 |
    // +-----+----------------+----+----+----+
    // | [3] | Sector trailer | 0  | 0  | 1  |
    // | [2] | Block 2        | 1  | 0  | 1  |
    // | [1] | Block 1        | 0  | 1  | 0  |
    // | [0] | Block 0        | 1  | 1  | 1  |
    // +-----+----------------+----+----+----+

    uint8_t c3 = 0b1101;
    uint8_t c2 = 0b0011;
    uint8_t c1 = 0b0101;

    uint8_t bytes[16];
    memset(bytes, 0, 16);

    bytes[6] = ((~c2) << 4) | ((~c1) & 0x0F);
    bytes[7] = (c1 << 4) | ((~c3) & 0x0F);
    bytes[8] = (c3 << 4) | (c2 & 0x0F);

    rc522_mifare_sector_trailer_info_t trailer_info;
    TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_parse_sector_trailer(bytes, 16, &trailer_info));

    TEST_ASSERT_EQUAL(0, trailer_info.access_bits[3].c1);
    TEST_ASSERT_EQUAL(0, trailer_info.access_bits[3].c2);
    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[3].c3);

    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[2].c1);
    TEST_ASSERT_EQUAL(0, trailer_info.access_bits[2].c2);
    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[2].c3);

    TEST_ASSERT_EQUAL(0, trailer_info.access_bits[1].c1);
    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[1].c2);
    TEST_ASSERT_EQUAL(0, trailer_info.access_bits[1].c3);

    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[0].c1);
    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[0].c2);
    TEST_ASSERT_EQUAL(1, trailer_info.access_bits[0].c3);

    // Invert just one bit (C20)
    // and expect integrity violation error
    bytes[8] ^= 0x01;
    TEST_ASSERT_EQUAL(RC522_ERR_MIFARE_ACCESS_BITS_INTEGRITY_VIOLATION,
        rc522_mifare_parse_sector_trailer(bytes, 16, &trailer_info));
}

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
        TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_number_of_blocks_in_sector(sector_index, &number_of_blocks));
        TEST_ASSERT_EQUAL(4, number_of_blocks);
    }

    for (uint8_t sector_index = 32; sector_index < 40; sector_index++) {
        TEST_ASSERT_EQUAL(ESP_OK, rc522_mifare_get_number_of_blocks_in_sector(sector_index, &number_of_blocks));
        TEST_ASSERT_EQUAL(16, number_of_blocks);
    }
}

TEST_CASE("test_Sector_number_of_blocks_returns_error_on_invalid_sector_index", "[mifare]")
{
    uint8_t number_of_blocks;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, rc522_mifare_get_number_of_blocks_in_sector(40, &number_of_blocks));
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
