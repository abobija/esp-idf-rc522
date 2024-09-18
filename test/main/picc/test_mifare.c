#include "unity.h"

#include "picc/rc522_mifare.c"

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
