// Copyright 2018 Project March.
#include "../mocks/mock_sdo_interface.h"
#include "march_hardware/ethercat/imotioncube_pdo_map.h"

#include <gtest/gtest.h>

class IMCPDOTest : public ::testing::Test {
protected:
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    march::SdoSlaveInterface sdo
        = march::SdoSlaveInterface(/*slave_index=*/1, mock_sdo);
};

TEST_F(IMCPDOTest, sortPDOmap)
{
    march::IMCPDOmap pdoMapMISO;
    pdoMapMISO.addObject(march::IMCObjectName::StatusWord);
    pdoMapMISO.addObject(march::IMCObjectName::ActualPosition);
    std::unordered_map<march::IMCObjectName, uint8_t> misoByteOffsets
        = pdoMapMISO.map(this->sdo, march::DataDirection::MISO);

    ASSERT_EQ(0u, misoByteOffsets[march::IMCObjectName::ActualPosition]);
    ASSERT_EQ(4u, misoByteOffsets[march::IMCObjectName::StatusWord]);
}

TEST_F(IMCPDOTest, InvalidDataDirection)
{
    march::IMCPDOmap map;
    std::unordered_map<march::IMCObjectName, uint8_t> expected;
    ASSERT_EQ(map.map(this->sdo, (march::DataDirection)7), expected);
}

TEST_F(IMCPDOTest, multipleAddObjects)
{
    march::IMCPDOmap pdoMapMISO;

    pdoMapMISO.addObject(march::IMCObjectName::ActualPosition);
    pdoMapMISO.addObject(march::IMCObjectName::StatusWord);
    pdoMapMISO.addObject(march::IMCObjectName::StatusWord);
    std::unordered_map<march::IMCObjectName, uint8_t> misoByteOffsets
        = pdoMapMISO.map(this->sdo, march::DataDirection::MISO);
    ASSERT_EQ(2u, misoByteOffsets.size());
}

TEST_F(IMCPDOTest, ObjectCounts)
{
    march::IMCPDOmap pdoMapMISO;

    pdoMapMISO.addObject(march::IMCObjectName::CurrentLimit);
    std::unordered_map<march::IMCObjectName, uint8_t> misoByteOffsets
        = pdoMapMISO.map(this->sdo, march::DataDirection::MISO);

    ASSERT_EQ(1u, misoByteOffsets.count(march::IMCObjectName::CurrentLimit));
    ASSERT_EQ(0u, misoByteOffsets.count(march::IMCObjectName::DCLinkVoltage));
}

TEST_F(IMCPDOTest, CombinedAddressConstruct)
{
    march::IMCPDOmap pdoMap;

    auto status_word
        = pdoMap.all_objects.find(march::IMCObjectName::StatusWord);
    uint32_t combined_address = status_word->second.combined_address;

    ASSERT_EQ(16u, (combined_address & 0xFF));
    ASSERT_EQ(0u, ((combined_address >> 8) & 0xFF));
    ASSERT_EQ(0x6041u, ((combined_address >> 16) & 0xFFFF));
}

TEST_F(IMCPDOTest, CombinedAdressConstructWithSubindexValue)
{
    march::IMCPDOmap pdoMap;

    auto test_object = march::IMCObject(
        /*_address=*/0x6060, /*_sub_index=*/2, /*_length=*/16);
    uint32_t combined_address = test_object.combined_address;

    ASSERT_EQ(16u, (combined_address & 0xFF));
    ASSERT_EQ(2u, ((combined_address >> 8) & 0xFF));
    ASSERT_EQ(0x6060u, ((combined_address >> 16) & 0xFFFF));
}
