// Copyright 2018 Project March.
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/Joint.h"
#include "mocks/MockTemperatureGES.cpp"
#include "mocks/MockIMotionCube.cpp"

class JointTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    this->imc = std::make_unique<MockIMotionCube>();
  }

  std::unique_ptr<MockIMotionCube> imc;
};

TEST_F(JointTest, AllowActuation)
{
  march::Joint joint("test", 0);
  joint.setAllowActuation(true);
  ASSERT_TRUE(joint.canActuate());
}

TEST_F(JointTest, DisableActuation)
{
  march::Joint joint("test", 0);
  joint.setAllowActuation(false);
  ASSERT_FALSE(joint.canActuate());
}

TEST_F(JointTest, ActuatePositionDisableActuation)
{
  march::Joint joint("actuate_false", 0, false, std::move(this->imc));
  EXPECT_FALSE(joint.canActuate());
  ASSERT_THROW(joint.actuateRad(0.3), march::error::HardwareException);
}

TEST_F(JointTest, ActuateTorqueDisableActuation)
{
  march::Joint joint("actuate_false", 0, false, std::move(this->imc));
  EXPECT_FALSE(joint.canActuate());
  ASSERT_THROW(joint.actuateTorque(3), march::error::HardwareException);
}

TEST_F(JointTest, PrepareForActuationWithUnknownMode)
{
  march::Joint joint("actuate_false", 0, true, std::move(this->imc));
  ASSERT_THROW(joint.prepareActuation(), march::error::HardwareException);
}
