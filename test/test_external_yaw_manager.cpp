// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <gtest/gtest.h>

#include "emcl2/ExternalYawManager.h"

using emcl2::ExternalYawManager;

TEST(ExternalYawManagerTest, InitializesOffsetOnFirstMeasurement)
{
  ExternalYawManager mgr(0.5);
  mgr.updateMeasurement(1.0);
  ASSERT_TRUE(mgr.ready());
  EXPECT_NEAR(mgr.yaw(), 0.5, 1e-6);
}

TEST(ExternalYawManagerTest, ResetOffsetWithInitialPose)
{
  ExternalYawManager mgr(0.0);
  mgr.updateMeasurement(0.2);
  ASSERT_TRUE(mgr.ready());
  EXPECT_NEAR(mgr.yaw(), 0.0, 1e-6);

  const bool ok = mgr.applyInitialPose(1.0);
  ASSERT_TRUE(ok);
  EXPECT_NEAR(mgr.yaw(), 1.0, 1e-6);
}

TEST(ExternalYawManagerTest, NotReadyWithoutMeasurement)
{
  ExternalYawManager mgr(0.0);
  EXPECT_FALSE(mgr.ready());
  EXPECT_FALSE(mgr.haveMeasurement());
  EXPECT_FALSE(mgr.applyInitialPose(1.0));
}
