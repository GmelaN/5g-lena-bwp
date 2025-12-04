// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/bwp-manager-ue.h"
#include "ns3/nr-control-messages.h"
#include "ns3/test.h"

using namespace ns3;

namespace
{

class BwpManagerUeSwitchTestCase : public TestCase
{
  public:
    BwpManagerUeSwitchTestCase()
        : TestCase("ForceActiveBwp overrides outgoing routing")
    {
    }

    void DoRun() override
    {
        Ptr<BwpManagerUe> mgr = CreateObject<BwpManagerUe>();

        // Default: route to source when no forced BWP
        Ptr<NrRarMessage> msg = Create<NrRarMessage>();
        msg->SetSourceBwp(1);
        uint8_t routed = mgr->RouteOutgoingCtrlMsg(msg, 1);
        NS_TEST_ASSERT_MSG_EQ(routed, 1, "Should route to source when no force is set");

        // Force BWP 2 and verify routing
        mgr->ForceActiveBwp(2);
        routed = mgr->RouteOutgoingCtrlMsg(msg, 1);
        NS_TEST_ASSERT_MSG_EQ(routed, 2, "Forced active BWP must override routing");

        NS_TEST_ASSERT_MSG_EQ(mgr->GetActiveBwp(), 2, "Getter mismatch");
    }
};

class BwpSwitchTestSuite : public TestSuite
{
  public:
    BwpSwitchTestSuite()
        : TestSuite("nr-bwp-switch", TestSuite::Type::UNIT)
    {
        AddTestCase(new BwpManagerUeSwitchTestCase, TestCase::Duration::QUICK);
    }
};

static BwpSwitchTestSuite g_bwpSwitchTestSuite;

} // namespace
