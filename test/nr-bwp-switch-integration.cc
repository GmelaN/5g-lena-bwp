// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/bwp-manager-ue.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-control-messages.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;

namespace
{

class BwpSwitchIntegrationTestCase : public TestCase
{
  public:
    BwpSwitchIntegrationTestCase()
        : TestCase("Inject DL DCI with bwpId=1 and verify UE switches active BWP")
    {
    }

    void DoRun() override
    {
        NodeContainer gnbNodes;
        gnbNodes.Create(1);
        NodeContainer ueNodes;
        ueNodes.Create(1);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(gnbNodes);
        mobility.Install(ueNodes);

        Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
        nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

        CcBwpCreator ccBwpCreator;
        CcBwpCreator::SimpleOperationBandConf bandConf;
        bandConf.m_centralFrequency = 3.5e9;
        bandConf.m_channelBandwidth = 20e6;
        bandConf.m_numCc = 1;
        bandConf.m_numBwp = 2;

        OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
        Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
        channelHelper->AssignChannelsToBands({band});
        auto allBwps = CcBwpCreator::GetAllBwps({band});

        NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
        NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

        nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

        Ptr<NrUeNetDevice> ue = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
        Ptr<BwpManagerUe> bwpMgr = ue->GetBwpManager();
        Ptr<NrUePhy> uePhy0 = ue->GetPhy(0);
        uint16_t ueRnti = uePhy0->GetRnti();

        // Ensure a defined starting active BWP
        bwpMgr->ForceActiveBwp(0);

        // Inject a DL DCI for BWP1 with k0=1 slot after 1 ms to trigger switch.
        Simulator::Schedule(MilliSeconds(1.0),
                            [uePhy0, ueRnti]() {
                                auto dci = std::make_shared<DciInfoElementTdma>(ueRnti,
                                                                                   DciInfoElementTdma::DL,
                                                                                   0,
                                                                                   1,
                                                                                   0,
                                                                                   1,
                                                                                   nullptr,
                                                                                   10,
                                                                                   1,
                                                                                   0,
                                                                                   DciInfoElementTdma::DATA,
                                                                                   1, // target BWP
                                                                                   0);
                                Ptr<NrDlDciMessage> msg = Create<NrDlDciMessage>(dci);
                                msg->SetKDelay(1);
                                msg->SetK1Delay(1);
                                uePhy0->PhyCtrlMessagesReceived(msg);
                            });

        // Check initial active BWP and post-switch state.
        Simulator::Schedule(MilliSeconds(0.5),
                            [this, bwpMgr]() {
                                NS_TEST_ASSERT_MSG_EQ(bwpMgr->GetActiveBwp(),
                                                      0,
                                                      "Initial active BWP must be 0");
                            });

        Simulator::Schedule(MilliSeconds(3.0),
                            [this, bwpMgr]() {
                                NS_TEST_ASSERT_MSG_EQ(bwpMgr->GetActiveBwp(),
                                                      1,
                                                      "Active BWP did not switch to 1 after DCI");
                            });

        Simulator::Stop(MilliSeconds(5.0));
        Simulator::Run();
        Simulator::Destroy();
    }
};

class BwpSwitchIntegrationTestSuite : public TestSuite
{
  public:
    BwpSwitchIntegrationTestSuite()
        : TestSuite("nr-bwp-switch-integration", TestSuite::Type::SYSTEM)
    {
        AddTestCase(new BwpSwitchIntegrationTestCase, TestCase::Duration::QUICK);
    }
};

static BwpSwitchIntegrationTestSuite g_bwpSwitchIntegrationTestSuite;

} // namespace
