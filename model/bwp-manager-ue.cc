// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "bwp-manager-ue.h"

#include "bwp-manager-algorithm.h"
#include "nr-control-messages.h"

#include "ns3/log.h"
#include "ns3/pointer.h"
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("BwpManagerUe");
NS_OBJECT_ENSURE_REGISTERED(BwpManagerUe);

BwpManagerUe::BwpManagerUe()
    : NrSimpleUeComponentCarrierManager()
{
    NS_LOG_FUNCTION(this);
}

BwpManagerUe::~BwpManagerUe()
{
    NS_LOG_FUNCTION(this);
}

void
BwpManagerUe::SetBwpManagerAlgorithm(const Ptr<BwpManagerAlgorithm>& algorithm)
{
    NS_LOG_FUNCTION(this);
    m_algorithm = algorithm;
}

TypeId
BwpManagerUe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::BwpManagerUe")
                            .SetParent<NrSimpleUeComponentCarrierManager>()
                            .SetGroupName("nr")
                            .AddConstructor<BwpManagerUe>()
                            // .AddAttribute("SwitchingDelay",
                            //               "Delay between force command and applying the new UE BWP.",
                            //               TimeValue(Seconds(0)),
                            //               MakeTimeAccessor(&BwpManagerUe::m_switchingDelay),
                            //               MakeTimeChecker())
                            .AddAttribute("BwpManagerAlgorithm",
                                          "The algorithm pointer",
                                          PointerValue(),
                                          MakePointerAccessor(&BwpManagerUe::m_algorithm),
                                          MakePointerChecker<BwpManagerAlgorithm>());
    return tid;
}

void
BwpManagerUe::SetAttributeSwitchingDelay(Time t)
{
    m_switchingDelay = t;
}

void
BwpManagerUe::DoTransmitBufferStatusReport(NrMacSapProvider::BufferStatusReportParameters params)
{
    NS_LOG_FUNCTION(this);
    if (m_switchingUntil != Seconds(0) && Simulator::Now() < m_switchingUntil)
    {
        return;
    }
    // Route everything to the currently forced/active BWP. If none set yet, default to BWP 0.
    uint8_t bwpIndex = (m_activeBwpId == std::numeric_limits<uint8_t>::max()) ? 0 : m_activeBwpId;

    NS_LOG_DEBUG("BSR of size " << params.txQueueSize
                                << " from RLC for LCID = " << static_cast<uint32_t>(params.lcid)
                                << " traffic type " << m_lcToBearerMap.at(params.lcid)
                                << " reported to CcId " << static_cast<uint32_t>(bwpIndex));

    m_componentCarrierLcMap.at(bwpIndex).at(params.lcid)->BufferStatusReport(params);
}

std::vector<NrUeCcmRrcSapProvider::LcsConfig>
BwpManagerUe::DoAddLc(uint8_t lcId,
                      NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
                      NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("For LC ID " << static_cast<uint32_t>(lcId) << " bearer qci "
                             << static_cast<uint32_t>(lcConfig.priority) << " from priority "
                             << static_cast<uint32_t>(lcConfig.priority));

    // see nr-gnb-rrc.cc
    m_lcToBearerMap.insert(std::make_pair(lcId, static_cast<NrEpsBearer::Qci>(lcConfig.priority)));

    return NrSimpleUeComponentCarrierManager::DoAddLc(lcId, lcConfig, msu);
}

NrMacSapUser*
BwpManagerUe::DoConfigureSignalBearer(uint8_t lcId,
                                      NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
                                      NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this);

    m_lcToBearerMap.insert(std::make_pair(lcId, static_cast<NrEpsBearer::Qci>(lcConfig.priority)));

    return NrSimpleUeComponentCarrierManager::DoConfigureSignalBearer(lcId, lcConfig, msu);
}

uint8_t
BwpManagerUe::RouteDlHarqFeedback(const DlHarqInfo& m) const
{
    NS_LOG_FUNCTION(this);

    return m.m_bwpIndex;
}

void
BwpManagerUe::SetOutputLink(uint32_t sourceBwp, uint32_t outputBwp)
{
    NS_LOG_FUNCTION(this);
    m_outputLinks.insert(std::make_pair(sourceBwp, outputBwp));
}

uint8_t
BwpManagerUe::RouteOutgoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint8_t sourceBwpId) const
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("Msg type " << msg->GetMessageType() << " that wants to go out from UE");

    if (m_outputLinks.empty())
    {
        NS_LOG_INFO("No linked BWP, routing outgoing msg to the source: " << +sourceBwpId);
        return sourceBwpId;
    }

    auto it = m_outputLinks.find(sourceBwpId);
    if (it == m_outputLinks.end())
    {
        NS_LOG_INFO("Source BWP not in the map, routing outgoing msg to itself: " << +sourceBwpId);
        return sourceBwpId;
    }

    NS_LOG_INFO("routing outgoing msg to bwp: " << +it->second);
    return it->second;
}

uint8_t
BwpManagerUe::RouteIngoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint8_t sourceBwpId) const
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("Msg type " << msg->GetMessageType() << " comes from BWP " << +sourceBwpId
                            << " that wants to go in the UE, goes in BWP " << msg->GetSourceBwp());
    return msg->GetSourceBwp();
}

Ptr<const BwpManagerAlgorithm>
BwpManagerUe::GetAlgorithm() const
{
    return m_algorithm;
}

void
BwpManagerUe::ForceActiveBwp(uint8_t bwpId)
{
    NS_LOG_UNCOND("WE FORCED OUR UE TO USE " << +bwpId);
    NS_LOG_FUNCTION(this << static_cast<uint32_t>(bwpId));
    Time end = Simulator::Now() + m_switchingDelay;
    m_switchingUntil = end;
    Simulator::Schedule(m_switchingDelay, [this, bwpId]() {
        m_activeBwpId = bwpId;
        m_switchingUntil = Seconds(0);
        // Flush queued BSRs
        for (auto params : m_pendingBsr)
        {
            DoTransmitBufferStatusReport(params);
        }
        m_pendingBsr.clear();
    });
}

uint8_t
BwpManagerUe::GetActiveBwp() const
{
    return m_activeBwpId;
}

} // namespace ns3
