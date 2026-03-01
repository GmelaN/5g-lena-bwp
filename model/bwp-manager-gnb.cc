// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "bwp-manager-gnb.h"

#include "bwp-manager-algorithm.h"
#include "nr-control-messages.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/object-map.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"

#include <limits>
#include "nr-no-op-component-carrier-manager.h"
#include "nr-gnb-mac.h"


namespace ns3
{

NS_LOG_COMPONENT_DEFINE("BwpManagerGnb");
NS_OBJECT_ENSURE_REGISTERED(BwpManagerGnb);

BwpManagerGnb::BwpManagerGnb()
    : NrRrComponentCarrierManager()
{
    NS_LOG_FUNCTION(this);
    m_energyConfig.SetSwitchEnergy(0, 1, 0.0015);
    m_energyConfig.SetSwitchEnergy(1, 0, 0.0015);
}

BwpManagerGnb::~BwpManagerGnb()
{
    NS_LOG_FUNCTION(this);
}

TypeId
BwpManagerGnb::GetTypeId()
{
    static TypeId tid = TypeId("ns3::BwpManagerGnb")
                            .SetParent<NrNoOpComponentCarrierManager>()
                            .SetGroupName("nr")
                            .AddConstructor<BwpManagerGnb>()
                            .AddAttribute("SwitchingDelay",
                                          "Delay between receiving a force command and activating the new BWP.",
                                          TimeValue(Seconds(0)),
                                          MakeTimeAccessor(&BwpManagerGnb::m_switchingDelay),
                                          MakeTimeChecker())
                            .AddAttribute("BwpManagerAlgorithm",
                                          "The algorithm pointer",
                                          PointerValue(),
                                          MakePointerAccessor(&BwpManagerGnb::m_algorithm),
                                          MakePointerChecker<BwpManagerAlgorithm>())
                            .AddAttribute("TxPowerBwp0Mw",
                                          "Tx power (mW) assumed for BWP 0.",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&BwpManagerGnb::m_txPowerBwp0Mw),
                                          MakeDoubleChecker<double>(0.0))
                            .AddAttribute("TxPowerBwp1Mw",
                                          "Tx power (mW) assumed for BWP 1.",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&BwpManagerGnb::m_txPowerBwp1Mw),
                                          MakeDoubleChecker<double>(0.0))
                            .AddAttribute("TxBandwidthBwp0Hz",
                                          "Effective bandwidth (Hz) for BWP 0.",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&BwpManagerGnb::m_txBandwidthBwp0Hz),
                                          MakeDoubleChecker<double>(0.0))
                            .AddAttribute("TxBandwidthBwp1Hz",
                                          "Effective bandwidth (Hz) for BWP 1.",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&BwpManagerGnb::m_txBandwidthBwp1Hz),
                                          MakeDoubleChecker<double>(0.0))
                            .AddAttribute("TxSpectralEfficiency",
                                          "Assumed spectral efficiency (bit/s/Hz) for Tx energy.",
                                          DoubleValue(1.0),
                                          MakeDoubleAccessor(&BwpManagerGnb::m_txSpectralEfficiency),
                                          MakeDoubleChecker<double>(1e-9))
                            .AddTraceSource("BsrReport",
                                            "Buffer status report routed through BWP manager with resolved BWP.",
                                            MakeTraceSourceAccessor(&BwpManagerGnb::m_bsrTracedCallback),
                                            "ns3::TracedCallback<uint16_t, uint8_t, uint8_t, "
                                            "const ns3::NrMacSapProvider::BufferStatusReportParameters&>")
                            .AddTraceSource("TxEnergy",
                                            "Energy consumed per DL PDU transmission (J).",
                                            MakeTraceSourceAccessor(&BwpManagerGnb::m_txEnergyTrace),
                                            "ns3::TracedCallback<uint16_t, uint8_t, uint32_t, double>")
                            .AddTraceSource("SwitchEnergy",
                                            "Energy consumed when a BWP switch completes for a UE.",
                                            MakeTraceSourceAccessor(&BwpManagerGnb::m_switchEnergyTrace),
                                            "ns3::TracedCallback<uint16_t, uint8_t, uint8_t, double>");
    return tid;
}

void
BwpManagerGnb::SetBwpManagerAlgorithm(const Ptr<BwpManagerAlgorithm>& algorithm)
{
    NS_LOG_FUNCTION(this);
    m_algorithm = algorithm;
}

void
BwpManagerGnb::SetAttributeSwitchingDelay(Time t)
{
    m_switchingDelay = t;
}

uint8_t
BwpManagerGnb::GetResourceType(NrMacSapProvider::BufferStatusReportParameters params)
{
    NS_ASSERT_MSG(m_ueInfo.find(params.rnti) != m_ueInfo.end(),
                  "Trying to check the QoS of unknown UE");
    NS_ASSERT_MSG(m_ueInfo.at(params.rnti).m_rlcLcInstantiated.find(params.lcid) !=
                      m_ueInfo.at(params.rnti).m_rlcLcInstantiated.end(),
                  "Trying to check the QoS of unknown logical channel");
    return (m_ueInfo[params.rnti].m_rlcLcInstantiated[params.lcid]).resourceType;
}

double
BwpManagerGnb::ComputeTxEnergyJ(uint8_t bwpId, uint32_t bytes) const
{
    // NS_ASSERT_MSG(false, "\\phi 한번 봐야 함");
    if (bytes == 0 || m_txSpectralEfficiency <= 0.0)
    {
        return 0.0;
    }

    double powerMw = 0.0;
    double bandwidthHz = 0.0;
    if (bwpId == 0)
    {
        powerMw = m_txPowerBwp0Mw;
        bandwidthHz = m_txBandwidthBwp0Hz;
    }
    else if (bwpId == 1)
    {
        powerMw = m_txPowerBwp1Mw;
        bandwidthHz = m_txBandwidthBwp1Hz;
    }

    if (powerMw <= 0.0 || bandwidthHz <= 0.0)
    {
        return 0.0;
    }

    double bits = static_cast<double>(bytes) * 8.0;
    double airTimeSeconds = bits / (bandwidthHz * m_txSpectralEfficiency);
    return powerMw * 1e-3 * airTimeSeconds;
}

std::vector<NrCcmRrcSapProvider::LcsConfig>
BwpManagerGnb::DoSetupDataRadioBearer(NrEpsBearer bearer,
                                      uint8_t bearerId,
                                      uint16_t rnti,
                                      uint8_t lcid,
                                      uint8_t lcGroup,
                                      NrMacSapUser* msu)
{
    NS_LOG_FUNCTION(this);

    std::vector<NrCcmRrcSapProvider::LcsConfig> lcsConfig =
        NrRrComponentCarrierManager::DoSetupDataRadioBearer(bearer,
                                                            bearerId,
                                                            rnti,
                                                            lcid,
                                                            lcGroup,
                                                            msu);
    return lcsConfig;
}

uint8_t
BwpManagerGnb::GetBwpIndex(uint16_t rnti, uint8_t lcid)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_algorithm != nullptr);
    NS_ASSERT_MSG(m_ueInfo.find(rnti) != m_ueInfo.end(), "Unknown UE");
    NS_ASSERT_MSG(m_ueInfo.at(rnti).m_rlcLcInstantiated.find(lcid) !=
                      m_ueInfo.at(rnti).m_rlcLcInstantiated.end(),
                  "Unknown logical channel of UE");

    auto forced = m_forcedUeBwp.find(rnti);
    if (forced != m_forcedUeBwp.end())
    {
        // Ignore algorithm when a forced BWP is present.
        return forced->second;
    }

    NS_LOG_WARN("WARN: RETURNING DEFAULT BWP FOR UNKNOWN RNTI.");

    return 0;
    // NS_ASSERT(false);

    // uint8_t qci = m_ueInfo[rnti].m_rlcLcInstantiated[lcid].qci;

    // // Force a conversion between the uint8_t type that comes from the LcInfo
    // // struct (yeah, using the NrEpsBearer::Qci type was too hard ...)
    // return m_algorithm->GetBwpForEpsBearer(static_cast<NrEpsBearer::Qci>(qci));
}

uint8_t
BwpManagerGnb::PeekBwpIndex(uint16_t rnti, uint8_t lcid) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_algorithm != nullptr);
    // For the moment, Get and Peek are the same, but they'll change
    NS_ASSERT_MSG(m_ueInfo.find(rnti) != m_ueInfo.end(), "Unknown UE");
    NS_ASSERT_MSG(m_ueInfo.at(rnti).m_rlcLcInstantiated.find(lcid) !=
                      m_ueInfo.at(rnti).m_rlcLcInstantiated.end(),
                  "Unknown logical channel of UE");

    auto forced = m_forcedUeBwp.find(rnti);
    if (forced != m_forcedUeBwp.end())
    {
        return forced->second;
    }

    uint8_t qci = m_ueInfo.at(rnti).m_rlcLcInstantiated.at(lcid).qci;

    // Force a conversion between the uint8_t type that comes from the LcInfo
    // struct (yeah, using the NrEpsBearer::Qci type was too hard ...)
    return m_algorithm->GetBwpForEpsBearer(static_cast<NrEpsBearer::Qci>(qci));
}

uint8_t
BwpManagerGnb::RouteIngoingCtrlMsgs(const Ptr<NrControlMessage>& msg, uint8_t sourceBwpId) const
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("Msg type " << msg->GetMessageType() << " from bwp " << +sourceBwpId
                            << " that wants to go in the gnb, goes in BWP " << msg->GetSourceBwp());
    return msg->GetSourceBwp();
}

uint8_t
BwpManagerGnb::RouteOutgoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint8_t sourceBwpId) const
{
    NS_LOG_FUNCTION(this);

    NS_LOG_INFO("Msg type " << msg->GetMessageType() << " from bwp " << +sourceBwpId
                            << " that wants to go out from gnb");

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

void
BwpManagerGnb::SetOutputLink(uint32_t sourceBwp, uint32_t outputBwp)
{
    NS_LOG_FUNCTION(this);
    m_outputLinks.insert(std::make_pair(sourceBwp, outputBwp));
}

void
BwpManagerGnb::ForceUeBwp(uint16_t rnti, uint8_t bwpId)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<uint32_t>(bwpId));
    NS_LOG_INFO("ForceUeBwp rnti=" << rnti << " targetBwp=" << +bwpId);
    // Mark switching window: deactivate everywhere now, activate target after delay.
    Time end = Simulator::Now() + m_switchingDelay;
    m_switchingUntil[rnti] = end;
    uint8_t fromBwp = GetForcedUeBwp(rnti);
    uint8_t toBwp = bwpId;
    for (const auto& kv : m_macObjects)
    {
        kv.second->SetUeActive(rnti, false);
    }
    Simulator::Schedule(m_switchingDelay, [=, this, fromBwp, toBwp]() {
        m_forcedUeBwp[rnti] = bwpId;
        m_switchingUntil.erase(rnti);
        for (const auto& kv : m_macObjects)
        {
            kv.second->SetUeActive(rnti, kv.first == bwpId);
        }
        double energyJ = m_energyConfig.GetSwitchEnergy(fromBwp, toBwp);
        if (energyJ > 0.0)
        {
            m_switchEnergyTrace(rnti, fromBwp, toBwp, energyJ);
        }
        FlushPending(rnti);
    });
}

uint8_t
BwpManagerGnb::GetForcedUeBwp(uint16_t rnti) const
{
    auto it = m_forcedUeBwp.find(rnti);
    if (it == m_forcedUeBwp.end())
    {
        return std::numeric_limits<uint8_t>::max();
    }
    return it->second;
}

bool
BwpManagerGnb::IsSwitching(uint16_t rnti) const
{
    auto it = m_switchingUntil.find(rnti);
    return it != m_switchingUntil.end() && Simulator::Now() < it->second;
}

Time
BwpManagerGnb::GetSwitchingRemaining(uint16_t rnti) const
{
    auto it = m_switchingUntil.find(rnti);
    if (it == m_switchingUntil.end())
    {
        return Seconds(0);
    }
    if (Simulator::Now() >= it->second)
    {
        return Seconds(0);
    }
    return it->second - Simulator::Now();
}

void
BwpManagerGnb::SetMacObjects(const std::map<uint8_t, Ptr<NrGnbMac>>& macObjects)
{
    m_macObjects = macObjects;
}

void
BwpManagerGnb::SetUePriority(uint16_t rnti, uint8_t priority)
{
    for (const auto& kv : m_macObjects)
    {
        kv.second->SetExternalUePriority(rnti, priority);
    }
}

const NrBwpEnergyConfig&
BwpManagerGnb::GetEnergyConfig() const
{
    return m_energyConfig;
}

NrBwpEnergyConfig&
BwpManagerGnb::GetEnergyConfig()
{
    return m_energyConfig;
}

void
BwpManagerGnb::DoTransmitPdu(NrMacSapProvider::TransmitPduParameters params)
{
    NS_LOG_FUNCTION(this);

    auto swIt = m_switchingUntil.find(params.rnti);
    if (swIt != m_switchingUntil.end() && Simulator::Now() < swIt->second)
    {
        m_pendingDlPdu[params.rnti].push_back(params);
        return;
    }

    // Override the ccId with a forced BWP if present.
    auto forcedIt = m_forcedUeBwp.find(params.rnti);
    if (forcedIt != m_forcedUeBwp.end())
    {
        params.componentCarrierId = forcedIt->second;
    }

    if (params.pdu)
    {
        uint32_t bytes = params.pdu->GetSize();
        double energyJ = ComputeTxEnergyJ(params.componentCarrierId, bytes);
        if (energyJ > 0.0)
        {
            m_txEnergyTrace(params.rnti, params.componentCarrierId, bytes, energyJ);
        }
    }

    auto it = m_macSapProvidersMap.find(params.componentCarrierId);
    NS_ABORT_MSG_IF(it == m_macSapProvidersMap.end(),
                    "could not find Sap for NrComponentCarrier "
                        << static_cast<uint32_t>(params.componentCarrierId));
    it->second->TransmitPdu(params);
}

void
BwpManagerGnb::DoTransmitBufferStatusReport(NrMacSapProvider::BufferStatusReportParameters params)
{
    NS_LOG_FUNCTION(this);

    auto swIt = m_switchingUntil.find(params.rnti);
    if (swIt != m_switchingUntil.end() && Simulator::Now() < swIt->second)
    {
        m_pendingBsr[params.rnti].push_back(params);
        return;
    }

    uint8_t bwpIndex = GetBwpIndex(params.rnti, params.lcid);
    m_bsrTracedCallback(params.rnti, params.lcid, bwpIndex, params);

    if (m_macSapProvidersMap.find(bwpIndex) != m_macSapProvidersMap.end())
    {
        m_macSapProvidersMap.find(bwpIndex)->second->BufferStatusReport(params);
    }
    else
    {
        NS_ABORT_MSG("Bwp index " << +bwpIndex << " not valid.");
    }
}

void
BwpManagerGnb::DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams)
{
    NS_LOG_FUNCTION(this);
    auto rntiIt = m_ueInfo.find(txOpParams.rnti);
    NS_ASSERT_MSG(rntiIt != m_ueInfo.end(), "could not find RNTI" << txOpParams.rnti);

    auto lcidIt = rntiIt->second.m_ueAttached.find(txOpParams.lcid);
    NS_ASSERT_MSG(lcidIt != rntiIt->second.m_ueAttached.end(),
                  "could not find LCID " << (uint16_t)txOpParams.lcid);

    (*lcidIt).second->NotifyTxOpportunity(txOpParams);
}

void
BwpManagerGnb::DoUlReceiveMacCe(nr::MacCeListElement_s bsr, uint8_t componentCarrierId)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_algorithm != nullptr);
    NS_ASSERT_MSG(bsr.m_macCeType == nr::MacCeListElement_s::BSR,
                  "Received a Control Message not allowed " << bsr.m_macCeType);
    NS_ASSERT_MSG(m_ccmMacSapProviderMap.find(componentCarrierId) != m_ccmMacSapProviderMap.end(),
                  "Mac sap provider does not exist.");

    NS_LOG_DEBUG("Routing BSR for UE " << bsr.m_rnti << " to source CC id "
                                       << static_cast<uint32_t>(componentCarrierId));

    auto swIt = m_switchingUntil.find(bsr.m_rnti);
    if (swIt != m_switchingUntil.end() && Simulator::Now() < swIt->second)
    {
        return; // UL CE not queued to keep logic simple
    }

    auto forcedIt = m_forcedUeBwp.find(bsr.m_rnti);
    uint8_t targetCc = componentCarrierId;
    if (forcedIt != m_forcedUeBwp.end())
    {
        targetCc = forcedIt->second;
    }

    if (m_ccmMacSapProviderMap.find(targetCc) != m_ccmMacSapProviderMap.end())
    {
        m_ccmMacSapProviderMap.find(targetCc)->second->ReportMacCeToScheduler(bsr);
    }
    else
    {
        NS_ABORT_MSG("Bwp index not valid.");
    }
}

void
BwpManagerGnb::DoUlReceiveSr(uint16_t rnti, uint8_t componentCarrierId)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_algorithm != nullptr);

    auto forcedIt = m_forcedUeBwp.find(rnti);
    auto swIt = m_switchingUntil.find(rnti);
    if (swIt != m_switchingUntil.end() && Simulator::Now() < swIt->second)
    {
        m_pendingSr[rnti].push_back(componentCarrierId);
        return;
    }
    uint8_t targetCc = componentCarrierId;
    if (forcedIt != m_forcedUeBwp.end())
    {
        targetCc = forcedIt->second;
    }

    NS_LOG_DEBUG("Routing SR for UE " << rnti << " to cc id "
                                      << static_cast<uint32_t>(targetCc));

    auto it = m_ccmMacSapProviderMap.find(targetCc);
    NS_ABORT_IF(it == m_ccmMacSapProviderMap.end());

    m_ccmMacSapProviderMap.find(targetCc)->second->ReportSrToScheduler(rnti);
}

void
BwpManagerGnb::FlushPending(uint16_t rnti)
{
    // Flush DL PDUs
    auto pduIt = m_pendingDlPdu.find(rnti);
    if (pduIt != m_pendingDlPdu.end())
    {
        for (auto params : pduIt->second)
        {
            auto forced = m_forcedUeBwp.find(rnti);
            if (forced != m_forcedUeBwp.end())
            {
                params.componentCarrierId = forced->second;
            }
            auto it = m_macSapProvidersMap.find(params.componentCarrierId);
            if (it != m_macSapProvidersMap.end())
            {
                it->second->TransmitPdu(params);
            }
        }
        m_pendingDlPdu.erase(pduIt);
    }

    // Flush BSRs
    auto bsrIt = m_pendingBsr.find(rnti);
    if (bsrIt != m_pendingBsr.end())
    {
        for (auto params : bsrIt->second)
        {
            DoTransmitBufferStatusReport(params);
        }
        m_pendingBsr.erase(bsrIt);
    }

    // Flush SRs
    auto srIt = m_pendingSr.find(rnti);
    if (srIt != m_pendingSr.end())
    {
        for (auto cc : srIt->second)
        {
            DoUlReceiveSr(rnti, cc);
        }
        m_pendingSr.erase(srIt);
    }
}

} // end of namespace ns3
