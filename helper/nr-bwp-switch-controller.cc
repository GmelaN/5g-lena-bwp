// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-bwp-switch-controller.h"
#include "nr-bwp-switch-trigger-helper.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrBwpSwitchController");
NS_OBJECT_ENSURE_REGISTERED(NrBwpSwitchController);

TypeId
NrBwpSwitchController::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrBwpSwitchController")
            .SetParent<Object>()
            .SetGroupName("nr")
            .AddConstructor<NrBwpSwitchController>()
            .AddAttribute("UseTriggerHelper",
                          "Route BSR updates to the trigger helper and use its decisions.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrBwpSwitchController::m_useTriggerHelper),
                          MakeBooleanChecker());
    return tid;
}

NrBwpSwitchController::NrBwpSwitchController()
    : Object()
{
    NS_LOG_FUNCTION(this);
    m_triggerHelper = CreateObject<NrBwpSwitchTriggerHelper>();
    m_triggerHelper->SetDecisionCallback(MakeCallback(&NrBwpSwitchController::OnHelperDecision, this));
    // Delay helper evaluations to avoid interfering with RA/RRC.
    m_triggerHelper->SetAttribute("StartDelay", TimeValue(Seconds(0.5)));
    m_triggerHelper->SetAttribute("EnableInternalPolicy", BooleanValue(false));
}

void
NrBwpSwitchController::SetGnbManager(const Ptr<BwpManagerGnb>& gnbManager)
{
    m_gnbManager = gnbManager;
    if (m_triggerHelper)
    {
        m_triggerHelper->SetGnbManager(gnbManager);
    }
}

void
NrBwpSwitchController::AddUeManager(uint16_t rnti, const Ptr<BwpManagerUe>& ueManager)
{
    m_ueManagers[rnti] = ueManager;
    if (m_triggerHelper)
    {
        m_triggerHelper->AddUeManager(rnti, ueManager);
    }
}

void
NrBwpSwitchController::SetPolicy(const PolicyCallback& cb)
{
    m_policy = cb;
    if (m_triggerHelper)
    {
        m_triggerHelper->SetPolicy(cb);
    }
}

Ptr<NrBwpSwitchTriggerHelper>
NrBwpSwitchController::GetTriggerHelper() const
{
    return m_triggerHelper;
}

void
NrBwpSwitchController::HandleBsr(uint16_t rnti,
                                 uint8_t lcid,
                                 uint8_t currentBwp,
                                 const NrMacSapProvider::BufferStatusReportParameters& params)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<uint32_t>(lcid) << static_cast<uint32_t>(currentBwp));

    if (m_gnbManager == nullptr)
    {
        return;
    }

    if (m_useTriggerHelper && m_triggerHelper)
    {
        m_triggerHelper->NotifyBsr(rnti, lcid, currentBwp, 0, params);
        return;
    }

    if (m_policy.IsNull())
    {
        return;
    }

    NrBwpSwitchState state;
    state.bsr = params;
    state.currentBwpId = currentBwp;
    state.switchingRemaining = m_gnbManager->GetSwitchingRemaining(rnti);

    auto decision = RunPolicy(state);
    if (decision.targetBwpId != std::numeric_limits<uint8_t>::max() &&
        decision.targetBwpId != currentBwp && !m_gnbManager->IsSwitching(rnti))
    {
        ApplySwitch(rnti, decision.targetBwpId);
    }
}

void
NrBwpSwitchController::RecordEnqueue(uint16_t rnti, uint8_t lcid)
{
    uint64_t key = (static_cast<uint64_t>(rnti) << 8) | lcid;
    m_lastEnqueue[key] = Simulator::Now();
    if (m_triggerHelper)
    {
        m_triggerHelper->RecordEnqueue(rnti, lcid);
    }
}

void
NrBwpSwitchController::RecordAck(uint16_t rnti, uint8_t lcid)
{
    uint64_t key = (static_cast<uint64_t>(rnti) << 8) | lcid;
    auto it = m_lastEnqueue.find(key);
    if (it == m_lastEnqueue.end())
    {
        return;
    }
    Time aoi = Simulator::Now() - it->second;
    m_aoiAccumulatedSeconds += aoi.GetSeconds();
    m_aoiSamples++;
    m_lastEnqueue.erase(it);
    if (m_triggerHelper)
    {
        m_triggerHelper->RecordAck(rnti, lcid);
    }
}

void
NrBwpSwitchController::RecordAoiSample(Time delay)
{
    m_aoiAccumulatedSeconds += delay.GetSeconds();
    m_aoiSamples++;
}

double
NrBwpSwitchController::GetAverageAoISeconds() const
{
    if (m_aoiSamples == 0)
    {
        return 0.0;
    }
    return m_aoiAccumulatedSeconds / static_cast<double>(m_aoiSamples);
}

double
NrBwpSwitchController::GetTotalSwitchEnergyJ() const
{
    return m_totalSwitchEnergyJ;
}

NrBwpSwitchDecision
NrBwpSwitchController::RunPolicy(const NrBwpSwitchState& state) const
{
    if (m_policy.IsNull())
    {
        return {};
    }
    return m_policy(state);
}

void
NrBwpSwitchController::OnHelperDecision(uint16_t rnti, const NrBwpSwitchDecision& decision)
{
    if (decision.targetBwpId != std::numeric_limits<uint8_t>::max() &&
        m_gnbManager != nullptr && !m_gnbManager->IsSwitching(rnti))
    {
        ApplySwitch(rnti, decision.targetBwpId);
    }
}

double
NrBwpSwitchController::LookupSwitchEnergy(uint8_t fromBwp, uint8_t toBwp) const
{
    if (m_gnbManager == nullptr)
    {
        return 0.0;
    }
    return m_gnbManager->GetEnergyConfig().GetSwitchEnergy(fromBwp, toBwp);
}

void
NrBwpSwitchController::ApplySwitch(uint16_t rnti, uint8_t targetBwp)
{
    DoForceBothSides(rnti, targetBwp);
}

void
NrBwpSwitchController::DoForceBothSides(uint16_t rnti, uint8_t targetBwp)
{
    if (m_gnbManager == nullptr)
    {
        return;
    }

    // Retry until not switching and UE entry present; mimic manual force logic.
    if (m_gnbManager->IsSwitching(rnti))
    {
        Simulator::Schedule(m_retryDelay, &NrBwpSwitchController::DoForceBothSides, this, rnti, targetBwp);
        return;
    }

    uint8_t current = m_gnbManager->GetForcedUeBwp(rnti);
    if (current == targetBwp)
    {
        return;
    }
    double e = LookupSwitchEnergy(current, targetBwp);
    m_totalSwitchEnergyJ += e;

    NS_LOG_UNCOND(Simulator::Now().As(Time::MS) << "\tSWITCH_APPLY rnti=" << rnti
                                                << " fromBwp=" << +current
                                                << " toBwp=" << +targetBwp
                                                << " switchEnergyJ=" << e);
    NS_LOG_INFO("Controller switching UE " << rnti << " from BWP " << +current << " to "
                                           << +targetBwp << " energyJ=" << e);
    m_gnbManager->ForceUeBwp(rnti, targetBwp);

    auto ueIt = m_ueManagers.find(rnti);
    if (ueIt != m_ueManagers.end())
    {
        ueIt->second->ForceActiveBwp(targetBwp);
    }
    else
    {
        // UE manager not yet registered; retry once later.
        Simulator::Schedule(m_retryDelay, &NrBwpSwitchController::DoForceBothSides, this, rnti, targetBwp);
    }
}

} // namespace ns3
