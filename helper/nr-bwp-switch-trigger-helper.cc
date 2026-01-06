// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-bwp-switch-trigger-helper.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/make-event.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrBwpSwitchTriggerHelper");
NS_OBJECT_ENSURE_REGISTERED(NrBwpSwitchTriggerHelper);

TypeId
NrBwpSwitchTriggerHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrBwpSwitchTriggerHelper")
            .SetParent<Object>()
            .SetGroupName("nr")
            .AddConstructor<NrBwpSwitchTriggerHelper>()
            .AddAttribute("EvaluationPeriod",
                          "Period to aggregate state and run the policy.",
                          TimeValue(MilliSeconds(50)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_evalPeriod),
                          MakeTimeChecker())
            .AddAttribute("QueueThreshold1",
                          "Lower queue threshold for discretization (bytes).",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_queueThreshold1),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("QueueThreshold2",
                          "Upper queue threshold for discretization (bytes).",
                          UintegerValue(8000),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_queueThreshold2),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("AoIThreshold1",
                          "Lower AoI threshold for discretization.",
                          TimeValue(MilliSeconds(0.5)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_aoiThreshold1),
                          MakeTimeChecker())
            .AddAttribute("AoIThreshold2",
                          "Upper AoI threshold for discretization.",
                          TimeValue(MilliSeconds(2)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_aoiThreshold2),
                          MakeTimeChecker())
            .AddAttribute("EnergyPerByteJ",
                          "Energy per byte transmitted on a BWP (approximate).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_energyPerByteJ),
                          MakeDoubleChecker<double>())
            .AddAttribute("PreferenceEnergy",
                          "Preference weight for energy in Pareto Q-learning scalarization.",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_prefEnergy),
                          MakeDoubleChecker<double>())
            .AddAttribute("PreferenceAoI",
                          "Preference weight for AoI in Pareto Q-learning scalarization.",
                          DoubleValue(0.8),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_prefAoI),
                          MakeDoubleChecker<double>())
            .AddAttribute("StaticPowerBwp0Mw",
                          "Static power draw (mW) when on BWP 0.",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_staticPowerBwp0Mw),
                          MakeDoubleChecker<double>())
            .AddAttribute("StaticPowerBwp1Mw",
                          "Static power draw (mW) when on BWP 1.",
                          DoubleValue(400.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_staticPowerBwp1Mw),
                          MakeDoubleChecker<double>())
            .AddAttribute("LearningRate",
                          "Q-learning step size alpha.",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_alpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("LearningRateMin",
                          "Lower bound for decayed learning rate.",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_alphaMin),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("Discount",
                          "Q-learning discount factor gamma.",
                          DoubleValue(0.95),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_gamma),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("Epsilon",
                          "Epsilon for epsilon-greedy action selection.",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_epsilon),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EpsilonMin",
                          "Minimum epsilon after decay.",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_epsilonMin),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EpsilonDecay",
                          "Decay rate for epsilon; applied as exp(-decay * evalCount).",
                          DoubleValue(1e-4),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_epsilonDecay),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("RewardEnergyNorm",
                          "Normalization constant (J) applied to energy reward.",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_rewardEnergyNorm),
                          MakeDoubleChecker<double>(1e-9))
            .AddAttribute("RewardAoiNorm",
                          "Normalization constant (s) applied to AoI reward.",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_rewardAoiNorm),
                          MakeDoubleChecker<double>(1e-9))
            .AddAttribute("EnableInternalPolicy",
                          "Enable built-in Pareto Q-learning policy when no external policy is set.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrBwpSwitchTriggerHelper::m_enableInternalPolicy),
                          MakeBooleanChecker())
            .AddAttribute("InternalPolicyMode",
                          "Select internal policy when no external policy is set.",
                          EnumValue(POLICY_Q_LEARNING),
                          MakeEnumAccessor(&NrBwpSwitchTriggerHelper::m_internalPolicyMode),
                          MakeEnumChecker(POLICY_Q_LEARNING,
                                          "Qlearning",
                                          POLICY_WINDOW_DETECT,
                                          "WindowDetect"))
            .AddAttribute("SwitchThresholdBytes",
                          "Queue threshold (tau) for window-based switching (bytes).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_switchThresholdBytes),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("ControlWindow",
                          "Periodic control window (T_C). Zero disables periodic decision.",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_controlWindow),
                          MakeTimeChecker())
            .AddAttribute("DetectTime",
                          "Detect time (T_D) to force Specific after sustained backlog on Default.",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_detectTime),
                          MakeTimeChecker())
            .AddAttribute("SpecificIdleTime",
                          "Specific BWP inactivity timer (T_B) before fallback to Default.",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_specificIdleTime),
                          MakeTimeChecker())
            .AddAttribute("SwitchingDelay",
                          "Switching delay (T_S) used by window-based policy.",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_switchingDelay),
                          MakeTimeChecker())
            .AddAttribute("DefaultBwpId",
                          "Default BWP id used by window-based policy.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_defaultBwpId),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("SpecificBwpId",
                          "Specific BWP id used by window-based policy.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_specificBwpId),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("EvalGroupModulo",
                          "Partition UEs into modulo groups; only the matching group updates per evaluation to reduce interference (1 disables grouping).",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_evalGroupModulo),
                          MakeUintegerChecker<uint8_t>(1))
            .AddAttribute("StartDelay",
                          "Delay before the first evaluation is scheduled (to let RA/RRC complete).",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_startDelay),
                          MakeTimeChecker())
            .AddTraceSource("DecisionTrace",
                            "State/action snapshot emitted at every evaluation window.",
                            MakeTraceSourceAccessor(
                                &NrBwpSwitchTriggerHelper::m_decisionTrace),
                            "ns3::TracedCallback<uint16_t, const ns3::NrBwpSwitchState&, "
                            "const ns3::NrBwpSwitchDecision&>");
    return tid;
}

NrBwpSwitchTriggerHelper::NrBwpSwitchTriggerHelper()
{
    NS_LOG_FUNCTION(this);
    m_rng = CreateObject<UniformRandomVariable>();
}

NrBwpSwitchTriggerHelper::~NrBwpSwitchTriggerHelper()
{
    NS_LOG_FUNCTION(this);
}

std::vector<std::pair<double, double>>&
NrBwpSwitchTriggerHelper::GetQTableForUe(uint16_t rnti)
{
    auto it = m_qTablesByUe.find(rnti);
    if (it == m_qTablesByUe.end())
    {
        it = m_qTablesByUe
                 .emplace(rnti,
                          std::vector<std::pair<double, double>>(m_stateCount * m_actionCount,
                                                                 std::pair<double, double>{0.0, 0.0}))
                 .first;
    }
    return it->second;
}

std::vector<uint32_t>&
NrBwpSwitchTriggerHelper::GetVisitTableForUe(uint16_t rnti)
{
    auto it = m_visitTablesByUe.find(rnti);
    if (it == m_visitTablesByUe.end())
    {
        it = m_visitTablesByUe.emplace(rnti, std::vector<uint32_t>(m_stateCount * m_actionCount, 0)).first;
    }
    return it->second;
}

uint32_t&
NrBwpSwitchTriggerHelper::GetEvalCountForUe(uint16_t rnti)
{
    return m_evalCountsByUe[rnti];
}

void
NrBwpSwitchTriggerHelper::DoDispose()
{
    if (m_evalEvent.IsPending())
    {
        m_evalEvent.Cancel();
    }
    m_ueContext.clear();
    m_ueManagers.clear();
    m_qTablesByUe.clear();
    m_visitTablesByUe.clear();
    m_evalCountsByUe.clear();
    m_gnbManager = nullptr;
    Object::DoDispose();
}

void
NrBwpSwitchTriggerHelper::SetPolicy(const NrBwpSwitchController::PolicyCallback& cb)
{
    m_policy = cb;
    if (!m_evalEvent.IsPending())
    {
        ScheduleEvaluation();
    }
}

void
NrBwpSwitchTriggerHelper::SetDecisionCallback(
    const Callback<void, uint16_t, const NrBwpSwitchDecision&>& cb)
{
    NS_LOG_UNCOND("CALLBACK REGISTERED");
    m_decisionCb = cb;
}

void
NrBwpSwitchTriggerHelper::SetGnbManager(const Ptr<BwpManagerGnb>& gnbManager)
{
    m_gnbManager = gnbManager;
}

void
NrBwpSwitchTriggerHelper::AddUeManager(uint16_t rnti, const Ptr<BwpManagerUe>& ueManager)
{
    m_ueManagers[rnti] = ueManager;
}

void
NrBwpSwitchTriggerHelper::NotifyBsr(
    uint16_t rnti,
    uint8_t lcid,
    uint8_t currentBwp,
    uint8_t priority,
    const NrMacSapProvider::BufferStatusReportParameters& params)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<uint32_t>(lcid)
                         << static_cast<uint32_t>(currentBwp) << static_cast<uint32_t>(priority));
    NS_LOG_INFO("NotifyBsr rnti=" << rnti << " lcid=" << +lcid << " bwp=" << +currentBwp
                                  << " priority=" << +priority << " txQ=" << params.txQueueSize
                                  << " retxQ=" << params.retxQueueSize);

    auto& ctx = m_ueContext[rnti];
    ctx.lastBsr = params;
    if (ctx.currentBwp != currentBwp)
    {
        ctx.lastSwitchTimeSeconds = Simulator::Now().GetSeconds();
    }
    ctx.currentBwp = currentBwp;
    ctx.priority = priority;

    ctx.queueSumBytes += params.txQueueSize;
    ctx.queueSamples++;

    ctx.energySumJ += params.txQueueSize; // * m_energyPerByteJ;

    if (!m_evalEvent.IsPending())
    {
        ScheduleEvaluation();
    }
}

void
NrBwpSwitchTriggerHelper::NotifyDlQueue(uint16_t rnti,
                                        uint8_t lcid,
                                        uint8_t currentBwp,
                                        uint8_t priority,
                                        uint32_t queueBytes)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<uint32_t>(lcid) << static_cast<uint32_t>(currentBwp)
                         << static_cast<uint32_t>(priority) << queueBytes);
    NS_LOG_INFO("NotifyDlQueue rnti=" << rnti << " lcid=" << +lcid << " bwp=" << +currentBwp
                                      << " priority=" << +priority << " queue=" << queueBytes);

    auto& ctx = m_ueContext[rnti];
    ctx.lastBsr.rnti = rnti;
    ctx.lastBsr.lcid = lcid;
    ctx.lastBsr.txQueueSize = queueBytes;
    ctx.lastBsr.retxQueueSize = 0;
    if (ctx.currentBwp != currentBwp)
    {
        ctx.lastSwitchTimeSeconds = Simulator::Now().GetSeconds();
    }
    ctx.currentBwp = currentBwp;
    ctx.priority = priority;

    ctx.queueSumBytes += queueBytes;
    ctx.queueSamples++;

    if (!m_evalEvent.IsPending())
    {
        ScheduleEvaluation();
    }
}

NrBwpSwitchTriggerHelper::PerUeContext&
NrBwpSwitchTriggerHelper::GetOrCreateCtx(uint16_t rnti)
{
    auto& ctx = m_ueContext[rnti];
    ctx.lastBsr.rnti = rnti; // seed in case only enqueue/ack arrive.
    if (m_gnbManager != nullptr) // && ctx.currentBwp == 0)
    {
        // Try to pick the last forced BWP as a best-effort default.
        ctx.currentBwp = m_gnbManager->GetForcedUeBwp(rnti);
    }
    double now = Simulator::Now().GetSeconds();
    if (ctx.lastSwitchTimeSeconds == 0.0)
    {
        ctx.lastSwitchTimeSeconds = now;
    }
    if (ctx.lastEvalTimeSeconds == 0.0)
    {
        ctx.lastEvalTimeSeconds = now;
    }
    return ctx;
}

void
NrBwpSwitchTriggerHelper::RecordEnqueue(uint16_t rnti, uint8_t lcid)
{
    uint64_t key = (static_cast<uint64_t>(rnti) << 8) | lcid;
    auto& ctx = GetOrCreateCtx(rnti);
    NS_LOG_INFO("RecordEnqueue rnti=" << rnti << " lcid=" << +lcid << " t=" << Simulator::Now().GetSeconds());
    // NS_LOG_UNCOND("RecordEnqueue rnti=" << rnti << " lcid=" << +lcid << " t=" << Simulator::Now().GetSeconds());
    ctx.pending[key] = Simulator::Now();
}

void
NrBwpSwitchTriggerHelper::RecordAck(uint16_t rnti, uint8_t lcid)
{
    uint64_t key = (static_cast<uint64_t>(rnti) << 8) | lcid;
    auto& ctx = GetOrCreateCtx(rnti);
    auto pendingIt = ctx.pending.find(key);
    if (pendingIt == ctx.pending.end())
    {
        NS_LOG_INFO("RecordAck rnti=" << rnti << " lcid=" << +lcid << " no_pending");
        return;
    }
    Time aoi = Simulator::Now() - pendingIt->second;
    ctx.aoiSumMs += aoi.GetMilliSeconds();
    ctx.aoiSamples++;
    ctx.pending.erase(pendingIt);
    NS_LOG_INFO("RecordAck rnti=" << rnti << " lcid=" << +lcid << " aoi=" << aoi.GetMilliSeconds()
                                  << "ms samples=" << ctx.aoiSamples);
    NS_LOG_UNCOND("RecordAck rnti=" << rnti << " lcid=" << +lcid << " aoi=" << aoi.GetMilliSeconds()
                                << "ms samples=" << ctx.aoiSamples);
}
// 100 ms = 0.1 s
void
NrBwpSwitchTriggerHelper::ScheduleEvaluation()
{
    if (m_evalEvent.IsPending())
    {
        return;
    }
    Time delay = m_evalPeriod;
    if (Simulator::Now() < m_startDelay)
    {
        delay = m_startDelay - Simulator::Now();
    }
    m_evalEvent = Simulator::Schedule(delay, &NrBwpSwitchTriggerHelper::EvaluateWindow, this);
}

uint8_t
NrBwpSwitchTriggerHelper::QuantizeQueue(double avgQueue) const
{
    if (avgQueue <= static_cast<double>(m_queueThreshold1))
    {
        return 0;
    }
    if (avgQueue <= static_cast<double>(m_queueThreshold2))
    {
        return 1;
    }
    return 2;
}

uint8_t
NrBwpSwitchTriggerHelper::QuantizeAoI(double avgAoI) const
{
    Time avg = MilliSeconds(avgAoI);
    if (avg <= m_aoiThreshold1)
    {
        return 0;
    }
    if (avg <= m_aoiThreshold2)
    {
        return 1;
    }
    return 2;
}

uint8_t
NrBwpSwitchTriggerHelper::QuantizeDwell(double dwellSeconds) const
{
    // 10 ms bins up to 100 ms (bin 10).
    uint8_t bin = static_cast<uint8_t>(dwellSeconds / 0.01);
    return std::min<uint8_t>(bin, static_cast<uint8_t>(m_dwellBins - 1));
}

uint8_t
NrBwpSwitchTriggerHelper::DecodeActionBwp(uint8_t actionIdx) const
{
    return actionIdx / 3; // 0 or 1 (since actionCount = 6)
}

uint8_t
NrBwpSwitchTriggerHelper::DecodeActionPriority(uint8_t actionIdx) const
{
    return actionIdx % 3;
}

uint32_t
NrBwpSwitchTriggerHelper::ComputeStateIndex(uint8_t queueBin,
                                            uint8_t aoiBin,
                                            uint8_t dwellBin,
                                            uint8_t bwp,
                                            uint8_t pri) const
{
    queueBin = std::min<uint8_t>(queueBin, 2);
    aoiBin = std::min<uint8_t>(aoiBin, 2);
    dwellBin = std::min<uint8_t>(dwellBin, static_cast<uint8_t>(m_dwellBins - 1));
    bwp = std::min<uint8_t>(bwp, 1);
    pri = std::min<uint8_t>(pri, 2);

    // ((((queue *3 + aoi)*dwellBins + dwell)*2 + bwp)*3 + pri)
    uint32_t idx = queueBin;
    idx = idx * 3 + aoiBin;
    idx = idx * m_dwellBins + dwellBin;
    idx = idx * 2 + bwp;
    idx = idx * 3 + pri;
    return idx;
}

NrBwpSwitchDecision
NrBwpSwitchTriggerHelper::RunInternalPolicy(uint16_t rnti,
                                            const NrBwpSwitchState& state,
                                            uint32_t stateIdx,
                                            PerUeContext& ctx)
{
    constexpr uint32_t kLogEvery = 30; // avoid log flood
    auto& qTable = GetQTableForUe(rnti);
    auto& visitTable = GetVisitTableForUe(rnti);
    uint32_t evalCount = ++GetEvalCountForUe(rnti);
    double epsilon =
        std::max(m_epsilonMin, m_epsilon * std::exp(-m_epsilonDecay * static_cast<double>(evalCount)));
    double energyNorm = std::max(m_rewardEnergyNorm, 1e-9);
    double aoiNorm = std::max(m_rewardAoiNorm, 1e-9);
    static uint32_t count = 0;
    // Update Q for previous window (transition prev -> current).
    if (ctx.hasPrev)
    {
        uint32_t idx = ctx.prevStateIdx * m_actionCount + ctx.prevActionIdx;
        uint32_t visits = ++visitTable[idx];
        double alpha =
            std::max(m_alphaMin, m_alpha / std::sqrt(static_cast<double>(std::max<uint32_t>(1, visits))));
        double rewardEnergy = -(ctx.energySumJ + ctx.pendingSwitchEnergy) / energyNorm;
        double rewardAoI = -(state.avgAoISeconds) / aoiNorm;

        // Greedy bootstrap from next state using preference scalarization.
        double bestScalar = -1e30;
        std::pair<double, double> bestQ{0.0, 0.0};
        for (uint8_t a = 0; a < m_actionCount; ++a)
        {
            auto q = qTable[stateIdx * m_actionCount + a];
            double scalar = m_prefEnergy * q.first + m_prefAoI * q.second;
            if (scalar > bestScalar)
            {
                bestScalar = scalar;
                bestQ = q;
            }
        }

        auto& qPrev = qTable[idx];
        qPrev.first += alpha * (rewardEnergy + m_gamma * bestQ.first - qPrev.first);
        qPrev.second += alpha * (rewardAoI + m_gamma * bestQ.second - qPrev.second);
        ctx.pendingSwitchEnergy = 0.0;
        
    }

    // Epsilon-greedy selection on scalarized Q values.
    bool explore = m_rng->GetValue() < epsilon;
    uint8_t chosen = 0;
    
    if (explore)
    {
        // NS_LOG_UNCOND("EXPLORE");
        chosen = static_cast<uint8_t>(m_rng->GetInteger(0, m_actionCount - 1));
    }
    else
    {
        // NS_LOG_UNCOND("EXPLOIT");
        double bestScalar = -1e30;
        for (uint8_t a = 0; a < m_actionCount; ++a)
        {
            auto q = qTable[stateIdx * m_actionCount + a];
            double scalar = m_prefEnergy * q.first + m_prefAoI * q.second;
            if (scalar > bestScalar)
            {
                bestScalar = scalar;
                chosen = a;
            }
        }
    }

    NrBwpSwitchDecision decision;
    decision.targetBwpId = DecodeActionBwp(chosen);
    decision.targetPriority = DecodeActionPriority(chosen);
    decision.bsr = state.bsr;

    if ((evalCount % kLogEvery) == 0)
    {
        double scalarPrev = m_prefEnergy * qTable[stateIdx * m_actionCount + chosen].first +
                            m_prefAoI * qTable[stateIdx * m_actionCount + chosen].second;
        NS_LOG_UNCOND("QDBG rnti=" << rnti << " eval=" << evalCount << " stateIdx=" << stateIdx
                                   << " queueBin=" << +state.queueBin << " aoiBin=" << +state.aoiBin
                                   << " dwellBin=" << +state.dwellBin << " bwp=" << +state.currentBwpId
                                   << " pri=" << +state.priority << " action=" << +chosen
                                   << " tgtBwp=" << +decision.targetBwpId
                                   << " tgtPri=" << +decision.targetPriority
                                   << " eps=" << epsilon
                                   << " alpha=" << ((ctx.hasPrev) ? std::max(m_alphaMin, m_alpha / std::sqrt(static_cast<double>(std::max<uint32_t>(1, visitTable[ctx.prevStateIdx * m_actionCount + ctx.prevActionIdx])))) : m_alpha)
                                   << " rE=" << ((ctx.hasPrev) ? (-(ctx.energySumJ + ctx.pendingSwitchEnergy) / energyNorm) : 0.0)
                                   << " rA=" << ((ctx.hasPrev) ? (-(state.avgAoISeconds) / aoiNorm) : 0.0)
                                   << " scalarQ=" << scalarPrev
                                   << " bestScalar=" << (ctx.hasPrev ? m_prefEnergy * qTable[stateIdx * m_actionCount + chosen].first + m_prefAoI * qTable[stateIdx * m_actionCount + chosen].second : 0.0));
    }


    if((++count) % 50 == 0)
    {
        NS_LOG_UNCOND(Simulator::Now().As(Time::S) << " RunInternalPolicy rnti=" << rnti << " stateIdx=" << stateIdx << " bwp=" << +state.currentBwpId
                                          << " pri=" << +state.priority << " queueBin=" << +state.queueBin
                                          << " aoiBin=" << +state.aoiBin << " dwellBin=" << +state.dwellBin
                                          << " action=" << +chosen << " tgtBwp=" << +decision.targetBwpId
                                          << " tgtPri=" << +decision.targetPriority);
        count = 0;
    }

    ctx.prevStateIdx = stateIdx;
    ctx.prevActionIdx = chosen;
    ctx.hasPrev = true;

    if (decision.targetBwpId != std::numeric_limits<uint8_t>::max() &&
        decision.targetBwpId != state.currentBwpId && m_gnbManager != nullptr)
    {
        // NS_LOG_UNCOND("SWITCH TO " << (int) decision.targetBwpId);
        ctx.pendingSwitchEnergy =
            m_gnbManager->GetEnergyConfig().GetSwitchEnergy(state.currentBwpId, decision.targetBwpId);
    }
    else
    {
        ctx.pendingSwitchEnergy = 0.0;
    }

    return decision;
}

NrBwpSwitchDecision
NrBwpSwitchTriggerHelper::RunWindowDetectPolicy(uint16_t rnti,
                                                const NrBwpSwitchState& state,
                                                PerUeContext& ctx,
                                                Time dt,
                                                bool isSwitching)
{
    (void)rnti;
    NrBwpSwitchDecision decision;
    decision.bsr = state.bsr;

    uint8_t defaultBwp = m_defaultBwpId;
    uint8_t specificBwp = m_specificBwpId;
    uint8_t observedBwp = state.currentBwpId;
    if (observedBwp == std::numeric_limits<uint8_t>::max())
    {
        observedBwp = defaultBwp;
    }

    if (!ctx.policyInitialized)
    {
        ctx.policyCurrentBwp = observedBwp;
        ctx.switchState = (observedBwp == specificBwp) ? PerUeContext::SPECIFIC_ACTIVE
                                                       : PerUeContext::DEFAULT_ACTIVE;
        ctx.tWindow = Seconds(0);
        ctx.tOnDefaultActive = Seconds(0);
        ctx.tSpecificIdle = Seconds(0);
        ctx.tSwitch = Seconds(0);
        ctx.policyInitialized = true;
    }

    if (!isSwitching && observedBwp != ctx.policyCurrentBwp)
    {
        ctx.policyCurrentBwp = observedBwp;
        ctx.switchState = (observedBwp == specificBwp) ? PerUeContext::SPECIFIC_ACTIVE
                                                       : PerUeContext::DEFAULT_ACTIVE;
        ctx.tOnDefaultActive = Seconds(0);
        ctx.tSpecificIdle = Seconds(0);
        ctx.tSwitch = Seconds(0);
    }

    bool newWindow = false;
    if (!m_controlWindow.IsZero() && dt > Seconds(0))
    {
        ctx.tWindow += dt;
        while (ctx.tWindow >= m_controlWindow)
        {
            ctx.tWindow -= m_controlWindow;
            newWindow = true;
        }
    }

    if (isSwitching || ctx.switchState == PerUeContext::SWITCHING)
    {
        if (!m_switchingDelay.IsZero() && dt > Seconds(0) &&
            ctx.switchState == PerUeContext::SWITCHING)
        {
            ctx.tSwitch += dt;
            if (ctx.tSwitch > m_switchingDelay)
            {
                ctx.tSwitch = m_switchingDelay;
            }
        }

        if (!isSwitching)
        {
            if (m_switchingDelay.IsZero() || ctx.tSwitch >= m_switchingDelay)
            {
                ctx.tSwitch = Seconds(0);
                ctx.switchState = (ctx.policyCurrentBwp == specificBwp)
                                      ? PerUeContext::SPECIFIC_ACTIVE
                                      : PerUeContext::DEFAULT_ACTIVE;
            }
        }
        return decision;
    }

    uint32_t backlogBytes = state.bsr.txQueueSize + state.bsr.retxQueueSize;

    auto requestSwitch = [&](uint8_t targetBwp) -> bool {
        if (targetBwp == ctx.policyCurrentBwp)
        {
            return false;
        }
        ctx.policyCurrentBwp = targetBwp;
        ctx.switchState = PerUeContext::SWITCHING;
        ctx.tSwitch = Seconds(0);
        ctx.tOnDefaultActive = Seconds(0);
        ctx.tSpecificIdle = Seconds(0);
        ctx.lastSwitchTimeSeconds = Simulator::Now().GetSeconds();
        decision.targetBwpId = targetBwp;
        return true;
    };

    if (newWindow)
    {
        if (backlogBytes > m_switchThresholdBytes)
        {
            if (requestSwitch(specificBwp))
            {
                return decision;
            }
        }
        else
        {
            if (requestSwitch(defaultBwp))
            {
                return decision;
            }
        }
    }

    if (ctx.switchState == PerUeContext::DEFAULT_ACTIVE)
    {
        if (backlogBytes > 0)
        {
            if (dt > Seconds(0))
            {
                ctx.tOnDefaultActive += dt;
            }
            if (!m_detectTime.IsZero() && ctx.tOnDefaultActive >= m_detectTime)
            {
                requestSwitch(specificBwp);
            }
        }
        else
        {
            ctx.tOnDefaultActive = Seconds(0);
        }
    }

    if (ctx.switchState == PerUeContext::SPECIFIC_ACTIVE)
    {
        if (backlogBytes == 0)
        {
            if (dt > Seconds(0))
            {
                ctx.tSpecificIdle += dt;
            }
            if (!m_specificIdleTime.IsZero() && ctx.tSpecificIdle >= m_specificIdleTime)
            {
                requestSwitch(defaultBwp);
            }
        }
        else
        {
            ctx.tSpecificIdle = Seconds(0);
        }
    }

    return decision;
}

void
NrBwpSwitchTriggerHelper::EvaluateWindow()
{
    NS_LOG_FUNCTION(this);

    for (auto& kv : m_ueContext)
    {
        uint16_t rnti = kv.first;
        auto& ctx = kv.second;

        uint8_t groupMod = std::max<uint8_t>(1, m_evalGroupModulo);
        if (groupMod > 1)
        {
            uint8_t group = static_cast<uint8_t>(rnti % groupMod);
            uint8_t active = static_cast<uint8_t>(m_evalRound % groupMod);
            if (group != active)
            {
                continue;
            }
        }

        ctx.currentBwp = m_gnbManager->GetForcedUeBwp(rnti);

        double now = Simulator::Now().GetSeconds();
        double dtSeconds = 0.0;
        if (ctx.lastEvalTimeSeconds > 0.0)
        {
            dtSeconds = now - ctx.lastEvalTimeSeconds;
            double mw = 0.0;
            if (ctx.currentBwp == 0)
            {
                mw = m_staticPowerBwp0Mw;
            }
            else if (ctx.currentBwp == 1)
            {
                mw = m_staticPowerBwp1Mw;
            }
            ctx.energySumJ += dtSeconds * mw * 1e-3;
        }
        ctx.lastEvalTimeSeconds = now;
        Time dt = Seconds(std::max(0.0, dtSeconds));

        double avgQueue = (ctx.queueSamples > 0) ? ctx.queueSumBytes / static_cast<double>(ctx.queueSamples) : 0.0;
        double avgAoI = (ctx.aoiSamples > 0) ? ctx.aoiSumMs / static_cast<double>(ctx.aoiSamples) : 0.0;
        // NS_LOG_INFO("Eval rnti=" << rnti << " avgQueue=" << avgQueue << "B avgAoI=" << avgAoI
                                //  << "s samplesAoi=" << ctx.aoiSamples << " samplesQ=" << ctx.queueSamples);
        NS_LOG_UNCOND("Eval rnti=" << rnti << " avgQueue=" << avgQueue << "B avgAoI=" << avgAoI
                                 << "ms samplesAoi=" << ctx.aoiSamples << " samplesQ=" << ctx.queueSamples);


        uint8_t queueBin = QuantizeQueue(avgQueue);
        uint8_t aoiBin = QuantizeAoI(avgAoI);
        double dwellSeconds =
            (ctx.lastSwitchTimeSeconds > 0.0) ? (now - ctx.lastSwitchTimeSeconds) : 0.0;
        uint8_t dwellBin = QuantizeDwell(dwellSeconds);

        NrBwpSwitchState state;
        state.currentBwpId = ctx.currentBwp;
        state.bsr = ctx.lastBsr;
        state.switchingRemaining =
            (m_gnbManager != nullptr) ? m_gnbManager->GetSwitchingRemaining(rnti) : Seconds(0);
        state.avgQueueSizeBytes = avgQueue;
        state.avgAoISeconds = avgAoI;
        state.priority = ctx.priority;
        state.queueBin = queueBin;
        state.aoiBin = aoiBin;
        state.dwellSeconds = dwellSeconds;
        state.dwellBin = dwellBin;

        uint32_t stateIdx = ComputeStateIndex(queueBin, aoiBin, dwellBin, state.currentBwpId, state.priority);

        NrBwpSwitchDecision decision;
        bool isSwitching = !state.switchingRemaining.IsZero();

        if (!m_policy.IsNull())
        {
            decision = m_policy(state);
        }
        else if (m_enableInternalPolicy)
        {
            if (m_internalPolicyMode == POLICY_WINDOW_DETECT)
            {
                decision = RunWindowDetectPolicy(rnti, state, ctx, dt, isSwitching);
            }
            else
            {
                decision = RunInternalPolicy(rnti, state, stateIdx, ctx);
            }
        }

        // Store potential switch energy to charge in the next reward window.
        if (decision.targetBwpId != std::numeric_limits<uint8_t>::max() &&
            decision.targetBwpId != state.currentBwpId && m_gnbManager != nullptr)
        {
            ctx.pendingSwitchEnergy =
                m_gnbManager->GetEnergyConfig().GetSwitchEnergy(state.currentBwpId, decision.targetBwpId);
        }

        m_decisionTrace(rnti, state, decision);

        if (!m_decisionCb.IsNull())
        {
            m_decisionCb(rnti, decision);
        }

        if (decision.targetPriority != std::numeric_limits<uint8_t>::max())
        {
            ctx.priority = decision.targetPriority;
            if (m_gnbManager != nullptr)
            {
                m_gnbManager->SetUePriority(rnti, decision.targetPriority);
            }
        }

        ctx.ResetPeriod();
    }

    m_evalRound++;
    ScheduleEvaluation();
}

} // namespace ns3
