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

#include <algorithm>
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
            .AddAttribute("EventDrivenEvaluation",
                          "Use event-driven evaluation for Q-learning; EvaluationPeriod becomes the minimum gap between evaluations.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrBwpSwitchTriggerHelper::m_eventDrivenEvaluation),
                          MakeBooleanChecker())
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
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_aoiThreshold1),
                          MakeTimeChecker())
            .AddAttribute("AoIThreshold2",
                          "Upper AoI threshold for discretization.",
                          TimeValue(MilliSeconds(5)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_aoiThreshold2),
                          MakeTimeChecker())
            .AddAttribute("EnergyPerByteJ",
                          "Energy per byte transmitted on a BWP (approximate).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_energyPerByteJ),
                          MakeDoubleChecker<double>())
            .AddAttribute("PreferenceEnergy",
                          "Preference weight for energy in Pareto Q-learning scalarization.",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_prefEnergy),
                          MakeDoubleChecker<double>())
            .AddAttribute("PreferenceAoI",
                          "Preference weight for AoI in Pareto Q-learning scalarization.",
                          DoubleValue(0.7),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_prefAoI),
                          MakeDoubleChecker<double>())
            .AddAttribute("UseAdaptivePreference",
                          "Adapt preference weights based on queue/AoI stress.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrBwpSwitchTriggerHelper::m_useAdaptivePreference),
                          MakeBooleanChecker())
            .AddAttribute("PreferenceAoIMin",
                          "Minimum AoI preference weight when stress is low.",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_prefAoiMin),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("PreferenceAoIMax",
                          "Maximum AoI preference weight when stress is high.",
                          DoubleValue(0.8),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_prefAoiMax),
                          MakeDoubleChecker<double>(0.0, 1.0))
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
                          DoubleValue(0.2),
                        //   DoubleValue(1.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_epsilon),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EpsilonMin",
                          "Minimum epsilon after decay.",
                          DoubleValue(0.02),
                            // DoubleValue(1.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_epsilonMin),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EpsilonDecay",
                          "Decay rate for epsilon; applied as exp(-decay * evalCount).",
                          DoubleValue(5e-3),
                        //   DoubleValue(0.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_epsilonDecay),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("RewardEnergyNorm",
                          "Normalization constant (J) applied to energy reward.",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_rewardEnergyNorm),
                          MakeDoubleChecker<double>(1e-9))
            .AddAttribute("RewardAoiNorm",
                          "Normalization constant (ms) applied to AoI reward.",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_rewardAoiNorm),
                          MakeDoubleChecker<double>(1e-9))
            .AddAttribute("SwitchPenaltyJ",
                          "Additional energy-equivalent penalty (J) applied when a switch is requested.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrBwpSwitchTriggerHelper::m_switchPenaltyJ),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("EnableInternalPolicy",
                          "Enable built-in Pareto Q-learning policy when no external policy is set.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrBwpSwitchTriggerHelper::m_enableInternalPolicy),
                          MakeBooleanChecker())
            .AddAttribute("InternalPolicyMode",
                          "Select internal policy when no external policy is set.",
                          EnumValue(POLICY_Q_LEARNING),
                          MakeEnumAccessor<NrBwpSwitchTriggerHelper::InternalPolicyMode>(&NrBwpSwitchTriggerHelper::m_internalPolicyMode),
                          MakeEnumChecker(POLICY_Q_LEARNING,
                                          "Qlearning",
                                          POLICY_WINDOW_DETECT,
                                          "WindowDetect"))
            .AddAttribute("SwitchThresholdBytes",
                          "Queue threshold (tau) for window-based switching (bytes).",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&NrBwpSwitchTriggerHelper::m_switchThresholdBytes),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("ControlWindow",
                          "Periodic control window (T_C). Zero disables periodic decision.",
                          TimeValue(MilliSeconds(50)),
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
            .AddAttribute("MinDwellTime",
                          "Minimum time to stay on a BWP before allowing another switch.",
                          TimeValue(Seconds(50)),
                          MakeTimeAccessor(&NrBwpSwitchTriggerHelper::m_minDwellTime),
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
    for (auto& kv : m_ueContext)
    {
        if (kv.second.evalEvent.IsPending())
        {
            kv.second.evalEvent.Cancel();
        }
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
    bool eventOnlyQlearning =
        m_eventDrivenEvaluation && m_enableInternalPolicy &&
        m_internalPolicyMode == POLICY_Q_LEARNING && m_policy.IsNull();
    if (!eventOnlyQlearning && !m_evalEvent.IsPending())
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
    if (m_gnbManager != nullptr && !m_txEnergyConnected)
    {
        m_gnbManager->TraceConnectWithoutContext(
            "TxEnergy",
            MakeCallback(&NrBwpSwitchTriggerHelper::HandleTxEnergy, this));
        m_txEnergyConnected = true;
    }
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

    ctx.energySumJ += static_cast<double>(params.txQueueSize) * m_energyPerByteJ;

    uint8_t queueBin = QuantizeQueue(static_cast<double>(params.txQueueSize));
    bool queueTrigger = false;
    if (!ctx.hasLastQueueBin)
    {
        ctx.lastQueueBin = queueBin;
        ctx.hasLastQueueBin = true;
    }
    else if (queueBin != ctx.lastQueueBin)
    {
        ctx.lastQueueBin = queueBin;
        queueTrigger = true;
    }
    MaybeTriggerEventEvaluation(rnti, queueTrigger, false);

    bool eventOnlyQlearning =
        m_eventDrivenEvaluation && m_enableInternalPolicy &&
        m_internalPolicyMode == POLICY_Q_LEARNING && m_policy.IsNull();
    if (!eventOnlyQlearning && !m_evalEvent.IsPending())
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

    uint8_t queueBin = QuantizeQueue(static_cast<double>(queueBytes));
    bool queueTrigger = false;
    if (!ctx.hasLastQueueBin)
    {
        ctx.lastQueueBin = queueBin;
        ctx.hasLastQueueBin = true;
    }
    else if (queueBin != ctx.lastQueueBin)
    {
        ctx.lastQueueBin = queueBin;
        queueTrigger = true;
    }
    MaybeTriggerEventEvaluation(rnti, queueTrigger, false);

    bool eventOnlyQlearning =
        m_eventDrivenEvaluation && m_enableInternalPolicy &&
        m_internalPolicyMode == POLICY_Q_LEARNING && m_policy.IsNull();
    if (!eventOnlyQlearning && !m_evalEvent.IsPending())
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
    double nowSeconds = Simulator::Now().GetSeconds();
    bool iatTrigger = false;
    if (ctx.lastEnqueueTimeSeconds >= 0.0)
    {
        double iatMs = (nowSeconds - ctx.lastEnqueueTimeSeconds) * 1000.0;
        if (iatMs >= 0.0)
        {
            uint8_t iatBin = QuantizeIatEvent(iatMs);
            if (!ctx.hasLastIatBin)
            {
                ctx.lastIatBin = iatBin;
                ctx.hasLastIatBin = true;
            }
            else if (iatBin != ctx.lastIatBin)
            {
                ctx.lastIatBin = iatBin;
                iatTrigger = true;
            }
        }
    }
    ctx.lastEnqueueTimeSeconds = nowSeconds;
    MaybeTriggerEventEvaluation(rnti, false, iatTrigger);
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
    Time issueTime = pendingIt->second;
    Time aoi = Simulator::Now() - issueTime;
    ctx.aoiSumMs += aoi.GetMilliSeconds();
    ctx.aoiSamples++;
    ctx.pending.erase(pendingIt);
    NS_LOG_UNCOND("ISSUE TIME: " << issueTime.As(Time::MS)
                                << ", ARRIVAL TIME: " << Simulator::Now().As(Time::MS)
                                << ", AoI: " << aoi.As(Time::MS));
    NS_LOG_INFO("RecordAck rnti=" << rnti << " lcid=" << +lcid << " aoi=" << aoi.GetMilliSeconds()
                                  << "ms samples=" << ctx.aoiSamples);
}

void
NrBwpSwitchTriggerHelper::RecordAoiSample(uint16_t rnti, uint8_t lcid, Time delay)
{
    (void)lcid;
    auto& ctx = GetOrCreateCtx(rnti);
    ctx.aoiSumMs += delay.GetMilliSeconds();
    ctx.aoiSamples++;
}

void
NrBwpSwitchTriggerHelper::HandleTxEnergy(uint16_t rnti,
                                         uint8_t bwpId,
                                         uint32_t bytes,
                                         double energyJ)
{
    (void)bwpId;
    (void)bytes;
    auto& ctx = GetOrCreateCtx(rnti);
    ctx.energySumJ += energyJ;
}

void
NrBwpSwitchTriggerHelper::UpdatePhaseFromQueueObservation(uint16_t rnti, uint32_t queueBytes)
{
    auto& ctx = m_ueContext[rnti];
    double nowSeconds = Simulator::Now().GetSeconds();

    if (!ctx.hasLastQueueObs)
    {
        ctx.lastQueueObsTimeSeconds = nowSeconds;
        ctx.lastQueueObsBytes = queueBytes;
        ctx.hasLastQueueObs = true;
        if (ctx.phase == PerUeContext::PHASE_UNKNOWN)
        {
            ctx.phase = (queueBytes > 0) ? PerUeContext::PHASE_BURST : PerUeContext::PHASE_QUIET;
        }
        return;
    }

    double dtMs = (nowSeconds - ctx.lastQueueObsTimeSeconds) * 1000.0;
    if (dtMs > 0.0)
    {
        double slopeBytesPerMs =
            (static_cast<double>(queueBytes) - static_cast<double>(ctx.lastQueueObsBytes)) / dtMs;
        constexpr double kSlopeAlpha = 0.2;
        ctx.qSlopeEwmaBytesPerMs =
            (1.0 - kSlopeAlpha) * ctx.qSlopeEwmaBytesPerMs + kSlopeAlpha * slopeBytesPerMs;
    }
    ctx.lastQueueObsTimeSeconds = nowSeconds;
    ctx.lastQueueObsBytes = queueBytes;

    double evalMs = std::max(1.0, static_cast<double>(m_evalPeriod.GetMilliSeconds()));
    double queueOnsetBytes = std::max(1.0, 0.5 * static_cast<double>(m_queueThreshold1));
    double growThresh = queueOnsetBytes / evalMs;
    double drainThresh = -0.25 * growThresh;
    double shortIatMs = std::max(1.0, 0.25 * evalMs);
    double longIatMs = std::max(1.0, evalMs);
    bool hasIat = ctx.iatEwmaMs >= 0.0;

    bool burstSignal =
        (queueBytes >= static_cast<uint32_t>(queueOnsetBytes)) ||
        (hasIat && ctx.iatEwmaMs <= shortIatMs && ctx.qSlopeEwmaBytesPerMs > growThresh);

    bool quietSignal =
        (queueBytes == 0 ||
         (queueBytes < static_cast<uint32_t>(0.25 * static_cast<double>(m_queueThreshold1)) &&
          ctx.qSlopeEwmaBytesPerMs <= 0.0)) &&
        (!hasIat || ctx.iatEwmaMs >= longIatMs || ctx.qSlopeEwmaBytesPerMs < drainThresh);

    if (burstSignal)
    {
        ctx.burstVotes = std::min<uint8_t>(3, static_cast<uint8_t>(ctx.burstVotes + 1));
        ctx.quietVotes = 0;
    }
    else if (quietSignal)
    {
        ctx.quietVotes = std::min<uint8_t>(3, static_cast<uint8_t>(ctx.quietVotes + 1));
        ctx.burstVotes = 0;
    }
    else
    {
        ctx.burstVotes = 0;
        ctx.quietVotes = 0;
    }

    auto prevPhase = ctx.phase;
    if (ctx.burstVotes >= 2)
    {
        ctx.phase = PerUeContext::PHASE_BURST;
    }
    else if (ctx.quietVotes >= 2)
    {
        ctx.phase = PerUeContext::PHASE_QUIET;
    }

    if (ctx.phase != prevPhase)
    {
        NS_LOG_INFO("PhaseChange rnti=" << rnti << " phase=" << static_cast<uint32_t>(ctx.phase)
                                        << " queue=" << queueBytes << " iatEwmaMs=" << ctx.iatEwmaMs
                                        << " qSlopeEwma=" << ctx.qSlopeEwmaBytesPerMs);
    }
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

void
NrBwpSwitchTriggerHelper::MaybeTriggerEventEvaluation(uint16_t rnti, bool queueTrigger, bool iatTrigger)
{
    if (!(m_eventDrivenEvaluation && m_enableInternalPolicy &&
          m_internalPolicyMode == POLICY_Q_LEARNING && m_policy.IsNull()))
    {
        return;
    }
    if (!queueTrigger && !iatTrigger)
    {
        return;
    }

    auto& ctx = GetOrCreateCtx(rnti);

    Time now = Simulator::Now();
    if (now < m_startDelay)
    {
        return;
    }
    double nowSeconds = now.GetSeconds();
    if (ctx.evalEvent.IsPending())
    {
        return;
    }
    ctx.lastEventEvalTimeSeconds = nowSeconds;
    ctx.evalEvent = Simulator::ScheduleNow(&NrBwpSwitchTriggerHelper::EvaluateUe, this, rnti, false);
}

uint8_t
NrBwpSwitchTriggerHelper::QuantizeIatEvent(double iatMs) const
{
    // Raw fixed thresholds (no smoothing) for event triggering only.
    if (iatMs <= 1.0)
    {
        return 0; // intra-burst / tightly packed arrivals
    }
    if (iatMs <= 10.0)
    {
        return 1; // moderate gap
    }
    return 2; // long gap / inter-burst
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
NrBwpSwitchTriggerHelper::QuantizeAoI(double avgAoIMs) const
{
    Time avg = MilliSeconds(avgAoIMs);
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
    // Coarse bins to keep state space small: <0.5s, <2.0s, >=2.0s.
    if (dwellSeconds < 0.5)
    {
        return 0;
    }
    if (dwellSeconds < 2.0)
    {
        return 1;
    }
    return 2;
}

uint8_t
NrBwpSwitchTriggerHelper::DecodeActionBwp(uint8_t actionIdx) const
{
    return actionIdx / 2; // 0 or 1 (since actionCount = 4)
}

uint8_t
NrBwpSwitchTriggerHelper::DecodeActionPriority(uint8_t actionIdx) const
{
    return actionIdx % 2;
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
    pri = std::min<uint8_t>(pri, 1);

    // ((((queue *3 + aoi) * dwellBins + dwell) *2 + bwp) *2 + pri)
    uint32_t idx = queueBin;
    idx = idx * 3 + aoiBin;
    idx = idx * m_dwellBins + dwellBin;
    idx = idx * 2 + bwp;
    idx = idx * 2 + pri;
    return idx;
}

NrBwpSwitchDecision
NrBwpSwitchTriggerHelper::RunInternalPolicy(uint16_t rnti,
                                            const NrBwpSwitchState& state,
                                            uint32_t stateIdx,
                                            PerUeContext& ctx,
                                            bool isSwitching)
{
    constexpr uint32_t kLogEvery = 10; // avoid log flood
    auto& qTable = GetQTableForUe(rnti);
    auto& visitTable = GetVisitTableForUe(rnti);
    // Decay epsilon only when we have meaningful observations in this window.
    bool hasObservation = (ctx.queueSamples > 0) || (ctx.aoiSamples > 0);
    uint32_t evalCount = GetEvalCountForUe(rnti);
    if (hasObservation)
    {
        evalCount = ++GetEvalCountForUe(rnti);
    }
    double epsilon =
        std::max(m_epsilonMin, m_epsilon * std::exp(-m_epsilonDecay * static_cast<double>(evalCount)));
    double energyNorm = std::max(m_rewardEnergyNorm, 1e-9);
    double aoiNorm = std::max(m_rewardAoiNorm, 1e-9);

    // Adaptive preference based on current stress (queue/AoI vs thresholds).
    double prefEnergy = m_prefEnergy;
    double prefAoI = m_prefAoI;
    if (m_useAdaptivePreference)
    {
        double qDen = std::max(1.0, static_cast<double>(m_queueThreshold2));
        double aDen = std::max(1.0, static_cast<double>(m_aoiThreshold2.GetMilliSeconds()));
        double qNorm = std::min(1.0, state.avgQueueSizeBytes / qDen);
        double aNorm = std::min(1.0, state.avgAoIMs / aDen);
        double stress = std::max(qNorm, aNorm);
        double aMin = std::min(m_prefAoiMin, m_prefAoiMax);
        double aMax = std::max(m_prefAoiMin, m_prefAoiMax);
        prefAoI = aMin + (aMax - aMin) * stress;
        prefAoI = std::min(1.0, std::max(0.0, prefAoI));
        prefEnergy = 1.0 - prefAoI;
    }
    // NS_LOG_UNCOND("energy:" << energyNorm << "\tAOI:" << aoiNorm);
    static uint32_t count = 0;
    // Update Q for previous window (transition prev -> current) only if we observed something now.
    if (ctx.hasPrev && hasObservation)
    {
        uint32_t idx = ctx.prevStateIdx * m_actionCount + ctx.prevActionIdx;
        uint32_t visits = ++visitTable[idx];
        double alpha =
            std::max(m_alphaMin, m_alpha / std::sqrt(static_cast<double>(std::max<uint32_t>(1, visits))));
        double switchPenalty = ctx.pendingSwitch ? m_switchPenaltyJ : 0.0;
        double deltaEnergy = ctx.energySumJ - ctx.lastRewardEnergySumJ;
        if (deltaEnergy < 0.0)
        {
            deltaEnergy = 0.0;
        }
        double deltaAoiSum = ctx.aoiSumMs - ctx.lastRewardAoiSumMs;
        if (deltaAoiSum < 0.0)
        {
            deltaAoiSum = 0.0;
        }
        uint64_t deltaAoiSamples = 0;
        if (ctx.aoiSamples >= ctx.lastRewardAoiSamples)
        {
            deltaAoiSamples = ctx.aoiSamples - ctx.lastRewardAoiSamples;
        }
        else
        {
            deltaAoiSamples = ctx.aoiSamples;
        }
        double deltaAoiMean =
            (deltaAoiSamples > 0) ? (deltaAoiSum / static_cast<double>(deltaAoiSamples)) : 0.0;
        double rewardEnergy = -(deltaEnergy + ctx.pendingSwitchEnergy) / energyNorm;
        double rewardAoI = -(deltaAoiMean) / aoiNorm;

        // Greedy bootstrap from next state using preference scalarization.
        double bestScalar = -1e30;
        std::pair<double, double> bestQ{0.0, 0.0};
        for (uint8_t a = 0; a < m_actionCount; ++a)
        {
            auto q = qTable[stateIdx * m_actionCount + a];
            double scalar = prefEnergy * q.first + prefAoI * q.second;
            if (scalar > bestScalar)
            {
                bestScalar = scalar;
                bestQ = q;
            }
        }

        auto& qPrev = qTable[idx];
        qPrev.first += alpha * (rewardEnergy - switchPenalty + m_gamma * bestQ.first - qPrev.first);
        qPrev.second += alpha * (rewardAoI - switchPenalty + m_gamma * bestQ.second - qPrev.second);
        ctx.pendingSwitchEnergy = 0.0;
        ctx.pendingSwitch = false;
        
    }

    if (isSwitching)
    {
        NrBwpSwitchDecision decision;
        decision.bsr = state.bsr;
        ctx.hasPrev = false;
        ctx.pendingSwitchEnergy = 0.0;
        return decision;
    }

    if (!hasObservation)
    {
        NrBwpSwitchDecision decision;
        decision.bsr = state.bsr;
        ctx.hasPrev = false;
        ctx.pendingSwitchEnergy = 0.0;
        return decision;
    }
    
    double rE = ((ctx.hasPrev) ? (-(ctx.energySumJ - ctx.lastRewardEnergySumJ + ctx.pendingSwitchEnergy) / energyNorm) : 0.0);
    double rA = ((ctx.hasPrev) ? (-( (ctx.aoiSamples >= ctx.lastRewardAoiSamples && (ctx.aoiSamples - ctx.lastRewardAoiSamples) > 0) ? ((ctx.aoiSumMs - ctx.lastRewardAoiSumMs) / static_cast<double>(ctx.aoiSamples - ctx.lastRewardAoiSamples)) : 0.0 ) / aoiNorm) : 0.0);

    ctx.lastRewardEnergySumJ = ctx.energySumJ;
    ctx.lastRewardAoiSumMs = ctx.aoiSumMs;
    ctx.lastRewardAoiSamples = ctx.aoiSamples;

    bool allowSwitch = m_minDwellTime.IsZero() ||
                       (state.dwellSeconds >= m_minDwellTime.GetSeconds());
    auto actionAllowed = [&](uint8_t a) -> bool {
        if (allowSwitch)
        {
            return true;
        }
        return DecodeActionBwp(a) == state.currentBwpId;
    };

    // Epsilon-greedy selection on scalarized Q values.
    bool explore = m_rng->GetValue() < epsilon;
    uint8_t chosen = 0;
    
    if (explore)
    {
        // NS_LOG_UNCOND("EXPLORE");
        if (allowSwitch)
        {
            chosen = static_cast<uint8_t>(m_rng->GetInteger(0, m_actionCount - 1));
        }
        else
        {
            uint8_t candidateCount = 0;
            for (uint8_t a = 0; a < m_actionCount; ++a)
            {
                if (actionAllowed(a))
                {
                    candidateCount++;
                }
            }
            uint8_t pick = static_cast<uint8_t>(m_rng->GetInteger(0, std::max<uint8_t>(1, candidateCount) - 1));
            for (uint8_t a = 0; a < m_actionCount; ++a)
            {
                if (!actionAllowed(a))
                {
                    continue;
                }
                if (pick == 0)
                {
                    chosen = a;
                    break;
                }
                pick--;
            }
        }
    }
    else
    {
        // NS_LOG_UNCOND("EXPLOIT");
        double bestScalar = -1e30;
        for (uint8_t a = 0; a < m_actionCount; ++a)
        {
            if (!actionAllowed(a))
            {
                continue;
            }
            auto q = qTable[stateIdx * m_actionCount + a];
            double scalar = prefEnergy * q.first + prefAoI * q.second;
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
        double scalarPrev = prefEnergy * qTable[stateIdx * m_actionCount + chosen].first +
                            prefAoI * qTable[stateIdx * m_actionCount + chosen].second;
        NS_LOG_UNCOND(Simulator::Now().As(Time::MS) << "\tQDBG rnti=" << rnti << " eval=" << evalCount << " stateIdx=" << stateIdx
                                   << " queueBin=" << +state.queueBin << " aoiBin=" << +state.aoiBin
                                   << " dwellBin=" << +state.dwellBin << " bwp=" << +state.currentBwpId
                                   << " pri=" << +state.priority << " action=" << +chosen
                                   << " tgtBwp=" << +decision.targetBwpId
                                   << " tgtPri=" << +decision.targetPriority
                                   << " eps=" << epsilon
                                   << " alpha=" << ((ctx.hasPrev) ? std::max(m_alphaMin, m_alpha / std::sqrt(static_cast<double>(std::max<uint32_t>(1, visitTable[ctx.prevStateIdx * m_actionCount + ctx.prevActionIdx])))) : m_alpha)
                                   << " rE=" << rE
                                   << " rEnergySumJ=" << ((ctx.hasPrev) ? ctx.energySumJ : 0.0)
                                   << " rPendingSwitchEnergy=" << ((ctx.hasPrev) ? ctx.pendingSwitchEnergy : 0.0)
                                   << " rENorm=" << energyNorm
                                   << " rSwitchPenaltyJ=" << ((ctx.hasPrev && ctx.pendingSwitch) ? m_switchPenaltyJ : 0.0)
                                   << " rA=" << rA
                                   << " rAvgAoI=" << ((ctx.hasPrev) ? state.avgAoIMs : 0.0)
                                   << " rAoINorm=" << aoiNorm
                                   << " wE=" << prefEnergy
                                   << " wA=" << prefAoI
                                   << " scalarQ=" << scalarPrev
                                   << " bestScalar=" << (ctx.hasPrev ? prefEnergy * qTable[stateIdx * m_actionCount + chosen].first + prefAoI * qTable[stateIdx * m_actionCount + chosen].second : 0.0));
    }

    if((++count) % 50 == 0)
    {
        NS_LOG_INFO(Simulator::Now().As(Time::S) << " RunInternalPolicy rnti=" << rnti << " stateIdx=" << stateIdx << " bwp=" << +state.currentBwpId
                                          << " pri=" << +state.priority << " queueBin=" << +state.queueBin
                                          << " aoiBin=" << +state.aoiBin << " dwellBin=" << +state.dwellBin
                                          << " action=" << +chosen << " tgtBwp=" << +decision.targetBwpId
                                          << " tgtPri=" << +decision.targetPriority);
        count = 0;
    }

    if (hasObservation)
    {
        ctx.prevStateIdx = stateIdx;
        ctx.prevActionIdx = chosen;
        ctx.hasPrev = true;
    }
    else
    {
        ctx.hasPrev = false;
    }

    if (decision.targetBwpId != std::numeric_limits<uint8_t>::max() &&
        decision.targetBwpId != state.currentBwpId && m_gnbManager != nullptr)
    {
        // NS_LOG_UNCOND("SWITCH TO " << (int) decision.targetBwpId);
        ctx.pendingSwitchEnergy =
            m_gnbManager->GetEnergyConfig().GetSwitchEnergy(state.currentBwpId, decision.targetBwpId);
        ctx.pendingSwitch = true;
    }
    else
    {
        ctx.pendingSwitchEnergy = 0.0;
        ctx.pendingSwitch = false;
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
        if (!m_minDwellTime.IsZero() &&
            state.dwellSeconds < m_minDwellTime.GetSeconds())
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

    bool eventOnlyQlearning =
        m_eventDrivenEvaluation && m_enableInternalPolicy &&
        m_internalPolicyMode == POLICY_Q_LEARNING && m_policy.IsNull();
    if (eventOnlyQlearning)
    {
        return;
    }

    for (auto& kv : m_ueContext)
    {
        uint16_t rnti = kv.first;
        auto& ctx = kv.second;

        // uint8_t groupMod = std::max<uint8_t>(1, m_evalGroupModulo);
        // if (groupMod > 1)
        // {
        //     uint8_t group = static_cast<uint8_t>(rnti % groupMod);
        //     uint8_t active = static_cast<uint8_t>(m_evalRound % groupMod);
        //     if (group != active)
        //     {
        //         continue;
        //     }
        // }

        EvaluateUe(rnti, true);
    }

    m_evalRound++;
    ScheduleEvaluation();
}

void
NrBwpSwitchTriggerHelper::EvaluateUe(uint16_t rnti, bool resetPeriod)
{
    auto it = m_ueContext.find(rnti);
    if (it == m_ueContext.end())
    {
        return;
    }
    auto& ctx = it->second;

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
    double avgAoIMs = (ctx.aoiSamples > 0) ? ctx.aoiSumMs / static_cast<double>(ctx.aoiSamples) : 0.0;
    bool isInternalQlearning =
        m_enableInternalPolicy && m_internalPolicyMode == POLICY_Q_LEARNING && m_policy.IsNull();
    if (isInternalQlearning && ctx.aoiSamples == 0 && ctx.queueSamples == 0)
    {
        // Skip no-observation windows to avoid treating "no packet observed" as AoI=0/queue=0.
        return;
    }
    uint8_t queueBin = QuantizeQueue(avgQueue);
    uint8_t aoiBin = QuantizeAoI(avgAoIMs);
    NS_LOG_UNCOND(Simulator::Now().As(Time::MS) << "\tEval rnti=" << rnti << "\tavgQueue=" << avgQueue << "B\tavgAoI=" << avgAoIMs
                             << "ms\tsamplesAoi=" << ctx.aoiSamples << "\tsamplesQ=" << ctx.queueSamples);

    double dwellSeconds =
        (ctx.lastSwitchTimeSeconds > 0.0) ? (now - ctx.lastSwitchTimeSeconds) : 0.0;
    uint8_t dwellBin = QuantizeDwell(dwellSeconds);

    NrBwpSwitchState state;
    state.currentBwpId = ctx.currentBwp;
    state.bsr = ctx.lastBsr;
    state.switchingRemaining =
        (m_gnbManager != nullptr) ? m_gnbManager->GetSwitchingRemaining(rnti) : Seconds(0);
    state.avgQueueSizeBytes = avgQueue;
    state.avgAoIMs = avgAoIMs;
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
            decision = RunInternalPolicy(rnti, state, stateIdx, ctx, isSwitching);
        }
    }

    if (decision.targetBwpId != std::numeric_limits<uint8_t>::max() &&
        decision.targetBwpId != state.currentBwpId)
    {
        NS_LOG_UNCOND(Simulator::Now().As(Time::MS) << "\tSWITCH_DECISION rnti=" << rnti
                                                   << " curBwp=" << +state.currentBwpId
                                                   << " tgtBwp=" << +decision.targetBwpId
                                                   << " avgQueue=" << state.avgQueueSizeBytes
                                                   << "B avgAoI=" << state.avgAoIMs
                                                       << "ms queueBin=" << +state.queueBin
                                                       << " aoiBin=" << +state.aoiBin
                                                   << " dwell=" << state.dwellSeconds << "s");
    }

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

    bool eventDrivenQlearning =
        m_eventDrivenEvaluation && isInternalQlearning;
    if (resetPeriod || (eventDrivenQlearning && !resetPeriod))
    {
        ctx.ResetPeriod();
    }
}

} // namespace ns3
