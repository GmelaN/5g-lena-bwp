// Copyright (c) 2026
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ofdma-aequitas.h"

#include "nr-mac-scheduler-ofdma.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerOfdmaAequitas");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerOfdmaAequitas);

TypeId
NrMacSchedulerOfdmaAequitas::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerOfdmaAequitas")
            .SetParent<NrMacSchedulerOfdmaRR>()
            .AddConstructor<NrMacSchedulerOfdmaAequitas>()
            .AddAttribute("DefaultAoiDeadlineMs",
                          "Default AoI deadline used for UEs without an explicit deadline.",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_defaultDeadlineMs),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AoiHistoryAlpha",
                          "EMA alpha used for outdated-ratio and served-bytes history.",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_historyAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("WeightAoI",
                          "Weight of normalized AoI in the priority score.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_weightAoi),
                          MakeDoubleChecker<double>())
            .AddAttribute("WeightDeadline",
                          "Weight of normalized deadline violation in the priority score.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_weightDeadline),
                          MakeDoubleChecker<double>())
            .AddAttribute("WeightOutdated",
                          "Weight of outdated-ratio estimate in the priority score.",
                          DoubleValue(4.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_weightOutdated),
                          MakeDoubleChecker<double>())
            .AddAttribute("WeightBuffer",
                          "Weight of normalized DL backlog in the priority score.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_weightBuffer),
                          MakeDoubleChecker<double>())
            .AddAttribute("WeightMcs",
                          "Weight of normalized DL MCS in the priority score.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_weightMcs),
                          MakeDoubleChecker<double>())
            .AddAttribute("WeightFairness",
                          "Weight of served-bytes fairness penalty in the priority score.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_weightFairness),
                          MakeDoubleChecker<double>())
            .AddAttribute("WeightAllocationPenalty",
                          "Penalty for already allocated RBGs in the current slot.",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(
                              &NrMacSchedulerOfdmaAequitas::m_weightAllocationPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("UnfinishedBonus",
                          "Priority bonus applied to a UE with an unfinished transmission from the previous TTI.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_unfinishedBonus),
                          MakeDoubleChecker<double>())
            .AddAttribute("EnableMcsSelection",
                          "Deprecated switch; Aequitas now keeps only AoI-aware UE ordering and uses default OFDMA RB/MCS assignment.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrMacSchedulerOfdmaAequitas::m_enableMcsSelection),
                          MakeBooleanChecker())
            .AddAttribute("BufferNormBytes",
                          "Normalization factor for DL backlog in bytes.",
                          DoubleValue(10000.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_bufferNormBytes),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("ServedBytesNorm",
                          "Normalization factor for fairness history in bytes.",
                          DoubleValue(4000.0),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaAequitas::m_servedBytesNorm),
                          MakeDoubleChecker<double>(1.0));
    return tid;
}

NrMacSchedulerOfdmaAequitas::NrMacSchedulerOfdmaAequitas()
    : NrMacSchedulerOfdmaRR()
{
}

void
NrMacSchedulerOfdmaAequitas::SetUeAoiState(uint16_t rnti, double aoiMs, double deadlineMs)
{
    auto& state = GetOrCreateState(rnti);
    state.aoiMs = std::max(0.0, aoiMs);
    if (deadlineMs >= 0.0)
    {
        state.deadlineMs = std::max(1e-6, deadlineMs);
    }

    const double outdated = (state.aoiMs > state.deadlineMs) ? 1.0 : 0.0;
    state.outdatedRatio =
        (1.0 - m_historyAlpha) * state.outdatedRatio + m_historyAlpha * outdated;
}

void
NrMacSchedulerOfdmaAequitas::SetUeAoiDeadline(uint16_t rnti, double deadlineMs)
{
    auto& state = GetOrCreateState(rnti);
    state.deadlineMs = std::max(1e-6, deadlineMs);
}

void
NrMacSchedulerOfdmaAequitas::SetUeOutdatedRatio(uint16_t rnti, double ratio)
{
    auto& state = GetOrCreateState(rnti);
    state.outdatedRatio = std::clamp(ratio, 0.0, 1.0);
}

void
NrMacSchedulerOfdmaAequitas::ClearAoiState()
{
    m_aoiStateByRnti.clear();
}

NrMacSchedulerOfdmaAequitas::AoiState&
NrMacSchedulerOfdmaAequitas::GetOrCreateState(uint16_t rnti)
{
    auto [it, inserted] = m_aoiStateByRnti.try_emplace(rnti);
    if (inserted)
    {
        it->second.deadlineMs = std::max(1e-6, m_defaultDeadlineMs);
    }
    return it->second;
}

double
NrMacSchedulerOfdmaAequitas::ComputePriority(const UePtrAndBufferReq& ue) const
{
    const auto rnti = ue.first->m_rnti;
    auto it = m_aoiStateByRnti.find(rnti);
    AoiState fallbackState{};
    const AoiState* statePtr = nullptr;
    if (it == m_aoiStateByRnti.end())
    {
        fallbackState.deadlineMs = std::max(1e-6, m_defaultDeadlineMs);
        statePtr = &fallbackState;
    }
    else
    {
        statePtr = &it->second;
    }

    const auto& state = *statePtr;
    const double deadlineMs = std::max(1e-6, state.deadlineMs);
    const double aoiNorm = state.aoiMs / deadlineMs;
    const double violationNorm = std::max(0.0, state.aoiMs - deadlineMs) / deadlineMs;
    const double bufferNorm =
        std::min(1.0, static_cast<double>(ue.first->GetTotalDlBuffer()) / m_bufferNormBytes);
    const double maxDlMcs =
        std::max(1.0, static_cast<double>(m_dlAmc ? m_dlAmc->GetMaxMcs() : 1U));
    const double mcsNorm = static_cast<double>(ue.first->GetDlMcs()) / maxDlMcs;
    const double fairnessPenalty =
        std::min(1.0, state.emaServedBytes / std::max(1.0, m_servedBytesNorm));
    const double allocationPenalty = static_cast<double>(ue.first->m_dlRBG.size());

    const double unfinishedBonus = state.unfinishedFromPreviousTti ? m_unfinishedBonus : 0.0;

    static uint32_t sAeqLogCount = 0;
    if (sAeqLogCount < 20)
    {
        NS_LOG_INFO("AEQUITAS rnti=" << rnti << " aoiNorm=" << aoiNorm
                                     << " violNorm=" << violationNorm
                                     << " outdated=" << state.outdatedRatio
                                     << " bufferNorm=" << bufferNorm << " mcsNorm=" << mcsNorm
                                     << " fairnessPenalty=" << fairnessPenalty
                                     << " allocPenalty=" << allocationPenalty
                                     << " unfinishedBonus=" << unfinishedBonus);
        ++sAeqLogCount;
    }

    return unfinishedBonus + m_weightAoi * aoiNorm + m_weightDeadline * violationNorm +
           m_weightOutdated * state.outdatedRatio + m_weightBuffer * bufferNorm +
           m_weightMcs * mcsNorm - m_weightFairness * fairnessPenalty -
           m_weightAllocationPenalty * allocationPenalty;
}

void
NrMacSchedulerOfdmaAequitas::UpdateServedHistory(uint16_t rnti, double servedBytes) const
{
    auto& state = const_cast<NrMacSchedulerOfdmaAequitas*>(this)->GetOrCreateState(rnti);
    const double clipped = std::max(0.0, servedBytes);
    state.emaServedBytes = (1.0 - m_historyAlpha) * state.emaServedBytes + m_historyAlpha * clipped;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaAequitas::GetUeCompareDlFn() const
{
    return [this](const UePtrAndBufferReq& lhs, const UePtrAndBufferReq& rhs) {
        const double lhsScore = ComputePriority(lhs);
        const double rhsScore = ComputePriority(rhs);
        if (std::abs(lhsScore - rhsScore) > 1e-9)
        {
            return lhsScore > rhsScore;
        }

        if (lhs.first->GetDlMcs() != rhs.first->GetDlMcs())
        {
            return lhs.first->GetDlMcs() > rhs.first->GetDlMcs();
        }

        if (lhs.first->GetTotalDlBuffer() != rhs.first->GetTotalDlBuffer())
        {
            return lhs.first->GetTotalDlBuffer() > rhs.first->GetTotalDlBuffer();
        }

        return lhs.first->m_rnti < rhs.first->m_rnti;
    };
}

NrMacSchedulerNs3::BeamSymbolMap
NrMacSchedulerOfdmaAequitas::AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const
{
    // Keep Aequitas as a pure priority-order scheduler and leave RB/MCS assignment
    // to the default OFDMA implementation.
    return NrMacSchedulerOfdma::AssignDLRBG(symAvail, activeDl);
}

void
NrMacSchedulerOfdmaAequitas::AssignedDlResources(const UePtrAndBufferReq& ue,
                                                 [[maybe_unused]] const FTResources& assigned,
                                                 [[maybe_unused]] const FTResources& totAssigned) const
{
    auto oldTbSize = ue.first->m_dlTbSize;
    ue.first->UpdateDlMetric();
    const double servedBytes = std::max<int64_t>(0, static_cast<int64_t>(ue.first->m_dlTbSize) -
                                                        static_cast<int64_t>(oldTbSize));
    UpdateServedHistory(ue.first->m_rnti, servedBytes);
}

void
NrMacSchedulerOfdmaAequitas::NotAssignedDlResources(
    const UePtrAndBufferReq& ue,
    [[maybe_unused]] const FTResources& notAssigned,
    [[maybe_unused]] const FTResources& totalAssigned) const
{
    UpdateServedHistory(ue.first->m_rnti, 0.0);
}

} // namespace ns3
