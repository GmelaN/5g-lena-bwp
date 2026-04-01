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
#include <optional>
#include <set>

namespace ns3
{

namespace
{

struct AequitasCandidate
{
    uint8_t mcs{0};
    bool canFinish{false};
    uint32_t requiredRbg{0};
    uint32_t usefulRbg{0};
    uint32_t allocRbg{0};
    double partialFraction{0.0};
    double score{-1e18};
    std::vector<uint32_t> selectedRbgs;
};

} // namespace

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
                          "Enable the Aequitas-specific MCS selection rule. If false, only the AoI-aware UE ordering is used.",
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
    if (it == m_aoiStateByRnti.end())
    {
        it = m_aoiStateByRnti.emplace(rnti, AoiState{}).first;
        it->second.deadlineMs = std::max(1e-6, m_defaultDeadlineMs);
    }

    const auto& state = it->second;
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
    if (!m_enableMcsSelection)
    {
        return NrMacSchedulerOfdma::AssignDLRBG(symAvail, activeDl);
    }

    GetFirst getBeamId;
    GetSecond getUeVector;
    BeamSymbolMap symPerBeam = GetSymPerBeam(symAvail, activeDl);

    auto allocateRbg = [](const std::shared_ptr<NrMacSchedulerUeInfo>& ue,
                          uint32_t rbg,
                          uint32_t beamSym,
                          FTResources& assignedResources,
                          std::vector<bool>& availableRbgs) {
        auto& assignedRbgs = ue->m_dlRBG;
        auto existingRbgs = assignedRbgs.size();
        assignedRbgs.resize(existingRbgs + beamSym);
        std::fill(assignedRbgs.begin() + existingRbgs, assignedRbgs.end(), rbg);

        auto& assignedSymbols = ue->m_dlSym;
        auto existingSymbols = assignedSymbols.size();
        assignedSymbols.resize(existingSymbols + beamSym);
        std::iota(assignedSymbols.begin() + existingSymbols, assignedSymbols.end(), 0);

        assignedResources.m_rbg++;
        assignedResources.m_sym = beamSym;
        availableRbgs.at(rbg) = false;
    };

    auto buildCandidate = [this](const UePtrAndBufferReq& ue,
                                 const std::set<uint32_t>& remainingRbgSet) -> std::optional<AequitasCandidate> {
        auto currentUe = ue.first;
        const uint32_t remainingBytes =
            (currentUe->m_dlTbSize >= ue.second) ? 0U : (ue.second - currentUe->m_dlTbSize);
        if (remainingBytes == 0 || remainingRbgSet.empty())
        {
            return std::nullopt;
        }

        const bool useWideband =
            currentUe->m_dlSbMcsInfo.empty() ||
            m_mcsCsiSource == NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS;
        const uint8_t widebandMcsCap = currentUe->m_dlMcs;

        auto countUsableRbgs = [&](uint8_t mcs) {
            uint32_t usable = 0;
            for (auto rbg : remainingRbgSet)
            {
                bool supported = useWideband || currentUe->m_dlSbMcsInfo
                                                   .at(currentUe->m_rbgToSb.at(rbg))
                                                   .mcs >= mcs;
                if (supported)
                {
                    usable++;
                }
            }
            return usable;
        };

        AequitasCandidate bestPartial;
        bestPartial.score = -1e18;
        int bestFullMcs = -1;
        uint32_t bestFullRequired = 0;

        const int maxMcs = std::min<int>(std::max(0, static_cast<int>(m_dlAmc ? m_dlAmc->GetMaxMcs() : 0U)),
                                         static_cast<int>(widebandMcsCap));
        for (int m = 0; m <= maxMcs; ++m)
        {
            const uint32_t usefulRbg = countUsableRbgs(static_cast<uint8_t>(m));
            if (usefulRbg == 0)
            {
                continue;
            }

            uint32_t requiredRbg = 0;
            for (uint32_t k = 1; k <= usefulRbg; ++k)
            {
                const uint32_t tbs =
                    m_dlAmc->CalculateTbSize(static_cast<uint8_t>(m),
                                             currentUe->m_dlRank,
                                             k * static_cast<uint32_t>(GetNumRbPerRbg()));
                if (tbs >= remainingBytes)
                {
                    requiredRbg = k;
                    break;
                }
            }

            if (requiredRbg > 0)
            {
                bestFullMcs = m;
                bestFullRequired = requiredRbg;
            }
            else
            {
                const double delivered = static_cast<double>(
                    m_dlAmc->CalculateTbSize(static_cast<uint8_t>(m),
                                             currentUe->m_dlRank,
                                             usefulRbg * static_cast<uint32_t>(GetNumRbPerRbg())));
                if (delivered > bestPartial.score)
                {
                    bestPartial.mcs = static_cast<uint8_t>(m);
                    bestPartial.canFinish = false;
                    bestPartial.requiredRbg = requiredRbg;
                    bestPartial.usefulRbg = usefulRbg;
                    bestPartial.allocRbg = usefulRbg;
                    bestPartial.partialFraction =
                        std::min(1.0, delivered / std::max(1.0, static_cast<double>(remainingBytes)));
                    bestPartial.score = delivered;
                }
            }
        }

        AequitasCandidate candidate;
        if (bestFullMcs >= 0)
        {
            candidate.mcs = static_cast<uint8_t>(bestFullMcs);
            candidate.canFinish = true;
            candidate.requiredRbg = bestFullRequired;
            candidate.usefulRbg = countUsableRbgs(candidate.mcs);
            candidate.allocRbg = bestFullRequired;
            candidate.partialFraction = 1.0;
        }
        else if (bestPartial.usefulRbg > 0)
        {
            candidate = bestPartial;
        }
        else
        {
            return std::nullopt;
        }

        std::vector<std::pair<uint32_t, uint8_t>> usableRbgs;
        usableRbgs.reserve(candidate.usefulRbg);
        for (auto rbg : remainingRbgSet)
        {
            uint8_t sbMcs = useWideband
                                ? candidate.mcs
                                : currentUe->m_dlSbMcsInfo.at(currentUe->m_rbgToSb.at(rbg)).mcs;
            if (sbMcs >= candidate.mcs)
            {
                usableRbgs.emplace_back(rbg, sbMcs);
            }
        }
        std::sort(usableRbgs.begin(),
                  usableRbgs.end(),
                  [](const auto& a, const auto& b) {
                      if (a.second != b.second)
                      {
                          return a.second > b.second;
                      }
                      return a.first < b.first;
                  });

        const uint32_t toAllocate = std::min<uint32_t>(candidate.allocRbg, usableRbgs.size());
        candidate.selectedRbgs.reserve(toAllocate);
        for (uint32_t i = 0; i < toAllocate; ++i)
        {
            candidate.selectedRbgs.push_back(usableRbgs[i].first);
        }
        candidate.usefulRbg = static_cast<uint32_t>(usableRbgs.size());
        candidate.allocRbg = toAllocate;

        if (candidate.selectedRbgs.empty())
        {
            return std::nullopt;
        }

        candidate.mcs = std::min(candidate.mcs, widebandMcsCap);
        double score = ComputePriority(ue);
        score += candidate.canFinish ? 100.0 : 0.0;
        score += 10.0 * candidate.partialFraction;
        score += 0.5 * static_cast<double>(candidate.mcs);
        candidate.score = score;
        return candidate;
    };

    for (const auto& el : activeDl)
    {
        const uint32_t beamSym = symPerBeam.at(getBeamId(el));
        std::vector<UePtrAndBufferReq> ueVector;
        FTResources assignedResources(0, 0);
        std::vector<bool> availableRbgs = GetDlBitmask();
        std::set<uint32_t> remainingRbgSet;

        for (size_t i = 0; i < availableRbgs.size(); ++i)
        {
            if (availableRbgs[i])
            {
                remainingRbgSet.emplace(i);
            }
        }

        for (const auto& ue : getUeVector(el))
        {
            ueVector.emplace_back(ue);
            BeforeDlSched(ueVector.back(), FTResources(beamSym, beamSym));
        }

        while (!remainingRbgSet.empty())
        {
            int bestIdx = -1;
            std::optional<AequitasCandidate> bestCandidate;

            for (size_t idx = 0; idx < ueVector.size(); ++idx)
            {
                auto candidate = buildCandidate(ueVector[idx], remainingRbgSet);
                if (!candidate.has_value())
                {
                    continue;
                }
                if (!bestCandidate.has_value() || candidate->score > bestCandidate->score)
                {
                    bestIdx = static_cast<int>(idx);
                    bestCandidate = std::move(candidate);
                }
            }

            if (bestIdx < 0 || !bestCandidate.has_value())
            {
                break;
            }

            auto ue = ueVector[bestIdx].first;
            const auto oldTbSize = ue->m_dlTbSize;
            const auto oldDlRbg = ue->m_dlRBG;
            const auto oldDlSym = ue->m_dlSym;
            const auto oldDlMcs = ue->m_dlMcs;
            const auto oldMcsCsiSource = ue->m_mcsCsiSource;
            const auto oldAssignedResources = assignedResources;
            ue->m_dlMcs = bestCandidate->mcs;
            ue->m_mcsCsiSource = NrMacSchedulerUeInfo::McsCsiSource::WIDEBAND_MCS;
            for (auto rbg : bestCandidate->selectedRbgs)
            {
                allocateRbg(ue, rbg, beamSym, assignedResources, availableRbgs);
                remainingRbgSet.erase(rbg);
            }
            ue->UpdateDlMetric();
            if (ue->m_dlTbSize < oldTbSize * 0.99 && ue->GetDlMcs() > 0)
            {
                ue->m_dlRBG = oldDlRbg;
                ue->m_dlSym = oldDlSym;
                ue->m_dlTbSize = oldTbSize;
                ue->m_dlMcs = oldDlMcs;
                ue->m_mcsCsiSource = oldMcsCsiSource;
                assignedResources = oldAssignedResources;
                for (auto rbg : bestCandidate->selectedRbgs)
                {
                    availableRbgs.at(rbg) = true;
                    remainingRbgSet.emplace(rbg);
                }
                auto& state =
                    const_cast<NrMacSchedulerOfdmaAequitas*>(this)->GetOrCreateState(ue->m_rnti);
                state.unfinishedFromPreviousTti = false;
                continue;
            }
            const double servedBytes = std::max<int64_t>(
                0,
                static_cast<int64_t>(ue->m_dlTbSize) - static_cast<int64_t>(oldTbSize));
            UpdateServedHistory(ue->m_rnti, servedBytes);

            auto& state = const_cast<NrMacSchedulerOfdmaAequitas*>(this)->GetOrCreateState(ue->m_rnti);
            state.unfinishedFromPreviousTti = !bestCandidate->canFinish;

            if (!bestCandidate->canFinish)
            {
                NotAssignedDlResources(ueVector[bestIdx],
                                       FTResources(beamSym, beamSym),
                                       assignedResources);
                continue;
            }

            for (size_t idx = 0; idx < ueVector.size(); ++idx)
            {
                if (static_cast<int>(idx) == bestIdx)
                {
                    continue;
                }
                NotAssignedDlResources(ueVector[idx], FTResources(beamSym, beamSym), assignedResources);
            }
        }

        for (const auto& ue : ueVector)
        {
            auto& state =
                const_cast<NrMacSchedulerOfdmaAequitas*>(this)->GetOrCreateState(ue.first->m_rnti);
            if (ue.first->m_dlTbSize >= std::max<uint32_t>(ue.second, 10U))
            {
                state.unfinishedFromPreviousTti = false;
            }
        }
    }

    return symPerBeam;
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
