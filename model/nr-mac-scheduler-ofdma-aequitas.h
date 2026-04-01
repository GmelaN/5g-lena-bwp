// Copyright (c) 2026
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "nr-mac-scheduler-ofdma-rr.h"

#include <unordered_map>

namespace ns3
{

/**
 * @ingroup scheduler
 * @brief AoI-aware OFDMA scheduler inspired by Aequitas.
 *
 * This scheduler keeps the 5G-LENA OFDMA scheduling flow and augments the
 * per-UE DL ordering with externally supplied AoI state. Resource allocation is
 * therefore still performed jointly with the standard CQI/MCS path in 5G-LENA,
 * while the user ordering is biased toward UEs with stale information.
 */
class NrMacSchedulerOfdmaAequitas : public NrMacSchedulerOfdmaRR
{
  public:
    static TypeId GetTypeId();

    NrMacSchedulerOfdmaAequitas();
    ~NrMacSchedulerOfdmaAequitas() override = default;

    /**
     * @brief Update the current AoI state for a UE.
     * @param rnti UE RNTI
     * @param aoiMs current AoI in milliseconds
     * @param deadlineMs optional deadline in milliseconds; negative keeps current/default
     */
    void SetUeAoiState(uint16_t rnti, double aoiMs, double deadlineMs = -1.0);

    /**
     * @brief Update only the AoI deadline for a UE.
     * @param rnti UE RNTI
     * @param deadlineMs deadline in milliseconds
     */
    void SetUeAoiDeadline(uint16_t rnti, double deadlineMs);

    /**
     * @brief Update the long-term outdated ratio estimate for a UE.
     * @param rnti UE RNTI
     * @param ratio value in [0,1]
     */
    void SetUeOutdatedRatio(uint16_t rnti, double ratio);

    /**
     * @brief Reset all externally supplied AoI state.
     */
    void ClearAoiState();

  protected:
    BeamSymbolMap AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const override;

    std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                       const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
    GetUeCompareDlFn() const override;

    void AssignedDlResources(const UePtrAndBufferReq& ue,
                             const FTResources& assigned,
                             const FTResources& totAssigned) const override;

    void NotAssignedDlResources(const UePtrAndBufferReq& ue,
                                const FTResources& notAssigned,
                                const FTResources& totalAssigned) const override;

  private:
    struct AoiState
    {
        double aoiMs{0.0};
        double deadlineMs{20.0};
        double outdatedRatio{0.0};
        double emaServedBytes{0.0};
        bool unfinishedFromPreviousTti{false};
    };

    AoiState& GetOrCreateState(uint16_t rnti);
    double ComputePriority(const NrMacSchedulerNs3::UePtrAndBufferReq& ue) const;
    void UpdateServedHistory(uint16_t rnti, double servedBytes) const;

    mutable std::unordered_map<uint16_t, AoiState> m_aoiStateByRnti;

    double m_defaultDeadlineMs{20.0};
    double m_historyAlpha{0.1};
    double m_weightAoi{2.0};
    double m_weightDeadline{3.0};
    double m_weightOutdated{4.0};
    double m_weightBuffer{1.0};
    double m_weightMcs{0.5};
    double m_weightFairness{1.0};
    double m_weightAllocationPenalty{0.25};
    double m_unfinishedBonus{5.0};
    bool m_enableMcsSelection{true};
    double m_bufferNormBytes{10000.0};
    double m_servedBytesNorm{4000.0};
};

} // namespace ns3
