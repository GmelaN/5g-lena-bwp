// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_BWP_SWITCH_TRIGGER_HELPER_H
#define NR_BWP_SWITCH_TRIGGER_HELPER_H

#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/nr-bwp-switch-controller.h"
#include "ns3/random-variable-stream.h"

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ns3
{

/**
 * @brief Helper that aggregates per-UE state and periodically runs a policy
 * to decide BWP switches. Designed to host Pareto Q-learning.
 */
class NrBwpSwitchTriggerHelper : public Object
{
  public:
    static TypeId GetTypeId();

    enum InternalPolicyMode
    {
        POLICY_Q_LEARNING = 0,
        POLICY_WINDOW_DETECT = 1,
        NO_POLICY_BWP0 = 2,
        NO_POLICY_BWP1 = 3
    };

    NrBwpSwitchTriggerHelper();
    ~NrBwpSwitchTriggerHelper() override;

    void SetPolicy(const NrBwpSwitchController::PolicyCallback& cb);
    void SetDecisionCallback(const Callback<void, uint16_t, const NrBwpSwitchDecision&>& cb);
    void SetGnbManager(const Ptr<BwpManagerGnb>& gnbManager);
    void AddUeManager(uint16_t rnti, const Ptr<BwpManagerUe>& ueManager);

    void NotifyDlQueue(uint16_t rnti, uint8_t lcid, uint8_t currentBwp, uint32_t queueBytes);
    void NotifyDlScheduling(uint16_t rnti, uint8_t bwpId, uint32_t tbSizeBytes);
    void RecordEnqueue(uint16_t rnti, uint8_t lcid);
    void RecordAck(uint16_t rnti, uint8_t lcid);
    void RecordAoiSample(uint16_t rnti, uint8_t lcid, Time delay);

  private:
    static constexpr uint8_t m_dwellBins = 11; // 0..10 (100 ms with 10 ms bins)
    static constexpr uint8_t m_actionCount = 2; // 2 BWP targets

    struct PerUeContext
    {
        uint8_t currentBwp{0};

        uint32_t prevStateIdx{0};
        uint8_t prevActionIdx{0};
        bool hasPrev{false};
        double pendingSwitchEnergy{0.0};

        double queueSumBytes{0.0};
        uint64_t queueSamples{0};

        double aoiSumMs{0.0};
        uint64_t aoiSamples{0};

        double energySumJ{0.0};

        std::unordered_map<uint64_t, Time> pending; // key: (rnti<<8 | lcid)
        double lastSwitchTimeSeconds{0.0};
        double lastEvalTimeSeconds{0.0};

        uint8_t policyCurrentBwp{0};
        bool policyInitialized{false};

        enum SwitchState
        {
            DEFAULT_ACTIVE,
            SPECIFIC_ACTIVE,
            SWITCHING
        };

        SwitchState switchState{DEFAULT_ACTIVE};
        Time tWindow{Seconds(0)};
        Time tOnDefaultActive{Seconds(0)};
        Time tSpecificIdle{Seconds(0)};
        Time tSwitch{Seconds(0)};

        void ResetPeriod()
        {
            queueSumBytes = 0.0;
            queueSamples = 0;
            aoiSumMs = 0.0;
            aoiSamples = 0;
            energySumJ = 0.0;
        }
    };

    PerUeContext& GetOrCreateCtx(uint16_t rnti);
    void DoDispose() override;
    void ScheduleEvaluation();
    void EvaluateWindow();
    uint8_t QuantizeQueue(double avgQueue) const;
    uint8_t QuantizeAoI(double avgAoIMs) const;
    uint8_t QuantizeDwell(double dwellSeconds) const;
    uint32_t ComputeStateIndex(uint8_t queueBin, uint8_t aoiBin, uint8_t dwellBin, uint8_t bwp) const;
    NrBwpSwitchDecision RunInternalPolicy(uint16_t rnti,
                                          const NrBwpSwitchState& state,
                                          uint32_t stateIdx,
                                          PerUeContext& ctx,
                                          bool isSwitching);
    NrBwpSwitchDecision RunWindowDetectPolicy(uint16_t rnti,
                                              const NrBwpSwitchState& state,
                                              PerUeContext& ctx,
                                              Time dt,
                                              bool isSwitching);
    void HandleTxEnergy(uint16_t rnti, uint8_t bwpId, uint32_t bytes, double energyJ);
    std::vector<std::pair<double, double>>& GetQTableForUe(uint16_t rnti);
    std::vector<uint32_t>& GetVisitTableForUe(uint16_t rnti);
    uint32_t& GetEvalCountForUe(uint16_t rnti);
    uint8_t DecodeActionBwp(uint8_t actionIdx) const;

    NrBwpSwitchController::PolicyCallback m_policy;
    Callback<void, uint16_t, const NrBwpSwitchDecision&> m_decisionCb;

    Ptr<BwpManagerGnb> m_gnbManager;
    std::unordered_map<uint16_t, Ptr<BwpManagerUe>> m_ueManagers;
    std::unordered_map<uint16_t, PerUeContext> m_ueContext;

    const uint32_t m_stateCount{3 * 3 * m_dwellBins * 2}; // queue x aoi x dwell x bwp
    std::unordered_map<uint16_t, std::vector<std::pair<double, double>>> m_qTablesByUe; // per-UE Q-table
    std::unordered_map<uint16_t, std::vector<uint32_t>> m_visitTablesByUe; // state-action visit counts
    std::unordered_map<uint16_t, uint32_t> m_evalCountsByUe; // per-UE evaluation windows processed
    Ptr<UniformRandomVariable> m_rng;
    bool m_txEnergyConnected{false};

    EventId m_evalEvent;

    Time m_evalPeriod{MilliSeconds(50)};
    Time m_startDelay{Seconds(60)}; // guard before starting evaluations
    bool m_enableInternalPolicy{true};
    uint32_t m_queueThreshold1{2000};
    uint32_t m_queueThreshold2{8000};
    Time m_aoiThreshold1{MilliSeconds(5)};
    Time m_aoiThreshold2{MilliSeconds(10)};
    double m_energyPerByteJ{0.0};
    double m_prefEnergy{0.5};
    double m_prefAoI{0.5};
    double m_staticPowerBwp0Mw{100.0};
    double m_staticPowerBwp1Mw{200.0};
    double m_alpha{0.15};
    double m_alphaMin{0.02};
    double m_gamma{0.95};
    double m_epsilon{0.15};
    double m_epsilonMin{0.02};
    double m_epsilonDecay{5e-3};
    double m_rewardEnergyNorm{0.01};
    double m_rewardAoiNorm{10.0};
    uint8_t m_evalGroupModulo{1};
    uint64_t m_evalRound{0};
    InternalPolicyMode m_internalPolicyMode{POLICY_Q_LEARNING};
    uint32_t m_switchThresholdBytes{0};
    Time m_controlWindow{Seconds(0)};
    Time m_detectTime{Seconds(0)};
    Time m_specificIdleTime{Seconds(0)};
    Time m_switchingDelay{Seconds(0)};
    uint8_t m_defaultBwpId{0};
    uint8_t m_specificBwpId{1};

    TracedCallback<uint16_t, const NrBwpSwitchState&, const NrBwpSwitchDecision&> m_decisionTrace;
};

} // namespace ns3

#endif // NR_BWP_SWITCH_TRIGGER_HELPER_H
