// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_BWP_SWITCH_CONTROLLER_H
#define NR_BWP_SWITCH_CONTROLLER_H

#include "ns3/object.h"
#include "ns3/callback.h"
#include "ns3/nstime.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/nr-bwp-energy-config.h"
#include "ns3/nr-phy-mac-common.h"

#include <map>
#include <unordered_map>
#include <limits>
#include <utility>

namespace ns3
{

class NrBwpSwitchTriggerHelper;

/**
 * @brief DTO carrying the policy decision (target BWP and optional metadata).
 */
struct NrBwpSwitchDecision
{
    uint8_t targetBwpId{std::numeric_limits<uint8_t>::max()};
};

/**
 * @brief DTO that aggregates the state passed to the policy.
 */
struct NrBwpSwitchState
{
    uint8_t currentBwpId{0};
    Time switchingRemaining{Seconds(0)};                  //!< Guard window left, if any
    double avgQueueSizeBytes{0.0};                        //!< Averaged over the evaluation window
    double avgAoIMs{0.0};                                 //!< Averaged over the evaluation window (ms)
    double avgSchedIntervalMs{0.0};                       //!< Averaged scheduling TB size (bytes)
    double lastSchedIntervalMs{0.0};                      //!< Last observed scheduling TB size (bytes)
    double filteredSchedIntervalMs{0.0};                  //!< Kalman-filtered scheduling TB size (bytes)
    double schedIntervalNis{0.0};                         //!< NIS for last scheduling update
    double schedIntervalConfidence{0.0};                  //!< Confidence derived from filter variance
    uint8_t queueBin{0};                                  //!< Discrete bin for queue size
    uint8_t aoiBin{0};                                    //!< Discrete bin for AoI
    double dwellSeconds{0.0};                             //!< Time since last BWP switch (s)
    uint8_t dwellBin{0};                                  //!< Discrete bin for dwell time
};

/**
 * @brief Controller that listens to MAC/BWP metrics and triggers BWP switches.
 *
 * The controller is policy-agnostic; a user-provided callback maps state -> decision.
 * The controller then invokes ForceUeBwp/ForceActiveBwp on the managers.
 */
class NrBwpSwitchController : public Object
{
  public:
    using PolicyCallback = Callback<NrBwpSwitchDecision, const NrBwpSwitchState&>;

    static TypeId GetTypeId();

    NrBwpSwitchController();
    ~NrBwpSwitchController() override = default;

    void SetGnbManager(const Ptr<BwpManagerGnb>& gnbManager);
    void AddUeManager(uint16_t rnti, const Ptr<BwpManagerUe>& ueManager);
    void SetPolicy(const PolicyCallback& cb);
    Ptr<NrBwpSwitchTriggerHelper> GetTriggerHelper() const;

    /**
     * @brief Record enqueue time for AoI calculation.
     */
    void RecordEnqueue(uint16_t rnti, uint8_t lcid);

    /**
     * @brief Record ACK arrival to update AoI.
     */
    void RecordAck(uint16_t rnti, uint8_t lcid);
    void RecordAoiSample(Time delay);

    /**
     * @brief Trace sink for MAC DL scheduling events (per-UE).
     */
    void HandleDlScheduling(const NrSchedulingCallbackInfo& info);
    static void DlSchedulingCallback(Ptr<NrBwpSwitchController> controller,
                                     std::string path,
                                     NrSchedulingCallbackInfo info);

    double GetAverageAoISeconds() const;
    double GetTotalSwitchEnergyJ() const;
    void OnHelperDecision(uint16_t rnti, const NrBwpSwitchDecision& decision);


  private:
    NrBwpSwitchDecision RunPolicy(const NrBwpSwitchState& state) const;
    void ApplySwitch(uint16_t rnti, uint8_t targetBwp);
    double LookupSwitchEnergy(uint8_t fromBwp, uint8_t toBwp) const;
    void DoForceBothSides(uint16_t rnti, uint8_t targetBwp);

    PolicyCallback m_policy;
    Ptr<BwpManagerGnb> m_gnbManager;
    std::unordered_map<uint16_t, Ptr<BwpManagerUe>> m_ueManagers;
    Ptr<NrBwpSwitchTriggerHelper> m_triggerHelper;
    bool m_useTriggerHelper{false};

    std::unordered_map<uint64_t, Time> m_lastEnqueue; // key: (rnti<<8 | lcid)
    uint64_t m_aoiSamples{0};
    double m_aoiAccumulatedSeconds{0.0};

    double m_totalSwitchEnergyJ{0.0};

    Time m_retryDelay{MilliSeconds(1)};
};

} // namespace ns3

#endif // NR_BWP_SWITCH_CONTROLLER_H
