/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef NR_MAC_SCHEDULER_OFDMA_BASELINE_H
#define NR_MAC_SCHEDULER_OFDMA_BASELINE_H

#include "nr-mac-scheduler-ofdma-pf.h"
#include <unordered_map>

namespace ns3 {

class NrMacSchedulerOfdmaBaseline : public NrMacSchedulerOfdmaPF
{
public:
    enum Mode { AGE_OPTIMAL, LYAPUNOV, TPS, DGS, DRQN, PF_DEFAULT };

    NrMacSchedulerOfdmaBaseline();
    virtual ~NrMacSchedulerOfdmaBaseline() override;

    static TypeId GetTypeId();

    void SetBaselineMode(const std::string& mode);
    void SetUeWeight(uint16_t rnti, double weight);
    void SetUeQueueSize(uint16_t rnti, uint32_t bytes);
    void SetUeAoiMs(uint16_t rnti, double aoiMs);
    void SetUeReceiverAoiMs(uint16_t rnti, double aoiMs);
    void SetUeTbErrorRate(uint16_t rnti, double errorRate);
    void SetUeDlMcs(uint16_t rnti, double mcs);
    void SetUeDeadlineMs(uint16_t rnti, double deadlineMs);
    void SetUeHolDelayMs(uint16_t rnti, double holDelayMs);
    void SetStepDurationMs(double stepMs);
    void SetAgeOptimalGamma(double gamma);

protected:
    std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                       const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
    GetUeCompareDlFn() const override;

private:
    double EstimateExpectedBytesPerPrbFromMcs(uint8_t mcs, uint32_t rank) const;

    Mode m_mode{PF_DEFAULT};
    mutable std::unordered_map<uint16_t, double> m_customWeight;
    mutable std::unordered_map<uint16_t, uint32_t> m_queueSize;
    mutable std::unordered_map<uint16_t, double> m_aoiMs;
    mutable std::unordered_map<uint16_t, double> m_receiverAoiMs;
    mutable std::unordered_map<uint16_t, double> m_tbErrorRate;
    mutable std::unordered_map<uint16_t, double> m_dlMcs;
    mutable std::unordered_map<uint16_t, double> m_deadlineMs;
    mutable std::unordered_map<uint16_t, double> m_holDelayMs;
    double m_stepDurationMs{20.0};
    double m_ageOptimalGamma{20.0};
};

} // namespace ns3

#endif // NR_MAC_SCHEDULER_OFDMA_BASELINE_H
