/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef NR_MAC_SCHEDULER_OFDMA_BASELINE_H
#define NR_MAC_SCHEDULER_OFDMA_BASELINE_H

#include "nr-mac-scheduler-ofdma-pf.h"
#include <unordered_map>

namespace ns3 {

class NrMacSchedulerOfdmaBaseline : public NrMacSchedulerOfdmaPF
{
public:
    enum Mode { AGE_OPTIMAL, LYAPUNOV, DRQN, PF_DEFAULT };

    NrMacSchedulerOfdmaBaseline();
    virtual ~NrMacSchedulerOfdmaBaseline() override;

    static TypeId GetTypeId();

    void SetBaselineMode(const std::string& mode);
    void SetUeWeight(uint16_t rnti, double weight);
    void SetUeQueueSize(uint16_t rnti, uint32_t bytes);
    void SetUeAoiMs(uint16_t rnti, double aoiMs);

protected:
    std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                       const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
    GetUeCompareDlFn() const override;

private:
    Mode m_mode{PF_DEFAULT};
    mutable std::unordered_map<uint16_t, double> m_customWeight;
    mutable std::unordered_map<uint16_t, uint32_t> m_queueSize;
    mutable std::unordered_map<uint16_t, double> m_aoiMs;
};

} // namespace ns3

#endif // NR_MAC_SCHEDULER_OFDMA_BASELINE_H