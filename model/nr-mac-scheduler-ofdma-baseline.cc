/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "nr-mac-scheduler-ofdma-baseline.h"
#include "nr-mac-scheduler-ue-info-pf.h"
#include "ns3/log.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerOfdmaBaseline");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerOfdmaBaseline);

NrMacSchedulerOfdmaBaseline::NrMacSchedulerOfdmaBaseline()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerOfdmaBaseline::~NrMacSchedulerOfdmaBaseline()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrMacSchedulerOfdmaBaseline::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerOfdmaBaseline")
            .SetParent<NrMacSchedulerOfdmaPF>()
            .AddConstructor<NrMacSchedulerOfdmaBaseline>()
            .AddAttribute("Mode",
                          "The baseline scheduling mode (AGE_OPTIMAL, LYAPUNOV, DRQN, PF)",
                          EnumValue(PF_DEFAULT),
                          MakeEnumAccessor<Mode>(&NrMacSchedulerOfdmaBaseline::m_mode),
                          MakeEnumChecker(AGE_OPTIMAL, "AGE_OPTIMAL",
                                          LYAPUNOV, "LYAPUNOV",
                                          DRQN, "DRQN",
                                          PF_DEFAULT, "PF"));
    return tid;
}

void
NrMacSchedulerOfdmaBaseline::SetBaselineMode(const std::string& mode)
{
    if (mode == "age_optimal") m_mode = AGE_OPTIMAL;
    else if (mode == "lyapunov") m_mode = LYAPUNOV;
    else if (mode == "drqn") m_mode = DRQN;
    else m_mode = PF_DEFAULT;
}

void
NrMacSchedulerOfdmaBaseline::SetUeWeight(uint16_t rnti, double weight)
{
    m_customWeight[rnti] = weight;
}

void
NrMacSchedulerOfdmaBaseline::SetUeQueueSize(uint16_t rnti, uint32_t bytes)
{
    m_queueSize[rnti] = bytes;
}

void
NrMacSchedulerOfdmaBaseline::SetUeAoiMs(uint16_t rnti, double aoiMs)
{
    m_aoiMs[rnti] = aoiMs;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaBaseline::GetUeCompareDlFn() const
{
    if (m_mode == PF_DEFAULT)
    {
        return NrMacSchedulerOfdmaPF::GetUeCompareDlFn();
    }

    return [this](const NrMacSchedulerNs3::UePtrAndBufferReq& a,
                  const NrMacSchedulerNs3::UePtrAndBufferReq& b) {
        auto getWeight = [this](const NrMacSchedulerNs3::UePtrAndBufferReq& req) {
            uint16_t rnti = req.first->m_rnti;

            switch (m_mode)
            {
            case AGE_OPTIMAL:
                return m_aoiMs.count(rnti) ? m_aoiMs.at(rnti) : 0.0;
            case LYAPUNOV:
                return m_queueSize.count(rnti) ? static_cast<double>(m_queueSize.at(rnti)) : 0.0;
            case DRQN:
                return m_customWeight.count(rnti) ? m_customWeight.at(rnti) : 1.0;
            case PF_DEFAULT:
            default:
                return 0.0; // Handled by if statement above
            }
        };

        return getWeight(a) > getWeight(b);
    };
}

} // namespace ns3

