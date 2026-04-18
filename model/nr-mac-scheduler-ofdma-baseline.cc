/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "nr-mac-scheduler-ofdma-baseline.h"
#include "nr-mac-scheduler-ue-info-pf.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>
#include <vector>

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
                          "The baseline scheduling mode (AGE_OPTIMAL, LYAPUNOV, TPS, DGS, DRQN, PF)",
                          EnumValue(PF_DEFAULT),
                          MakeEnumAccessor<Mode>(&NrMacSchedulerOfdmaBaseline::m_mode),
                          MakeEnumChecker(AGE_OPTIMAL, "AGE_OPTIMAL",
                                          LYAPUNOV, "LYAPUNOV",
                                          TPS, "TPS",
                                          DGS, "DGS",
                                          DRQN, "DRQN",
                                          PF_DEFAULT, "PF"));
    return tid;
}

void
NrMacSchedulerOfdmaBaseline::SetBaselineMode(const std::string& mode)
{
    if (mode == "age_optimal") m_mode = AGE_OPTIMAL;
    else if (mode == "lyapunov") m_mode = LYAPUNOV;
    else if (mode == "tps") m_mode = TPS;
    else if (mode == "dgs") m_mode = DGS;
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

void
NrMacSchedulerOfdmaBaseline::SetUeReceiverAoiMs(uint16_t rnti, double aoiMs)
{
    m_receiverAoiMs[rnti] = std::max(0.0, aoiMs);
}

void
NrMacSchedulerOfdmaBaseline::SetUeTbErrorRate(uint16_t rnti, double errorRate)
{
    m_tbErrorRate[rnti] = std::max(0.0, std::min(0.99, errorRate));
}

void
NrMacSchedulerOfdmaBaseline::SetUeDlMcs(uint16_t rnti, double mcs)
{
    m_dlMcs[rnti] = std::max(0.0, mcs);
}

void
NrMacSchedulerOfdmaBaseline::SetUeDeadlineMs(uint16_t rnti, double deadlineMs)
{
    m_deadlineMs[rnti] = std::max(1e-6, deadlineMs);
}

void
NrMacSchedulerOfdmaBaseline::SetUeHolDelayMs(uint16_t rnti, double holDelayMs)
{
    m_holDelayMs[rnti] = std::max(0.0, holDelayMs);
}

void
NrMacSchedulerOfdmaBaseline::SetStepDurationMs(double stepMs)
{
    m_stepDurationMs = std::max(1e-6, stepMs);
}

void
NrMacSchedulerOfdmaBaseline::SetAgeOptimalGamma(double gamma)
{
    m_ageOptimalGamma = std::max(0.0, gamma);
}

double
NrMacSchedulerOfdmaBaseline::EstimateExpectedBytesPerPrbFromMcs(uint8_t mcs, uint32_t rank) const
{
    const uint32_t rbPerRbg = std::max<uint32_t>(1u, static_cast<uint32_t>(GetNumRbPerRbg()));
    const uint32_t usedRank = std::max<uint32_t>(1u, rank);
    const uint32_t tbsOneRbg =
        m_dlAmc ? m_dlAmc->CalculateTbSize(mcs, usedRank, rbPerRbg) : 0u;
    return std::max(1.0, static_cast<double>(tbsOneRbg) / static_cast<double>(rbPerRbg));
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaBaseline::GetUeCompareDlFn() const
{
    if (m_mode == PF_DEFAULT)
    {
        return NrMacSchedulerOfdmaPF::GetUeCompareDlFn();
    }

    std::unordered_map<uint16_t, double> tpsXStar;
    std::unordered_map<uint16_t, bool> tpsAdmitted;
    if (m_mode == TPS)
    {
        struct TpsCandidate
        {
            uint16_t rnti{0};
            double xStar{0.0};
        };

        std::vector<TpsCandidate> candidates;
        candidates.reserve(m_queueSize.size());
        const double rbAvail = static_cast<double>(std::max<uint16_t>(1u, GetBandwidthInRbg())) *
                               static_cast<double>(std::max<uint64_t>(1u, GetNumRbPerRbg()));

        for (const auto& [rnti, q] : m_queueSize)
        {
            const double queueBytes = static_cast<double>(q);
            if (queueBytes <= 0.0)
            {
                continue;
            }

            const double mcs = m_dlMcs.count(rnti) ? m_dlMcs.at(rnti) : 0.0;
            const double bytesPerPrb =
                EstimateExpectedBytesPerPrbFromMcs(static_cast<uint8_t>(std::clamp(mcs, 0.0, 31.0)),
                                                   1u);
            const double deadlineMs = m_deadlineMs.count(rnti) ? m_deadlineMs.at(rnti) : 100.0;
            const double ttiPerDeadline =
                std::max(1.0, std::floor(deadlineMs / std::max(1e-6, m_stepDurationMs)));
            const double reqPerTti = std::ceil(queueBytes / ttiPerDeadline);
            const double xStar = std::ceil(reqPerTti / std::max(1.0, bytesPerPrb));
            tpsXStar[rnti] = xStar;
            candidates.push_back({rnti, xStar});
        }

        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            if (a.xStar != b.xStar)
            {
                return a.xStar < b.xStar;
            }
            return a.rnti < b.rnti;
        });

        double remainingRb = rbAvail;
        for (const auto& c : candidates)
        {
            const bool admitted = (remainingRb >= c.xStar);
            tpsAdmitted[c.rnti] = admitted;
            if (admitted)
            {
                remainingRb -= c.xStar;
            }
        }
    }

    return [this,
            tpsXStar = std::move(tpsXStar),
            tpsAdmitted = std::move(tpsAdmitted)](const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                                                  const NrMacSchedulerNs3::UePtrAndBufferReq& rhs) {
        auto getWeight = [this, &tpsXStar, &tpsAdmitted](const NrMacSchedulerNs3::UePtrAndBufferReq& req) {
            uint16_t rnti = req.first->m_rnti;

            switch (m_mode)
            {
            case AGE_OPTIMAL:
            {
                const double deltaT = m_aoiMs.count(rnti) ? std::max(0.0, m_aoiMs.at(rnti)) : 0.0;
                const double deltaR =
                    m_receiverAoiMs.count(rnti) ? std::max(0.0, m_receiverAoiMs.at(rnti)) : deltaT;
                const double epsilon =
                    m_tbErrorRate.count(rnti) ? std::max(0.0, std::min(0.99, m_tbErrorRate.at(rnti))) : 0.0;
                const double reliability = std::max(1e-6, 1.0 - epsilon);
                const double gap = deltaR - deltaT;
                const double threshold = m_ageOptimalGamma / reliability;
                static uint32_t sAgeLogCount = 0;
                if (sAgeLogCount < 20)
                {
                    NS_LOG_INFO("AGE_OPTIMAL rnti=" << rnti << " deltaR=" << deltaR
                                                    << " deltaT=" << deltaT << " eps=" << epsilon
                                                    << " gap=" << gap << " thr=" << threshold);
                    ++sAgeLogCount;
                }
                return (gap >= threshold) ? (reliability * gap) : 0.0;
            }
            case LYAPUNOV:
            {
                const double q = m_queueSize.count(rnti) ? static_cast<double>(m_queueSize.at(rnti)) : 0.0;
                static uint32_t sLyaLogCount = 0;
                if (sLyaLogCount < 20)
                {
                    NS_LOG_INFO("LYAPUNOV rnti=" << rnti << " queueBytes=" << q);
                    ++sLyaLogCount;
                }
                return m_queueSize.count(rnti) ? static_cast<double>(m_queueSize.at(rnti)) : 0.0;
            }
            case TPS:
            {
                const double queueBytes =
                    m_queueSize.count(rnti) ? static_cast<double>(m_queueSize.at(rnti)) : 0.0;
                if (queueBytes <= 0.0)
                {
                    return 0.0;
                }
                const double xStar = tpsXStar.count(rnti) ? tpsXStar.at(rnti) : 1.0e9;
                const bool admitted = tpsAdmitted.count(rnti) ? tpsAdmitted.at(rnti) : false;
                static uint32_t sTpsLogCount = 0;
                if (sTpsLogCount < 20)
                {
                    NS_LOG_INFO("TPS rnti=" << rnti << " queue=" << queueBytes
                                             << " xStar=" << xStar << " admitted=" << admitted);
                    ++sTpsLogCount;
                }
                if (admitted)
                {
                    return 1000.0 + 1.0 / (1.0 + xStar);
                }
                return 1.0 / (1.0 + xStar);
            }
            case DGS:
            {
                const double tau = m_deadlineMs.count(rnti) ? m_deadlineMs.at(rnti) : 100.0;
                const double elapsed =
                    m_holDelayMs.count(rnti) ? std::max(0.0, m_holDelayMs.at(rnti)) : 0.0;
                const uint8_t mcs = req.first ? req.first->GetDlMcs() : 0U;
                const uint32_t rank = req.first ? std::max<uint32_t>(1u, req.first->m_dlRank) : 1u;
                const double dr = EstimateExpectedBytesPerPrbFromMcs(mcs, rank);
                const double slack = tau - elapsed;
                static uint32_t sDgsLogCount = 0;
                if (sDgsLogCount < 20)
                {
                    NS_LOG_INFO("DGS rnti=" << rnti << " tau=" << tau << " elapsed=" << elapsed
                                             << " slack=" << slack << " dr=" << dr);
                    ++sDgsLogCount;
                }
                return -slack + ((slack <= 0.0) ? (1000.0 + std::abs(slack)) : 0.0) + 0.01 * dr;
            }
            case DRQN:
                return m_customWeight.count(rnti) ? m_customWeight.at(rnti) : 1.0;
            case PF_DEFAULT:
            default:
                return 0.0; // Handled by if statement above
            }
        };

        return getWeight(lhs) > getWeight(rhs);
    };
}

} // namespace ns3
