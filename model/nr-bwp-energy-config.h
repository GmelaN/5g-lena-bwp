// Copyright (c) 2024
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_BWP_ENERGY_CONFIG_H
#define NR_BWP_ENERGY_CONFIG_H

#include <map>
#include <utility>
#include <cstdint>

namespace ns3
{

/**
 * @brief Simple container for static switch energy costs between BWPs.
 */
class NrBwpEnergyConfig
{
  public:
    void SetSwitchEnergy(uint8_t fromBwp, uint8_t toBwp, double energyJ)
    {
        m_switchEnergyJ[{fromBwp, toBwp}] = energyJ;
    }

    double GetSwitchEnergy(uint8_t fromBwp, uint8_t toBwp) const
    {
        auto it = m_switchEnergyJ.find({fromBwp, toBwp});
        if (it == m_switchEnergyJ.end())
        {
            return 0.0;
        }
        return it->second;
    }

  private:
    std::map<std::pair<uint8_t, uint8_t>, double> m_switchEnergyJ;
};

} // namespace ns3

#endif // NR_BWP_ENERGY_CONFIG_H
