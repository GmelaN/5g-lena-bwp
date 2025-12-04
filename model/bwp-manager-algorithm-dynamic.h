// Copyright (c) 2025 Jo Seoung Hyeon <gmelan@gnu.ac.kr>
//
// SPDX-License-Identifier: GPL-2.0-only


#ifndef BWPMANAGERALGORITHMDYNAMIC_H
#define BWPMANAGERALGORITHMDYNAMIC_H


#include "ns3/object.h"
#include "ns3/bwp-manager-algorithm.h"
namespace ns3
{

/**
 * @ingroup bwp
 * @brief The BwpManagerAlgorithmDynamic class
 *
 * A dynamic manager: it gets the association through a series of Attributes.
 */
class BwpManagerAlgorithmDynamic : public BwpManagerAlgorithm
{
  public:
    /**
     * @brief GetTypeId
     * @return The TypeId of the object
     */
    static TypeId GetTypeId();

    /**
     * @brief constructor
     */
    BwpManagerAlgorithmDynamic() = default;
    /**
     * ~BwpManagerAlgorithm
     */
    ~BwpManagerAlgorithmDynamic() override = default;
    /**
     * @brief Get the bandwidth part id for the Qci specified
     * @param v the qci
     * @return the bwp id that the algorithm selects for the qci specified
     */
    virtual uint8_t GetBwpForEpsBearer(const NrEpsBearer::Qci& v) const override;
};
}
#endif


