// Copyright (c) 2025 Jo Seoung Hyeon <gmelan@gnu.ac.kr>
//
// SPDX-License-Identifier: GPL-2.0-only


#include "ns3/object.h"
#include "ns3/bwp-manager-algorithm-dynamic.h"
#include "nr-eps-bearer.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("BwpManagerAlgorithmDynamic");
NS_OBJECT_ENSURE_REGISTERED(BwpManagerAlgorithmDynamic);

TypeId
BwpManagerAlgorithmDynamic::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::BwpManagerAlgorithmDynamic")
            .SetParent<BwpManagerAlgorithm>()
            .SetGroupName("nr")
            ;
    return tid;
}


uint8_t
BwpManagerAlgorithmDynamic::GetBwpForEpsBearer(const NrEpsBearer::Qci& v) const
{
    return 0;
}

}