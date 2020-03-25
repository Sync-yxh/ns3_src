/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: ye xiaohan<>
 */

#include "uan-prop-model-range.h"
#include "uan-tx-mode.h"
#include "ns3/mobility-model.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (UanPropModelRange);

UanPropModelRange::UanPropModelRange ()
{
}

UanPropModelRange::~UanPropModelRange ()
{
}

TypeId
UanPropModelRange::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::UanPropModelRange")
        .SetParent<UanPropModel> ()
        .SetGroupName ("Uan")
        .AddConstructor<UanPropModelRange> ()
  ;
    return tid;
}


double
UanPropModelRange::GetPathLossDb (Ptr<MobilityModel> a, Ptr<MobilityModel> b, UanTxMode mode)
{
    if(a->GetDistanceFrom (b) > 1500){
        return 2000; // initial value of tx power for db is 190 db
    }
    else{
        return 0;
    }
}
UanPdp
UanPropModelRange::GetPdp (Ptr<MobilityModel> a, Ptr<MobilityModel> b, UanTxMode mode)
{
    return UanPdp::CreateImpulsePdp ();
}

Time
UanPropModelRange::GetDelay (Ptr<MobilityModel> a, Ptr<MobilityModel> b, UanTxMode mode)
{
    return Seconds (a->GetDistanceFrom (b) / 1500.0);
}


} // namespace ns3
