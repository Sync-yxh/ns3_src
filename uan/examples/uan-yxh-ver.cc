#include "uan-yxh-ver.h"
#include "uan-yxh-ql.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/stats-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/rng-seed-manager.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UanYXH");

Experiment::Experiment(): 
    sinkNums(2),
    sensorNums(4),
	dataRate(10240),
    cwMin(10),
    gapDataData(4),        // second
    gapStateData(10),
    gapStateState(10),       // minite
    slotTime(Seconds(0.2))
{
}

void Experiment::SetNodes()
{
    sinks = NodeContainer();
    sinks.Create(sinkNums);
    sensors = NodeContainer();
    sensors.Create(sensorNums);

    nodes = NodeContainer();
    nodes = NodeContainer(sinks,sensors);
    nodeNums = nodes.GetN();

    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
    energySourceHelper.Install (nodes);
    
    for(NodeContainer::Iterator node = nodes.Begin(); node != nodes.End(); node++)
    {
        statePktIndex[(*node)] = 0;
        dataPktIndex[(*node)] = 0;
        staticSendNum[(*node)] = 0;
    }
    staticSumPacket.clear();
}

void Experiment::SetPhy()
{
    std::string perModel = "ns3::UanPhyPerGenDefault";
    std::string sinrModel = "ns3::UanPhyCalcSinrDefault";
    ObjectFactory obf;
    obf.SetTypeId (perModel);
    Ptr<UanPhyPer> per = obf.Create<UanPhyPer> ();
    obf.SetTypeId (sinrModel);
    Ptr<UanPhyCalcSinr> sinr = obf.Create<UanPhyCalcSinr> ();

    UanTxMode mode;
    // Type,DataRateBps,PhyRateSps,cfHz,bwHz,constSize,name
    mode = UanTxModeFactory::CreateMode (UanTxMode::FSK, 
                                    dataRate,
                                    dataRate, 
                                    12000,
                                    dataRate, 
                                    2,"Default mode");
    UanModesList myModes;
    myModes.AppendMode (mode);

    uan.SetPhy ("ns3::UanPhyGen",
            "PerModel", PointerValue (per),
            "SinrModel", PointerValue (sinr),
            "SupportedModes", UanModesListValue (myModes));

}
void Experiment::SetLinks()
{
    uan.SetMac ("ns3::UanMacCw", "CW", UintegerValue (cwMin), "SlotTime", TimeValue (slotTime));

    Ptr<UanPropModelRange> prop = CreateObjectWithAttributes<UanPropModelRange> ();
    Ptr<UanChannel> channel = CreateObjectWithAttributes<UanChannel> ("PropagationModel", PointerValue (prop));
    
    NetDeviceContainer devices = uan.Install (nodes, channel);

    EnergySourceContainer energySourceContainer;
    NodeContainer::Iterator node = nodes.Begin ();
    while (node != nodes.End ())
    {
        energySourceContainer.Add ((*node)->GetObject<EnergySourceContainer> ()->Get (0));
        node++;
    }
    AcousticModemEnergyModelHelper acousticModemEnergyModelHelper;
    acousticModemEnergyModelHelper.Install (devices, energySourceContainer);
    
}

void Experiment::SetMobility()
{
    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobilityHelper.Install (nodes);
    nodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 0, 0));
    nodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (1500, 0, 0));
    nodes.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (1500, 1000, 0));
    nodes.Get (3)->GetObject<MobilityModel> ()->SetPosition (Vector (1000, 1000, 0));
    nodes.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (500, 2000, 0));
    nodes.Get (5)->GetObject<MobilityModel> ()->SetPosition (Vector (1500, 2000, 0));
}

void Experiment::UpdateMobility()
{
    NS_LOG_INFO("Time: " << Simulator::Now ().GetSeconds () << "s"  << " update mobility ");
    if(nodes.Get(1)->GetObject<MobilityModel>()->GetPosition().x > 200){
            nodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (150, 0, 0));
    }
    else{
            nodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (300, 0, 0));
    }
}

void Experiment::SetApplications()
{
    NodeContainer::Iterator node = nodes.Begin ();
    PacketSocketHelper packetSocketHelper;
    while (node != nodes.End ())
    {
        packetSocketHelper.Install (*node);
        PacketSocketAddress socketAddress;
        socketAddress.SetSingleDevice ((*node)->GetDevice (0)->GetIfIndex ());
        socketAddress.SetProtocol (0);
        sockets[*node] = Socket::CreateSocket (*node, TypeId::LookupByName ("ns3::PacketSocketFactory"));
        sockets[*node]->Bind ();
        sockets[*node]->Connect (socketAddress);
        sockets[*node]->SetRecvCallback (MakeCallback (&Experiment::RecvPacketCallback, this));
        node++;
    }
}

void Experiment::Prepare()
{
    RngSeedManager::SetSeed(8);
    SetNodes();
    SetMobility();
    SetPhy();
    SetLinks();
    SetApplications();
}

void Experiment::TeardownSocket ()
{
    std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;

    for (socket = sockets.begin (); socket != sockets.end (); socket++)
    {
    socket->second->Close ();
    }
}

void Experiment::ScheduleInit()
{
    //Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable> ();

    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;

        // expState[node] = INIT_STATE;

        uint32_t depth = node->GetObject<MobilityModel> ()->GetPosition().y;

        Ptr<Packet> pkt = Create<Packet> (1);

        UanPacketTag tag;
        tag.SetPacketType(UanPacketTag::INIT_PACKET);
        tag.SetSourceAddress( Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) );
        tag.SetPacketIndex(statePktIndex[node]);
        tag.SetDepth(depth);
        pkt->AddPacketTag(tag);
        statePktIndex[node] ++;
        dataPktIndex[node] = 0;

        //double time = uniformRandomVariable->GetValue (0, 2);
        Simulator::Schedule (Seconds (1), &Experiment::SendBroadCastPacket, this, node, pkt);
    }
    Simulator::Schedule (Seconds (gapStateData), &Experiment::ScheduleSendData, this);
    Simulator::Schedule (Minutes (gapStateState), &Experiment::ScheduleBeacon, this);
}

void Experiment::ScheduleBeacon()
{
    // Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable> ();

    staticSumPacket.push_back(0);

    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;

        // expState[node] = TRAIN_STATE;
        nextHopBackup[node].clear();
        
        uint8_t energy = (node->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
        uint32_t depth = node->GetObject<MobilityModel> ()->GetPosition().y;
        uint32_t bandwidth = sendPktBuff[node].size() + sendFailCnt[node];
        uint8_t connect = recieveRecord[node].size();

        NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " failedcnt:  " << sendFailCnt[node] );
        NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " bandwidth:  " << bandwidth );
        NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " connect:  " <<+connect );


        // static recieve nums and sinks' total nums
        uint32_t staticNum = 0;
        for(std::map<Mac8Address,uint32_t>::iterator iter = recieveRecord[node].begin(); iter != recieveRecord[node].end(); iter++)
        {
            staticNum += iter->second;
        }
        if(sinks.Contains(node->GetId()))
        {
            staticSumPacket[staticSumPacket.size()-1] += staticNum;
        }
        staticConnect[node].push_back(connect);
        staticBandwidth[node].push_back(bandwidth);

        sendPktBuff[node].clear();
        sendAddrBuff[node].clear();
        recieveRecord[node].clear();
        sendFailCnt[node] = 0;

        staticSendNum[node] = 0;

        Ptr<Packet> pkt = Create<Packet> (1);

        UanPacketTag tag;
        tag.SetPacketType(UanPacketTag::STATE_PACKET);
        tag.SetSourceAddress( Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) );
        tag.SetPacketIndex(statePktIndex[node]);
        tag.SetEnergy(energy);
        tag.SetDepth(depth);
        tag.SetBandwidth(bandwidth);
        tag.SetConnectNum(connect);
        pkt->AddPacketTag(tag);
        statePktIndex[node] ++;
        dataPktIndex[node] = 0;

        // NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " bandwidth:  " << bandwidth );


        // double time = uniformRandomVariable->GetValue (0, 2);
        // Simulator::Schedule (Seconds (time), &Experiment::SendBroadCastPacket, this, node, pkt);
        Simulator::Schedule (Seconds (1), &Experiment::SendBroadCastPacket, this, node, pkt);
    }

    Simulator::Schedule (Seconds (4), &Experiment::UpdateAgent, this);
    Simulator::Cancel(eventSendDataID);
    Simulator::Schedule (Seconds (gapStateData), &Experiment::ScheduleSendData, this);
    Simulator::Schedule (Minutes (gapStateState), &Experiment::ScheduleBeacon, this);
}

void Experiment::ScheduleSendData()
{
	// double t = Simulator::Now ().GetSeconds ();
	// NS_UNUSED(t);
    Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable> ();

    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;
        if(sinks.Contains(node->GetId())){
            continue;
        }

        Ptr<Packet> pkt = Create<Packet> (1024);
        Mac8Address dst;
        if(agent[node].GetAction(dst))
        {
            UanPacketTag tag;
            tag.SetPacketType(UanPacketTag::DATA_PACKET);
            tag.SetSourceAddress( Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) );
            tag.SetDestAddress(dst);
            tag.SetRelayAddress( Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) );
            tag.SetPacketIndex(dataPktIndex[node]);
            pkt->AddPacketTag(tag);
            dataPktIndex[node] ++;

            double time = uniformRandomVariable->GetValue (0, 1);
            Simulator::Schedule (Seconds (time), &Experiment::SendSinglePacket, this, node, pkt,dst);
        }
        else if(ChooseNextHop(node, dst))
        {
            UanPacketTag tag;
            tag.SetPacketType(UanPacketTag::DATA_PACKET);
            tag.SetSourceAddress( Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) );
            tag.SetDestAddress(dst);
            tag.SetRelayAddress( Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) );
            tag.SetPacketIndex(dataPktIndex[node]);
            pkt->AddPacketTag(tag);
            dataPktIndex[node] ++;

            double time = uniformRandomVariable->GetValue (0, 1);
            Simulator::Schedule (Seconds (time), &Experiment::SendSinglePacket, this, node, pkt,dst);

        }
    }

    eventSendDataID =  Simulator::Schedule (Seconds (gapDataData), &Experiment::ScheduleSendData, this);
}

void Experiment::SendSinglePacket(Ptr<Node> node, Ptr<Packet> pkt, Mac8Address dst)
{
    sendPktBuff[node].push_back(pkt);
    sendAddrBuff[node].push_back(dst);
    SendHandle(node);
}

void Experiment::SendBroadCastPacket(Ptr<Node> node, Ptr<Packet> pkt)
{
    Mac8Address dst( uint8_t(255) );
    sendPktBuff[node].push_back(pkt);
    sendAddrBuff[node].push_back(dst);
    SendHandle(node);
}

void Experiment::SendHandle(Ptr<Node> node)
{
    Ptr<NetDevice> device = node->GetDevice (0);
    if(!sendPktBuff[node].empty())
    {
        if(!device->IsPhyTx())
        {
            Mac8Address self = Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress());
            Mac8Address dst = sendAddrBuff[node].front();
            Ptr<Packet> pkt = sendPktBuff[node].front();
            PacketSocketAddress socketAddress;
            socketAddress.SetSingleDevice (node->GetDevice (0)->GetIfIndex ());
            socketAddress.SetPhysicalAddress (dst);
            socketAddress.SetProtocol (0);

            if(sockets[node]->SendTo (pkt, 0, socketAddress) != -1)
            {
                NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< self << " send packet to " << dst );
            }
    	    else
            {
                // NS_LOG_DEBUG( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< self << " packet sent faild " );
                NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " packet sent faild " );
                if(sendFailCnt.find(node) != sendFailCnt.end()){
                    sendFailCnt[node] ++;
                }
                else{
                    sendFailCnt[node] = 1;
                }
            }
            sendAddrBuff[node].pop_front();
            sendPktBuff[node].pop_front();
            txDelay[node] = Simulator::Now () + Seconds(double(pkt->GetSize() * 8) / dataRate) + Seconds(0.1);
            staticSendNum[node] ++;
        }
        else
        {
        	if(txDelay[node] <= Simulator::Now ())
        	{
                Simulator::Schedule (Seconds(0.02),&Experiment::SendHandle, this,node);
        	}
        	else
        	{
        		Simulator::Schedule (txDelay[node]-Simulator::Now (), &Experiment::SendHandle, this,node);
                txDelay[node] += Seconds(0.01);
        	}
        }
        
    }
}

void Experiment::SendPacketCallback(Ptr<Socket> skt, uint32_t available)
{
    NS_LOG_INFO( "SendCallback Called");
}

void Experiment::RecvPacketCallback(Ptr<Socket> skt)
{
    Address srcAddress;
    while (skt->GetRxAvailable () > 0)
    {
        Ptr<Packet> packet = skt->RecvFrom (srcAddress);
        RecvHandle(skt,packet);
    }
}

void Experiment::RecvHandle(Ptr<Socket> socket,Ptr<Packet> pkt)
{
    PacketTagIterator pktTagIter = pkt->GetPacketTagIterator();
    UanPacketTag tag;
    while(pktTagIter.HasNext()){
    	PacketTagIterator::Item tagItem  = pktTagIter.Next();
        if(tagItem.GetTypeId() == tag.GetTypeId()){
            tagItem.GetTag(tag);
            break;
        }
    }
    Ptr<Node> node = socket->GetNode();
    Mac8Address self = Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress());
    UanPacketTag::PacketType type = tag.GetPacketType();

    switch (type)
    {
    case UanPacketTag::INIT_PACKET:
    {
        if(sensors.Contains(node->GetId()))
        {
            Mac8Address src = tag.GetSourceAddress();
            uint32_t depth = tag.GetDepth();
            uint32_t m_depth = node->GetObject<MobilityModel>()->GetPosition().y;
            if(depth < m_depth)
            {
                nextHopBackup[node].push_back(src);
                NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " recv init pkt from Node " << src);
            }
        }
    }   break;
    case UanPacketTag::STATE_PACKET:
    {
        if( sensors.Contains(node->GetId()) )
        {
            StatePacketHandle(node,tag);
        }
    }   break;
    case UanPacketTag::DATA_PACKET:
    {
        Mac8Address dst = tag.GetDestAddress();
        Mac8Address src = tag.GetSourceAddress();
        uint32_t index = tag.GetPacketIndex();
        if(dst == self)
        {
            Mac8Address relay = tag.GetRelayAddress();
            if(recieveRecord[node].find(relay) != recieveRecord[node].end())
            {
                recieveRecord[node][relay] ++;
            }
            else
            {
                recieveRecord[node][relay] = 1;
            }

            tag.SetRelayAddress(self);

            NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " recv data pkt from Node " << src);

            if(agent[node].GetAction(dst))
            {
                tag.SetDestAddress(dst);
                pkt->RemoveAllPacketTags();
                pkt->AddPacketTag(tag);
                SendSinglePacket(node,pkt,dst);
                NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " relay data pkt  for Node " << src << " pkt No "<<+index);
            }
            else if(ChooseNextHop(node, dst))
            {
                tag.SetDestAddress(dst);
                pkt->RemoveAllPacketTags();
                pkt->AddPacketTag(tag);
                SendSinglePacket(node,pkt,dst);
                NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " relay data pkt  for Node " << src << " pkt No "<<+index);
            }
        }
    }   break;
    default:
        break;
    }
}

void Experiment::StatePacketHandle(Ptr<Node> node,  UanPacketTag tag)
{
    Mac8Address self = Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress());
    Mac8Address src = tag.GetSourceAddress();
    uint8_t energy = tag.GetEnergy();
    uint32_t depth = tag.GetDepth();
    uint32_t bandwidth = tag.GetBandwidth();
    uint8_t connect = tag.GetConnectNum();
    NS_UNUSED(energy);
    uint32_t m_depth = node->GetObject<MobilityModel>()->GetPosition().y;
    if(depth < m_depth)
    {
        State s(src, connect, m_depth-depth, bandwidth);
        QstateVector[node].push_back(s);
        nextHopBackup[node].push_back(src);
    }
    NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " recv state pkt  from Node " << src);
}

void Experiment::UpdateAgent()
{
    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;

        agent[node].NewStateHandle(QstateVector[node]);
    }
}

bool Experiment::ChooseNextHop(Ptr<Node> node, Mac8Address& next)
{
    if(nextHopBackup[node].empty())
        return false;

    next = nextHopBackup[node].front();
    return true;
}

int main (int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse (argc, argv);

    // LogComponentEnable ("UanYXH", LOG_INFO);
    LogComponentEnable ("UanYXH", LOG_DEBUG);
    // LogComponentEnable ("UanPhyGen", LOG_DEBUG);

    Experiment experiment;

    NS_ASSERT(true);
    //NS_ASSERT(false);

    experiment.Prepare();
    experiment.ScheduleInit();

    AnimationInterface anim("uan_yxh.xml");
  
    Simulator::Stop (Minutes (55));
    Simulator::Run ();
    Simulator::Destroy ();

    experiment.TeardownSocket();

    return 0;
}


