#include "uan-yxh-ver3.h"
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
    sinkNums(3),
    sensorNums(9),
	dataRate(10240),
    dataPktSize(2560),
    simulatorTime(1500),
    gapDataData(5),           // second
    gapStateData(10),       // second
    gapStateAgent(4),       // second
    gapStateState(5),       // minite
    gapTrainTest(1200.03),    // minite
    gapUpdateMob(2.5),   //minite
    randomSeed(55),
    protocal(1)
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
    energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (90000000));
    energySourceHelper.Install (nodes);
    
    for(NodeContainer::Iterator node = nodes.Begin(); node != nodes.End(); node++)
    {
        uint32_t nodeId = (*node)->GetId();

        // exp buffer
        statePktIndex[(*node)] = 0;
        dataPktIndex[(*node)] = 0;
        sendPktBuff[(*node)].clear();
        sendAddrBuff[(*node)].clear();

        // static buffer
        staticSendNum[nodeId] = 0;
        staticSendFailCnt[nodeId] = 0;
        staticConnect[nodeId].clear();
        staticBandwidth[nodeId].clear();
        staticRecieveRecord[nodeId].clear();
        staticNextHopBackup[nodeId].clear();

        QstateVector[nodeId].clear();

        AVEindex[nodeId] = 0;
        staticActionOtherProt[nodeId].clear();
        staticRecieveRecordForDBR[nodeId].clear();
    }
    staticSumPacket.clear();
}

void Experiment::SetAnimNodesSize(AnimationInterface& anim)
{
    for(NodeContainer::Iterator node = nodes.Begin(); node != nodes.End(); node++)
    {
        uint32_t nodeId = (*node)->GetId();
        anim.UpdateNodeSize(nodeId,50,50);
    }
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
    uan.SetMac ("ns3::UanMacAloha");

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
    //sinks
    sinks.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (800, 0, 0));
    sinks.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (2500, 0, 0));
    sinks.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (4200, 0, 0));
    //sensors
    sensors.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (100, 900, 0));
    sensors.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (1700, 900, 0));
    sensors.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (3250, 700, 0));
    sensors.Get (3)->GetObject<MobilityModel> ()->SetPosition (Vector (4100, 1000, 0));
    sensors.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (250, 1800, 0));
    sensors.Get (5)->GetObject<MobilityModel> ()->SetPosition (Vector (1600, 1850, 0));
    sensors.Get (6)->GetObject<MobilityModel> ()->SetPosition (Vector (2200, 1600, 0));
    sensors.Get (7)->GetObject<MobilityModel> ()->SetPosition (Vector (3600, 1750, 0));
    sensors.Get (8)->GetObject<MobilityModel> ()->SetPosition (Vector (4300, 1900, 0));

    velocity[nodes.Get(9)->GetId()].X = -0.04;
    velocity[nodes.Get(9)->GetId()].Y = -0.001;
    velocity[nodes.Get(10)->GetId()].X = -0.04;
    velocity[nodes.Get(10)->GetId()].Y = 0; 
    boundary[nodes.Get(9)->GetId()].lowerX = 2200 - 1800;
    boundary[nodes.Get(9)->GetId()].upperX = 2200 + 500;
    boundary[nodes.Get(10)->GetId()].lowerX = 3600 - 1800;
    boundary[nodes.Get(10)->GetId()].upperX = 3600 + 500;
}

void Experiment::UpdateMobility()
{
    for(uint32_t id = 0; id<nodeNums; id++)
    {
        if(velocity.find(id) != velocity.end()){
            Vector pos = nodes.Get(id)->GetObject<MobilityModel>()->GetPosition();
            Vector newpos = pos;
            newpos.x += velocity[id].X * gapUpdateMob * 60;
            newpos.y += velocity[id].Y * gapUpdateMob * 60;
            if(newpos.x > boundary[id].upperX || newpos.x < boundary[id].lowerX){
                // velocity[id].X = -1 * velocity[id].X;
                // velocity[id].Y = -1 * velocity[id].Y;
                velocity[id].X = 0;
                velocity[id].Y = 0;
            }
            else{
                nodes.Get(id)->GetObject<MobilityModel>()->SetPosition(newpos);
            }
        }
    }
    Simulator::Schedule (Minutes (gapUpdateMob), &Experiment::UpdateMobility, this);
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
    RngSeedManager::SetSeed(randomSeed);
    gapTrainTest = simulatorTime - 300 + 0.03;
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
    NS_LOG_LOGIC(" Init " << Simulator::Now ().GetSeconds () << "s");
    expState = ExpState::CONTROL_STATE;

    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;

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
    // Simulator::Schedule (Minutes (gapTrainTest), &Experiment::ChangeAgentStage, this);
    Simulator::Schedule (Minutes (gapUpdateMob), &Experiment::UpdateMobility, this);
}

void Experiment::ScheduleBeacon()
{
    // Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable> ();
    expState = ExpState::CONTROL_STATE;

    staticSumPacket.push_back(0);
    NS_LOG_LOGIC(" Beacon " << Simulator::Now ().GetSeconds () << "s");
    if(protocal == 3){
        std::map<Mac8Address,std::map<uint32_t,uint32_t>> staticPktTotalDBR;
        for(NodeContainer::Iterator nodeIter = sinks.Begin(); nodeIter != sinks.End(); nodeIter++)
        {
            uint32_t nodeId = (*nodeIter)->GetId();
            std::map<Mac8Address,std::map<uint32_t,uint32_t>> singleRecv = staticRecieveRecordForDBR[nodeId];
            std::map<Mac8Address,std::map<uint32_t,uint32_t>>::iterator s_iter = singleRecv.begin();
            for(;s_iter != singleRecv.end(); s_iter++)
            {
                Mac8Address s_src = s_iter->first;
                std::map<uint32_t,uint32_t> i_map = s_iter->second;
                for(std::map<uint32_t,uint32_t>::iterator i_iter = i_map.begin(); i_iter!=i_map.end();i_iter++)
                {
                    uint32_t indexkey = i_iter->first;
                    staticPktTotalDBR[s_src][indexkey] = 1;
                }
            }
            staticRecieveRecordForDBR[nodeId].clear();
        }
        for(std::map<Mac8Address,std::map<uint32_t,uint32_t>>::iterator iter = staticPktTotalDBR.begin(); iter!=staticPktTotalDBR.end(); iter++)
        {
            staticSumPacket[staticSumPacket.size()-1] += iter->second.size();
        }
    }

    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;
        uint32_t nodeId = node->GetId();
        
        uint8_t energy = (node->GetObject<EnergySourceContainer> ()->Get (0)->GetEnergyFraction ()) * 100;
        uint32_t depth = node->GetObject<MobilityModel> ()->GetPosition().y;
        uint32_t bandwidth = sendPktBuff[node].size() + staticSendFailCnt[nodeId];
        uint8_t connect = staticConnectBackup[nodeId].size();
        //  for(std::map<Mac8Address,uint32_t>::iterator iter = staticRecieveRecord[nodeId].begin(); iter != staticRecieveRecord[nodeId].end(); iter++)
        // {
        //    if(iter->second > 3){
        //        connect ++;
        //    }
        // }

        // NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " failedcnt:  " << staticSendFailCnt[nodeId] );
        NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " bandwidth:  " << bandwidth );
        // NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " Node "<< Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress()) << " connect:  " <<+connect );

        // static recieve nums and sinks' total nums
        uint32_t staticNum = 0;
        if(protocal != 3){
            for(std::map<Mac8Address,uint32_t>::iterator iter = staticRecieveRecord[nodeId].begin(); iter != staticRecieveRecord[nodeId].end(); iter++)
            {
                staticNum += iter->second;
            }
            if(sinks.Contains(node->GetId()))
            {
                staticSumPacket[staticSumPacket.size()-1] += staticNum;
            }
        }
        staticConnect[nodeId].push_back(connect);
        staticBandwidth[nodeId].push_back(bandwidth);

        sendPktBuff[node].clear();
        sendAddrBuff[node].clear();

        staticRecieveRecord[nodeId].clear();
        staticSendFailCnt[nodeId] = 0;
        staticSendNum[nodeId] = 0;
        staticNextHopBackup[nodeId].clear();
        staticConnectBackup[nodeId].clear();

        QstateVector[nodeId].clear();

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

        // double time = uniformRandomVariable->GetValue (0, 2);
        Simulator::Schedule (Seconds (1), &Experiment::SendBroadCastPacket, this, node, pkt);
    }

    NS_LOG_DEBUG("Time: " << Simulator::Now ().GetSeconds () << "s" << " total sink recv " <<  staticSumPacket[staticSumPacket.size()-1] ) ;
    Simulator::Cancel(eventSendDataID);
    Simulator::Schedule (Seconds (5), &Experiment::UpdateAgent, this);
    Simulator::Schedule (Seconds (gapStateData), &Experiment::ScheduleSendData, this);
    Simulator::Schedule (Minutes (gapStateState), &Experiment::ScheduleBeacon, this);
}

void Experiment::ScheduleSendData()
{
    // NS_LOG_LOGIC(" Data " << Simulator::Now ().GetSeconds () << "s");
    expState = ExpState::DATA_STATE;
	// double t = Simulator::Now ().GetSeconds ();
	// NS_UNUSED(t);
    Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable> ();

    for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
    {
        Ptr<Node> node = *nodeIter;
        uint32_t nodeId= node->GetId();
        if(sinks.Contains(nodeId)){
            continue;
        }

        Ptr<Packet> pkt = Create<Packet> (dataPktSize);
        Mac8Address dst;

        if(ChooseNextHop(node, dst))
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
            Simulator::Schedule (Seconds (time), &Experiment::SendSinglePacket, this, node, pkt,dst,true);
        }
    }

    eventSendDataID =  Simulator::Schedule (Seconds (gapDataData), &Experiment::ScheduleSendData, this);
}

void Experiment::SendSinglePacket(Ptr<Node> node, Ptr<Packet> pkt, Mac8Address dst, bool flag)
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
    uint32_t nodeId = node->GetId();
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

            if(sockets[node]->SendTo (pkt, 0, socketAddress) == -1){
                if(staticSendFailCnt.find(nodeId) != staticSendFailCnt.end()){
                    staticSendFailCnt[nodeId] ++;
                }
                else{
                    staticSendFailCnt[nodeId] = 1;
                }
            }
            sendAddrBuff[node].pop_front();
            sendPktBuff[node].pop_front();
            txDelay[node] = Simulator::Now () + Seconds(double(pkt->GetSize() * 8) / dataRate) + Seconds(0.001);
            staticSendNum[nodeId] ++;
        }
        else{
            Simulator::Schedule( txDelay[node]-Simulator::Now(), &Experiment::SendHandle, this, node);
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
    uint32_t nodeId = node->GetId();
    Mac8Address self = Mac8Address::ConvertFrom(node->GetDevice(0)->GetAddress());
    UanPacketTag::PacketType type = tag.GetPacketType();

    switch (type)
    {
    case UanPacketTag::INIT_PACKET:
    {
        if(sensors.Contains(nodeId))
        {
            Mac8Address src = tag.GetSourceAddress();
            uint32_t depth = tag.GetDepth();
            uint32_t m_depth = node->GetObject<MobilityModel>()->GetPosition().y;
            if(depth < m_depth)
            {
                staticNextHopBackup[nodeId].push_back(src);
                NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " recv init pkt from Node " << src);
            }
            else if(depth > m_depth)
            {
                staticConnectBackup[nodeId].push_back(src);
            }
        }
    }   break;
    case UanPacketTag::STATE_PACKET:
    {
        if( sensors.Contains(nodeId) )
        {
            StatePacketHandle(node,tag);
        }
    }   break;
    case UanPacketTag::DATA_PACKET:
    {
        Mac8Address dst = tag.GetDestAddress();
        Mac8Address src = tag.GetSourceAddress();
        uint32_t index = tag.GetPacketIndex();
        if(dst == self && protocal != 3)
        {
            Mac8Address relay = tag.GetRelayAddress();
            if(staticRecieveRecord[nodeId].find(relay) != staticRecieveRecord[nodeId].end())
            {
                staticRecieveRecord[nodeId][relay] ++;
            }
            else
            {
                staticRecieveRecord[nodeId][relay] = 1;
            }

            NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " recv data pkt from Node " << src);

            if(sensors.Contains(nodeId)){
                uint32_t depth = tag.GetDepth();
                uint32_t m_depth = node->GetObject<MobilityModel>()->GetPosition().y;
                if(depth > m_depth){
                    tag.SetRelayAddress(self);
                    if(ChooseNextHop(node, dst))
                    {
                        tag.SetDestAddress(dst);
                        pkt->RemoveAllPacketTags();
                        pkt->AddPacketTag(tag);
                        SendSinglePacket(node,pkt,dst,false);
                    }
                }
            }
        }
        else if(dst == Mac8Address(255) && protocal == 3){
            if(sinks.Contains(nodeId))
            {
                staticRecieveRecordForDBR[nodeId][src][index] = 1;
            }
            if(sensors.Contains(nodeId)){
                tag.SetRelayAddress(self);

                if(ChooseNextHop(node, dst))
                {
                    tag.SetDestAddress(dst);
                    pkt->RemoveAllPacketTags();
                    pkt->AddPacketTag(tag);
                    SendSinglePacket(node,pkt,dst,false);
                    NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " relay data pkt  for Node " << src << " pkt No "<<+index);
                }
            }
        }
    }   break;
    default:
        break;
    }
}

void Experiment::StatePacketHandle(Ptr<Node> node,  UanPacketTag tag)
{
    uint32_t nodeId = node->GetId();
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
        QstateVector[nodeId].push_back(s);
        staticNextHopBackup[nodeId].push_back(src);
    }
    else if(depth > m_depth)
    {
        staticConnectBackup[nodeId].push_back(src);
    }
    
    NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " Node " << self << " recv state pkt  from Node " << src);
}

void Experiment::UpdateAgent()
{
    if(protocal == 1){
        NS_LOG_LOGIC(" UpdataAgent " << Simulator::Now ().GetSeconds () << "s");
        for(NodeContainer::Iterator nodeIter = nodes.Begin(); nodeIter != nodes.End(); nodeIter++)
        {
            Ptr<Node> node = *nodeIter;
            uint32_t nodeId = node->GetId();

            agent[nodeId].NewStateHandle(QstateVector[nodeId]);
        }
    }
}

bool Experiment::ChooseNextHop(Ptr<Node> node, Mac8Address& next)
{
	if(expState == ExpState::CONTROL_STATE){
        return false;
    }
    uint32_t nodeId = node->GetId();
    switch (protocal)
    {
    case 1:{        //QL
        if(agent[nodeId].GetAction(next)){
            return true;
        }
        else{
            if(staticNextHopBackup[nodeId].empty()){
                return false;
            }
            next = staticNextHopBackup[nodeId].front();
            agent[nodeId].nodeAction = next;
            return true;
        }
    } break;
    case 2:{        //AVE
        if(staticNextHopBackup[nodeId].empty()){
            return false;
        }
        if(AVEindex[nodeId] > staticNextHopBackup[nodeId].size()-1 ){
            AVEindex[nodeId] = 0;
            next = staticNextHopBackup[nodeId][AVEindex[nodeId]];
            AVEindex[nodeId] ++;
        }
        else{
        	next = staticNextHopBackup[nodeId][AVEindex[nodeId]];
            AVEindex[nodeId] ++;
        }
        agent[nodeId].nodeAction = next;
        staticActionOtherProt[nodeId].push_back(next);
        return true;
    } break;
    case 3:{        //DBR
        next = Mac8Address(255);
        return true;
    } break;
    case 4:{        //Random
        if(staticNextHopBackup[nodeId].empty()){
            return false;
        }
        next = staticNextHopBackup[nodeId].front();
        staticActionOtherProt[nodeId].push_back(next);
        return true;
    } break;
    default:
        break;
    }
    return false;
}

void Experiment::WriteToFile(std::string filename,std::string filenameA)
{
    std::ofstream file(filename.c_str());
    // file.open();
    if(file.is_open()){
        NS_LOG_LOGIC("Open File");
    }
    for(uint32_t i=0;i<staticSumPacket.size();i++)
    {
        std::string str = std::to_string(staticSumPacket[i]);
        file << str << ",";
    }
    std::string str = std::to_string(staticSumPacket[staticSumPacket.size()-1]);
    file << str ;
    file.close();

    for(NodeContainer::Iterator node = sensors.Begin(); node != sensors.End(); node++)
    {
        uint32_t nodeId = (*node)->GetId();
        std::vector<Mac8Address> action;
        switch (protocal)
        {
        case 1:{
            action = agent[nodeId].staticActionVec;
        } break;
        case 2:{
            action = staticActionOtherProt[nodeId];
        } break;
        case 3:{
            action.push_back(Mac8Address(255));
        } break;
        case 4:{
            action = staticActionOtherProt[nodeId];
        } break;
        default:
            break;
        }
        std::string name = filenameA + std::to_string(nodeId) + std::string(".csv");
        std::ofstream file_a(name.c_str());
        if(file_a.is_open()){
            NS_LOG_LOGIC("Open File");
        }
        for(uint32_t i=0;i<action.size();i++)
        {
            uint8_t id;
            action[i].CopyTo(&id);
            std::string str_a = std::to_string(id);
            file_a << str_a << ",";
        }
        uint8_t id;
        action[action.size()-1].CopyTo(&id);
        std::string str_a = std::to_string(id);
        file_a << str_a ;
        file_a.close();
        }
}

void Experiment::ChangeAgentStage()
{
    NS_LOG_LOGIC(" ChangeS " << Simulator::Now ().GetSeconds () << "s");
    for(NodeContainer::Iterator node = nodes.Begin(); node != nodes.End(); node++)
    {
        uint32_t nodeId = (*node)->GetId();
        //NS_UNUSED(nodeId);
        agent[nodeId].stage = Agent::Stage::TEST;
    }
}

int main (int argc, char *argv[])
{
    Experiment experiment;

    CommandLine cmd;
    cmd.AddValue ("seed", "Random Seed", experiment.randomSeed); 
    cmd.AddValue ("time", "Simulator Time", experiment.simulatorTime);
    cmd.AddValue ("protocal", "Protocal", experiment.protocal);
    cmd.Parse (argc, argv);

    // LogComponentEnable ("UanYXH", LOG_INFO);
    // LogComponentEnable ("UanYXH", LOG_DEBUG);
    // LogComponentEnable ("UanPhyGen", LOG_DEBUG);
    // LogComponentEnable("UanMacAloha", LOG_DEBUG);
    LogComponentEnable("UanYXH", LOG_LOGIC);
    // LogComponentEnable("UanYXHQL", LOG_INFO);

    NS_LOG_LOGIC(" Random Seed:  " << +experiment.randomSeed);
    NS_LOG_LOGIC(" Simulator Time:  " << +experiment.simulatorTime);
    NS_LOG_LOGIC(" Protocal:  " << experiment.protocal);

    experiment.Prepare();
    experiment.ScheduleInit();

    AnimationInterface anim("uan_yxh.xml");
    anim.SetMaxPktsPerTraceFile(6000000);
    experiment.SetAnimNodesSize(anim);
  
    Simulator::Stop (Minutes (experiment.simulatorTime));
    Simulator::Run ();
    Simulator::Destroy ();

    experiment.WriteToFile("total.csv","action_node");
    experiment.TeardownSocket();

    return 0;
}


