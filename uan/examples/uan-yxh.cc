#include "uan-yxh.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/stats-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/energy-source-container.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UanYXH");

Experiment::Experiment(): 
    nodeNums(3),
	dataRate(10240),
    cwMin(10),
    slotTime(Seconds(0.2))
{}

void Experiment::SetNodes()
{
    nodes = NodeContainer();
    nodes.Create(nodeNums);

    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
    energySourceHelper.Install (nodes);
    
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
    nodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (150, 0, 0));
    nodes.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (150, 100, 0));
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
        sockets[*node]->SetSendCallback (MakeCallback (&Experiment::SendPacketCallback, this));
        node++;
    }
}

void Experiment::ScheduleSendPacket()
{
    Mac8Address dst = Mac8Address::ConvertFrom(nodes.Get(2)->GetDevice(0)->GetAddress());

    Ptr<Packet> pkt = Create<Packet> (10240);  // payload size : 1024 byte
    UanPacketTag tag;
    tag.SetPacketType(UanPacketTag::DATA_PACKET);
    tag.SetDestAddress(dst);
    tag.SetSourceAddress(Mac8Address::ConvertFrom(nodes.Get(1)->GetDevice(0)->GetAddress()));
    pkt->AddPacketTag(tag);

    Simulator::Schedule (Seconds (10), &Experiment::SendSinglePacket, this, nodes.Get(1), pkt, dst);
    Simulator::Schedule (Seconds (10.01), &Experiment::SendSinglePacket, this, nodes.Get(1), pkt, dst);
    Simulator::Schedule (Seconds (10.02), &Experiment::SendSinglePacket, this, nodes.Get(1), pkt, dst);
    Simulator::Schedule (Seconds (29), &Experiment::SendSinglePacket, this, nodes.Get(1), pkt, dst);

    // Simulator::Schedule (Seconds (10), &Experiment::SendBroadCastPacket, this, nodes.Get(1), pkt);

    // Simulator::Schedule (Seconds (20), &Experiment::UpdateMobility, this);

    // Simulator::Schedule (Seconds (100), &Experiment::ScheduleSendPacket, this);
}

void Experiment::SendSinglePacket(Ptr<Node> node, Ptr<Packet> pkt, Mac8Address dst)
{
    PacketSocketAddress socketAddress;
    socketAddress.SetSingleDevice (node->GetDevice (0)->GetIfIndex ());
    socketAddress.SetPhysicalAddress (dst);
    socketAddress.SetProtocol (0);
    if(sockets.find(node) == sockets.end()){
        NS_LOG_INFO("Not Found node in sockets");
        return;
    }
    sockets[node]->SendTo (pkt, 0, socketAddress);
    NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " packet sent to " << dst );
    // Ptr<NetDevice> device = node->GetDevice (socketAddress.GetSingleDevice ());

    // if(!device->IsPhyTx())
    // {
    // 	sockets[node]->SendTo (pkt, 0, socketAddress);
    // 	NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " packet sent to " << dst );
    // }
    // else
    // {
    // 	NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " packet drop" );
    // }
    
}

void Experiment::SendBroadCastPacket(Ptr<Node> node, Ptr<Packet> pkt)
{
    if(sockets.find(node) == sockets.end()){
        NS_LOG_INFO("Not Found node in sockets");
        return;
    }
    sockets[node]->Send(pkt);
    NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " packet sent broadcast " );
}

void Experiment::RecvPacketCallback(Ptr<Socket> skt)
{
    Address srcAddress;
    while (skt->GetRxAvailable () > 0)
    {
        Ptr<Packet> packet = skt->RecvFrom (srcAddress);
        Protocol::HandleRecv(skt,packet,this);
    }
}

void Experiment::SendPacketCallback(Ptr<Socket> skt, uint32_t available)
{
    NS_LOG_INFO( "SendCallback Called, availsize: " << available);
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

void Experiment::TeardownSocket ()
{
    std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;

    for (socket = sockets.begin (); socket != sockets.end (); socket++)
    {
    socket->second->Close ();
    }
}

void Experiment::Prepare()
{
    SetNodes();
    SetMobility();
    SetPhy();
    SetLinks();
    SetApplications();
}

void Protocol::HandleRecv(Ptr<Socket> socket,Ptr<Packet> pkt, Experiment* exp)
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
    Mac8Address dst = tag.GetDestAddress();
    Mac8Address src = tag.GetSourceAddress();
    Mac8Address self = Mac8Address::ConvertFrom(socket->GetNode()->GetDevice(0)->GetAddress());
    if(dst != self && src != self){
        pkt->RemoveAllPacketTags();
        pkt->AddPacketTag(tag);
        exp->SendSinglePacket(socket->GetNode(),pkt,dst);
        NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " packet relay to " << dst  << " by " << self);
    }
    else if(dst == self)
    {
        uint32_t pktSize =  pkt->GetSize();
        NS_LOG_INFO( "Time: " << Simulator::Now ().GetSeconds () << "s" << " node " << self << " recieve " << pktSize << " byte " );
    }
}

int main (int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse (argc, argv);

    LogComponentEnable ("UanYXH", LOG_INFO);

    Experiment experiment;

    experiment.Prepare();
    experiment.ScheduleSendPacket();

    AnimationInterface anim("uan_yxh.xml");
  
    Simulator::Stop (Minutes (30));
    Simulator::Run ();
    Simulator::Destroy ();

    experiment.TeardownSocket();

    return 0;
}


