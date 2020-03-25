#ifndef UAN_YXH_H
#define UAN_YXH_H

#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/uan-module.h"

using namespace ns3;

class Protocol;

class Experiment
{
public:
    Experiment();

    void Prepare();

    void SetNodes();

    void SetMobility();

    void SetPhy();

    void SetLinks();

    void UpdateMobility();

    void SetApplications();

    void RecvPacketCallback(Ptr<Socket> socket);

    void SendPacketCallback(Ptr<Socket> socket, uint32_t);

    void ScheduleSendPacket();

    void SendSinglePacket(Ptr<Node> node, Ptr<Packet> pkt, Mac8Address dst);

    void SendBroadCastPacket(Ptr<Node> node, Ptr<Packet> pkt);

    void TeardownSocket();

    int nodeNums;
    int dataRate;
    int cwMin;
    std::map<Ptr<Node>, Ptr<Socket> > sockets;

    Time slotTime;

    NodeContainer  nodes;
    UanHelper uan;

};

class Protocol
{
public:
    Protocol();
    static void HandleRecv(Ptr<Socket>,Ptr<Packet>,Experiment*);
};
#endif /* UAN_YXH_H */