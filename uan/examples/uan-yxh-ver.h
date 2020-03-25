#ifndef UAN_YXH_VER_H
#define UAN_YXH_VER_H

#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/uan-module.h"

#include "uan-yxh-ql.h"

using namespace ns3;

class Experiment
{
public:
    Experiment();

    enum ExpState
    {
        INIT_STATE = 1,
        TRAIN_STATE = 2,
        COMMON_STATE = 3,
    };

    void Prepare();

    void SetNodes();

    void SetMobility();

    void SetPhy();

    void SetLinks();

    void UpdateMobility();

    void SetApplications();

    void RecvPacketCallback(Ptr<Socket> socket);

    void SendPacketCallback(Ptr<Socket> socket, uint32_t);

    void ScheduleSendData();

    void ScheduleInit();

    void ScheduleBeacon();

    void SendSinglePacket(Ptr<Node> node, Ptr<Packet> pkt, Mac8Address dst);

    void SendBroadCastPacket(Ptr<Node> node, Ptr<Packet> pkt);

    void SendHandle(Ptr<Node> node);

    void RecvHandle(Ptr<Socket>,Ptr<Packet>);

    void StatePacketHandle(Ptr<Node> node,  UanPacketTag tag);

    void UpdateAgent();

    bool ChooseNextHop(Ptr<Node> node, Mac8Address& next);

    void TeardownSocket();

    int nodeNums;
    int sinkNums;
    int sensorNums;
    int dataRate;
    int cwMin;
    double gapDataData;
    double gapStateData;
    double gapStateAgent;
    double gapStateState;

    std::map<Ptr<Node>, Ptr<Socket> > sockets;
    std::map<Ptr<Node>, std::deque<Ptr<Packet>> >  sendPktBuff;
    std::map<Ptr<Node>, std::deque<Mac8Address> >  sendAddrBuff;
    std::map<Ptr<Node>, uint32_t> sendFailCnt;
    std::map<Ptr<Node>, Time> txDelay;
    std::map<Ptr<Node>, uint32_t> statePktIndex;
    std::map<Ptr<Node>, uint32_t> dataPktIndex;
    std::map<Ptr<Node>, std::map<Mac8Address,uint32_t>> recieveRecord;
    std::map<Ptr<Node>, std::vector<Mac8Address>> nextHopBackup;
    std::map<Ptr<Node>, Mac8Address> nextHopAddr;
    std::map<Ptr<Node>, ExpState> expState;

    std::map<Ptr<Node>, std::vector<State>>  QstateVector;
    std::map<Ptr<Node>, Agent> agent;

    Time slotTime;
    EventId eventSendDataID; 

    NodeContainer nodes;
    NodeContainer sinks;
    NodeContainer sensors;

    std::vector<uint32_t> staticSumPacket;
    std::map<Ptr<Node>, std::vector<uint8_t>> staticConnect;
    std::map<Ptr<Node>, std::vector<uint32_t>> staticBandwidth;
    std::map<Ptr<Node>, uint32_t> staticSendNum;

    UanHelper uan;

};

#endif /* UAN_YXH_VER_H */