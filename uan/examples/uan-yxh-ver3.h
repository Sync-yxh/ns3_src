#ifndef UAN_YXH_VER3_H
#define UAN_YXH_VER3_H

#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/uan-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

class Experiment
{
public:
    Experiment();

    enum ExpState
    {
        CONTROL_STATE = 1,
        DATA_STATE = 2,
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

    void SendSinglePacket(Ptr<Node> node, Ptr<Packet> pkt, Mac8Address dst, bool flag);

    void SendBroadCastPacket(Ptr<Node> node, Ptr<Packet> pkt);

    void SendHandle(Ptr<Node> node);

    void RecvHandle(Ptr<Socket>,Ptr<Packet>);

    void StatePacketHandle(Ptr<Node> node,  UanPacketTag tag);

    void UpdateAgent();

    bool ChooseNextHop(Ptr<Node> node, Mac8Address& next);

    void TeardownSocket();

    void SetAnimNodesSize(AnimationInterface& anim);

    void WriteToFile(std::string filename,std::string filenameA);

    void ChangeAgentStage();

    void CheckRemainEnergy();

    uint32_t nodeNums;
    uint32_t sinkNums;
    uint32_t sensorNums;
    uint32_t dataRate;
    uint32_t dataPktSize;

    uint32_t simulatorTime;
    double gapDataData;
    double gapStateData;
    double gapStateAgent;
    double gapStateState;
    double gapTrainTest;
    double gapUpdateMob;

    std::map<Ptr<Node>, Ptr<Socket> > sockets;
    std::map<Ptr<Node>, std::deque<Ptr<Packet>> >  sendPktBuff;
    std::map<Ptr<Node>, std::deque<Mac8Address> >  sendAddrBuff;
    std::map<Ptr<Node>, Time> txDelay;
    std::map<Ptr<Node>, uint32_t> statePktIndex;
    std::map<Ptr<Node>, uint32_t> dataPktIndex;

    std::map<uint32_t, std::vector<State>>  QstateVector;
    std::map<uint32_t, Agent> agent;

    std::vector<uint32_t> staticSumPacket;
    std::map<uint32_t, std::vector<uint8_t>> staticConnect;
    std::map<uint32_t, std::vector<uint32_t>> staticBandwidth;
    std::map<uint32_t, uint32_t> staticSendNum;
    std::map<uint32_t, uint32_t> staticSendFailCnt;
    std::map<uint32_t, std::map<Mac8Address,uint32_t>> staticRecieveRecord;
    std::map<uint32_t, std::vector<Mac8Address>> staticNextHopBackup;
    std::map<uint32_t, std::vector<Mac8Address>> staticConnectBackup;

    std::map<uint32_t, uint8_t> staticRemainEnergy;

    EventId eventSendDataID; 
    ExpState expState;

    NodeContainer nodes;
    NodeContainer sinks;
    NodeContainer sensors;

    UanHelper uan;
    uint32_t randomSeed;
    uint32_t protocal;

    struct bound
    {
        double upperX;
        double lowerX;
        double upperY;
        double lowerY;
    };

    struct speed
    {
        double X;
        double Y;
    };

    std::map<uint32_t, speed> velocity;
    std::map<uint32_t, bound> boundary;

    std::map<uint32_t,uint32_t> AVEindex;
    std::map<uint32_t, std::vector<Mac8Address> > staticActionOtherProt;
    std::map<uint32_t, std::map<Mac8Address,std::map<uint32_t,uint32_t> > > staticRecieveRecordForDBR;

};

#endif /* UAN_YXH_VER_H */
