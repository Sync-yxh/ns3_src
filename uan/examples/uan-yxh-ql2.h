#ifndef UAN_YXH_QL2_H
#define UAN_YXH_QL2_H

#include "ns3/uan-module.h"

using namespace ns3;

class State
{
public:
    State();
    State(Mac8Address id, uint8_t connect, uint32_t distance, uint32_t bandwidth);

    enum DisType{
        SHORT = 1,
        MEDIAN = 2,
        LONG = 3,
    };
    enum BandType{
        OVERFIT = 1,
        FIT = 2,
        SMALLCONGEST = 3,
        MEDIANCONGEST = 4,
        LARGECONGEST = 5,
    };

    uint8_t id;
    uint8_t connect;
    DisType distance;
    BandType bandwidth;

    uint32_t threshDis = 1300;
    uint32_t threshBand = 20;

    bool operator == ( const State &s ) const;

    bool operator < ( const State &s ) const; 
};

class Step
{
public:
    State state;
    bool action;
    std::vector<State> stateNextVec;
    double reward;
};

class Agent
{
public:

    enum Stage{
        INIT = 1,
        TRAIN = 2,
        TEST = 3,
    };

    bool GetAction(Mac8Address& addr);

    void ChooseAction(std::vector<State> QstateVec);

    void UpdataQ(Step);

    double CalcMaxNextQ(std::vector<State> stateVec);

    void NewStateHandle(std::vector<State>); 

    int32_t GetReward(State old, std::vector<State> nextVec);

    void CheckNewState(std::vector<State> stateVec);

    std::map<State, double> QTable;
    
    Stage stage = Stage::INIT;
    State nodeState;
    Mac8Address nodeAction;
    std::vector<Mac8Address> nodeActionVec;
    bool hasAction = false;
    uint32_t actionVecindex;

    std::vector<Mac8Address> staticActionVec;

    double alpha = 0.7;
    double gamma = 0.3;
    double rewardBase = 2;
    double k = 1;

    uint32_t m_TrainCnt = 0;
    uint32_t maxTrainCnt = 220;
    uint32_t unitTrainCntForTrainStage = 5;
    uint32_t unitTrainCntForTestStage = 30;
};

#endif  /*  UAN_YXH_QL2_H  */