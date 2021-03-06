#ifndef UAN_YXH_QL_H
#define UAN_YXH_QL_H

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
    uint32_t threshBand = 30;

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

    std::map<State, double> QTable;
    
    Stage stage = Stage::INIT;
    State nodeState;
    Mac8Address nodeAction;
    std::vector<Mac8Address> nodeActionVec;
    bool hasAction = false;
    uint32_t actionVecindex;

    double alpha = 0.7;
    double gamma = 0.1;
    double rewardBase = 2;
};

#endif  /*  UAN_YXH_QL_H  */