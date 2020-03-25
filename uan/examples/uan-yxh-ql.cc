#include "uan-yxh-ql.h"

using namespace ns3;

State::State(){}


State::State(Mac8Address i, uint8_t c, uint32_t d, uint32_t b)
{
    i.CopyTo(&id);

    connect = c;

    if(d <= threshDis){
        distance = DisType::SHORT;
    }
    // else if(d <= threshDis*2){
    //     distance = DisType::MEDIAN;
    // }
    else{
        distance = DisType::LONG;
    }
    if(b <= -1.5*threshBand){
        bandwidth = BandType::OVERFIT;
    }
    else if(b <= threshBand){
        bandwidth = BandType::FIT;
    }
    // else if( b <= threshBand*2){
    //     bandwidth = BandType::SMALLCONGEST;
    // }
    else if( b <= threshBand*3){
        bandwidth = BandType::MEDIANCONGEST;
    }
    else{
        bandwidth = BandType::LARGECONGEST;
    }
}

bool State::operator==(const State &s) const
{
    if(connect != s.connect)
        return false;
    if(distance != s.distance )
        return false;
    if(bandwidth != s.bandwidth )
        return false;
    return true;    
}

bool State::operator < (const State &s) const
{
    if(connect < s.connect || (connect == s.connect && distance < s.distance) || (connect == s.connect && distance == s.distance && bandwidth < s.bandwidth)){
        return true;
    }
    else{
        return false;
    }
}

void Agent::NewStateHandle(std::vector<State> QstateVec)
{
    if(QstateVec.empty())
    {
        hasAction = false;
        return;
    }

    switch (stage)
    {
    case Stage::INIT:
    {
        nodeState = QstateVec.back();
        for(std::vector<State>::iterator iter = QstateVec.begin(); iter != QstateVec.end(); iter++)
        {
            if(iter->distance == State::DisType::SHORT)
            {
                nodeState = (*iter);
                break;
            }
        }
        nodeAction = Mac8Address(nodeState.id);
        hasAction = true;

        stage = Stage::TRAIN;
    } break;
    case Stage::TRAIN:
    {
        Step data;
        data.state = nodeState;
        data.action = true;
        data.stateNextVec = QstateVec;
        data.reward = GetReward(nodeState, QstateVec);
        UpdataQ(data);
        ChooseAction(QstateVec);
    } break;
    case Stage::TEST:
    {
        ChooseAction(QstateVec);
    } break;
    default:
        break;
    }
}

void Agent::ChooseAction(std::vector<State> QstateVec)
{
    switch (stage)
    {
    case Stage::TRAIN:
    {
        double minQ = INT_MAX;
        double maxQ = INT_MIN;
        for(std::vector<State>::iterator iter = QstateVec.begin(); iter != QstateVec.end(); iter++)
        {
            State one = *iter;
            if(QTable.find(one) == QTable.end()){
                QTable[one] = 0;
            }
            if(QTable[one] >= maxQ){
                maxQ = QTable[one];
            }
            if(QTable[one] <= minQ){
                minQ = QTable[one];
            }
        }
        // calculate the base for prob:  (max-base) / (min-base) = (max - min) / rewardbase * coef
        double k = 3;  // coef
        //double base = ( (maxQ - minQ) * k * minQ - rewardBase * maxQ ) / ( (maxQ - minQ) * k - rewardBase );
        double base = minQ - rewardBase/k;
        std::vector<double> prob;
        double diffSum = 0;
        for(std::vector<State>::iterator iter = QstateVec.begin(); iter != QstateVec.end(); iter++)
        {
            State one = *iter;
            double diff = QTable[one] - base;
            prob.push_back(diff);
            diffSum += diff;
        }
        if(diffSum == 0){
            prob[0] = 1.0 / prob.size();
            for(uint i=1; i<prob.size(); i++)
            {
                prob[i] = 1.0 / prob.size() + prob[i-1];
            }
        }
        else{
            prob[0] = prob[0] / diffSum;
            for(uint i=1; i<prob.size(); i++)
            {
                prob[i] = prob[i] / diffSum + prob[i-1];
            }
        }
        Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable> ();
        double randomProb = uniformRandomVariable->GetValue (0, 1);
        uint indexChoose = 0;
        for(; indexChoose < prob.size(); indexChoose ++){
            if(prob[indexChoose] > randomProb){
                break;
            }
        }
        if(indexChoose != prob.size()){
            nodeAction = Mac8Address(QstateVec[indexChoose].id);
            nodeState = QstateVec[indexChoose];
        }
        else{
            nodeAction = Mac8Address(QstateVec[indexChoose-1].id);
            nodeState = QstateVec[indexChoose-1];
        }
        hasAction = true;

    } break;
    case Stage::TEST:
    {
        nodeActionVec.clear();
        hasAction = true;
        actionVecindex = 0;
        if(QstateVec.size()==1){
            nodeActionVec.push_back(Mac8Address(QstateVec[0].id));
        }
        else{
            double maxQ2 = INT_MIN;
            State maxS2;
            double maxQ1 = INT_MIN;
            State maxS1;
            for(std::vector<State>::iterator iter = QstateVec.begin(); iter != QstateVec.end(); iter++)
            {
                State one = *iter;
                if(QTable.find(one) != QTable.end()){
                    if(QTable[one] >= maxQ1){
                        maxQ2 = maxQ1;
                        maxS2 = maxS1;
                        maxQ1 = QTable[one];
                        maxS1 = one;
                    }
                    else if(QTable[one] > maxQ2){
                        maxQ2 = QTable[one];
                        maxS2 = one;
                    }
                }
            }
            if(maxQ2 ==  maxQ1){
                nodeActionVec.push_back(Mac8Address(maxS1.id));
                nodeActionVec.push_back(Mac8Address(maxS2.id));
            }
            else if(maxQ2 > 0 && maxQ1 > 0){
                int32_t p = maxQ1/maxQ2 * 10;
                for(int32_t i = 0;i<10;i++)
                {
                    nodeActionVec.push_back(Mac8Address(maxS1.id));
                    nodeActionVec.push_back(Mac8Address(maxS2.id));
                }
                for(int32_t i = 0;i<(p-10);i++)
                {
                    nodeActionVec.push_back(Mac8Address(maxS1.id));
                }
            }
            else if(maxQ2 ==0 && maxQ1 > 0){
                nodeActionVec.push_back(Mac8Address(maxS1.id));
            }
            else{
                nodeActionVec.push_back(Mac8Address(maxS1.id));
            }
        }
    } break;
    default:
        break;
    }
}

bool Agent::GetAction(Mac8Address& addr)
{
    switch (stage)
    {
    case Stage::INIT:
    {
        return false;
    } break;
    case Stage::TRAIN :
    {
        addr = nodeAction;
        return hasAction;
    }  break;
    case Stage::TEST :
    {
    	if(nodeActionVec.empty()){
    		return false;
    	}
        addr = nodeActionVec[actionVecindex];
        actionVecindex++;
        if(actionVecindex == nodeActionVec.size()){
            actionVecindex = 0;
        }
        return hasAction;
    }  break;
    default:
        break;
    }
    return false;
}

int32_t Agent::GetReward(State old, std::vector<State> nextVec)
{
    State newState = nextVec.front();
    for( std::vector<State>::iterator iter  = nextVec.begin(); iter != nextVec.end(); iter++)
    {
        if(iter->id == nodeState.id){
            newState = *iter;
            break;
        }
    }
    if(newState.id == old.id){
        if(newState.bandwidth == State::BandType::FIT || newState.bandwidth == State::BandType::OVERFIT){
            return 2*rewardBase;
        }
        else if(newState.bandwidth < old.bandwidth){
            return rewardBase;
        }
        else if(newState.bandwidth == old.bandwidth){
           for( std::vector<State>::iterator iter  = nextVec.begin(); iter != nextVec.end(); iter++)
            {
                if(iter->id != nodeState.id && iter->bandwidth < old.bandwidth){
                    return -2*rewardBase;
                }
            }
            return 0;
        }
        else{
            return -2*rewardBase;
        }
    }
    else{
        return -2*rewardBase;
    }
}

void Agent::UpdataQ(Step data)
{
    if(QTable.find(data.state) == QTable.end()){
        QTable[data.state] = 0;
    }
    double oldQ = QTable[data.state];
    double maxNextQ = CalcMaxNextQ(data.stateNextVec);

    double newQ = oldQ + alpha*(data.reward + gamma*maxNextQ - oldQ);
    QTable[data.state] = newQ;
}

double Agent::CalcMaxNextQ(std::vector<State> stateVec)
{
    double maxQ = INT_MIN;
    bool found = false;
    NS_UNUSED(found);
    for(std::vector<State>::iterator iter = stateVec.begin(); iter != stateVec.end(); iter++)
    {
        State one = *iter;
        if(QTable.find(one) != QTable.end()){
            if(QTable[one] >= maxQ){
                maxQ = QTable[one];
            }
        }
    }
    maxQ = (maxQ > 0)?maxQ:0;

    return maxQ;
}
