/*
 * ClusterAlgorithm.cc
 *
 *  Created on: Mar 5, 2020
 *      Author: veins @JJC
 */

#include "veins/modules/application/ieee80211p/ClusterAlgorithm.h"


using namespace veins;

Define_Module(veins::ClusterAlgorithm);

void ClusterAlgorithm::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength = par("headerLength");
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");
        collectTime = par("collectTime");
        Tch = par("Tch");
        Tbch = par("Tbch");
        Tcm = par("Tcm");
        Tack = par("Tack");

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        CHElectionEvt  = new cMessage("CH election evt",CH_ELECTION_EVT);
        CHABoardcastEvt = new cMessage("CHABoardcast evt",CHA_BOARDCAST_EVT);
        SendReqJoinEvt = new cMessage("Send ReqJoin evt",SEND_REQJOIN_EVT);
        SendACKJoinEvt = new cMessage("send ACKJoin evt",SEND_ACKJOIN_EVT);
        UpdateCMLEvt = new cMessage("Update CML evt",UPDATE_CML_EVT);
        UpdateBCHLEvt = new cMessage("Update BCHL evt",UPDATE_BCHL_EVT);
        CheckMYCHEvt = new cMessage("check connected with myCH evt",CHECK_MYCH_EVT);
        BCHElectionEvt = new cMessage("BCH election evt",BCH_ELECTION_EVT);


        generatedBSMs = 0;
        generatedCHAs = 0;
        generatedReqJoins = 0;
        generatedACKJoins = 0;
        receivedBSMs = 0;
        receivedCHAs = 0;
        receivedReqJoins = 0;
        receivedACKJoins = 0;

        state = UN;
        MCalculateMethod = LLT_BASED_METHOD;

        EV << "Initializing " << par("appName").stringValue() << std::endl;

    }
    else if (stage == 1)
    {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        // start to send beacons

        scheduleAt(simTime()+uniform(0,1.0), sendBeaconEvt);
        scheduleAt(300.0+collectTime,CHElectionEvt);
    }

}
void ClusterAlgorithm::onBSM(DemoSafetyMessage* bsm)
{
    if(state == UN)
    {// UN receives the beacon
        bool isInSNVector = false;
        bool senderIsPosDir;//whether the sender's direction is positive
        bool receiverIsPosDir;//whether the receiver's direction is positive

        //findHost()->getDisplayString().setTagArg("i", 1, "blue");
        EV<<"Received beacon from:"<< bsm->getSenderId()<<std::endl;
        EV<<"sender's position is:"<<bsm->getSenderPos()<<" | sender's velocity is :"<<bsm->getSenderSpeed()<<std::endl;
        EV<<"sender's state is:"<<bsm->getSenderState()<<endl;
        if(bsm->getSenderSpeed().x>0)
        {
            senderIsPosDir = true;
        }else
        {
            senderIsPosDir = false;
        }
        if(curSpeed.x>0)
        {
            receiverIsPosDir = true;
        }else
        {
            receiverIsPosDir= false;

        }
        if((senderIsPosDir==receiverIsPosDir)&&(std::abs(bsm->getSenderSpeed().x-curSpeed.x)<deltaVelocityTH))
        {
            SurroundingVehicle * mySurroundingVehicle =new SurroundingVehicle(bsm->getSenderId(),-1,bsm->getSenderPos(),bsm->getSenderSpeed(),curPosition,curSpeed,false);
            if(SN.size()!=0)//vector is not empty
            {
                for(int j=0;j<SN.size();j++)
                {
                    if(SN[j]->getVehicleId() == bsm->getSenderId())//find this SNmember
                    {
                        isInSNVector = true;
                        SN[j]->updateInfo(bsm->getSenderId(),-1,bsm->getSenderPos(),bsm->getSenderSpeed(),curPosition,curSpeed,false);
                    }
                }
                if(isInSNVector == false)
                {
                    SN.push_back(mySurroundingVehicle);
                }

            }else// vector is empty, the SNmember is not is the vector
            {
                SN.push_back(mySurroundingVehicle);
            }
        }
    }
    else if(state == CH)
    {//check the beacon sender's state and update the CML

        if(bsm->getSenderState()==CM && bsm->getClusterId()==myId)
        {//receive from CM
            bool isInCMLVector = false;
            SurroundingVehicle * mySurroundingCM = new  SurroundingVehicle(bsm->getSenderId(),myId,bsm->getSenderPos(),bsm->getSenderSpeed(),curPosition,curSpeed,true);
            if(CML.size()!=0)
            {
                for(int j=0;j<CML.size();j++)
                {
                    if(CML[j]->getVehicleId()==bsm->getSenderId())
                    {
                        isInCMLVector = true;
                        CML[j]->updateInfo(bsm->getSenderId(), myId, bsm->getSenderPos(),bsm->getSenderSpeed(), curPosition, curSpeed, true);
                    }
                }
                if(isInCMLVector == false)
                {
                    CML.push_back(mySurroundingCM);
                }

            }else
            {
                CML.push_back(mySurroundingCM);
            }
        }

        if(bsm->getSenderState()==CH)
        {//cluster Merge
            bool SenderIsPosDir;//whether the sender's direction is positive
            bool ReceiverIsPosDir;//whether the receiver's direction is positive
            if(bsm->getSenderSpeed().x>0)
            {
                 SenderIsPosDir = true;
             }else
             {
                 SenderIsPosDir = false;
             }
             if(curSpeed.x>0)
             {
                  ReceiverIsPosDir = true;
              }else
             {
                  ReceiverIsPosDir= false;
             }
            if(bsm->getCMNum()>CML.size() && SenderIsPosDir == ReceiverIsPosDir && (std::abs(bsm->getSenderSpeed().x-curSpeed.x)<deltaVelocityTH))
            {

                state = CM;
                clusterId = bsm->getSenderId();
            }

        }


    }
    else if(state == CM or state== CCM)
    {//check the beacon sender's state and update the BCHL
        bool isInBCHLVector = false;
        bool mysenderIsPosDir;//whether the sender's direction is positive
        bool myreceiverIsPosDir;//whether the receiver's direction is positive
        if(bsm->getSenderSpeed().x>0)
        {
             mysenderIsPosDir = true;
        }else
        {
             mysenderIsPosDir = false;
         }
         if(curSpeed.x>0)
        {
             myreceiverIsPosDir = true;
        }else
         {
             myreceiverIsPosDir= false;

         }
        if(bsm->getSenderState()==CH)
        {
            if((mysenderIsPosDir==myreceiverIsPosDir)&&(std::abs(bsm->getSenderSpeed().x-curSpeed.x)<deltaVelocityTH))
            {
                if(BCHL.size()<MAX_BCH)
                {
                    if(bsm->getSenderId()!= clusterId)// receive beacon from other CH
                    {
                        //update the BCHL info
                        SurroundingVehicle * mySurroundingCH = new  SurroundingVehicle(bsm->getSenderId(),bsm->getSenderId(),bsm->getSenderPos(),bsm->getSenderSpeed(),curPosition,curSpeed,true);
                        if(BCHL.size()!=0)
                        {
                            for(int j=0;j<BCHL.size();j++)
                            {
                                if(BCHL[j]->getVehicleId() == bsm->getSenderId())
                                {
                                    isInBCHLVector = true;
                                    BCHL[j]->updateInfo(bsm->getSenderId(), bsm->getSenderId(),bsm->getSenderPos(),bsm->getSenderSpeed(), curPosition, curSpeed, true);
                                }
                            }
                            if(isInBCHLVector == false)
                            {
                                BCHL.push_back(mySurroundingCH);
                            }
                            //BCHL ranking
                        }else
                        {
                            BCHL.push_back(mySurroundingCH);
                        }
                    }
                    else// receive beacon from current CH
                    {
                        isInCluster = true;//confirm the node is still in the cluster
                    }
                }
            }

        }

    }
}

void ClusterAlgorithm::onCHA(CHABoardcastMessage* CHABoardcast)
{

    if(state == UN && CHABoardcast->getSenderState() == CH)
    {
        state = CCM;
        cancelEvent(CHABoardcastEvt);
        clusterId = CHABoardcast->getSenderId();
        scheduleAt(simTime()+uniform(0.01,0.3),SendReqJoinEvt);
        //must add some random time,otherwise all node would send the ReqJoin to CH together, the channel would be busy and blocked
        scheduleAt(simTime()+Tbch,UpdateBCHLEvt);
    }
}

void ClusterAlgorithm::onReqJoin(ReqJoinMessage* ReqJoin){
    EV<<"enter onReqJoin function"<<endl;
    if(state == CH  && ReqJoin->getSenderState() == CCM && ReqJoin->getReceiverId()==myId){//&&
        //add this CCM to the CML;

        currentReqCCMId = ReqJoin->getSenderId();
        scheduleAt(simTime(),SendACKJoinEvt);
    }
    else
    {
        EV<<"drop the reqJoin"<<endl;
        delete(ReqJoin);
    }
}
void ClusterAlgorithm::onACKJoin(ACKJoinMessage* ACKJoin){
    //state = CM;
    EV<<"enter onACKJoin function"<<endl;
    if(state == CCM && ACKJoin->getSenderState()==CH && ACKJoin->getReceiverId()== myId){//&& ACKJoin->getSenderState()==CH && ACKJoin->getTargetCCMId()== myId
        state = CM;
        clusterId = ACKJoin->getSenderId();
        cancelEvent(BCHElectionEvt);
        scheduleAt(simTime()+Tcm,CheckMYCHEvt);

    }
    else
    {
        EV<<"the state is:"<<state<<"   drop the ACKJoin"<<endl;
        delete(ACKJoin);
    }
}

void ClusterAlgorithm::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    EV<<"enter populate funtion"<<endl;
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);


    if(CHABoardcastMessage* CHABoardcast = dynamic_cast<CHABoardcastMessage*>(wsm)){
        EV<<"populate the CHA"<<endl;
        CHABoardcast->setSenderPos(curPosition);
        CHABoardcast->setSenderSpeed(curSpeed);
        CHABoardcast->setSenderId(myId);
        CHABoardcast->setSenderState(state);
        CHABoardcast->setReceiverId(-1);
        CHABoardcast->setReceiverState(UN);
        CHABoardcast->setChannelNumber(static_cast<int>(Channel::cch));
        CHABoardcast->setSNNum(SN.size());
    }
    else if(ReqJoinMessage * ReqJoin = dynamic_cast<ReqJoinMessage*>(wsm))
    {
        EV<<"populate the ReqJoin"<<endl;
        ReqJoin->setSenderPos(curPosition);
        ReqJoin->setSenderSpeed(curSpeed);
        ReqJoin->setSenderId(myId);
        ReqJoin->setSenderState(state);
        ReqJoin->setReceiverId(clusterId);
        ReqJoin->setReceiverState(CH);
        ReqJoin->setChannelNumber(static_cast<int>(Channel::cch));
    }
    else if(ACKJoinMessage * ACKJoin = dynamic_cast<ACKJoinMessage*>(wsm)){
        EV<<"populate the ACKJoin"<<endl;
        ACKJoin->setSenderPos(curPosition);
        ACKJoin->setSenderSpeed(curSpeed);
        ACKJoin->setSenderId(myId);
        ACKJoin->setSenderState(state);
        ACKJoin->setReceiverId(currentReqCCMId);
        ACKJoin->setReceiverState(CCM);
        ACKJoin->setChannelNumber(static_cast<int>(Channel::cch));
    }
    // attention: write new message populate above this
    else if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        EV<<"populate the beacon"<<endl;
            bsm->setSenderPos(curPosition);
            bsm->setSenderSpeed(curSpeed);
            bsm->setSenderId(myId);//sender's MAC address
            bsm->setSenderState(state);
            bsm->setReceiverId(-1);
            bsm->setReceiverState(UN);
            if(state == CM)
            {
                bsm->setClusterId(clusterId);
            }else if(state == CH)
            {
                bsm->setClusterId(myId);
            }
            if(state == CH)
            {
                bsm->setCMNum(CML.size());
            }
            bsm->setChannelNumber(static_cast<int>(Channel::cch));
            bsm->addBitLength(beaconLengthBits);
            bsm->setUserPriority(beaconUserPriority);
        }

}

void ClusterAlgorithm::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}

void ClusterAlgorithm::handlePositionUpdate(cObject* obj)
{

    ChannelMobilityPtrType const themobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = themobility->getPositionAt(simTime());
    curSpeed = themobility->getCurrentSpeed();
    EV<<"car moving, the position is :"<<curPosition<<std::endl;
    EV<<"car moving,  curSpeed is :"<<curSpeed<<std::endl;

    switch(state){
            case CH:
            {
                findHost()->getDisplayString().setTagArg("i", 1, "yellow");
                break;
            }
            case CCM:
            {
                findHost()->getDisplayString().setTagArg("i", 1, "blue");
                break;
            }
            case UN:
            {
                findHost()->getDisplayString().setTagArg("i", 1, "red");
                break;
            }
            case CM:
            {
                findHost()->getDisplayString().setTagArg("i", 1, "green");
                break;
            }

        }


}

void ClusterAlgorithm::handleLowerMsg(cMessage* msg)
{/// have some problem

    DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(msg);
    ASSERT(bsm);
    EV<<"enter the handle lower message function"<<endl;

    if(CHABoardcastMessage *CHABoardcast = dynamic_cast<CHABoardcastMessage*>(bsm)){
        receivedCHAs++;
        EV<<"handle the lower msg: CHA"<<endl;
        onCHA(CHABoardcast);
    }
    else if(ReqJoinMessage * ReqJoin = dynamic_cast<ReqJoinMessage*>(bsm)){
        receivedReqJoins++;
        EV<<"handle the lower msg: reqjoin"<<endl;
        onReqJoin(ReqJoin);
    }
    else if(ACKJoinMessage * ACKJoin = dynamic_cast<ACKJoinMessage*>(bsm)){
        receivedACKJoins++;
        EV<<"handle the lower msg: ACKjoin"<<endl;
        onACKJoin(ACKJoin);
    }
    //attention: write new message handler above this
    else {
       receivedBSMs++;
       EV<<"handle the lower msg: beacon"<<endl;
       onBSM(bsm);
    }

    //delete (msg);
}

void ClusterAlgorithm::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        EV<<"generate beacon and send down"<<endl;
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    case CH_ELECTION_EVT:{
        EV<<"Beacon Collect Time is over,start to find suitable CH"<<endl;
        //cancelEvent(sendBeaconEvt);
        startTimerTw();
        break;
    }
    case CHA_BOARDCAST_EVT:{
        state = CH;
        EV<<"generate CHA Boardcast and send down"<<endl;
        CHABoardcastMessage* CHABoardcast = new CHABoardcastMessage();
        populateWSM(CHABoardcast);
        sendDown(CHABoardcast);
        if(UpdateCMLEvt->isScheduled()== false)
        {
            scheduleAt(simTime()+Tch,UpdateCMLEvt);
        }
        break;
    }
    case SEND_REQJOIN_EVT:{
        EV<<"sending reqJoin message to target CH..."<<endl;
        ReqJoinMessage* ReqJoin = new ReqJoinMessage();
        populateWSM(ReqJoin);
        sendDown(ReqJoin);
        EV<<"already finished send reqJoin message to target CH"<<endl;
        if(BCHElectionEvt->isScheduled() == false)
        {
            scheduleAt(simTime()+Tack,BCHElectionEvt);
        }
        break;
    }
    case SEND_ACKJOIN_EVT:{
        EV<<"send ackJoin message to target CCM"<<endl;
        ACKJoinMessage* ACKJoin = new ACKJoinMessage();
        populateWSM(ACKJoin);
        sendDown(ACKJoin);
        break;
    }
    case UPDATE_CML_EVT:{
        std::vector<SurroundingVehicle *> ::iterator it;
        for(it = CML.begin();it != CML.end();)
        {
            SurroundingVehicle * thisVechile = *it;
            if(thisVechile->getIsConnected()== false)//if lose connected with CM
            {
                it = CML.erase(it);//remove this CM from the CML
            }else
            {
                it++;
            }

        }
        for(int j = 0;j<CML.size();j++)
        {
            CML[j]->setIsConnected(false);//reset the isConnected label
        }
        if(state == CH)
        {
            if(UpdateCMLEvt->isScheduled()==false)
            {
                scheduleAt(simTime()+Tch,UpdateCMLEvt);
            }
        }
        break;
    }
    case UPDATE_BCHL_EVT:{
        std::vector<SurroundingVehicle *> ::iterator iter;
        for(iter = BCHL.begin();iter != BCHL.end();)
        {
            SurroundingVehicle * thatVechile = *iter;
            if(thatVechile->getIsConnected()== false)//if lose connected with CM
            {
                iter = BCHL.erase(iter);
            }else
            {
                iter++;
            }
        }
        for(int j = 0;j<BCHL.size();j++)
        {
            BCHL[j]->setIsConnected(false);//reset the isConnected label
        }
        if(state == CM or state == CCM)
        {
            scheduleAt(simTime()+Tbch,UpdateBCHLEvt);
        }
        break;
    }
    case CHECK_MYCH_EVT:{
        if(state == CM)
        {
            if(isInCluster == false)
            {
                if(BCHElectionEvt->isScheduled() == false)
                {
                    scheduleAt(simTime(),BCHElectionEvt);//lose connect with CH, start BCH Election
                }
            }
            isInCluster = false;// reset the label
            scheduleAt(simTime()+Tcm,CheckMYCHEvt);
        }
        break;
    }
    case BCH_ELECTION_EVT:{
// write algorithm 6
        double MAX_LLT = 0;
        std::vector<SurroundingVehicle *> ::iterator myiter;
        if(BCHL.size()==0)
        {
            if(state == CCM)
            {
                if(BCHElectionEvt->isScheduled() == false){
                    scheduleAt(simTime()+Tbch,BCHElectionEvt);
                }
            }
            if(state==CM)
            {
                state = CH;
                scheduleAt(simTime(),CHABoardcastEvt);
            }
        }
        else
        {
            for(int j =0;j<BCHL.size();j++)
            {//find the maxLLT value in the BCHL
                if(BCHL[j]->getLLT()>MAX_LLT)
                {
                    MAX_LLT = BCHL[j]->getLLT();
                }
            }

            for(myiter=BCHL.begin();myiter!= BCHL.end();)
            {//find the node with the maxLLT value in the BCHL and erase the node from the BCHL after send ReqJoin
                SurroundingVehicle * curVehicle = *myiter;

                if(curVehicle->getLLT()==MAX_LLT)
                {
                    clusterId = curVehicle->getVehicleId();
                    scheduleAt(simTime(),SendReqJoinEvt);
                    myiter=BCHL.erase(myiter);
                }else
                {
                    myiter++;
                }

            }

        }
        break;
    }

    default: {
        if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void ClusterAlgorithm::finish()
{
    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("generatedCHAs",generatedCHAs);
    recordScalar("generatedReqJoins",generatedReqJoins);
    recordScalar("generatedReqJoins",generatedACKJoins);
    recordScalar("receivedBSMs", receivedBSMs);
    recordScalar("receivedCHAs", receivedCHAs);
    recordScalar("receivedReqJoins", receivedReqJoins);
    recordScalar("receivedReqJoins", receivedACKJoins);
}

ClusterAlgorithm::~ClusterAlgorithm()
{
    cancelAndDelete(sendBeaconEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

void ClusterAlgorithm::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void ClusterAlgorithm::checkAndTrackPacket(cMessage* msg)
{
    EV<<"enter check and track packet function"<<endl;

    if(dynamic_cast<CHABoardcastMessage*>(msg)){
        EV << "sending down a CHA" << std::endl;
        generatedCHAs++;
    }
    else if(dynamic_cast<ReqJoinMessage*>(msg)){
        EV << "sending down a ReqJoin message" << std::endl;
        generatedReqJoins++;
    }
    else if(dynamic_cast<ACKJoinMessage*>(msg)){
            EV << "sending down a ACKJoin message" << std::endl;
            generatedACKJoins++;
    }
    else if(dynamic_cast<DemoSafetyMessage*>(msg)) {
        EV << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
}

void ClusterAlgorithm::startTimerTw()
{//when calling this method, make sure the SN vector is not empty;
    double T_min = 0,T_max = 2;
    double M;
    double LLT_max = 200;//simulation time = 200s
    double D_max = 300;//TR = 300m
    double V_max = deltaVelocityTH;//5m/s
    double LLT_sum=0,D_sum=0,V_sum=0;
    double LLT_avg=0,D_avg=0,V_avg=0;
    if(SN.size() != 0)
    {
        switch(MCalculateMethod)
        {
        case LLT_BASED_METHOD:{
            for(int j=0;j<SN.size();j++)
            {
                LLT_sum += SN[j]->getLLT();
            }
            LLT_avg = LLT_sum/SN.size();
            M = std::abs((1-LLT_avg)/LLT_max);
            break;
        }
        case D_BASED_METHOD:{
            for(int j=0;j<SN.size();j++)
            {
                D_sum += SN[j]->getDeltaDistance();
            }
            D_avg = D_sum/SN.size();
            M = D_avg/D_max;
            break;
        }
        case V_BASED_METHOD:{
            for(int j=0;j<SN.size();j++)
            {
                V_sum += SN[j]->getDeltaVelocity();
            }
            V_avg = V_sum/SN.size();
            M = V_avg/V_max;
            break;
        }
        default: {
            EV << "APP: Error: Got MCalculateMethod of unknown kind! " << endl;
            break;
        }
        }
        Tw = T_min+ (T_max-T_min)*M;
        EV<<"the Tw value is:"<<Tw<<endl;
        if(state == UN)
        {
            scheduleAt(simTime() + Tw,CHABoardcastEvt);
        //after receiving the CHA, cancel this event
        }
    }else//T_collect == 0 and does not find Stable Neighbors
    {
        //don't participate in Backoff-based CH Selection
        state = CCM;
    }
}
