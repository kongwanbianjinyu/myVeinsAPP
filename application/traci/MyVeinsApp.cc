//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/MyVeinsApp.h"
#include "veins/modules/application/traci/CHAMessage_m.h"

using namespace veins;

Define_Module(veins::MyVeinsApp);

void MyVeinsApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        MCalculateMethod = LLT_BASED_METHOD;
        state = UN;
        // Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
    }
}

void MyVeinsApp::finish()
{
    DemoBaseApplLayer::finish();
    // statistics recording goes here
}

void MyVeinsApp::onBSM(DemoSafetyMessage* bsm)
{
    bool isInVector = false;
    bool senderIsPosDir;//whether the sender's direction is positive
    bool receiverIsPosDir;//whether the receiver's direction is positive
    // Your application has received a beacon message from another car or RSU
    // code for handling the message goes here
    //findHost()->getDisplayString().setTagArg("i", 1, "blue");
    EV<<"Received beacon from:"<< bsm->getSenderAddress()<<std::endl;
    EV<<"sender's position is:"<<bsm->getSenderPos()<<" | sender's velocity is :"<<bsm->getSenderSpeed()<<std::endl;
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
        SNmember * mySNmember =new SNmember(bsm->getSenderAddress(),bsm->getSenderPos(),bsm->getSenderSpeed(),curPosition,curSpeed);
        if(SN.size()!=0)//vector is not empty
        {
            for(int j=0;j<SN.size();j++)
            {
                if(SN[j]->getMemberId() == bsm->getSenderAddress())//find this SNmember
                {
                    isInVector = true;
                    SN[j]->updateInfo(bsm->getSenderAddress(),bsm->getSenderPos(),bsm->getSenderSpeed(),curPosition,curSpeed);
                }
            }
            if(isInVector == false)
            {
                SN.push_back(mySNmember);
            }

        }else// vector is empty, the SNmember is not is the vector
        {
            SN.push_back(mySNmember);
        }



    }

}

void MyVeinsApp::onWSM(BaseFrame1609_4* wsm)
{
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void MyVeinsApp::onWSA(DemoServiceAdvertisment* wsa)
{
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void MyVeinsApp::onCHA(CHAMessage* CHA)
{
    if(state == UN && isAlreadyOnCHA == false){//this module can only onCHA once..
    //after received the CHABoardcast, it turns out that this module is not CH;
    EV<<"cancel CHABoardcast Event after received the CHA"<<endl;
    cancelEvent(CHABoardcast);//stop to boardCast the CHA
    state = CCM;
    targetCHId = CHA->getSenderAddress();
    //start to sendReqJoin to the CH;
    EV<<"current state:CCM||start to send ReqJoinMessage to CH:"<<targetCHId<<endl;

    scheduleAt(simTime()+uniform(0,1.0),sendReqJoinEvt);

    //start timer T_ack

    scheduleAt(simTime()+ACKTime,ACKTimeExpire);
    isAlreadyOnCHA = true;
    //Algorithom3 cluster Formation
    }else
    {
        delete CHA;
    }
}

void MyVeinsApp::onReqJoin(ReqJoinMessage *ReqJoin)
{
    if((state == CH) && (ReqJoin->getTargetCHId()==myId))//only CH can receive the ReqJoin
    {
        if(CML.size()< MAX_CM)
        {
            CML.push_back(ReqJoin->getSourceCCMId());//add the CCM to the CML
            TargetCCMId = ReqJoin->getSourceCCMId();

            scheduleAt(simTime(),sendACKJoinEvt);

            //send the ACKJoin back to the  CCM
        }
        //if N<= MAX_CM, add the CCM to the CML list and send ACKJoin back to the CCM.
    }else// drop the packet
    {
        delete ReqJoin;
    }
}

void MyVeinsApp::onACKJoin(ACKJoinMessage *ACKJoin)
{
    if((state == CCM)&&(ACKJoin->getTargetCCMId()==myId))
    {
        state = CM;
        EV<<"now the state is CM"<<endl;
        EV<<"myId :"<<myId<<"|| TargetCCMId:"<<ACKJoin->getTargetCCMId()<<endl;
        EV<<"get ACKJoin from:"<< ACKJoin->getSourceCHId()<<endl;
        cancelEvent(ACKTimeExpire);

    }else
    {
        delete ACKJoin;
    }
    //cancel the event::ACKTimeExpire
    //change state to CM,store the CH Id
}

void MyVeinsApp::handleSelfMsg(cMessage* msg)
{
    switch(msg->getKind())
    {
        case SEND_BEACON_END:{
            cancelEvent(sendBeaconEvt);//stop sending beacons
            EV<<"Beacon Collect Time is over,beacon sending stop"<<endl;
            //Algorithm2, When T_collect = 0
            startTimerTw();
            break;
        }
        case CHA_BOARDCAST:{
            state = CH;
            //boardCast the CHA, CHA extends from WSM, defined by myself
            EV<<"start to boardCast a CHA"<<endl;
            CHAMessage *CHA = new CHAMessage();
            EV<<"start to populateMyMessage"<<endl;
            populateMyMessage(CHA);//populateCHA
            EV<<"start to sendDown CHA"<<endl;
            sendDown(CHA);//sendDown
            break;
        }
        case SEND_REQJOIN_EVT:{

            EV<<"start to send ReqJoin to the CH"<<endl;
            ReqJoinMessage* ReqJoin = new ReqJoinMessage();
            populateMyMessage(ReqJoin);
            sendDown(ReqJoin);
            break;

        }
        case SEND_ACKJOIN_EVT:{
            ACKJoinMessage *ACKJoin = new ACKJoinMessage();
            populateMyMessage(ACKJoin);
            sendDown(ACKJoin);
            EV<<"state:CH||send ACKJoinMessage to CCM"<<endl;
            break;
        }

        case ACK_TIME_EXPIRE:{
            EV<<"time is expired"<<endl;
            ////go to BCH election
            break;
        }
        default:{
            DemoBaseApplLayer::handleSelfMsg(msg);
            // this method is for self messages (mostly timers)
            // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
        }
    }
}

void MyVeinsApp::handlePositionUpdate(cObject* obj)
{

    DemoBaseApplLayer::handlePositionUpdate(obj);
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
    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}

void MyVeinsApp::startTimerTw()
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
            scheduleAt(simTime() + Tw +uniform(0.01,0.1),CHABoardcast);
        //after receiving the CHA, cancel this event
        }
    }else//T_collect == 0 and does not find Stable Neighbors
    {
        //don't participate in Backoff-based CH Selection
        state = CCM;
    }
}

void MyVeinsApp::populateMyMessage(BaseFrame1609_4* wsm)
{
    if(CHAMessage *CHA = dynamic_cast<CHAMessage*>(wsm))
        {
        // all information obtained in the BaseFrame1609_4 and CHAMessage need to be assigned
            CHA->setSenderAddress(myId);
            CHA->setCurPosition(curPosition);
            CHA->setCurSpeed(curSpeed);
            CHA->setSNNum(SN.size());
            CHA->setPsid(-1);
            CHA->setChannelNumber(static_cast<int>(Channel::cch));//must assign a channel

        }
    if(ReqJoinMessage * ReqJoin = dynamic_cast<ReqJoinMessage*>(wsm))
    {
        ReqJoin->setTargetCHId(targetCHId);//set the target CH Id,get from the CHA;
        ReqJoin->setSourceCCMId(myId);
        ReqJoin->setPsid(-1);
        ReqJoin->setChannelNumber(static_cast<int>(Channel::cch));

    }
    if(ACKJoinMessage * ACKJoin = dynamic_cast<ACKJoinMessage*>(wsm))
    {
        ACKJoin->setSourceCHId(myId);//set the source CH Id;
        ACKJoin->setTargetCCMId(TargetCCMId);//set the target CCM, get from the ReqJoin
        ACKJoin->setPsid(-1);
        ACKJoin->setChannelNumber(static_cast<int>(Channel::cch));
    }
}
