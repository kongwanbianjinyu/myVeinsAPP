/*
 * ClusterAlgorithm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: veins
 */

//
// Copyright (C) 2016 David Eckhoff <eckhoff@cs.fau.de>
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

#pragma once

#include <map>

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/DemoBaseApplLayerToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

#include <vector>
#include "SurroundingVehicle.h"

#include "veins/modules/application/ieee80211p/CHABoardcastMessage_m.h"
#include "veins/modules/application/ieee80211p/ReqJoinMessage_m.h"
#include "veins/modules/application/ieee80211p/ACKJoinMessage_m.h"


namespace veins {

using veins::AnnotationManager;
using veins::AnnotationManagerAccess;
using veins::TraCICommandInterface;
using veins::TraCIMobility;
using veins::TraCIMobilityAccess;

/**
 * @brief
 * ClusterAlgorithm .
 *
 * @author JJC
 *
 * @ingroup applLayer
 *
 * @see DemoBaseApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class VEINS_API ClusterAlgorithm : public BaseApplLayer {

public:
    ~ClusterAlgorithm() override;
    void initialize(int stage) override;
    void finish() override;

    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

    enum DemoApplMessageKinds {
        SEND_BEACON_EVT,
        CH_ELECTION_EVT,
        CHA_BOARDCAST_EVT,
        SEND_REQJOIN_EVT,
        SEND_ACKJOIN_EVT,
        UPDATE_CML_EVT,
        UPDATE_BCHL_EVT,
        CHECK_MYCH_EVT,
        BCH_ELECTION_EVT
    };

protected:
    /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
    void handleLowerMsg(cMessage* msg) override;

    /** @brief handle self messages */
    void handleSelfMsg(cMessage* msg) override;

    /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
    virtual void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId = LAddress::L2BROADCAST(), int serial = 0);

    /** @brief this function is called every time the vehicle receives a position update signal */
    virtual void handlePositionUpdate(cObject* obj);

    /**
     * @brief overloaded for error handling and stats recording purposes
     *
     * @param msg the message to be sent. Must be a WSM/BSM/WSA
     */
    virtual void sendDown(cMessage* msg);

    /**
     * @brief helper function for error handling and stats recording purposes
     *
     * @param msg the message to be checked and tracked
     */
    virtual void checkAndTrackPacket(cMessage* msg);

    /** @brief this function is called upon receiving a DemoSafetyMessage, also referred to as a beacon  */
    void onBSM(DemoSafetyMessage* bsm);

    /** @brief this function enable UN calculate the M value,then start the timer(schedule an event) */
    void startTimerTw();

    /** @brief this function is called upon receiving a CHA */
    void onCHA(CHABoardcastMessage* CHABoardcast);

    /** @brief this function is called upon receiving a ReqJoin */
    void onReqJoin(ReqJoinMessage* ReqJoin);

    /** @brief this function is called upon receiving a ReqJoin */
    void onACKJoin(ACKJoinMessage* ACKJoin);

protected:
    /* pointers ill be set when used with TraCIMobility */
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;

    AnnotationManager* annotations;
    DemoBaseApplLayerToMac1609_4Interface* mac;

    /* BSM (beacon) settings */
    uint32_t beaconLengthBits;
    uint32_t beaconUserPriority;
    simtime_t beaconInterval;
    simtime_t collectTime;


    /* state of the vehicle */
    Coord curPosition;
    Coord curSpeed;
    LAddress::L2Type myId = 0;
    LAddress::L2Type clusterId = -1;//CCM or CM 's cluster ID
    LAddress::L2Type currentReqCCMId = -1;//CH record current reqCCM's ID
    int mySCH;


    /* stats */
    uint32_t generatedBSMs;
    uint32_t generatedCHAs;
    uint32_t generatedReqJoins;
    uint32_t generatedACKJoins;
    uint32_t receivedBSMs;
    uint32_t receivedCHAs;
    uint32_t receivedReqJoins;
    uint32_t receivedACKJoins;

    /* messages for periodic events such as beacon and WSA transmissions */
    cMessage* sendBeaconEvt;
    cMessage* CHElectionEvt;
    cMessage* CHABoardcastEvt;
    cMessage* SendReqJoinEvt;
    cMessage* SendACKJoinEvt;
    cMessage* UpdateCMLEvt;
    cMessage* UpdateBCHLEvt;
    cMessage* CheckMYCHEvt;
    cMessage* BCHElectionEvt;

    /*data structure for recording Surrounding Vehicle's information */
    std::vector<SurroundingVehicle *> SN;
    std::vector<SurroundingVehicle *> CML;
    std::vector<SurroundingVehicle *> BCHL;

    double deltaVelocityTH = 5;

    enum STATE
        {
            UN,//0
            CCM,//1
            CM,//2
            CH//3
        }state;
    enum MCALCULATE
    {
       LLT_BASED_METHOD,
       D_BASED_METHOD,
       V_BASED_METHOD
    }MCalculateMethod;
    simtime_t Tw;
    simtime_t Tch;
    simtime_t Tbch;
    simtime_t Tcm;
    simtime_t Tack;

    bool isInCluster = false;
    int MAX_BCH = 2;


};

} // namespace veins

