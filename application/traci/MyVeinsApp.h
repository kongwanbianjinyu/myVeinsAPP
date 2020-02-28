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

#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

#include <vector>
#include "SNmember.h"

using namespace omnetpp;


namespace veins {

/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

class VEINS_API MyVeinsApp : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    void finish() override;

protected:
    void onBSM(DemoSafetyMessage* bsm) override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onCHA(CHAMessage* CHA)override;
    void onReqJoin(ReqJoinMessage *ReqJoin) override;
    void onACKJoin(ACKJoinMessage *ACKJoin) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    //functions defined by myself
    void startTimerTw();// UN calculate the M value,then start the timer(schedule an event)
    void populateMyMessage(BaseFrame1609_4* wsm);
private:
    enum STATE
    {
        UN,
        CCM,
        CM,
        CH
    }state;
    std::vector<SNmember *> SN;
    double deltaVelocityTH = 5;
    enum MCALCULATE
    {
        LLT_BASED_METHOD,
        D_BASED_METHOD,
        V_BASED_METHOD
    }MCalculateMethod;
    simtime_t Tw;
    LAddress::L2Type targetCHId = -1;
    int MAX_CM = 10;
    std::vector<LAddress::L2Type> CML;
    LAddress::L2Type TargetCCMId = -1;
    bool isAlreadyOnCHA = false;
};//class

} // namespace veins
