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

using namespace veins;

Define_Module(veins::MyVeinsApp);

void MyVeinsApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
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
    findHost()->getDisplayString().setTagArg("i", 1, "green");
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
        findHost()->getDisplayString().setTagArg("i", 1, "red");
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

void MyVeinsApp::handleSelfMsg(cMessage* msg)
{
    DemoBaseApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission

}

void MyVeinsApp::handlePositionUpdate(cObject* obj)
{

    DemoBaseApplLayer::handlePositionUpdate(obj);
    findHost()->getDisplayString().setTagArg("i", 1, "red");

    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}
