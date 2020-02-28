/*
 * SNmember.h
 *
 *  Created on: Feb 25, 2020
 *      Author: veins
 */
#ifndef SRC_VEINS_MODULES_APPLICATION_TRACI_SNMEMBER_H_
#define SRC_VEINS_MODULES_APPLICATION_TRACI_SNMEMBER_H_

#include <list>
#include <string>
#include <stdint.h>

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace omnetpp;

namespace veins{


class SNmember
{
protected:
    LAddress::L2Type memberId;// each car's id,received from the beacon
    Coord position;//the position of the car which sends beacon
    Coord velocity;//the velocity of the car which sends beacon
    double LLT;//link life time between this car and the car which sends beacon
    double deltaVelocity;//relative speed between this car and the car which sends beacon
    double deltaDistance;// relative distance between this car and the car which sends beacon
public:
    SNmember(LAddress::L2Type bmemberId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed);
    ~SNmember();

    void updateInfo(LAddress::L2Type bmemberId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed);

    LAddress::L2Type getMemberId();
    Coord getPosition();
    Coord getVelocity();
    double getLLT();
    double getDeltaVelocity();
    double getDeltaDistance();
};

}

#endif /* SRC_VEINS_MODULES_APPLICATION_TRACI_SNMEMBER_H_ */
