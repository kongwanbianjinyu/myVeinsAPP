/*
 * SurroundingVehicle.h
 *
 *  Created on: Mar 5, 2020
 *      Author: veins
 */

#ifndef SRC_VEINS_MODULES_APPLICATION_IEEE80211P_SURROUNDINGVEHICLE_H_
#define SRC_VEINS_MODULES_APPLICATION_IEEE80211P_SURROUNDINGVEHICLE_H_

#include <list>
#include <string>
#include <stdint.h>

#include "veins/veins.h"

//#include "veins/modules/application/ieee80211p/ClusterAlgorithm.h"
#include "veins/base/utils/Coord.h"
#include "veins/base/utils/SimpleAddress.h"

using namespace omnetpp;

namespace veins{


class SurroundingVehicle
{
protected:
    LAddress::L2Type vehicleId;// each car's id,received from the beacon
    LAddress::L2Type clusterId;//if state == CM or CCM, record which cluster it belongs to.(default = -1,no cluster)
    Coord position;//the position of the car which sends beacon
    Coord velocity;//the velocity of the car which sends beacon
    double LLT;//link life time between this car and the car which sends beacon
    double deltaVelocity;//relative speed between this car and the car which sends beacon
    double deltaDistance;// relative distance between this car and the car which sends beacon
    bool isConnected = false;
public:
    SurroundingVehicle(LAddress::L2Type bvehicleId,LAddress::L2Type bclusterId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed,bool bisConnected);
    ~SurroundingVehicle();

    void updateInfo(LAddress::L2Type bvehicleId,LAddress::L2Type bclusterId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed,bool bisConnected);

    LAddress::L2Type getVehicleId();
    LAddress::L2Type getClusterId();
    Coord getPosition();
    Coord getVelocity();
    double getLLT();
    double getDeltaVelocity();
    double getDeltaDistance();
    bool getIsConnected();
    void setIsConnected(bool bisConnected);
};

}




#endif /* SRC_VEINS_MODULES_APPLICATION_IEEE80211P_SURROUNDINGVEHICLE_H_ */
