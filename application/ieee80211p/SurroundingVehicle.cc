/*
 * SurroundingVehicle.cc
 *
 *  Created on: Mar 5, 2020
 *      Author: veins
 */


#include "SurroundingVehicle.h"

#define TR 300 //default: transmission range = 300m
using namespace veins;

SurroundingVehicle::SurroundingVehicle(LAddress::L2Type bvehicleId,LAddress::L2Type bclusterId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed,bool bisConnected)
{
    vehicleId = bvehicleId;
    clusterId = bclusterId;
    position = bposition;
    velocity = bvelocity;
    deltaVelocity = curSpeed.x - bvelocity.x;
    deltaDistance = curPosition.distance(position);
    LLT = (-deltaVelocity*deltaDistance+std::abs(deltaVelocity)*TR)/(deltaVelocity*deltaVelocity);
    isConnected = bisConnected;

}
void SurroundingVehicle::updateInfo(LAddress::L2Type bvehicleId,LAddress::L2Type bclusterId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed,bool bisConnected)
{
    vehicleId = bvehicleId;
    clusterId = bclusterId;
    position = bposition;
    velocity = bvelocity;
    deltaVelocity = curSpeed.x - bvelocity.x;
    deltaDistance = curPosition.distance(position);
    LLT = (-deltaVelocity*deltaDistance+std::abs(deltaVelocity)*TR)/(deltaVelocity*deltaVelocity);
    isConnected = bisConnected;

}

LAddress::L2Type SurroundingVehicle::getVehicleId()
{
    return this->vehicleId;
}

LAddress::L2Type SurroundingVehicle::getClusterId()
{
    return this->clusterId;
}

Coord SurroundingVehicle::getPosition()
{
    return this->position;
}

Coord SurroundingVehicle::getVelocity()
{
    return this->velocity;
}

double SurroundingVehicle::getLLT()
{
    return this->LLT;
}

double SurroundingVehicle::getDeltaVelocity()
{
    return this->deltaVelocity;
}
double SurroundingVehicle::getDeltaDistance()
{
    return this->deltaDistance;
}

bool SurroundingVehicle::getIsConnected()
{
    return this->isConnected;
}
void SurroundingVehicle::setIsConnected(bool bisConnected)
{
    isConnected = bisConnected;
}

