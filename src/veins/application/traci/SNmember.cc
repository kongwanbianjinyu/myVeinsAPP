/*
 * SNmember.cc
 *
 *  Created on: Feb 25, 2020
 *      Author: veins
 */

#include "SNmember.h"

#define TR 300 //default: transmission range = 300m
using namespace veins;

SNmember::SNmember(LAddress::L2Type bmemberId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed)
{
    memberId = bmemberId;
    position = bposition;
    velocity = bvelocity;
    deltaVelocity = curSpeed.x - bvelocity.x;
    deltaDistance = curPosition.distance(position);
    LLT = (-deltaVelocity*deltaDistance+std::abs(deltaVelocity)*TR)/(deltaVelocity*deltaVelocity);

}
void SNmember::updateInfo(LAddress::L2Type bmemberId,Coord bposition,Coord bvelocity,Coord curPosition,Coord curSpeed)
{
    memberId = bmemberId;
    position = bposition;
    velocity = bvelocity;
    deltaVelocity = curSpeed.x - bvelocity.x;
    deltaDistance = curPosition.distance(position);
    LLT = (-deltaVelocity*deltaDistance+std::abs(deltaVelocity)*TR)/(deltaVelocity*deltaVelocity);


}

LAddress::L2Type SNmember::getMemberId()
{
    return this->memberId;
}



Coord SNmember::getPosition()
{
    return this->position;
}

Coord SNmember::getVelocity()
{
    return this->velocity;
}

double SNmember::getLLT()
{
    return this->LLT;
}

double SNmember::getDeltaVelocity()
{
    return this->deltaVelocity;
}
double SNmember::getDeltaDistance()
{
    return this->deltaDistance;
}

