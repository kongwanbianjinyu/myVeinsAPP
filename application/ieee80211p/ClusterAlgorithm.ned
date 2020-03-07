//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.modules.application.ieee80211p;
import org.car2x.veins.base.modules.IBaseApplLayer;
//
// mobility-based cluster algorithm
// @author JJC
//
simple ClusterAlgorithm like IBaseApplLayer
{
    parameters:
        @class(veins::ClusterAlgorithm);
    	string appName = default("The cluster Algorithm!");
    	int headerLength = default(88bit) @unit(bit); //header length of the application
    	
    	int beaconLengthBits = default(528bit) @unit(bit); //the length of a beacon packet
        int beaconUserPriority = default(7); //the user priority (UP) of the beacon messages
    	double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages
        double collectTime = default(3s) @unit(s);//the collect time limit for collecting beacons
        double Tch = default(5s) @unit(s);//CH periodically check the CML 
        double Tbch = default(5s) @unit(s);//CM or CCM periodically check the BCHL
        double Tcm = default(5s) @unit(s);//CM periodically check the the connection with CH
        double Tack = default(2s) @unit(s);//after send the reqJoin, go to BCH election if not receive the ACKjoin after Tack
        
    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
    
}
