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

#include "MobilityROS.h"


Define_Module(MobilityROS);

void MobilityROS::handleSelfMessage(cMessage *msg) {
    ASSERT(false);
}

int MobilityROS::numInitStages()
{
    return inet::INITSTAGE_PHYSICAL_ENVIRONMENT_2 + 1;
}

void MobilityROS::initialize (int stage){

	MobilityBase::initialize(stage);

}

void MobilityROS::setCurrentPosition(Coord coordinates) {
	bool changed = false;
	//std::cout << "debug" << endl;
	getCurrentPosition();

	if (lastPosition.x != coordinates.x) {
		lastPosition.x = coordinates.x;
		changed = true;
	}

	if (lastPosition.y != coordinates.y) {
		lastPosition.y = coordinates.y;
		changed = true;
	}

    if (lastPosition.z != coordinates.z) {
    	lastPosition.z = coordinates.z;
    	changed = true;
    }


    if (changed) {
    	inet::MobilityBase::emitMobilityStateChangedSignal();
    	updateVisualRepresentation();
    }
}


Coord MobilityROS::getCurrentPosition() {
    return lastPosition;
}

Coord MobilityROS::getCurrentSpeed() {
    return Coord::ZERO;
}
