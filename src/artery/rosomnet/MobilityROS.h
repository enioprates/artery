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

#ifndef __MANET_MobilityROS_H
#define __MANET_MobilityROS_H

#include <inet/mobility/base/MobilityBase.h>
#include <inet/common/geometry/common/Coord.h>
#include <inet/mobility/base/MovingMobilityBase.h>


using namespace inet;

class MobilityROS: public INET_API::MobilityBase
 {
protected:
	virtual void handleSelfMessage(cMessage *msg);
public:
	virtual void setCurrentPosition(Coord coordinates);
	virtual Coord getCurrentPosition();
	virtual Coord getCurrentSpeed();

	void initialize(int stage);
	int numInitStages();

};

#endif // __MANET_MobilityROS_H
