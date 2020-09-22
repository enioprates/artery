#ifndef RAWSERVICE_H_
#define RAWSERVICE_H_

#include "artery/rosomnet/ItsG5BaseService.h"
#include "artery/utility/Geometry.h"
#include <artery/rosomnet/asn1/raw.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "artery/rosomnet/Timer.h"
#include "artery/rosomnet/VehicleDataProvider.h"

class RawService : public ItsG5BaseService
{

public:
		RawService();
		void sendRaw(const omnetpp::SimTime&);
		void initialize() override;
	private:
		
		omnetpp::SimTime genRawDcc();

		//typedef Timer TimerosomnetA; //rosomnet compliancy

		const VehicleDataProvider* mVehicleDataProvider;
		//const TimerosomnetA* mTimer; //rosomnet compliancy
		//const Timer* mTimer;
		artery::LocalDynamicMap* mLocalDynamicMap;
		Position mLastRawPosition;
		vanetza::units::Velocity mLastRawSpeed;
		vanetza::units::Angle mLastRawHeading;
		omnetpp::SimTime mLastRawTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;

};

vanetza::asn1::Raw createRawMessage(const VehicleDataProvider&);

#endif /*RAWSERVICE_H_ */