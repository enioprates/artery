//
// Copyright (C) 2014 Raphael Riebl <raphael.riebl@thi.de>
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
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "VehicleDataProvider.h"
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/csimulation.h>
#include <vanetza/units/frequency.hpp>
#include <vanetza/units/time.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/angular_velocity.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <algorithm>


#include "MobilityROS.h"
//#include "traci/sumo/libsumo/TraCIDefs.h"

//using namespace inet;
//using namespace omnetpp;


const double pi = boost::math::constants::pi<double>();
const auto degree_per_second_squared = vanetza::units::degree / (vanetza::units::si::second * vanetza::units::si::second);
const std::map<VehicleDataProvider::AngularAcceleration, double> VehicleDataProvider::mConfidenceTable {
    { AngularAcceleration(0.0 * degree_per_second_squared), 1.0 },
    { AngularAcceleration(0.5 * degree_per_second_squared), 0.9 },
    { AngularAcceleration(1.0 * degree_per_second_squared), 0.8 },
    { AngularAcceleration(1.5 * degree_per_second_squared), 0.7 },
    { AngularAcceleration(2.0 * degree_per_second_squared), 0.6 },
    { AngularAcceleration(2.5 * degree_per_second_squared), 0.5 },
    { AngularAcceleration(5.0 * degree_per_second_squared), 0.4 },
    { AngularAcceleration(10.0 * degree_per_second_squared), 0.3 },
    { AngularAcceleration(15.0 * degree_per_second_squared), 0.2 },
    { AngularAcceleration(20.0 * degree_per_second_squared), 0.1 },
    { AngularAcceleration(25.0 * degree_per_second_squared), 0.0 },
    { AngularAcceleration(std::numeric_limits<double>::infinity() * degree_per_second_squared), 0.0 }
};

uint32_t VehicleDataProvider::instanceCounter = 1;

vanetza::units::Angle convertMobilityAngle(Angle angle)
{
	using vanetza::units::si::radians;
	// change rotation ccw -> cw
	angle.value *= -1.0;
	// rotate zero from east to north
	angle.value += 0.5 * pi * radians;
	// normalize angle to [0; 2*pi[
	angle.value -= 2.0 * pi * radians * std::floor(angle.value / (2.0 * pi * radians));

	assert(angle.value >= 0.0 * radians);
	assert(angle.value < 2.0 * pi * radians);
	return angle.value;
}

VehicleDataProvider::VehicleDataProvider() : VehicleDataProvider(rand()) 
{	
	
	//ROSOMNeT &rosomnet = ROSOMNeT::getInstance();
	//string stringROS = "ROSvdp";
	//cout << "Starting ROS node:VDP" << endl;

	//if (instanceCounter == 1)
	//rosomnet.runROSNode();

	string ROStopic = "/car" + to_string(instanceCounter-1) + "/omnetCAM";

	//cout << nameSpace << ":" << ROStopic << endl;

	RXNetSubscriber = rosomnet.getROSNode().subscribe(ROStopic, 2 ,&VehicleDataProvider::RXNetCallback,this);
	//RXNetSubscriber = rosomnet.getROSNode().subscribe("/Platoon_dist", 2 ,&VehicleDataProvider::RXNetCallback,this);	
	//RXNetSubscriber = rosomnet.getROSNode().subscribe("/car1/carINFO", 2 ,&VehicleDataProvider::RXNetCallback,this);
	//RXNetSubscriber = rosomnet.getROSNode().subscribe("/car1/RXNetwork", 2 ,&VehicleDataProvider::RXNetCallback,this);
	//RXNetSubscriber = rosomnet.getROSNode().subscribe("/clock", 1000 ,&VehicleDataProvider::RXNetCallback,this);
	//cout << "VDP:" << &ROSOMNeT::mutexROS << endl;
	
}

VehicleDataProvider::VehicleDataProvider(uint32_t id) :
	mStationId(id), mStationType(StationType::PASSENGER_CAR),
	mConfidence(0.0), mLastUpdate(omnetpp::SimTime::getMaxTime()),
	mCurvatureOutput(2), mCurvatureConfidenceOutput(2), 
	nameSpace("/VDP_" + to_string(instanceCounter++) + "/"), 
	rosomnet(ROSOMNeT::getInstance("VDP"))
{
	//cout << nameSpace << endl;
	while (!mCurvatureConfidenceOutput.full()) {
		using namespace vanetza::units::si;
		mCurvatureConfidenceOutput.push_front(0.0 * radians_per_second / second);
	}
	i = 0;

	//VehicleDataProvider::setPosition(10, 10, 10);


}	

void VehicleDataProvider::calculateCurvature()
{
	using namespace vanetza::units::si;
	static const vanetza::units::Frequency f_cut = 0.33 * hertz;
	static const vanetza::units::Duration t_sample = 0.1 * seconds;
	static const vanetza::units::Curvature lower_threshold = 1.0 / 2500.0 * vanetza::units::reciprocal_metre;
	static const vanetza::units::Curvature upper_threshold = 1.0 * vanetza::units::reciprocal_metre;
	static const double damping = 1.0;

	if (fabs(mSpeed) < 1.0 * meter_per_second) {
		// assume straight road below minimum speed
		mCurvature = 0.0 * vanetza::units::reciprocal_metre;
	} else {
		// curvature calculation algorithm
		mCurvature = (mYawRate / radians) / mSpeed;

		if (!mCurvatureOutput.full()) {
			// save first two values for initialization
			mCurvatureOutput.push_front(mCurvature);
			mCurvature = 0.0 * vanetza::units::reciprocal_metre;
		} else {
			static const auto omega = 2.0 * pi * f_cut;
			mCurvature = - mCurvatureOutput[1] +
				(2.0 + 2.0 * omega * damping * t_sample) * mCurvatureOutput[0] +
				omega * omega * t_sample * t_sample * mCurvature;
			mCurvature /= 1.0 + 2.0 * omega * damping * t_sample + omega * omega * t_sample * t_sample;
			mCurvatureOutput.push_front(mCurvature);

			// assume straight road below threshold
			if (fabs(mCurvature) < lower_threshold) {
				mCurvature = 0.0 * vanetza::units::reciprocal_metre;
			} else if (fabs(mCurvature) > upper_threshold) {
				// clamp minimum radius to 1 meter
				mCurvature = upper_threshold;
			}
		}
	}
}

void VehicleDataProvider::calculateCurvatureConfidence()
{
	assert(mCurvatureConfidenceOutput.full());
	using namespace vanetza::units::si;
	static const vanetza::units::Frequency f_cut = 1.0 * hertz;
	static const vanetza::units::Duration t_sample = 100.0 * seconds;
	static const double damping = 1.0;
	static const auto omega = 2.0 * pi * f_cut;

	AngularAcceleration filter = -mCurvatureConfidenceOutput[1] +
		(2.0 + 2.0 * omega * damping * t_sample) * mCurvatureConfidenceOutput[0] +
		omega * omega * t_sample * mYawRate -
		omega * omega * t_sample * mCurvatureConfidenceInput;
	filter /= 1.0 + 2.0 * omega * damping * t_sample + omega * omega * t_sample * t_sample;
	mCurvatureConfidenceOutput.push_front(filter);
	mCurvatureConfidenceInput = mYawRate;
	mConfidence = mapOntoConfidence(abs(filter));
}

void VehicleDataProvider::update(const traci::VehicleController* controller)
{
	using namespace omnetpp;
	using namespace vanetza::units::si;
	using boost::units::si::milli;
	const vanetza::units::Duration delta {
		(simTime() - mLastUpdate).inUnit(SIMTIME_MS) * milli * seconds
	};

	/*if (delta > 0.0 * seconds) {
		using boost::units::abs;

		//SPEED ACCELERATION
		auto new_speed = vanetza::units::Velocity::from_value(ros_speed);
		mAccel = (new_speed - mSpeed) / delta;
		mSpeed = new_speed;

		//HEADING
		auto new_heading = vanetza::units::Angle::from_value(ros_heading);
		auto diff_heading = mHeading - new_heading; // left turn positive
		if (diff_heading > pi * radian) {
			diff_heading -= 2.0 * pi * radians;
		} else if (diff_heading < -pi * radians) {
			diff_heading += 2.0 * pi * radians;
		}

		//mYawRate = diff_heading / delta;
		mHeading = new_heading;
	} else if (delta < 0.0 * seconds) {
		// initialization
		mSpeed = vanetza::units::Velocity::from_value(ros_speed);
		mHeading = vanetza::units::Angle::from_value(ros_heading);
	} else {
		// update has been called for this time step already before
		return;
	}
	*/

	//mPosition = controller->getPosition();
	mPosition = Position { ros_x, ros_y};
	//mPosition = libsumo::TraCIPosition { ros_x, ros_y, 0.0 };

	//mGeoPosition = controller->getGeoPosition();
	//mGeoPosition.latitude = ros_latitude * 0.0000001 * boost::units::degree::degree;// * boost::units::si::micro;
	//mGeoPosition.longitude = ros_longitude * 0.0000001 *  boost::units::degree::degree;// * boost::units::si::micro;

	mGeoPosition.latitude = ros_x * 1000000 * boost::units::degree::degree;
	mGeoPosition.longitude = ros_y * 1000000 * boost::units::degree::degree;

	mSpeed = vanetza::units::Velocity::from_value(ros_speed);
	mHeading = vanetza::units::Angle::from_value((ros_heading + M_PI) * 0.1);
	mAccel = vanetza::units::Acceleration::from_value(ros_acceleration);
	//std::cout  << "VehicleDataProvider::update: " << mAccel.value() << std::endl;//DEBUG_ENIO
	
	//mHeading = 
	//EV_WARN << "before cam.long:"<< ros_longitude << std::endl;
	////EV_WARN << "VehicleDataProvider:"<< std::endl;
	//EV_WARN << "vdp pointer:"<< (void*)this << std::endl;
	////EV_WARN << "mStationID:"<< this->mStationId << std::endl;
	//EV_WARN << "latitude:"<< ros_latitude << std::endl; //FINISHED (slight error)
	//EV_WARN << "longitude:"<< ros_longitude << std::endl; //FINISHED (slight error)
	//EV_WARN << "latitude:"<< mGeoPosition.latitude.value() << std::endl; //FINISHED (slight error)
	//EV_WARN << "longitude:"<< mGeoPosition.longitude.value() << std::endl; //FINISHED (slight error)
	//EV_WARN << "ros_x:"<< ros_x << std::endl;
	//EV_WARN << "ros_y:"<< ros_y << std::endl;
	////EV_WARN << "speed:"<< mSpeed.value() << std::endl; // FINISHED
	//EV_WARN << "heading:"<< ros_heading << std::endl;
	////EV_WARN << "heading:"<< mHeading.value() << std::endl; // FINISHED (slight error)
	////EV_WARN << "Position_X:"<< mPosition.x.value() << std::endl;
	////EV_WARN << "Position_Y:"<< mPosition.y.value() << std::endl;
	////EV_WARN << simTime().dbl()<< std::endl; //ENIO_DEBUG
	//TIME
	mLastUpdate = simTime();

}

double VehicleDataProvider::mapOntoConfidence(AngularAcceleration x) const
{
	auto it = mConfidenceTable.lower_bound(x);
	if (it == mConfidenceTable.end()) {
		throw std::domain_error("input value is less than smallest entry in confidence table");
	}
	return it->second;
}

void VehicleDataProvider::setStationType(StationType type)
{
	mStationType = type;
}

auto VehicleDataProvider::getStationType() const -> StationType
{
	return mStationType;
}

void VehicleDataProvider::RXNetCallback (const ros_its_msgs::OMNET_CAM &msg) { 
	
    car_name = msg.car_name;

	ros_heading = msg.heading;

    ros_speed = msg.speed;// m/s

	ros_acceleration = msg.acceleration;

	double R = 6731;//km
	ros_x = msg.latitude;
    ros_y = msg.longitude;
    ros_z = msg.altitude;
	ros_latitude = (asin(ros_z/R) * 180.0 / M_PI);// -9000000000 900000000 _ 0.1 microdeg
	ros_longitude = (atan2(ros_y,ros_x) * 180.0 / M_PI);// -1800000000 1800000000 _ 0.1 microdeg

	//std::cout  << "VehicleDataProvider::RXNetCallback: " << ros_acceleration << std::endl;//DEBUG_ENIO
	
	//if (ros_speed > 16382)
	//ros_speed = 163282; // 0 16382 _ 0,1 m/s

	//EV_WARN << "topic speed:"<< ros_speed << std::endl;

	//EV_WARN << "topic heading:"<< msg.heading << std::endl;
	//EV_WARN << "ros_heading:"<< ros_heading << std::endl;

 }

