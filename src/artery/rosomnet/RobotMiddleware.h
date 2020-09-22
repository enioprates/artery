/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_ROBOTMIDDLEWARE_H_SYJDG2DX
#define ARTERY_ROBOTMIDDLEWARE_H_SYJDG2DX

#include <string>
#include <queue>

#include "artery/rosomnet/Middleware.h"
#include "artery/rosomnet/VehicleDataProvider.h"
#include "artery/rosomnet/MobilityROS.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <omnetpp.h>
#include <ros_its_msgs/CAM_simplified.h>
#include "ROSOMNeT.h"


using namespace ros;
using namespace std;

namespace artery
{

class RobotMiddleware : public Middleware
{
	public:
		RobotMiddleware();
		void initialize(int stage) override;
		void finish() override;
		void setPosition(double x, double y, double z);
		void receiveSignal(cComponent*, omnetpp::simsignal_t, cObject*, cObject* = nullptr) override;
		ros::Publisher pub_RX_omnet;

	protected:
		void initializeIdentity(Identity&) override;
		void initializeManagementInformationBase(vanetza::geonet::MIB&) override;
		void update() override;

	private:
		long numReceived_RM;				//Contador de mensagens recebidas
		
	const long TOPIC_QUEUE_LENGTH = 1000;
	const double PACKET_TRANSMIT_INTERVAL = 0.010;

	//void RXNetCallback(const ros_its_msgs::CAM_simplified& msg);
	void RXNetCallback(const ros_its_msgs::OMNET_CAM& msg);
	ROSOMNeT& rosomnet;

		static int instanceCounter;
		const string nameSpace;

		//static int ii;

		string car_name;
		double ros_x;
		double ros_y;
		double ros_z;
		double heading;
		double acceleration;

		void initializeVehicleController();
		void updatePosition();
	
		Subscriber RXNetSubscriber;

		traci::VehicleController* mVehicleController;
		VehicleDataProvider mVehicleDataProvider;
		//INET_API::inet::IMobility* getMobilityModule();
		MobilityROS* getMobilityModule();
};

} // namespace artery

#endif /* ARTERY_ROBOTMIDDLEWARE_H_SYJDG2DX */

