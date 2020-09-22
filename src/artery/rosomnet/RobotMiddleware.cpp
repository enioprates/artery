/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/rosomnet/RobotMiddleware.h"
#include "artery/traci/ControllableVehicle.h"
#include "artery/traci/MobilityBase.h"
#include "inet/common/ModuleAccess.h"
#include <vanetza/common/position_fix.hpp>
#include "artery/rosomnet/ROSOMNeT.h"
#include "inet/mobility/base/MovingMobilityBase.h"
#include "artery/rosomnet/Asn1PacketVisitor.h"
#include <fstream>
#include <chrono>


using namespace std;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
//int number_msg = 0;																						//variavel para contagem do numero de pacotes recebidos
int num_receive_msg_RM = 0;


namespace artery
{

Define_Module(RobotMiddleware)

/*RobotMiddleware::RobotMiddleware() : 
		nameSpace("/robot_" + to_string(instanceCounter++) + "/"), rosomnet(ROSOMNeT::getInstance()) {
		//nameSpace("/robot_" + to_string(ii++) + "/"), rosomnet(ROSOMNeT::getInstance()) {

	cout << "RobotMiddleware constructor: " << nameSpace << endl;
	//timerMessage = new cMessage(TIMER_MESSAGE);
}*/
int RobotMiddleware::instanceCounter = 1;

RobotMiddleware::RobotMiddleware() : 
rosomnet(ROSOMNeT::getInstance("MID")), nameSpace("/MID_" + to_string(instanceCounter++) + "/")
{
	cout << "RobotMiddleware constructor" << endl;
}

void RobotMiddleware::initialize(int stage)
{
	numReceived_RM=0;
	WATCH(numReceived_RM);
	if (stage == 0) {

		
		findHost()->subscribe(INET_API::MobilityBase::stateChangedSignal, this);
		getFacilities().register_const(&mVehicleDataProvider);
		initializeVehicleController();
		findHost()->subscribe(scSignalCamReceived,this);
		std::stringstream rx_omnet_topic;
		string ROStopic;

		// if ( (int)nameSpace.at(6) == 47 ){
		// 	//ROStopic = std::string("/car") + nameSpace.at(5) + "/omnetCAM";
		// 	ROStopic = std::string("/car3/omnetCAM"); //MODIFICAR PARA HAVER MOVIMENTACAO!!
		// 	//rx_omnet_topic << "/car" << nameSpace.at(5) << "/RXNetwork_OMNET";
		// 	rx_omnet_topic << "/car3/RXNetwork_OMNET"; //MODIFICAR PARA HAVER MOVIMENTACAO!!
		// }
		// else {
		// 	//ROStopic = std::string("/car") + "FAKE" + "/omnetCAM";
		// 	//rx_omnet_topic << "/car" << "FAKE" << "/RXNetwork_OMNET";
		// 	ROStopic = std::string("/car3/omnetCAM"); //MODIFICAR PARA HAVER MOVIMENTACAO!!
		// 	rx_omnet_topic << "/car3/RXNetwork_OMNET"; //MODIFICAR PARA HAVER MOVIMENTACAO!!

		// }
		if ( (int)nameSpace.at(5) <= 53 ){
			ROStopic = std::string("/car") + nameSpace.at(5) + "/omnetCAM";
			//ROStopic = std::string("/car3/omnetCAM"); //MODIFICAR PARA HAVER MOVIMENTACAO!!
			rx_omnet_topic << "/car" << nameSpace.at(5) << "/RXNetwork_OMNET";
			//rx_omnet_topic << "/car3/RXNetwork_OMNET"; //MODIFICAR PARA HAVER MOVIMENTACAO!!
		} else{
			//ROStopic = std::string("/car") + nameSpace.at(5) + "/omnetCAM";
			ROStopic = std::string("/car5/omnetCAM"); //MODIFICAR PARA HAVER MOVIMENTACAO!!
			//rx_omnet_topic << "/car" << nameSpace.at(5) << "/RXNetwork_OMNET";
			rx_omnet_topic << "/car5/RXNetwork_OMNET"; //MODIFICAR PARA HAVER MOVIMENTACAO!!
		}



		cout << "RM advertises " << rx_omnet_topic.str() << endl;

		RXNetSubscriber = rosomnet.getROSNode().subscribe(ROStopic,TOPIC_QUEUE_LENGTH, &RobotMiddleware::RXNetCallback ,this);

		//Publish (CAM simplified messages) to RX car topic
		pub_RX_omnet	                   = rosomnet.getROSNode().advertise<ros_its_msgs::CAM_simplified>(rx_omnet_topic.str(), 1);
		
	} else if (stage == 1) {
		mVehicleDataProvider.update(mVehicleController);
		updatePosition();
	}

	Middleware::initialize(stage);
}

void RobotMiddleware::finish()
{
	Middleware::finish();
	findHost()->unsubscribe(INET_API::MobilityBase::stateChangedSignal, this);
}

void RobotMiddleware::initializeIdentity(Identity& id)
{
	Middleware::initializeIdentity(id);
	//id.traci = mVehicleController->getVehicleId();
	id.application = mVehicleDataProvider.station_id();
}

void RobotMiddleware::initializeManagementInformationBase(vanetza::geonet::MIB& mib)
{
	using vanetza::geonet::StationType;
	Middleware::initializeManagementInformationBase(mib);

	mGnStationType = StationType::PASSENGER_CAR;
	mVehicleDataProvider.setStationType(mGnStationType);

}

void RobotMiddleware::initializeVehicleController()
{

}

void RobotMiddleware::RXNetCallback(const ros_its_msgs::OMNET_CAM &msg) {
    
    car_name = msg.car_name;
    ros_x = msg.latitude;
    ros_y = msg.longitude;
    ros_z = msg.altitude;
    heading = msg.heading;
	acceleration = msg.acceleration;

	//std::cout  << "RobotMiddleware::carName: "<< car_name << " , " << acceleration << std::endl;//DEBUG_ENIO

	RobotMiddleware::setPosition(ros_x + 1000, ros_y + 300, ros_z); //OMNET axis offset
}

void RobotMiddleware::receiveSignal(cComponent* source, simsignal_t signal, cObject *obj, cObject*)
{
	//num_receive_msg_RM = num_receive_msg_RM +1;
	
	if ((signal == scSignalCamReceived) ) {
        numReceived_RM++;
		auto* cam = dynamic_cast<CaObject*>(obj);
        if (cam) {
            uint32_t stationID = cam->asn1()->header.stationID;

			//std::ofstream myfile_received;
			//myfile_received.open("CAMReceivedXX.csv", std::ofstream::out | std::ofstream::app);
			//double time_message = simTime().dbl() * 1000000;															//salvando tempo da msg em seg *1000000 (no tempo do omnet)
			double time_message = simTime().dbl();															//salvando tempo da msg em seg (no tempo do omnet)
			num_receive_msg_RM = num_receive_msg_RM + 1; 
			//myfile_received << number_msg << "," << mVehicleDataProvider.station_id() << "," << stationID


			std::ofstream myfile;

			//(ENIO)Recuperando numero da Mensagem
			int id_message = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue;
			double teste_h = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue;
			teste_h = (teste_h * M_PI / 180.00000) - M_PI ; //to rad * 0.1
			double teste_long = cam->asn1()->cam.camParameters.basicContainer.referencePosition.longitude / 1000000.000 ;
			double teste_lat = cam->asn1()->cam.camParameters.basicContainer.referencePosition.latitude/ 1000000.000 ;
			
			
			if (num_receive_msg_RM == 1){
				myfile.open("CAMReceivedRM.csv", std::ofstream::out | std::ofstream::trunc);
				//myfile << "num_msg" << "," << "Sender" 
			 	//		<< "," << "Receiver" << "," << "Time" << "," << "head" << "," << "lat" << "," << "long" << "," << "ID_msg"<<std::endl;										//enviando a variavel para o arquivo
    			auto t0 = std::chrono::high_resolution_clock::now();        
    			auto nanosec = t0.time_since_epoch();
				//myfile << nanosec.count() << std::endl;
				//double ms1 = simTime().dbl() * 1000;															//salvando tempo da msg em seg *1000000 (no tempo do omnet)
				double ms1 = simTime().dbl();
				//myfile << nanosec.count() << std::endl;
				myfile << num_receive_msg_RM << "," << stationID << "," << mVehicleDataProvider.station_id() <<"," 
						<< ms1 << "," << teste_h << "," << teste_lat << "," << teste_long << "," << id_message << std::endl;																//enviando a variavel para o arquivo
				//myfile << ms1 << std::endl;																		//enviando a variavel para o arquivo
				myfile.close();
			}else{
				myfile.open("CAMReceivedRM.csv", std::ofstream::out | std::ofstream::app);
    			auto t0 = std::chrono::high_resolution_clock::now();        
    			auto nanosec = t0.time_since_epoch();
				//myfile << nanosec.count() << std::endl;
				double ms1 = simTime().dbl();															//salvando tempo da msg em seg *1000000 (no tempo do omnet)
				//myfile << nanosec.count() << std::endl;
				myfile << num_receive_msg_RM << "," << stationID << "," << mVehicleDataProvider.station_id() <<"," 
						<< ms1 << "," << teste_h << "," << teste_lat << "," << teste_long << "," << id_message << std::endl;																//enviando a variavel para o arquivo
				//myfile << ms1 << std::endl;																		//enviando a variavel para o arquivo
				myfile.close();
			}



			// if (mVehicleDataProvider.station_id() == 1804289383){
			// 	std::ofstream myfile;
			// 	myfile.open("CAMReceivedCar1.csv", std::ofstream::out | std::ofstream::app);
    		// 	auto t0 = std::chrono::high_resolution_clock::now();        
    		// 	auto nanosec = t0.time_since_epoch();
			// 	//myfile << nanosec.count() << std::endl;
			// 	double ms1 = simTime().dbl() * 1000000;															//salvando tempo da msg em seg *1000000 (no tempo do omnet)
			// 	//myfile << nanosec.count() << std::endl;
			// 	myfile << stationID << "," << ms1 << std::endl;																//enviando a variavel para o arquivo
			// 	//myfile << ms1 << std::endl;																		//enviando a variavel para o arquivo
			// 	myfile.close();
			// 	}



			double Speed = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue;
			Speed = Speed * 0.01;
			double Heading = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue;
			Heading = (Heading * M_PI / 180.00000) - M_PI ; //to rad * 0.1
			double latitude = cam->asn1()->cam.camParameters.basicContainer.referencePosition.latitude / 1000000.000 ;
			//latitude = latitude * M_PI / 180;
			double longitude = cam->asn1()->cam.camParameters.basicContainer.referencePosition.longitude / 1000000.000;
			//longitude = longitude * M_PI / 180;
			double altitude = cam->asn1()->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue;
			double acceleration = cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue;
			//std::cout  << "RobotMiddleware::receiveSignal: " << acceleration << std::endl;//DEBUG_ENIO


			double R = 6731;//km

			double xx = R * cos (latitude) * cos (longitude);
			double yy =	R * cos (latitude) * sin (longitude);
			double zz = R * sin (latitude);

			EV_WARN << "CAM RECEPTION: " << id_message << std::endl;
			EV_WARN << "station ID:" << stationID << std::endl;
			//EV_WARN << "latitude:" << latitude << std::endl; //FINISHED
			//EV_WARN << "longitude:" << longitude << std::endl; //FINISHED
			//EV_WARN << "altitude:" << altitude << std::endl;
			//EV_WARN << "X:" << xx << std::endl;
			//EV_WARN << "Y:" << yy << std::endl;
			EV_WARN << "speed:"<< Speed << std::endl; //m/s FINISHED
			EV_WARN << "heading:"<< Heading << std::endl; //rad FINISHED
			EV_WARN << "N_message:"<< id_message << std::endl; //rad FINISHED
			EV_WARN << simTime().dbl()<< std::endl; //ENIO_DEBUG

			ros_its_msgs::CAM_simplified msg_to_send;

			if (stationID == 846930886)
				msg_to_send.car_name = "/car2/";
			else if (stationID == 1804289383 )
				msg_to_send.car_name = "/car1/";
			else if (stationID == 1681692777 )
				msg_to_send.car_name = "/car3/";
			else if (stationID == 1714636915)
				msg_to_send.car_name = "/car4/";
			else if (stationID == 1957747793)
				msg_to_send.car_name = "/car5/";
			else
				msg_to_send.car_name = "/car6/";

  			msg_to_send.Station_ID = to_string(stationID);
			msg_to_send.latitude =  latitude;
  			msg_to_send.longitude = longitude;
  			msg_to_send.altitude_altitudeValue = zz;
  			msg_to_send.heading_headingValue = Heading; //FINISHED
  			msg_to_send.speed_speedValue = Speed; //FINISHED
			msg_to_send.gasPedalPercent_Value = acceleration;



  			//msg_to_send.driveDirection = strtof((msgFields[6]).c_str(),0);
  			//msg_to_send.steeringWheelAngle_steeringWheelAngleValue = strtof((msgFields[2]).c_str(),0);

			pub_RX_omnet.publish(msg_to_send);




        } else {
            EV_ERROR << "received signal has no CaObject";
        }
    }

}

void RobotMiddleware::update()
{
	updatePosition();
	mVehicleDataProvider.update(mVehicleController);
	Middleware::update();
}

void RobotMiddleware::updatePosition()
{
	using namespace vanetza::units;
	static const TrueNorth north;
	vanetza::PositionFix position_fix;
	position_fix.timestamp = getRuntime().now();


	position_fix.latitude = GeoAngle::from_value(ros_x);
	position_fix.longitude = GeoAngle::from_value(ros_y);

	position_fix.confidence.semi_minor = 5.0 * si::meter;
	position_fix.confidence.semi_major = 5.0 * si::meter;

	position_fix.course.assign(north + GeoAngle::from_value(heading), north + 3.0 * degree);
	//position_fix.speed.assign(mVehicleDataProvider.speed(), 1.0 * si::meter_per_second);
	

	getRouter().update_position(position_fix);
}

void RobotMiddleware::setPosition(const double x, const double y, const double z) {

	MobilityROS *mobility = getMobilityModule();

	if (mobility != NULL) {
		
		//Coord currentPosition = mobility->getCurrentPosition();
		Coord newPosition = Coord::ZERO;
		newPosition.x = x;
		newPosition.y = y;
		newPosition.z = z;

		mobility->setCurrentPosition(newPosition);

	} else {
		std::cerr << "Attempt to set position with custom mobility == NULL" << endl;
	}
}

MobilityROS* RobotMiddleware::getMobilityModule() {
	//cout << "get Mobility module" << endl;
	
	cModule *host = getContainingNode(this);
	MobilityROS *mobility = check_and_cast<MobilityROS *>(host->getSubmodule("mobility"));
	//IMobility *mobility = check_and_cast<IMobility *>(getParentModule()->getSubmodule("mobility"));
	
	Coord currentPosition = mobility->getCurrentPosition();
	//cout << "OMNET++ Mobility" << endl;
	//cout <<"car:" << car_name <<endl;
	//cout << "x:" << currentPosition.x << endl;
	//cout << "y:" << currentPosition.y << endl;
	//cout << "z:" << currentPosition.z << endl;

	

	return mobility;
}

} // namespace artery

