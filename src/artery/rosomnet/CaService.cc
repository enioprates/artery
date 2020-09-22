#include "artery/rosomnet/CaObject.h"
#include "artery/rosomnet/RawObject.h"
#include "artery/rosomnet/CaService.h"
//#include "artery/rosomnet/RawService.h"
#include "artery/rosomnet/Asn1PacketVisitor.h"
#include "artery/rosomnet/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <chrono>
#include <fstream>

/*#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"

#include "inet/common/lifecycle/NodeOperations.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/contract/IL3AddressType.h"
*/

using namespace omnetpp;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");

static const simsignal_t scSignalRawReceived = cComponent::registerSignal("RawReceived");
static const simsignal_t scSignalRawSent = cComponent::registerSignal("RawSent");

Define_Module(CaService)

int num_send_msg = 0;			//Variable to count the number of Send messages
int num_receive_msg = 0;		//Variable to count the number of Received messages

double heading_teste = 0.0;
double lat_teste = 0.0;
double long_teste = 0.0;

static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


CaService::CaService() :
		mVehicleDataProvider(nullptr),
		mTimer(nullptr),
		mGenCamMin { 30, SIMTIME_MS }, 
		mGenCamMax { 100, SIMTIME_MS }, 
		deltattargetMin {500 ,SIMTIME_MS },
		deltattargetMax {2000 ,SIMTIME_MS }, 
		deltattarget{1500, SIMTIME_MS},
		mGenCam(mGenCamMax),
		mGenCamLowDynamicsCounter(0),
		mGenCamLowDynamicsLimit(3)

{
}

void CaService::initialize()
{
	ItsG5BaseService::initialize();
    
	// Initialize variables
    numSent = 0;					//Variable to count the number of Send messages
    numReceived = 0;				//Variable to count the number of Received messages

	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timerosomnet>(); //rosomnet compliancy (Timer)
	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCamTimestamp = simTime();
	// first generated CAM shall include the low frequency container
	mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// generation rate boundaries
	mGenCamMin = par("minInterval");
	mGenCamMax = par("maxInterval");

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");
	mPlatoonMode = par("platoonMode");

	//delta generation values
    deltattargetMin = par("mindeltattarget");
	deltattargetMax = par("maxdeltattarget");
	deltattarget= par("deltattarget");

	WATCH(numSent);				//count sent messages
    WATCH(numReceived);			//Count received messages

}

void CaService::trigger()
{
	checkTriggeringConditions(simTime());
}

void CaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	double receive_time = simTime().dbl() * 1000000;											//salvando tempo da msg em seg *1000000 (no tempo do omnet)
	num_receive_msg = num_receive_msg +1;
	numReceived++;

	
	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	if (cam && cam->validate()) {
		CaObject obj = visitor.shared_wrapper;
		emit(scSignalCamReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
		//std::cout << "CAM received" << std::endl;

		//auto cam = dynamic_cast<CaObject>(obj);

		//TVlatitude = obj->asn1()->cam.camParameters.basicContainer.referencePosition.latitude / 1000000.000;
		//TVlongitude = obj->asn1()->cam.camParameters.basicContainer.referencePosition.longitude / 1000000.000;



	}
}

void CaService::receiveSignal(cComponent* source, simsignal_t signal, cObject *obj, cObject*)
{
	//std::cout  << "CaService::receiveSignal: " << std::endl;//DEBUG_ENIO
	if (signal == scSignalCamReceived) {
        auto* cam = dynamic_cast<CaObject*>(obj);
        if (cam) {
			//if (cam->asn1()->header.stationID == 1804289383 && mVehicleDataProvider->station_id()== 846930886 || cam->asn1()->header.stationID == 846930886 && mVehicleDataProvider->station_id()== 1681692777){
			double latitude = cam->asn1()->cam.camParameters.basicContainer.referencePosition.latitude / 1000000.000 ;
			//latitude = latitude * M_PI / 180;
			double longitude = cam->asn1()->cam.camParameters.basicContainer.referencePosition.longitude / 1000000.000;
			//longitude = longitude * M_PI / 180;
			double altitude = cam->asn1()->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue;
			
			double R = 6731;//km

			TVX = R * cos (latitude) * cos (longitude);
			TVY = R * cos (latitude) * sin (longitude);

			//std::cout << "TVX:" << TVX << "TVY:" << TVY <<std::endl;
			
			

		//}
	}
	}
}			

void CaService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : mGenCamMin;
	const SimTime T_elapsed = T_now - mLastCamTimestamp;
	
	double send_time = simTime().dbl();												//enviando dados de tempo da msg em seg


	if (T_elapsed >= T_GenCamDcc) {
		//std::cout  << "CaService::checkTriggeringCondition (1): "<< send_time << std::endl;//DEBUG_ENIO
		if (!mPlatoonMode){
			//send_time = simTime().dbl();
			//std::cout  << "CaService::checkTriggeringCondition (2): "<< send_time << std::endl;//DEBUG_ENIO
			if (mFixedRate) {
			
				sendCam(T_now);
				//send_time = simTime().dbl();
				//std::cout  << "CaService::checkTriggeringCondition (3): "<< send_time << std::endl;//DEBUG_ENIO
			} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
				sendCam(T_now);
				T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
				mGenCamLowDynamicsCounter = 0;
			} else if (T_elapsed >= T_GenCam) {
				sendCam(T_now);
				if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
					T_GenCam = T_GenCamMax;
				}
			}
		}
 
		if (mPlatoonMode){
			//send_time = simTime().dbl();
			//std::cout  << "CaService::checkTriggeringCondition (4): "<< send_time << std::endl;//DEBUG_ENIO
			const VehicleDataProvider& vdp = *mVehicleDataProvider;

			double latitude = vdp.latitude().value() / 1000000.000 ;
			//latitude = latitude * M_PI / 180;
			double longitude = vdp.longitude().value() / 1000000.000;
			
			double R = 6731;//km

			double TSX = R * cos (latitude) * cos (longitude);
			double TSY = R * cos (latitude) * sin (longitude);

			distanceleader = sqrt(pow(TSX - TVX, 2) + pow(TSY - TVY, 2));

			
			T_GenCam = ((mGenCamMax - mGenCamMin)/(deltattargetMax-deltattargetMin))*deltattarget + (mGenCamMax - (mGenCamMax - mGenCamMin)/(deltattargetMax-deltattargetMin) * deltattargetMax);
			
			std::cout << "T_GenCam:" << T_GenCam << std::endl;
			std::cout << "T_GenCamMax:" << T_GenCamMax << std::endl;
			std::cout << "T_elapsed:" << T_elapsed << std::endl;
			if (T_elapsed >= T_GenCam || T_elapsed >=T_GenCamMax) {
				sendCam(T_now);
				//send_time = simTime().dbl();
				//std::cout  << "CaService::checkTriggeringCondition (5): "<< send_time << std::endl;//DEBUG_ENIO	
			}
		}
	}
}

bool CaService::checkHeadingDelta() const
{
	return abs(mLastCamHeading - mVehicleDataProvider->heading()) > mHeadingDelta; //mHeadingDelta 4º
}

bool CaService::checkPositionDelta() const
{
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);//mPositionDelta 4m
}

bool CaService::checkSpeedDelta() const
{
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;//mSpeedDelta 0.5 m/s
}

void CaService::sendCam(const SimTime& T_now)
{
	//if (num_send_msg<10 and mVehicleDataProvider->station_id()==1804289383){//DEBUG_ENIO
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod);

	mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
	mLastCamTimestamp = simTime();
	
	if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		//addLowFrequencyContainer(cam);  //29/05 - ENIO - Commented for not send low frequency messages 
		mLastLowCamTimestamp = T_now;
	} //UnComment for LowFreq Container sending

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CAM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::_1_S, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CaObject obj(std::move(cam));
	emit(scSignalCamSent, &obj);
	auto size = obj.asn1().size(); //variavel de teste do tamanho

	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };

	//calculando o THROUGHPUT
	//num_send_msg = num_send_msg + 1;
	numSent++;
	double teste; 
	//teste = mVehicleDataProvider->position();	//(ENIO) Enviando um numero da mensagem
	//double send_time = simTime().dbl() * 1000000;												//enviando dados de tempo da msg em seg *1000000
	double send_time = simTime().dbl();												//enviando dados de tempo da msg em seg
	double troughput = (num_send_msg*332*8)/(send_time*6000000);									//calculando o throughput

	std::cout  << "----------------"<< std::endl;//DEBUG_ENIO
	std::cout  << "CaService::sendCam (1): "<< mVehicleDataProvider->station_id() << " , "<<num_send_msg << " , "<< send_time << std::endl;//DEBUG_ENIO



	payload->layer(OsiLayer::Application) = std::move(buffer);
	//send_time = simTime().dbl();
	//std::cout  << "CaService::sendCam (2): "<<  num_send_msg << " , "<< send_time << std::endl;//DEBUG_ENIO
	this->request(request, std::move(payload));
	//send_time = simTime().dbl();
	//std::cout  << "CaService::sendCam (3): "<<  num_send_msg << " , "<< send_time << std::endl;//DEBUG_ENIO


	std::ofstream myfile_send;																	//criando o objeto do qrquivo
	if (num_send_msg == 1){
		myfile_send.open("CAMSent.csv", std::ofstream::out | std::ofstream::trunc);					//abrindo o arquivo
		//myfile_send << "num" << "," << "send_time" << "," 											//enviando os dados para o arquivo
		//			<< "sender" << "," << "troughput" <<  "," << "head" <<  "," << "lat_id" 
		//			<<  "," << "long_id"<< std::endl;		//

		//myfile_send << num_send_msg << "," << send_time << "," 											//enviando os dados para o arquivo
		//			<< mVehicleDataProvider->station_id() << "," << troughput <<  "," 
		//			<< heading_teste <<  "," << lat_teste <<  "," << long_teste << std::endl;		//

		myfile_send << num_send_msg << "," << send_time << "," 											//enviando os dados para o arquivo
					<< mVehicleDataProvider->station_id() << "," << troughput << std::endl;		//

		myfile_send.close();																		//fechando o arquivo

		//myfile_send.open("BACKOFFS.csv", std::ofstream::out | std::ofstream::trunc);					
		//myfile_send << "MessageID" << std::endl << "BackoffSlots" << "," << "WaitTime" << "," 
		//			<< "SlotTime" << std::endl;	
		//myfile_send << "MessageID" << "," << "BackoffSlots" << "," << "WaitTime" << "," 
		//			<< "SlotTime" << "," << "NumMessageCont" <<std::endl;	


		//myfile_send << num_send_msg << std::endl;
		
		//myfile_send << num_send_msg << ",";
		//myfile_send.close();

	}else{
		myfile_send.open("CAMSent.csv", std::ofstream::out | std::ofstream::app);					//abrindo o arquivo
		//myfile_send << num_send_msg << "," << send_time << "," 											//enviando os dados para o arquivo
		//				<< mVehicleDataProvider->station_id() << "," << troughput  <<  "," 
		//				<< heading_teste <<  "," << lat_teste <<  "," << long_teste << std::endl;		//
		myfile_send << num_send_msg << "," << send_time << "," 											//enviando os dados para o arquivo
					<< mVehicleDataProvider->station_id() << "," << troughput << std::endl;		//		
		myfile_send.close();																		//fechando o arquivo

		myfile_send.open("BACKOFFS.csv", std::ofstream::out | std::ofstream::app);
		myfile_send << num_send_msg << ",";
		myfile_send.close();

	}
	EV_WARN << "CAM SENT " << num_send_msg << "," << send_time << "," 											//enviando os dados para o arquivo
						<< mVehicleDataProvider->station_id() << std::endl; //ENIO_DEBUG
	//std::cout << "CAM sent" << std::endl;
	//send_time = simTime().dbl();
	//std::cout  << "CaService::sendCam (4): "<<  num_send_msg << " , "<< send_time << std::endl;//DEBUG_ENIO

	//}
}

SimTime CaService::genCamDcc()
{
	vanetza::Clock::duration delay = getFacilities().getDccScheduler().delay(vanetza::dcc::Profile::DP2);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS };
	//std::cout  << "CaService::genCamDcc: " << std::endl;//DEBUG_ENIO
	//return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}

vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Cam message;
	//std::cout  << "createCooperativeAwarenessMessage (1) " << std::endl;//DEBUG_ENIO
	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 1;
	header.messageID = ItsPduHeader__messageID_cam;
	header.stationID = vdp.station_id();

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;


	if(abs(vdp.longitude().value())>Longitude_unavailable)
		basic.referencePosition.longitude = 0.0;//round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	else
		basic.referencePosition.longitude = vdp.longitude().value();//round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
		

	if(abs(vdp.latitude().value())>Latitude_unavailable)
		basic.referencePosition.latitude = 0.0;//round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	else
		basic.referencePosition.latitude = vdp.latitude().value();//round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	long_teste = vdp.longitude().value()/ 1000000.000 ;////(ENIO)
	lat_teste = vdp.latitude().value()/ 1000000.000 ;////(ENIO)

	long_teste = vdp.acceleration().value();
	//std::cout  << "createCooperativeAwarenessMessage- Accel: " << long_teste << std::endl;//DEBUG_ENIO
	
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	if(abs(round(vdp.heading(), decidegree))>HeadingValue_unavailable)
		bvc.heading.headingValue = 0.0;
	else
		bvc.heading.headingValue = round(vdp.heading(), decidegree);
	heading_teste = round(vdp.heading(), decidegree); //(ENIO)
	//heading_teste = (heading_teste * M_PI / 180.00000) - M_PI ; //to rad * 0.1
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = round(vdp.speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	//std::cout  << "createCooperativeAwarenessMessage- vdp.acceleration(): " << vdp.acceleration() << std::endl;//DEBUG_ENIO
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		//std::cout  << "createCooperativeAwarenessMessage- Accel: " << lonAccelValue << std::endl;//DEBUG_ENIO
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue*100.0 * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
		//bvc.longitudinalAcceleration.longitudinalAccelerationValue = round(vdp.speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec;;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	
	/*bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (abs(bvc.yawRate.yawRateValue) >= YawRateValue_unavailable) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}*/
	
	
	num_send_msg = num_send_msg + 1;			//(ENIO) Incrementando o numro de mensagens enviadas
	bvc.yawRate.yawRateValue = num_send_msg;	//(ENIO) Enviando um numero da mensagem

	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

	/*if (vdp.station_id() == 1804289383){
	std::ofstream myfile;
	myfile.open("CaServiceCar1.txt", std::ofstream::out | std::ofstream::app);
	myfile << ros::Time::now().toNSec() * 0.000000001  << std::endl;
	myfile.close();
	}*/
	//std::cout  << "createCooperativeAwarenessMessage (2) " << std::endl;//DEBUG_ENIO
	return message;
}




void addLowFrequencyContainer(vanetza::asn1::Cam& message)
{
	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	bvc.vehicleRole = VehicleRole_default;
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);
	// TODO: add pathHistory

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}





}

void CaService::sendRaw(const SimTime& T_now)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto raw = createRawMessage(*mVehicleDataProvider, genDeltaTimeMod);

	//mLastCamPosition = mVehicleDataProvider->position();
	//mLastCamSpeed = mVehicleDataProvider->speed();
	//mLastCamHeading = mVehicleDataProvider->heading();

	mLastRawTimestamp = T_now;
	
	/*if (T_now - mLastLowRawTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		addLowFrequencyContainer(cam);
		mLastLowRawTimestamp = T_now;
	}*/

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::RAW;
	request.gn.its_aid = aid::RAW;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::_1_S, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	RawObject obj(std::move(raw));
	emit(scSignalRawSent, &obj);
	//string rawmsg = "[1,55555555,0,0,800,0,0,0]";

	//emit(scSignalRawSent,rawmsg);

	/*char msgName[32];
    sprintf(msgName, "RAW");

    auto packet = new inet::Packet(msgName);
*/
	using RawByteBuffer = convertible::byte_buffer_impl<asn1::Raw>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new RawByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));

}

vanetza::asn1::Raw createRawMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Raw message;

	RAWHeader_t& header = (*message).header;
	header.rawstationID = vdp.station_id();

	PlatoonPayload_t& platoon = (*message).platoon;
	platoon.carX = 1; //(1..127)
	platoon.rawstationID = vdp.station_id(); //(0..4294967295)
	platoon.rawsteeringWheelAngle = 0; //(-511..512) milirad/s
	platoon.rawheading = 0; //(-511..512) milirad/s
	platoon.rawspeed = 800; //(0..16383) cm/s 
	platoon.xxCoords = 0; //(-2097150..2097151) mm
	platoon.yyCoords = 0; //(-2097150..2097151) mm 
	platoon.zzCoords = 0; //(-2097150..2097151) mm






	//std::string error;
	//if (!message.validate(error)) {
	//	throw cRuntimeError("Invalid Raw: %s", error.c_str());
	//}

	return message;
}


