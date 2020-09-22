#include "artery/rosomnet/RawObject.h"
#include "artery/rosomnet/RawService.h"
#include "artery/rosomnet/Asn1PacketVisitor.h"
#include "artery/rosomnet/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <chrono>


using namespace omnetpp;

/*
auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;
*/
static const simsignal_t scSignalRawReceived = cComponent::registerSignal("RawReceived");
static const simsignal_t scSignalRawSent = cComponent::registerSignal("RawSent");

static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


RawService::RawService() :
		mVehicleDataProvider(nullptr)
		//mTimer(nullptr)
		//mGenRawMin { 100, SIMTIME_MS },
		//mGenRawMax { 1000, SIMTIME_MS },
		//mGenRaw(mGenRawMax),
		//mGenRawLowDynamicsCounter(0),
		//mGenRawLowDynamicsLimit(3)
{
}

void RawService::initialize()
{
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

}

void RawService::sendRaw(const SimTime& T_now)
{
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	//uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	//auto raw = createRawMessage(*mVehicleDataProvider, genDeltaTimeMod);
	auto raw = createRawMessage(*mVehicleDataProvider);
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

	using RawByteBuffer = convertible::byte_buffer_impl<asn1::Raw>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new RawByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));

}

vanetza::asn1::Raw createRawMessage(const VehicleDataProvider& vdp)
{
	vanetza::asn1::Raw message;

	RAWHeader_t& header = (*message).header;

	header.rawstationID = vdp.station_id();



	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency Raw: %s", error.c_str());
	}

	return message;
}

