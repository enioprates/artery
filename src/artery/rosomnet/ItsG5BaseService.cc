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

#include "artery/application/ItsG5Service.h"
#include "artery/rosomnet/Middleware.h"
#include "veins/base/utils/FindModule.h"
#include <cassert>

using namespace artery;
using namespace omnetpp;

ItsG5BaseService::ItsG5BaseService() :
	m_middleware(nullptr)
{
}

ItsG5BaseService::~ItsG5BaseService()
{
}

Facilities& ItsG5BaseService::getFacilities()
{
	assert(m_middleware);
	return m_middleware->getFacilities();
}

const Facilities& ItsG5BaseService::getFacilities() const
{
	assert(m_middleware);
	return m_middleware->getFacilities();
}

bool ItsG5BaseService::requiresListener() const
{
	return true;
}

ItsG5BaseService::port_type ItsG5BaseService::getPortNumber() const
{
	assert(m_middleware);
	return m_middleware->getPortNumber(this);
}

cModule* ItsG5BaseService::findHost()
{
	return FindModule<>::findHost(this);
}

void ItsG5BaseService::initialize()
{
	Middleware* middleware = dynamic_cast<Middleware*>(getParentModule());
	if (middleware == nullptr) {
		throw cRuntimeError("Middleware not found");
	}

	m_middleware = middleware;
}

void ItsG5BaseService::finish()
{
	cSimpleModule::finish();
}

void ItsG5BaseService::subscribe(const simsignal_t& signal)
{
	//double send_time = simTime().dbl();												//enviando dados de tempo da msg em seg
	//std::cout  << "ItsG5BaseService::subscribe (1): " << send_time <<std::endl;//DEBUG_ENIO
	assert(m_middleware);
	//send_time = simTime().dbl();												//enviando dados de tempo da msg em seg
	//std::cout  << "ItsG5BaseService::subscribe (2): " << send_time <<std::endl;//DEBUG_ENIO
	m_middleware->subscribe(signal, this);
	//send_time = simTime().dbl();												//enviando dados de tempo da msg em seg
	//std::cout  << "ItsG5BaseService::subscribe (3): " << send_time <<std::endl;//DEBUG_ENIO

}

void ItsG5BaseService::unsubscribe(const simsignal_t& signal)
{
	assert(m_middleware);
	m_middleware->unsubscribe(signal, this);
}

void ItsG5BaseService::trigger()
{
}

void ItsG5BaseService::request(const vanetza::btp::DataRequestB& req, std::unique_ptr<vanetza::DownPacket> packet)
{
	//double send_time = simTime().dbl();												//enviando dados de tempo da msg em seg
	//std::cout  << "ItsG5BaseService::request (1): " << send_time <<std::endl;//DEBUG_ENIO
	assert(m_middleware);
	//send_time = simTime().dbl();
	//std::cout  << "ItsG5BaseService::request (2): " << send_time <<std::endl;//DEBUG_ENIO
	m_middleware->request(req, std::move(packet));
	//send_time = simTime().dbl();
	//std::cout  << "ItsG5BaseService::request (3): " << send_time <<std::endl;//DEBUG_ENIO
}

void ItsG5BaseService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
}
