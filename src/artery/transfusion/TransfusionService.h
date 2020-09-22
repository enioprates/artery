//
// Copyright (C) 2015 Raphael Riebl <raphael.riebl@thi.de>
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

#ifndef __ARTERY_EXAMPLEPROMISCUOUSSERVICE_H_
#define __ARTERY_EXAMPLEPROMISCUOUSSERVICE_H_

#include "artery/application/ItsG5PromiscuousService.h"
#include "artery/utility/AsioTask.h"
#include <vanetza/common/byte_buffer.hpp>
#include <vanetza/geonet/areas.hpp>
#include <memory>

// forward declaration of types generated by protobuf compiler
namespace Transfusion {
class GeoBroadcast;
class TransfusionMsg;
} // namespace Transfusion

class TransfusionService : public ItsG5PromiscuousService
{
    public:
        void tapPacket(const vanetza::btp::DataIndication&, const vanetza::UpPacket&) override;

    protected:
        void initialize() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        void processMessage(const Transfusion::TransfusionMsg&);
        vanetza::geonet::Area buildDestinationArea(const Transfusion::GeoBroadcast&);
        std::unique_ptr<AsioTask> m_asio_task;
        vanetza::ByteBuffer m_buffer;
};

#endif
