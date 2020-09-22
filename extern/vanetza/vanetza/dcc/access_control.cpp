#include "access_control.hpp"
#include "mapping.hpp"
#include "scheduler.hpp"
#include <vanetza/access/data_request.hpp>
#include <vanetza/access/interface.hpp>
#include <iostream> //DEBUG_ENIO

int num_send_msg_acsess = 0;			//DEBUG_ENIO

namespace vanetza
{
namespace dcc
{

AccessControl::AccessControl(Scheduler& sc, access::Interface& ifc) :
    m_scheduler(sc), m_access(ifc), m_drop_excess(true)
{
}

void AccessControl::request(const DataRequest& dcc_req, std::unique_ptr<ChunkPacket> packet)
{
    num_send_msg_acsess++;
    std::cout  << "AccessControl::request : "<< num_send_msg_acsess <<std::endl;//DEBUG_ENIO
    const auto tx_delay = m_scheduler.delay(dcc_req.dcc_profile);
    const auto ac = map_profile_onto_ac(dcc_req.dcc_profile);

    if (tx_delay <= std::chrono::milliseconds(0) || !m_drop_excess) {
        access::DataRequest mac_req;
        mac_req.source_addr = dcc_req.source;
        mac_req.destination_addr = dcc_req.destination;
        mac_req.ether_type = dcc_req.ether_type;
        mac_req.access_category = ac;

        m_scheduler.notify(dcc_req.dcc_profile);
        m_access.request(mac_req, std::move(packet));
    } else {
        // drop packet
        hook_dropped(dcc_req, std::move(packet));
    }
}

} // namespace dcc
} // namespace vanetza
