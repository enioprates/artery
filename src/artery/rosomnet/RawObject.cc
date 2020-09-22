#include <artery/rosomnet/RawObject.h>
#include <omnetpp.h>
#include <cassert>

using namespace vanetza::asn1;

Register_Abstract_Class(RawObject)

RawObject::RawObject(Raw&& raw) :
    m_raw_wrapper(std::make_shared<Raw>(std::move(raw)))
{
}

RawObject& RawObject::operator=(Raw&& raw)
{
    m_raw_wrapper = std::make_shared<Raw>(std::move(raw));
    return *this;
}

RawObject::RawObject(const Raw& raw) :
    m_raw_wrapper(std::make_shared<Raw>(raw))
{
}

RawObject& RawObject::operator=(const Raw& raw)
{
    m_raw_wrapper = std::make_shared<Raw>(raw);
    return *this;
}

RawObject::RawObject(const std::shared_ptr<const Raw>& ptr) :
    m_raw_wrapper(ptr)
{
    assert(m_raw_wrapper);
}

RawObject& RawObject::operator=(const std::shared_ptr<const Raw>& ptr)
{
    m_raw_wrapper = ptr;
    assert(m_raw_wrapper);
    return *this;
}

std::shared_ptr<const Raw> RawObject::shared_ptr() const
{
    assert(m_raw_wrapper);
    return m_raw_wrapper;
}

const vanetza::asn1::Raw& RawObject::asn1() const
{
    return *m_raw_wrapper;
}



using namespace omnetpp;

class RawStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto raw = dynamic_cast<RawObject*>(object)) {
            const auto id = raw->asn1()->header.rawstationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("rawStationId", RawStationIdResultFilter)


/*class RawGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto raw = dynamic_cast<RawObject*>(object)) {
            const auto genDeltaTime = raw->asn1()->raw.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("rawGenerationDeltaTime", RawGenerationDeltaTimeResultFilter)
*/
