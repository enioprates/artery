#ifndef ARTERY_RAWOBJECT_H_
#define ARTERY_RAWOBJECT_H_

#include <omnetpp/cobject.h>
//#include <vanetza/asn1/raw.hpp>
#include <artery/rosomnet/asn1/raw.hpp>
#include <memory>

class RawObject : public omnetpp::cObject
{
public:
    RawObject(const RawObject&) = default;
    RawObject& operator=(const RawObject&) = default;

    RawObject(vanetza::asn1::Raw&&);
    RawObject& operator=(vanetza::asn1::Raw&&);

    RawObject(const vanetza::asn1::Raw&);
    RawObject& operator=(const vanetza::asn1::Raw&);

    RawObject(const std::shared_ptr<const vanetza::asn1::Raw>&);
    RawObject& operator=(const std::shared_ptr<const vanetza::asn1::Raw>&);

    const vanetza::asn1::Raw& asn1() const;

    std::shared_ptr<const vanetza::asn1::Raw> shared_ptr() const;

private:
    std::shared_ptr<const vanetza::asn1::Raw> m_raw_wrapper;
};

#endif /* ARTERY_RAWOBJECT_H_ */
