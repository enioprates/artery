#ifndef RAW_HPP_WXYNEKFN
#define RAW_HPP_WXYNEKFN

#include <vanetza/asn1/asn1c_conversion.hpp>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <artery/rosomnet/asn1/RAW.h>

namespace vanetza
{
namespace asn1
{

class Raw : public asn1c_wrapper<RAW_t>
{
public:
    Raw() : asn1c_wrapper(asn_DEF_RAW) {}
};

} // namespace asn1
} // namespace vanetza

#endif /* RAW_HPP_WXYNEKFN */

