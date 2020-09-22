/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "RAW"
 * 	found in "RAW.asn"
 */

#ifndef	_RawHeading_H_
#define	_RawHeading_H_


#include "asn_application.h"

/* Including external dependencies */
#include"NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RawHeading {
	RawHeading_straight	= 0,
	RawHeading_tenMiliRadiansRadiansToRight	= -1,
	RawHeading_tenMiliRadiansRadiansToLeft	= 1,
	RawHeading_unavailable	= 512
} e_RawHeading;

/* RawHeading */
typedef long	 RawHeading_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RawHeading;
asn_struct_free_f RawHeading_free;
asn_struct_print_f RawHeading_print;
asn_constr_check_f RawHeading_constraint;
ber_type_decoder_f RawHeading_decode_ber;
der_type_encoder_f RawHeading_encode_der;
xer_type_decoder_f RawHeading_decode_xer;
xer_type_encoder_f RawHeading_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _RawHeading_H_ */
#include "asn_internal.h"