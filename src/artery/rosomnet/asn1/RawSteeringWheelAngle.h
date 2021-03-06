/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "RAW"
 * 	found in "RAW.asn"
 */

#ifndef	_RawSteeringWheelAngle_H_
#define	_RawSteeringWheelAngle_H_


#include "asn_application.h"

/* Including external dependencies */
#include"NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RawSteeringWheelAngle {
	RawSteeringWheelAngle_straight	= 0,
	RawSteeringWheelAngle_tenMiliRadiansToRight	= -1,
	RawSteeringWheelAngle_tenMiliRadiansRadiansToLeft	= 1,
	RawSteeringWheelAngle_unavailable	= 512
} e_RawSteeringWheelAngle;

/* RawSteeringWheelAngle */
typedef long	 RawSteeringWheelAngle_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RawSteeringWheelAngle;
asn_struct_free_f RawSteeringWheelAngle_free;
asn_struct_print_f RawSteeringWheelAngle_print;
asn_constr_check_f RawSteeringWheelAngle_constraint;
ber_type_decoder_f RawSteeringWheelAngle_decode_ber;
der_type_encoder_f RawSteeringWheelAngle_encode_der;
xer_type_decoder_f RawSteeringWheelAngle_decode_xer;
xer_type_encoder_f RawSteeringWheelAngle_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _RawSteeringWheelAngle_H_ */
#include "asn_internal.h"