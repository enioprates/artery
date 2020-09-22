/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "RAW"
 * 	found in "RAW.asn"
 */

#ifndef	_CarID_H_
#define	_CarID_H_


#include "asn_application.h"

/* Including external dependencies */
#include"NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CarID {
	CarID_car1	= 1,
	CarID_car2	= 2,
	CarID_car3	= 3,
	CarID_unavailable	= 127
} e_CarID;

/* CarID */
typedef long	 CarID_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CarID;
asn_struct_free_f CarID_free;
asn_struct_print_f CarID_print;
asn_constr_check_f CarID_constraint;
ber_type_decoder_f CarID_decode_ber;
der_type_encoder_f CarID_encode_der;
xer_type_decoder_f CarID_decode_xer;
xer_type_encoder_f CarID_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _CarID_H_ */
#include "asn_internal.h"