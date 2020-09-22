/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "RAW"
 * 	found in "RAW.asn"
 */

#ifndef	_XX_H_
#define	_XX_H_


#include "asn_application.h"

/* Including external dependencies */
#include"NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum XX {
	XX_xAxis	= 0,
	XX_oneNegativeMilimeter	= -1,
	XX_onePositiveMilimeter	= 1,
	XX_unavailable	= 2097151
} e_XX;

/* XX */
typedef long	 XX_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_XX;
asn_struct_free_f XX_free;
asn_struct_print_f XX_print;
asn_constr_check_f XX_constraint;
ber_type_decoder_f XX_decode_ber;
der_type_encoder_f XX_encode_der;
xer_type_decoder_f XX_decode_xer;
xer_type_encoder_f XX_encode_xer;

#ifdef __cplusplus
}
#endif

#endif	/* _XX_H_ */
#include "asn_internal.h"