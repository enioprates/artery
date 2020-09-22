/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AVIAEINumberingAndDataStructures"
 * 	found in "IS_TS103301/ISO_TS_14816.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_IssuerIdentifier_H_
#define	_IssuerIdentifier_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IssuerIdentifier */
typedef long	 IssuerIdentifier_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_IssuerIdentifier_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_IssuerIdentifier;
asn_struct_free_f IssuerIdentifier_free;
asn_struct_print_f IssuerIdentifier_print;
asn_constr_check_f IssuerIdentifier_constraint;
ber_type_decoder_f IssuerIdentifier_decode_ber;
der_type_encoder_f IssuerIdentifier_encode_der;
xer_type_decoder_f IssuerIdentifier_decode_xer;
xer_type_encoder_f IssuerIdentifier_encode_xer;
oer_type_decoder_f IssuerIdentifier_decode_oer;
oer_type_encoder_f IssuerIdentifier_encode_oer;
per_type_decoder_f IssuerIdentifier_decode_uper;
per_type_encoder_f IssuerIdentifier_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _IssuerIdentifier_H_ */
#include "asn_internal.h"
