/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "IS_TS103301/ISO_TS_19091.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_Connection_H_
#define	_Connection_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ConnectingLane.h"
#include "SignalGroupID.h"
#include "RestrictionClassID.h"
#include "LaneConnectionID.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct IntersectionReferenceID;

/* Connection */
typedef struct Connection {
	ConnectingLane_t	 connectingLane;
	struct IntersectionReferenceID	*remoteIntersection	/* OPTIONAL */;
	SignalGroupID_t	*signalGroup	/* OPTIONAL */;
	RestrictionClassID_t	*userClass	/* OPTIONAL */;
	LaneConnectionID_t	*connectionID	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Connection_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Connection;
extern asn_SEQUENCE_specifics_t asn_SPC_Connection_specs_1;
extern asn_TYPE_member_t asn_MBR_Connection_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "IntersectionReferenceID.h"

#endif	/* _Connection_H_ */
#include "asn_internal.h"
