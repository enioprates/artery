/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "CAM_EN302637-2/CAM.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_RoadWorksContainerBasic_H_
#define	_RoadWorksContainerBasic_H_


#include "asn_application.h"

/* Including external dependencies */
#include "RoadworksSubCauseCode.h"
#include "LightBarSirenInUse.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ClosedLanes;

/* RoadWorksContainerBasic */
typedef struct RoadWorksContainerBasic {
	RoadworksSubCauseCode_t	*roadworksSubCauseCode	/* OPTIONAL */;
	LightBarSirenInUse_t	 lightBarSirenInUse;
	struct ClosedLanes	*closedLanes	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadWorksContainerBasic_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadWorksContainerBasic;
extern asn_SEQUENCE_specifics_t asn_SPC_RoadWorksContainerBasic_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadWorksContainerBasic_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ClosedLanes.h"

#endif	/* _RoadWorksContainerBasic_H_ */
#include "asn_internal.h"
