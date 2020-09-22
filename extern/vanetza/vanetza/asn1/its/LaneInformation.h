/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "IS_TS103301/ISO_TS_19321.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_LaneInformation_H_
#define	_LaneInformation_H_


#include "asn_application.h"

/* Including external dependencies */
#include "LanePosition.h"
#include "Direction.h"
#include "LaneType.h"
#include "LaneStatus.h"
#include "IVILaneWidth.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct DTM;
struct CompleteVehicleCharacteristics;

/* LaneInformation */
typedef struct LaneInformation {
	LanePosition_t	 laneNumber;
	Direction_t	 direction;
	struct DTM	*validity	/* OPTIONAL */;
	LaneType_t	 laneType;
	struct CompleteVehicleCharacteristics	*laneTypeQualifier	/* OPTIONAL */;
	LaneStatus_t	 laneStatus;
	IVILaneWidth_t	*laneWidth	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LaneInformation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LaneInformation;
extern asn_SEQUENCE_specifics_t asn_SPC_LaneInformation_specs_1;
extern asn_TYPE_member_t asn_MBR_LaneInformation_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "DTM.h"
#include "CompleteVehicleCharacteristics.h"

#endif	/* _LaneInformation_H_ */
#include "asn_internal.h"
