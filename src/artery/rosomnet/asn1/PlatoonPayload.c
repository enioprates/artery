/*
 * Generated by asn1c-0.9.28 (http://lionet.info/asn1c)
 * From ASN.1 module "RAW"
 * 	found in "RAW.asn"
 */

#include "PlatoonPayload.h"

static asn_TYPE_member_t asn_MBR_PlatoonPayload_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, carX),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_CarID,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"carX"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, rawstationID),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_RawStationID,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rawstationID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, rawsteeringWheelAngle),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_RawSteeringWheelAngle,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rawsteeringWheelAngle"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, rawheading),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_RawHeading,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rawheading"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, rawspeed),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_RawSpeed,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"rawspeed"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, xxCoords),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_XX,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"xxCoords"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, yyCoords),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_YY,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"yyCoords"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PlatoonPayload, zzCoords),
		(ASN_TAG_CLASS_UNIVERSAL | (2 << 2)),
		0,
		&asn_DEF_ZZ,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"zzCoords"
		},
};
static const ber_tlv_tag_t asn_DEF_PlatoonPayload_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PlatoonPayload_tag2el_1[] = {
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 0, 0, 7 }, /* carX */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 1, -1, 6 }, /* rawstationID */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 2, -2, 5 }, /* rawsteeringWheelAngle */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 3, -3, 4 }, /* rawheading */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 4, -4, 3 }, /* rawspeed */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 5, -5, 2 }, /* xxCoords */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 6, -6, 1 }, /* yyCoords */
    { (ASN_TAG_CLASS_UNIVERSAL | (2 << 2)), 7, -7, 0 } /* zzCoords */
};
static asn_SEQUENCE_specifics_t asn_SPC_PlatoonPayload_specs_1 = {
	sizeof(struct PlatoonPayload),
	offsetof(struct PlatoonPayload, _asn_ctx),
	asn_MAP_PlatoonPayload_tag2el_1,
	8,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_PlatoonPayload = {
	"PlatoonPayload",
	"PlatoonPayload",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_PlatoonPayload_tags_1,
	sizeof(asn_DEF_PlatoonPayload_tags_1)
		/sizeof(asn_DEF_PlatoonPayload_tags_1[0]), /* 1 */
	asn_DEF_PlatoonPayload_tags_1,	/* Same as above */
	sizeof(asn_DEF_PlatoonPayload_tags_1)
		/sizeof(asn_DEF_PlatoonPayload_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_PlatoonPayload_1,
	8,	/* Elements count */
	&asn_SPC_PlatoonPayload_specs_1	/* Additional specs */
};

