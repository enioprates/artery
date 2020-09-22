/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "IS_TS103301/ISO_TS_19321.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "ISO14823Attributes.h"

static asn_oer_constraints_t asn_OER_type_Member_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_Member_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_ISO14823Attributes_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
asn_per_constraints_t asn_PER_type_ISO14823Attributes_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_Member_2[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.dtm),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DTM,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dtm"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.edt),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EDT,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"edt"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.dfl),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DFL,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dfl"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.ved),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VED,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ved"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.spe),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SPE,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"spe"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.roi),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ROI,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"roi"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ISO14823Attributes__Member, choice.dbv),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DBV,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dbv"
		},
	{ ATF_POINTER, 0, offsetof(struct ISO14823Attributes__Member, choice.ddd),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DDD,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ddd"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_Member_tag2el_2[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* dtm */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* edt */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* dfl */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* ved */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* spe */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* roi */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* dbv */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 } /* ddd */
};
static asn_CHOICE_specifics_t asn_SPC_Member_specs_2 = {
	sizeof(struct ISO14823Attributes__Member),
	offsetof(struct ISO14823Attributes__Member, _asn_ctx),
	offsetof(struct ISO14823Attributes__Member, present),
	sizeof(((struct ISO14823Attributes__Member *)0)->present),
	asn_MAP_Member_tag2el_2,
	8,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_Member_2 = {
	"CHOICE",
	"CHOICE",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_Member_constr_2, &asn_PER_type_Member_constr_2, CHOICE_constraint },
	asn_MBR_Member_2,
	8,	/* Elements count */
	&asn_SPC_Member_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_ISO14823Attributes_1[] = {
	{ ATF_POINTER, 0, 0,
		-1 /* Ambiguous tag (CHOICE?) */,
		0,
		&asn_DEF_Member_2,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_ISO14823Attributes_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_ISO14823Attributes_specs_1 = {
	sizeof(struct ISO14823Attributes),
	offsetof(struct ISO14823Attributes, _asn_ctx),
	2,	/* XER encoding is XMLValueList */
};
asn_TYPE_descriptor_t asn_DEF_ISO14823Attributes = {
	"ISO14823Attributes",
	"ISO14823Attributes",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_ISO14823Attributes_tags_1,
	sizeof(asn_DEF_ISO14823Attributes_tags_1)
		/sizeof(asn_DEF_ISO14823Attributes_tags_1[0]), /* 1 */
	asn_DEF_ISO14823Attributes_tags_1,	/* Same as above */
	sizeof(asn_DEF_ISO14823Attributes_tags_1)
		/sizeof(asn_DEF_ISO14823Attributes_tags_1[0]), /* 1 */
	{ &asn_OER_type_ISO14823Attributes_constr_1, &asn_PER_type_ISO14823Attributes_constr_1, SEQUENCE_OF_constraint },
	asn_MBR_ISO14823Attributes_1,
	1,	/* Single element */
	&asn_SPC_ISO14823Attributes_specs_1	/* Additional specs */
};

