/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.3-dev at Wed May 25 22:29:15 2016. */

#include <machinetalk/generated/nanopb.npb.h>
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const IntSize NanoPBOptions_int_size_default = IntSize_IS_DEFAULT;
const FieldType NanoPBOptions_type_default = FieldType_FT_DEFAULT;
const bool NanoPBOptions_long_names_default = true;
const bool NanoPBOptions_packed_struct_default = false;
const bool NanoPBOptions_skip_message_default = false;
const bool NanoPBOptions_no_unions_default = false;


const pb_field_t NanoPBOptions_fields[10] = {
    PB_FIELD(  1, INT32   , OPTIONAL, STATIC  , FIRST, NanoPBOptions, max_size, max_size, 0),
    PB_FIELD(  2, INT32   , OPTIONAL, STATIC  , OTHER, NanoPBOptions, max_count, max_size, 0),
    PB_FIELD(  3, ENUM    , OPTIONAL, STATIC  , OTHER, NanoPBOptions, type, max_count, &NanoPBOptions_type_default),
    PB_FIELD(  4, BOOL    , OPTIONAL, STATIC  , OTHER, NanoPBOptions, long_names, type, &NanoPBOptions_long_names_default),
    PB_FIELD(  5, BOOL    , OPTIONAL, STATIC  , OTHER, NanoPBOptions, packed_struct, long_names, &NanoPBOptions_packed_struct_default),
    PB_FIELD(  6, BOOL    , OPTIONAL, STATIC  , OTHER, NanoPBOptions, skip_message, packed_struct, &NanoPBOptions_skip_message_default),
    PB_FIELD(  7, ENUM    , OPTIONAL, STATIC  , OTHER, NanoPBOptions, int_size, skip_message, &NanoPBOptions_int_size_default),
    PB_FIELD(  8, BOOL    , OPTIONAL, STATIC  , OTHER, NanoPBOptions, no_unions, int_size, &NanoPBOptions_no_unions_default),
    PB_FIELD(  9, UINT32  , OPTIONAL, STATIC  , OTHER, NanoPBOptions, msgid, no_unions, 0),
    PB_LAST_FIELD
};

typedef struct {
    NanoPBOptions nanopb_fileopt;
} nanopb_fileopt_struct;

static const pb_field_t nanopb_fileopt_field = 
      PB_FIELD(1010, MESSAGE , OPTEXT, STATIC  , FIRST, nanopb_fileopt_struct, nanopb_fileopt, nanopb_fileopt, &NanoPBOptions_fields);

const pb_extension_type_t nanopb_fileopt = {
    NULL,
    NULL,
    &nanopb_fileopt_field
};

typedef struct {
    NanoPBOptions nanopb_msgopt;
} nanopb_msgopt_struct;

static const pb_field_t nanopb_msgopt_field = 
      PB_FIELD(1010, MESSAGE , OPTEXT, STATIC  , FIRST, nanopb_msgopt_struct, nanopb_msgopt, nanopb_msgopt, &NanoPBOptions_fields);

const pb_extension_type_t nanopb_msgopt = {
    NULL,
    NULL,
    &nanopb_msgopt_field
};

typedef struct {
    NanoPBOptions nanopb_enumopt;
} nanopb_enumopt_struct;

static const pb_field_t nanopb_enumopt_field = 
      PB_FIELD(1010, MESSAGE , OPTEXT, STATIC  , FIRST, nanopb_enumopt_struct, nanopb_enumopt, nanopb_enumopt, &NanoPBOptions_fields);

const pb_extension_type_t nanopb_enumopt = {
    NULL,
    NULL,
    &nanopb_enumopt_field
};

typedef struct {
    NanoPBOptions nanopb;
} nanopb_struct;

static const pb_field_t nanopb_field = 
      PB_FIELD(1010, MESSAGE , OPTEXT, STATIC  , FIRST, nanopb_struct, nanopb, nanopb, &NanoPBOptions_fields);

const pb_extension_type_t nanopb = {
    NULL,
    NULL,
    &nanopb_field
};


