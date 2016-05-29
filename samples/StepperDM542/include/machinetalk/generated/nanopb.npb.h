/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.3-dev at Wed May 25 22:29:15 2016. */

#ifndef PB_NANOPB_NPB_H_INCLUDED
#define PB_NANOPB_NPB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _FieldType {
    FieldType_FT_DEFAULT = 0,
    FieldType_FT_CALLBACK = 1,
    FieldType_FT_POINTER = 4,
    FieldType_FT_STATIC = 2,
    FieldType_FT_IGNORE = 3
} FieldType;

typedef enum _IntSize {
    IntSize_IS_DEFAULT = 0,
    IntSize_IS_8 = 8,
    IntSize_IS_16 = 16,
    IntSize_IS_32 = 32,
    IntSize_IS_64 = 64
} IntSize;

/* Struct definitions */
typedef struct _NanoPBOptions {
    bool has_max_size;
    int32_t max_size;
    bool has_max_count;
    int32_t max_count;
    bool has_type;
    FieldType type;
    bool has_long_names;
    bool long_names;
    bool has_packed_struct;
    bool packed_struct;
    bool has_skip_message;
    bool skip_message;
    bool has_int_size;
    IntSize int_size;
    bool has_no_unions;
    bool no_unions;
    bool has_msgid;
    uint32_t msgid;
} NanoPBOptions;

/* Extensions */
extern const pb_extension_type_t nanopb_fileopt; /* field type: NanoPBOptions nanopb_fileopt; */
extern const pb_extension_type_t nanopb_msgopt; /* field type: NanoPBOptions nanopb_msgopt; */
extern const pb_extension_type_t nanopb_enumopt; /* field type: NanoPBOptions nanopb_enumopt; */
extern const pb_extension_type_t nanopb; /* field type: NanoPBOptions nanopb; */

/* Default values for struct fields */
extern const IntSize NanoPBOptions_int_size_default;
extern const FieldType NanoPBOptions_type_default;
extern const bool NanoPBOptions_long_names_default;
extern const bool NanoPBOptions_packed_struct_default;
extern const bool NanoPBOptions_skip_message_default;
extern const bool NanoPBOptions_no_unions_default;

/* Initializer values for message structs */
#define NanoPBOptions_init_default               {false, 0, false, 0, false, FieldType_FT_DEFAULT, false, true, false, false, false, false, false, IntSize_IS_DEFAULT, false, false, false, 0}
#define NanoPBOptions_init_zero                  {false, 0, false, 0, false, (FieldType)0, false, 0, false, 0, false, 0, false, (IntSize)0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define NanoPBOptions_max_size_tag               1
#define NanoPBOptions_max_count_tag              2
#define NanoPBOptions_int_size_tag               7
#define NanoPBOptions_type_tag                   3
#define NanoPBOptions_long_names_tag             4
#define NanoPBOptions_packed_struct_tag          5
#define NanoPBOptions_skip_message_tag           6
#define NanoPBOptions_no_unions_tag              8
#define NanoPBOptions_msgid_tag                  9
#define nanopb_fileopt_tag                       1010
#define nanopb_msgopt_tag                        1010
#define nanopb_enumopt_tag                       1010
#define nanopb_tag                               1010

/* Struct field encoding specification for nanopb */
extern const pb_field_t NanoPBOptions_fields[10];

/* Maximum encoded size of messages (where known) */
#define NanoPBOptions_size                       48

/* helper macros for message type ids if set with */
/* option (nanopb_msgopt).msgid = <id>; */

#ifdef PB_MSGID

#define NANOPB_MESSAGES \

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
