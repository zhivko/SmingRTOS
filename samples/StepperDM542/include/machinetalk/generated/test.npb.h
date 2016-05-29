/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.3-dev at Wed May 25 22:29:17 2016. */

#ifndef PB_TEST_NPB_H_INCLUDED
#define PB_TEST_NPB_H_INCLUDED
#include <pb.h>

#include <machinetalk/generated/emcclass.npb.h>
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _pb_TestOpType {
    pb_TestOpType_LINE = 10,
    pb_TestOpType_CIRCLE = 20
} pb_TestOpType;

/* Struct definitions */
typedef struct _pb_Test2 {
    uint8_t dummy_field;
} pb_Test2;

typedef struct _pb_Test3 {
    uint8_t dummy_field;
} pb_Test3;

typedef struct _pb_Test1 {
    pb_TestOpType op;
    pb_EmcPose end;
    bool has_center;
    pb_PmCartesian center;
    bool has_normal;
    pb_PmCartesian normal;
    bool has_turn;
    int32_t turn;
} pb_Test1;

/* Default values for struct fields */

/* Initializer values for message structs */
#define pb_Test1_init_default                    {(pb_TestOpType)0, pb_EmcPose_init_default, false, pb_PmCartesian_init_default, false, pb_PmCartesian_init_default, false, 0}
#define pb_Test2_init_default                    {0}
#define pb_Test3_init_default                    {0}
#define pb_Test1_init_zero                       {(pb_TestOpType)0, pb_EmcPose_init_zero, false, pb_PmCartesian_init_zero, false, pb_PmCartesian_init_zero, false, 0}
#define pb_Test2_init_zero                       {0}
#define pb_Test3_init_zero                       {0}

/* Field tags (for use in manual encoding/decoding) */
#define pb_Test1_op_tag                          10
#define pb_Test1_end_tag                         20
#define pb_Test1_center_tag                      30
#define pb_Test1_normal_tag                      40
#define pb_Test1_turn_tag                        50

/* Struct field encoding specification for nanopb */
extern const pb_field_t pb_Test1_fields[6];
extern const pb_field_t pb_Test2_fields[1];
extern const pb_field_t pb_Test3_fields[1];

/* Maximum encoded size of messages (where known) */
#define pb_Test1_size                            (39 + pb_EmcPose_size + pb_PmCartesian_size + pb_PmCartesian_size)
#define pb_Test2_size                            0
#define pb_Test3_size                            0

/* helper macros for message type ids if set with */
/* option (nanopb_msgopt).msgid = <id>; */

#ifdef PB_MSGID
#define PB_MSG_1300 pb_Test1
#define PB_MSG_1301 pb_Test2
#define PB_MSG_1302 pb_Test3

#define TEST_MESSAGES \
	PB_MSG(1300,(39 + pb_EmcPose_size + pb_PmCartesian_size + pb_PmCartesian_size),pb_Test1) \
	PB_MSG(1301,0,pb_Test2) \
	PB_MSG(1302,0,pb_Test3) \

#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
