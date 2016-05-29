/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.3-dev at Wed May 25 22:29:17 2016. */

#include <machinetalk/generated/preview.npb.h>
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const pb_KinematicsType pb_Preview_kins_default = pb_KinematicsType_KT_JOINT;


const pb_field_t pb_Position_fields[10] = {
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , FIRST, pb_Position, x, x, 0),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, y, x, 0),
    PB_FIELD(  5, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, z, y, 0),
    PB_FIELD(  6, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, a, z, 0),
    PB_FIELD(  7, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, b, a, 0),
    PB_FIELD(  8, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, c, b, 0),
    PB_FIELD(  9, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, u, c, 0),
    PB_FIELD( 10, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, v, u, 0),
    PB_FIELD( 11, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Position, w, v, 0),
    PB_LAST_FIELD
};

const pb_field_t pb_Preview_fields[27] = {
    PB_FIELD(  1, ENUM    , REQUIRED, STATIC  , FIRST, pb_Preview, type, type, 0),
    PB_FIELD(  2, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, line_number, type, 0),
    PB_FIELD(  3, MESSAGE , OPTIONAL, STATIC  , OTHER, pb_Preview, pos, line_number, &pb_Position_fields),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, first_end, pos, 0),
    PB_FIELD(  5, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, second_end, first_end, 0),
    PB_FIELD(  6, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, first_axis, second_end, 0),
    PB_FIELD(  7, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, second_axis, first_axis, 0),
    PB_FIELD(  8, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, rotation, second_axis, 0),
    PB_FIELD(  9, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, axis_end_point, rotation, 0),
    PB_FIELD( 10, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, xy_rotation, axis_end_point, 0),
    PB_FIELD( 11, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, plane, xy_rotation, 0),
    PB_FIELD( 12, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, rate, plane, 0),
    PB_FIELD( 13, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, feed_mode, rate, 0),
    PB_FIELD( 14, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, time, feed_mode, 0),
    PB_FIELD( 15, STRING  , OPTIONAL, CALLBACK, OTHER, pb_Preview, text, time, 0),
    PB_FIELD(101, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, angular_units, text, 0),
    PB_FIELD(102, DOUBLE  , OPTIONAL, STATIC  , OTHER, pb_Preview, length_units, angular_units, 0),
    PB_FIELD(103, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, probetype, length_units, 0),
    PB_FIELD(104, ENUM    , OPTIONAL, STATIC  , OTHER, pb_Preview, kins, probetype, &pb_Preview_kins_default),
    PB_FIELD(105, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, axismask, kins, 0),
    PB_FIELD(106, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, g5_index, axismask, 0),
    PB_FIELD(107, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, pocket, g5_index, 0),
    PB_FIELD(110, ENUM    , OPTIONAL, STATIC  , OTHER, pb_Preview, stype, pocket, 0),
    PB_FIELD(111, STRING  , OPTIONAL, CALLBACK, OTHER, pb_Preview, filename, stype, 0),
    PB_FIELD(112, STRING  , OPTIONAL, CALLBACK, OTHER, pb_Preview, cmdstring, filename, 0),
    PB_FIELD(113, INT32   , OPTIONAL, STATIC  , OTHER, pb_Preview, call_level, cmdstring, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(pb_Preview, pos) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_pb_Position_pb_Preview)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(pb_Preview, pos) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_pb_Position_pb_Preview)
#endif


/* On some platforms (such as AVR), double is really float.
 * These are not directly supported by nanopb, but see example_avr_double.
 * To get rid of this error, remove any double fields from your .proto.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)

