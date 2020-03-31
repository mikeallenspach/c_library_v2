#pragma once
// MESSAGE MPC_COMMAND PACKING

#define MAVLINK_MSG_ID_MPC_COMMAND 229

MAVPACKED(
typedef struct __mavlink_mpc_command_t {
 uint64_t time_usec; /*< [us] */
 int32_t flag; /*<  */
 float thrust; /*<   */
 float tilt_angle; /*<   */
 float torque_x; /*<   */
 float torque_y; /*<   */
 float torque_z; /*<   */
}) mavlink_mpc_command_t;

#define MAVLINK_MSG_ID_MPC_COMMAND_LEN 32
#define MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN 32
#define MAVLINK_MSG_ID_229_LEN 32
#define MAVLINK_MSG_ID_229_MIN_LEN 32

#define MAVLINK_MSG_ID_MPC_COMMAND_CRC 0
#define MAVLINK_MSG_ID_229_CRC 0



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MPC_COMMAND { \
    229, \
    "MPC_COMMAND", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mpc_command_t, time_usec) }, \
         { "flag", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mpc_command_t, flag) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mpc_command_t, thrust) }, \
         { "tilt_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mpc_command_t, tilt_angle) }, \
         { "torque_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mpc_command_t, torque_x) }, \
         { "torque_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mpc_command_t, torque_y) }, \
         { "torque_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mpc_command_t, torque_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MPC_COMMAND { \
    "MPC_COMMAND", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mpc_command_t, time_usec) }, \
         { "flag", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mpc_command_t, flag) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mpc_command_t, thrust) }, \
         { "tilt_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mpc_command_t, tilt_angle) }, \
         { "torque_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mpc_command_t, torque_x) }, \
         { "torque_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mpc_command_t, torque_y) }, \
         { "torque_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mpc_command_t, torque_z) }, \
         } \
}
#endif

/**
 * @brief Pack a mpc_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] 
 * @param flag  
 * @param thrust   
 * @param tilt_angle   
 * @param torque_x   
 * @param torque_y   
 * @param torque_z   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mpc_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, int32_t flag, float thrust, float tilt_angle, float torque_x, float torque_y, float torque_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPC_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, flag);
    _mav_put_float(buf, 12, thrust);
    _mav_put_float(buf, 16, tilt_angle);
    _mav_put_float(buf, 20, torque_x);
    _mav_put_float(buf, 24, torque_y);
    _mav_put_float(buf, 28, torque_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPC_COMMAND_LEN);
#else
    mavlink_mpc_command_t packet;
    packet.time_usec = time_usec;
    packet.flag = flag;
    packet.thrust = thrust;
    packet.tilt_angle = tilt_angle;
    packet.torque_x = torque_x;
    packet.torque_y = torque_y;
    packet.torque_z = torque_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPC_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPC_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
}

/**
 * @brief Pack a mpc_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] 
 * @param flag  
 * @param thrust   
 * @param tilt_angle   
 * @param torque_x   
 * @param torque_y   
 * @param torque_z   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mpc_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,int32_t flag,float thrust,float tilt_angle,float torque_x,float torque_y,float torque_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPC_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, flag);
    _mav_put_float(buf, 12, thrust);
    _mav_put_float(buf, 16, tilt_angle);
    _mav_put_float(buf, 20, torque_x);
    _mav_put_float(buf, 24, torque_y);
    _mav_put_float(buf, 28, torque_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPC_COMMAND_LEN);
#else
    mavlink_mpc_command_t packet;
    packet.time_usec = time_usec;
    packet.flag = flag;
    packet.thrust = thrust;
    packet.tilt_angle = tilt_angle;
    packet.torque_x = torque_x;
    packet.torque_y = torque_y;
    packet.torque_z = torque_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPC_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPC_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
}

/**
 * @brief Encode a mpc_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mpc_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mpc_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mpc_command_t* mpc_command)
{
    return mavlink_msg_mpc_command_pack(system_id, component_id, msg, mpc_command->time_usec, mpc_command->flag, mpc_command->thrust, mpc_command->tilt_angle, mpc_command->torque_x, mpc_command->torque_y, mpc_command->torque_z);
}

/**
 * @brief Encode a mpc_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mpc_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mpc_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mpc_command_t* mpc_command)
{
    return mavlink_msg_mpc_command_pack_chan(system_id, component_id, chan, msg, mpc_command->time_usec, mpc_command->flag, mpc_command->thrust, mpc_command->tilt_angle, mpc_command->torque_x, mpc_command->torque_y, mpc_command->torque_z);
}

/**
 * @brief Send a mpc_command message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] 
 * @param flag  
 * @param thrust   
 * @param tilt_angle   
 * @param torque_x   
 * @param torque_y   
 * @param torque_z   
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mpc_command_send(mavlink_channel_t chan, uint64_t time_usec, int32_t flag, float thrust, float tilt_angle, float torque_x, float torque_y, float torque_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPC_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, flag);
    _mav_put_float(buf, 12, thrust);
    _mav_put_float(buf, 16, tilt_angle);
    _mav_put_float(buf, 20, torque_x);
    _mav_put_float(buf, 24, torque_y);
    _mav_put_float(buf, 28, torque_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_COMMAND, buf, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
#else
    mavlink_mpc_command_t packet;
    packet.time_usec = time_usec;
    packet.flag = flag;
    packet.thrust = thrust;
    packet.tilt_angle = tilt_angle;
    packet.torque_x = torque_x;
    packet.torque_y = torque_y;
    packet.torque_z = torque_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
#endif
}

/**
 * @brief Send a mpc_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mpc_command_send_struct(mavlink_channel_t chan, const mavlink_mpc_command_t* mpc_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mpc_command_send(chan, mpc_command->time_usec, mpc_command->flag, mpc_command->thrust, mpc_command->tilt_angle, mpc_command->torque_x, mpc_command->torque_y, mpc_command->torque_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_COMMAND, (const char *)mpc_command, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_MPC_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mpc_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int32_t flag, float thrust, float tilt_angle, float torque_x, float torque_y, float torque_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, flag);
    _mav_put_float(buf, 12, thrust);
    _mav_put_float(buf, 16, tilt_angle);
    _mav_put_float(buf, 20, torque_x);
    _mav_put_float(buf, 24, torque_y);
    _mav_put_float(buf, 28, torque_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_COMMAND, buf, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
#else
    mavlink_mpc_command_t *packet = (mavlink_mpc_command_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->flag = flag;
    packet->thrust = thrust;
    packet->tilt_angle = tilt_angle;
    packet->torque_x = torque_x;
    packet->torque_y = torque_y;
    packet->torque_z = torque_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_COMMAND, (const char *)packet, MAVLINK_MSG_ID_MPC_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MPC_COMMAND_LEN, MAVLINK_MSG_ID_MPC_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE MPC_COMMAND UNPACKING


/**
 * @brief Get field time_usec from mpc_command message
 *
 * @return [us] 
 */
static inline uint64_t mavlink_msg_mpc_command_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field flag from mpc_command message
 *
 * @return  
 */
static inline int32_t mavlink_msg_mpc_command_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field thrust from mpc_command message
 *
 * @return   
 */
static inline float mavlink_msg_mpc_command_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field tilt_angle from mpc_command message
 *
 * @return   
 */
static inline float mavlink_msg_mpc_command_get_tilt_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field torque_x from mpc_command message
 *
 * @return   
 */
static inline float mavlink_msg_mpc_command_get_torque_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field torque_y from mpc_command message
 *
 * @return   
 */
static inline float mavlink_msg_mpc_command_get_torque_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field torque_z from mpc_command message
 *
 * @return   
 */
static inline float mavlink_msg_mpc_command_get_torque_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a mpc_command message into a struct
 *
 * @param msg The message to decode
 * @param mpc_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_mpc_command_decode(const mavlink_message_t* msg, mavlink_mpc_command_t* mpc_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mpc_command->time_usec = mavlink_msg_mpc_command_get_time_usec(msg);
    mpc_command->flag = mavlink_msg_mpc_command_get_flag(msg);
    mpc_command->thrust = mavlink_msg_mpc_command_get_thrust(msg);
    mpc_command->tilt_angle = mavlink_msg_mpc_command_get_tilt_angle(msg);
    mpc_command->torque_x = mavlink_msg_mpc_command_get_torque_x(msg);
    mpc_command->torque_y = mavlink_msg_mpc_command_get_torque_y(msg);
    mpc_command->torque_z = mavlink_msg_mpc_command_get_torque_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MPC_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_MPC_COMMAND_LEN;
        memset(mpc_command, 0, MAVLINK_MSG_ID_MPC_COMMAND_LEN);
    memcpy(mpc_command, _MAV_PAYLOAD(msg), len);
#endif
}
