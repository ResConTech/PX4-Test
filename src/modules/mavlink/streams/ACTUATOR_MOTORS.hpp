/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef ACTUATOR_MOTORS_HPP
#define ACTUATOR_MOTORS_HPP

#include <uORB/topics/actuator_motors.h>


class MavlinkStreamActuatorMotors : public MavlinkStream
{
public:
    const char *get_name() const
    {
        return MavlinkStreamActuatorMotors::get_name_static();
    }
    static const char *get_name_static()
    {
        return "ACTUATOR_MOTORS";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_ACTUATOR_MOTORS;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamActuatorMotors(mavlink);
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::Subscription _sub{ORB_ID(actuator_motors)};

    /* do not allow top copying this class */
    MavlinkStreamActuatorMotors(MavlinkStreamActuatorMotors &);
    MavlinkStreamActuatorMotors& operator = (const MavlinkStreamActuatorMotors &);

protected:
    explicit MavlinkStreamActuatorMotors(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
        struct actuator_motors_s _actuator_motors;    //make sure actuator_motors_s is the definition of your uORB topic

        if (_sub.update(&_actuator_motors)) {
            mavlink_actuator_motors_t _msg_actuator_motors;  //make sure mavlink_actuator_motors_t is the definition of your custom MAVLink message

            _msg_actuator_motors.time_boot_ms = _actuator_motors.timestamp;
            _msg_actuator_motors.num = _actuator_motors.NUM_CONTROLS;
            _msg_actuator_motors.thrust_1_raw  = _actuator_motors.control[0];
            _msg_actuator_motors.thrust_2_raw  = _actuator_motors.control[1];
            _msg_actuator_motors.thrust_3_raw  = _actuator_motors.control[2];
            _msg_actuator_motors.thrust_4_raw  = _actuator_motors.control[3];
            _msg_actuator_motors.thrust_5_raw  = _actuator_motors.control[4];
            _msg_actuator_motors.thrust_6_raw  = _actuator_motors.control[5];
            _msg_actuator_motors.thrust_7_raw  = _actuator_motors.control[6];
            _msg_actuator_motors.thrust_8_raw  = _actuator_motors.control[7]; 

            mavlink_msg_actuator_motors_send_struct(_mavlink->get_channel(), &_msg_actuator_motors);
            
            return true;
        }

        return false;
    }
};


#endif // ACTUATOR_MOTORS_HPP


