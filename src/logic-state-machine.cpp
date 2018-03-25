/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "logic-state-machine.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>
//#include <stdlib.h>

float StateMachine::decode(const std::string &data) noexcept {
    std::cout << "[UDP] Got data:" << data << std::endl;
    float temp = std::stof(data);
    return temp;
}

StateMachine::StateMachine(bool verbose, uint32_t id)
    : m_debug(verbose)
    , m_bbbId(id)
    , m_senderStampOffsetGpio(id*1000)
    , m_senderStampOffsetAnalog(id*1000+200)
    , m_senderStampOffsetPwm(id*1000+300)
    , m_initialised()
    , m_pressureEbsAct()
    , m_pressureEbsLine()
    , m_pressureServiceTank()
    , m_pressureServiceReg()
    , m_heartbeat()
    , m_asms()

{
	StateMachine::setUp();
}

StateMachine::~StateMachine() 
{
  StateMachine::tearDown();
}

void StateMachine::callOnReceive(cluon::data::Envelope data){
    if (!m_initialised) {
        return;
    }
   if (data.dataType() == static_cast<int32_t>(opendlv::proxy::VoltageReading::ID())) {

        if (data.senderStamp()- m_senderStampOffsetAnalog == m_analogPinPressureEbsAct){
          opendlv::proxy::VoltageReading analogInput = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(data));
          m_pressureEbsAct = analogInput.torque()/((float) m_analogConvPressureEbsAct)-((float) m_analogOffsetPressureEbsAct);
          if(m_debug)
             std::cout << "[LOGIC-ASS-PRESSURE-EBS-ACT] Pressure reading:" << m_pressureEbsAct << std::endl;
        }else if (data.senderStamp() - m_senderStampOffsetAnalog == m_analogPinPressureEbsLine){
          opendlv::proxy::VoltageReading analogInput = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(data));
          m_pressureEbsLine = analogInput.torque()/((float) m_analogConvPressureEbsLine)-((float) m_analogOffsetPressureEbsLine);
          if(m_debug)
             std::cout << "[LOGIC-ASS-PRESSURE-EBS-LINE] Pressure reading:" << m_pressureEbsLine << std::endl;
        }else if (data.senderStamp() - m_senderStampOffsetAnalog == m_analogPinPressureServiceTank){
          opendlv::proxy::VoltageReading analogInput = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(data));
          m_pressureServiceTank = analogInput.torque()/((float) m_analogConvPressureServiceTank)-((float) m_analogOffsetPressureServiceTank);
          if(m_debug)
             std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-TANK] Pressure reading:" << m_pressureServiceTank << std::endl;
        }else if (data.senderStamp() - m_senderStampOffsetAnalog == m_analogPinPressureServiceReg){
          opendlv::proxy::VoltageReading analogInput = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(data));
          m_pressureServiceReg = analogInput.torque()/((float) m_analogConvPressureServiceReg)-((float) m_analogOffsetPressureServiceReg);
          if(m_debug)
             std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-REGULATOR] Pressure reading:" << m_pressureServiceReg << std::endl;
        }
    }else if (data.dataType() == static_cast<int32_t>(opendlv::proxy::SwitchStateReading::ID())) {
        uint16_t pin = data.senderStamp()-m_senderStampOffsetGpio;
        if (pin == m_gpioPinAsms){
            opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
            m_asms = gpioState.state();
        }
     }
}

void StateMachine::body(cluon::OD4Session &od4)
{

    m_heartbeat = !m_heartbeat;

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    int16_t senderStamp = 0;

    opendlv::proxy::SwitchStateRequest msgGpio;

    senderStamp = m_gpioPinHeartbeat + m_senderStampOffsetGpio;
    msgGpio.state(m_heartbeat);
	od4.send(msgGpio, sampleTime, senderStamp);
    
}

void StateMachine::setUp()
{
  m_initialised = true;
}

void StateMachine::tearDown()
{
}
