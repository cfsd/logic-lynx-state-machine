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

StateMachine::StateMachine(bool verbose, uint32_t id, cluon::OD4Session &od4, cluon::OD4Session &od4Gpio, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Pwm)
    : m_od4(od4)
    , m_od4Gpio(od4Gpio)
    , m_od4Analog(od4Analog)
    , m_od4Pwm(od4Pwm)
    , m_debug(verbose)
    , m_bbbId(id)
    , m_senderStampOffsetGpio(id*1000)
    , m_senderStampOffsetAnalog(id*1000+200)
    , m_senderStampOffsetPwm(id*1000+300)
    , m_initialised()
    , m_pressureEbsAct()
    , m_pressureEbsLine()
    , m_pressureServiceTank()
    , m_pressureServiceReg()
    , m_ebsOk()
    , m_asms()
    , m_heartbeat()
    , m_ebsSpeaker()
    , m_compressor()
    , m_ebsRelief()
    , m_finished()
    , m_shutdown()
    , m_currentState()
    , m_flash2Hz()
    , m_nextFlashTime()
    , m_findRackSeqNo()
    , m_serviceBrakeOk()
    , m_ebsPressureOk()

{
    m_currentState = asState::AS_OFF;
	StateMachine::setUp();
}

StateMachine::~StateMachine() 
{
  StateMachine::tearDown();
}

void StateMachine::body()
{

    stateMachine();
    setAssi(m_currentState);
    m_heartbeat = !m_heartbeat;

    if (m_pressureEbsLine < 6 || m_pressureServiceTank < 6){
        m_compressor = 1;
    }else{
        m_compressor = 0;
    }

    m_serviceBrakeOk = m_pressureServiceTank >= 6;
    m_ebsPressureOk = m_pressureEbsLine >= 6;
    
    // Sending std messages
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    int16_t senderStamp = 0;

    // GPIO Msg
    opendlv::proxy::SwitchStateRequest msgGpio;

    // Heartbeat Msg
    senderStamp = m_gpioPinHeartbeat + m_senderStampOffsetGpio;
    msgGpio.state(m_heartbeat);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    // m_ebsSpeaker Msg
    senderStamp = m_gpioPinEbsSpeaker + m_senderStampOffsetGpio;
    msgGpio.state(m_ebsSpeaker);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    // m_compressor Msg
    senderStamp = m_gpioPinCompressor + m_senderStampOffsetGpio;
    msgGpio.state(m_compressor);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    // m_ebsRelief Msg
    senderStamp = m_gpioPinEbsRelief + m_senderStampOffsetGpio;
    msgGpio.state(m_ebsRelief);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    // m_finished Msg
    senderStamp = m_gpioPinFinished + m_senderStampOffsetGpio;
    msgGpio.state(m_finished);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    // m_shutdown Msg
    senderStamp = m_gpioPinShutdown + m_senderStampOffsetGpio;
    msgGpio.state(m_shutdown);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

}

void StateMachine::stateMachine(){

    //if (!m_ebsOk){
    //    m_currentState = asState::EBS_TRIGGERED;
    //} 

    m_ebsSpeaker = 0;
    m_ebsRelief = 1;
    m_finished = 0;
    m_shutdown = 0;

    switch(m_currentState){
        case asState::AS_OFF:
            if (m_asms /*&& precharge done && steering initialization done && EBS OK && Mission selected && computer ON*/){
                m_currentState = asState::AS_READY;
            }
            break;
        case asState::AS_READY:
            if (true /*&& RES GO signal*/){
                m_currentState = asState::AS_DRIVING;
            }else if(!m_asms){
                m_currentState = asState::AS_OFF;
            }
            break;
        case asState::AS_DRIVING:
            m_ebsRelief = 0;
            if (false /*&& Mission complete and spd = 0*/){
                m_currentState = asState::AS_FINISHED;
            }else if(!m_asms){
                m_currentState = asState::EBS_TRIGGERED;
            }
            break;
        case asState::AS_FINISHED:
            m_finished = 1;
            m_shutdown = 1;
            if(!m_asms){
                m_currentState = asState::AS_OFF;
            }
            break;
        case asState::EBS_TRIGGERED:
            m_ebsRelief = 0;
            m_ebsSpeaker = 1;
            m_shutdown = 1;
            if(!m_asms /*&& spd = 0*/){
                m_currentState = asState::AS_OFF;
            }
            break;

        default:
        break;
    }
}

bool StateMachine::setAssi(asState assi){

    uint32_t redDuty = 0;
    uint32_t greenDuty = 0;
    uint32_t blueDuty = 0;
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
    auto value = tp_ms.time_since_epoch();
    long timeMillis = value.count();
    //std::cout << "Time" << timeMillis << std::endl;

    if (m_nextFlashTime <= timeMillis){
        m_flash2Hz  = !m_flash2Hz;
        m_nextFlashTime = timeMillis + 250;
    }

    //std::cout << "2Hz" << m_flash2Hz << std::endl;

    switch(assi){
        case asState::AS_OFF:
        break;
        case asState::AS_READY:
            blueDuty = 0;
            greenDuty = 50000;
            redDuty = 50000;
            break;
        case asState::AS_DRIVING:
            blueDuty = 0;
            greenDuty = 50000*m_flash2Hz;
            redDuty = 50000*m_flash2Hz;
            break;
        case asState::AS_FINISHED:
            blueDuty = 50000;
            greenDuty = 0;
            redDuty = 0;
            break;
        case asState::EBS_TRIGGERED:
            blueDuty = 50000*m_flash2Hz;
            greenDuty = 0;
            redDuty = 0;
            break;

        default:
        break;
    }

    // Send pwm Requests

    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    int16_t senderStamp = 0;
    
    opendlv::proxy::PulseWidthModulationRequest msgPwm;
 
    senderStamp = m_pwmPinAssiRed + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(redDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);

    senderStamp = m_pwmPinAssiGreen + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(greenDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);

    senderStamp = m_pwmPinAssiBlue + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(blueDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);

    return 0;

}

void StateMachine::setUp()
{
  m_initialised = true;
}

void StateMachine::tearDown()
{
}

uint16_t StateMachine::getGpioPinEbsOk(){
  return m_gpioPinEbsOk;
}

uint16_t StateMachine::getGpioPinAsms(){
  return m_gpioPinAsms;
}

uint16_t StateMachine::getAnalogPinEbsLine(){
  return m_analogPinEbsLine;
}

uint16_t StateMachine::getAnalogPinServiceTank(){
  return m_analogPinServiceTank;
}

uint16_t StateMachine::getAnalogPinEbsActuator(){
  return m_analogPinEbsActuator;
}

uint16_t StateMachine::getAnalogPinPressureReg(){
  return m_analogPinPressureReg;
}

uint32_t StateMachine::getSenderStampOffsetGpio(){
  return m_senderStampOffsetGpio;
}

uint32_t StateMachine::getSenderStampOffsetAnalog(){
  return m_senderStampOffsetAnalog;
}

void StateMachine::setPressureEbsAct(float pos){
    m_pressureEbsAct = pos;
}
void StateMachine::setPressureEbsLine(float pos){
    m_pressureEbsLine = pos;
}
void StateMachine::setPressureServiceTank(float pos){
    m_pressureServiceTank = pos;
}
void StateMachine::setPressureServiceReg(float pos){
    m_pressureServiceReg = pos;
}

void StateMachine::setEbsOk(bool state){
    m_ebsOk = state;
}
void StateMachine::setAsms(bool state){
    m_asms = state;
}

bool StateMachine::getInitialised(){
  return m_initialised;
}