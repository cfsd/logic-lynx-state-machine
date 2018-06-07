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
    , m_serviceBrake()
    , m_ebsSpeakerOld()
    , m_compressorOld()
    , m_ebsReliefOld()
    , m_finishedOld()
    , m_shutdownOld()
    , m_serviceBrakeOld()
    , m_blueDuty()
    , m_greenDuty()
    , m_redDuty()
    , m_blueDutyOld()
    , m_greenDutyOld()
    , m_redDutyOld()
    , m_currentState()
    , m_prevState()
    , m_flash2Hz()
    , m_nextFlashTime()
    , m_findRackSeqNo()
    , m_serviceBrakeOk()
    , m_ebsPressureOk()
    , m_clampExtended()
    , m_ebsTriggeredTime()
    , m_goSignal()
    , m_finishSignal()
    , m_first(1)

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
    if(m_first){
          sleep(1);
          m_first = false;
    }
    if (m_debug){
        std::cout << "[ASS-Machine] Current inputs: m_asms: " << m_asms << "\t m_ebsOk: " << m_ebsOk << std::endl;
    }
    stateMachine();
    setAssi(m_currentState);
    m_heartbeat = !m_heartbeat;
    m_serviceBrake = 1;

    if (m_pressureEbsLine < 5 || m_pressureServiceTank < 6){
        m_compressor = 1;
    }else if ((m_pressureEbsLine > 6 && m_pressureServiceTank > 8) || m_pressureServiceTank > 9){
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
    //senderStamp = m_gpioPinHeartbeat + m_senderStampOffsetGpio;
    //msgGpio.state(m_heartbeat);
	//m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    if (m_debug){
        std::cout << "[ASS-Machine] Current outputs: m_finished: " << m_finished << "\t m_shutdown: " << m_shutdown << std::endl;
    }
    // m_ebsSpeaker Msg
    if(m_ebsSpeaker != m_ebsSpeakerOld){
        senderStamp = m_gpioPinEbsSpeaker + m_senderStampOffsetGpio;
        msgGpio.state(m_ebsSpeaker);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_ebsSpeakerOld = m_ebsSpeaker;
    }

    // m_compressor Msg
    if(m_compressor != m_compressorOld){
        senderStamp = m_gpioPinCompressor + m_senderStampOffsetGpio;
        msgGpio.state(m_compressor);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_compressorOld = m_compressor;
    }

    // m_ebsRelief Msg
    if(m_ebsRelief != m_ebsReliefOld){
        senderStamp = m_gpioPinEbsRelief + m_senderStampOffsetGpio;
        msgGpio.state(m_ebsRelief);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_ebsReliefOld = m_ebsRelief;
    }
    // m_finished Msg
    if(m_finished != m_finishedOld){
        senderStamp = m_gpioPinFinished + m_senderStampOffsetGpio;
        msgGpio.state(m_finished);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_finishedOld = m_finished;
    }

    // m_shutdown Msg
    if(m_shutdown != m_shutdownOld){
        senderStamp = m_gpioPinShutdown + m_senderStampOffsetGpio;
        msgGpio.state(m_shutdown);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_shutdownOld = m_shutdown;
    }

    // m_serviceBrake
    if(m_serviceBrake != m_serviceBrakeOld){
        senderStamp = m_gpioPinServiceBrake + m_senderStampOffsetGpio;
        msgGpio.state(m_serviceBrake);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_serviceBrakeOld = m_serviceBrake;
    }


    //Send Current state of state machine
        
    opendlv::proxy::SwitchStateReading msgGpioRead;
    senderStamp = m_senderStampCurrentState;
    msgGpioRead.state((uint16_t) m_currentState);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);
}

void StateMachine::stateMachine(){

    //if (!m_ebsOk){
    //    m_currentState = asState::EBS_TRIGGERED;
    //} 

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
    auto value = tp_ms.time_since_epoch();
    uint64_t timeMillis = value.count();

    m_ebsSpeaker = 0;
    m_ebsRelief = !m_asms;
    m_finished = 1; // Change this!!!! (Later...)
    m_shutdown = 0;
    if(!m_ebsOk && m_currentState != asState::EBS_TRIGGERED && m_currentState != asState::AS_OFF){
        m_ebsTriggeredTime = timeMillis;
        m_prevState = m_currentState;
        m_currentState = asState::EBS_TRIGGERED;
        std::cout << "[ASS-Machine] Current state: " << m_currentState << std::endl;
        
    }

    switch(m_currentState){
        case asState::AS_OFF:
            if (m_asms && m_serviceBrakeOk && m_ebsPressureOk && m_clampExtended/*&& precharge done && steering initialization done && EBS OK && Mission selected && computer ON*/){
                m_prevState = asState::AS_OFF;
                m_currentState = asState::AS_READY;
            }
            break;
        case asState::AS_READY:
            if (m_goSignal /*&& RES GO signal*/){
                m_prevState = asState::AS_READY;
                m_currentState = asState::AS_DRIVING;
            }else if(!m_asms){
                m_prevState = asState::AS_READY;
                m_currentState = asState::AS_OFF;
            }
            break;
        case asState::AS_DRIVING:
            m_ebsRelief = 0;
            if (m_finishSignal /*&& Mission complete and spd = 0*/){
                m_prevState = asState::AS_DRIVING;
                m_currentState = asState::AS_FINISHED;
            }else if(!m_asms){
                m_ebsTriggeredTime = timeMillis;
                m_prevState = asState::AS_DRIVING;
                m_currentState = asState::EBS_TRIGGERED;
            }
            break;
        case asState::AS_FINISHED:
            m_finished = 1;
            m_shutdown = 1;
            if(!m_asms){
                m_prevState = asState::AS_FINISHED;
                m_currentState = asState::AS_OFF;
            }
            break;
        case asState::EBS_TRIGGERED:
            m_ebsRelief = 0;
            m_ebsSpeaker = ((m_ebsTriggeredTime+5000) >= timeMillis);
            m_finished = 0;
            m_shutdown = 1;
            if(!m_asms && (((m_ebsTriggeredTime+5000) <= timeMillis) || (m_prevState != asState::AS_DRIVING))/*&& spd = 0*/){
                m_prevState = asState::EBS_TRIGGERED;
                m_currentState = asState::AS_OFF;
            }
            break;

        default:
        break;
    }

    if (m_debug){
        std::cout << "[ASS-Machine] Current state: " << m_currentState << std::endl;
    }

}

bool StateMachine::setAssi(asState assi){

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
    auto value = tp_ms.time_since_epoch();
    uint64_t timeMillis = value.count();

    if (m_nextFlashTime <= timeMillis){
        m_flash2Hz  = !m_flash2Hz;
        m_nextFlashTime = timeMillis + 250;
    }

    //

    if (m_debug){
        std::cout << "[ASSI-Time] Time: " << timeMillis << "ms \t2Hz toogle:" << m_flash2Hz << std::endl;
    }


    switch(assi){
        case asState::AS_OFF:
            m_blueDuty = 0;
            m_greenDuty = 0;
            m_redDuty = 0;
        break;
        case asState::AS_READY:
            m_blueDuty = 0;
            m_greenDuty = 50000;
            m_redDuty = 50000;
            break;
        case asState::AS_DRIVING:
            m_blueDuty = 0;
            m_greenDuty = 50000*m_flash2Hz;
            m_redDuty = 50000*m_flash2Hz;
            break;
        case asState::AS_FINISHED:
            m_blueDuty = 50000;
            m_greenDuty = 0;
            m_redDuty = 0;
            break;
        case asState::EBS_TRIGGERED:
            m_blueDuty = 50000*m_flash2Hz;
            m_greenDuty = 0;
            m_redDuty = 0;
            break;
        default:
            m_blueDuty = 0;
            m_greenDuty = 0;
            m_redDuty = 0;
        break;
    }

    if (m_debug){
        std::cout << "[ASSI-Duty] ASSI pwm (new, old): Red:\t(" << m_redDuty << ", " << m_redDutyOld << ")\t Green:\t(" << m_greenDuty << ", " << m_greenDutyOld << ")\t Blue:\t(" << m_blueDuty << ", " << m_blueDutyOld << ")" << std::endl;
    }


    // Send pwm Requests

    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    int16_t senderStamp = 0;
    
    opendlv::proxy::PulseWidthModulationRequest msgPwm;
 
    if (m_redDuty != m_redDutyOld){
        senderStamp = m_pwmPinAssiRed + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_redDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_redDutyOld = m_redDuty;
    }
    if (m_greenDuty != m_greenDutyOld){
        senderStamp = m_pwmPinAssiGreen + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_greenDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_greenDutyOld = m_greenDuty;
    }
    if (m_blueDuty != m_blueDutyOld){
        senderStamp = m_pwmPinAssiBlue + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_blueDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_blueDutyOld = m_blueDuty;
    }

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

uint16_t StateMachine::getGpioPinClampSensor(){
  return m_gpioPinClampSensor;
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
void StateMachine::setClampExtended(bool state){
    m_clampExtended = state;
}
void StateMachine::setFinishSignal(bool state){
    m_finishSignal = state;
}
void StateMachine::setGoSignal(bool state){
    m_goSignal = state;
}

bool StateMachine::getInitialised(){
  return m_initialised;
}

