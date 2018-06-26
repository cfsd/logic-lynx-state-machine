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

#ifndef STATEMACHINE
#define STATEMACHINE

#include "opendlv-standard-message-set.hpp"

#include <memory>
#include <string>
#include <vector>
#include <utility>

enum asState {
    AS_OFF,
    AS_READY, 
    AS_DRIVING, 
    AS_FINISHED, 
    EBS_TRIGGERED
 };


class StateMachine {
   private:
    //StateMachine(const StateMachine &) = delete;
    //StateMachine(StateMachine &&)      = delete;
    //StateMachine &operator=(const StateMachine &) = delete;
    //StateMachine &operator=(StateMachine &&) = delete;

   public:
    StateMachine(bool verbose, uint32_t id, cluon::OD4Session &od4, cluon::OD4Session &od4Gpio, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Pwm, cluon::OD4Session &od4Gen);
    ~StateMachine();

   public:
    float decode(const std::string &data) noexcept;
    void body();
    uint16_t getGpioPinEbsOk();
    uint16_t getGpioPinAsms();
    uint16_t getAnalogPinEbsLine();
    uint16_t getAnalogPinServiceTank();
    uint16_t getAnalogPinEbsActuator();
    uint16_t getAnalogPinPressureReg();
    uint32_t getSenderStampOffsetGpio();
    uint32_t getSenderStampOffsetAnalog();
    uint16_t getGpioPinClampSensor();
    void setPressureEbsAct(float pos);
    void setPressureEbsLine(float pos);
    void setPressureServiceTank(float pos);
    void setPressureServiceReg(float pos);
    void setEbsOk(bool state);
    void setAsms(bool state);
    void setClampExtended(bool state);
    void setFinishSignal(bool state);
    void setGoSignal(bool state);
    void setDutyCycleBrake(uint32_t duty);
    void setTorqueReqLeft(int16_t torque);
    void setTorqueReqRight(int16_t torque);
    void setSteerPositionRack(float pos);
    void setSteerPosition(float pos);
    bool getInitialised();
    cluon::data::TimeStamp m_lastUpdateAnalog;
    cluon::data::TimeStamp m_lastUpdateGpio;

   private:
    void stateMachine();
    bool setAssi(asState assi);
    void setUp();
    void tearDown();
    
    
    cluon::OD4Session &m_od4;
    cluon::OD4Session &m_od4Gpio;
    cluon::OD4Session &m_od4Analog;
    cluon::OD4Session &m_od4Pwm;
    cluon::OD4Session &m_od4Gen;
    bool m_debug;
    uint32_t m_bbbId;
    uint32_t m_senderStampOffsetGpio;
    uint32_t m_senderStampOffsetAnalog;
    uint32_t m_senderStampOffsetPwm;
    bool m_initialised;
    float m_pressureEbsAct;
    float m_pressureEbsLine;
    float m_pressureServiceTank;
    float m_pressureServiceReg;
    bool m_ebsOk;
    bool m_asms;
    bool m_heartbeat;
    bool m_ebsSpeaker;
    bool m_compressor;
    bool m_ebsRelief;
    bool m_finished;
    bool m_shutdown;
    bool m_serviceBrake;
    bool m_ebsSpeakerOld;
    bool m_compressorOld;
    bool m_ebsReliefOld;
    bool m_finishedOld;
    bool m_shutdownOld;
    bool m_serviceBrakeOld;
    uint32_t m_brakeDuty;
    uint32_t m_brakeDutyOld;
    uint32_t m_brakeDutyRequest;
    uint32_t m_blueDuty;
    uint32_t m_greenDuty;
    uint32_t m_redDuty;
    uint32_t m_blueDutyOld;
    uint32_t m_greenDutyOld;
    uint32_t m_redDutyOld;
    asState m_currentState;
    asState m_prevState;
    bool m_flash2Hz; 
    uint64_t m_nextFlashTime;
    int m_findRackSeqNo;
    bool m_serviceBrakeOk;
    bool m_ebsPressureOk;
    bool m_clampExtended;
    uint64_t m_ebsTriggeredTime;
    bool m_goSignal;
    bool m_finishSignal;
    bool m_first;
    int16_t m_torqueReqLeft;
    int16_t m_torqueReqRight;
    int16_t m_torqueReqLeftCan;
    int16_t m_torqueReqRightCan;
    uint8_t m_rtd;
    bool m_modulesRunning;
    bool m_ebsFault;
    float m_steerPosition;
    float m_steerPositionRack;
    bool m_steerFault;


    const uint16_t m_gpioPinAsms = 115;
    const uint16_t m_gpioPinEbsOk = 49;
    const uint16_t m_gpioPinClampSensor = 112;

    const uint16_t m_gpioPinHeartbeat = 27;
    const uint16_t m_gpioPinEbsSpeaker = 44;
    const uint16_t m_gpioPinCompressor = 45;
    const uint16_t m_gpioPinEbsRelief = 61;
    const uint16_t m_gpioPinFinished = 66;
    const uint16_t m_gpioPinShutdown = 67;
    const uint16_t m_gpioPinServiceBrake = 69;

    const uint16_t m_pwmPinAssiBlue = 0;
    const uint16_t m_pwmPinAssiRed = 20;
    const uint16_t m_pwmPinAssiGreen = 21;
    const uint16_t m_pwmPinBrake = 41;
 
    const uint16_t m_analogPinEbsLine = 1;
    const uint16_t m_analogPinServiceTank = 2;
    const uint16_t m_analogPinEbsActuator = 3;
    const uint16_t m_analogPinPressureReg = 5;
    const uint16_t m_analogPinSteerPosition = 0;
    const uint16_t m_analogPinSteerPositionRack = 6;

    const uint32_t m_senderStampCurrentState = 1401;
    const uint32_t m_senderStampRTD = 1404;
    const uint32_t m_senderStampEBSFault = 1405;

};

#endif

