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

class StateMachine {
   private:
    //StateMachine(const StateMachine &) = delete;
    //StateMachine(StateMachine &&)      = delete;
    //StateMachine &operator=(const StateMachine &) = delete;
    //StateMachine &operator=(StateMachine &&) = delete;

   public:
    StateMachine(bool verbose, uint32_t id);
    ~StateMachine();

   public:
    float decode(const std::string &data) noexcept;
    void callOnReceive(cluon::data::Envelope data);
    void body(cluon::OD4Session &od4);

   private:
    bool controlPosition(cluon::OD4Session &od4, float setPoint);
    void findRack(cluon::OD4Session &od4);
    void setUp();
    void tearDown();
    

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
    bool m_heartbeat;
    bool m_asms;


    const uint16_t m_gpioPinAsms = 115;
    const uint16_t m_gpioPinHeartbeat = 27;
    const uint16_t m_gpioPinHeartbeat = 27;

    const uint16_t m_pwmPinAssiBlue = 0;
    const uint16_t m_pwmPinAssiRed = 20;
    const uint16_t m_pwmPinAssiGreen = 21;



    const uint16_t m_analogPinPressureEbsAct = 3;
    const uint16_t m_analogPinPressureEbsLine = 1;
    const uint16_t m_analogPinPressureServiceTank = 2;
    const uint16_t m_analogPinPressureServiceReg = 5;

    const double m_analogConvPressureEbsAct = 1;
    const double m_analogConvPressureEbsLine = 1;
    const double m_analogConvPressureServiceTank = 1;
    const double m_analogConvPressureServiceReg = 1;

    const double m_analogOffsetPressureEbsAct = 0;
    const double m_analogOffsetPressureEbsLine = 0;
    const double m_analogOffsetPressureServiceTank = 0;
    const double m_analogOffsetPressureServiceReg = 0;    
};

#endif

