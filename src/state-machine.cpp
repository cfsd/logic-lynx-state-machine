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
#include "opendlv-standard-message-set.hpp"

#include "logic-state-machine.hpp"

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <ctime>
#include <chrono>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("freq")) || (0 == commandlineArguments.count("cid"))) {
        std::cerr << argv[0] << "Module running state-machine for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple beaglebone units>] [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=219 --cidgpio=220 --cidanalog=221 --cidpwm=222 --cidgen=48 --id=1 --verbose=1 --freq=30" << std::endl;
        retCode = 1;
    } else {
        const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const float FREQ{std::stof(commandlineArguments["freq"])};
        std::cout << "Micro-Service ID:" << ID << std::endl;

        // Interface to a running OpenDaVINCI session.
        

        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        cluon::OD4Session od4Gpio{static_cast<uint16_t>(std::stoi(commandlineArguments["cidgpio"]))};
        cluon::OD4Session od4Gpio2{static_cast<uint16_t>(std::stoi(commandlineArguments["cidgpio"]))};
        cluon::OD4Session od4Analog{static_cast<uint16_t>(std::stoi(commandlineArguments["cidanalog"]))};
        cluon::OD4Session od4Pwm{static_cast<uint16_t>(std::stoi(commandlineArguments["cidpwm"]))};
        cluon::OD4Session od4Gen{static_cast<uint16_t>(std::stoi(commandlineArguments["cidgen"]))};

        StateMachine stateMachine(VERBOSE, ID, od4, od4Gpio, od4Analog, od4Pwm, od4Gen);

        auto onPressureReading{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                uint16_t channel = envelope.senderStamp()-stateMachine.getSenderStampOffsetAnalog();
                opendlv::proxy::PressureReading analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));

                if (channel == stateMachine.getAnalogPinEbsActuator()){
                    stateMachine.setPressureEbsAct(analogInput.pressure());
                    stateMachine.m_lastUpdateAnalog = cluon::time::now();
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-EBS-ACT] Pressure reading:" << analogInput.pressure() << std::endl;
                }else if (channel == stateMachine.getAnalogPinEbsLine()){
                    stateMachine.setPressureEbsLine(analogInput.pressure());
                    stateMachine.m_lastUpdateAnalog = cluon::time::now();
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-EBS-LINE] Pressure reading:" << analogInput.pressure() << std::endl;
                }else if (channel == stateMachine.getAnalogPinServiceTank()){
                    stateMachine.setPressureServiceTank(analogInput.pressure());
                    stateMachine.m_lastUpdateAnalog = cluon::time::now();
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-TANK] Pressure reading:" << analogInput.pressure() << std::endl;
                }else if (channel == stateMachine.getAnalogPinPressureReg()){
                    stateMachine.setPressureServiceReg(analogInput.pressure());
                    stateMachine.m_lastUpdateAnalog = cluon::time::now();
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-REGULATOR] Pressure reading:" << analogInput.pressure() << std::endl;
                }
            }};
            od4Analog.dataTrigger(opendlv::proxy::PressureReading::ID(), onPressureReading);

        auto onSwitchStateReadingGpio{[&stateMachine](cluon::data::Envelope &&envelope)
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                uint16_t pin = envelope.senderStamp()-stateMachine.getSenderStampOffsetGpio();
                if (pin == stateMachine.getGpioPinEbsOk()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setEbsOk(gpioState.state());
                    stateMachine.m_lastUpdateGpio = cluon::time::now();
                }else if (pin == stateMachine.getGpioPinAsms()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setAsms(gpioState.state());
                    stateMachine.m_lastUpdateGpio = cluon::time::now();
                }else if (pin == stateMachine.getGpioPinClampSensor()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setClampExtended(gpioState.state());
                    stateMachine.m_lastUpdateGpio = cluon::time::now();
                }
            }};
            od4Gpio.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReadingGpio);

        auto onGroundSteeringReading{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                opendlv::proxy::GroundSteeringReading analogInput = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(envelope));
                if (envelope.senderStamp() == 1200){
                stateMachine.setSteerPosition(analogInput.groundSteering());
                   if (VERBOSE)
                        std::cout << "[LOGIC-STEERING-POSITION-ACT] Position reading:" << analogInput.groundSteering() << std::endl;

                }else if (envelope.senderStamp() == 1206){
                stateMachine.setSteerPositionRack(analogInput.groundSteering());
                    if (VERBOSE)
                        std::cout << "[LOGIC-STEERING-POSITION-RACK] Position reading:" << analogInput.groundSteering() << std::endl;
                }
            }};
            od4Analog.dataTrigger(opendlv::proxy::GroundSteeringReading::ID(), onGroundSteeringReading);

        auto onSwitchStateReading{[&stateMachine](cluon::data::Envelope &&envelope)
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                uint16_t pin = envelope.senderStamp();
                if (pin == 1402){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setGoSignal(gpioState.state());
                }else if (pin == 1410){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setFinishSignal(gpioState.state()&0x04);
                }else if (pin == 1406){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setMission(gpioState.state());
                }
            }};
            od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);

        auto onPulseWidthModulationRequest{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {   
            if (!stateMachine.getInitialised()){
                return;
            }
            if (envelope.senderStamp() == 1341){
            auto const pwmState = cluon::extractMessage<opendlv::proxy::PulseWidthModulationRequest>(std::move(envelope));
            stateMachine.setDutyCycleBrake(pwmState.dutyCycleNs());
            }
        }};
        od4.dataTrigger(opendlv::proxy::PulseWidthModulationRequest::ID(), onPulseWidthModulationRequest);

        auto onTorqueRequest{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {   
            if (!stateMachine.getInitialised()){
                return;
            }
            if (envelope.senderStamp() == 1502){
            auto const torqueReq = cluon::extractMessage<opendlv::proxy::TorqueRequest>(std::move(envelope));
            stateMachine.setTorqueReqLeft((int)round(torqueReq.torque()));
            }else if (envelope.senderStamp() == 1503){
            auto const torqueReq = cluon::extractMessage<opendlv::proxy::TorqueRequest>(std::move(envelope));
            stateMachine.setTorqueReqRight((int)round(torqueReq.torque()));
            }
        }};
        od4.dataTrigger(opendlv::proxy::TorqueRequest::ID(), onTorqueRequest);


        using namespace std::literals::chrono_literals;

        auto atFrequency{[&od4, &stateMachine]() -> bool
        {            
            stateMachine.body();
            return true;
        }};

        od4.timeTrigger(FREQ, atFrequency);
    }
    return retCode;
}

