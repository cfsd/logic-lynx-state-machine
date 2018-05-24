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
        std::cerr << "Example: " << argv[0] << " --cid=111 --cidgpio=220 --cidanalog=221 --cidpwm=222 --id=1 --verbose=1 --freq=30" << std::endl;
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

        StateMachine stateMachine(VERBOSE, ID, od4, od4Gpio, od4Analog, od4Pwm);

        auto onPressureReading{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                uint16_t channel = envelope.senderStamp()-stateMachine.getSenderStampOffsetAnalog();
                opendlv::proxy::PressureReading analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));

                if (channel == stateMachine.getAnalogPinEbsActuator()){
                    stateMachine.setPressureEbsAct(analogInput.pressure());
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-EBS-ACT] Pressure reading:" << analogInput.pressure() << std::endl;
                }else if (channel == stateMachine.getAnalogPinEbsLine()){
                    stateMachine.setPressureEbsLine(analogInput.pressure());
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-EBS-LINE] Pressure reading:" << analogInput.pressure() << std::endl;
                }else if (channel == stateMachine.getAnalogPinServiceTank()){
                    stateMachine.setPressureServiceTank(analogInput.pressure());
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-TANK] Pressure reading:" << analogInput.pressure() << std::endl;
                }else if (channel == stateMachine.getAnalogPinPressureReg()){
                    stateMachine.setPressureServiceReg(analogInput.pressure());
                    if(VERBOSE)
                        std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-REGULATOR] Pressure reading:" << analogInput.pressure() << std::endl;
                }
            }};
            od4Analog.dataTrigger(opendlv::proxy::PressureReading::ID(), onPressureReading);

        auto onSwitchStateReading{[&stateMachine](cluon::data::Envelope &&envelope)
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                uint16_t pin = envelope.senderStamp()-stateMachine.getSenderStampOffsetGpio();
                if (pin == stateMachine.getGpioPinEbsOk()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setEbsOk(gpioState.state());
                }else if (pin == stateMachine.getGpioPinAsms()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setAsms(gpioState.state());
                }else if (pin == stateMachine.getGpioPinClampSensor()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    stateMachine.setClampExtended(gpioState.state());
                }
            }};
            od4Gpio.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);


            auto heartbeatThread{[&stateMachine, &od4Gpio2]()
            {
                if (!stateMachine.getInitialised()){
                    return;
                }
                using namespace std::literals::chrono_literals;
                bool heatbeat = 0;
                int16_t senderStamp = stateMachine.getSenderStampOffsetGpio() + 27;
                std::chrono::system_clock::time_point threadTime = std::chrono::system_clock::now();
                opendlv::proxy::SwitchStateRequest msgGpio;

                while (true) {
                    heatbeat = !heatbeat;
                    std::this_thread::sleep_until(std::chrono::duration<double>(0.033)+threadTime);
                    threadTime = std::chrono::system_clock::now();
                    cluon::data::TimeStamp sampleTime = cluon::time::convert(threadTime);
                    msgGpio.state(heatbeat);
                    od4Gpio2.send(msgGpio, sampleTime, senderStamp);
                }

            }};
            std::thread hbThread(heartbeatThread);



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

