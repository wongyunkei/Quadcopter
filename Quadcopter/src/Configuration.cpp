/*
 * Configuration.cpp
 *
 *  Created on: 2015�~11��27��
 *      Author: wongy
 */

#include <Configuration.h>

Configuration::Configuration(uint32_t rcc, GPIO_TypeDef* port, uint16_t pin) : _rcc(rcc), _port(port), _pin(pin){
}

