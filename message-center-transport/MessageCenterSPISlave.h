/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __MESSAGE_CENTER_SPI_SLAVE_H__
#define __MESSAGE_CENTER_SPI_SLAVE_H__

#include "mbed-drivers/mbed.h"

#include "message-center-transport/MessageCenterTransport.h"

#include "mbed-block/BlockStatic.h"
#include "mbed-block/BlockDynamic.h"
#include "core-util/SharedPointer.h"

/*  Native SPI Slave driver from Nordic SDK 8.1
*/
extern "C" {
#include "spis-nrf51/spi_slave.h"
}

using namespace mbed::util;


class MessageCenterSPISlave : public MessageCenterTransport
{
public:
    MessageCenterSPISlave(spi_slave_config_t& config, PinName cs, PinName irq);

public:
    void transferDoneTask(uint32_t txLength, uint32_t rxLength);
    void transferArmedTask(void);

    void printTask(const char*);

private:

    PinName    csPin;
    DigitalOut irqPin;

    typedef enum {
        STATE_IDLE,
        STATE_IDLE_ARM,
        STATE_SEND_ARM_COMMAND,
        STATE_SEND_COMMAND,
        STATE_SEND_ARM_MESSAGE,
        STATE_SEND_MESSAGE,
        STATE_SEND_ARM_DONE,
        STATE_RECEIVE_ARMING,
        STATE_RECEIVE_READY
    } state_t;

    state_t state;

    virtual bool internalSendTask(uint16_t port, BlockStatic& block);

    void sendCommandTask(uint16_t port, uint32_t length);
    void timeoutTask();

    uint16_t callbackPort;

    SharedPointer<BlockStatic> receiveBlock;
    BlockStatic sendBlock;

    minar::callback_handle_t timeoutHandle;
};

#endif // __MESSAGE_CENTER_SPI_SLAVE_H__
