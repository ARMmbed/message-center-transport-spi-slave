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


#include "message-center-transport/MessageCenterSPISlave.h"

#include "core-util/CriticalSectionLock.h"

#define VERBOSE_DEBUG_OUTPUT 0

#if 0
#define DEBUG_OUT(...) { printf(__VA_ARGS__); }
#else
#define DEBUG_OUT(...) /* nothing */
#endif

#define DEF_CHARACTER 0xFF             /**< SPI default character. Character clocked out in case of an ignored transaction. */
#define ORC_CHARACTER 0xEE             /**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */

#define TIMEOUT_IN_MS 500

#define SPIS_MAX_MESSAGE_SIZE 0xFF
#define SPIS_COMMAND_SIZE 0x06
static uint8_t cmdTxBuffer[SPIS_COMMAND_SIZE] = {0};
static uint8_t cmdRxBuffer[SPIS_COMMAND_SIZE] = {0};

/*****************************************************************************/
/* IRQ handler                                                               */
/*****************************************************************************/
static MessageCenterSPISlave* bridge = NULL;

/*  Function for SPI slave event callback.
*/
extern "C" {
static void bridgeEventHandlerIRQ(spi_slave_evt_t event)
{
    switch (event.evt_type)
    {
        case SPI_SLAVE_BUFFERS_SET_DONE:
            {
                if (bridge)
                {
                    minar::Scheduler::postCallback(bridge, &MessageCenterSPISlave::transferArmedTask)
                        .delay(minar::milliseconds(2 * MessageCenterTransport::MinimumIRQDelay))
                        .tolerance(0);
                }
            }
            break;

        case SPI_SLAVE_XFER_DONE:
            {
                if (bridge)
                {
                    FunctionPointer2<void, uint32_t, uint32_t> fp(bridge, &MessageCenterSPISlave::transferDoneTask);
                    minar::Scheduler::postCallback(fp.bind(event.tx_amount, event.rx_amount));
                }
            }
            break;

        case SPI_SLAVE_EVT_TYPE_MAX:
            {
                FunctionPointer1<void, const char*> fp(bridge, &MessageCenterSPISlave::printTask);
                minar::Scheduler::postCallback(fp.bind("Event type max\r\n"));
            }
            break;

        default:
            {
                FunctionPointer1<void, const char*> fp(bridge, &MessageCenterSPISlave::printTask);
                minar::Scheduler::postCallback(fp.bind("Unknown SPI event\r\n"));
            }
            break;
    }
}
}

/*****************************************************************************/
/* Constructor                                                               */
/*****************************************************************************/

MessageCenterSPISlave::MessageCenterSPISlave(spi_slave_config_t& config, PinName cs, PinName irq)
:       csPin(cs),
        irqPin(irq),
        state(STATE_IDLE),
        callbackPort(0),
        sendBlock(),
        timeoutHandle(NULL)
{
    // default high, active low
    irqPin = 1;

    /*************************************************************************/
    // configure Nordic SPI Slave driver

    // register event handler
    spi_slave_evt_handler_register(bridgeEventHandlerIRQ);

#if 0
    // setup configuration struct and register with device
    spi_slave_config_t spi_slave_config = {
        .pin_miso         = SPIS_MISO_PIN,
        .pin_mosi         = SPIS_MOSI_PIN,
        .pin_sck          = SPIS_SCK_PIN,
        .pin_csn          = SPIS_CSN_PIN,
        .mode             = SPI_MODE_0,
        .bit_order        = SPIM_MSB_FIRST,
        .def_tx_character = DEF_CHARACTER,
        .orc_tx_character = ORC_CHARACTER
    };
#endif

    config.pin_csn          = cs;
    config.mode             = SPI_MODE_0;
    config.bit_order        = SPIM_MSB_FIRST;
    config.def_tx_character = DEF_CHARACTER;
    config.orc_tx_character = ORC_CHARACTER;

    spi_slave_init(&config);

    // clear tx buffer
    cmdTxBuffer[0] = ORC_CHARACTER;

    // arm transmit and receive buffers
    spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);

    /*************************************************************************/
    // set most recent instantiation
    bridge = this;
}

/*****************************************************************************/
/* Send                                                                      */
/*****************************************************************************/

bool MessageCenterSPISlave::internalSendTask(uint16_t port, BlockStatic& block)
{
    DigitalIn cs(csPin);

    bool result = false;

    // begin critical section
    {
        CriticalSectionLock lock;

        if ((state == STATE_IDLE) &&
            (cs == 1))
        {
            // set new state
            state = STATE_SEND_ARM_COMMAND;

            // signal SPI master that slave has control
            irqPin = 0;

            // SPI acquired successfully
            result = true;
        }
    }
    // end critical section

    //
    if (result)
    {
        // copy block
        sendBlock = block;

        // check and set length
        uint32_t length = sendBlock.getLength();

        if (length > SPIS_MAX_MESSAGE_SIZE)
        {
            length = SPIS_MAX_MESSAGE_SIZE;
            sendBlock.setLength(length);
        }

        // send read command to master
        FunctionPointer2<void, uint16_t, uint32_t> fp(this, &MessageCenterSPISlave::sendCommandTask);
        minar::Scheduler::postCallback(fp.bind(port, length));

        // set timeout
        timeoutHandle = minar::Scheduler::postCallback(this, &MessageCenterSPISlave::timeoutTask)
                            .delay(minar::milliseconds(TIMEOUT_IN_MS))
                            .getHandle();
    }

    return result;
}

void MessageCenterSPISlave::sendCommandTask(uint16_t port, uint32_t length)
{
    // construct send command
    cmdTxBuffer[0] = length;
    cmdTxBuffer[1] = length >> 8;
    cmdTxBuffer[2] = length >> 16;
    cmdTxBuffer[3] = length >> 24;

    cmdTxBuffer[4] = port;
    cmdTxBuffer[5] = port >> 8;

    // arm buffer
    spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, SPIS_COMMAND_SIZE, SPIS_COMMAND_SIZE);
}

/*****************************************************************************/
/* Event handlers                                                            */
/*****************************************************************************/

/*  SPI transfer done.
    Depending on the state, this is
    (1) rx command received
    (2) tx command send
    (3) message received
    (4) message send
*/
void MessageCenterSPISlave::transferDoneTask(uint32_t txLength, uint32_t rxLength)
{
    // Master has just read the command from the Slave
    if ((state == STATE_SEND_COMMAND) && (txLength > 0))
    {
        state = STATE_SEND_ARM_MESSAGE;

        // re-arm buffers
        spi_slave_buffers_set(sendBlock.getData(), cmdRxBuffer, sendBlock.getLength(), SPIS_COMMAND_SIZE);
    }
    // Master has just read the message from the Slave
    else if ((state == STATE_SEND_MESSAGE) && (rxLength > 0))
    {
        state = STATE_SEND_ARM_DONE;

        // re-arm buffers
        spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
    }
    // Slave has just received command from Master
    else if ((state == STATE_IDLE) && (rxLength > 0))
    {
        // get length
        uint32_t length;
        length = cmdRxBuffer[3];
        length = (length << 8) | cmdRxBuffer[2];
        length = (length << 8) | cmdRxBuffer[1];
        length = (length << 8) | cmdRxBuffer[0];

        if (length <= SPIS_MAX_MESSAGE_SIZE)
        {
            state = STATE_RECEIVE_ARMING;

            // store port
            callbackPort = cmdRxBuffer[5];
            callbackPort = (callbackPort << 8) | cmdRxBuffer[4];

            // setup rx buffer
            uint8_t* rxBuffer = (uint8_t*) malloc(length);
            receiveBlock = SharedPointer<BlockStatic>(new BlockDynamic(rxBuffer, length));

            // re-arm buffers
            spi_slave_buffers_set(cmdTxBuffer, rxBuffer, 1, length);

            // set timeout
            timeoutHandle = minar::Scheduler::postCallback(this, &MessageCenterSPISlave::timeoutTask)
                                .delay(minar::milliseconds(TIMEOUT_IN_MS))
                                .getHandle();
        }
        else
        {
            // wrong length - reset and re-arm buffers
            spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
        }
    }
    // Slave has just received message from Master
    else if ((state == STATE_RECEIVE_READY) && (rxLength > 0))
    {
        // receive complete
        state = STATE_IDLE_ARM;

        // post received block
        if (callbackReceive)
        {
            minar::Scheduler::postCallback(callbackReceive.bind(callbackPort, receiveBlock));
        }

        // clear reference to shared block
        receiveBlock = SharedPointer<BlockStatic>();

        // cancel timeout
        minar::Scheduler::cancelCallback(timeoutHandle);
        timeoutHandle = NULL;

        // re-arm buffers
        spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
    }
    // Unknown state, reset to known state
    else
    {
        state = STATE_IDLE;

        // clear tx buffer
        cmdTxBuffer[0] = ORC_CHARACTER;

        // re-arm buffers
        spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
    }
}

/*
    SPI buffers armed and ready for Master to initiate tranfer.
*/
void MessageCenterSPISlave::transferArmedTask()
{
    if (state == STATE_SEND_ARM_COMMAND)
    {
        state = STATE_SEND_COMMAND;

        // signal master, SPI slave has command to be read
        irqPin = 1;
    }
    else if (state == STATE_SEND_ARM_MESSAGE)
    {
        state = STATE_SEND_MESSAGE;

        // signal master, SPI slave has message to be read
        irqPin = 0;
    }
    // Master has read message, and buffers have been rearmed.
    // Signal send done callback.
    else if (state == STATE_SEND_ARM_DONE)
    {
        state = STATE_IDLE;

        // cancel timeout
        minar::Scheduler::cancelCallback(timeoutHandle);
        timeoutHandle = NULL;

        if (callbackSend)
        {
            minar::Scheduler::postCallback(callbackSend);
        }

        // signal master, SPI slave is ready for next command
        irqPin = 1;
    }
    else if (state == STATE_RECEIVE_ARMING)
    {
        state = STATE_RECEIVE_READY;

        // signal master, SPI slave is ready to be read
        irqPin = 0;
    }
    else if (state == STATE_IDLE_ARM)
    {
        // re-arm done, ready for next command
        state = STATE_IDLE;

        // signal master, SPI slave is ready for next command
        irqPin = 1;
    }
    else
    {
        // unknown state, clear IRQ by setting it high
        irqPin = 1;
    }
}

void MessageCenterSPISlave::timeoutTask()
{
    state = STATE_IDLE_ARM;

    // clear tx buffer
    cmdTxBuffer[0] = ORC_CHARACTER;

    // re-arm buffers
    spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
}

void MessageCenterSPISlave::printTask(const char* str)
{
    DEBUG_OUT("spis: %s", str);
}

