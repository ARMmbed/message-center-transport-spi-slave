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

/*  Native SPI Slave driver from Nordic SDK 8.1
*/
extern "C" {
#include "spis-nrf51/spi_slave.h"
}

#define VERBOSE_DEBUG_OUTPUT 0

#if 1
#define DEBUG_OUT(...) { printf(__VA_ARGS__); }
#else
#define DEBUG_OUT(...) /* nothing */
#endif


#if defined(TARGET_LIKE_WATCH_BLE_NRF51)
#define SPIS_MOSI_PIN  SPIS_MOSI    // SPI MOSI signal.
#define SPIS_MISO_PIN  SPIS_MISO    // SPI MISO signal.
#define SPIS_SCK_PIN   SPIS_SCK     // SPI SCK signal.
#define SPIS_CSN_PIN   SPIS_CSN     // SPI CSN signal.
#define SPIS_IRQ_PIN   SPIS_NRDY    // IRQ signal.
#else
#define SPIS_MOSI_PIN  P0_1    // SPI MOSI signal.
#define SPIS_MISO_PIN  P0_2    // SPI MISO signal.
#define SPIS_SCK_PIN   P0_3    // SPI SCK signal.
#define SPIS_CSN_PIN   P0_4    // SPI CSN signal.
#define SPIS_IRQ_PIN   P0_5    // IRQ signal.
#endif

#define DEF_CHARACTER 0xFF             /**< SPI default character. Character clocked out in case of an ignored transaction. */
#define ORC_CHARACTER 0xEE             /**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */

#define SPIS_MAX_MESSAGE_SIZE 0xFF
#define SPIS_COMMAND_SIZE 0x04
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
                FunctionPointer1<void, const char*> fp(bridge, &MessageCenterSPISlave::printTask);
                minar::Scheduler::postCallback(fp.bind("Buffers set, SPI ready\r\n"));

                if (bridge)
                {
                    minar::Scheduler::postCallback(bridge, &MessageCenterSPISlave::transferArmedTask);
                }
            }
            break;

        case SPI_SLAVE_XFER_DONE:
            {
                FunctionPointer1<void, const char*> fp(bridge, &MessageCenterSPISlave::printTask);
                minar::Scheduler::postCallback(fp.bind("Transfer done, SPI ready\r\n"));

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

MessageCenterSPISlave::MessageCenterSPISlave()
    :   irqPin(SPIS_IRQ_PIN),
        state(STATE_IDLE),
        sendFlag(false),
        callbackSend(),
        callbackReceive()
{
    // default high, active low
    irqPin = 1;

    /*************************************************************************/
    // configure Nordic SPI Slave driver

    // register event handler
    spi_slave_evt_handler_register(bridgeEventHandlerIRQ);

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

    spi_slave_init(&spi_slave_config);

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

bool MessageCenterSPISlave::internalSendTask(BlockStatic* block)
{

#if 0
    // check length is set correctly
    uint32_t length = block->getLength();
    length = (length <= SPIS_MESSAGE_SIZE) ? length : SPIS_MESSAGE_SIZE;
    block->at(0) = length;

    sendFlag = true;

    // re-arm buffer with the correct length
    spi_slave_buffers_set(block->getData(), m_rx_buf, length, SPIS_MESSAGE_SIZE);

    // signal update
    minar::Scheduler::postCallback(this, &MessageCenterSPISlave::internalIRQSet);
#endif
    return false;
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
    DEBUG_OUT("MC: %lu %lu\r\n", txLength, rxLength);

    if ((state == STATE_IDLE) && (rxLength > 0))
    {
        state = STATE_RECEIVE_ARMING;

        for (std::size_t idx = 0; idx < SPIS_COMMAND_SIZE; idx++)
        {
            DEBUG_OUT("%02X", cmdRxBuffer[idx]);
        }
        DEBUG_OUT("\r\n");

        // setup rx buffer
        uint32_t length;
        length = cmdRxBuffer[3];
        length = length << 8 | cmdRxBuffer[2];
        length = length << 8 | cmdRxBuffer[1];
        length = length << 8 | cmdRxBuffer[0];

        uint8_t* rxBuffer = (uint8_t*) malloc(length);
        receiveBlock = SharedPointer<Block>(new BlockDynamic(rxBuffer, length));

        spi_slave_buffers_set(cmdTxBuffer, rxBuffer, 1, length);
    }
    else if ((state == STATE_RECEIVE_READY) && (rxLength > 0))
    {
        // receive complete
        state = STATE_RECEIVE_REARM;

        // post received block
        if (callbackReceive)
        {
            minar::Scheduler::postCallback(callbackReceive.bind(receiveBlock));
        }

        // clear reference to shared block
        receiveBlock = SharedPointer<Block>();

        // re-arm
        spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
    }
    else
    {
        // unknown state reset
        state = STATE_IDLE;

        // clear tx buffer
        cmdTxBuffer[0] = ORC_CHARACTER;

        // re-arm
        spi_slave_buffers_set(cmdTxBuffer, cmdRxBuffer, 1, SPIS_COMMAND_SIZE);
    }
}

/*
    SPI buffers armed and ready for Master to initiate tranfer.
*/
void MessageCenterSPISlave::transferArmedTask()
{
    if (state == STATE_RECEIVE_ARMING)
    {
        state = STATE_RECEIVE_READY;

        DEBUG_OUT("armed\r\n");

        // signal master, SPI slave is ready to be read
        irqPin = 0;
    }
    else if (state == STATE_RECEIVE_REARM)
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

void MessageCenterSPISlave::printTask(const char* str)
{
    DEBUG_OUT("spis: %s", str);
}

#if 0

    uint8_t length = m_rx_buf[0];

    // first byte is length; it is non-zero if data was written
    if (length != 0)
    {
        m_rx_buf[0] = 0;

        SharedPointer<Block> data(new BlockDynamic(length));
        data->memcpy(0, &m_rx_buf[1], length);

        minar::Scheduler::postCallback(callbackReceive.bind(data));
    }

    if (sendFlag)
    {
        // send done
        // reset transmit and receive buffers to default
        spi_slave_buffers_set(m_tx_buf, m_rx_buf, 0, SPIS_MESSAGE_SIZE);

        // call callback function
        minar::Scheduler::postCallback(callbackSend);
    }

    // reset transmit and receive buffers to default
    spi_slave_buffers_set(m_tx_buf, m_rx_buf, 0, SPIS_MESSAGE_SIZE);
}
#endif
