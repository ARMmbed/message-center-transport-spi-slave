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


#include "message-center-transport/ExternalFlashServer.h"

/*  Native SPI Slave driver from Nordic SDK
    mbed 2.0 drivers not working properly.
*/
extern "C" {
#include "spis-nrf51/spi_slave.h"
}

#define VERBOSE_DEBUG_OUTPUT 0

#if 0
#define DEBUG_OUT(...) { printf(__VA_ARGS__); }
#else
#define DEBUG_OUT(...) /* nothing */
#endif


static ExternalFlashServer* flashBridge;


#if defined(TARGET_LIKE_WATCH_BLE_NRF51)
#define SPIS_MOSI_PIN  SPIS_MOSI    // SPI MOSI signal.
#define SPIS_MISO_PIN  SPIS_MISO    // SPI MISO signal.
#define SPIS_SCK_PIN   SPIS_SCK     // SPI SCK signal.
#define SPIS_CSN_PIN   SPIS_CSN     // SPI CSN signal.
#define SPIS_IRQ_PIN   SPIS_NRDY    // IRQ signal.
#else
#define SPIS_MOSI_PIN  P0_17    // SPI MOSI signal.
#define SPIS_MISO_PIN  P0_18    // SPI MISO signal.
#define SPIS_SCK_PIN   P0_19    // SPI SCK signal.
#define SPIS_CSN_PIN   P0_16    // SPI CSN signal.
#define SPIS_IRQ_PIN   P0_15    // IRQ signal.
#endif

#define DEF_CHARACTER 0xFF             /**< SPI default character. Character clocked out in case of an ignored transaction. */
#define ORC_CHARACTER 0xEE             /**< SPI over-read character. Character clocked out after an over-read of the transmit buffer. */

#define SPIS_MESSAGE_SIZE 0xFF
static uint8_t m_tx_buf[SPIS_MESSAGE_SIZE] = {0};
static uint8_t m_rx_buf[SPIS_MESSAGE_SIZE] = {0};

/*  Global variables, should be class variables.
    Need proper interaction with SPI slave driver.
*/

// global storage block for passing data back and forth between flash and envoy
static uint8_t storageBuffer[100];
static uint16_t availableLength = 0;


/*  Helper function for interpreting the received SPI command
*/
void parseSPICommand()
{
    /*  Get command, address, and length from receive buffer.
        Note, the command is always valid, but the address
        and length are context specific.
    */
    uint8_t command = m_rx_buf[0];

    uint32_t address = m_rx_buf[4];
    address = address << 8 | m_rx_buf[3];
    address = address << 8 | m_rx_buf[2];
    address = address << 8 | m_rx_buf[1];

    uint16_t length = m_rx_buf[6];
    length = length << 8 | m_rx_buf[5];

    DEBUG_OUT("%d %08lX %d\r\n", command, address, length);

    switch (command)
    {
        case FLASH_CMD_WRITE:
                            {
                                // check if the write request is within bounds
                                if ((address + length) < sizeof(storageBuffer))
                                {
                                    // copy spi receive buffer to internal memory buffer
                                    // TODO: write memory buffer to flash
                                    memcpy(&(storageBuffer[address]), &(m_rx_buf[7]), length);

                                    // reset transmit and receive buffers to default
                                    spi_slave_buffers_set(m_tx_buf, m_rx_buf, 0, SPIS_MESSAGE_SIZE);
                                }
                            }
                            break;

        case FLASH_CMD_READ:
                            {
                                availableLength = 0;

                                // check if the read request is within bounds
                                // if not, the READ_CONTINUE will read back the ORC_CHARACTER
                                if (address < sizeof(storageBuffer))
                                {
                                    // ensure we do not over-read from memory
                                    availableLength = (address + length < sizeof(storageBuffer)) ?
                                                       length : sizeof(storageBuffer) - address;

                                    // copy memory buffer to SPI transmit buffer
                                    memcpy(m_tx_buf, &(storageBuffer[address]), availableLength);
                                }

                                // re-arm buffer with the correct length
                                spi_slave_buffers_set(m_tx_buf, m_rx_buf, availableLength, 0);

                                // signal SPI master that the SPI slave is ready for the READ_CONTINUE
                                minar::Scheduler::postCallback(flashBridge, &ExternalFlashServer::internalIRQSet)
                                    .delay(minar::milliseconds(10));
                            }
                            break;

        case FLASH_CMD_ERASE:
                            {
                                flashBridge->internalErase();
                            }
                            break;

        case FLASH_CMD_READ_CONTINUE:
        default:
                            {
                                //
                                memset(m_tx_buf, DEF_CHARACTER, availableLength);

                                // reset transmit and receive buffers to default
                                spi_slave_buffers_set(m_tx_buf, m_rx_buf, 0, SPIS_MESSAGE_SIZE);
                            }
                            break;
    }

    // clear command from receive buffer
    memset(m_rx_buf, 0, 7);
}

/*  Function for SPI slave event callback.
*/
extern "C" {
static void eventHandler(spi_slave_evt_t event)
{
    switch (event.evt_type)
    {
        case SPI_SLAVE_BUFFERS_SET_DONE:
            DEBUG_OUT("Buffers set, SPI ready\r\n");
            break;

        case SPI_SLAVE_XFER_DONE:
            DEBUG_OUT("Transfer done, SPI ready\r\n");
            parseSPICommand();
            break;

        case SPI_SLAVE_EVT_TYPE_MAX:
            DEBUG_OUT("Event type max\r\n");
            break;

        default:
            DEBUG_OUT("Unknown SPI event\r\n");
            break;
    }
}
}


ExternalFlashServer::ExternalFlashServer()
    :   callback(),
        callbackUpdate()
{
    // default high, active low
    DigitalOut irqPin(SPIS_IRQ_PIN);
    irqPin = 1;

    /*************************************************************************/
    // configure Nordic SPI Slave driver

    // register event handler
    spi_slave_evt_handler_register(eventHandler);

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

    // arm transmit and receive buffers
    // default to send nothing, receive at most SPIS_MESSAGE_SIZE (255) bytes
    spi_slave_buffers_set(m_tx_buf, m_rx_buf, 0, SPIS_MESSAGE_SIZE);

    /*************************************************************************/
    // set most recent instantiation
    flashBridge = this;
}

void ExternalFlashServer::internalRead(uint32_t address, Block* block)
{
    availableLength = 0;

    // check if the read request is within bounds
    // if not, the READ_CONTINUE will read back the ORC_CHARACTER
    if (address < sizeof(storageBuffer))
    {
        // ensure we do not over-read from memory
        availableLength = (address + block->getLength() < sizeof(storageBuffer)) ?
                           block->getLength() : sizeof(storageBuffer) - address;

        // copy memory buffer to SPI transmit buffer
        block->memcpy(0, &(storageBuffer[address]), availableLength);
    }

    // call callback function
    minar::Scheduler::postCallback(callback);
}

void ExternalFlashServer::internalWrite(uint32_t address, Block* block)
{
    // check if the write request is within bounds
    if ((address + block->getLength()) < sizeof(storageBuffer))
    {
        // copy block buffer to internal memory buffer
        block->memcpy(&(storageBuffer[address]), 0, block->getLength());

        // signal update
        minar::Scheduler::postCallback(this, &ExternalFlashServer::internalIRQSet);
    }

    // call callback function
    minar::Scheduler::postCallback(callback);
}

void ExternalFlashServer::internalIRQSet()
{
    DigitalOut irqPin(SPIS_IRQ_PIN);
    irqPin = 0;

    minar::Scheduler::postCallback(this, &ExternalFlashServer::internalIRQClear);
}

void ExternalFlashServer::internalIRQClear()
{
    DigitalOut irqPin(SPIS_IRQ_PIN);
    irqPin = 1;
}

void ExternalFlashServer::internalErase()
{
    memset(storageBuffer, 0xFF, sizeof(storageBuffer));
}
