/**
 * @file    main.cpp
 * @brief   mbed Connected Home Endpoint main entry point
 * @author  Doug Anson
 * @version 1.0
 * @see
 *
 * Copyright (c) 2014
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

#include "mbed.h"
#include "ble/BLE.h"

#include "message-center-transport/MessageCenterSPISlave.h"


#define SPIS_MOSI_PIN  P0_1    // SPI MOSI signal.
#define SPIS_MISO_PIN  P0_2    // SPI MISO signal.
#define SPIS_SCK_PIN   P0_3    // SPI SCK signal.
#define SPIS_CSN_PIN   P0_4    // SPI CSN signal.
#define SPIS_IRQ_PIN   P0_5    // IRQ signal.

static spi_slave_config_t spi_slave_config = {
    .pin_miso         = SPIS_MISO_PIN,
    .pin_mosi         = SPIS_MOSI_PIN,
    .pin_sck          = SPIS_SCK_PIN,
};

static MessageCenterSPISlave transport(spi_slave_config, SPIS_CSN_PIN, SPIS_IRQ_PIN);

static uint8_t buffer[100];
static BlockStatic block(buffer, sizeof(buffer));


// enable buttons to initiate transfer
static InterruptIn button1(BUTTON1);
static InterruptIn button2(BUTTON2);

// liveness led
static DigitalOut led1(LED1);
static Ticker ticker;

void receivedBlock(SharedPointer<Block> block)
{
    printf("main:received: %p\r\n", &(block->at(0)));
    for (std::size_t idx = 0; idx < block->getLength(); idx++)
    {
        printf("%02X", block->at(idx));
    }
    printf("\r\n");
}


/*****************************************************************************/
/* Buttons                                                                   */
/*****************************************************************************/

int32_t counter = 0;

void sendDone()
{
    printf("send done\r\n");

    if (--counter > 0)
    {
        transport.sendTask(&block, sendDone);
    }
}

void button1Task()
{
    printf("button 1\r\n");

    for (std::size_t idx = 0; idx < block.getMaxLength(); idx++)
    {
        block.at(idx) = idx;
    }

    counter = 1;

    transport.sendTask(&block, sendDone);
}

void button1ISR()
{
    minar::Scheduler::postCallback(button1Task);
}

void button2ISR()
{
    printf("button 2\r\n");
}

/*****************************************************************************/
/* App start                                                                 */
/*****************************************************************************/


void ledISR()
{
    led1 = !led1;
}

void app_start(int, char *[])
{
    button1.fall(button1ISR);
    button2.fall(button2ISR);

    ticker.attach(ledISR, 1.0);

    transport.onReceiveTask(receivedBlock);

    printf("SPI Slave: %s %s\r\n", __DATE__, __TIME__);
}
