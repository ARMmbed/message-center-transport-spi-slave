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

#ifndef __EXTERNALFLASHSERVER_H__
#define __EXTERNALFLASHSERVER_H__

#include "mbed.h"
#include "mbed-block/BlockStatic.h"
#include "message-center-transport/ExternalFlash.h"

#if defined(YOTTA_MINAR_VERSION_STRING)
using namespace mbed::util;
#endif

class ExternalFlashServer
{
public:
    ExternalFlashServer();

    /*  Write data pointed to by block to the given address in the flash.
    */
    void write(uint32_t address, Block* block, void (*_callback)(void))
    {
        callback.attach(_callback);

        internalWrite(address, block);
    }

    template <typename T>
    void write(uint32_t address, Block* block, T* object, void (T::*_callback)(void))
    {
        callback.attach(object, _callback);

        internalWrite(address, block);
    }

    /*  Read from address in flash and store it in the given block.
    */
    void read(uint32_t address, Block* block, void (*_callback)(void))
    {
        callback.attach(_callback);

        internalRead(address, block);
    }

    template <typename T>
    void read(uint32_t address, Block* block, T* object, void (T::*_callback)(void))
    {
        callback.attach(object, _callback);

        internalRead(address, block);
    }

    /*  Register update callback. */
    void onUpdates(void (*callback)(void))
    {
        callbackUpdate.attach(callback);
    }

    template <typename T>
    void onUpdates(T* object, void (T::*callback)(void))
    {
        callbackUpdate.attach(object, callback);
    }

    void internalIRQSet();
    void internalIRQClear();
    void internalErase();

private:
    void internalWrite(uint32_t address, Block* block);
    void internalRead(uint32_t address, Block* block);

    FunctionPointer     callback;
    FunctionPointer     callbackUpdate;
};

#endif // __EXTERNALFLASHSERVER_H__
