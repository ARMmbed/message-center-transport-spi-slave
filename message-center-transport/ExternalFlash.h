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

#ifndef __EXTERNALFLASH_H__
#define __EXTERNALFLASH_H__

typedef enum {
    FLASH_CMD_WRITE         = 0x01,
    FLASH_CMD_READ          = 0x02,
    FLASH_CMD_ERASE         = 0x04,
    FLASH_CMD_READ_CONTINUE = 0xFF
} flash_command_t;

#endif // __EXTERNALFLASH_H__
