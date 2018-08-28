#
#    Copyright (c) 2015-2018 Nest Labs, Inc.
#    All rights reserved.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

#
#    Description:
#      This makefile builds the bootloader library for NlPlatform nRF52x. It
#      compiles sources that are also compiled by the top-level Makefile. This
#      is safe because the build system isolates object files in the .build
#      directory according to the name of the Makefile that produced them.
#

include pre.mak

VPATH                       = src

ARCHIVES                    = nlplatform_nrf52x_boot

nlplatform_nrf52x_boot_SOURCES      = cstartup_nortos.c \
                                      nlcrypto.c \
                                      nlflash_nrf52x.c \
                                      nlgpio.c \
                                      nlplatform.c \
                                      nlspi.c \
                                      nltimer.c \
                                      nluart.c \
                                      nlwatchdog.c \
                                      nllibc-lite-support.c

ifeq ($(BUILD_FEATURE_PLATFORM_SPI_SLAVE),1)
nlplatform_nrf52x_boot_SOURCES     += nlspi_slave.c
endif

ifeq ($(BUILD_FEATURE_SWO_VIRTUAL_UART),1)
nlplatform_nrf52x_boot_SOURCES     += swo_vuart.c
endif

nlplatform_nrf52x_boot_INCLUDES     = $(CMSISIncludePaths) \
                                      $(MicroECCIncludePaths) \
                                      $(NlBootloaderIncludePaths) \
                                      $(NlCryptoIncludePaths) \
                                      $(NlPlatformIncludePaths) \
                                      $(NlPlatformNRF52xIncludePaths) \
                                      $(NlAssertIncludePaths) \
                                      $(FreeRTOSIncludePaths)

nlplatform_nrf52x_boot_DEFINES      = NL_ASSERT_PRODUCTION \
                                      NL_BOOTLOADER \
                                      NL_NO_RTOS \
                                      STATIC_HASH_RAM_BUFFER_SIZE=4096

nlplatform_nrf52x_boot_CPPFLAGS     = -Werror

.DEFAULT_GOAL = all

include post.mak
