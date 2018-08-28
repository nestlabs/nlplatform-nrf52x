#
#    Copyright (c) 2017-2018 Nest Labs, Inc.
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

#
#    Description:
#      This is the top-level Makefile for the nlplatform_nrf52x library. All
#      work toward creating the library is done directly or indirectly by this
#      Makefile.
#

include pre.mak

VPATH                       = src

ARCHIVES                    = nlplatform_nrf52x

nlplatform_nrf52x_SOURCES   = cstartup.c \
                              nladc.c \
                              nlcrypto.c \
                              nlflash_nrf52x.c \
                              nlgpio.c \
                              nli2c.c \
                              nlplatform.c \
                              nlradio_diags.c \
                              nlpwm.c \
                              nlspi.c \
                              nltimer.c \
                              nluart.c \
                              nlwatchdog.c \
                              nllibc-lite-support.c \
                              soc-utils.c \
                              $(NULL)

ifeq ($(BUILD_FEATURE_SWO_VIRTUAL_UART),1)
nlplatform_nrf52x_SOURCES  += swo_vuart.c
endif

ifeq ($(BUILD_FEATURE_PLATFORM_WPAN_RADIO),1)
ifneq ($(BUILD_FEATURE_USE_NORDIC_OT_RADIO_DRIVER),1)
ifeq ($(BUILD_FEATURE_NLRADIO_USES_OT_RADIO_DRIVER),1)
nlplatform_nrf52x_SOURCES  += src/nlradio-nrf52x-ot.c
else
nlplatform_nrf52x_SOURCES  += src/nlradio.c
endif
endif
endif

ifeq ($(BUILD_FEATURE_PLATFORM_SPI_SLAVE),1)
nlplatform_nrf52x_SOURCES  += src/nlspi_slave.c
endif

nlplatform_nrf52x_INCLUDES  = $(NordicIncludePaths) \
                              $(MicroECCIncludePaths) \
                              $(NlBootloaderIncludePaths) \
                              $(NlCryptoIncludePaths) \
                              $(NlPlatformIncludePaths) \
                              $(NlPlatformNRF52xIncludePaths) \
                              $(NlAssertIncludePaths) \
                              $(NlUtilitiesIncludePaths) \
                              $(NlOpenThreadIncludePaths) \
                              $(OpenThreadIncludePaths) \
                              $(FreeRTOSIncludePaths) \
                              $(NlEnvIncludePaths) \
                              $(NLERIncludePaths) \
                              $(NULL)

ifeq ($(BUILD_FEATURE_NLRADIO_USES_OT_RADIO_DRIVER),1)
nlplatform_nrf52x_INCLUDES += $(NordicOTRadioIncludePaths)
endif

nlplatform_nrf52x_DEFINES   = NL_UART_IS_ASYNC

ifeq ($(BUILD_FEATURE_NLRADIO_USES_OT_RADIO_DRIVER),1)
nlplatform_nrf52x_DEFINES  += NRF52840_XXAA \
			      ENABLE_FEM \
			      NEST_NRF52X_DRIVER_CONFIG
endif

nlplatform_nrf52x_CPPFLAGS  = -Werror

# Headers to install
nlplatform_nrf52x_HEADERS   = include/nlgpio_defines.h \
                              include/nlplatform_soc.h \
                              $(NULL)

# These headers we want installed under an additional subdir include/nlplatform_nrf52x/*
SocIncludeFiles             = nrf52x_gpio.h \
                              nlflash_nrf52x.h \
                              soc-utils.h \
                              cstartup.h \
                              $(NULL)

SocIncludePaths = $(foreach headerfile,$(SocIncludeFiles),include/$(headerfile):include/$(NlPlatformNRF52xNames)/$(headerfile))

nlplatform_nrf52x_HEADERS  += $(SocIncludePaths)

.DEFAULT_GOAL = all

SubMakefiles  = bootloader.mak

ifeq ($(BUILD_FEATURE_USE_NORDIC_OT_RADIO_DRIVER),1)
SubMakefiles += nlradio-nrf52x-ot.mak
endif

include post.mak
