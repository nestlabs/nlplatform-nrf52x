#
#    Copyright (c) 2018 Nest Labs, Inc.
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
#    Description: This is a nlplatform_nrf52x sub-make used to build
#                 an indpendent library for the nlradio abstraction
#                 layer for use with the openthread nordic third party
#                 radio driver.  The independent sub-make and library
#                 are required because the OT nordic radio driver has
#                 dependencies to nordic hardware abstraction header
#                 files in the OT nordic radio driver and these header
#                 files will occassionally diverge in name and content
#                 from the nordic hardware abstraction header files in
#                 the nordic SDK that are used by the rest of the drivers
#                 in nlplatform_nrf52x which are built as a single library
#                 by the main nlplatform_nrf52x makefile.  Basically,
#                 this sub-make uses NordicOTRadioIncludePaths instead
#                 of NordicIncludePaths used by the main nlplatform_nrf52x
#                 makefile.
#

include pre.mak

VPATH                       = src

ARCHIVES                    = nlplatform_nrf52x_nlradio

nlplatform_nrf52x_nlradio_SOURCES	= nlradio-nrf52x-ot-v2.c

nlplatform_nrf52x_nlradio_INCLUDES      = $(NlAssertIncludePaths) \
                                          $(NlUtilitiesIncludePaths) \
                                          $(NlEnvIncludePaths) \
                                          $(NlOpenThreadIncludePaths) \
                                          $(OpenThreadIncludePaths) \
                                          $(NlPlatformIncludePaths) \
                                          $(NordicOTRadioIncludePaths) \
										  $(FreeRTOSIncludePaths)

nlplatform_nrf52x_nlradio_CPPFLAGS     = -Werror

.DEFAULT_GOAL = all

include post.mak
