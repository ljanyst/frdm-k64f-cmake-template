#-------------------------------------------------------------------------------
# Copyright (c) 2016 by Lukasz Janyst <lukasz@jany.st>
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED 'AS IS' AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
# PERFORMANCE OF THIS SOFTWARE.
#-------------------------------------------------------------------------------

include(CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(TM4C_FLAGS "-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp")
set(TM4C_FLAGS "${TM4C_FLAGS} -O0 -ffunction-sections -fdata-sections ")
set(TM4C_FLAGS "${TM4C_FLAGS} -Wall -pedantic")
set(CMAKE_C_FLAGS "${TM4C_FLAGS} -std=c11" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections,-T,${CMAKE_SOURCE_DIR}/mk64f/MK64F.ld" CACHE STRING "" FORCE)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
