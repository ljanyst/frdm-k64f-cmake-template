
frdm-k64f-cmake-template
========================

This is a start-up template for pojects using FRDM-K64F boards. The example code
blinks the LED without the need for any external library.

Look here for some more info:

 * http://jany.st/post/2017-03-05-frdm-k64f-linux-cmake.html

Usage
-----

```cmake
#-------------------------------------------------------------------------------
# Some boilerplate
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/MK64F_toolchain.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
include(Firmware)

#-------------------------------------------------------------------------------
# Configure your project
#-------------------------------------------------------------------------------
project(mk64f-template)
add_executable(mk64f-template.axf main.c mk64f/MK64F_startup.c)
add_raw_binary(mk64f-template.bin mk64f-template.axf)
```

License
-------

Copyright (c) 2017 by Lukasz Janyst <lukasz@jany.st>

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED 'AS IS' AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
