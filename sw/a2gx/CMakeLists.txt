#
# Copyright (C) 2013 Vlad Lazarenko <vlad@lazarenko.me>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

cmake_minimum_required(VERSION 2.8)

project(a2gx)

set(A2GX_VERSION_MAJOR 0)
set(A2GX_VERSION_MINOR 0)
set(A2GX_VERSION_PATCH 0)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(A2GX_SRC_INC_DIR "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}")

include_directories("${CMAKE_BINARY_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/include")

include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)

macro(a2gx_add_c_flag flag)
  check_c_compiler_flag(${flag} a2gx_c_has_${flag})
  if (a2gx_c_has_${flag})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${flag}")
  else()
    message("Flag ${flag} is not supported by C compiler.")
  endif()
endmacro()

macro(a2gx_add_cc_flag flag)
  check_cxx_compiler_flag(${flag} a2gx_cc_has_${flag})
  if (a2gx_cc_has_${flag})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
  else()
    message("Flag ${flag} is not supported by C++ compiler.")
  endif()
endmacro()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
foreach(flag
    -fvisibility=hidden -pipe -Wall -Wextra -Winit-self
    -Wmissing-include-dirs -Wno-unused-result -Wfloat-equal
    -Wconversion -Wpacked -Wpointer-arith -Wcast-qual -Wsequence-point
    -Wsuggest-attribute=pure -Wsuggest-attribute=noreturn
    -DNDEBUG)
  a2gx_add_cc_flag(${flag})
  a2gx_add_c_flag(${flag})
endforeach()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  foreach(flag -O3 -flto -march=native -mtune=native)
    a2gx_add_c_flag(${flag})
    a2gx_add_cc_flag(${flag})
  endforeach()
else()
  foreach(flag -O0 -ggdb)
    a2gx_add_c_flag(${flag})
    a2gx_add_cc_flag(${flag})
  endforeach()
endif()

add_subdirectory(src)
add_subdirectory(test)
