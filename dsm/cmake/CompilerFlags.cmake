# 
# This file is part of DSM.
# 
# Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
# Developed by Jon Zubizarreta <jzubizarreta at ceit dot es>,
# for more information see <https://github.com/jzubizarreta/dsm>.
# If you use this code, please cite the respective publications as
# listed on the above website.
# 
# DSM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DSM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DSM. If not, see <http://www.gnu.org/licenses/>.
# 

message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Version: ${CMAKE_CXX_COMPILER_VERSION}")

if(UNIX)
    add_compile_options("-std=c++14") #needed for compiling Frame.cpp in linux        
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(MSVC)

	if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
        add_definitions(/arch:SSE2)                        #only required for 32 bits
    endif()

    message(STATUS " - Defining _SCL_SECURE_NO_WARNINGS")
    message(STATUS " - Defining NOMINMAX")
    message(STATUS " - Atomic Alignment fix: Instantiated std::atomic<T> with sizeof(T) equal to 2/4/8 and alignof(T) < sizeof(T). (_ENABLE_ATOMIC_ALIGNMENT_FIX)")
    message(STATUS " - Extended Alignment fix: Instantiated std::aligned_storage<Len, Align> with an extended alignment. (_ENABLE_EXTENDED_ALIGNED_STORAGE)")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_SCL_SECURE_NO_WARNINGS 
                                            -D_ENABLE_ATOMIC_ALIGNMENT_FIX 
                                            -D_ENABLE_EXTENDED_ALIGNED_STORAGE 
                                            -DNOMINMAX")

endif(MSVC)
