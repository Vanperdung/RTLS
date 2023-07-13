# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ESP32/Espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "D:/Final-Project/FIRMWARE/gateway/build/bootloader"
  "D:/Final-Project/FIRMWARE/gateway/build/bootloader-prefix"
  "D:/Final-Project/FIRMWARE/gateway/build/bootloader-prefix/tmp"
  "D:/Final-Project/FIRMWARE/gateway/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Final-Project/FIRMWARE/gateway/build/bootloader-prefix/src"
  "D:/Final-Project/FIRMWARE/gateway/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Final-Project/FIRMWARE/gateway/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
