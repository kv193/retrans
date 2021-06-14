#!/bin/bash

set +x
PRJ_NAME=retrans
echo '**** GCC version ****'
arm-none-eabi-gcc --version

echo 'Generating binary and Printing size information:'
arm-none-eabi-objcopy -O binary "${PRJ_NAME}.elf" "${PRJ_NAME}.bin"; 
arm-none-eabi-size "${PRJ_NAME}.elf"
