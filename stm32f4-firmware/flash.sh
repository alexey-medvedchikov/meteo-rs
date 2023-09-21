#!/bin/sh

if [ $# != 1 ]; then
    echo "Usage:"
    echo "$0 <filename of firmware in ELF format>"
    exit 1
fi

openocd \
  -f interface/stlink-v2.cfg \
  -f target/stm32f4x.cfg \
  -c init \
  -c targets \
  -c 'reset init' \
  -c 'poll off' \
  -c "flash write_image erase $1" \
  -c "verify_image $1" \
  -c 'reset run' \
  -c shutdown
