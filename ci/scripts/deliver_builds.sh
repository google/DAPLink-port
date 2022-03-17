#!/bin/bash

source "${WORKSPACE}/buildtools/images/images-wrapper.sh"

bamboo_BuildVersion="${bamboo_VersionBase}${bamboo_buildNumber}"
TEMPSHARE="$(mktemp -d /tmp/udp-XXXXXXXX)"
trap "rm -rf ${TEMPSHARE}" EXIT

mkdir -p "${TEMPSHARE}/${bamboo_BuildVersion}/interface"
mkdir -p "${TEMPSHARE}/${bamboo_BuildVersion}/bootloader"

find ./projectfiles/make_gcc_arm/stm32h743ii_udb_if/build \( -name "*.c" -o -name "*.bin" -o -name "*.hex" -o -name "*.elf" -o -name "*.txt" -o -name "*.ld" -name "*.map" \) -exec cp {} "${TEMPSHARE}/${bamboo_BuildVersion}/interface" \;
find ./projectfiles/make_gcc_arm/stm32h743ii_udb_bl/build \( -name "*.c" -o -name "*.bin" -o -name "*.hex" -o -name "*.elf" -o -name "*.txt" -o -name "*.ld" -name "*.map" \) -exec cp {} "${TEMPSHARE}/${bamboo_BuildVersion}/bootloader" \;

copy_to_images "$TEMPSHARE/${bamboo_BuildVersion}" "$bamboo_BuildImagesPath"
