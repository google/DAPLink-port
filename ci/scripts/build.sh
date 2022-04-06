#!/bin/bash
REPO_ROOT="$(pwd)"

python3 -m virtualenv -p python3 "${REPO_ROOT}/venv"
source "${REPO_ROOT}/venv/bin/activate"

set -eux

python3 -m pip install -r "${REPO_ROOT}/requirements.txt"

pushd "$REPO_ROOT"

python3 ./tools/add_macro_to_record.py ./records/board/stm32h743ii_udb_if.yaml "UDB_VERSION_BASE=${bamboo_VersionBase}"
python3 ./tools/add_macro_to_record.py ./records/board/stm32h743ii_udb_if.yaml "UDB_BUILD_NUMBER=${bamboo_buildNumber}"

#build generic daplink interface and bootloader applications to check they still compile
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_if --clean --parallel
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_bl --clean --parallel

#build UDB bootloader
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_udb_bl --clean --parallel

# When the DAPLINK_BOOTLOADER_UPDATE macro is defined in the interface, the interface
# will include the bootloader binary into itself so that it can update the bootloader
# on boot. Replace the freshly built bootloader image with a known working BL version to
# make sure we stay in control of the bootloader version and only update it when necessary.
BOOTLOADER_IMAGE_TO_BUNDLE_INTO_INTERFACE="0.12d25"
gsutil cp "gs://images-store/UDB-DAPLink/Builds/${BOOTLOADER_IMAGE_TO_BUNDLE_INTO_INTERFACE}/bootloader/bootloader_image.c" .
cp bootloader_image.c projectfiles/make_gcc_arm/stm32h743ii_udb_bl/build/

#build UDB daplink interface application
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_udb_if --clean --parallel

# Progen_compile generates a garbage bootloader_image.c in the interface output folder.
# To deliver the correct record, copy again the actual file used during compilation.
cp bootloader_image.c projectfiles/make_gcc_arm/stm32h743ii_udb_if/build/
