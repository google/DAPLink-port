#!/bin/bash
REPO_ROOT="$(pwd)"

python3 -m virtualenv -p python3 "${REPO_ROOT}/venv"
source "${REPO_ROOT}/venv/bin/activate"

set -eux

python3 -m pip install -r "${REPO_ROOT}/requirements.txt"

pushd "$REPO_ROOT"

python3 ./tools/add_macro_to_record.py ./records/board/stm32h743ii_udb_if.yaml "UDB_VERSION_BASE=${bamboo_VersionBase}"
python3 ./tools/add_macro_to_record.py ./records/board/stm32h743ii_udb_if.yaml "UDB_BUILD_NUMBER=${bamboo_buildNumber}"

export ARM_GCC_PATH=/usr/local/nestlabs/toolchains/gcc-arm-7.3.1/bin/

# Build generic daplink interface and bootloader applications to check they still compile
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_if --clean --parallel
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_bl --clean --parallel

# When the DAPLINK_BOOTLOADER_UPDATE macro is defined in the interface, the interface
# will include the bootloader binary into itself so that it can update the bootloader
# on boot. Bundle a known working BL version to make sure we stay in control of the
# bootloader version and only update it when necessary.
BOOTLOADER_IMAGE_TO_BUNDLE_INTO_INTERFACE="0.12d25"
BL_BUILD_OUTPUT_PATH="projectfiles/make_gcc_arm/stm32h743ii_udb_bl/build"
IF_BUILD_OUTPUT_PATH="projectfiles/make_gcc_arm/stm32h743ii_udb_if/build"
# 1) Build the UDB bootloader.
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_udb_bl --clean --parallel
# 2) Rename the freshly build bootloader_image.c, because we want to upload this to the
#    cloud for potential later use.
mv ${BL_BUILD_OUTPUT_PATH}/bootloader_image.c ${BL_BUILD_OUTPUT_PATH}/bootloader_image_temp.c
# 3) Download the known the bootloader that we want to bundle into the interface. Place it
#    into the bootloader build output folder, because that's where the interface build looks
#    for it.
gsutil cp "gs://images-store/UDB-DAPLink/Builds/${BOOTLOADER_IMAGE_TO_BUNDLE_INTO_INTERFACE}/bootloader/bootloader_image.c" ${BL_BUILD_OUTPUT_PATH}
# 4) Build the UDB daplink interface application.
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_udb_if --clean --parallel
# 5) Progen_compile generates a new garbage bootloader_image.c in the interface output folder.
#    Replace the garbage file with the actual file used during compilation so that the
#    deliver step can upload the correct record to the cloud.
mv ${BL_BUILD_OUTPUT_PATH}/bootloader_image.c ${IF_BUILD_OUTPUT_PATH}
# 6) Rename the original freshly built bootloader image back to it's original name, so that
#    the deliver step uploads it correctly in case we want to update the desired bootloader
#    to this build's
mv ${BL_BUILD_OUTPUT_PATH}/bootloader_image_temp.c ${BL_BUILD_OUTPUT_PATH}/bootloader_image.c
