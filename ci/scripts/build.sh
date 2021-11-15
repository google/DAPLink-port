#!/bin/bash
REPO_ROOT="$(pwd)"

python3 -m virtualenv -p python3 "${REPO_ROOT}/venv"
source "${REPO_ROOT}/venv/bin/activate"

set -eux

python3 -m pip install -r "${REPO_ROOT}/requirements.txt"

pushd "$REPO_ROOT"

python3 ./tools/add_macro_to_record.py ./records/hic_hal/stm32h743ii.yaml "UDB_VERSION_BASE=${bamboo_VersionBase}"
python3 ./tools/add_macro_to_record.py ./records/hic_hal/stm32h743ii.yaml "UDB_BUILD_NUMBER=${bamboo_buildNumber}"

#build interface application
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_if --clean --parallel

#build bootloader
python3 ./tools/progen_compile.py -t make_gcc_arm stm32h743ii_bl --clean --parallel
