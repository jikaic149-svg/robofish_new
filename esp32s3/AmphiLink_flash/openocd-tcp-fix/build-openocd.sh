#!/usr/bin/env bash
set -euo pipefail
export PATH=/ucrt64/bin:/usr/bin:$PATH
export MSYSTEM=UCRT64
cd "$1"
mkdir -p build-tmp
export TMPDIR="$PWD/build-tmp"
if [[ ! -f configure ]]; then ./bootstrap nosubmodule; fi
export CFLAGS='-O2 -g -std=gnu11'
./configure --build=x86_64-w64-mingw32 --host=x86_64-w64-mingw32 \
  --enable-internal-jimtcl --disable-internal-libjaylink \
  --enable-cmsis-dap --disable-jlink --disable-werror --without-capstone \
  --disable-doxygen-html --disable-doxygen-pdf --prefix="$PWD/package"
make -j4
make install
