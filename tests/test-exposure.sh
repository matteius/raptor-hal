#!/bin/sh
set -eu

test_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
headers=${INGENIC_HEADERS:?Set INGENIC_HEADERS to the ingenic-headers checkout}
output=$(mktemp -d /tmp/raptor-hal-exposure.XXXXXX)

for target in T32/1.0.6/en T40/1.3.1/en T41/1.2.5/en; do
    platform=${target%%/*}
    "${CC:-cc}" -std=c11 -Wall -Wextra -Werror \
        -D"PLATFORM_$platform" -I"$headers/$target" \
        -I"$test_dir/../include" -I"$test_dir/../src" \
        -ffunction-sections -fdata-sections \
        "$test_dir/hal_isp_exposure_test.c" "$test_dir/../src/hal_isp.c" \
        -Wl,--gc-sections -o "$output/$platform"
    "$output/$platform"
    printf '%s exposure units: PASS\n' "$platform"
done
printf 'Test binaries: %s\n' "$output"
