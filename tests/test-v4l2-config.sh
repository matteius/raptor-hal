#!/bin/sh
set -eu
test_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
headers=${INGENIC_HEADERS:?Set INGENIC_HEADERS to the ingenic-headers checkout}
output=$(mktemp -d /tmp/raptor-hal-v4l2.XXXXXX)
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -DPLATFORM_T41 -DV4L2_OPENIMP \
    -I"$headers/T41/1.2.5/en" -I"$test_dir/../include" -I"$test_dir/../src" \
    -ffunction-sections -fdata-sections -fsanitize=address,undefined \
    "$test_dir/hal_v4l2_config_test.c" -Wl,--gc-sections -o "$output/config"
"$output/config"
printf 'Test binary: %s/config\n' "$output"
