#!/bin/sh
set -eu
test_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
output=$(mktemp -d /tmp/raptor-hal-annexb.XXXXXX)
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -I"$test_dir/../include" -I"$test_dir/../src" \
    "$test_dir/hal_h264_annexb_test.c" -o "$output/parser"
"$output/parser"
common=${RAPTOR_COMMON:-$test_dir/../../raptor-common}
if [ -f "$common/src/rss_vui.c" ]; then
    "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
        -DTEST_VUI -I"$test_dir/../include" -I"$test_dir/../src" -I"$common/include" \
        "$test_dir/hal_h264_annexb_test.c" "$common/src/rss_vui.c" -o "$output/vui"
    "$output/vui"
else
    printf 'VUI integration not run: set RAPTOR_COMMON to the common checkout\n'
fi
printf 'Test binaries: %s\n' "$output"
