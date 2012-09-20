#!/bin/sh
########################################################################### ###
#@File
#@Title         Test the nature of the C compiler.
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

LANG=C
export LANG

usage() {
	echo "usage: $0 [--64] --cc CC --out OUT [cflag]"
	exit 1
}

# NOTE: The program passed to the compiler is deliberately incorrect
# (`return;' should be `return 0;') but we do this to emit a warning.
#
# Emitting a warning is necessary to get GCC to print out additional
# warnings about any unsupported -Wno options, so we can handle these
# as unsupported by the build.
#
do_cc() {
	echo "int main(void){return;}" | $CC -W -Wall $3 -xc -c - -o $1 >$2 2>&1
}

while [ 1 ]; do
	if [ "$1" = "--64" ]; then
		BIT_CHECK=1
	elif [ "$1" = "--cc" ]; then
		[ "x$2" = "x" ] && usage
		CC="$2" && shift
	elif [ "$1" = "--out" ]; then
		[ "x$2" = "x" ] && usage
		OUT="$2" && shift
	elif [ "${1#--}" != "$1" ]; then
		usage
	else
		break
	fi
	shift
done

[ "x$CC" = "x" ] && usage
[ "x$OUT" = "x" ] && usage
ccof=$OUT/cc-sanity-check
log=${ccof}.log

if [ "x$BIT_CHECK" = "x1" ]; then
	do_cc $ccof $log ""
	file $ccof | grep -q 64-bit
	[ "$?" = "0" ] && echo true || echo false
else
	[ "x$1" = "x" ] && usage
	do_cc $ccof $log $1
	if [ "$?" = "0" ]; then
		# compile passed, but was the warning unrecognized?
		grep -q "^cc1: warning: unrecognized command line option \"$1\"" $log
		[ "$?" = "1" ] && echo $1
	fi
fi

rm -f $ccof $log
exit 0
