#!/bin/bash

function echo_exec
{
	echo "$1"
	sh -c "$1" || exit 1
}

fs=`find . -name "*.o"`
for f in $fs ; do
	echo_exec "rm -f $f"
done

echo_exec "rm -f testserial"

