#!/bin/bash

target=testserial
active="$1"

function echo_exec
{
	echo "$1"
	sh -c "$1" || exit 1
}

function comp
{
	cmd=`echo -n "$1" | sed 's/^\.\/\(.*\)\([^\/]*\)\.cc$/g++ '$active' -Wall -std=c++03 -ggdb -I. -c \1\2.cc -o \1\2.o/'`
	echo_exec "$cmd"
}

inst=`find . -name "*.cc"`
for f in $inst ; do
	comp $f
done
echo_exec "g++ `find . -name \"*.o\" | sed 's/^\.\///' | xargs echo` -o $target -lpthread"

