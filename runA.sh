#!/bin/bash

./make.sh -DACTIVE || exit 1
./testserial

