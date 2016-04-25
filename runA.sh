#!/bin/bash

git pull
./make.sh -DACTIVE || exit 1

