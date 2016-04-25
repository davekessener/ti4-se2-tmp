#!/bin/bash

ssh dave@serialA "cd git/ti4-se2-tmp && ./serial 2>&1 > logA.txt"
ssh dave@serialB "cd git/ti4-se2-tmp && ./serial 2>&1 > logB.txt"
scp dave@serialA:/home/dave/git/ti4-se2-tmp/logA.txt .
scp dave@serialB:/home/dave/git/ti4-se2-tmp/logB.txt .

