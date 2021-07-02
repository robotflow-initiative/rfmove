#!/bin/bash
# Indent a path with separators.
# Usage ./path_indent.sh <paths> <separator> <indent>

if [ $# -lt 2 ]
then
  exit
fi

for i in $(echo $1 | tr ";" "\n")
do
  cp -r "${i}"/* "$2"
done
