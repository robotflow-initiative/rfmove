#!/bin/bash
# Indent a path with separators.
# Usage ./path_indent.sh <paths> <separator> <indent>

if [ $# -lt 3 ]
then
  exit
fi

for i in $(echo $1 | tr $2 "\n")
do
  echo "$3$i"
done
