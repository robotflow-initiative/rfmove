#!/bin/bash
dir=$(pwd)

for f in $(ls)
do
	if [ -h $f ];then
		rm -f "${dir}/$f"
  elif [ -d "$f" ]; then
      for f2 in $(ls $f)
      do
        if [ -h "${f}/${f2}" ];then
          rm -f "${f}/${f2}"
        fi
      done
	fi
done
