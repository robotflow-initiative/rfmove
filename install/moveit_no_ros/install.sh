#!/bin/bash
cd lib
bash mklink.sh
cd ..
pythonlib=$(python -c 'import site; print(site.getsitepackages()[0])')
echo "Install moveit_noros to ${pythonlib}"
cp lib/moveit_noros* ${pythonlib}