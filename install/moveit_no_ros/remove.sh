#!/bin/bash
cd lib
bash rmlink.sh
cd ..
pythonlib=$(python -c 'import site; print(site.getsitepackages()[0])')
echo "Remove moveit_noros from ${pythonlib}"
rm -f ${pythonlib}/moveit_noros*
