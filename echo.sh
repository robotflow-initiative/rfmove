#!/bin/bash
var=$(python -c 'import site; print(site.getsitepackages()[0])')
echo ${var}
