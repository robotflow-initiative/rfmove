#!/bin/bash
dir=$(pwd)

for f in $(ls)
do
	if [[ $f == *.so.* ]];then
		ln -s "${dir}/$f" "${dir}/${f%.so*}.so"
	fi
done

ln -s "${dir}/liboctomap.so.1.8.1" "${dir}/liboctomap.so.1.8"
ln -s "${dir}/liboctomath.so.1.8.1" "${dir}/liboctomath.so.1.8"
ln -s "${dir}/liborocos-kdl.so.1.3.2" "${dir}/liborocos-kdl.so.1.3"
ln -s "${dir}/tinyxml2/libtinyxml2.so.2.2.0" "${dir}/tinyxml2/libtinyxml2.so.2"
ln -s "${dir}/tinyxml2/libtinyxml2.so.2.2.0" "${dir}/tinyxml2/libtinyxml2.so"
ln -s "${dir}/assimp/libassimp.so.3.2.0" "${dir}/assimp/libassimp.so.3"
ln -s "${dir}/assimp/libassimp.so.3.2.0" "${dir}/assimp/libassimp.so"
ln -s "${dir}/log4cxx/liblog4cxx.so.10.0.0" "${dir}/log4cxx/liblog4cxx.so.10"
ln -s "${dir}/log4cxx/liblog4cxx.so.10.0.0" "${dir}/log4cxx/liblog4cxx.so"
ln -s "${dir}/fcl/libfcl.so.0.5.0" "${dir}/fcl/libfcl.so.0.5"
ln -s "${dir}/fcl/libfcl.so.0.5.0" "${dir}/fcl/libfcl.so"
ln -s "${dir}/icu/libicui18n.so.55.1" "${dir}/icu/libicui18n.so.55"
ln -s "${dir}/icu/libicui18n.so.55.1" "${dir}/icu/libicui18n.so"
ln -s "${dir}/icu/libicuuc.so.55.1" "${dir}/icu/libicuuc.so.55"
ln -s "${dir}/icu/libicuuc.so.55.1" "${dir}/icu/libicuuc.so"
ln -s "${dir}/icu/libicudata.so.55.1" "${dir}/icu/libicudata.so.55"
ln -s "${dir}/icu/libicudata.so.55.1" "${dir}/icu/libicudata.so"
ln -s "${dir}/minizip/libminizip.so.1.0.0" "${dir}/minizip/libminizip.so.1"
ln -s "${dir}/apr/libapr-1.so.0.5.2" "${dir}/apr/libapr-1.so.0"
ln -s "${dir}/apr/libaprutil-1.so.0.5.4" "${dir}/apr/libaprutil-1.so.0"
ln -s "${dir}/ccd/libccd.so.2.0" "${dir}/ccd/libccd.so.2"
ln -s "${dir}/curl/libcurl.so.4.4.0" "${dir}/curl/libcurl.so.4"
ln -s "${dir}/libompl.so.1.2.1" "${dir}/libompl.so.12"

for f in $(ls "${dir}/boost")
do
	if [[ "$f" == *.so.* ]];then
		ln -s "${dir}/boost/$f" "${dir}/boost/${f%.so*}.so"
	fi
done
