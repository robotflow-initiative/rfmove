//
// Created by yongxi on 2021/5/22.
//
#include "path_util.h"
#include <string>
#include <fstream>
#include <sstream>

std::string ioToString(std::istream& inStream) {
    std::ostringstream sstr;
    sstr << inStream.rdbuf();
    return sstr.str();
}