//
// Created by yongxi on 2021/7/28.
//

#include "urdf_converter.h"
#include <sstream>
#include <fstream>

UrdfConverter::UrdfConverter(std::istream &input, const std::string& absolute_dir_path) {
    std::ostringstream sstr;
    sstr << input.rdbuf();
    raw_ = sstr.str();
    absolute_dir_path_ = absolute_dir_path;
}

bool UrdfConverter::convert(std::ostream &output) {
    output << std::regex_replace(raw_, package_prefix_, local_prefix_ + absolute_dir_path_);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: urdf_converter <input file path> <replaced absolute path> [<output file path>]" << std::endl;
        return 0;
    }

    std::ifstream input(argv[1]);
    UrdfConverter* converter;
    if(argc < 3) {
        converter = new UrdfConverter(input);
    } else {
        converter = new UrdfConverter(input, argv[2]);
    }
    input.close();

    if(argc < 4) {
        converter -> convert(std::cout);
    } else {
        std::ofstream output(argv[3]);
        converter -> convert(output);
        output.close();
    }
    delete converter;
    return 0;
}