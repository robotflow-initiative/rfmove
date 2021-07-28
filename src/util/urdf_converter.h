/**
 * Util used to convert urdf file so that we can use it without ros environment.
 * @details For now, this util only replace all package urls with local file path.
 */

#ifndef MOVEIT_NO_ROS_URDF_CONVERTER_H
#define MOVEIT_NO_ROS_URDF_CONVERTER_H

#include <regex>
#include <iostream>

class UrdfConverter {
public:
    explicit UrdfConverter(std::istream& input, const std::string& absolute_dir_path = "");
    bool convert(std::ostream& output);
private:
    const std::regex package_prefix_ = std::regex(R"(filename\s*=\s*"package://)");
    const std::string local_prefix_ = "filename=\"file://";
    std::string absolute_dir_path_;
    std::string raw_;
};

#endif //MOVEIT_NO_ROS_URDF_CONVERTER_H
