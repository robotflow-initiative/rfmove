/**
 * Binding geometric_shapes package into python.
 */

#ifndef MOVEIT_NO_ROS_SHAPES_H
#define MOVEIT_NO_ROS_SHAPES_H

#include <geometric_shapes/shapes.h>
#include <pybind11/pybind11.h>
#include <sstream>

namespace py = pybind11;

void declare_shapes(py::module &m){
    py::class_<shapes::Shape, std::shared_ptr<shapes::Shape>>(m, "Shapes");
    py::class_<shapes::Box, shapes::Shape, std::shared_ptr<shapes::Box>>(m, "Box")
        ///Initialize a box with three axes.
        .def(py::init<double, double, double>())
        .def("__repr__", [](shapes::Box& self){
            std::ostringstream sstr;
            sstr << "<shapes::Box object at " << &self << '>' << std::endl;
            self.print(sstr);
            return sstr.str();
        })
        .def("__str__", [](shapes::Box &self){
            std::ostringstream sstr;
            self.print(sstr);
            return sstr.str();
        });
}

#endif //MOVEIT_NO_ROS_SHAPES_H
