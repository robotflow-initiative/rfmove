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
    py::class_<shapes::Sphere, shapes::Shape, std::shared_ptr<shapes::Sphere>>(m, "Sphere")
        ///Initialize a box with three axes.
        // r
        .def(py::init<double>())
        .def("__repr__", [](shapes::Sphere& self){
            std::ostringstream sstr;
            sstr << "<shapes::Sphere object at " << &self << '>' << std::endl;
            self.print(sstr);
            return sstr.str();
        });
    py::class_<shapes::Cylinder, shapes::Shape, std::shared_ptr<shapes::Cylinder>>(m, "Cylinder")
        ///Initialize a box with three axes.
        // r ,l
        .def(py::init<double,double>())
        .def("__repr__", [](shapes::Cylinder& self){
            std::ostringstream sstr;
            sstr << "<shapes::Cylinder object at " << &self << '>' << std::endl;
            self.print(sstr);
            return sstr.str();
        });
    py::class_<shapes::Cone, shapes::Shape, std::shared_ptr<shapes::Cone>>(m, "Cone")
        ///Initialize a box with three axes.
        // r ,l
        .def(py::init<double,double>())
        .def("__repr__", [](shapes::Cone& self){
            std::ostringstream sstr;
            sstr << "<shapes::Cone object at " << &self << '>' << std::endl;
            self.print(sstr);
            return sstr.str();
        });
    py::class_<shapes::Plane, shapes::Shape, std::shared_ptr<shapes::Plane>>(m, "Plane")
        ///Initialize a box with three axes.
        //  Definition of a plane with equation ax + by + cz + d = 0 */
        .def(py::init<double,double,double,double>())
        .def("__repr__", [](shapes::Plane& self){
            std::ostringstream sstr;
            sstr << "<shapes::Plane object at " << &self << '>' << std::endl;
            self.print(sstr);
            return sstr.str();
        });
}

#endif //MOVEIT_NO_ROS_SHAPES_H
