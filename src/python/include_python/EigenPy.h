//
// Created by yongxi on 2021/5/31.
//

#ifndef MOVEIT_NO_ROS_EIGENPY_H
#define MOVEIT_NO_ROS_EIGENPY_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Geometry>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <sstream>

namespace py = pybind11;

template<typename ScalarName, int _Dim, int _Mode, int _Options>
void declare_transform(py::module &m, const std::string &typeName) {
    py::class_<Eigen::Transform<ScalarName, _Dim, _Mode, _Options>>(m, typeName, py::buffer_protocol(), py::dynamic_attr());
        //.def("translation", static_cast
        //    <typename Eigen::Transform<ScalarName, _Dim, _Mode, _Options>::ConstTranslationPart(Eigen::Transform<ScalarName, _Dim, _Mode, _Options>::*)()const>
        //    (&Eigen::Transform<ScalarName, _Dim, _Mode, _Options>::translation));
        //.def("", &TransformClass::rotation);
}

/**
 * Rotate affine matrix <angle> along side <axis>
 * @param affine Served as self pointer.
 * @param angle Rotation angle.
 * @param axis Rotation axis.
 */
void affine3dRotate(Eigen::Affine3d &affine, double angle, Eigen::Vector3d axis) {
    axis.normalize();
    Eigen::AngleAxis<double> rotation(angle, axis);
    affine.rotate(rotation);
    // affine.makeAffine();
}

/**
 * Translate affine matrix.
 * @details The translation is according to global coordinates so it uses Eigen's pretranslate method instead of translate.
 * @param affine Served as self pointer.
 * @param trans 3d Translation vector (x, y, z).
 */
void affine3dTranslate(Eigen::Affine3d &affine, const Eigen::Vector3d& trans) {
    std::cout << trans << std::endl;
    affine.pretranslate(trans);
}

/**
 * Reset affine matrix to 1.
 * @param affine Served as self pointer.
 */
void affine3dReset(Eigen::Affine3d &affine) {
    affine.matrix() << 1,0,0,0,
                       0,1,0,0,
                       0,0,1,0,
                       0,0,0,1;
}

/**
 * Create a Eigen::Quaternion from python list
 * @param data
 * @return
 */
Eigen::Quaterniond createQuaternion(py::array_t<double> data) {
    /// w, x, y, z
    return Eigen::Quaterniond(data.at(3),
                              data.at(0),
                              data.at(1),
                              data.at(2));
}

/**
 * @brief Declare all eigen binding class used by moveit.
 * @param m
 */
void declare_eigen(py::module &m) {
    py::class_<Eigen::Quaterniond> (m, "EigenQuaterniond")
        .def(py::init(&createQuaternion))
        .def("__str__", [](Eigen::Quaterniond &self){
            std::ostringstream sstr;
            sstr << "[ " << self.x()
                 << ", " << self.y()
                 << ", " << self.z()
                 << ", " << self.w()
                 << ']';
            return sstr.str();
        })
        .def("__repr__", [](Eigen::Quaterniond &self){
            std::ostringstream sstr;
            sstr << "<Eigen::Quaterniond at " << &self << '>'<< std::endl;
            sstr << "[ " << self.x()
                 << ", " << self.y()
                 << ", " << self.z()
                 << ", " << self.w()
                 << ']';
            return sstr.str();
        })
        .def_property("value", [](Eigen::Quaterniond &self){
            auto data = new double[4];
            data[0] = self.x();
            data[1] = self.y();
            data[2] = self.z();
            data[3] = self.w();

            py::capsule free_when_done(data, [](void *f) {
                auto data = reinterpret_cast<double*>(f);
                //std::cerr << "Free underlining numpy array memory @" << f << '\n';
                delete [] data;
            });
            return py::array_t<double>(
                    {4},    //shape
                    {sizeof(double)}, //strides
                    data,                   //data pointer
                    free_when_done          //handler for free
            );
        }, nullptr)
        .def_property("matrix", [](Eigen::Quaterniond &self) {
            return self.matrix();
        }, nullptr);

    py::class_<Eigen::Affine3d>(m, "EigenAffine3d")
        /// Note that new Affine matrix has random value.
        .def(py::init())
        .def("reset", &affine3dReset)
        .def("__str__", [](Eigen::Affine3d &self) {
            std::ostringstream sstr;
            sstr << "Translation:\n" << self.translation() << '\n'
                 << "Rotation:\n" << self.rotation() << std::endl;
            return sstr.str();
        })
        .def("__repr__", [](Eigen::Affine3d &self) {
            std::ostringstream sstr;
            sstr << "<EigenAffine3d from Eigen::Affine3d>\n"
                 << "Translation:\n" << self.translation() << '\n'
                 << "Rotation:\n" << self.rotation() << std::endl;
            return sstr.str();
        })
        /// pose should be the inverse affine matrix.
        .def("assign", [](Eigen::Affine3d & self, const py::array_t<double>& pose) -> void {
            // If pose contains more than 16 elements, we only copy the first 16 element.
            // Otherwise we copy all elements in pose.
            size_t copy_size = pose.size() > 16 ? 16 : pose.size();
            memcpy(self.matrix().data(), pose.data(), copy_size*sizeof(double));
        }, "Copy a numpy array data into Affine3d")
        .def_property("rotation", &Eigen::Affine3d::rotation, nullptr)
        .def_property("quaternion",
            [](Eigen::Affine3d& self){
                return Eigen::Quaterniond(self.rotation());
            },
            [](Eigen::Affine3d& self, py::array_t<double>& data){
                /// Set rotation by 4d list
                Eigen::Quaterniond q(data.at(3),
                                     data.at(0),
                                     data.at(1),
                                     data.at(2));

                self.matrix() << 1,0,0,self.translation().x(),
                                 0,1,0,self.translation().y(),
                                 0,0,1,self.translation().z(),
                                 0,0,0,1;

                self.rotate(q.toRotationMatrix());
            })
        .def("rotate", &affine3dRotate)
        //.def("translation", &Eigen::Affine3d::translation)
        /** Return translation as a 3-d numpy array
         *  Note that the data is copyed.*/
        .def_property("translation", [](Eigen::Affine3d &self){
            Eigen::Affine3d::TranslationPart trans = self.translation();
            //std::cout << "Translation within declaration" << trans << std::endl;
            //return std::vector<double>{trans.x(), trans.y(), trans.z()};

            auto data = new double[3];
            data[0] = trans.x();
            data[1] = trans.y();
            data[2] = trans.z();

            py::capsule free_when_done(data, [](void *f) {
                auto data = reinterpret_cast<double*>(f);
                //std::cerr << "Free underlining numpy array memory @" << f << '\n';
                delete [] data;
            });
            return py::array_t<double>(
                {3},    //shape
                {sizeof(double)}, //strides
                data,                   //data pointer
                free_when_done          //handler for free
            );
        }, [](Eigen::Affine3d &self, const py::array_t<double>& trans) -> void {
            size_t copy_size = trans.size() > 3 ? 3 : trans.size();
            memcpy(self.translation().data(), trans.data(), copy_size * sizeof(double));}
        )
        .def("translate", &affine3dTranslate)
        .def_property("inverse", [](Eigen::Affine3d &self){
            return self.inverse(Eigen::TransformTraits::Affine);
        }, nullptr)
        .def(py::self * py::self); // Binding for operator *

        /*
        .def("rotation", [](Eigen::Affine3d &self){
            auto rota = self.rotation();
            std::cout << "Rotation during declaration:\n" << rota << std::endl;
            auto data = new double[rota.size()];
            memcpy(data, rota.data(), rota.size() * sizeof(double));
            py::capsule free_when_done(data, [](void *f) {
                auto data_ =  reinterpret_cast<double*>(f);
                delete [] data_;
            });
            return py::array_t<double>(
                {3,3},
                {3*sizeof(double), sizeof(double)},
                data,
                free_when_done
            );
        })*/

}

#endif //MOVEIT_NO_ROS_EIGENPY_H
