#ifndef OPTUTILS_H
#define OPTUTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Opt_Utils{
public:

    Opt_Utils(vector<double,std::allocator<double>>& hdparam,vector<double> transparam):hdparam_(hdparam),transparam_(transparam)
    {
        this->computeDH_toMatrix(this->hdparam_);
        this->computeTrans_toMatrix(this->transparam_);
    }

    //Eigen
    Matrix4d& computeDH_toMatrix(vector<double>& hdparam_);
    Matrix4d& get_DHMatirx();
    void printDH_matrix();

    //return Matrix4d of transform
    Matrix4d& computeTrans_toMatrix(vector<double>& transfrome_);
    Matrix4d& get_TransMatrix();
    void printTrans_matrix();
private:
    vector<double> hdparam_;
    //引用初始化变量
    Matrix4d hdMatrix;

    vector<double> transparam_;
    Matrix4d transMatrix;
    int joint_id;
};
#endif