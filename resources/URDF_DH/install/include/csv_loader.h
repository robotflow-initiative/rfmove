#ifndef CSV_LOADER_H
#define CSV_LOADER_H

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <iomanip>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

class csv_loader{
public:
    enum loaderType{hd,trans};

    csv_loader(string hdfile,string transfile);
    
    //output dh parameter list
    vector<vector<double>>& get_dhlist();
    //output trans parameter list
    vector<vector<double>>& get_translist();

    void printVector(vector<double>& list);
private:
    
    bool initvector(string file,loaderType lt);
    
    //dhParameter list
    vector<vector<double>> dhlist;
    vector<string> dhtitle;
    
    //transformslist  
    vector<vector<double>> transformslist;
    vector<int> transform_num;
};

#endif