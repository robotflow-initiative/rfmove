#include <opt_utils.h>

// hdparam: a ,alpha ,d ,theta
Matrix4d& Opt_Utils::computeDH_toMatrix(vector<double>& hdparam)
{
    Isometry3d T_r_z=Isometry3d::Identity();
    Isometry3d T_p_z=Isometry3d::Identity();
    Isometry3d T_r_x=Isometry3d::Identity();
    Isometry3d T_p_x=Isometry3d::Identity();

    if(hdparam.size()!=5)
    {
        std::cerr<<"wrong hd parameters!"<<std::endl;
        exit(1);
    }

    this->joint_id=floor(hdparam[0]);

    AngleAxisd zrvector(hdparam[4],Vector3d(0,0,1));
    T_r_z.rotate(zrvector);
    
    T_p_z.pretranslate(Vector3d(0,0,hdparam[3]));
    
    T_p_x.pretranslate(Vector3d(hdparam[1],0,0));
    
    AngleAxisd xrvector(hdparam[2],Vector3d(1,0,0));
    T_r_x.rotate(xrvector);

    this->hdMatrix=T_r_z.matrix()*T_p_z.matrix()*T_p_x.matrix()*T_r_x.matrix();
    return this->hdMatrix;
}

Matrix4d& Opt_Utils::get_DHMatirx()
{
    return this->hdMatrix;
}

Matrix4d& Opt_Utils::computeTrans_toMatrix(vector<double>& transparam)
{   
    this->joint_id=floor(transparam[0]);

    this->transMatrix<<transparam[1],transparam[2],transparam[3],transparam[4],
                       transparam[5],transparam[6],transparam[7],transparam[8],
                       transparam[9],transparam[10],transparam[11],transparam[12],
                       transparam[13],transparam[14],transparam[15],transparam[16];

    return this->transMatrix;
}

Matrix4d& Opt_Utils::get_TransMatrix()
{
    return this->transMatrix;
}

void  Opt_Utils::printDH_matrix()
{
    std::cout<<this->hdMatrix<<std::endl;
}

void Opt_Utils::printTrans_matrix()
{
    std::cout<<this->transMatrix<<std::endl;
}