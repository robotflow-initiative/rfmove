#include<csv_loader.h>
#include<opt_utils.h>

int main(int argc,char **argv){
    if(argc!=3)
    {
        std::cerr<<"输出的参数不正确,请重新输入"<<std::endl;
    }
    csv_loader csvloader(argv[1],argv[2]);
    auto dhlist=csvloader.get_dhlist();
    auto translist=csvloader.get_translist();
    Matrix4d dh_m=Matrix4d::Identity();
    for(int i=0;i<dhlist.size();i++)
    {
        //csvloader.printVector(dhlist[i]);
        //csvloader.printVector(translist[i]);
        Opt_Utils pu(dhlist[i],translist[i]);
        std::cout<<"DH_matrix==============================="<<std::endl;
        //pu.printDH_matrix();
        dh_m=dh_m*pu.get_DHMatirx();
        std::cout<<dh_m<<std::endl;
        std::cout<<"Trans_matrix============================"<<std::endl;
        pu.printTrans_matrix();

    }
}