#include <csv_loader.h>

double strDou(string str) {	//string 转换 double
	char *ch = new char[0];
	double d;
	for (int i = 0; i != str.length(); i++)
		ch[i] = str[i];
	d = atof(ch);
	return d;
}

csv_loader::csv_loader(string hdfile,string transfile)
{
   initvector(hdfile,loaderType::hd);
   initvector(transfile,loaderType::trans);
}

bool csv_loader::initvector(string file,loaderType lt)
{
    ifstream hd(file, ios::in);
    if (!hd)
    {
        cout << "打开文件失败！" << endl;
        return false;
    }
    string line;
    bool isHeader=true;

    while (getline(hd, line))//getline(inFile, line)表示按行读取CSV文件中的数据
    {
        string field;
        istringstream sin(line); //将整行字符串line读入到字符串流sin中
        if (isHeader)
        {
            while (getline(sin,field, ',')){
                if (lt==loaderType::hd){
                    this->dhtitle.push_back(field);
                }
                if (lt==loaderType::trans){
                    this->transform_num.push_back(atoi(field.c_str()));
                }
            }
            isHeader=false;
        }else{
            if(lt==loaderType::hd)
            {
                vector<double> dhparam;
                while (getline(sin,field, ',')){
                    dhparam.push_back(strDou(field));
                }
                this->dhlist.push_back(dhparam);
            }
            if(lt==loaderType::trans)
            {
                vector<double> transparam;
                while (getline(sin,field, ',')){
                    transparam.push_back(strDou(field));
                }
                this->transformslist.push_back(transparam);
            }
        }
    }
    
    hd.close();
    return true;
}

void csv_loader::printVector(vector<double>& list)
{
    for(int i=0;i<list.size();i++)
    {
        if(i==list.size()-1)
            std::cout<<setw(3)<<std::left<<setprecision(3)<<list[i]<<std::endl;
        else
            std::cout<<setw(3)<<std::left<<setprecision(3)<<list[i];
    }
}
vector<vector<double>>& csv_loader::get_dhlist()
{
    return this->dhlist;
}
vector<vector<double>>& csv_loader::get_translist()
{
    return this->transformslist;
}
