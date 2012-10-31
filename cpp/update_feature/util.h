#pragma once

#include <fstream>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <map>


class Util
{
public:
	Util(void){};
	~Util(void){};
	
    //map的索引无序，所以再生成一个vector来按顺序保存ni.txt里的索引
    static void GetNeighborhoodAndWeightsFromFile(int index, std::map<int,double>& r, std::vector<int>& si)
    {
        //文件名写死了，就是debug/nx.txt，要注意文件是从n1.txt开始的
        std::string file="debug/n"+boost::lexical_cast<std::string>(index+1)+".txt";
        std::cout<<"file:"<<file<<std::endl;
        std::ifstream ff(file.c_str());
        int idx;
        double weight;
        while(ff>>idx>>weight){
            si.push_back(idx);
            r.insert(std::make_pair<int,double>(idx,weight));
        }
        ff.close();
    }

    //n为全局状态下的邻域索引，local_index为在特征估计类里得到的局部邻域索引（subpointcloud里的索引），该函数返回local_index在全局中的索引值
    static int GetGlobalIndex(int local_index,std::vector<int>& n)
    {
        return n[local_index];
    }

    //直接从local_index得到其所对应的weight
    static double GetWeight(int local_index,std::vector<int>& n,std::map<int,double>& weight_map)
    {
        int global_index=GetGlobalIndex(local_index,n);
        return weight_map[global_index];
    }
};
