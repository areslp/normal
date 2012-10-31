#pragma once

#include <linalg.h>
#include <fstream>

//template <typename T>
//class Matrix<T>;
//
//template <typename T>
//class Vector<T>;

class Util
{
public:
	Util(void){};
	~Util(void){};
	
    template <typename T>
	static void StdVector2Vector(std::vector<T>& stdvector,Vector<T>& v);
	
    template <typename T>
	static void StdVector2Matrix(std::vector<T>& stdvector,Matrix<T>& m); //copy vector to one column matrix

    template <typename T>
    static void StdVector2MatrixCol(std::vector<T>& stdvector,Matrix<T>& m,int j);

    template <typename T>
    static void Matrix2File(Matrix<T>& m, std::string file);

    //TODO: not implemented
    template <typename T>
    static void SpMatrix2Vec(SpMatrix<T>& m,std::vector<int>& neighborhood);

    template <typename T>
    static void SubSpMatrix(SpMatrix<T>& m,SpMatrix<T>& subsp,int num);
};

template <typename T>
inline void Util::StdVector2Vector(std::vector<T>& stdvector,Vector<T>& v){
	v.resize(stdvector.size());
	for (int i=0;i<stdvector.size();i++)
	{
		v[i]=stdvector[i];
	}
}


template <typename T>
inline void Util::StdVector2Matrix(std::vector<T>& stdvector,Matrix<T>& m)
{
	m.resize(stdvector.size(),1);
    for (int i = 0; i < m.m(); i++) {
        m(i,0)=stdvector[i];
    }
}

template <typename T>
inline void Util::StdVector2MatrixCol(std::vector<T>& stdvector,Matrix<T>& m,int j)
{
    for (int i = 0; i < stdvector.size(); i++) {
        m(i,j)=stdvector[i];
    }
}

template <typename T>
inline void Util::Matrix2File(Matrix<T>& m, std::string file)
{
    ofstream out(file.c_str());
    out.precision(4);
    out.width(10);
    out.setf(std::ios::fixed, std::ios::floatfield);
    for (int i = 0; i < m.m(); i++) {
        for (int j = 0; j < m.n(); j++) {
            out<<m(i,j)<<" ";
        }
        out<<std::endl;
    }
    out.close();
}

template <typename T>
inline void Util::SubSpMatrix(SpMatrix<T>& m,SpMatrix<T>& subsp,int num)
{
    Matrix<T> dense;
    m.toFullTrans(dense);
    Matrix<T> sub_dense;
    dense.refSubMat(0,num,sub_dense); 
    sub_dense.toSparseTrans(subsp);
}
