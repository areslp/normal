/*
 * =====================================================================================
 *
 *       Filename:  l1graph.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  2012年08月27日 14时14分34秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#include "l1graph.h"
#include <boost/lexical_cast.hpp>
#include "util.h"


void
L1Graph::L1Minimization(SpMatrix<float>& alpha,std::vector<int>& neighborhood)
{
    //neighborhood contains cidx
    int p_num=index_->size();
    int f_dim=(*features_)[0].size();
    Matrix<float> X(f_dim, p_num - 1);

    //construct matrix X
    std::map<int,int> map_gl; //map between global and local point index
    for(int j = 0, k = 0; j < p_num; j++) { //cols
        int id=(*index_)[j];
        if(id==cidx_) {
            continue;
        }
        Util::StdVector2MatrixCol((*features_)[id], X, k);
        map_gl.insert(make_pair<int,int>(k,id));
        ++k;
    }

    //TODO:dont use I
    //construct I
    // Matrix<float> I(f_dim,f_dim);
    // I.eye();
    // Util::Matrix2File(I,"I.txt");
    // Matrix<float> B(f_dim, f_dim + p_num - 1);
    // X.merge(I, B);
    // Util::Matrix2File(B,"B.txt");
    //clean
    // X.clear();
    // I.clear();
    //using lasso
    Matrix<float> x; //x=Ba
    Util::StdVector2Matrix((*features_)[cidx_], x);
    float lambda = 0.01; //lambda
    int L = (*features_)[0].size(); //max non-zero number
    // int L = 20; //max non-zero number
    SpMatrix<float> all;

    //TODO:debug in matlab
    ofstream fout("f.txt");
    for (int i = 0; i < x.m(); i++) {
        for (int j = 0; j < x.n(); j++) {
            fout<<x(i,j)<<" "; 
        }
        fout<<std::endl;
    }
    fout.close();
    ofstream dout("d.txt");
    for (int i = 0; i < X.m(); i++) {
        for (int j = 0; j < X.n(); j++) {
            dout<<X(i,j)<<" "; 
        }
        dout<<std::endl;
    }
    dout.close();


    lasso2<float>(x, X, all, L, lambda , 0 , PENALTY , true); //TODO:X->B

    Util::SubSpMatrix(all,alpha,p_num-1);
    // all.print("all");
    // alpha.print("alpha");
    // getchar();


    Matrix<float> tmp;
    X.mult(alpha,tmp);
    x.add(tmp,-1);
    std::cout<<(0.5*x.normFsq())<<" "<<(lambda*alpha.asum())<<" "<<(0.5*x.normFsq())/(lambda*alpha.asum())<<std::endl;


    //save for vis
    std::vector<int> mk;
    std::vector<float> mv;

    for(int ii = 0; ii < alpha.n(); ++ii) {
        //TODO:calls for pB and pE are not consist
        for(int j = alpha.pB(ii); j < alpha.pE()[ii]; ++j) {
            //<i,j>->all.v(j)
            mk.push_back(map_gl[j]);
            mv.push_back(alpha.v(j));
            neighborhood.push_back(map_gl[j]);
        }
    }

    std::string namev = "debug/" + boost::lexical_cast<std::string>(cidx_) + ".xyznq";
    ofstream ov(namev.c_str());

    for(int ii = 0; ii < cloud_->size(); ++ii) {
        if(ii == cidx_) {
            ov << cloud_->points[ii].x << " " << cloud_->points[ii].y << " " << cloud_->points[ii].z << " 0 0 0 1" << std::endl;
        } else {
            std::vector<int>::iterator it = find(mk.begin(), mk.end(), ii);
            if(it != mk.end()) {
                int dis = std::distance(mk.begin(), it);
                ov << cloud_->points[ii].x << " " << cloud_->points[ii].y << " " << cloud_->points[ii].z << " 0 0 0 " << mv[dis] << std::endl;
                // ov << cloud_->points[ii].x <<" " << cloud_->points[ii].y << " " << cloud_->points[ii].z << " 0 0 0 0" << std::endl;
            } else {
                ov << cloud_->points[ii].x << " " << cloud_->points[ii].y << " " << cloud_->points[ii].z << " 0 0 0 -1" << std::endl;
            }
        }
    }

    ov.close();
    return ;
}       /* -----  end of method L1Graph::L1Minimization  ----- */

