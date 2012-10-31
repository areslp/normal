#include <pcl/io/pcd_io.h>
#include <decomp.h>
#include "util.h"
#include <fstream>
#include <pcl/io/ply_io.h>
#include "feature_extraction.h"
#include "l1graph.h"
#include <pcl/common/io.h>

int main(int argc, char** argv)
{
    if (argc<2) {
        std::cout<<"l1-graph model_name"<<std::endl;
        return 0;
    }
    std::string inname=argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //read point cloud data
    if (pcl::io::loadPLYFile<pcl::PointXYZ> (inname.c_str(), *cloud) == 
            -1) //* load the file 
    { 
        PCL_ERROR ("Couldn't read file test_ply.pl \n"); 
        return (-1); 
    } 
    std::cout<<"cloud size:"<<cloud->size()<<std::endl;
    int p_num = cloud->size();

    FeatureExtraction estimator(cloud);
    std::vector<std::vector<float> > features;
    estimator.GetFeatureVector(features);
    std::cout<<"feature vector size:"<<estimator.GetFeatureDimension()<<std::endl;

    //=======================write features to file for matlab================================
    ofstream of("features.txt");
    std::cout<<"cloud size:"<<cloud->size()<<std::endl;
    for (int i = 0; i < cloud->size(); i++) {
        std::vector<float> fperp = features[i];
        // of<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<" ";
        for (int j = 0; j < fperp.size(); j++) {
            if (j==fperp.size()-1){
                of<<fperp[j]<<std::endl;
            }else{
                of<<fperp[j]<<" ";
            }
        }
    }
    of.close();
    return 0;
    
    std::cout<<"===========================ready for dictionary learning==================="<<std::endl;
    Timer time;
    time.reset();
    time.start();
    //first time
    std::vector<std::vector<int> > neighborhoods;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());
    tree1->setInputCloud(cloud);
    for(int i = 0; i < cloud->size(); i++) {
        if (i!=5143) {
            continue;
        }
        pcl::PointXYZ sp=cloud->points[i];
        std::vector<int> index;
        std::vector<float> dist;
        tree1->nearestKSearch(sp,100,index,dist);

        // std::string namev = "debug/" + boost::lexical_cast<std::string>(i) + "_all.xyznq";
        // ofstream ov(namev.c_str());
        // for(int ii = 0; ii < cloud->size(); ++ii) {
            // if(ii == i) {
                // ov << cloud->points[ii].x << " " << cloud->points[ii].y << " " << cloud->points[ii].z << " 0 0 0 1" << std::endl;
            // } else {
                // std::vector<int>::iterator it = find(index.begin(), index.end(), ii);
                // if(it != index.end()) {
                    // ov << cloud->points[ii].x << " " << cloud->points[ii].y << " " << cloud->points[ii].z << " 0 0 0 1" << std::endl;
                // } else {
                    // ov << cloud->points[ii].x << " " << cloud->points[ii].y << " " << cloud->points[ii].z << " 0 0 0 -1" << std::endl;
                // }
            // }
        // }
        // ov.close();

        std::vector<int> n;
        L1Graph graph(cloud,i,index,features);
        SpMatrix<float> alpha;
        graph.L1Minimization(alpha,n);
        neighborhoods.push_back(n);
        if (n.size()<2) {
            alpha.print("alpha");
            std::cout<<"index:"<<i<<",n:"<<n.size()<<std::endl;
            return -1;
        }
    }
    time.stop();
    std::cout<<"total takes:"<<time.getElapsed()<<std::endl;
    std::cout<<"each point takes:"<<time.getElapsed()/p_num<<std::endl;

    return 0;

    //iteration
    for (int it = 0; it < 5; it++) {
        //update features
        estimator.setUseN(true);
        estimator.setN(neighborhoods);
        features.clear();
        estimator.GetFeatureVector(features);
        //SR
        neighborhoods.clear();
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());
        tree1->setInputCloud(cloud);
        for(int i = 0; i < cloud->size(); i++) {
            pcl::PointXYZ sp=cloud->points[i];
            std::vector<int> index;
            std::vector<float> dist;
            tree1->nearestKSearch(sp,100,index,dist);

            std::vector<int> n;
            L1Graph graph(cloud,i,index,features);
            SpMatrix<float> alpha;
            graph.L1Minimization(alpha,n);
            neighborhoods.push_back(n);
            if (n.size()<2) {
                alpha.print("alpha");
                std::cout<<"index:"<<i<<",n:"<<n.size()<<std::endl;
            }
        }
        //do only one time
        std::cout<<"press any key to continue"<<std::endl;
        getchar();
        break;
    }

    //output final result
    // std::ofstream final("final.xyzn");
    // for (int i = 0; i < cloud->size(); i++) {
        // pcl::PointXYZ point=cloud->at(i);
        // pcl::Normal normal=updatenormals->at(i);
        // final<<point.x<<" "<<point.y<<" "<<point.z<<" "<<normal.normal_x<<" "<<normal.normal_y<<" "<<normal.normal_z<<std::endl;
    // }
    // final.close();


    // std::cout<<"press any key to continue"<<std::endl;
    // getchar();
    // break;

    return 0;
}
