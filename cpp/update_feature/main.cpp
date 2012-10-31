#include <pcl/io/pcd_io.h>
#include "util.h"
#include <fstream>
#include <pcl/io/ply_io.h>
#include "feature_extraction.h"
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
}
