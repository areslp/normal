/*
 * =====================================================================================
 *
 *       Filename:  feature_extraction.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  2012年08月26日 13时18分32秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include "feature_extraction.h"
#include "util.h"

#define K 6 //TODO:increase K, normal estimation, estimate radius
#define NK 15 //K for normal
#define LK 20 //定义一个较大的K，使search surface里的点都能被包括进去

void
FeatureExtraction::EstimateRadius(int idx)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> flannkdtree;
    flannkdtree.setInputCloud(cloud_);
    pcl::PointXYZ searchPoint = cloud_->points[idx];
    // K nearest neighbor search
    std::vector<int> pointIdxNKNSearch(K + 1);
    std::vector<float> pointNKNSquaredDistance(K + 1);

    if(flannkdtree.nearestKSearch(searchPoint, K + 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for(size_t i = 0; i < pointNKNSquaredDistance.size(); ++i) {
            if(pointIdxNKNSearch[i] == 0) {
                continue;
            }

            std::cout << "Squared Distance is:" << pointNKNSquaredDistance[i] << std::endl;
            radius_ += sqrt(pointNKNSquaredDistance[i]);
        }

        std::cout << "radius is:" << radius_ << std::endl;
        radius_ /= K;
    }

    std::cout << "radius is:" << radius_ << std::endl;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if(flannkdtree.radiusSearch(searchPoint, radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        std::cout << "search with radius " << radius_ << ": result contains " << pointIdxRadiusSearch.size() << " points" << std::endl;

        for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    "  <<   cloud_->points[ pointIdxRadiusSearch[i] ].x
                      << " " << cloud_->points[ pointIdxRadiusSearch[i] ].y
                      << " " << cloud_->points[ pointIdxRadiusSearch[i] ].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    return ;
}       /* -----  end of method FeatureExtraction::EstimateRadius  ----- */

void
FeatureExtraction::EstimateNormal(pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    normals->clear();
    normals->resize(cloud_->size());
    for(int i = 0; i < cloud_->size(); i++) {
        std::vector<int> n; //邻域点
        std::map<int,double> weight_map;
        Util::GetNeighborhoodAndWeightsFromFile(i,weight_map,n); //得到i点的邻域和权值-索引隐射
        // n.push_back(i); //n doesn't contains the query point itself //应该是不需要当前点自己吧
        // 用n里的点创建一个子点云，子点云的点顺序应该是和n里的点顺序一致的，我们就可以利用这一点，在估算SC时，用得到的索引反算出它在n里的位置
        // 这里注意：由于n中并不包含i自己，因此subcloud中并不包含i点
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud_, n, *subcloud);
        // Assertion: 子点云的点顺序应该是和n里的点顺序一致的
        // for (int j = 0; j < n.size(); j++) {
            // int global_index=n[j];
            // int local_index=j;
            // pcl::PointXYZ p1=cloud_->points[global_index];
            // pcl::PointXYZ p2=subcloud->points[local_index];
            // std::cout<<"p1: x:"<<p1.x<<" y:"<<p1.y<<" z:"<<p1.z<<std::endl;
            // std::cout<<"p2: x:"<<p2.x<<" y:"<<p2.y<<" z:"<<p2.z<<std::endl;
        // }

        pcl::WeightedNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        // 为NormalEstimation设置基本信息
        normalEstimation.setInputCloud(cloud_);
        normalEstimation.setSearchMethod(tree_);
        normalEstimation.setRadiusSearch(LK * radius_);
        normalEstimation.setSearchSurface(subcloud);
        // 这个类不需要设置weight和n，在头文件里直接读ni.txt了
        //
        // 只求indicesptr里指定点
        boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int> ());
        indicesptr->push_back(i);
        normalEstimation.setIndices(indicesptr);
        pcl::PointCloud<pcl::Normal>::Ptr outn(new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute(*outn);
        std::cout<<"outn size:"<<outn->size()<<std::endl;
        normals->points[i] = outn->at(0);
    }
    f_dim_ += 3;
    return ;
}       /* -----  end of method FeatureExtraction::EstimateNormal  ----- */


void
FeatureExtraction::EstimateShapeContext(pcl::PointCloud<pcl::ShapeContext>::Ptr shapeContextFeatures)
{
    shapeContextFeatures->clear();
    shapeContextFeatures->resize(cloud_->size());
    for(int i = 0; i < cloud_->size(); i++) {
        std::vector<int> n; //邻域点
        std::map<int,double> weight_map;
        Util::GetNeighborhoodAndWeightsFromFile(i,weight_map,n); //得到i点的邻域和权值-索引隐射
        // n.push_back(i); //n doesn't contains the query point itself //应该是不需要当前点自己吧
        // 用n里的点创建一个子点云，子点云的点顺序应该是和n里的点顺序一致的，我们就可以利用这一点，在估算SC时，用得到的索引反算出它在n里的位置
        // 这里注意：由于n中并不包含i自己，因此subcloud中并不包含i点
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud_, n, *subcloud);
        // 将subpointcloud中点对应的normal取出来
        pcl::PointCloud<pcl::Normal>::Ptr subnormal(new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud<pcl::Normal>(*normals_, n, *subnormal);
        // Assertion: 子点云的点顺序应该是和n里的点顺序一致的
        // for (int j = 0; j < n.size(); j++) {
            // int global_index=n[j];
            // int local_index=j;
            // pcl::PointXYZ p1=cloud_->points[global_index];
            // pcl::PointXYZ p2=subcloud->points[local_index];
            // std::cout<<"p1: x:"<<p1.x<<" y:"<<p1.y<<" z:"<<p1.z<<std::endl;
            // std::cout<<"p2: x:"<<p2.x<<" y:"<<p2.y<<" z:"<<p2.z<<std::endl;
        // }

        pcl::WeightedShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext > shapeContext;
        // 为SC设置基本信息
        shapeContext.setInputCloud(cloud_);
        shapeContext.setInputNormals(subnormal); //TODO: out is passed in, but normals_ didn't, this is not consist
        shapeContext.setSearchMethod(tree_);
        shapeContext.setMinimalRadius(radius_);
        shapeContext.setRadiusSearch(LK * radius_);
        shapeContext.setSearchSurface(subcloud);
        // 为SC设置Weighted信息
        shapeContext.setWeightsMap(weight_map);
        shapeContext.setGlobalIndex(n);
        // 只求indicesptr里指定点的SC
        boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int> ());
        indicesptr->push_back(i);
        shapeContext.setIndices(indicesptr);
        pcl::PointCloud<pcl::ShapeContext>::Ptr outn(new pcl::PointCloud<pcl::ShapeContext>);
        shapeContext.compute(*outn);
        shapeContextFeatures->points[i] = outn->at(0);
    }
    // Display and retrieve the shape context descriptor vector for the 0th point.
    f_dim_ += shapeContextFeatures->points[0].descriptor.size();
    std::cout << "shape context contains " << shapeContextFeatures->points[0].descriptor.size() << " features" << std::endl;
    return ;
}       /* -----  end of method FeatureExtraction::EstimateShapeContext  ----- */


void
FeatureExtraction::EstimateSpinImage(pcl::PointCloud<SpinImage>::Ptr spinImages)
{
    spinImages->clear();
    spinImages->resize(cloud_->size());
    for(int i = 0; i < cloud_->size(); i++) {
        std::vector<int> n; //邻域点
        std::map<int,double> weight_map;
        Util::GetNeighborhoodAndWeightsFromFile(i,weight_map,n); //得到i点的邻域和权值-索引隐射
        // n.push_back(i); //n doesn't contains the query point itself //应该是不需要当前点自己吧
        // 用n里的点创建一个子点云，子点云的点顺序应该是和n里的点顺序一致的，我们就可以利用这一点，在估算SC时，用得到的索引反算出它在n里的位置
        // 这里注意：由于n中并不包含i自己，因此subcloud中并不包含i点
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud_, n, *subcloud);
        // 将subpointcloud中点对应的normal取出来
        pcl::PointCloud<pcl::Normal>::Ptr subnormal(new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud<pcl::Normal>(*normals_, n, *subnormal);
        // Assertion: 子点云的点顺序应该是和n里的点顺序一致的
        // for (int j = 0; j < n.size(); j++) {
            // int global_index=n[j];
            // int local_index=j;
            // pcl::PointXYZ p1=cloud_->points[global_index];
            // pcl::PointXYZ p2=subcloud->points[local_index];
            // std::cout<<"p1: x:"<<p1.x<<" y:"<<p1.y<<" z:"<<p1.z<<std::endl;
            // std::cout<<"p2: x:"<<p2.x<<" y:"<<p2.y<<" z:"<<p2.z<<std::endl;
        // }

        pcl::WeightedSpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> spinImageEstimation(8, 0, K);
        // 设置SI的基本信息
        spinImageEstimation.setInputCloud(cloud_);
        spinImageEstimation.setInputNormals(normals_); //TODO: out is passed in, but normals_ didn't, this is not consist
        spinImageEstimation.setSearchMethod(tree_);
        spinImageEstimation.setRadiusSearch(LK * radius_);
        spinImageEstimation.setSearchSurface(subcloud);
        // 设置Weighted信息
        spinImageEstimation.setWeightsMap(weight_map);
        spinImageEstimation.setGlobalIndex(n);
        // 只求indicesptr里点的SI
        boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int> ());
        indicesptr->push_back(i);
        spinImageEstimation.setIndices(indicesptr);
        pcl::PointCloud<SpinImage>::Ptr outn(new pcl::PointCloud<SpinImage>);
        spinImageEstimation.compute(*outn);
        spinImages->points[i] = outn->at(0);
    }
    f_dim_ += 153;
    return ;
}       /* -----  end of method FeatureExtraction::EstimateSpinImage  ----- */

void
FeatureExtraction::GetFeatureVector(std::vector<std::vector<float> >& features)
{
    EstimateNormal(normals_); //Normal不作为特征加入特征向量
    EstimateShapeContext(shapeContextFeatures_); //主要还是用SC
    // EstimateSpinImage(spinImages_); //SP暂时不用
    int p_num = cloud_->size();
    features.resize(p_num);

    for(size_t i = 0; i < features.size(); i++) {
        features[i].reserve(f_dim_);
    }

    for(int i = 0; i < cloud_->size(); i++) {
        std::vector<float>& fperp = features[i];
        //point coordinate
        fperp.push_back(cloud_->points[i].x);
        fperp.push_back(cloud_->points[i].y);
        fperp.push_back(cloud_->points[i].z);
        //normal
        fperp.push_back(normals_->points[i].normal_x);
        fperp.push_back(normals_->points[i].normal_y);
        fperp.push_back(normals_->points[i].normal_z);
        // shape context descriptor
        std::copy(shapeContextFeatures_->points[i].descriptor.begin(), shapeContextFeatures_->points[i].descriptor.end(), std::back_inserter(fperp));
        //spin image descriptor
        std::copy(spinImages_->points[i].histogram, spinImages_->points[i].histogram + 153, std::back_inserter(fperp));
        assert(fperp.size() == f_dim_);
    }

    // std::cout << features[0][0] << std::endl;

    //normalize
    for(int i = 0; i < features.size(); i++) {
        std::vector<float>& fperp = features[i];
        float norm = 0;

        for(int j = 0; j < fperp.size(); j++) {
            norm += fperp[j] * fperp[j];
        }

        norm = sqrt(norm);

        for(int j = 0; j < fperp.size(); j++) {
            fperp[j] = fperp[j] / norm;
        }
    }

    // std::cout << features[0][0] << std::endl;
    return ;
}       /* -----  end of method FeatureExtraction::GetFeatureVector  ----- */

