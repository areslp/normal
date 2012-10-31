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
    Timer time;
    time.reset();
    time.start();

    //normal============================
    if(!useN_) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_);
        ne.setSearchMethod(tree_);
        ne.setKSearch(NK);
        ne.compute(*normals);
    } else {
        normals->clear();
        normals->resize(cloud_->size());

        for(int i = 0; i < cloud_->size(); i++) {
            std::vector<int>& n = (*n_)[i];
            n.push_back(i); //n doesn't contains the query point itself
            pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud_, n, *subcloud);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(cloud_);
            ne.setSearchMethod(tree_);
            ne.setSearchSurface(subcloud);
            ne.setKSearch(100);
            boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int> ());
            indicesptr->push_back(i);
            ne.setIndices(indicesptr);
            pcl::PointCloud<pcl::Normal>::Ptr outn(new pcl::PointCloud<pcl::Normal>);
            ne.compute(*outn);
            normals->points[i] = outn->at(0);
        }
    }

    f_dim_ += 3;
    time.stop();
    std::cout << "normal calculate takes:" << time.getElapsed() << std::endl;
    return ;
}       /* -----  end of method FeatureExtraction::EstimateNormal  ----- */


void
FeatureExtraction::EstimateShapeContext(pcl::PointCloud<pcl::ShapeContext>::Ptr shapeContextFeatures)
{
    Timer time;
    time.start();

    //shape context descriptor============================
    if(!useN_) {
        pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext > shapeContext;
        shapeContext.setAzimuthBins(3);
        shapeContext.setElevationBins(3);
        shapeContext.setRadiusBins(3);
        // shapeContext.setPointDensityRadius (0.008);
        // Provide the original point cloud (without normals)
        shapeContext.setInputCloud(cloud_);
        // Provide the point cloud with normals
        shapeContext.setInputNormals(normals_); //TODO: out is passed in, but normals_ didn't, this is not consist
        // Use the same KdTree from the normal estimation
        shapeContext.setSearchMethod(tree_);
        // The search radius must be set to above the minimal search radius
        shapeContext.setMinimalRadius(radius_);
        shapeContext.setRadiusSearch(3 * radius_);
        // Actually compute the shape contexts
        shapeContext.compute(*shapeContextFeatures);
    } else {
        shapeContextFeatures->clear();
        shapeContextFeatures->resize(cloud_->size());
        for(int i = 0; i < cloud_->size(); i++) {
            std::vector<int>& n = (*n_)[i];
            n.push_back(i); //n doesn't contains the query point itself
            pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud_, n, *subcloud);
            pcl::PointCloud<pcl::Normal>::Ptr subnormal(new pcl::PointCloud<pcl::Normal>);
            pcl::copyPointCloud<pcl::Normal>(*normals_, n, *subnormal);
            // std::cout<<"subnormal:"<<subnormal->points[0]<<std::endl;

            pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext > shapeContext;
            shapeContext.setInputCloud(cloud_);
            shapeContext.setInputNormals(subnormal); //TODO: out is passed in, but normals_ didn't, this is not consist
            shapeContext.setSearchMethod(tree_);
            shapeContext.setMinimalRadius(radius_);
            shapeContext.setRadiusSearch(10 * radius_);
            shapeContext.setSearchSurface(subcloud);
            boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int> ());
            indicesptr->push_back(i);
            shapeContext.setIndices(indicesptr);
            pcl::PointCloud<pcl::ShapeContext>::Ptr outn(new pcl::PointCloud<pcl::ShapeContext>);
            shapeContext.compute(*outn);
            shapeContextFeatures->points[i] = outn->at(0);
        }
    }
    // Display and retrieve the shape context descriptor vector for the 0th point.
    f_dim_ += shapeContextFeatures->points[0].descriptor.size();
    std::cout << "shape context contains " << shapeContextFeatures->points[0].descriptor.size() << " features" << std::endl;
    time.stop();
    std::cout << "shape context takes:" << time.getElapsed() << std::endl;
    return ;
}       /* -----  end of method FeatureExtraction::EstimateShapeContext  ----- */


void
FeatureExtraction::EstimateSpinImage(pcl::PointCloud<SpinImage>::Ptr spinImages)
{
    Timer time;
    time.start();
    //spin image descriptor============================
    if (!useN_) {
        pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> spinImageEstimation(8, 0, K);
        // Provide the original point cloud (without normals)
        spinImageEstimation.setInputCloud(cloud_);
        // Provide the point cloud with normals
        spinImageEstimation.setInputNormals(normals_);
        // Use the same KdTree from the normal estimation
        spinImageEstimation.setSearchMethod(tree_);
        // estimate
        spinImageEstimation.setRadiusSearch(3 * radius_);
        // Actually compute the shape contexts
        spinImageEstimation.compute(*spinImages);
    }else{
        spinImages->clear();
        spinImages->resize(cloud_->size());
        for(int i = 0; i < cloud_->size(); i++) {
            std::vector<int>& n = (*n_)[i];
            n.push_back(i); //n doesn't contains the query point itself
            pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud_, n, *subcloud);

            pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> spinImageEstimation(8, 0, K);
            spinImageEstimation.setInputCloud(cloud_);
            spinImageEstimation.setInputNormals(normals_); //TODO: out is passed in, but normals_ didn't, this is not consist
            spinImageEstimation.setSearchMethod(tree_);
            spinImageEstimation.setRadiusSearch(10 * radius_);
            spinImageEstimation.setSearchSurface(subcloud);
            boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int> ());
            indicesptr->push_back(i);
            spinImageEstimation.setIndices(indicesptr);
            pcl::PointCloud<SpinImage>::Ptr outn(new pcl::PointCloud<SpinImage>);
            spinImageEstimation.compute(*outn);
            spinImages->points[i] = outn->at(0);
        }
    }
    f_dim_ += 153;
    time.stop();
    std::cout << "spin image takes:" << time.getElapsed() << std::endl;
    return ;
}       /* -----  end of method FeatureExtraction::EstimateSpinImage  ----- */

void
FeatureExtraction::GetFeatureVector(std::vector<std::vector<float> >& features)
{
    EstimateNormal(normals_);
    EstimateShapeContext(shapeContextFeatures_);
    EstimateSpinImage(spinImages_);
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

    std::cout << features[0][0] << std::endl;

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

    std::cout << features[0][0] << std::endl;
    return ;
}       /* -----  end of method FeatureExtraction::GetFeatureVector  ----- */

