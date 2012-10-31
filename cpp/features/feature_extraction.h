/*
 * =====================================================================================
 *
 *       Filename:  feature_extraction.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2012年08月26日 13时14分26秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/kdtree/kdtree_flann.h>
/*
 * =====================================================================================
 *        Class:  FeatureExtraction
 *  Description:  
 * =====================================================================================
 */
class FeatureExtraction
{
    public:
        typedef pcl::Histogram<153> SpinImage;
        /* ====================  LIFECYCLE     ======================================= */
        FeatureExtraction (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud):
            tree_(new pcl::search::KdTree<pcl::PointXYZ> ()),
            cloud_(cloud),
            normals_(new pcl::PointCloud<pcl::Normal>),
            shapeContextFeatures_(new pcl::PointCloud<pcl::ShapeContext>),
            spinImages_ (new pcl::PointCloud<SpinImage> ()),
            f_dim_(3), //point coordinate
            useN_(false),
            radius_(0.0)
            {
                EstimateRadius(0);
            }; /* constructor      */
        FeatureExtraction ( const FeatureExtraction &other );   /* copy constructor */
        ~FeatureExtraction () {};                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void GetFeatureVector(std::vector<std::vector<float> >&); 
        int GetFeatureDimension(){ return f_dim_; }
        float GetRadius(){ return radius_; }
        void setUseN(bool f){ useN_=f; }
        void setN(std::vector<std::vector<int> >& nn){ n_=&nn; }

        /* ====================  OPERATORS     ======================================= */

        FeatureExtraction& operator = ( const FeatureExtraction &other ); /* assignment operator */

    protected:
        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  DATA MEMBERS  ======================================= */
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        pcl::PointCloud<pcl::ShapeContext>::Ptr shapeContextFeatures_;
        pcl::PointCloud<SpinImage>::Ptr spinImages_;
        int f_dim_;
        float radius_;
        //method
        void EstimateRadius(int);
        void EstimateNormal(pcl::PointCloud<pcl::Normal>::Ptr);
        void EstimateShapeContext(pcl::PointCloud<pcl::ShapeContext>::Ptr);
        void EstimateSpinImage(pcl::PointCloud<SpinImage>::Ptr);
        //
        bool useN_;
        std::vector<std::vector<int> > *n_;

}; /* -----  end of class FeatureExtraction  ----- */

