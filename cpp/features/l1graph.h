/*
 * =====================================================================================
 *
 *       Filename:  l1graph.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2012年08月27日 14时05分28秒
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
#include <pcl/kdtree/kdtree_flann.h>
#include <decomp.h>

/*
 * =====================================================================================
 *        Class:  L1Graph
 *  Description:  
 * =====================================================================================
 */
class L1Graph
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        L1Graph (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cidx, std::vector<int>& index, std::vector<std::vector<float> >& features)
            :cloud_(cloud),
            index_(&index),
            features_(&features),
            cidx_(cidx)
        {
            ;
        }                             /* constructor      */
        L1Graph ( const L1Graph &other );   /* copy constructor */
        ~L1Graph () {};                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void L1Minimization(SpMatrix<float>& alpha,std::vector<int>& neighborhood);

        /* ====================  OPERATORS     ======================================= */

        L1Graph& operator = ( const L1Graph &other ); /* assignment operator */

    protected:
        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  DATA MEMBERS  ======================================= */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        int cidx_;
        std::vector<int>* index_;
        std::vector<std::vector<float> >* features_;

}; /* -----  end of class L1Graph  ----- */

