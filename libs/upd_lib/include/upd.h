/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // Exclude rarely-used stuff from Windows headers
#endif

#include "export.h"

// standard libraries 
#include <iostream>
#include <cmath>

// pcl libraries 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/ml/svm.h>
//#include <pcl/ml/svm_wrapper.h>

// boost libraries 
#include <boost/thread/thread.hpp>

using namespace std;

/**  Simple class for upd 
  *
  *  <b>Usage:</b><br>
  *		- Write me
  *
  *  <b>About the algorithm:</b><br>
  *		- Write me
  *
  *
  * <b>Changes history</b>
  *		- JAN/2015: Creation (MB). 
  *  \ingroup __
  */
 class UPD_EXPORT upd
{
public:

    upd(); //!< Constructor

    //~upd(); //!< Destructor ---- maybe not necessary for now, let the compiler to do it
	
    /* Set input cloud
     *
     **/
    inline void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &_input_cloud) 
							{ pcl::copyPointCloud(*_input_cloud, *input_cloud); }

    /* Set input cloud
     *
     **/
    inline void setInputCloud(pcl::PointCloud<pcl::PointSurfel>::Ptr &_input_cloud)
                            { pcl::copyPointCloud(*_input_cloud, *input_cloud); }

    /* Set the use of flipNormalsTowardViewPoint to avoid sign misinterpretation issues
     *
     **/
    inline void setFlip (bool _flip){ m_flip_normals = _flip;}

 
	/* Set the use of flipNormalsTowardViewPoint to avoid sign misinterpretation issues
	*
	**/
	inline void setColorMapType(bool _colorMap){ m_colorMap = _colorMap; }
	
	
	/* Set the search radius
	 * /note default value is 0.5 m 
	 * /note strongly advisable to set it according users cloud
     *
     **/
    inline void setSearchRadius(double _search_radius){ m_search_radius = _search_radius;}

    /* Set the search radius 
	 * /note default value is 0.5 m 
	 * /note strongly advisable to set it according users cloud
     *
     **/
    inline void setKneighbors(int _k_neighbors){ m_k_neighbors = _k_neighbors;}
	
	/* Set viewpoint
     *
     **/
    inline void setViewPoint (Eigen::Vector3d _m_viewpoint){ m_viewpoint = _m_viewpoint;}

	/* Set viewpoint  overload function
     *
     **/
	inline void setViewPoint (float _x, float _y, float _z){ setViewPoint( Eigen::Vector3d(_x, _y, _z) );}

    /* Run UPD with radius method
     *
     **/
    void runUPD_radius( );

     /* Run UPD with k-search method
     *
     **/
    void runUPD_kSearch( );
	
	/* read a processed upd cloud - return false for any error
     *
     **/
    bool readUPDcloud(std::string _fileName );

	/* read a processed upd cloud - return false for any error
     *
     **/
    bool writeUPDcloud(std::string _fileName );

    /* Get the search radius used in the last calculation
     *
     **/
    inline double getSearchRadius( ){ return m_search_radius;}

    /* get the result upd cloud
     *
     **/
	pcl::PointCloud<pcl::PointSurfel>::Ptr getUPD( ){ return UPD_cloud; }

    /* get the result cloud normals
     *
     **/
	pcl::PointCloud<pcl::Normal>::Ptr getNormals( ){ return normals; }

	/* get the result as a RGB color map in traversability analysis
     *
     **/
	void getAsColorMap(	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _rgb_upd_cloud, 
						double _traversability_index_threshold, double _suitable_angle);




private :

	void calculate_Rcloud_radius( );

	void calculate_Rcloud_kSearch( );

	void computeUPD();

	uint32_t GiveRainbowColor(float position); 

	uint32_t GiveJetColour(double _value, double _vmin, double _vmax);

    //PointCloud<PointXYZRGBA>::Ptr colored_cloud (new PointCloud<PointXYZRGBA>);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud;  //--> define input cloud
    pcl::PointCloud<pcl::PointSurfel>::Ptr UPD_cloud;     //--> processed cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals;            //--> normals
	pcl::PointCloud<pcl::Normal>::Ptr r_cloud;            //--> r_cloud
	Eigen::Vector3d m_viewpoint;                          //--> viewpoint for upd calculation

    bool m_flip_normals;     //--> true to flip
	bool m_colorMap;         //--> true to JET color map, false to rainbow color map
    double m_search_radius;  //--> search radius
	int m_k_neighbors;       //--> number of neighbours
};
