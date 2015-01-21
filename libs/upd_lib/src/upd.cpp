/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "upd.h"


upd::upd() :
	m_viewpoint(0.0, 0.0, 0.0),
	m_flip_normals(true),
	m_search_radius(0.5),
	m_k_neighbors(1)
{
    input_cloud.reset( new pcl::PointCloud<pcl::PointXYZRGBA> );
    UPD_cloud.reset( new pcl::PointCloud<pcl::PointSurfel> );
	normals.reset( new pcl::PointCloud<pcl::Normal> );
	r_cloud.reset( new pcl::PointCloud<pcl::Normal> );

}

void upd::runUPD_radius( )
{
    
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    input_cloud->resize(input_cloud->size()*1);
    ne.setRadiusSearch (m_search_radius);
    ne.setInputCloud(input_cloud);
	ne.setViewPoint(m_viewpoint[0], m_viewpoint[1], m_viewpoint[2]);
	//float d = m_viewpoint[2];
    ne.compute(*normals);/**/

	if (m_flip_normals)//flip normals towards a viewpoint
	{
		for (size_t j = 0; j < input_cloud->size (); ++j)
	   {
	   pcl::flipNormalTowardsViewpoint (input_cloud->points[j], m_viewpoint[0], m_viewpoint[1], m_viewpoint[2],  
								        normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
	   }
	}

   r_cloud->resize(input_cloud->size()*1);
   //r_cloud->height = input_cloud->height;
   //r_cloud->width = input_cloud->width;

   calculate_Rcloud_radius();  // TODO - For now only radius is properly tested

   computeUPD();

}

void upd::runUPD_kSearch()
{
//TODO - fix this code repetition
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    input_cloud->resize(input_cloud->size()*1);
    ne.setRadiusSearch (m_search_radius);
    ne.setInputCloud(input_cloud);
	ne.setViewPoint(m_viewpoint[0], m_viewpoint[1], m_viewpoint[2]);
	//float d = m_viewpoint[2];
    ne.compute(*normals);/**/

	if (m_flip_normals)//flip normals towards a viewpoint
	{
		for (size_t j = 0; j < input_cloud->size (); ++j)
	   {
	   pcl::flipNormalTowardsViewpoint (input_cloud->points[j], m_viewpoint[0], m_viewpoint[1], m_viewpoint[2],
								   normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
	   }
	}

   r_cloud->resize(input_cloud->size()*1);
   //r_cloud->height = input_cloud->height;
   //r_cloud->width = input_cloud->width;
   calculate_Rcloud_kSearch();

   computeUPD();
}

void upd::calculate_Rcloud_radius()
{
	// Neighbours within radius search
   Eigen::Vector3d r_vector;
   vector<int> pointIdxRadiusSearch;
   vector<float> pointRadiusSquaredDistance;
   pcl::PointXYZRGBA searchPoint;
   searchPoint.x = input_cloud->points[1].x;
   searchPoint.y = input_cloud->points[1].y;
   searchPoint.z = input_cloud->points[1].z;
   pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
   kdtree.setInputCloud (input_cloud);

    for (size_t j = 0; j < input_cloud->size (); ++j)
    {
      r_cloud->points[j].normal_x = 0;
      r_cloud->points[j].normal_y = 0;
      r_cloud->points[j].normal_z = 0;
      r_cloud->points[j].curvature = 0;
      searchPoint.x = input_cloud->points[j].x;
      searchPoint.y = input_cloud->points[j].y;
      searchPoint.z = input_cloud->points[j].z;

    if ( kdtree.radiusSearch (searchPoint, m_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      {
        r_cloud->points[j].normal_x = r_cloud->points[j].normal_x+normals->points[ pointIdxRadiusSearch[i] ].normal_x;
        r_cloud->points[j].normal_y = r_cloud->points[j].normal_y+normals->points[ pointIdxRadiusSearch[i] ].normal_y;
        r_cloud->points[j].normal_z = r_cloud->points[j].normal_z+normals->points[ pointIdxRadiusSearch[i] ].normal_z;
      }
      r_vector(0)= r_cloud->points[j].normal_x;
      r_vector(1)= r_cloud->points[j].normal_y;
      r_vector(2)= r_cloud->points[j].normal_z;
      float r_norm=r_vector.norm();

      r_cloud->points[j].curvature = r_norm/pointIdxRadiusSearch.size ();
      if (r_norm<(pointIdxRadiusSearch.size ()/2)*M_SQRT2)
      {
        r_cloud->points[j].curvature = 0;//r_vector.norm()/(pointIdxRadiusSearch.size ()+1);
      }
    }
  }
}


void upd::calculate_Rcloud_kSearch( )
{
	// K nearest neighbor search
   Eigen::Vector3d r_vector;
   vector<int> pointIdxNKNSearch(m_k_neighbors);
   vector<float> pointNKNSquaredDistance(m_k_neighbors);
   pcl::PointXYZRGBA searchPoint;
   searchPoint.x = input_cloud->points[1].x;
   searchPoint.y = input_cloud->points[1].y;
   searchPoint.z = input_cloud->points[1].z;

   pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
   kdtree.setInputCloud (input_cloud);

   for (size_t j = 0; j < input_cloud->size (); ++j)
   {
     r_cloud->points[j].normal_x = 0;
     r_cloud->points[j].normal_y = 0;
     r_cloud->points[j].normal_z = 0;
     r_cloud->points[j].curvature = 0;
     searchPoint.x = input_cloud->points[j].x;
     searchPoint.y = input_cloud->points[j].y;
     searchPoint.z = input_cloud->points[j].z;

   if ( kdtree.nearestKSearch (searchPoint, m_k_neighbors, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
   {
     for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
     {
       r_cloud->points[j].normal_x = r_cloud->points[j].normal_x+normals->points[ pointIdxNKNSearch[i] ].normal_x;
       r_cloud->points[j].normal_y = r_cloud->points[j].normal_y+normals->points[ pointIdxNKNSearch[i] ].normal_y;
       r_cloud->points[j].normal_z = r_cloud->points[j].normal_z+normals->points[ pointIdxNKNSearch[i] ].normal_z;
     }
     r_vector(0)= r_cloud->points[j].normal_x;
     r_vector(1)= r_cloud->points[j].normal_y;
     r_vector(2)= r_cloud->points[j].normal_z;
     r_cloud->points[j].curvature = r_vector.norm()/pointIdxNKNSearch.size (); 
     if (r_vector.norm()<(pointIdxNKNSearch.size ()/2)*M_SQRT2)
      {
       r_cloud->points[j].curvature = 0;
      }

   }
 }
}

void upd::computeUPD()
{
   ///UPD CLOUD CREATING
   //float rgb_value;
   //colored_cloud->clear();
   //colored_cloud->resize(input_cloud->size()*1);

      UPD_cloud->resize(input_cloud->size()*1);
      for (size_t j = 0; j < input_cloud->size (); ++j)
      {
        UPD_cloud->points[j].x = input_cloud->points[j].x;
        UPD_cloud->points[j].y = input_cloud->points[j].y;
        UPD_cloud->points[j].z = input_cloud->points[j].z;
        UPD_cloud->points[j].rgba = input_cloud->points[j].rgba;
        UPD_cloud->points[j].normal_x = r_cloud->points[j].normal_x;
        UPD_cloud->points[j].normal_y = r_cloud->points[j].normal_y;
        UPD_cloud->points[j].normal_z = r_cloud->points[j].normal_z;
        UPD_cloud->points[j].radius = r_cloud->points[j].curvature;
//TODO -- finish this implementation about confidence 
        //float sink = sqrt(r_cloud->points[j].normal_x*r_cloud->points[j].normal_x+
        //                  r_cloud->points[j].normal_y*r_cloud->points[j].normal_y);
        UPD_cloud->points[j].confidence = 0;//(360/3.1415926536)*acos(r_cloud->points[j].normal_y/sink);
        //sink = sqrt(r_cloud->points[j].normal_z*r_cloud->points[j].normal_z+
                          //r_cloud->points[j].normal_y*r_cloud->points[j].normal_y);
        UPD_cloud->points[j].curvature =0;// (360/3.1415926536)*acos(r_cloud->points[j].normal_y/sink);


      }
      UPD_cloud->height = input_cloud->height;
      UPD_cloud->width = input_cloud->width;


}

bool upd::readUPDcloud(std::string _fileName)
{
	if (pcl::io::loadPCDFile (_fileName, *UPD_cloud) == -1) return false;
	else return true;

}

bool upd::writeUPDcloud(std::string _fileName)
{
//TODO  -- finish this implementation 
    pcl::PCDWriter w;
    //s.writeASCII<PointXYZRGBA> ("../Exp_data/terrainAnalysisDataset/in.pcd", *image_cloud);
    //s.writeASCII<Normal> ("../Exp_data/terrainAnalysisDataset/Out.pcd", *normals);
    //std::string str_time1 = "";
    //str_time1 = file_name.substr (file_name.length()-17,file_name.length());
    //ss << results_output_dir << "/UPD_" << str_time1;
    //file_name = ss.str ();
    if(w.writeASCII<pcl::PointSurfel> (_fileName, *UPD_cloud) == -1) return false;
	else
		return true;
    //cout << "Saved " << UPD_cloud ->size () << " data points to " << file_name << " ... " << endl;
    //ss.str("");/**/
    //s.writeASCII<PointXYZRGB> ("../Exp_data/terrainAnalysisDataset/tmp_cloud.pcd", *tmp_cloud);
    //s.writeASCII<Normal> ("../Exp_data/terrainAnalysisDataset/normal_cloud.pcd", *normals);


}


void upd::getAsColorMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _rgb_upd_cloud, 
						double _traversability_index_threshold, double _suitable_angle)
{
	_rgb_upd_cloud->resize(UPD_cloud->size());
	for (unsigned int i = 0; i< UPD_cloud->size(); i++)
	{
     _rgb_upd_cloud->points[i].x = UPD_cloud->points[i].x;
     _rgb_upd_cloud->points[i].y = UPD_cloud->points[i].y;
     _rgb_upd_cloud->points[i].z = UPD_cloud->points[i].z;
	 float rgb_value = 0;

	 if (tan(UPD_cloud->points[i].normal_x/UPD_cloud->points[i].normal_y) > tan(_suitable_angle) ||
		 tan(UPD_cloud->points[i].normal_z/UPD_cloud->points[i].normal_y) > tan(_suitable_angle))
	 {
	 _rgb_upd_cloud->points[i].rgba = 0x00FF0000;   //set color to RED in case of orientation fail
	 }
	 else {
		 if(UPD_cloud->points[i].radius > _traversability_index_threshold )//&& UPD_cloud->points[i].curvature<0.9999)
		{
			rgb_value = (UPD_cloud->points[i].radius-_traversability_index_threshold)/(1-_traversability_index_threshold);
			_rgb_upd_cloud->points[i].rgba = GiveRainbowColor(rgb_value);
		}
		else
		{
			rgb_value = (UPD_cloud->points[i].radius-0.7)/0.3;
			_rgb_upd_cloud->points[i].rgba = 0x00000000;//GiveRainbowColor(rgb_value);
		}

	 }

	}


}

uint32_t upd::GiveRainbowColor(float position)
{
	
// this function gives 1D linear RGB color gradient
// color is proportional to position
// position  <0;1>
// position means position of color in color gradient

  if (position>1)position=1;//position-int(position);
  // if position > 1 then we have repetition of colors
  // it maybe useful
  uint8_t R = 0;// byte
  uint8_t G = 0;// byte
  uint8_t B = 0;// byte
  int nmax=6;// number of color bars
  float m=nmax* position;
  int n=int(m); // integer of m
  float f=m-n;  // fraction of m
  uint8_t t=int(f*255);


switch( n){
   case 0:
    {
     R = 0;
     G = 255;
     B = t;
       break;
    }

   case 1:
    {
     R = 0;
     G = 255 - t;
     B = 255;
       break;
    }
   case 2:
    {
     R = t;
     G = 0;
     B = 255;
       break;
    }
   case 3:
    {
     R = 255;
     G = 0;
     B = 255 - t;
       break;
    }
   case 4:
    {
     R = 255;
     G = t;
     B = 0;
       break;
    }
   case 5: {
     R = 255 - t;
     G = 255;
     B = 0;
       break;
    }
   case 6:
    {
     R = 0;
     G = 255;
     B = 0;
       break;
    }

}; // case


  return (R << 16) | (G << 8) | B;
}
