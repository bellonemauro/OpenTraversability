/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */


#ifndef PCL_upd_DEMO_H_
#define PCL_upd_DEMO_H_

// standard libraries
#include <iostream>

// Qt
#include <QMainWindow>
#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>
#include "ui_PCL_upd_DEMO.h"

// Point Cloud Library
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>
// for filtering
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//for transformation
#include <pcl/registration/transforms.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Boost
#include <boost/filesystem.hpp>



// UPD
#include <upd.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

namespace Ui
{
  class PCL_upd_DEMO;
  class Kinect2Framework;
}


class PCL_upd_DEMO : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCL_upd_DEMO (QWidget *parent = 0);
  ~PCL_upd_DEMO ();

    void m_try();

public slots:
 /** change the point size dimension for the visualization
   * \note 
   */
  void
  pSliderValueChanged (int value);

  /** open the file list
   * \note 
   */
  void
  openFileList();

  /** open a single pcd
   * \note 
   */
  void
  openFile();

    /** open a folder -- attempt to automatically list all pcd or binary files
   * \note  
   */
  void  openPCDFolder();
  
  /** open a folder -- attempt ot automatically list all images
   * \note  
   */
  void openImagesFolder();

  /** open a single camera image
   * \note 
   */
  void
  openImage();

  /** open a list of images for dataset
   * \note  
   */
  void
  openImageList();

  /** save the visualized cloud
   * \note  
   */
  void
  saveFile();

  /** save the visualized cloud
   * \note  
   */
  void
  saveUPDFile();

  /** set the results save folder
   * \note 
   */
  void
  setSaveFolder();

  /** update the visualizer on doubleclick on the file list
 * \note
 */
  void
  updatePCDview();

  /** apply passthrogh filter
 * \note
 */
  void
  applyPassthrogh();

  /** apply voxelization
 * \note
 */
  void
  applyVoxelization();

  /** apply SOR
 * \note
 */
  void
  applySOR();

  /** remove applied filters
  * \note
  */
  void
  removeFilters();

  /** apply transformation
  * \note
  */
  void
  applyTransformation();

  /** generate a sample cloud for simulations
  * \note
  */
  void
  GenerateSampleCloud();

  /** set r / k
 * \note
 */
  void 
  setRadiusOrKNeighborsMethod();


 /** set flip
 * \note
 */
  void 
  unevenessSliderChange(int value);

  /** set flip
 * \note
 */
  void 
  angleSliderChange(int value);

  void 
  switchVisualization();


  /** run UPD
 * \note
 */
  void 
  runUPD();

  /** visualize the about dialog
 * \note
 */
  void 
  about();

  void 
  enablePCDview();
  
  void
  updateImagesView();



protected:
  
  void pp_callback ( pcl::visualization::PointPickingEvent& event, void* args);	
	
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;           			//--> allocate a cloud for visualization and data processing
  PointCloudT::Ptr cloud_filtered;  			//--> allocate a cloud for filtering
  PointCloudT::Ptr cloud_color_UPD;             //--> allocate a cloud for visualization and data processing
  PointCloudT::Ptr clicked_points_3d;// (new PointCloudT);

  upd *m_upd;
  pcl::PointCloud<pcl::PointSurfel>::Ptr UPD_cloud;     //--> processed cloud

  Eigen::Affine3f m_transformation;             //--> transformation matrix
  
  QString m_path_to_pcd_list;      //--> container for the filename list, it is expected for the user to link a list of pcd files
  QString m_path_to_image_list;    //--> container for the filename list, it is expected for the user to link a list of pcd files
  QString m_folder_to_list;        //--> the application is expected to extract the folder
  QString m_safe_folder;	       //--> safe results folder, it is set as default to source data dir, see PCL_upd_DEMO::openFileList for details
  QStringList m_file_pcd_list;     //--> vector of paths to pcd files
  QStringList m_file_image_list;   //--> vector of paths to pcd files

  unsigned int red;
  unsigned int green;
  unsigned int blue;


  struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
  callback_args(pcl::visualization::PointPickingEvent& , callback_args){}
  //pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

  struct callback_args cb_args;

private:

  Ui::PCL_upd_DEMO *ui;    //--> the user interface

};


#endif /* PCL_Kinect2SDK_DEMO_H_ */
