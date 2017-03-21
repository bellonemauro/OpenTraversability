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

// include tiny dnn for neural network classification
// TODO : tiny_dnn does not compile in MVSC, so fix this !
// #if MSVC
#include "tiny_dnn/tiny_dnn.h"
// #endif MSVC

// Qt
#include <QMainWindow>
#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QShortcut>
#include "ui_PCL_upd_DEMO.h"

// Point Cloud Library
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
// for filtering
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
//for transformation
#include <pcl/registration/transforms.h>
// this is the only header to be included to implement a cloud SVM classifier
#include <pcl/ml/svm_wrapper.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Boost
#include <boost/filesystem.hpp>

// UPD
#include <upd.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// --------------------------------------------------------------------
// -----Create our own point type with feature and classification -----
// --------------------------------------------------------------------

struct PointClassifiable
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int idx;        // It's the feature index. It has to be an integer number greater or equal to zero.
  float feature;  // The value assigned to the correspondent feature.
  float label;    // The label value. It is a mandatory to train the classifier.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

// here we assume a XYZ + "idx" + "feature"
POINT_CLOUD_REGISTER_POINT_STRUCT (PointClassifiable,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, idx, idx)
                                  (float, feature, feature)
                                  (float, label, label)
)

using namespace std;

namespace Ui
{
  class PCL_upd_DEMO;
}


class PCL_upd_DEMO : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCL_upd_DEMO (QWidget *parent = 0);
  ~PCL_upd_DEMO ();

   /**  Point picking callback,
     *   press shift + click to activate
     *
     */
    void pp_callback ( const pcl::visualization::PointPickingEvent& event, void* args);


    /**  All the mouse event can be passed from the visualizer,
      *
      */
    void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* _viewer);


    /**  Extract a patch in a specific point from the point cloud to be labelled,
      *
      */
    void extractPatch(double _size, float _x, float _y, float _z);



private slots:
 /** change the point size dimension for the visualization
   * \note 
   */
  void
  pointSizeSliderValueChanged (int value);

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

  /** save labeled cloud
    * \note
    */
  void
  saveLabeledFile();

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
  applyMLS();

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

  /** \brief Start and stop a manual labeling procedure
    * \note the procedure allows to label as grund or not ground an intere scenario using point by point labelling
    * \note
    */
  void
  startStopLabelling();

  /** \brief Label the point as "GROUND"
    * \note  it just copy the point that is enphazed in the cloud in the labeled set
    * \note
    */
  void 
  labelGround();

  /** \brief Label the point as "NOT GROUND"
    * \note  it just copy the point that is enphazed in the cloud in the labeled set
    * \note
    */
  void
  labelNotGround();

  /** \brief Allows the user to select a point in the labeled point list and correct its value
    *
	**/
  void
  selectPointLabel();

  /** \brief clear the last labelling session
    *
	**/
  void 
  clearLabelling ();

  /** \brief set a new unevenness value
    * \note
    */
  void 
  unevenessSliderChange(int _value);

  /** \brief set new angle for the traversability analysis
    * \note
    */
  void 
  angleSliderChange(int _value);

  /** \brief Changes between UPD colormap visualization and normal RGB-D visualization
    * \note
    */
  void
  switchVisualization();

  /** \brief Add coordinate system in the visualizer
    * \note
    */
  void
  addRemoveCoordinateSystem();

  /** \brief run UPD
    * \note
    */
  void 
  runUPD();

  void
  runUPDpatch();

  /// SVM Classification block

  void
  trainSVMClassifier();

  void
  saveClassifierModel();

  void
  saveTrainingDataset();

  void
  saveParameters();

  void
  saveTrainingProblem();

  void
  loadClassifierModel();

  void
  loadTrainingDataset();

  void
  loadTrainingProblem();

  void
  loadSVMparameters();

  void
  loadClassificationProblem();

  /**  \brief Run a test using the crossvalidation
    *  \note
    */
  void
  SVMclassificationTest();

  /**  \brief Classify a single patch (button 'what's this?')
    *  \note
    */
  std::vector< std::vector<double> >
  SVMpatchClassification();

  /**  \brief This function classifies every patch in a point cloud
    *  \note  For now it extracts 10 random patch and classifies them
    *  \return
    */
  void
  SVMclassifyCloud();

  /**  \brief This function is useful to generate training data,
    *  \note  it will ask 50 times what a patch is in order to generate training
    *  \return
    */
  void
  SVMgenerateTraining();


  /// CNN Classification block

  void
  trainCNN();

  void
  CNNclassification();

  void
  testCNN();

  /** \brief Normalize data for CNN
   *  \note
   *  \return
   */
  void
  normalizeCNNdata();

  void
  optimizationChanged(int _value);

  void
  adaptTrainingSetFromSVMdataset();

  /** visualize the about dialog
    * \note
    */
  void 
  about();

  
  void
  updateImagesView();



protected:
  
 /** \brief add data to the training set according to a set of features,
  *  \note -- deprecated
  *  \return
  */
//  void
//  addSVMdataToTrainingSet(float _f1, float _f2, float _f3, float _label );

  /** \brief add data to the SVM training set according to a set of features,
   *  \note generalization to consider a not specified size vector
   *  \return
   */
  void
  addSVMdataToTrainingSet(std::vector<float> _features, float _label );

  /** \brief add data to the CNN training set according to a set of features,
   *  \note generalization to consider a not specified size vector
   *  \return
   */
  void
  addCNNdataToTrainingSet(std::vector<float> _features, uint32_t _label );

  /** \brief create 4 features according to this reference,
   *  \note  G. Reina, and A. Milella
   *         "Towards autonomous agriculture: Automatic ground detection using trinocular stereovision."
   *         Sensors 12.9 (2012): 12405-12423.
   *  \return
   */
  std::vector<float>
  createFeaturesReina( );

  /** \brief create color features according to this reference,
   *  \note  R. Blomley, M. Weinmann, J. Leitloff, and B. Jutzi
   *         "Shape Distribution Features for Point Cloud Analysis
   *          - A Geometric Histogram Approach on Multiple Scales"
   *          SPRS Annals of the Photogrammetry,
   *          Remote Sensing and Spatial Information Sciences, Volume II-3, 2014 .
   *  \return
   */
  std::vector<float>
  createFeaturesShape( );


  /** \brief create color features according to this reference,
   *  \note  Pedro Santana, Magno Guedes, Luis Correia and Jose Barata
   *         "A Saliency-Based Solution for Robust Off-Road Obstacle Detection."
   *          IEEE International Conference on Robotics and Automation (ICRA), 2010 .
   *  \return
   */
  std::vector<float>
  createColorFeatures( );

  /** \brief create 4 features based on UPD,
   *  \note
   *  \return
   */
  std::vector<float>
  createFeaturesUPD();


  /** \brief create all the features we need according to the user selection
   *  \note
   *  \return
   */
  std::vector<float>
  createAllFeatures();

  /** \brief initialize the cnn according to the user parameter
   *  \note
   *  \return
   */
  void
  initCNN();



  void
  addSVMdataToTable(std::vector<pcl::SVMData> _data);

  void
  tryCNNfunctions();


  QShortcut *shortcut_ground;
  QShortcut *shortcut_notground;
	
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr m_cloud;           			//--> allocate a cloud for visualization and data processing
  PointCloudT::Ptr m_cloud_classifiedGNG;		//--> allocate a cloud for classification visualization \green = ground -- \red = not-ground
  PointCloudT::Ptr m_cloud_filtered;  			//--> allocate a cloud for filtering
  PointCloudT::Ptr m_cloud_color_UPD;           //--> allocate a cloud for visualization and data processing
  PointCloudT::Ptr m_labeled_cloud;				//--> labeled cloud - a colored cloud is used to label ground and not ground - green = ground --- red = NOT ground 
  PointCloudT::Ptr m_clicked_points_3d;         //--> single point to detect a click in the visualizer
  PointCloudT::Ptr m_labeled_point;             //--> single point to show a point that can be labels - see function for more details
  pcl::PointCloud<pcl::PointSurfel>::Ptr m_cloud_patch;               //--> extracted patch from a point cloud
  pcl::PointCloud<PointClassifiable>::Ptr m_classifiable_cloud;  //--> define a classifiable point cloud to be used for SVM

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> m_rgb_color;

  upd *m_upd;
  pcl::PointCloud<pcl::PointSurfel>::Ptr UPD_cloud;     //--> processed cloud
  Eigen::Affine3f m_transformation;             //--> transformation matrix
  
  QString m_path_to_pcd_list;      //--> container for the filename list, it is expected for the user to link a list of pcd files
  QString m_path_to_image_list;    //--> container for the filename list, it is expected for the user to link a list of pcd files
  QString m_folder_to_list;        //--> the application is expected to extract the folder
  QString m_safe_folder;	       //--> safe results folder, it is set as default to source data dir, see PCL_upd_DEMO::openFileList for details
  QStringList m_file_pcd_list;     //--> vector of paths to pcd files
  QStringList m_file_image_list;   //--> vector of paths to pcd files

  pcl::SVMTrain m_svm_trainer; //--> our trainer, to be used for store training data or for a new training procedure
  pcl::SVMClassify m_svm_classifier;  //--> our classifier
  pcl::SVMModel m_svm_model;   //--> classifier model, this is automatically generated after the training or loaded for the classification
  pcl::SVMParam m_svm_parameters; //--> our own configuration parameters
  std::vector<pcl::SVMData> m_svm_training_set;   //--> the training set is a vector of data
  std::vector<pcl::SVMData> m_svm_test_set;   //--> the test set is a vector of data



  /** \brief My custom structure for data points in CNN
   */
   struct tiny_dnn_data_point
   {
       tiny_dnn::label_t _label;   // a single label identify the classe of a vector
       tiny_dnn::vec_t   _vec;     // this is 1 single vector o data (e.g. one image)

   };

   struct  tiny_dnn_dataset     //this vector contains 1 label for each vector image
   {
    std::vector<tiny_dnn_data_point> _data;
    std::vector<float> _data_mean;
    std::vector<float> _data_std;

    std::vector<tiny_dnn::vec_t> dataVectors_t ()
    {
        std::vector<tiny_dnn::vec_t> _out;
        for (unsigned int i=0; i<this->_data.size(); i++)
        {
            _out.push_back(this->_data.at(i)._vec);

        }
     return _out;
    }

    std::vector<tiny_dnn::label_t> dataLabels_t ()
    {
        std::vector<tiny_dnn::label_t> _out;
        for (unsigned int i=0; i<this->_data.size(); i++)
        {
            _out.push_back(this->_data.at(i)._label);
        }
     return _out;
    }


    void normalize_data()
    {
        if (this->_data.size() < 1 )
        {
            return;
        }
// implementation of a standardization method x'=(x-E(x))/sigma

       this->_data_mean.resize(this->_data.at(0)._vec.size());
       this->_data_std.resize(this->_data.at(0)._vec.size());

        std::vector<float> sum;
        std::vector<float> sq_sum;
        sum.resize(this->_data.at(0)._vec.size());
        sq_sum.resize(this->_data.at(0)._vec.size());
        for ( int i = 0; i < this->_data.size(); i++)
       {

           for ( int j = 0; j < this->_data.at(i)._vec.size(); j++)
           {
             sum.at(j) += this->_data.at(i)._vec.at(j);
           }

           for ( int j = 0; j < this->_data.at(i)._vec.size(); j++)
           {
             this->_data_mean.at(j) = sum.at(j)/this->_data.at(i)._vec.size();
           }

           for ( int j = 0; j < this->_data.at(i)._vec.size(); j++)
           {
             sq_sum.at(j) += std::pow(this->_data.at(i)._vec.at(j) - this->_data_mean.at(j),2);
           }

           for ( int j = 0; j < this->_data.at(i)._vec.size(); j++)
           {
             this->_data_std.at(j) = std::sqrt(sq_sum.at(j)/this->_data.at(i)._vec.size());
           }

       }

        for ( int i = 0; i < this->_data.size(); i++)
        {
            for ( int j = 0; j < this->_data.at(i)._vec.size(); j++)
            {
                this->_data.at(i)._vec.at(j) = (this->_data.at(i)._vec.at(j) - this->_data_mean.at(j))/this->_data_std.at(j);
            }
        }

        return;
    }


   };

   tiny_dnn_dataset m_cnn_training_set;  //--> the training set is a vector of tiny cnn data
   tiny_dnn_dataset m_cnn_test_set;  //--> the training set is a vector of tiny cnn data
   tiny_dnn::network<tiny_dnn::sequential> m_mlp;  // defines a multilayer perceptron (MLP)
   double m_alpha_adagrad;
   double m_alpha_RMSprop;
   double m_mu_RMSprop;
   double m_alpha_adam;
   double m_b1_adam;
   double m_b2_adam;
   double m_b1_t_adam;
   double m_b2_t_adam;
   double m_alpha_sgd;
   double m_lambda_sgd;
   double m_alpha_sgdm;
   double m_lambda_sgdm;
   double m_mu_sgdm;
   size_t m_cnn_batch_size;
   int m_cnn_epoch;

  unsigned int red;
  unsigned int green;
  unsigned int blue;
  unsigned int _label_counter;
  unsigned int m_patch_labelling_index;  //--> is the index of the center point of the extracted patch
  bool m_labelling_active;
  bool m_labelled_paused;

  struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr m_clicked_points_3d;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
  //callback_args(pcl::visualization::PointPickingEvent& , callback_args){}
  //pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

  struct callback_args cb_args;




  /**  Copy the specific patch to a classifiable point cloud with assigned label and feature
    *
    */
  void copyPatchToClassifiable(const pcl::PointCloud<pcl::PointSurfel> &_cloud_in,
                               pcl::PointCloud<PointClassifiable> &_cloud_out,
                               float _label);

  /**  Call this to be sure that the parameters are properly passed from the UI to the SVM params
    *
    */
  void getGUI_SVMclassifierParams();

  void getCNNoptimizationParams( );

private:

  Ui::PCL_upd_DEMO *ui;    //--> the user interface


};


#endif /* PCL_Kinect2SDK_DEMO_H_ */
