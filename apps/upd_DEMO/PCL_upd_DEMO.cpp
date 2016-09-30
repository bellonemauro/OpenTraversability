/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"


PCL_upd_DEMO::PCL_upd_DEMO (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCL_upd_DEMO)
{
  ui->setupUi (this);
  setCentralWidget(ui->scrollArea_central);
  this->setWindowTitle (" PCL - Traversability - DEMO");

  // Setup the cloud pointer
  m_cloud.reset (new PointCloudT);
  m_cloud_color_UPD.reset(new PointCloudT);
  m_cloud_filtered.reset (new PointCloudT);
  m_labeled_point.reset (new PointCloudT);
  m_labeled_cloud.reset (new PointCloudT);
  m_cloud_patch.reset (new pcl::PointCloud<pcl::PointSurfel>);
  m_classifiable_cloud.reset (new pcl::PointCloud<PointClassifiable>);
  UPD_cloud.reset( new pcl::PointCloud<pcl::PointSurfel> );
  m_transformation = Eigen::Affine3f::Identity();


  _label_counter = 0;
  m_patch_labelling_index = 0;
  m_labelling_active = false;
  m_labelled_paused = false;

  if (pcl::io::loadPCDFile ("./logo.pcd", *m_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file logo.pcd \n");
	PCL_ERROR ("Using example random cloud \n");
	    // The number of points in the cloud
	m_cloud->points.resize (200);
	    // Fill the cloud with some points
	  for (size_t i = 0; i < m_cloud->points.size (); ++i)
	  {
		m_cloud->points[i].x = 1.024 * rand () / (RAND_MAX + 1.0f);
		m_cloud->points[i].y = 1.024 * rand () / (RAND_MAX + 1.0f);
		m_cloud->points[i].z = 1.024 * rand () / (RAND_MAX + 1.0f);

		m_cloud->points[i].r = red;
		m_cloud->points[i].g = green;
		m_cloud->points[i].b = blue;
	  }


  }

  // set area for image display 
     ui->label_image->setBackgroundRole(QPalette::Base);
     ui->label_image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
     ui->label_image->setScaledContents(true);

     ui->scrollArea_image->setBackgroundRole(QPalette::Dark);
     //ui->scrollArea_image->setWidget(ui->label_image);
     //setCentralWidget(ui->scrollArea_image);
  ui->treeWidget_classification->resizeColumnToContents(0);
  ui->treeWidget_classification->resizeColumnToContents(1);
  ui->treeWidget_classification->resizeColumnToContents(2);
  ui->treeWidget_classification->resizeColumnToContents(3);

      // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  viewer->setBackgroundColor(1.0, 1.0, 1.0);  //set background to black
  ui->qvtkWidget->update ();

  // actions from menu
  connect (ui->actionOpen_list, SIGNAL(triggered()), this, SLOT(openFileList()));
  connect (ui->actionOpen_cloud, SIGNAL(triggered()), this, SLOT(openFile()));
  connect (ui->actionOpen_Image, SIGNAL(triggered()), this, SLOT(openImage()));
  connect (ui->actionOpen_Images_List, SIGNAL(triggered()), this, SLOT(openImageList()));
  connect (ui->actionOpen_PCDFolder, SIGNAL(triggered()), this, SLOT(openPCDFolder()));
  connect (ui->actionImages_Folder, SIGNAL(triggered()), this, SLOT(openImagesFolder()));
  connect (ui->actionSave_Cloud, SIGNAL(triggered()), this, SLOT(saveFile()));
  connect (ui->actionSave_UPD, SIGNAL(triggered()), this, SLOT(saveUPDFile()));
  connect (ui->actionLabelled_Cloud, SIGNAL(triggered()), this, SLOT(saveLabeledFile()));
  connect (ui->actionRemove_Filters, SIGNAL(triggered()), this, SLOT(removeFilters()) );
  connect (ui->actionGenerate_sample_cloud, SIGNAL(triggered()), this, SLOT(GenerateSampleCloud()));

  connect (ui->actionSave_classifier_model, SIGNAL(triggered()), this, SLOT (saveClassifierModel()));
  connect (ui->actionSave_training_set, SIGNAL(triggered()), this, SLOT (saveTrainingDataset()));
  connect (ui->actionSave_parameters, SIGNAL(triggered()), this, SLOT (saveParameters()));
  connect (ui->actionSave_training_problem, SIGNAL(triggered()), this, SLOT (saveTrainingProblem()));

  connect (ui->actionLoad_classifier_model, SIGNAL(triggered()), this, SLOT (loadClassifierModel()));
  connect (ui->actionLoad_training_data, SIGNAL(triggered()), this, SLOT (loadTrainingDataset()));
  connect (ui->actionLoad_parameters, SIGNAL(triggered()), this, SLOT (loadSVMparameters()));
  connect (ui->actionLoad_training_problem, SIGNAL(triggered()), this, SLOT (loadTrainingProblem()));
  connect (ui->actionLoad_classification_problem, SIGNAL(triggered()), this, SLOT (loadClassificationProblem()));

  connect (ui->actionAbout, SIGNAL(triggered()), this, SLOT (about()));




  //buttons
  connect (ui->pushButton_applyPfilter, SIGNAL(clicked()), this, SLOT (applyPassthrogh()));
  connect (ui->pushButton_applyVoxel, SIGNAL(clicked()), this, SLOT (applyVoxelization()));
  connect (ui->pushButton_applySOR, SIGNAL(clicked()), this, SLOT (applySOR()));
  connect (ui->pushButton_applyTransformation, SIGNAL(clicked()), this, SLOT(applyTransformation()));
  connect (ui->pushButton_updateVis, SIGNAL (clicked()), this, SLOT(switchVisualization()));
  connect (ui->pushButton_runUPD, SIGNAL(clicked()), this, SLOT(runUPD()));
  connect (ui->pushButton_start_stopLabelling, SIGNAL(clicked()), this, SLOT (startStopLabelling()));
  connect (ui->pushButton_ground, SIGNAL(clicked()), this, SLOT (labelGround()));
  connect (ui->pushButton_notGround, SIGNAL(clicked()), this, SLOT (labelNotGround()));
  connect (ui->pushButton_clearLabelling, SIGNAL(clicked()), this, SLOT (clearLabelling()));
  connect (ui->pushButton_train, SIGNAL(clicked()), this, SLOT (trainClassifier()));
  //connect (ui->pushButton_saveTrainingSet, SIGNAL(clicked()), this, SLOT (saveTrainingDataset()));
  //connect (ui->pushButton_classModel, SIGNAL(clicked()), this, SLOT (saveClassifierModel()));
  connect (ui->pushButton_classify, SIGNAL(clicked()), this, SLOT (classification()));
  connect (ui->pushButton_smvTest, SIGNAL(clicked()), this, SLOT (classificationTest()));

  // sliders
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pointSizeSliderValueChanged (int)));
  connect (ui->horizontalSlider_unevennessIndex, SIGNAL (valueChanged (int)), this, SLOT (unevenessSliderChange (int)));
  connect (ui->horizontalSlider_traversabilityAngle, SIGNAL (valueChanged (int)), this, SLOT (angleSliderChange (int)));


  // others
  connect (ui->lineEdit_saveFolder, SIGNAL(returnPressed()), this, SLOT (setSaveFolder()));
  connect (ui->listWidget_pcdNames, SIGNAL(itemDoubleClicked (QListWidgetItem*)), this, SLOT(updatePCDview()));
  connect (ui->listWidget_imageNames, SIGNAL(itemDoubleClicked (QListWidgetItem*)), this, SLOT(updateImagesView()));
  connect (ui->treeWidget_classification, SIGNAL(itemDoubleClicked (QTreeWidgetItem *,int)), this, SLOT(selectPointLabel())); 
  connect (ui->radioButton_radius, SIGNAL(clicked()), this, SLOT(setRadiusOrKNeighborsMethod()));
  connect (ui->radioButton_kNeighbors, SIGNAL(clicked()), this, SLOT(setRadiusOrKNeighborsMethod()));

  connect (ui->checkBox_visTraversability, SIGNAL (clicked()), this, SLOT(switchVisualization()));
  connect (ui->checkBox_coordinateSystem, SIGNAL (clicked()), this, SLOT(addRemoveCoordinateSystem()));

   ui->treeWidget_classification->resizeColumnToContents(0);
   ui->treeWidget_classification->resizeColumnToContents(1);

  if (ui->checkBox_coordinateSystem->isChecked()) viewer->addCoordinateSystem ( );
  //viewer->addPointCloud (m_cloud, "cloud");   // new 1.8 requrire the color attributes to be explicit ???
  m_rgb_color.setInputCloud(m_cloud );
  viewer->addPointCloud (m_cloud, m_rgb_color, "cloud");
 
  // Add point picking callback to viewer
  m_clicked_points_3d.reset(new PointCloudT);
  cb_args.m_clicked_points_3d = m_clicked_points_3d;
  cb_args.viewerPtr =  boost::shared_ptr<pcl::visualization::PCLVisualizer> (viewer);
  viewer->registerPointPickingCallback (&PCL_upd_DEMO::pp_callback, *this, (void*)&cb_args);
  viewer->registerMouseCallback(&PCL_upd_DEMO::mouseEventOccurred, *this, (void*)&viewer);
  //std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  pointSizeSliderValueChanged (2);
  viewer->resetCamera ();

  ui->qvtkWidget->update ();
}


void PCL_upd_DEMO::pointSizeSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->lcdNumber_p->display(value);
  ui->qvtkWidget->update ();
}



void PCL_upd_DEMO::updateImagesView()
{
	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	int image_index = ui->listWidget_imageNames->currentRow();      // take the pcd address from the listWidget selected row and read the cloud

	if (!m_file_image_list.at(image_index).isEmpty()) {
		QImage image(m_file_image_list.at(image_index));
         if (image.isNull()) {
             QMessageBox::information(this, tr("Image Viewer"),
                                      tr("Cannot load %1.").arg(m_file_image_list.at(image_index)));
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
         }
	ui->label_image->setPixmap(QPixmap::fromImage(image));
    ui->label_fileStatus_image->setText(m_file_image_list.at(image_index));  // set the status bar to the current pcd
	}

	if (m_file_pcd_list.size () > image_index)
	{
		if (pcl::io::loadPCDFile (m_file_pcd_list.at(image_index).toStdString(), *m_cloud)) 
			QMessageBox::warning(this, "Warning !", "File not found !");

    pcl::transformPointCloud(*m_cloud, *m_cloud, m_transformation);
    ui->label_fileStatus_cloud->setText(m_file_pcd_list.at(image_index));  // set the status bar to the current pcd
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
   if(!viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud")) {
		QMessageBox::warning(this, "Warning !", " Viewer NOT updated! What's up?");
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return; 
       }	
	}   //else QMessageBox::warning(this, "Warning !", " Viewer updated!");
	ui->qvtkWidget->update ();
    pcl::copyPointCloud(*m_cloud, *m_cloud_filtered);   // copy the cloud into the filtered cloud to avoid filtering mistakes
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}



void PCL_upd_DEMO::updatePCDview()
{
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
   int pcd_index = ui->listWidget_pcdNames->currentRow();      // take the pcd address from the listWidget selected row and read the cloud
  
   if (pcl::io::loadPCDFile (m_file_pcd_list.at(pcd_index).toStdString(), *m_cloud)) 
	   QMessageBox::warning(this, "Warning !", "File not found !");

    pcl::transformPointCloud(*m_cloud, *m_cloud, m_transformation);
    ui->label_fileStatus_cloud->setText(m_file_pcd_list.at(pcd_index));  // set the status bar to the current pcd
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
   if(!viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud")){
	   QMessageBox::warning(this, "Warning !", " Viewer NOT updated! What's up?");
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
   }
	//else QMessageBox::warning(this, "Warning !", " Viewer updated!");


	if (m_file_image_list.size () > pcd_index)
	{
		QImage image(m_file_image_list.at(pcd_index));
         if (image.isNull()) {
             QMessageBox::information(this, tr("Image Viewer"),
                                      tr("Cannot load %1.").arg(m_file_image_list.at(pcd_index)));
  		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
         }
	ui->label_image->setPixmap(QPixmap::fromImage(image));
    ui->label_fileStatus_image->setText(m_file_image_list.at(pcd_index));  // set the status bar to the current pcd

	}
   ui->qvtkWidget->update ();
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
//   viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
   ui->qvtkWidget->update ();
    pcl::copyPointCloud(*m_cloud, *m_cloud_filtered);   // copy the cloud into the filtered cloud to avoid filtering mistakes
}



void PCL_upd_DEMO::setSaveFolder()
{
	m_safe_folder = ui->lineEdit_saveFolder->text();

	if(!boost::filesystem::exists(m_safe_folder.toUtf8().constData()))
	{
		if (QMessageBox::information(this, " Message ",
			"the folder " + m_safe_folder +" doesn't exist, do you want to create it ? ",
			" Yes ", "No") == 0)
			{boost::filesystem::create_directories(m_safe_folder.toUtf8().constData());

            if(boost::filesystem::exists(m_safe_folder.toUtf8().constData()))
                QMessageBox::warning(this, "Warning !", "Created !");
			}
		else
		{
			QMessageBox::warning(this, "Warning !", "Not valid folder, using default value");
			m_safe_folder=m_folder_to_list;
			ui->lineEdit_saveFolder->setText(m_safe_folder);
		}
	}

}


void PCL_upd_DEMO::mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* _viewer)
{
    if (!ui->actionRegister_visualizer_to_mouse_events->isChecked()) return;

    //pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (_viewer);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
      std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

      // This code is to add a text in the visualizer, not active now
      //char str[512];
      //sprintf (str, "text");//(str, "text#%03d", text_id ++);
      //std::cout << "mouseEventOccurred DO somethig cool" << std::endl;
      //viewer->addText ("clicked here", event.getX (), event.getY (), str);
    }
    if (event.getButton () == pcl::visualization::MouseEvent::RightButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
      std::cout << "Right mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
    }
    if (event.getButton () == pcl::visualization::MouseEvent::MiddleButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
      std::cout << "Middle mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
    }



}

void PCL_upd_DEMO::pp_callback ( const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1){
      return;}

  if (m_labelling_active == false)  {
  return;
  }

  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->m_clicked_points_3d->clear();
  data->m_clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->m_clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->m_clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

  double patch_size = 0.0;
  bool isNumeric;
  patch_size = ui->lineEdit_patchSize->text().toDouble(&isNumeric);
  if(!isNumeric)
  {
      QMessageBox::warning(this, "Warning !", "Patch size is not a valid number - The patch cannot be extracted !");
      return;
  };
  extractPatch(patch_size, current_point.x, current_point.y, current_point.z);

  // the patch has no points to run the PCA,
  // better to not perform normal analysis
  if (m_cloud_patch->size()<3) {
             QMessageBox::information(this, "info !", " Not enough points,\nextract anothe patch ");
      return;
  }

  // we run the UPD on the patch on click
  runUPDpatch( );

  // K nearest neighbor search
  pcl::KdTreeFLANN<pcl::PointSurfel> kdtree;
  kdtree.setInputCloud (UPD_cloud);
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  pcl::PointCloud<pcl::PointSurfel>::Ptr search_point (new pcl::PointCloud<pcl::PointSurfel>);
  //cout << "PCL_upd_DEMO::labelGround:: MESSAGE m_clicked_points_3d = " << m_clicked_points_3d->points[0] << std::endl;
  search_point->resize(1);
  {
   search_point->points[0].x = m_clicked_points_3d->points[0].x;
   search_point->points[0].y = m_clicked_points_3d->points[0].y;
   search_point->points[0].z = m_clicked_points_3d->points[0].z;
  }

  if ( kdtree.nearestKSearch (search_point->at(0), K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    std::cout << " Found " << pointIdxNKNSearch.size () << " points " <<std::endl;
    // pointIdxNKNSearch[0] is the point we are looking for
  }

  // visualize an arrow in the point
  double norm = Eigen::Vector3d(UPD_cloud->points[ pointIdxNKNSearch[0] ].normal_x,
          UPD_cloud->points[ pointIdxNKNSearch[0] ].normal_y, UPD_cloud->points[ pointIdxNKNSearch[0] ].normal_z).norm();

  pcl::PointXYZ P1 ( UPD_cloud->points[ pointIdxNKNSearch[0] ].x + UPD_cloud->points[ pointIdxNKNSearch[0] ].normal_x/norm,
                     UPD_cloud->points[ pointIdxNKNSearch[0] ].y + UPD_cloud->points[ pointIdxNKNSearch[0] ].normal_y/norm,
                     UPD_cloud->points[ pointIdxNKNSearch[0] ].z + UPD_cloud->points[ pointIdxNKNSearch[0] ].normal_z/norm);
  pcl::PointXYZ P2 ( UPD_cloud->points[ pointIdxNKNSearch[0] ].x,
                     UPD_cloud->points[ pointIdxNKNSearch[0] ].y,
                     UPD_cloud->points[ pointIdxNKNSearch[0] ].z);

  m_patch_labelling_index = pointIdxNKNSearch[0];

  // as we still don't know whether is ground or not we visualize it as grey
  if(viewer->addArrow(P1, P2, 0.2, 0.2, 0.2, false, "arrow", 0)) //the arrow is attached to P1
      ui->qvtkWidget->update ();
  else { viewer->removeShape("arrow",0);
  viewer->addArrow(P1, P2, 0.2, 0.2, 0.2, false, "arrow", 0);
  ui->qvtkWidget->update ();
  }

}

void PCL_upd_DEMO::addRemoveCoordinateSystem()
{
  if (ui->checkBox_coordinateSystem->isChecked()) viewer->addCoordinateSystem ( );
  else viewer->removeCoordinateSystem ( );

  ui->qvtkWidget->update ();
}



PCL_upd_DEMO::~PCL_upd_DEMO ()
{
  delete ui;
}

