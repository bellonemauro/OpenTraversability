/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"

#include <pcl/visualization/common/actor_map.h>

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
  m_cloud_patch.reset (new PointCloudT);
  UPD_cloud.reset( new pcl::PointCloud<pcl::PointSurfel> );
  m_transformation = Eigen::Affine3f::Identity();

  

  _label_counter = 0;
  m_labelling_active = false;
  m_labelled_paused = false;

  //pcl::PCDReader reader;
  //reader.read("./logo.pcd", *cloud);

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
  connect (ui->actionAbout, SIGNAL(triggered()), this, SLOT (about()));


  //buttons
  connect (ui->pushButton_PCDView, SIGNAL(clicked()), this, SLOT (enablePCDview()));
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
  
  
  // sliders
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pointSizeSliderValueChanged (int)));
  connect (ui->horizontalSlider_unevennessIndex, SIGNAL (valueChanged (int)), this, SLOT (unevenessSliderChange (int)));
  connect (ui->horizontalSlider_traversabilityAngle, SIGNAL (valueChanged (int)), this, SLOT (angleSliderChange (int)));


  // others
  connect (ui->lineEdit_saveFolder, SIGNAL(returnPressed()), this, SLOT (setSaveFolder()));
  connect (ui->listWidget_pcdNames, SIGNAL(itemDoubleClicked (QListWidgetItem*)), this, SLOT(updatePCDview()));
  connect (ui->listWidget_imageNames, SIGNAL(itemDoubleClicked (QListWidgetItem*)), this, SLOT(updateImagesView()));
  connect (ui->treeWidget_classification, SIGNAL(itemDoubleClicked (QTreeWidgetItem *,int)), this, SLOT(selectPointLabel())); 
  //connect (ui->checkBox_filpNormals, SIGNAL(clicked()), this, SLOT(setFlip()));
  connect (ui->radioButton_radius, SIGNAL(clicked()), this, SLOT(setRadiusOrKNeighborsMethod()));
  connect (ui->radioButton_kNeighbors, SIGNAL(clicked()), this, SLOT(setRadiusOrKNeighborsMethod()));

  connect (ui->checkBox_visTraversability, SIGNAL (clicked()), this, SLOT(switchVisualization()));

   ui->treeWidget_classification->resizeColumnToContents(0);
   ui->treeWidget_classification->resizeColumnToContents(1);

  viewer->addCoordinateSystem (1.0);
  m_rgb_color.setInputCloud(m_cloud );
  viewer->addPointCloud (m_cloud, m_rgb_color, "cloud");
  //viewer->addPointCloud (m_cloud, "cloud");

  // Add point picking callback to viewer
  m_clicked_points_3d.reset(new PointCloudT);
  cb_args.m_clicked_points_3d = m_clicked_points_3d;
  cb_args.viewerPtr =  boost::shared_ptr<pcl::visualization::PCLVisualizer> (viewer);//pcl::visualization::PCLVisualizer::Ptr(viewer);
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

void PCL_upd_DEMO::enablePCDview()
{
	// first check if the file is empty to prevent user mistake
	if (m_file_pcd_list.empty())
	{
		QMessageBox::warning(this, "Warning !", "File list not loaded");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
	}
	else
	{
	// then extract the file list

/*    if (pcl::io::loadPCDFile (m_file_pdc_list.at(0), *cloud) == -1) QMessageBox::warning(this, "Warning !", "File not found !");
            //cout << "Loaded " << cloud->size () << " data points from " << m_file_pcd_list.at(0) << endl;

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");
    viewer->updatePointCloud (cloud, m_rgb_color, "cloud");
	ui->qvtkWidget->update ();
*/
	QMessageBox::information(this, " Message ", "TODO MB: play with pcds ",
		" OK ", m_file_pcd_list.at(0));
	}


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
		if (pcl::io::loadPCDFile (m_file_pcd_list.at(image_index).toStdString(), *m_cloud)) QMessageBox::warning(this, "Warning !", "File not found !");
//TODO - the status bar doesn't properly work
    pcl::transformPointCloud(*m_cloud, *m_cloud, m_transformation);
    ui->label_fileStatus_cloud->setText(m_file_pcd_list.at(image_index));  // set the status bar to the current pcd
    if(!viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud"))QMessageBox::warning(this, "Warning !", " Viewer NOT updated! What's up?");
	//else QMessageBox::warning(this, "Warning !", " Viewer updated!");
		}
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}



void PCL_upd_DEMO::updatePCDview()
{
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
   int pcd_index = ui->listWidget_pcdNames->currentRow();      // take the pcd address from the listWidget selected row and read the cloud
  
   if (pcl::io::loadPCDFile (m_file_pcd_list.at(pcd_index).toStdString(), *m_cloud)) QMessageBox::warning(this, "Warning !", "File not found !");
//TODO - the status bar doesn't properly work
    pcl::transformPointCloud(*m_cloud, *m_cloud, m_transformation);
    ui->label_fileStatus_cloud->setText(m_file_pcd_list.at(pcd_index));  // set the status bar to the current pcd
    if(!viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud"))QMessageBox::warning(this, "Warning !", " Viewer NOT updated! What's up?");
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

	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
   viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
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

  extractPatch(0.2, current_point.x, current_point.y, current_point.z);

  std::cout << " estracted a patch with size : " << m_cloud_patch->size() << std::endl;

}





void PCL_upd_DEMO::extractPatch(double _size, float _x, float _y, float _z)
{
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

    m_cloud_patch->clear();  // clean the cloud in case of old patches in memory

    pcl::copyPointCloud(*m_cloud, *m_cloud_patch);

    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    double z_min = _z - _size;
    double z_max = _z + _size;
    pass.setInputCloud (m_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*m_cloud_patch);

    double y_min = _y - _size;
    double y_max = _y + _size;
    pass.setInputCloud (m_cloud_patch);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*m_cloud_patch);

    double x_min = _x - _size;
    double x_max = _x + _size;
    pass.setInputCloud (m_cloud_patch);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*m_cloud_patch);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb_color(m_cloud_patch, 255, 0, 255);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "m_cloud_patch");   //5 is for bigger size - in this way the point is enphasized
    /// this is just a trick to have the visualization and the update but we might get a lot of errors in the cml -- TODO fix this
    if (!viewer->addPointCloud(m_cloud_patch, rgb_color, "m_cloud_patch",0))
        viewer->updatePointCloud(m_cloud_patch, rgb_color, "m_cloud_patch");

    /*   // here we look for the same id in the actor map to avoid the issue above but it does not work properly yet
    pcl::visualization::ShapeActorMapPtr actorMap = viewer->getShapeActorMap();
    pcl::visualization::CloudActorMap::iterator am_it = actorMap->find ("m_cloud_patch");
    if (am_it != actorMap->end ()) {
        viewer->addPointCloud(m_cloud_patch, rgb_color, "m_cloud_patch",0);
    }
    else  {
        viewer->updatePointCloud(m_cloud_patch, rgb_color, "m_cloud_patch");
    }*/

    ui->qvtkWidget->update ();
    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
}





void PCL_upd_DEMO::startStopLabelling()
{
	if (ui->horizontalSlider_p->value() != 1 )    //set the visualization to the minimum point size if not already done --- only for visualization
	{ 
		ui->horizontalSlider_p->setValue(1);
        pointSizeSliderValueChanged(1);
	}

    if (m_labelled_paused == true)
	{
	ui->pushButton_start_stopLabelling->setEnabled(false);
    ui->pushButton_start_stopLabelling->setText("Start/Stop Labelling");
    m_labelled_paused = false;
	}

    m_labelling_active = true;


    QMessageBox::information(this, "info !", " Labelling procedure starting,\npress shift and click to select a patch");


    if (_label_counter == 0 && m_labelled_paused == false)     // the first step --- deactivate button for manual labelling - activate ground and not ground  ---- then set the visualization properties
	{ 
		//QMessageBox::information(this, "info !", " Manual labelling procedure starting !");

		ui->pushButton_start_stopLabelling->setEnabled(false);
		ui->pushButton_ground->setEnabled(true);
		ui->pushButton_notGround->setEnabled(true);

		m_labeled_point->clear();
		m_labeled_point->push_back( m_cloud->at(_label_counter));
		m_labeled_point->at(0).rgba = 0x00FF00FF;
        pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb_color(m_labeled_point, 255, 0, 255);
        viewer->updatePointCloud (m_labeled_point, rgb_color, "label_point");

        viewer->addPointCloud(m_labeled_point, rgb_color, "label_point",0);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "label_point");   //5 is for bigger size - in this way the point is enphasized
		_label_counter++;
		return;
	}


    if (_label_counter < m_cloud->size() && m_labelled_paused == false)
	{   
		m_labeled_point->clear();
		m_labeled_point->push_back( m_cloud->at(_label_counter));
		m_labeled_point->at(0).rgba = 0x00FF00FF;

		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "label_point");   //5 is for bigger size - in this way the point is enphasized
        pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb_color(m_labeled_point, 255, 0, 255);
        viewer->updatePointCloud (m_labeled_point, rgb_color, "label_point");
        viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
		ui->qvtkWidget->update ();
		_label_counter++;
		return;
	}

}


void PCL_upd_DEMO::labelGround()
{
	 if (_label_counter < m_cloud->size())	
	 { 
		 m_labeled_cloud->push_back(m_labeled_point->at(0));
 		 m_labeled_cloud->at(m_labeled_cloud->size()-1).rgba = 0x0000FF00;
 		 m_cloud->at(_label_counter-1).rgba = 0x0000FF00;
//		 cout << "PCL_upd_DEMO::labelGround:: MESSAGE --- ground --- x " << m_labeled_point->at(0).x << " y " << m_labeled_point->at(0).y << " z " << m_labeled_point->at(0).z << endl;
		
		 stringstream ss;
		 ss << " x " << m_labeled_point->at(0).x << ", y = " << m_labeled_point->at(0).y << ", z =" << m_labeled_point->at(0).z;
		 QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
		 item->setText(1,QString::fromStdString(ss.str()));
		 item->setText(0,"Ground");
		 ui->treeWidget_classification->insertTopLevelItem(0,item);
		 
		 startStopLabelling();
	 }
	 else //this is the last step!
	 {
		m_labeled_cloud->push_back(m_labeled_point->at(0));
		m_labeled_cloud->at(m_labeled_cloud->size()-1).rgba = 0x0000FF00;
		m_cloud->at(_label_counter-1).rgba = 0x0000FF00;
	 	ui->pushButton_start_stopLabelling->setEnabled(true);
		ui->pushButton_ground->setEnabled(false);
		ui->pushButton_notGround->setEnabled(false);
        m_labelling_active = false;
        QMessageBox::information(this, "info !", " Congratulation you labeled all cloud !!");
     }

}

void PCL_upd_DEMO::labelNotGround()
{
	 if (_label_counter < m_cloud->size())	
	 { 
		 
		 m_labeled_cloud->push_back(m_labeled_point->at(0));
 		 m_labeled_cloud->at(m_labeled_cloud->size()-1).rgba = 0x00FF0000;
		 m_cloud->at(_label_counter-1).rgba = 0x00FF0000;
//		 cout << "PCL_upd_DEMO::labelGround:: MESSAGE --- not ground --- x " << m_labeled_point->at(0).x << " y " << m_labeled_point->at(0).y << " z " << m_labeled_point->at(0).z << endl;

		 stringstream ss;
		 ss << " x " << m_labeled_point->at(0).x << ", y = " << m_labeled_point->at(0).y << ", z =" << m_labeled_point->at(0).z;
		 QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
		 item->setText(1,QString::fromStdString(ss.str()));
		 item->setText(0,"NOT Ground");
		 ui->treeWidget_classification->insertTopLevelItem(0,item);
		 
		 startStopLabelling(); 

	 }
	 else //this is the last step!
	 {		
		m_labeled_cloud->push_back(m_labeled_point->at(0));
 		m_labeled_cloud->at(m_labeled_cloud->size()-1).rgba = 0x00FF0000;
		m_cloud->at(_label_counter-1).rgba = 0x00FF0000;
	 	ui->pushButton_start_stopLabelling->setEnabled(true);
		ui->pushButton_ground->setEnabled(false);
		ui->pushButton_notGround->setEnabled(false);
        m_labelling_active = false;
		QMessageBox::information(this, "info !", " Congratulation you labeled all cloud !!");
	 }
}

void PCL_upd_DEMO::selectPointLabel()
{
	ui->pushButton_start_stopLabelling->setEnabled(true);
	ui->pushButton_start_stopLabelling->setText("Continue ...");

    m_labelled_paused = true;

	int _index = ui->treeWidget_classification->indexOfTopLevelItem(ui->treeWidget_classification->currentItem());      // takes the index from the listWidget selected row and read the cloud
	m_labeled_point->push_back( m_cloud->at(_index));

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "label_point");   //5 is for bigger size - in this way the point is enphasized
	viewer->updatePointCloud (m_labeled_point, "label_point");
    viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
	ui->qvtkWidget->update ();
}


void PCL_upd_DEMO::clearLabelling()
{
	m_labeled_point->clear();
	m_labeled_cloud->clear();
    m_labelling_active = false;
	ui->treeWidget_classification->clear();
	 	ui->pushButton_start_stopLabelling->setEnabled(true);
		ui->pushButton_ground->setEnabled(false);
		ui->pushButton_notGround->setEnabled(false);
		_label_counter=0;
		QMessageBox::information(this, "info !", " Cleared !!");

}


PCL_upd_DEMO::~PCL_upd_DEMO ()
{
  delete ui;
}

