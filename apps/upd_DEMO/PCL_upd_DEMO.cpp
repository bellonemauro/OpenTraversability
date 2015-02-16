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
  UPD_cloud.reset( new pcl::PointCloud<pcl::PointSurfel> );
  m_transformation = Eigen::Affine3f::Identity();

  

  _label_counter = 0;
  _labelled_paused = false;

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
  connect (ui->actionOpen_list, SIGNAL(activated()), this, SLOT(openFileList()));
  connect (ui->actionOpen_cloud, SIGNAL(activated()), this, SLOT(openFile()));
  connect (ui->actionOpen_Image, SIGNAL(activated()), this, SLOT(openImage()));
  connect (ui->actionOpen_Images_List, SIGNAL(activated()), this, SLOT(openImageList()));
  connect (ui->actionOpen_PCDFolder, SIGNAL(activated()), this, SLOT(openPCDFolder()));
  connect (ui->actionImages_Folder, SIGNAL(activated()), this, SLOT(openImagesFolder()));
  connect (ui->actionSave_Cloud, SIGNAL(activated()), this, SLOT(saveFile()));
  connect (ui->actionSave_UPD, SIGNAL(activated()), this, SLOT(saveUPDFile()));
  connect (ui->actionLabelled_Cloud, SIGNAL(activated()), this, SLOT(saveLabeledFile()));
  connect (ui->actionAbout, SIGNAL(activated()), this, SLOT (about()));
  connect (ui->actionRemove_Filters, SIGNAL(activated()), this, SLOT(removeFilters()) );
  connect (ui->actionGenerate_sample_cloud, SIGNAL(activated()), this, SLOT(GenerateSampleCloud()));


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
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));
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
  viewer->addPointCloud (m_cloud, "cloud");

  // Add point picking callback to viewer:


  m_clicked_points_3d.reset(new PointCloudT);
  //cb_args.m_clicked_points_3d = m_clicked_points_3d;
  //cb_args.viewerPtr =  boost::shared_ptr<pcl::visualization::PCLVisualizer> (viewer);//pcl::visualization::PCLVisualizer::Ptr(viewer);
  //viewer->registerPointPickingCallback (PCL_upd_DEMO::pp_callback, (void*)&cb_args);
  //std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  pSliderValueChanged (2);
  viewer->resetCamera ();

  ui->qvtkWidget->update ();
}


 void PCL_upd_DEMO::openImage()
 {
     QString fileName = QFileDialog::getOpenFileName(this, tr("Open Image "), QDir::currentPath(),	
													"Image file (*.jpg);; All Files(*.*)" , 0);
     if (!fileName.isEmpty()) {
         QImage image(fileName);
         if (image.isNull()) {
             QMessageBox::information(this, tr("Image Viewer"),
                                      tr("Cannot load %1.").arg(fileName));
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
         }
		 ui->label_image->setPixmap(QPixmap::fromImage(image));
         //scaleFactor = 1.0;

         //printAct->setEnabled(true);
         //fitToWindowAct->setEnabled(true);
         //updateActions();

         //if (!fitToWindowAct->isChecked())
             ui->label_image->adjustSize();
     }
 }
 
void PCL_upd_DEMO::openImageList()
{
		//QFileDialog::getOpenFileName (parent, dialog title, default folder, allowed formats)
	  m_path_to_image_list = QFileDialog::getOpenFileName (	this, tr("Open File list"), QDir::currentPath(),  // dialog to open files
						"Text file (*.txt);; All Files(*.*)" , 0);
	  m_folder_to_list = m_path_to_image_list.section("/",0,-2);    //--> extract the path
	  //QMessageBox::information(this, "Properly opened folder : ", m_folder_to_list);
	  //QMessageBox::information(this, "Properly opened file : ", m_path_to_pcd_list);

        //open the file
		std::ifstream list;
        list.open(m_path_to_image_list.toUtf8().constData());
        m_file_image_list.clear();   // clean the list in case of multiple open actions
        ui->listWidget_imageNames->clear();
		
		//load list of file names into a vector
		do
		{
		std::string f;
		list >> f;
		std::string ff = m_folder_to_list.toUtf8().constData();
		ff.append("/");
		ff.append(f);
		m_file_image_list.push_back(QString::fromStdString(ff));
        ui->listWidget_imageNames->addItem(QString::fromStdString(ff));
		}
		while(!list.eof());    //while list has elements

		list.close();    // close the file safely

		//set the save folder to the default value of the source data directory
		m_safe_folder = m_folder_to_list;
		ui->lineEdit_saveFolder->setText(m_safe_folder);

}


void
PCL_upd_DEMO::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->lcdNumber_p->display(value);
  ui->qvtkWidget->update ();
}

void
PCL_upd_DEMO::enablePCDview()
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
	viewer->updatePointCloud (cloud, "cloud");
	ui->qvtkWidget->update ();
*/
	QMessageBox::information(this, " Message ", "TODO MB: play with pcds ",
		" OK ", m_file_pcd_list.at(0));
	}


}

void PCL_upd_DEMO::openPCDFolder ()
{
		QDir m_dir = QFileDialog::getExistingDirectory(this, tr("Open folder"), QDir::currentPath(), 0);
		QStringList nameFilter;
		nameFilter << "*.pcd" << "*.bin";
		m_file_pcd_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );
		ui->listWidget_pcdNames->clear();
		ui->listWidget_pcdNames->addItems(m_file_pcd_list);
}

void PCL_upd_DEMO::openImagesFolder ()
{
		QDir m_dir = QFileDialog::getExistingDirectory(this, tr("Open folder"), QDir::currentPath(), 0);
		QStringList nameFilter;
		nameFilter << "*.jpg" << "*.pgn" << "*.pgm" << "*.bmp";
		m_file_image_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );
		ui->listWidget_imageNames->clear();
		ui->listWidget_imageNames->addItems(m_file_image_list);
}

void
PCL_upd_DEMO::openFileList ( )
{
	//QFileDialog::getOpenFileName (parent, dialog title, default folder, allowed formats)
	  m_path_to_pcd_list = QFileDialog::getOpenFileName (	this, tr("Open File list"),   // dialog to open files
					QDir::currentPath(),	"Text files (*.txt);; All Files(*.*)" , 0);
	  m_folder_to_list = m_path_to_pcd_list.section("/",0,-2);    //--> extract the path
	  //QMessageBox::information(this, "Properly opened folder : ", m_folder_to_list);
	  //QMessageBox::information(this, "Properly opened file : ", m_path_to_pcd_list);



        //open the file
		std::ifstream list;
        list.open(m_path_to_pcd_list.toUtf8().constData());
        m_file_pcd_list.clear();   // clean the list in case of multiple open actions
        ui->listWidget_pcdNames->clear();
		//load list of file names into a vector
		do
		{
		std::string f;
		list >> f;
		std::string ff = m_folder_to_list.toUtf8().constData();
		ff.append("/");
		ff.append(f);
		m_file_pcd_list.push_back(QString::fromStdString(ff));
        ui->listWidget_pcdNames->addItem(QString::fromStdString(ff));
		}
		while(!list.eof());    //while list has elements

		list.close();    // close the file safely

		//set the save folder to the default value of the source data directory
		m_safe_folder = m_folder_to_list;
		ui->lineEdit_saveFolder->setText(m_safe_folder);

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
    if(!viewer->updatePointCloud (m_cloud, "cloud"))QMessageBox::warning(this, "Warning !", " Viewer NOT updated! What's up?");
	//else QMessageBox::warning(this, "Warning !", " Viewer updated!");
		}
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void PCL_upd_DEMO::openFile()
{

	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	QString cloud_path = QFileDialog::getOpenFileName (this, tr("Open cloud"), QDir::currentPath(),  // dialog to open files
						"ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);

	if (pcl::io::loadPCDFile (cloud_path.toUtf8().constData(), *m_cloud) == -1) QMessageBox::warning(this, "Warning !", "File not found ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pcd_list.at(0) << endl;

   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

   ui->label_fileStatus_cloud->setText(QString::fromStdString(cloud_path.toUtf8().constData()));  // set the status bar to the current pcd
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");
   viewer->updatePointCloud (m_cloud, "cloud");
		ui->qvtkWidget->update ();
}

void 
PCL_upd_DEMO::saveFile()
{

	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	QString cloud_path = QFileDialog::getSaveFileName (this, tr("Save ASCII cloud"), QDir::currentPath(),  // dialog to open files
						"ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);

	if (pcl::io::savePCDFileASCII (cloud_path.toUtf8().constData(), *m_cloud) == -1) QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pdc_list.at(0) << endl;

   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
	
}

void 
PCL_upd_DEMO::saveUPDFile()
{

	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	/*if (UPD_cloud->isEmpty()) 
	{QMessageBox::warning(this, "Warning !", "UPD cloud empty not saved  ! " );
	}*/
	QString cloud_path = QFileDialog::getSaveFileName (this, tr("Save ASCII cloud"), QDir::currentPath(),  // dialog to open files
						"ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);

	if (pcl::io::savePCDFileASCII (cloud_path.toUtf8().constData(), *UPD_cloud) == -1) QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pcd_list.at(0) << endl;

   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void 
PCL_upd_DEMO::saveLabeledFile()
{

	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	/*if (UPD_cloud->isEmpty()) 
	{QMessageBox::warning(this, "Warning !", "UPD cloud empty not saved  ! " );
	}*/
	QString cloud_path = QFileDialog::getSaveFileName (this, tr("Save ASCII cloud"), QDir::currentPath(),  // dialog to open files
						"ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);

	if (pcl::io::savePCDFileASCII (cloud_path.toUtf8().constData(), *m_labeled_cloud) == -1) QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pcd_list.at(0) << endl;

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
    if(!viewer->updatePointCloud (m_cloud, "cloud"))QMessageBox::warning(this, "Warning !", " Viewer NOT updated! What's up?");
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
   viewer->updatePointCloud (m_cloud, "cloud");
   ui->qvtkWidget->update ();
    pcl::copyPointCloud(*m_cloud, *m_cloud_filtered);   // copy the cloud into the filtered cloud to avoid filtering mistakes
}

void PCL_upd_DEMO::applyPassthrogh()
{
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
   
   if(m_cloud_filtered->empty())removeFilters();

    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    double z_min = 0;
    double z_max = 0;
    bool isNumeric;
    z_min = ui->lineEdit_z_min_pfilter->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "z min is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    };

    z_max = ui->lineEdit_z_max_pfilter->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "z max is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    };
    pass.setInputCloud (m_cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*m_cloud_filtered);

    double y_min = 0;
    double y_max = 0;
    y_min = ui->lineEdit_y_min_pfilter->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "y min is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    };

    y_max = ui->lineEdit_y_max_pfilter->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "y max is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }
    pass.setInputCloud (m_cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*m_cloud_filtered);

    double x_min = 0;
    double x_max = 0;
    x_min = ui->lineEdit_x_min_pfilter->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "x min is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }

    x_max = ui->lineEdit_x_max_pfilter->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "x max is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }
    pass.setInputCloud (m_cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*m_cloud_filtered);

//    QMessageBox::warning(this, "Warning !", "Only " + QString::number(m_cloud_filtered->size()) + " points remained ");

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
    viewer->updatePointCloud (m_cloud_filtered, "cloud");
    ui->qvtkWidget->update ();
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
}

void PCL_upd_DEMO::applyVoxelization()
{
	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
   //QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

	if(m_cloud_filtered->empty())removeFilters();

    pcl::VoxelGrid<pcl::PointXYZRGBA> ds;  //create downsampling filter
    ds.setInputCloud (m_cloud_filtered);
    double leaf_size = 0;
    bool isNumeric;
    leaf_size = ui->lineEdit_leafSize->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "Leaf size is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }

	int minPointsPerVoxel;
    minPointsPerVoxel = ui->lineEdit_minPointsPerVoxel->text().toInt(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "Number of points per voxel is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }

    ds.setLeafSize (leaf_size, leaf_size, leaf_size);
	//ds.setMinimumPointsNumberPerVoxel (minPointsPerVoxel);
    ds.filter (*m_cloud_filtered);
   
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
    viewer->updatePointCloud (m_cloud_filtered, "cloud");
    ui->qvtkWidget->update ();

}

void PCL_upd_DEMO::applySOR()
{
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
   //QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
   if(m_cloud_filtered->empty())removeFilters();

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (m_cloud_filtered);

    double sor_meanK = 0;
    double sor_std = 0;
    bool isNumeric;
    sor_meanK = ui->lineEdit_outlierMean->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "SOR mean is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }

    sor_std = ui->lineEdit_outlierStd->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "SOR standard deviation is not a valid number - Filter cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
    }
    sor.setMeanK (sor_meanK);
    sor.setStddevMulThresh (sor_std);
    sor.filter (*m_cloud_filtered);

    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
    viewer->updatePointCloud (m_cloud_filtered, "cloud");
    ui->qvtkWidget->update ();
}

void PCL_upd_DEMO::removeFilters()
{
    pcl::copyPointCloud(*m_cloud, *m_cloud_filtered);
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
    viewer->updatePointCloud (m_cloud, "cloud");
    ui->qvtkWidget->update ();

}


void PCL_upd_DEMO::applyTransformation()
{
	   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

double roll = 0;
bool isNumeric;
roll = ui->lineEdit_roll->text().toDouble(&isNumeric);
if(!isNumeric)
{
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "Roll is not a valid number - the transformation cannot be applied !");
		return;
}
double pitch = 0;
pitch = ui->lineEdit_pitch->text().toDouble(&isNumeric);
if(!isNumeric)
{
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
   QMessageBox::warning(this, "Warning !", "pitch is not a valid number - the transformation cannot be applied !");
 		return;
}
double yaw = 0;
yaw = ui->lineEdit_yaw->text().toDouble(&isNumeric);
if(!isNumeric)
{
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "yaw is not a valid number - the transformation cannot be applied !");
		return;
}
double xT = 0;
xT = ui->lineEdit_xT->text().toDouble(&isNumeric);
if(!isNumeric)
{
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "x is not a valid number - the transformation cannot be applied !");
		return;
}
double yT = 0;
yT = ui->lineEdit_yT->text().toDouble(&isNumeric);
if(!isNumeric)
{
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "y is not a valid number - the transformation cannot be applied !");
		return;
}
double zT = 0;
zT = ui->lineEdit_zT->text().toDouble(&isNumeric);
if(!isNumeric)
{
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "z is not a valid number - the transformation cannot be applied !");
		return;
}

if (!ui->checkBox_applyTransformation->isChecked())
{
Eigen::Affine3f axis_transformation = pcl::getTransformation (xT, yT, zT, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw));
pcl::transformPointCloud(*m_cloud, *m_cloud, axis_transformation);
}
else
{
    m_transformation = pcl::getTransformation (xT, yT, zT, pcl::deg2rad(roll), pcl::deg2rad(pitch), pcl::deg2rad(yaw));
		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::information(this, "Information !", " the transformation matrix will be applied to all loaded points cloud !");
}

   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
viewer->updatePointCloud (m_cloud, "cloud");
ui->qvtkWidget->update ();
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

void  PCL_upd_DEMO::GenerateSampleCloud()
{
	   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr surface (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point (new pcl::PointCloud<pcl::PointXYZRGBA>);
point->resize(1*1);
for (float x= -0.5f; x<=0.5f; x+=0.1f)
{
  for (float y= -0.5f; y<=0.5f; y+=0.1f)
  {

    point->points[0].x = x;
    point->points[0].y = y;
    point->points[0].z = 0;// 2.0f - y;
    point->points[0].rgba = 0x00000000;

    *surface+=*point;
  }
}
for (float x=-0.5f; x<=0.5f; x+=0.1f)
{
  for (float z=-0.5f; z<=0.5f; z+=0.1f)
  {

    point->points[0].x = x;
    point->points[0].y = 0.51f;
    point->points[0].z = 0.5f+z;// 2.0f - y;
    //point->points[0].rgba = 0x00000000;
	point->points[0].r = red;
	point->points[0].g = green;
	point->points[0].b = blue;

    *surface+=*point;
  }
}
for (float y=-0.5f; y<=0.5f; y+=0.05f)
{
  for (float z=-0.5f; z<=0.5f; z+=0.05f)
  {

    point->points[0].x = 0.5f;
    point->points[0].y = y;
    point->points[0].z = 0.5f+z;// 2.0f - y;
    //point->points[0].rgba = 0x00000000;
	point->points[0].r = red;
	point->points[0].g = green;
	point->points[0].b = blue;

    *surface+=*point;
  }
}
 QApplication::restoreOverrideCursor();    //transform the cursor for waiting mode

	QMessageBox::warning(this, "Warning !", "Surface created");
	pcl::copyPointCloud(*surface,*m_cloud);
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
	viewer->updatePointCloud (m_cloud, "cloud");
	ui->qvtkWidget->update ();
}

  void PCL_upd_DEMO::setRadiusOrKNeighborsMethod()
  {
	if (ui->radioButton_kNeighbors->isChecked())
	{
		ui->radioButton_radius->setChecked(false);
	}
	else
	{	ui->radioButton_kNeighbors->setChecked(false);
	}



  }


void PCL_upd_DEMO::runUPD()
  {
  m_upd = new upd;
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
   //QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

	if (ui->checkBox_applyTrans->isChecked())
	{
		ui->checkBox_applyTransformation->setChecked(true);
		applyTransformation();
	}

	pcl::copyPointCloud(*m_cloud, *m_cloud_filtered);
	removeFilters();
	
	if (ui->checkBox_applyPTF->isChecked() )applyPassthrogh();

	if (ui->checkBox_applySOR->isChecked()) applySOR();	

	if (ui->checkBox_applyVox->isChecked()) applyVoxelization();

	m_upd->setInputCloud(m_cloud_filtered);
	m_upd->setFlip(ui->checkBox_filpNormals->isChecked());

	if (ui->radioButton_kNeighbors->isChecked())
	{
	int k_neighbors = 0;
	bool isNumeric;
	k_neighbors = ui->lineEdit_searchRadius->text().toDouble(&isNumeric);
	if(!isNumeric)
	{
		QMessageBox::warning(this, "Warning !", "K must be an integer number - the UPD cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
	}
	m_upd->setSearchRadius(k_neighbors);
	m_upd->runUPD_kSearch();
	}
	else
	{
	double search_radius = 0.5;
	bool isNumeric;
	search_radius = ui->lineEdit_searchRadius->text().toDouble(&isNumeric);
	if(!isNumeric)
	{
		QMessageBox::warning(this, "Warning !", "Search radius is not a valid number - the UPD cannot be applied !");
 		QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
		return;
	}
	m_upd->setSearchRadius(search_radius);
	m_upd->runUPD_radius();
	}


	UPD_cloud = m_upd->getUPD( );

 QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode  

 QMessageBox::warning(this, "Warning !", "upd properly generated with size " + QString::number( UPD_cloud->size() ) );

  }

void PCL_upd_DEMO::switchVisualization()
{

  	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	
	if(!ui->checkBox_visTraversability->isChecked()) 
	{    
		viewer->updatePointCloud (m_cloud, "cloud");
		ui->qvtkWidget->update ();
	}
	else
	{
		m_upd->getAsColorMap(   m_cloud_color_UPD,
								ui->lcdNumber_unevenness->value()/ui->horizontalSlider_unevennessIndex->maximum(),
								pcl::deg2rad(ui->lcdNumber_thresholdAngle->value()));
		viewer->removePointCloud();
		viewer->addPointCloud (m_cloud_color_UPD, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	
		viewer->updatePointCloud (m_cloud_color_UPD, "cloud");
		ui->qvtkWidget->update ();
	}
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

//TODO: this is a try to build a colormap with vtk instead of upd function 
/*	vtkColorTransferFunction *color_map = vtkColorTransferFunction::New();
	color_map->SetNanColor(0x000000);  //set NaN to black
	color_map->SetColorSpaceToRGB();

	 vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(0.0, 1.0);
	colorLookupTable->SetHueRange(0.0,0.667);
	colorLookupTable->SetNumberOfColors(16);
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	//color_map->SetBelowRangeColor(0x000000);
	//color_map->SetAboveRangeColor(0xFFFFFF);*/
}


void   PCL_upd_DEMO::unevenessSliderChange(int value)
  {
	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

	double _value = value/10000;
	ui->lcdNumber_unevenness->display(value);
	//TODO - fix this
	//ui->lcdNumber_unevenness->display(_value);  // 100 to bring the slider integer between 0-100 to 0-1 as unevenness index value
	
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
  }


void PCL_upd_DEMO::angleSliderChange(int value)
  {
  	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	
	ui->lcdNumber_thresholdAngle->display(value);

	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
  }



void
PCL_upd_DEMO::pp_callback ( pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->m_clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->m_clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->m_clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}


void PCL_upd_DEMO::startStopLabelling()
{
	if (ui->horizontalSlider_p->value() != 1 )    //set the visualization to the minimum point size if not already done --- only for visualization
	{ 
		ui->horizontalSlider_p->setValue(1);
		pSliderValueChanged(1);
	}

	if (_labelled_paused == true)
	{
	ui->pushButton_start_stopLabelling->setEnabled(false);
	ui->pushButton_start_stopLabelling->setText("Start/Stop Labelling");	
	_labelled_paused = false;
	}

	if (_label_counter == 0 && _labelled_paused == false)     // the first step --- deactivate button for manual labelling - activate ground and not ground  ---- then set the visualization properties
	{ 
		//QMessageBox::information(this, "info !", " Manual labelling procedure starting !");

		ui->pushButton_start_stopLabelling->setEnabled(false);
		ui->pushButton_ground->setEnabled(true);
		ui->pushButton_notGround->setEnabled(true);

		m_labeled_point->clear();
		m_labeled_point->push_back( m_cloud->at(_label_counter));
		m_labeled_point->at(0).rgba = 0x00FF00FF;

		viewer->addPointCloud(m_labeled_point, "label_point",0);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "label_point");   //5 is for bigger size - in this way the point is enphasized
		_label_counter++;
		return;
	}


	if (_label_counter < m_cloud->size() && _labelled_paused == false)
	{   
		m_labeled_point->clear();
		m_labeled_point->push_back( m_cloud->at(_label_counter));
		m_labeled_point->at(0).rgba = 0x00FF00FF;

		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "label_point");   //5 is for bigger size - in this way the point is enphasized
		viewer->updatePointCloud (m_labeled_point, "label_point");
		viewer->updatePointCloud (m_cloud, "cloud");
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
		QMessageBox::information(this, "info !", " Congratulation you labeled all cloud !!");
	 }
}

void PCL_upd_DEMO::selectPointLabel()
{
	ui->pushButton_start_stopLabelling->setEnabled(true);
	ui->pushButton_start_stopLabelling->setText("Continue ...");

	_labelled_paused = true;


	int _index = ui->treeWidget_classification->indexOfTopLevelItem(ui->treeWidget_classification->currentItem());      // takes the index from the listWidget selected row and read the cloud
	m_labeled_point->push_back( m_cloud->at(_index));

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "label_point");   //5 is for bigger size - in this way the point is enphasized
	viewer->updatePointCloud (m_labeled_point, "label_point");
	viewer->updatePointCloud (m_cloud, "cloud");
	ui->qvtkWidget->update ();





}




void PCL_upd_DEMO::clearLabelling()
{

	m_labeled_point->clear();
	m_labeled_cloud->clear();
	ui->treeWidget_classification->clear();
	 	ui->pushButton_start_stopLabelling->setEnabled(true);
		ui->pushButton_ground->setEnabled(false);
		ui->pushButton_notGround->setEnabled(false);
		_label_counter=0;
		QMessageBox::information(this, "info !", " Cleared !!");

}



void PCL_upd_DEMO::about()
{
	 QMessageBox messageBox;
	 messageBox.about(0,"About MB"," https://sites.google.com/site/bellonemauro ");
	 messageBox.icon();
	 messageBox.setFixedSize(500,700);
}


void PCL_upd_DEMO::m_try()
{
//empty function just to try commands
	QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
	
	QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
}

PCL_upd_DEMO::~PCL_upd_DEMO ()
{
  delete ui;
}

