/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"


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

        viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
		ui->qvtkWidget->update ();
	}
	else
	{
		m_upd->setColorMapType(false);
		m_upd->getAsColorMap(   m_cloud_color_UPD,
								ui->lcdNumber_unevenness->value()/ui->horizontalSlider_unevennessIndex->maximum(),
								pcl::deg2rad(ui->lcdNumber_thresholdAngle->value()));
		viewer->removePointCloud();
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_color_UPD);
        viewer->addPointCloud (m_cloud_color_UPD, rgb_color, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");	

/* TRY for the orientation analysis of tractor and trailer
        viewer->updatePointCloud (m_cloud_color_UPD, m_rgb_color, "cloud");
		pcl::PointCloud<pcl::Normal>::Ptr normali_ptr (new pcl::PointCloud<pcl::Normal>);
		normali_ptr = m_upd->getNormals();
		if (normali_ptr->size()>1)
		{viewer->removePointCloud("normals");
		viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(m_cloud_color_UPD, normali_ptr, 25, 0.5, "normals");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "normals");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 50, "normals");	

		 

		std::cout << "point index : " << (normali_ptr->width >> 1) * (normali_ptr->height + 1) << std::endl;
		size_t normalIndex = 50;
		std::cout << "Value x :" << normali_ptr->points [normalIndex].normal_x << " _ " <<  std::endl;
		std::cout << "Value y :" << normali_ptr->points [normalIndex].normal_y << " _ " <<  std::endl;
		std::cout << "Value z :" << normali_ptr->points [normalIndex].normal_z << " _ " <<  std::endl;

		std::cout << "Orientation x :" << acos(normali_ptr->points [normalIndex].normal_x)*360.0/M_PI << " deg " <<  std::endl;
		std::cout << "Orientation y :" << acos(normali_ptr->points [normalIndex].normal_y)*360.0/M_PI << " deg " <<  std::endl;
		std::cout << "Orientation z :" << acos(normali_ptr->points [normalIndex].normal_z)*360.0/M_PI << " deg " <<  std::endl;

		viewer->addCoordinateSystem(0.2, m_cloud_color_UPD->points [normalIndex].x, m_cloud_color_UPD->points [normalIndex].y, m_cloud_color_UPD->points [normalIndex].z);

		}
*/
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
