/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"




void PCL_upd_DEMO::startStopLabelling()
{
	if (ui->horizontalSlider_p->value() != 1 )    //set the visualization to the minimum point size if not already done --- only for visualization
	{ 
		ui->horizontalSlider_p->setValue(1);
        pointSizeSliderValueChanged(1);
	}

    if (UPD_cloud->size() == 0)    {
        QMessageBox::information(this, "info !", " Run UPD before performing a cloud labelling\nthe process needs a feature !!");
        return;
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
        ui->pushButton_ground->setStyleSheet("color: black;"
                                             "background-color: lightgreen;"
                                             "selection-color: red;");
        ui->pushButton_notGround->setStyleSheet("color: black;"
                                             "background-color: red;"
                                             "selection-color: red;");


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
m_classifiable_cloud->clear();
    copyPatchToClassifiable(*m_cloud_patch, *m_classifiable_cloud, 1.0);

std::cout << " classifiable cloud size = "<< m_classifiable_cloud->size() << std::endl;

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
    m_classifiable_cloud->clear();
        copyPatchToClassifiable(*m_cloud_patch, *m_classifiable_cloud, -1.0);

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

void PCL_upd_DEMO::copyPatchToClassifiable(const pcl::PointCloud<pcl::PointSurfel> &_cloud_in,
                                            pcl::PointCloud<PointClassifiable> &_cloud_out ,
                                           float _label)
{

    if (_cloud_in.size() < 1 )
    {
        PCL_ERROR ("Input cloud size too small");
                return;
    }
        PCL_ERROR ("classifying ");
    pcl::PointCloud<PointClassifiable> classifiable_point;
    for (size_t i=0; i<_cloud_in.size(); i++)
    {
        classifiable_point.resize(1*1);
        classifiable_point.points[0].x = _cloud_in.points[i].x;
        classifiable_point.points[0].y = _cloud_in.points[i].y;
        classifiable_point.points[0].z = _cloud_in.points[i].z;
        classifiable_point.points[0].idx = 1;
        classifiable_point.points[0].feature = _cloud_in.points[i].curvature;
        classifiable_point.points[0].label = _label;
        _cloud_out.push_back(classifiable_point.points[0]);
    }




return;
}


void PCL_upd_DEMO::extractPatch(double _size, float _x, float _y, float _z)
{
   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

    m_cloud_patch->clear();  // clean the cloud in case of old patches in memory

    pcl::copyPointCloud(*m_cloud, *m_cloud_patch);

    double patch_side = _size/2;

    pcl::PassThrough<pcl::PointSurfel> pass;
    double z_min = _z - patch_side;
    double z_max = _z + patch_side;
    pass.setInputCloud (m_cloud_patch);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*m_cloud_patch);

    double y_min = _y - patch_side;
    double y_max = _y + patch_side;
    pass.setInputCloud (m_cloud_patch);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*m_cloud_patch);

    double x_min = _x - patch_side;
    double x_max = _x + patch_side;
    pass.setInputCloud (m_cloud_patch);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*m_cloud_patch);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointSurfel> rgb_color(m_cloud_patch, 255, 0, 255);
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



void PCL_upd_DEMO::clearLabelling()
{
	m_labeled_point->clear();
	m_labeled_cloud->clear();
    m_labelling_active = false;
	ui->treeWidget_classification->clear();
	 	ui->pushButton_start_stopLabelling->setEnabled(true);
		ui->pushButton_ground->setEnabled(false);
		ui->pushButton_notGround->setEnabled(false);
        ui->pushButton_ground->setStyleSheet("");
        ui->pushButton_notGround->setStyleSheet(""); // reset to default

		_label_counter=0;
		QMessageBox::information(this, "info !", " Cleared !!");

}

