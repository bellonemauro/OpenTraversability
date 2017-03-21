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

    if (m_labelled_paused == true)
	{
	ui->pushButton_start_stopLabelling->setEnabled(false);
//    ui->pushButton_start_stopLabelling->setText("Start/Stop Labelling");
    m_labelled_paused = false;
	}


    if (!m_labelling_active) // if not active, start
    {
        ui->pushButton_ground->setEnabled(true);
        ui->pushButton_notGround->setEnabled(true);
        ui->pushButton_ground->setStyleSheet("color: black;"
                                             "background-color: lightgreen;"
                                             "selection-color: red;");
        ui->pushButton_notGround->setStyleSheet("color: black;"
                                             "background-color: red;"
                                             "selection-color: red;");
        m_labelling_active = true;
    }
    else // it was active, stop
    {
        ui->pushButton_ground->setEnabled(false);
        ui->pushButton_notGround->setEnabled(false);
        ui->pushButton_ground->setStyleSheet("");
        ui->pushButton_notGround->setStyleSheet(""); // reset to default
        m_labelling_active = false;
    }

}


void PCL_upd_DEMO::labelGround()
{
    // prepare data for the classifier
    std::vector<float> features = createAllFeatures();

    if (features.size() < 1 )return; // do nothing if we do not have features

    addSVMdataToTrainingSet( features, +1.0 );
    addCNNdataToTrainingSet( features, 1 );

     QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
     item->setText(0,"Ground");
     stringstream ss;
     //ss << " x " << m_labeled_point->at(0).x << ", y = " << m_labeled_point->at(0).y << ", z =" << m_labeled_point->at(0).z;
     ss << " x " << UPD_cloud->points[ m_patch_labelling_index ].x
        << ", y = " << UPD_cloud->points[ m_patch_labelling_index ].y
        << ", z = " << UPD_cloud->points[ m_patch_labelling_index ].z;
     item->setText(1,QString::fromStdString(ss.str()));

     ss.str("");
     ss << " r-vector = " << features.at(0) << " -- "
                       << features.at(1) << " -- "
                       << features.at(2) << " -- ";
     item->setText(3,QString::fromStdString(ss.str()) );
     ss.str("");
     ss << " upd = " << features.at(3);
     item->setText(2,QString::fromStdString(ss.str()) );


     ui->treeWidget_classification->resizeColumnToContents(0);
     ui->treeWidget_classification->resizeColumnToContents(1);
     ui->treeWidget_classification->resizeColumnToContents(2);
     ui->treeWidget_classification->resizeColumnToContents(3);

     ui->treeWidget_classification->insertTopLevelItem(0,item);

     double norm = Eigen::Vector3d(UPD_cloud->points[ m_patch_labelling_index ].normal_x,
             UPD_cloud->points[ m_patch_labelling_index ].normal_y, UPD_cloud->points[ m_patch_labelling_index ].normal_z).norm();

     pcl::PointXYZ P1 ( UPD_cloud->points[ m_patch_labelling_index ].x + UPD_cloud->points[ m_patch_labelling_index ].normal_x/norm,
                        UPD_cloud->points[ m_patch_labelling_index ].y + UPD_cloud->points[ m_patch_labelling_index ].normal_y/norm,
                        UPD_cloud->points[ m_patch_labelling_index ].z + UPD_cloud->points[ m_patch_labelling_index ].normal_z/norm);
     pcl::PointXYZ P2 ( UPD_cloud->points[ m_patch_labelling_index ].x,
                        UPD_cloud->points[ m_patch_labelling_index ].y,
                        UPD_cloud->points[ m_patch_labelling_index ].z);

     if(viewer->addArrow(P1, P2, 0.0, 1.0, 0.0, false, "arrow", 0)) //the arrow is attached to P1
         ui->qvtkWidget->update ();
     else { viewer->removeShape("arrow",0);
     viewer->addArrow(P1, P2, 0.0, 1.0, 0.0, false, "arrow", 0);
     }
     ui->qvtkWidget->update ();
     int num = ui->lcdNumber_ground->intValue();
     num++;
     ui->lcdNumber_ground->display(num);

}

void PCL_upd_DEMO::labelNotGround()
{
    // prepare data for the classifier
    std::vector<float> features = createAllFeatures();

    if (features.size() < 1 )return; // do nothing if we do not have features

    addSVMdataToTrainingSet( features, -1.0 );  // for SVM labels are \in [-1.0, 1.0]
    addCNNdataToTrainingSet( features,  0 );  // for cnn labels is a uint32 not a float


     QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
     item->setText(0,"NOT Ground");
     stringstream ss;
     //ss << " x " << m_labeled_point->at(0).x << ", y = " << m_labeled_point->at(0).y << ", z =" << m_labeled_point->at(0).z;
     ss << " x " << UPD_cloud->points[ m_patch_labelling_index ].x
        << ", y = " << UPD_cloud->points[ m_patch_labelling_index ].y
        << ", z = " << UPD_cloud->points[ m_patch_labelling_index ].z;
     item->setText(1,QString::fromStdString(ss.str()));

     ss.str("");
     ss << " r-vector = " << features.at(0) << " -- "
                       << features.at(1) << " -- "
                       << features.at(2) << " -- ";
     item->setText(3,QString::fromStdString(ss.str()) );
     ss.str("");
     ss << " upd = " << features.at(3);
     item->setText(2,QString::fromStdString(ss.str()) );


     ui->treeWidget_classification->resizeColumnToContents(0);
     ui->treeWidget_classification->resizeColumnToContents(1);
     ui->treeWidget_classification->resizeColumnToContents(2);
     ui->treeWidget_classification->resizeColumnToContents(3);

     ui->treeWidget_classification->insertTopLevelItem(0,item);

     double norm = Eigen::Vector3d(UPD_cloud->points[ m_patch_labelling_index ].normal_x,
             UPD_cloud->points[ m_patch_labelling_index ].normal_y, UPD_cloud->points[ m_patch_labelling_index ].normal_z).norm();

     pcl::PointXYZ P1 ( UPD_cloud->points[ m_patch_labelling_index ].x + UPD_cloud->points[ m_patch_labelling_index ].normal_x/norm,
                        UPD_cloud->points[ m_patch_labelling_index ].y + UPD_cloud->points[ m_patch_labelling_index ].normal_y/norm,
                        UPD_cloud->points[ m_patch_labelling_index ].z + UPD_cloud->points[ m_patch_labelling_index ].normal_z/norm);
     pcl::PointXYZ P2 ( UPD_cloud->points[ m_patch_labelling_index ].x,
                        UPD_cloud->points[ m_patch_labelling_index ].y,
                        UPD_cloud->points[ m_patch_labelling_index ].z);

     if(viewer->addArrow(P1, P2, 0.0, 1.0, 0.0, false, "arrow", 0)) //the arrow is attached to P1
         ui->qvtkWidget->update ();
     else { viewer->removeShape("arrow",0);
     viewer->addArrow(P1, P2, 1.0, 0.0, 0.0, false, "arrow", 0);
     }ui->qvtkWidget->update ();
     int num = ui->lcdNumber_notGround->intValue();
     num++;
     ui->lcdNumber_notGround->display(num);

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


    pcl::NormalEstimation<pcl::PointSurfel, pcl::PointSurfel> ne;
    ne.setInputCloud (m_cloud_patch);

    pcl::search::KdTree<pcl::PointSurfel>::Ptr tree (new pcl::search::KdTree<pcl::PointSurfel> ());
    ne.setSearchMethod (tree);
    double d_m = 0;
    bool isNumeric;
    d_m = ui->lineEdit_patchSize->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "the patch size is not a valid number - Filter cannot be applied !");
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }
    ne.setRadiusSearch (d_m);

    // Compute the features
    ne.compute (*m_cloud_patch);


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

