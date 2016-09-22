/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"

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
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_filtered);
    viewer->updatePointCloud (m_cloud_filtered, rgb_color, "cloud");
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
   pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_filtered);
   viewer->updatePointCloud (m_cloud_filtered, rgb_color, "cloud");
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
   pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_filtered);
   viewer->updatePointCloud (m_cloud_filtered, rgb_color, "cloud");
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
viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
ui->qvtkWidget->update ();
}

