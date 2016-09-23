/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"

void PCL_upd_DEMO::openFileList ( )
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
   viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
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

void PCL_upd_DEMO::openPCDFolder ()
{
        QDir m_dir = QFileDialog::getExistingDirectory(this, tr("Open folder"), QDir::currentPath(), 0);
        QStringList nameFilter;
        nameFilter << "*.pcd" << "*.bin";
        QStringList file_pcd_list;
        file_pcd_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );

        m_file_pcd_list.clear();
        for (size_t i=0; i < file_pcd_list.size(); i++)
        {
            QString file_with_absolute_path = m_dir.path();
            std::cout << file_pcd_list.at(i).toStdString() << std::endl;

            file_with_absolute_path.append("/");
            file_with_absolute_path.append(file_pcd_list.at(i));

            std::cout << file_with_absolute_path.toStdString() << std::endl;
            m_file_pcd_list.push_back(file_with_absolute_path);

        }
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

void PCL_upd_DEMO::removeFilters()
{
    pcl::copyPointCloud(*m_cloud, *m_cloud_filtered);
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");
    viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
    ui->qvtkWidget->update ();

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
    viewer->updatePointCloud (m_cloud, m_rgb_color, "cloud");
    ui->qvtkWidget->update ();
}


void PCL_upd_DEMO::about()
{
	 QMessageBox messageBox;
	 messageBox.about(0,"About MB"," https://www.maurobellone.com ");
	 messageBox.icon();
	 messageBox.setFixedSize(500,700);
}
