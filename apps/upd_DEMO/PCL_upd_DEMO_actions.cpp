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
      if (m_path_to_pcd_list.isEmpty()){
          QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
          return;
      }
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
    QString load_path = QFileDialog::getOpenFileName (this, tr("Open cloud"), QDir::currentPath(),  // dialog to open files
                        "ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);
    if (load_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    if (pcl::io::loadPCDFile (load_path.toUtf8().constData(), *m_cloud) == -1){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        QMessageBox::warning(this, "Warning !", "File not found ! <br>" + load_path);
	}
        
   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

   ui->label_fileStatus_cloud->setText(QString::fromStdString(load_path.toUtf8().constData()));  // set the status bar to the current pcd
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
			QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
             QMessageBox::information(this, tr("Image Viewer"),
                                      tr("Cannot load %1.").arg(fileName));
        return;
         }
         ui->label_image->setPixmap(QPixmap::fromImage(image));
         ui->label_image->adjustSize();
     }
}

void PCL_upd_DEMO::openImageList()
{
        //QFileDialog::getOpenFileName (parent, dialog title, default folder, allowed formats)
      m_path_to_image_list = QFileDialog::getOpenFileName (	this, tr("Open File list"), QDir::currentPath(),  // dialog to open files
                        "Text file (*.txt);; All Files(*.*)" , 0);
      if (m_path_to_image_list.isEmpty()){
          QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
          return;
      }
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
        // specify what happens when we press "cancel" on the window
        // now it will get the dot folder   m_dir = ./

        QStringList nameFilter;
        nameFilter << "*.pcd" << "*.bin";
        QStringList file_pcd_list;
        file_pcd_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );

        if (file_pcd_list.size() < 1 )
        {
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            QMessageBox::warning(this, "Warning !", "No PCD files in folder " + m_dir.currentPath());
            return;
        }

        m_file_pcd_list.clear();
        for (unsigned int i=0; i < file_pcd_list.size(); i++)
        {
            QString file_with_absolute_path = m_dir.path();
            //std::cout << file_pcd_list.at(i).toStdString() << std::endl;

            file_with_absolute_path.append("/");
            file_with_absolute_path.append(file_pcd_list.at(i));

            //std::cout << file_with_absolute_path.toStdString() << std::endl;
            m_file_pcd_list.push_back(file_with_absolute_path);

        }
        ui->listWidget_pcdNames->clear();
        ui->listWidget_pcdNames->addItems(m_file_pcd_list);


        nameFilter.clear();
        nameFilter << "*.jpg" << "*.pgn" << "*.pgm" << "*.bmp";
        QStringList file_images_list;
        file_images_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );

        if (file_images_list.size()>1)
        {
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            QMessageBox::StandardButton answer;
            answer = QMessageBox::question( this, "Warning !",
                                            "Images files found in the folder " + m_dir.currentPath() + " Do you want to load images ?",
                                            QMessageBox::Yes|QMessageBox::No);
            if (answer == QMessageBox::No )
            {
                return;
            }
            else
            {
                m_file_image_list.clear();
                for (unsigned int i=0; i < file_images_list.size(); i++)
                {
                    QString file_with_absolute_path = m_dir.path();
                    //std::cout << file_images_list.at(i).toStdString() << std::endl;

                    file_with_absolute_path.append("/");
                    file_with_absolute_path.append(file_images_list.at(i));

                    //std::cout << file_with_absolute_path.toStdString() << std::endl;
                    m_file_image_list.push_back(file_with_absolute_path);

                }
                ui->listWidget_imageNames->clear();
                ui->listWidget_imageNames->addItems(m_file_image_list);
            }
        }


}

void PCL_upd_DEMO::openImagesFolder ()
{
        QDir m_dir = QFileDialog::getExistingDirectory(this, tr("Open folder"), QDir::currentPath(), 0);
        // specify what happens when we press "cancel" on the window
        // now it will get the dot folder   m_dir = ./

        QStringList nameFilter;
        nameFilter << "*.jpg" << "*.pgn" << "*.pgm" << "*.bmp";
        QStringList file_images_list;
        file_images_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );

        if (file_images_list.size() < 1 )
        {
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            QMessageBox::warning(this, "Warning !", "No image files in folder " + m_dir.currentPath());
            return;
        }


        m_file_image_list.clear();
        for (unsigned int i=0; i < file_images_list.size(); i++)
        {
            QString file_with_absolute_path = m_dir.path();
            //std::cout << file_images_list.at(i).toStdString() << std::endl;

            file_with_absolute_path.append("/");
            file_with_absolute_path.append(file_images_list.at(i));

            //std::cout << file_with_absolute_path.toStdString() << std::endl;
            m_file_image_list.push_back(file_with_absolute_path);

        }
        ui->listWidget_imageNames->clear();
        ui->listWidget_imageNames->addItems(m_file_image_list);

        nameFilter.clear();
        nameFilter << "*.pcd" << "*.bin";
        QStringList file_pcd_list;
        file_pcd_list = m_dir.entryList( nameFilter, QDir::Files | QDir::NoDotAndDotDot );

        if (file_pcd_list.size()>1)
        {
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            QMessageBox::StandardButton answer;
            answer = QMessageBox::question( this, "Warning !",
                                            "Point cloud files found in the folder " + m_dir.currentPath() + " Do you want to load images ?",
                                            QMessageBox::Yes|QMessageBox::No);
            if (answer == QMessageBox::No )
            {
                return;
            }
            else
            {
                m_file_pcd_list.clear();
                for (unsigned int i=0; i < file_pcd_list.size(); i++)
                {
                    QString file_with_absolute_path = m_dir.path();
                    //std::cout << file_pcd_list.at(i).toStdString() << std::endl;

                    file_with_absolute_path.append("/");
                    file_with_absolute_path.append(file_pcd_list.at(i));

                    //std::cout << file_with_absolute_path.toStdString() << std::endl;
                    m_file_pcd_list.push_back(file_with_absolute_path);

                }
                ui->listWidget_pcdNames->clear();
                ui->listWidget_pcdNames->addItems(m_file_pcd_list);
            }
        }
}


void
PCL_upd_DEMO::saveFile()
{

    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

    if (m_cloud->empty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        QMessageBox::warning(this, "Warning !", "Cloud empty cannot be saved  ! " );
        return;
    }

    QString cloud_path = QFileDialog::getSaveFileName (this, tr("Save ASCII cloud"), QDir::currentPath(),  // dialog to open files
                        "ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);
    if (cloud_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    m_cloud->resize(m_cloud->size());
    if (pcl::io::savePCDFileASCII (cloud_path.toUtf8().constData(), *m_cloud) == -1) QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pdc_list.at(0) << endl;

    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void
PCL_upd_DEMO::saveUPDFile()
{

    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

    if (UPD_cloud->empty()) {
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        QMessageBox::warning(this, "Warning !", "UPD cloud empty cannot be saved  ! " );
        return;
    }

    QString cloud_path = QFileDialog::getSaveFileName (this, tr("Save ASCII cloud"), QDir::currentPath(),  // dialog to open files
                        "ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);
    if (cloud_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    m_cloud->resize(m_cloud->size());
    if (pcl::io::savePCDFileASCII (cloud_path.toUtf8().constData(), *UPD_cloud) == -1)
        QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pcd_list.at(0) << endl;

   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void
PCL_upd_DEMO::saveLabeledFile()
{
    QMessageBox::warning(this, "Warning !", "Not used ! " );
    return;

    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
    /*if (UPD_cloud->isEmpty())
    {QMessageBox::warning(this, "Warning !", "UPD cloud empty not saved  ! " );
    }*/
//    QString cloud_path = QFileDialog::getSaveFileName (this, tr("Save ASCII cloud"), QDir::currentPath(),  // dialog to open files
//                        "ASCII Point Cloud File (*.pcd);; Binary Cloud File (*.bin);; All Files(*.*)" , 0);

//    if (pcl::io::savePCDFileASCII (cloud_path.toUtf8().constData(), *m_labeled_cloud) == -1) QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + cloud_path);
                //cout << "Loaded " << cloud->size () << " data points from " << m_file_pcd_list.at(0) << endl;

   QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}


void
PCL_upd_DEMO::saveClassifierModel()
{

    if (!m_svm_model.isValid()) {
        QMessageBox::warning(this, "Warning !", "No valid SVM model" );
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode


    QString save_path = QFileDialog::getSaveFileName (this, tr("Save SVM classifier model"), QDir::currentPath(),  // dialog to open files
                        " Data File (*.dat);; All Files(*.*)" , 0);
    if (save_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    if (!m_svm_trainer.saveClassifierModel ( save_path.toUtf8().constData() ) )
        QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + save_path);
    else
        QMessageBox::information(this, "info !", " PCL-SVM demo - SVM classifier model file saved at " +  save_path );

    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void
PCL_upd_DEMO::saveTrainingDataset()
{

    if (m_training_set.size()<1 ) {
        QMessageBox::warning(this, "Warning !", "NO training data to be saved ! " );
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode


    QString save_path = QFileDialog::getSaveFileName (this, tr("Save SVM training set"), QDir::currentPath(),  // dialog to open files
                        "Data File (*.dat);; All Files(*)" , 0);
    if (save_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    m_svm_trainer.resetTrainingSet();
    m_svm_trainer.setInputTrainingSet(m_training_set);
    if (!m_svm_trainer.saveNormTrainingSet ( save_path.toUtf8().constData() ) )
        QMessageBox::warning(this, "Warning !", "File not saved ! <br>" + save_path);
    else
        QMessageBox::information(this, "info !", " PCL-SVM demo - SVM training set file saved at " +  save_path );

    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void
PCL_upd_DEMO::saveParameters()
{

    QString save_path = QFileDialog::getSaveFileName (this, tr("Save SVM training set"), QDir::currentPath(),  // dialog to open files
                        "Data File (*.dat);; All Files(*)" , 0);
    if (save_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    // to add time
    time_t now = time(0);
    tm *ltm = localtime(&now);

    string _Log_file_name;
    _Log_file_name = save_path.toStdString();


    { //save file
        ofstream _LogFile(_Log_file_name.c_str(), ios_base::app | ios_base::out);
        if (_LogFile.is_open())
        {
            _LogFile << "%% +---------------------------------------------------------------------------+" << endl;
            _LogFile << "%% |                   Chalmers University of technology                       |" << endl;
            _LogFile << "%% |                              Mauro Bellone                                |" << endl;
            _LogFile << "%% |                                                                           |" << endl;
            _LogFile << "%% |                      http://www.maurobellone.com/                         |" << endl;
            _LogFile << "%% +---------------------------------------------------------------------------+" << endl;
            _LogFile << "" << endl;
            _LogFile << "%% Parameter File Header V. 0.1 " << endl;
            _LogFile << "%% file created on dd/mm/yyyy " << 1 + ltm->tm_mon << "/" << ltm->tm_mday << "/" << 1900 + ltm->tm_year << " at " << ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << endl;
            _LogFile << "%% SVM classifier parameters report " << endl;
            _LogFile << "%% +---------------------------------------------------------------------------+" << endl;
            _LogFile << "%% Configuration Parameters: " << endl;
            _LogFile << "%% Gamma       " << ui->lineEdit_SVMgamma->text().toStdString() << "; "<< endl;
            _LogFile << "%% C           " << ui->lineEdit_SVMc->text().toStdString() << "; " << endl;
            _LogFile << "%% Degree      " << ui->lineEdit_SVMdegree->text().toStdString() << "; " << endl;
            _LogFile << "%% SVM type    " << ui->comboBox_SVMtype->currentText().toStdString() << "; " << endl;
            _LogFile << "%% Kernel type " << ui->comboBox_svmKernelType->currentText().toStdString() << "; " << endl;
            _LogFile << "%% Shrinking   " << ui->checkBox_Shrinking->isChecked() << "; " << endl;
            _LogFile << "%% Probability " << ui->checkBox_probability->isChecked() << "; " << endl;
            _LogFile << "%% +---------------------------------------------------------------------------+ " << endl;

            //at the first call one sample is lost !!
            _LogFile.close();
            QMessageBox::information(this, "info !", " PCL-SVM demo - SVM parameter file saved at " +  QString::fromStdString(_Log_file_name) );

        }
    }



}

void
PCL_upd_DEMO::saveTrainingProblem()
{
    // check the problem before saving !
    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode


    QString save_path = QFileDialog::getSaveFileName (this, tr("Save SVM training problem"), QDir::currentPath(),  // dialog to open files
                        "Data File (*.dat);; All Files(*)" , 0);
    if (save_path.isEmpty()){
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }
/// TODO protected function ! see what's going on here !
//    if (!m_svm_trainer.saveProblem ( save_path.toUtf8().constData(), true ) ) ///TODO: true if labelled/ false otherwise
        QMessageBox::warning(this, "Warning !", "This is still experimental \n File not saved ! <br>" + save_path);
//        else
//            QMessageBox::information(this, "info !", " PCL-SVM demo - SVM training problem file saved at " +  save_path );

    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
}



void
PCL_upd_DEMO::loadClassifierModel()
{

QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
QString load_path = QFileDialog::getOpenFileName (this, tr("Open SVM model"), QDir::currentPath(),  // dialog to open files
                    "Model Data file (*.dat);; All Files(*.*)" , 0);
if (load_path.isEmpty()){
    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    return;
}

if (!m_svm_classifier.loadClassifierModel (load_path.toUtf8().constData()) ){
    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "File not found ! <br>" + load_path);
}
// set the model from the classifier to our internal class member
m_svm_model = m_svm_classifier.getClassifierModel();
QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

}

void
PCL_upd_DEMO::loadTrainingDataset()
{

QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode
QString load_path = QFileDialog::getOpenFileName (this, tr("Open training dataset"), QDir::currentPath(),  // dialog to open files
                                                  "Training Data file (*.dat);; All Files(*.*)" , 0);
if (load_path.isEmpty()){
    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    return;
}

if (!m_svm_trainer.loadProblem (load_path.toUtf8().constData()) ){
//if (!m_svm_classifier.loadClassProblem(load_path.toUtf8().constData()) ){
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
    QMessageBox::warning(this, "Warning !", "File not found ! <br>" + load_path);
}

///TODO IMPORTANT: if the training set has data, they may be replaced
m_svm_trainer.adaptProbToInput();
m_training_set = m_svm_trainer.getInputTrainingSet();
addSVMdataToTable(m_training_set);

//m_training_set = m_svm_classifier.getInputTrainingSet();

QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

std::cout << " dataset loaded " << m_training_set.size() << std::endl;

}


void
PCL_upd_DEMO::loadTrainingProblem()
{

}

void
PCL_upd_DEMO::loadSVMparameters()
{

}

void
PCL_upd_DEMO::loadClassificationProblem()
{

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
