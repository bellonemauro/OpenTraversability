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

std::vector<float>
PCL_upd_DEMO::createFeaturesReina()
{

    // prepare data for the classifier
    std::vector<float> features;
    // if we have no patch we cannot classify
    if ( m_cloud_patch->size() < 1 || m_cloud_patch->size() < m_patch_labelling_index ) {
        QMessageBox::information(this, "info !", " Patch size error !!");
        return features; }

    // covariance matrix
    Eigen::Matrix3d covariance_matrix;
    pcl::computeCovarianceMatrix(*m_cloud_patch, covariance_matrix);
    std::cout << " covariance matrix is \n"<< covariance_matrix << std::endl;

    // singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> SVD (covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = SVD.matrixU();
    Eigen::MatrixXd V = SVD.matrixV();
    Eigen::MatrixXd singular_values = SVD.singularValues().asDiagonal();

    std::cout << "SVD decomposition : \n"
              << " U = \n" << U << "\n"
              << " V = \n" << V << "\n"
              << " singular values: \n" << singular_values << std::endl;
    std::cout << "verification : COV = U * sigma * V \n" << U * singular_values * V.transpose() << std::endl;

    // check for the rank !
    Eigen::FullPivLU<Eigen::Matrix3d> lu_decomp(covariance_matrix);
    std::cout <<  " The rank of the covariance matrix is " << lu_decomp.rank() << std::endl;
/*    if (lu_decomp.rank()>2) // we have a 3d surface
    {

    }
    else if (lu_decomp.rank() == 2 ) // we are on a plane
    {

    }
    else if (lu_decomp.rank() == 1 ) // we are on a line
    {

    }
    else  // the matrix is zero
    {
        QMessageBox::information(this, "info !", " Patch size error - matrix 0 !!");
        return features;
    }*/


    // slope
    U.normalize();
    std::cout << " U normalized = \n" << U << std::endl;
    Eigen::Vector3d normalDirection (0.0, 0.0, 1.0);//.transpose()
    Eigen::Vector3d product = U.transpose() * normalDirection;
    std::cout << " product = " << product << "\n norm = " << product.norm() <<  std::endl;
    double slope = std::acos( product.norm());
    std::cout << " slope = " << slope << std::endl; //this must be a number so U is actually the eigenvector associated to the smallest eigenvalue

    // smallest singular value
    float small_singular_value = singular_values(2, 2);
    std::cout << " the smallest singular value is " << small_singular_value << std::endl;

    // mean along z
    float mean = 0.0;
    for (int i = 0; i< m_cloud_patch->size(); i++) {
        mean +=m_cloud_patch->at(i).z;
    }
    mean = mean / m_cloud_patch->size();
    std::cout << " the mean is : " << mean << std::endl;

    // variance in the vertical direction
    float variance = 0.0;
    for (int i = 0; i< m_cloud_patch->size(); i++)  {
        variance += (m_cloud_patch->at(i).z - mean)*(m_cloud_patch->at(i).z - mean);
    }
    std::cout << " the variance is : " << variance << std::endl;

    return features;
}

std::vector<float>
PCL_upd_DEMO::createFeaturesUPD(  )
{
    // prepare data for the classifier
    std::vector<float> features;

    // if we have no patch we cannot classify
    if ( m_cloud_patch->size() < 1 || m_cloud_patch->size() < m_patch_labelling_index ) {
        QMessageBox::information(this, "info !", " Patch size error !!");
        return features; }

    Eigen::Vector3d vec ( UPD_cloud->points[m_patch_labelling_index].normal_x,
                          UPD_cloud->points[m_patch_labelling_index].normal_y,
                          UPD_cloud->points[m_patch_labelling_index].normal_z);
    double norm = vec.norm();
    features.push_back( UPD_cloud->points[m_patch_labelling_index].normal_x / norm );
    features.push_back( UPD_cloud->points[m_patch_labelling_index].normal_y / norm );
    features.push_back( UPD_cloud->points[m_patch_labelling_index].normal_z / norm );
    features.push_back( UPD_cloud->points[ m_patch_labelling_index ].radius );  // upd value
    features.push_back( norm );
    double cloud_density = 100.0; //points per square meter
    double m_e = cloud_density * M_PI * std::pow(m_upd->getSearchRadius(), 2);
    std::cout << " the UPD cloud size for the patch is " << UPD_cloud->size()
              << " m_ e = " << m_e
              << " C = K/M_E " << UPD_cloud->size() / m_e << std::endl;
    features.push_back( UPD_cloud->size() / m_e );


    return features;

}



void PCL_upd_DEMO::labelGround()
{

    // prepare data for the classifier
    std::vector<float> featuresUPD = createFeaturesUPD();

    std::vector<float> featuresReina = createFeaturesReina();

    addSVMdataToTrainingSet( featuresUPD, +1.0 );


     QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
     item->setText(0,"Ground");
     stringstream ss;
     //ss << " x " << m_labeled_point->at(0).x << ", y = " << m_labeled_point->at(0).y << ", z =" << m_labeled_point->at(0).z;
     ss << " x " << UPD_cloud->points[ m_patch_labelling_index ].x
        << ", y = " << UPD_cloud->points[ m_patch_labelling_index ].y
        << ", z = " << UPD_cloud->points[ m_patch_labelling_index ].z;
     item->setText(1,QString::fromStdString(ss.str()));

     ss.str("");
     ss << " r-vector = " << featuresUPD.at(0) << " -- "
                       << featuresUPD.at(1) << " -- "
                       << featuresUPD.at(2) << " -- ";
     item->setText(3,QString::fromStdString(ss.str()) );
     ss.str("");
     ss << " upd = " << featuresUPD.at(3);
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
    std::vector<float> featuresUPD = createFeaturesUPD();

    std::vector<float> featuresReina = createFeaturesReina();

    addSVMdataToTrainingSet( featuresUPD, -1.0 );

     QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
     item->setText(0,"NOT Ground");
     stringstream ss;
     //ss << " x " << m_labeled_point->at(0).x << ", y = " << m_labeled_point->at(0).y << ", z =" << m_labeled_point->at(0).z;
     ss << " x " << UPD_cloud->points[ m_patch_labelling_index ].x
        << ", y = " << UPD_cloud->points[ m_patch_labelling_index ].y
        << ", z = " << UPD_cloud->points[ m_patch_labelling_index ].z;
     item->setText(1,QString::fromStdString(ss.str()));

     ss.str("");
     ss << " r-vector = " << featuresUPD.at(0) << " -- "
                       << featuresUPD.at(1) << " -- "
                       << featuresUPD.at(2) << " -- ";
     item->setText(3,QString::fromStdString(ss.str()) );
     ss.str("");
     ss << " upd = " << featuresUPD.at(3);
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

void PCL_upd_DEMO::addSVMdataToTrainingSet(float _f1, float _f2, float _f3, float _label )
{


    pcl::SVMDataPoint svm_data_point; //--> a data point just for simplicity
    pcl::SVMData svm_data;

    svm_data.label = _label;

    svm_data_point.idx = 1;
    svm_data_point.value = _f1;
    svm_data.SV.push_back(svm_data_point);

    svm_data_point.idx = 2;
    svm_data_point.value = _f2;
    svm_data.SV.push_back(svm_data_point);

    svm_data_point.idx = 3;
    svm_data_point.value = _f3;
    svm_data.SV.push_back(svm_data_point);

    m_training_set.push_back(svm_data);
}

void PCL_upd_DEMO::addSVMdataToTrainingSet(std::vector<float> _features, float _label )
{

    if (_features.size() < 1 )
    {
        std::cout << "PCL_upd_DEMO::addSVMdataToTrainingSet  << error >> : no features " << std::endl;
        return;
    }

    pcl::SVMDataPoint svm_data_point; //--> a data point just for simplicity
    pcl::SVMData svm_data;

    svm_data.label = _label;

    for (int i=0; i<_features.size(); i++)
    {
        svm_data_point.idx = i+1;
        svm_data_point.value = _features.at(i);
        svm_data.SV.push_back(svm_data_point);

    }

    m_training_set.push_back(svm_data);
}

void
PCL_upd_DEMO::getGUIclassifierParams()
{
    // set SVM type
    int svm_type = 0;// C_SVC, NU_SVC, ONE_CLASS, EPSILON_SVR, NU_SVR  // svm_type
    svm_type = ui->comboBox_SVMtype->currentIndex();
    m_svm_parameters.svm_type = svm_type;

    // set SVM kernel
    int kernel_type = 0;// LINEAR, POLY, RBF, SIGMOID, PRECOMPUTED  // kernel_type
    kernel_type = ui->comboBox_svmKernelType->currentIndex();
    m_svm_parameters.kernel_type = kernel_type;

    // set SVM gamma -- this is used for poly/rbf/sigmoid kernels
    double gamma = 0;
    bool isNumeric;
    gamma = ui->lineEdit_SVMgamma->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "GAMMA is not a valid number !");
        return;
    };
    m_svm_parameters.gamma = gamma;

    // set SVM C param
    double C = 0;
    C = ui->lineEdit_SVMc->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "C is not a valid number !");
        return;
    };
    m_svm_parameters.C = C;

    double degree = 0;
    degree = ui->lineEdit_SVMdegree->text().toDouble(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "SVM degree is not a valid number !");
        return;
    };
    m_svm_parameters.degree = degree;

    m_svm_parameters.probability = ui->checkBox_probability->isChecked();
    m_svm_parameters.shrinking = ui->checkBox_Shrinking->isChecked();
}

void
PCL_upd_DEMO::trainClassifier()
{
    getGUIclassifierParams();
    m_svm_trainer.setParameters(m_svm_parameters);  // set the parameters for the trainer

    m_svm_trainer.resetTrainingSet();
    m_svm_trainer.setInputTrainingSet(m_training_set);

    // train the classifier
    if (m_svm_trainer.trainClassifier() )
       {
         std::cout << "\t The classifier has been successfully tranined \n\n";
       }
    else
       {
         std::cout << "\t The classifier has NOT been tranined  - Exit now ! \n\n";
         return ;
       }

    // check the model for the classifier
    m_svm_model = m_svm_trainer.getClassifierModel();

    std::cout << "\t Model parameters summary : \n";
    if ((m_svm_parameters.probability?true:false)){
        std::cout << "\t\t  Probability support \t active  \n";
        std::cout << "\t\t  ProbA = \t" << *m_svm_model.probA << " \n"
                  << "\t\t  ProbB = \t" << *m_svm_model.probB << " \n";
    }
    else {
     std::cout << "\t\t  Probability support \t NOT active  \n";
    }
    std::cout << "\t\t  l  \t \t \t " << m_svm_model.l  << " \n";
    std::cout << "\t\t  Number of classes   \t " << m_svm_model.nr_class << " \n";
    std::cout << "\t\t  sv_coef   \t  \t " <<  *(*m_svm_model.sv_coef) << " \n";
    std::cout << "\t\t  Rho   \t  \t " <<  *m_svm_model.rho << " \n";

    if ( m_svm_model.label == NULL ) std::cout << "\t\t  label   \t  \t  NULL \n";
    else std::cout << "\t\t  label   \t  \t " <<  *m_svm_model.label << " \n";

    if (m_svm_model.nSV == NULL ) std::cout << "\t\t  nSV \t   \t  \t  NULL \n";
    else std::cout << "\t\t  nSV \t   \t  \t " <<  *m_svm_model.nSV << " \n\n";

}


void PCL_upd_DEMO::classification()
{

    pcl::SVMDataPoint svm_data_point; //--> a data point just for simplicity
    pcl::SVMData svm_data;

    if (UPD_cloud->empty() || UPD_cloud->size() < m_patch_labelling_index) {
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        QMessageBox::warning(this, "Warning !", "UPD cloud cannot be labelled  ! " );
        return;
    }


    std::vector<float> featuresUPD = createFeaturesUPD();

    for (int i=0; i<featuresUPD.size(); i++)
    {
        svm_data_point.idx = i+1;
        svm_data_point.value = featuresUPD.at(i);
        svm_data.SV.push_back(svm_data_point);

    }

    ////////////////////////////////////////////////////////TODO !!!!!!!!!!!

    std::vector<pcl::SVMData> test_set;
    test_set.push_back(svm_data);

    if(!m_svm_model.isValid() ){
        std::cout << " null model" << std::endl;
        return;
    }
    m_svm_classifier.setClassifierModel(m_svm_model);

    getGUIclassifierParams();

    m_svm_classifier.setProbabilityEstimates((m_svm_parameters.probability?true:false));

    m_svm_classifier.resetTrainingSet();
    m_svm_classifier.setInputTrainingSet(test_set);

    if ( m_svm_classifier.classification( ) )
      std::cout << "\t Classification DONE ! \n";
    else {
      std::cout << "\t Classification ERROR --- Exit now ! \n\n";
      return ;
    }
std::vector< std::vector<double> > classification_result;

m_svm_classifier.getClassificationResult(classification_result);
//std::cout << "\t  Classification result size = \t  " << classification_result.size() << " \n";
//std::cout << "\t  Classification result = \t  " << classification_result.at(0).at(0) << " \n";

if (classification_result.at(0).at(0) == 1 )
    QMessageBox::information(this, "info !", " it's ground!! \n result size " +  QString::number(classification_result.size()) );

if (classification_result.at(0).at(0) == -1 )
    QMessageBox::information(this, "info !", " it's NOT ground!! \n result size " +  QString::number(classification_result.size()) );

}



void PCL_upd_DEMO::classificationTest()
{
// TODO: check if the model is valid
m_svm_classifier.setClassifierModel(m_svm_model);

m_svm_classifier.setProbabilityEstimates((m_svm_parameters.probability?true:false));
m_svm_classifier.resetTrainingSet();
m_svm_classifier.setInputTrainingSet(m_training_set);

// set some vars for the test report
int number_of_positive_samples = 0;
int number_of_negative_samples = 0;
int number_of_unclassified_samples = 0;
std::vector< std::vector<double> > classification_result;

if ( m_svm_classifier.hasLabelledTrainingSet())
{
    std::cout << "\t Loaded dataset has labels, the classification test will run \n";
if ( m_svm_classifier.classificationTest( ) ) {
  m_svm_classifier.getClassificationResult(classification_result);
  std::cout << "\t Classification result size = \t  " << classification_result.size() << " \n";
  std::cout << "\t Classification test SUCCESS ! \n\n";
}
else  {
  std::cout << "\t Classification test NOT SUCCESS \n\n";  }
}
else std::cout << "\t Loaded dataset has NO labels, the classification test cannot be executed \n";

m_svm_classifier.getClassificationResult(classification_result);
std::cout << "\t  Classification result size = \t  " << classification_result.size() << " \n";
for (size_t i = 0; i < classification_result.size(); i++) {
 for (size_t j = 0; j < classification_result.at(i).size(); j++) {
    if ( classification_result.at(i).at(j) == 1 ) {
        number_of_positive_samples++;
      }
    else {
      if ( classification_result.at(i).at(j) == -1) {
         number_of_negative_samples++; }
      else {
         number_of_unclassified_samples++;
       }
     }
  }
}
std::cout << "\n\t Classification Results : \n";
std::cout << "\t\t  number of positive samples = \t " << number_of_positive_samples << " \n";
std::cout << "\t\t  number of negative samples = \t " << number_of_negative_samples << " \n";
std::cout << "\t\t  number of unclassified samples = \t " << number_of_unclassified_samples << " \n";
std::cout << "\t NOTE: using probability parameter will always results in \n"
          << "\t       high number of unclassified samples \n\n";


pcl::SVMtestReport svm_test_report = m_svm_classifier.getClassificationTestReport();

double accuracy = svm_test_report.accuracy;//double(test_correct)/m_training_set.size() *100.0 ;

std::cout << " percentage " << accuracy << std::endl;

QMessageBox::information(this, "info !", " Classification test accuracy " +
                                         QString::number(svm_test_report.accuracy) +
                                         "%  (" + QString::number(svm_test_report.correctPredictionsIdx ) +
                                         "/" + QString::number(svm_test_report.totalSamples ) + ")" );

return;
}

void
PCL_upd_DEMO::addSVMdataToTable(std::vector<pcl::SVMData> _data)
{
    if (_data.size() < 1 )
    {
        std::cout << "<< PCL_upd_DEMO::addSVMdataToTable : error >> too small dataset " << std::endl;
        return;
    }

ui->treeWidget_classification->clear();

for (int i = 0; i< _data.size(); i++)
{
    QTreeWidgetItem * item = new QTreeWidgetItem();   // and update the widget for the visualization
///TODO check for the right format of the input file, otherwise it will crash !
    if (_data.at(i).label == -1 )  item->setText(0,"NOT Ground");
    else if (_data.at(i).label == 1) item->setText(0,"Ground");
    else std::cout << " Label not reconized at line " << i << std::endl;

    stringstream ss;
    ss << " NO coordinate from file ";
    item->setText(1,QString::fromStdString(ss.str()));

    ss.str("");
    ss << " angle = " << _data.at(i).SV.at(0).value << " deg "
                      << _data.at(i).SV.at(1).value << " deg ";
    item->setText(3,QString::fromStdString(ss.str()) );
    ss.str("");
    ss << " upd = " << _data.at(i).SV.at(2).value;
    item->setText(2,QString::fromStdString(ss.str()) );


    ui->treeWidget_classification->resizeColumnToContents(0);
    ui->treeWidget_classification->resizeColumnToContents(1);
    ui->treeWidget_classification->resizeColumnToContents(2);
    ui->treeWidget_classification->resizeColumnToContents(3);

    ui->treeWidget_classification->insertTopLevelItem(0,item);
}


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

