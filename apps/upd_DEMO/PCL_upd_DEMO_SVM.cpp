/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"

/*
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

    m_svm_training_set.push_back(svm_data);
}
*/

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

    m_svm_training_set.push_back(svm_data);
}

void
PCL_upd_DEMO::getGUI_SVMclassifierParams()
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
PCL_upd_DEMO::trainSVMClassifier()
{
    getGUI_SVMclassifierParams();
    m_svm_trainer.setParameters(m_svm_parameters);  // set the parameters for the trainer


    ///TODO there is no check for the validity of the training set !
    m_svm_trainer.resetTrainingSet();
    m_svm_trainer.setInputTrainingSet(m_svm_training_set);

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


std::vector< std::vector<double> > PCL_upd_DEMO::SVMpatchClassification()
{

    pcl::SVMDataPoint svm_data_point; //--> a data point just for simplicity
    pcl::SVMData svm_data;
std::vector< std::vector<double> > classification_result;

    if (UPD_cloud->empty() || UPD_cloud->size() < m_patch_labelling_index) {
        //QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        //QMessageBox::warning(this, "Warning !", "UPD cloud cannot be labelled  ! " );
        return classification_result;
    }


    // create features
    std::vector<float> features = createAllFeatures();


    for (int i=0; i<features.size(); i++)
    {
        svm_data_point.idx = i+1;
        svm_data_point.value = features.at(i);
        svm_data.SV.push_back(svm_data_point);

    }

    ////////////////////////////////////////////////////////TODO !!!!!!!!!!!

    std::vector<pcl::SVMData> test_set;
    test_set.push_back(svm_data);

    if(!m_svm_model.isValid() ){
        std::cout << " null model" << std::endl;
        return classification_result;
    }
    m_svm_classifier.setClassifierModel(m_svm_model);

    getGUI_SVMclassifierParams();

    m_svm_classifier.setProbabilityEstimates((m_svm_parameters.probability?true:false));

    m_svm_classifier.resetTrainingSet();
    m_svm_classifier.setInputTrainingSet(test_set);

    if ( m_svm_classifier.classification( ) )
      std::cout << "\t Classification DONE ! \n";
    else {
      std::cout << "\t Classification ERROR --- Exit now ! \n\n";
      return classification_result;
    }


m_svm_classifier.getClassificationResult(classification_result);
//std::cout << "\t  Classification result size = \t  " << classification_result.size() << " \n";
//std::cout << "\t  Classification result size = \t  " << classification_result.at(0).size() << " \n";
//std::cout << "\t  \t\t Label = \t  " << classification_result.at(0).at(0) << " \n"; //this is the label
//std::cout << "\t  \t\t Probability class 1 = \t  " << classification_result.at(0).at(1) << " \n"; //probability of class 1
//std::cout << "\t  \t\t Probability class 2 = \t  " << classification_result.at(0).at(2) << " \n"; //probability of class 2

if (classification_result.at(0).at(0) == 1 ){
    //QMessageBox::information(this, "info !", " it's ground!! \n result size " +  QString::number(classification_result.size()) );
    std::cout << " ground " << std::endl;
    return classification_result;
}

if (classification_result.at(0).at(0) == -1 ){
    //QMessageBox::information(this, "info !", " it's NOT ground!! \n result size " +  QString::number(classification_result.size()) );
std::cout << " NOT ground " << std::endl;
return classification_result;
}
return classification_result;
}



void PCL_upd_DEMO::SVMclassificationTest()
{
// TODO: check if the model is valid
    m_svm_classifier.setClassifierModel(m_svm_model);  // the model must be loaded before the data

 // if the classifier has data, reset and add the right training set
 if (m_svm_classifier.getInputTrainingSet().size()>0)
     m_svm_classifier.resetTrainingSet();

m_svm_classifier.setInputTrainingSet(m_svm_training_set);
m_svm_classifier.setProbabilityEstimates((m_svm_parameters.probability?true:false));


// initiate some vars for the test report
int number_of_positive_samples = 0;
int number_of_negative_samples = 0;
int number_of_unclassified_samples = 0;
int number_of_true_positive = 0;
int number_of_true_negative = 0;
int number_of_false_positive = 0;
int number_of_false_negative = 0;

std::vector< std::vector<double> > classification_result;

// if we actually have training set and labels we can run the test
if ( m_svm_classifier.hasLabelledTrainingSet())
{
    std::cout << "\t Loaded dataset has labels, the classification test will run \n" << std::endl;
    if ( m_svm_classifier.classificationTest( ) )
    {
        m_svm_classifier.getClassificationResult(classification_result);
        std::cout << "\t Classification result size = \t  " << classification_result.size() << " \n" << std::endl;
        std::cout << "\t Classification test SUCCESS ! \n\n" << std::endl;
    }
    else  {
        QMessageBox::information(this, "info !", " Classification test NOT SUCCESS " );
        return;
    }
}
else std::cout << "\t Loaded dataset has NO labels, the classification test cannot be executed \n" << std::endl;


m_svm_classifier.getClassificationResult(classification_result);
std::cout << "\t  Classification result size = \t  " << classification_result.size() << " \n"
          << "\t classification_result.at(0).size()" << classification_result.at(0).size() << " \n"
          << "\t  \t\t Label = \t  " << classification_result.at(0).at(0) << " \n" //this is the label
          << "\t  \t\t Probability class 1 = \t  " << classification_result.at(0).at(1) << " \n" //probability of class 1
          << "\t  \t\t Probability class 2 = \t  " << classification_result.at(0).at(2) << " \n" //probability of class 2
          << " \n" << std::endl;
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

if ( classification_result.at(i).at(j) == 1 && m_svm_training_set.at(i).label == 1)
number_of_true_positive++;

if ( classification_result.at(i).at(j) == -1 && m_svm_training_set.at(i).label == -1)
number_of_true_negative++;

if ( classification_result.at(i).at(j) == 1 && m_svm_training_set.at(i).label == -1)
number_of_false_positive++;

if ( classification_result.at(i).at(j) == -1 && m_svm_training_set.at(i).label == 1)
number_of_false_negative++;





//std::copy(classification_result.begin(), classification_result.end(), output_iterator_2);
//std::copy(output_iterator_2.begin(), output_iterator_2.end(), output_file);

  }
}
std::cout << "\n\t Classification test report : \n";
std::cout << "\t\t  number of positive samples = \t " << number_of_positive_samples << " \n";
std::cout << "\t\t  number of negative samples = \t " << number_of_negative_samples << " \n";
std::cout << "\t\t  number of unclassified samples = \t " << number_of_unclassified_samples << " \n";
std::cout << "\t\t  number of true posivive samples = \t " << number_of_true_positive << " \n";
std::cout << "\t\t  number of true negative samples = \t " << number_of_true_negative << " \n";
std::cout << "\t\t  number of false posivive samples = \t " << number_of_false_positive << " \n";
std::cout << "\t\t  number of false negative samples = \t " << number_of_false_negative << " \n";
std::cout << "\t NOTE: using probability parameter will always results in \n"
          << "\t       high number of unclassified samples \n\n";

std::cout << " WRITING TO FILE ..... " << std::endl;
std::ofstream output_file("./classificationOutput.txt");
std::ostream_iterator<double> output_iterator(output_file, " ");

for (int i=0; i<classification_result.size(); i++)
{
    std::copy(classification_result.at(i).begin(), classification_result.at(i).end(), output_iterator);
    output_file << "\n";
}

pcl::SVMtestReport svm_test_report = m_svm_classifier.getClassificationTestReport();
//int correctPredictions = svm_test_report.correctPredictionsIdx;
//float MSE = svm_test_report.MSE;

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
    if (_data.at(i).label == -1 )  {
        item->setText(0,"NOT Ground");
        int num = ui->lcdNumber_notGround->intValue();
        num++;
        ui->lcdNumber_notGround->display(num);
    }
    else if (_data.at(i).label == 1){
        item->setText(0,"Ground");
        int num = ui->lcdNumber_ground->intValue();
        num++;
        ui->lcdNumber_ground->display(num);
    }
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


void PCL_upd_DEMO::SVMgenerateTraining()
{

    //safety check if have the training ect
    m_cloud_classifiedGNG->resize(m_cloud->size());
    for (unsigned int i = 0; i<50; i++)
    {
        //pick a random point
        PointT next_point;
        int rand_idx = rand() % m_cloud->size();
            next_point = m_cloud->at(rand_idx); // not possible to use 2D indexing


        //extract the patch
        double patch_size = 0.0;
        bool isNumeric;
        patch_size = ui->lineEdit_patchSize->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "Patch size is not a valid number - The patch cannot be extracted !");
            return;
        };
        extractPatch(patch_size, next_point.x, next_point.y, next_point.z);


        // the patch has no points to run the PCA,
        // better to not perform normal analysis
        if (m_cloud_patch->size()<3) {
            //       QMessageBox::information(this, "info !", " Not enough points,\nextract anothe patch ");
            std::cout << " if no neighborhood the point cannot be classified"<< std::endl;
            //return;
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
         search_point->points[0].x = next_point.x;
         search_point->points[0].y = next_point.y;
         search_point->points[0].z = next_point.z;
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


        // classification

        QMessageBox::StandardButton reply;
          reply = QMessageBox::question(this, "Test", "Ground ?",
                                        QMessageBox::Yes|QMessageBox::No);
          if (reply == QMessageBox::Yes) {
            std::cout << "Yes was clicked" << std::endl;
            labelGround();
          } else {
            std::cout << "no was clicked" << std::endl;
            labelNotGround();
          }



/*        std::cout << " ciao " << std::endl;
        if (res == true)
        {
            next_point.r = 0;
            next_point.g = 255;
            next_point.b = 0;
            m_cloud_classifiedGNG->push_back(next_point);
        }
        if (res == false)
        {
            next_point.r = 255;
            next_point.g = 0;
            next_point.b = 0;
            m_cloud_classifiedGNG->push_back(next_point);
        }*/

    }

//    viewer->removePointCloud();
//    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_classifiedGNG);
//    viewer->addPointCloud (m_cloud_classifiedGNG, rgb_color, "cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");

//    ui->qvtkWidget->update ();





}

void PCL_upd_DEMO::SVMclassifyCloud()
{

    //run the classification for all the patches in the cloud
    //safety check if have the training ect
    m_cloud_classifiedGNG->resize(m_cloud->size());
    for (unsigned int i = 0; i<m_cloud->size(); i++)
    {
        //pick a random point
        PointT next_point;
        //if(!m_cloud->isOrganized())
        //{
            next_point = m_cloud->at(i); // not possible to use 2D indexing
        //}
        //else
        //{
        //    random_point = m_cloud->at(0,0);//if the cloud is organized than it is possible to use 2D indexing
        //}



        //extract the patch
        double patch_size = 0.0;
        bool isNumeric;
        patch_size = ui->lineEdit_patchSize->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "Patch size is not a valid number - The patch cannot be extracted !");
            return;
        };
        extractPatch(patch_size, next_point.x, next_point.y, next_point.z);


        // the patch has no points to run the PCA,
        // better to not perform normal analysis
        if (m_cloud_patch->size()<3) {
            //       QMessageBox::information(this, "info !", " Not enough points,\nextract anothe patch ");
            std::cout << " if no neighborhood the point cannot be classified"<< std::endl;
            //return;
        }

        // we run the UPD on the patch on click
        runUPDpatch( );

        // classification
        std::vector< std::vector<double> > res = SVMpatchClassification();

        std::cout << " ciao " << std::endl;
        if (res.at(0).at(0) == 1)
        {
            next_point.r = 0;
            next_point.g = 255; //((res.at(0).at(1)-0.5)*2)*255;
            next_point.b = 0;
            m_cloud_classifiedGNG->push_back(next_point);
        }
        if (res.at(0).at(0) == -1)
        {
            next_point.r = 255;//((res.at(0).at(2)-0.5)*2)*255;
            next_point.g = 0;
            next_point.b = 0;
            m_cloud_classifiedGNG->push_back(next_point);
        }

    }

    viewer->removeAllPointClouds();
//    viewer->removePointCloud();
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_color(m_cloud_classifiedGNG);
    viewer->addPointCloud (m_cloud_classifiedGNG, rgb_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->lcdNumber_p->value(), "cloud");

    ui->qvtkWidget->update ();




}


/*void PCL_upd_DEMO::SVMclassifyCloud_randomPoints()
{

    //run the classification for all the patches in the cloud

    //safety check if have the training ect
    unsigned int num_of_samples = 10;
    for (unsigned int i = 0; i<num_of_samples; i++)
    {
        //pick a random point
        PointT random_point;
        //if(!m_cloud->isOrganized())
        //{
            int rand_idx = rand() % m_cloud->size();
            random_point = m_cloud->at(rand_idx); // not possible to use 2D indexing
            std::cout << " the random point is " << random_point << std::endl;
        //}
        //else
        //{
        //    random_point = m_cloud->at(0,0);//if the cloud is organized than it is possible to use 2D indexing
        //}




        //extract the patch
        double patch_size = 0.0;
        bool isNumeric;
        patch_size = ui->lineEdit_patchSize->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "Patch size is not a valid number - The patch cannot be extracted !");
            return;
        };
        extractPatch(patch_size, random_point.x, random_point.y, random_point.z);

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

        sleep(1);

        // classification
        SVMclassification();
    }
}

*/
