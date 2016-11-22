
#include <iostream>
#include <ostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/common_headers.h>

#include <pcl/ml/svm_wrapper.h>


//////// OUTDATED
int outdated(int argc, char** argv)
{
    return 0;

    // this is the old outdated function
    {
        // what we need to configure the SVM classifier
        pcl::SVMData my_svm_dataA;  //--> dataset A
        pcl::SVMData my_svm_dataB;  //--> dataset B
        pcl::SVMData my_svm_dataC;  //--> dataset C
        pcl::SVMData my_svm_dataD;  //--> dataset D
        pcl::SVMData my_svm_dataE;
        pcl::SVMDataPoint my_svm_data_point; //--> a data point just for simplicity
        pcl::SVMParam my_svm_parameters; //--> our own configuration parameters
        pcl::SVMModel my_svm_model;   //--> classifier model, this is automatically generated after the training

        pcl::SVMTrain my_svm_trainer; //--> our trainer, to be used for store training data or for a new training procedure
        pcl::SVMClassify my_svm_classifier;  //--> our classifier,

        // generate randomly sampled datasets, taking care to be separated each other
        my_svm_dataA.label =  +1.0;  // assign two different labels to the dataset
        my_svm_dataB.label =  +1.0;
        my_svm_dataC.label =  -1.0;
        my_svm_dataD.label =  -1.0;
    //    my_svm_dataE.label =  +1.0;

        // create my training set based on the data I have created
        std::vector<pcl::SVMData> my_training_set;
        std::vector<pcl::SVMData> my_test_set;


        std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
        std::cout.precision(3);
        std::cout << " +---------------------------------------------------------------------------+ \n"
                  << " |                  Generated point number in the dataset                    | \n"
                  << " +---------------------------------------------------------------------------+ \n"
                  << " |   \t  Label   \t\t Feature   \t\t   value             | \n"
                  << " +---------------------------------------------------------------------------+ \n";

        for (size_t i = 0; i < 20; i++)
        {
            //i<10?my_svm_data_point.idx = 1:my_svm_data_point.idx = 0;
            // the dataset A is composed of 10 random points between 0 and 1
            if (i<10){
                my_svm_data_point.idx = 1;
                my_svm_data_point.value = 0.2*(1.024 * rand () / (RAND_MAX + 1.0f))+0.5;
                my_svm_dataA.SV.push_back( my_svm_data_point);
                my_training_set.push_back(my_svm_dataA);

                my_svm_data_point.idx = 1;
                my_svm_data_point.value = 0.2*(1.024 * rand () / (RAND_MAX + 1.0f))+0.5;
                my_svm_dataB.SV.push_back( my_svm_data_point );
                my_training_set.push_back(my_svm_dataB);
                std::cout << " |   \t  " << my_svm_dataA.label << " "
                          << "     \t \t  " << my_svm_dataA.SV.back().idx  << " "
                          << "     \t \t  " << my_svm_dataA.SV.back().value << "             |" << std::endl;
                std::cout << " |   \t  " << my_svm_dataB.label << " "
                          << "     \t \t  " << my_svm_dataB.SV.back().idx  << " "
                          << "     \t \t  " << my_svm_dataB.SV.back().value << "             |" << std::endl;
                my_svm_dataA.SV.clear( );
                my_svm_dataB.SV.clear( );
            }
            // the dataset B is composed of 10 random points between 5 and 6
            else
            {
                my_svm_data_point.idx = 1;
                my_svm_data_point.value = 0.2*(1.024 * rand () / (RAND_MAX + 1.0f));   // the +5 takes care of the separation
                my_svm_dataC.SV.push_back( my_svm_data_point );
                my_training_set.push_back(my_svm_dataC);

                my_svm_data_point.idx = 1;
                my_svm_data_point.value = 0.2*(1.024 * rand () / (RAND_MAX + 1.0f));   // the +5 takes care of the separation
                my_svm_dataD.SV.push_back( my_svm_data_point );
                my_training_set.push_back(my_svm_dataD);

                std::cout << " |   \t  " << my_svm_dataC.label  << " "
                          << "     \t \t  "  << my_svm_dataC.SV.back().idx << " "
                          << "     \t \t  " << my_svm_dataC.SV.back().value << "             |" << std::endl;
                std::cout << " |   \t  " << my_svm_dataD.label  << " "
                          << "     \t \t  "  << my_svm_dataD.SV.back().idx << " "
                          << "     \t \t  " << my_svm_dataD.SV.back().value << "             |" << std::endl;
                my_svm_dataC.SV.clear( );
                my_svm_dataD.SV.clear( );
            }

        } // end - generate the random datasets
        std::cout << " +---------------------------------------------------------------------------+ \n";

        for (size_t i = 0; i < 20; i++) // generate the random dataset E
        {
            if (i<10){
                my_svm_data_point.idx = 1;
                my_svm_data_point.value = 1.024 * rand () / (RAND_MAX + 1.0f);
                my_svm_dataE.SV.push_back( my_svm_data_point );
                my_test_set.push_back(my_svm_dataE);

                std::cout << " |   \t  " << my_svm_dataE.label << " "   // the dataset C does not have any label
                          << "     \t \t  " << my_svm_dataE.SV.back().idx  << " "
                          << "     \t \t  " << my_svm_dataE.SV.back().value << "             |" << std::endl;
                my_svm_dataE.SV.clear( );

            }
            else
            {
                my_svm_data_point.idx = 1;
                my_svm_data_point.value = 1.024 * rand () / (RAND_MAX + 1.0f);
                my_svm_dataE.SV.push_back( my_svm_data_point );
                my_test_set.push_back(my_svm_dataE);

                std::cout << " |   \t  " << my_svm_dataE.label << " "   // the dataset C does not have any label
                          << "     \t \t  " << my_svm_dataE.SV.back().idx  << " "
                          << "     \t \t  " << my_svm_dataE.SV.back().value << "             |" << std::endl;
                my_svm_dataE.SV.clear( );
            }
        } // end - generate the random datasets E
        std::cout << " +---------------------------------------------------------------------------+ \n";

        // Configuration of the SVM parameters --
        // The constructor define all defaults parameters
        // Here we configure a few just as example
        my_svm_parameters.kernel_type = RBF;
        my_svm_parameters.shrinking = 1;
        my_svm_parameters.gamma = 0.5;
        my_svm_parameters.C = 0.9;
        my_svm_parameters.probability = 1;

        my_svm_trainer.setParameters(my_svm_parameters);



        // I give as input the data from the trainer
        my_svm_trainer.setInputTrainingSet(my_training_set);

        //if (my_svm_trainer.loadProblem("/home/mauro_veas/code/OTA/OpenTraversability_build_1.8/bin/example1/train.dat"))
        //    std::cout << " |                     The problem has been loaded                      |" << std::endl;
        //else
        //    std::cout << " |                    The problem has NOT been loaded                   |" << std::endl;
        //my_svm_trainer.adaptProbToInput();
        my_training_set = my_svm_trainer.getInputTrainingSet();
        std::cout << " training set size = " << my_training_set.size() << std::endl;


        my_svm_trainer.saveTrainingSet("./training_set.dat");

        // train the classifier
        bool svm_trained = my_svm_trainer.trainClassifier();

        if (svm_trained )
            std::cout << " |                     The classifier has been tranined                      |" << std::endl;
        else
            std::cout << " |                    The classifier has NOT been tranined                   |" << std::endl;

        std::cout << " +---------------------------------------------------------------------------+ \n";

        // check the model for the classifier
        my_svm_model = my_svm_trainer.getClassifierModel();
        std::cout << " |   \t  l   \t\t\t probA   \t\t   probB             | \n";
        std::cout << " +---------------------------------------------------------------------------+ \n";
        std::cout << " |   \t  " << my_svm_model.l  << " "
                  << "     \t \t  "  << *my_svm_model.probA << " "
                  << "     \t \t  " << *my_svm_model.probB << "                |" << std::endl;
        std::cout << " +---------------------------------------------------------------------------+ \n";



        //my_svm_trainer.resetTrainingSet();
        //my_svm_trainer.setInputTrainingSet(my_test_set);
        //my_svm_trainer.saveTrainingSet("./verificationDataset.dat");




        my_svm_trainer.saveClassifierModel("./model.dat");

        //if (my_svm_classifier.loadClassifierModel("/home/mauro_veas/code/OTA/OpenTraversability_build_1.8/bin/example1/model"))//("./example1/model"))
        //    std::cout << " Model successfully loaded " << std::endl;
        //else
        //    std::cout << " Model NOT loaded " << std::endl;


        // start the classification
        //my_svm_classifier.setInputTrainingSet(my_training_set);
        my_svm_classifier.setClassifierModel(my_svm_model);   // set as input the model we have just generated
        my_svm_classifier.setProbabilityEstimates(my_svm_parameters.probability);

        my_svm_classifier.loadClassProblem("./verificationDataset.dat");

        //std::vector<double> classification_result;
        std::vector< std::vector<double> > classification_result;
        //classification_result = my_svm_classifier.classification(my_svm_dataE);
        if ( my_svm_classifier.classificationTest( ) )
            std::cout << " CLASSIFICATION SUCCESS ! " << std::endl;
        else
            std::cout << " CLASSIFICATION not SUCCESS " << std::endl;

        my_svm_classifier.getClassificationResult(classification_result);
        std::cout << " Classification result size = " << classification_result.size();// << " value = " << classification_result.at(0) << std::endl;
        int number_of_success = 0;
        int number_of_not_success = 0;
        int number_of_errors = 0;
        //std::cout << " Classification result size = " << classification_result.size() << " value = " << classification_result.at(0) << std::endl;

    /*    for (size_t i = 0; i < 20; i++)
        {
            my_svm_data_point.idx = -1;
            my_svm_data_point.value = (1.024 * rand () / (RAND_MAX + 1.0f));
            my_svm_dataE.SV.push_back(my_svm_data_point);
            classification_result = my_svm_classifier.classification(my_svm_dataE);
            std::cout << " |   \t  " << my_svm_dataE.label << " "   // the dataset C does not have any label
                      << "     \t \t  " << my_svm_dataE.SV.back().idx  << " "
                      << "     \t \t  " << my_svm_dataE.SV.back().value << "             |" << std::endl;
            std::cout << " result = " << classification_result.at(0) << " on data "  << my_svm_dataE.SV.at(0).value << std::endl;
            my_svm_dataE.SV.clear ();

        }*/
    my_svm_classifier.classification();
        // classifier verification
        bool classifier_verification = my_svm_classifier.classificationTest ();
        std::cout << "classification test returned " << classification_result.size() << std::endl;
        std::vector< std::vector<double> > percentage;
        if (classifier_verification)
        {
          my_svm_classifier.getClassificationResult(percentage);
        }
        std::cout << " percentage size = " << percentage.size() << std::endl;

        for (size_t i = 0; i < classification_result.size(); i++) {
         for (size_t j = 0; j < classification_result.at(i).size(); j++) {
         //std::cout << classification_result1.at(i).at(j) << " \n";
            if ( classification_result.at(i).at(j) == 1 ) {
                  std::cout << " SUCCESS -- > Classification result at i = " << i << " and j = " << j << " is " << classification_result.at(i).at(j) << std::endl;
                number_of_success++;
              }
            else {
              if ( classification_result.at(i).at(j) == -1) {
                   std::cout << " NOT SUCCESS -- > Classification result at i = " << i << " and j = " << j << " is " << classification_result.at(i).at(j) << std::endl;
                 number_of_not_success++; }
              else {
                 number_of_errors++;
                   std::cout << " Classification result at i = " << i << " and j = " << j << " is " << classification_result.at(i).at(j) << std::endl;
               }
              }
         }
        }
        std::cout << " |                                                                           | \n";
        std::cout << " |    \t  number of success = \t " << number_of_success << " \t     \t          | \n";
        std::cout << " |    \t  number of not success = \t" << number_of_not_success << "\t     \t         | \n";
        std::cout << " |    \t  number of errors = \t " << number_of_errors << " \t           \t       | \n";
        std::cout << " |                                                                           | \n";
        std::cout << " +---------------------------------------------------------------------------+ \n";


        if (pcl::console::find_argument (argc, argv, "-v") >= 0)
       {

       return 0;
       }
        if (pcl::console::find_argument (argc, argv, "-b") >= 0)
       {
         return 0;
       }
        if (pcl::console::find_argument (argc, argv, "") <= 0)
       {
         std::cout<<"\nArgument not recognized ... please refer to this help for application usage\n\n"<<std::endl;
         std::cout<<"-b ---> benchmark \n"<<std::endl;
         std::cout<<"-v ---> write me \n"<<std::endl;
        return 0;
       }
        return (0);
    }

}
//////// OUTDATED END



// --------------------------------------------------------------------
// -----Create our own point type with feature and classification -----
// --------------------------------------------------------------------

struct PointClassifiable
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int idx;        // It's the feature index. It has to be an integer number greater or equal to zero.
  float feature;  // The value assigned to the correspondent feature.
  float label;    // The label value. It is a mandatory to train the classifier.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointClassifiable,           // here we assume a XYZ + "idx" + "feature"
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, idx, idx)
                                  (float, feature, feature)
                                  (float, label, label)
)

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\n"
            << " +---------------------------------------------------------------------------+ \n"
            << " |                     pcl SVM training example help                         | \n"
            << " +---------------------------------------------------------------------------+ \n"
            << " |   \t  Description:  Example of the SVM usage within the                  | \n"
            << " |                      point cloud libray                                   | \n"
            << " |                                                                           | \n"
            << " +---------------------------------------------------------------------------+ \n"
            << " |   \t  Usage: "<<progName<<" [options]                          | \n"
            << " |                                                                           | \n"
            << " +---------------------------------------------------------------------------+ \n"
            << " |   \t  Options:                                                           | \n"
            << " |                                                                           | \n"
            << " |       -h / --help       --->   visualize this help                        | \n"
            << " |       -t / --try        --->   just an initial try to generate a new pcd  | \n"
            << " |                                                                           | \n"
            << " +---------------------------------------------------------------------------+ \n" << std::endl;

}

/** Transform a classifiable point cloud in SVMdata to train a classifier
  * \return true if success
  */
bool fromClassifiablePDCtoSVMdata (pcl::PointCloud<PointClassifiable> &_c_cloud, pcl::SVMData &_svm_data, float label)
{
/// TODO: check for cloud validy, empty etc. ---> return false

    if (_c_cloud.size()<1) {
        std::cout << " fromClassifiablePDCtoSVMdata ERROR : no data in the point cloud";
        return false;}

    pcl::SVMDataPoint svm_data_point; //--> a data point just for simplicity
    _svm_data.label = label;

    for (size_t i=0; i<_c_cloud.size(); i++)
    {
        svm_data_point.idx = _c_cloud.at(i).idx;
        svm_data_point.value = _c_cloud.at(i).feature;
        _svm_data.SV.push_back(svm_data_point);
    }

}



/** Generate a sample of squared surface patch
  * \return a point cloud
  */
pcl::PointCloud<pcl::PointXYZ> generateSampleSurfaceSquaredCloud(float surface_size, float surface_density)
{
  if (surface_size == 0) surface_size = 0.5f; // default value

  if (surface_density == 0) surface_density = 0.1f; // default value

  pcl::PointCloud<pcl::PointXYZ>::Ptr surface (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point (new pcl::PointCloud<pcl::PointXYZ>);
    point->resize(1*1);
    for (float x= -surface_size; x<=surface_size; x+=surface_density)
    {
      for (float y= -surface_size; y<=surface_size; y+=surface_density)
      {

        point->points[0].x = -1;
        point->points[0].y = y;
        point->points[0].z = x;   // the reference system has z pointing forward, y downward and x leftward

        *surface+=*point;
        //std::cout << " Generated point " << point->points[0] << std::endl;
      }
    }
  return *surface;
}

/** Generate a sample of curved surface patch
  * \return a point cloud
  */
pcl::PointCloud<pcl::PointXYZ> generateSampleSurfaceCurvedCloud(float surface_size, float surface_density)
{
  if (surface_size == 0) surface_size = 0.5f; // default value

  if (surface_density == 0) surface_density = 0.1f; // default value

  pcl::PointCloud<pcl::PointXYZ>::Ptr surface (new pcl::PointCloud<pcl::PointXYZ>);

    for (float z(-surface_size); z <= surface_size; z += surface_density)
    {
      for (float angle(0.0); angle <= 180.0; angle += 1.0)
      {
        pcl::PointXYZ point;
        point.x = 0.5 * cosf (pcl::deg2rad(angle));
        point.y = sinf (pcl::deg2rad(angle));
        point.z = z;
//        std::cout << " Generated point " << point << std::endl;
        surface->points.push_back (point);
      }
    }
    surface->resize(surface->size());  //ensure to have right size = height x width
  return *surface;
}

int main (int argc, char** argv)
{
  // first of all we need a new point type which includes classification fields
    pcl::PointCloud<PointClassifiable> c_cloud_positive;
    pcl::PointCloud<PointClassifiable> c_cloud_negative;

    pcl::SVMData c_cloud_positive_svm_data;  //--> extract SMVdata for the classifier
    pcl::SVMData c_cloud_negative_svm_data;  //--> extract SMVdata for the classifier


  std::cout << "\n\n"
            << " +---------------------------------------------------------------------------+ \n"
            << " |                       pcl SVM PCD example                                 | \n"
            << " +---------------------------------------------------------------------------+ \n";

   // --------------------------------------
   // -----Parse Command Line Arguments-----
   // --------------------------------------
   if (pcl::console::find_argument (argc, argv, "-h") >= 0 ||
        pcl::console::find_argument (argc, argv, "--help") >= 0)   // the help
   {
    printUsage (argv[0]);
    return 0;
   }
   if (pcl::console::find_argument (argc, argv, "-t") >= 0 ||
        pcl::console::find_argument (argc, argv, "--try") >= 0)   // the help
   {
    // here we actually do something

       pcl::PointCloud<pcl::PointXYZ>::Ptr planarSurfacePatch (new pcl::PointCloud<pcl::PointXYZ>);
       *planarSurfacePatch = generateSampleSurfaceSquaredCloud(0.5f, 0.1f);
       pcl::io::savePCDFile ("planarSurfacePatch.pcd", *planarSurfacePatch);
       std::cout << "Generated and saved a sample planar surface cloud with size "<< planarSurfacePatch->size() << std::endl;

       // I want to classify the generated patch as a positive sample
       c_cloud_positive.points.resize (planarSurfacePatch->size());
       c_cloud_positive.width = planarSurfacePatch->width;
       c_cloud_positive.height = planarSurfacePatch->height;
       for (size_t i=0; i<planarSurfacePatch->size(); i++)
       {
           c_cloud_positive.points[i].x = planarSurfacePatch->points[i].x;
           c_cloud_positive.points[i].y = planarSurfacePatch->points[i].y;
           c_cloud_positive.points[i].z = planarSurfacePatch->points[i].z;
           c_cloud_positive.points[i].idx = 1;
           c_cloud_positive.points[i].feature = 1.0;  // any feature will be here ok to be used in the classifier
           c_cloud_positive.points[i].label = 1.0;
       }
       pcl::io::savePCDFile ("c_cloud_positive.pcd", c_cloud_positive);

       pcl::PointCloud<pcl::PointXYZ>::Ptr curvedSurfacePatch (new pcl::PointCloud<pcl::PointXYZ>);
       *curvedSurfacePatch = generateSampleSurfaceCurvedCloud(0.5f, 0.1f);
       pcl::io::savePCDFile ("curvedSurfacePatch.pcd", *curvedSurfacePatch);
       std::cout << "Generated and saved a sample curved surface cloud with size "<< curvedSurfacePatch->size() << std::endl;

       // Now, I want to classify the generated patch as a negative sample
       c_cloud_negative.points.resize (curvedSurfacePatch->size());
       c_cloud_negative.width = curvedSurfacePatch->width;
       c_cloud_negative.height = curvedSurfacePatch->height;
       for (size_t i=0; i<curvedSurfacePatch->size(); i++)
       {
           c_cloud_negative.points[i].x = curvedSurfacePatch->points[i].x;
           c_cloud_negative.points[i].y = curvedSurfacePatch->points[i].y;
           c_cloud_negative.points[i].z = curvedSurfacePatch->points[i].z;
           c_cloud_negative.points[i].idx = 1;
           c_cloud_negative.points[i].feature = 0.5;  // any feature will be here ok to be used in the classifier
           c_cloud_negative.points[i].label = -1.0;
       }
       pcl::io::savePCDFile ("c_cloud_negative.pcd", c_cloud_negative);


       // supposing that we have now loaded a classified point cloud,
       // extract the SVM data
       fromClassifiablePDCtoSVMdata(c_cloud_positive, c_cloud_positive_svm_data, 1.0);
       std::cout << "Extracted positive svm data with size :  " << c_cloud_positive_svm_data.SV.size() << std::endl;

       fromClassifiablePDCtoSVMdata(c_cloud_negative, c_cloud_negative_svm_data, -1.0);
       std::cout << "Extracted negative svm data with size :  " << c_cloud_negative_svm_data.SV.size() << std::endl;



       pcl::SVMTrain my_svm_trainer; //--> our trainer, to be used for store training data or for a new training procedure
       pcl::SVMClassify my_svm_classifier;  //--> our classifier
       std::vector<pcl::SVMData> my_training_set;   //--> the training set is a vector of data
       pcl::SVMModel my_svm_model;   //--> classifier model, this is automatically generated after the training or loaded for the classification
       pcl::SVMParam my_svm_parameters; //--> our own configuration parameters


       // add data into the training set
       my_training_set.push_back(c_cloud_positive_svm_data);
       my_training_set.push_back(c_cloud_negative_svm_data);

       // configure some useful parameters
       // TODO: let the user configure this parameters from the command line !
       my_svm_parameters.kernel_type = RBF;
       my_svm_parameters.shrinking = 1;
       my_svm_parameters.gamma = 0.0005;
       my_svm_parameters.C = 10;
       my_svm_parameters.probability = 0;
       my_svm_trainer.setParameters(my_svm_parameters);  // set the parameters for the trainer

       my_svm_trainer.setInputTrainingSet(my_training_set);


       // train the classifier
       if (my_svm_trainer.trainClassifier() )
          {
            std::cout << "\t The classifier has been successfully tranined \n\n";
          }
       else
          {
            std::cout << "\t The classifier has NOT been tranined  - Exit now ! \n\n";
            return 0;
          }

       // check the model for the classifier
       my_svm_model = my_svm_trainer.getClassifierModel();



       // check the model for the classifier
       my_svm_model = my_svm_trainer.getClassifierModel();

       std::cout << "\t Model parameters summary : \n";
       if ((my_svm_parameters.probability?true:false)){
           std::cout << "\t\t  Probability support \t active  \n";
           std::cout << "\t\t  ProbA = \t" << *my_svm_model.probA << " \n"
                     << "\t\t  ProbB = \t" << *my_svm_model.probB << " \n";
       }
       else {
        std::cout << "\t\t  Probability support \t NOT active  \n";
       }
       std::cout << "\t\t  l  \t \t \t " << my_svm_model.l  << " \n";
       std::cout << "\t\t  Number of classes   \t " << my_svm_model.nr_class << " \n";
       std::cout << "\t\t  sv_coef   \t  \t " <<  *(*my_svm_model.sv_coef) << " \n";
       std::cout << "\t\t  Rho   \t  \t " <<  *my_svm_model.rho << " \n";
       std::cout << "\t\t  label   \t  \t " <<  *my_svm_model.label << " \n";
       std::cout << "\t\t  nSV \t   \t  \t " <<  *my_svm_model.nSV << " \n\n";





    return 0;
   }
   if (pcl::console::find_argument (argc, argv, "") <= 0)
   {
    printUsage (argv[0]);
    return 0;
   }

 return (0);
}

