/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */



#include "PCL_upd_DEMO.h"






void
PCL_upd_DEMO::addCNNdataToTrainingSet(std::vector<float> _features, uint32_t _label )
{

    if (_features.size() < 1 )
    {
        std::cout << "PCL_upd_DEMO::addCNNdataToTrainingSet  << error >> : no features " << std::endl;
        return;
    }

     tiny_dnn_data_point cnn_data_point; //--> a data point just for simplicity

    cnn_data_point._label = _label;

    for (int i=0; i<_features.size(); i++)  {
        cnn_data_point._vec.push_back(_features.at(i));
    }

    m_cnn_training_set._data.push_back(cnn_data_point);

}

void
PCL_upd_DEMO::adaptTrainingSetFromSVMdataset()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

    if (m_svm_training_set.size()<1)
    {
        //no data
            std::cout << " PCL_upd_DEMO::adaptTrainingSetFromSVMdataset <<error>> No data " << std::endl;
        return;
    }
    ///TODO : this cannot be done in this way
    //m_cnn_training_set._data.clear();

    std::vector<float> data_point; //--> a data point just for simplicity
    for (int i = 0; i< m_svm_training_set.size(); i++)
    {
        data_point.clear();
        double label = m_svm_training_set.at(i).label;
        uint32_t labelInt;
        // if label = -1.0 ==> not ground --- if label = 1.0 ==> ground
        if (label == -1.0)
        {//not ground
            labelInt = 0;
        }
        else {
            if (label == 1.0)
            {// ground
                labelInt = 1;
            }
            else {
                            std::cout << " PCL_upd_DEMO::adaptTrainingSetFromSVMdataset <<error>> label not recognized " << std::endl;
            }
        }
        std::vector<pcl::SVMDataPoint> point = m_svm_training_set.at(i).SV;
        for (int j=0; j<point.size(); j++)
        {
            data_point.push_back(point.at(j).value);
        }
        addCNNdataToTrainingSet(data_point, labelInt);
    }





    //m_training_set = m_svm_classifier.getInputTrainingSet();

    QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode

    std::cout << " dataset loaded " << m_svm_training_set.size() << std::endl;



}

void
PCL_upd_DEMO::initCNN()
{
    unsigned int n_input = m_cnn_training_set._data.at(0)._vec.size();     // number of inputs for the neural network

    int neurons = 0;
    bool isNumeric;
    neurons = ui->lineEdit_cnnNeurons->text().toInt(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "the number of neurons is not a valid number !");
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }

    unsigned int n_input_l1 = neurons; // number of inputs for the first layer
    unsigned int n_input_l2 = neurons; // number of inputs for the second layer
    unsigned int n_input_l3 = neurons; // number of inputs for the third layer
    unsigned int n_input_l4 = neurons; // number of inputs for the fourth layer
    unsigned int n_output = 2;    // number of outputs  --- this depends of the number of classes we have

    std::vector<tiny_dnn::cnn_size_t> net_conf ({ n_input,
                                                  n_input_l1,
                                                  n_input_l2,
                                                  n_input_l3,
                                                  n_input_l4,
                                                  n_output });
   //{ // just print some net configurations
   //  for (int i = 0; i < net_conf.size(); i++) {
   //      std::cout << "PCL_upd_DEMO::trainCNN  << message >> : net_conf " << net_conf.at(i)
   //                << std::endl;
   //   }
   //  std::cout << "PCL_upd_DEMO::trainCNN  << message >> : net_conf.begin " << *net_conf.begin()
   //            << std::endl;
   //  std::cout << "PCL_upd_DEMO::trainCNN  << message >> : net_conf.end " << *net_conf.end()
   //            << std::endl;
   //}




    // set activation function type
    int cnn_activation_type = 0;    // tanh, sigmoid, softmax, rectified linear(relu), leaky relu, identity, exponential linear units(elu)
    cnn_activation_type = ui->comboBox_activationFunction->currentIndex();

    // create a full connected networks
    switch (cnn_activation_type) {
            case 0: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::tan_h>(net_conf.begin(), net_conf.end());
                    break;
            case 1: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::sigmoid>(net_conf.begin(), net_conf.end());
                    break;
            case 2: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::softmax>(net_conf.begin(), net_conf.end());
                    break;
            case 3: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::relu>(net_conf.begin(), net_conf.end());
                    break;
            case 4: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::leaky_relu>(net_conf.begin(), net_conf.end());
                    break;
            case 5: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::identity>(net_conf.begin(), net_conf.end());
                    break;
            case 6: m_mlp = tiny_dnn::make_mlp<tiny_dnn::activation::identity>(net_conf.begin(), net_conf.end());
                    break;

            default: std::cout << "PCL_upd_DEMO::trainCNN  << message >> not recognized activation function " << std::endl;
        }






}

void
PCL_upd_DEMO::trainCNN()
{

 if (m_cnn_training_set._data.size() < 1)
 {
     std::cout << "PCL_upd_DEMO::trainCNN  << error >> : no data in the trainin set "
               << std::endl;
     return;
 }

   QApplication::setOverrideCursor(Qt::WaitCursor);    //transform the cursor for waiting mode

 std::cout << "PCL_upd_DEMO::trainCNN  << error >> : m_cnn_training_set._data.at(0)._vec.size() " << m_cnn_training_set._data.at(0)._vec.size()
           << std::endl;



  initCNN();


 std::vector<tiny_dnn::vec_t> data = m_cnn_training_set.dataVectors_t();
 std::vector<tiny_dnn::label_t> labels = m_cnn_training_set.dataLabels_t();

 if (data.size()<1 || labels.size()<1)
 {
  std::cout << " PCL_upd_DEMO::trainCNN  << error >> : no data to train !  " << std::endl;
     return;
 }

 std::cout << "PCL_upd_DEMO::trainCNN  << message >> : data.size() = " << data.size()
           << " labels.size () = " << labels.size()
           << std::endl;


// print data in input
// for (int i = 0; i < data.size(); i++) {
//
//     std::cout << "PCL_upd_DEMO::trainCNN  << message >> : data = \n\n "
//               << data.at(i).at(0) << "  " << data.at(i).at(1) << "  " << data.at(i).at(2)
//               << std::endl;
// }
// for (int i = 0; i < labels.size(); i++) {
// std::cout << "PCL_upd_DEMO::trainCNN  << message >> : labels = \n\n " << labels.at(i)
//           << std::endl;
//}

 // tiny_dnn::adagrad optimizermlp;
getCNNoptimizationParams( );
int value = ui->comboBox_optimizationAlgorithms->currentIndex();
optimizationChanged(value);

m_mlp.init_weight();

// I hate mydself doing this !!!
int lossValue = ui->comboBox_lossFunction->currentIndex();
switch (lossValue) {
case 0:
{

    {
        switch (value) {
        case 0: //adagrad
        {
            tiny_dnn::adagrad opt;
            opt.alpha = m_alpha_adagrad;
            m_mlp.train<tiny_dnn::cross_entropy>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
         }
        case 1: //RMSprop
        {
            tiny_dnn::RMSprop opt;
            opt.alpha = m_alpha_RMSprop;
            opt.mu = m_mu_RMSprop;
            m_mlp.train<tiny_dnn::cross_entropy>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 2: //adam
        {
            tiny_dnn::adam opt;
            opt.alpha = m_alpha_adam;
            opt.b1 = m_b1_adam;
            opt.b2 = m_b2_adam;
            opt.b1_t = m_b1_t_adam;
            opt.b2_t = m_b2_t_adam;
            m_mlp.train<tiny_dnn::cross_entropy>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 3: //gradient descend SGD
        {
            tiny_dnn::gradient_descent opt;
            opt.alpha = m_alpha_sgd;
            opt.lambda = m_lambda_sgd;
            m_mlp.train<tiny_dnn::cross_entropy>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 4: //gradient descend with momentum
        {
            tiny_dnn::momentum opt;
            opt.alpha = m_alpha_sgdm;
            opt.lambda = m_lambda_sgdm;
            opt.mu = m_mu_sgdm;
            m_mlp.train<tiny_dnn::cross_entropy>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        default:
            std::cout << "PCL_upd_DEMO::optimizationChanged  << message >> not recognized optimization function "
                      << std::endl;
        }
    }

    break;
}
case 1:
{
    {
        switch (value) {
        case 0: //adagrad
        {
            tiny_dnn::adagrad opt;
            opt.alpha = m_alpha_adagrad;
            m_mlp.train<tiny_dnn::mse>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
         }
        case 1: //RMSprop
        {
            tiny_dnn::RMSprop opt;
            opt.alpha = m_alpha_RMSprop;
            opt.mu = m_mu_RMSprop;
            m_mlp.train<tiny_dnn::mse>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 2: //adam
        {
            tiny_dnn::adam opt;
            opt.alpha = m_alpha_adam;
            opt.b1 = m_b1_adam;
            opt.b2 = m_b2_adam;
            opt.b1_t = m_b1_t_adam;
            opt.b2_t = m_b2_t_adam;
            m_mlp.train<tiny_dnn::mse>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 3: //gradient descend SGD
        {
            tiny_dnn::gradient_descent opt;
            opt.alpha = m_alpha_sgd;
            opt.lambda = m_lambda_sgd;
            m_mlp.train<tiny_dnn::mse>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 4: //gradient descend with momentum
        {
            tiny_dnn::momentum opt;
            opt.alpha = m_alpha_sgdm;
            opt.lambda = m_lambda_sgdm;
            opt.mu = m_mu_sgdm;
            m_mlp.train<tiny_dnn::mse>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        default:
            std::cout << "PCL_upd_DEMO::optimizationChanged  << message >> not recognized optimization function "
                      << std::endl;
        }
    }
    break;
}
case 2:
{

    {
        switch (value) {
        case 0: //adagrad
        {
            tiny_dnn::adagrad opt;
            opt.alpha = m_alpha_adagrad;
            m_mlp.train<tiny_dnn::absolute>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
         }
        case 1: //RMSprop
        {
            tiny_dnn::RMSprop opt;
            opt.alpha = m_alpha_RMSprop;
            opt.mu = m_mu_RMSprop;
            m_mlp.train<tiny_dnn::absolute>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 2: //adam
        {
            tiny_dnn::adam opt;
            opt.alpha = m_alpha_adam;
            opt.b1 = m_b1_adam;
            opt.b2 = m_b2_adam;
            opt.b1_t = m_b1_t_adam;
            opt.b2_t = m_b2_t_adam;
            m_mlp.train<tiny_dnn::absolute>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 3: //gradient descend SGD
        {
            tiny_dnn::gradient_descent opt;
            opt.alpha = m_alpha_sgd;
            opt.lambda = m_lambda_sgd;
            m_mlp.train<tiny_dnn::absolute>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        case 4: //gradient descend with momentum
        {
            tiny_dnn::momentum opt;
            opt.alpha = m_alpha_sgdm;
            opt.lambda = m_lambda_sgdm;
            opt.mu = m_mu_sgdm;
            m_mlp.train<tiny_dnn::absolute>(opt, data, labels, m_cnn_batch_size, m_cnn_epoch);
            break;
        }
        default:
            std::cout << "PCL_upd_DEMO::optimizationChanged  << message >> not recognized optimization function "
                      << std::endl;
        }
    }

    break;
}
case 3:
{   std::cout << "PCL_upd_DEMO::optimizationChanged  << message >> not supported yet "
              << std::endl;
    break;
}

default: typedef  tiny_dnn::cross_entropy lossFunction;
    break;
}









 std::cout << " PCL_upd_DEMO::trainCNN  << message >> : trained !  " << std::endl;

 for (int i = 0; i < m_mlp.depth(); i++) {
     std::cout << "#layer:" << i << "\n";
     std::cout << "layer type:" << m_mlp[i]->layer_type() << "\n";
     std::cout << "input:" << m_mlp[i]->in_size() << "(" << m_mlp[i]->in_shape() << ")\n";
     std::cout << "output:" << m_mlp[i]->out_size() << "(" << m_mlp[i]->out_shape() << ")\n";
     std::vector<tiny_dnn::vec_t*> nn_weights = m_mlp[i]->weights();

     std::cout << "# weights at layer:" << i << "\n";
     for (int j = 0; j< nn_weights.size(); j++) {
         for (int k = 0; k< nn_weights.at(j)->size(); k++)
         {
         std::cout << " j = " << j << " k = " << k << " weight " << nn_weights.at(j)->at(k) << "\n";
         }
     }

  QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
 }




 // perform a test on the training data
 m_mlp.test(data, labels).print_detail(std::cout);


 m_mlp.save("my_network.txt");

}

void
PCL_upd_DEMO::CNNclassification()
{

    tiny_dnn_data_point cnn_data_point; //--> a data point just for simplicity
    tiny_dnn_dataset cnn_data;

    if (UPD_cloud->empty() || UPD_cloud->size() < m_patch_labelling_index) {
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        QMessageBox::warning(this, "Warning !", "UPD cloud cannot be labelled  ! " );
        return;
    }


    // create features
    std::vector<float> features = createAllFeatures();

    for (int i=0; i<features.size(); i++)
    {
        cnn_data_point._vec.push_back(features.at(i));
    }
    cnn_data_point._label = 1.0;
    cnn_data._data.push_back(cnn_data_point);

    std::vector<tiny_dnn::vec_t> data = cnn_data.dataVectors_t();
    std::vector<tiny_dnn::label_t> labels = cnn_data.dataLabels_t();

    if (data.size()<1 || labels.size()<1)
    {
     std::cout << " PCL_upd_DEMO::classCNN  << error >> : no data to test the classification !  " << std::endl;
        return;
    }


    std::vector<tiny_dnn::vec_t> prediction = m_mlp.predict(data);
    //std::vector<tiny_dnn::label_t> prediction_label =
    //const auto prediction_label =
    const tiny_dnn::label_t pl = m_mlp.predict_label(data.at(0));

    std::cout << " PCL_upd_DEMO::classCNN  << message >> : predicted label is :  " << pl
          << std::endl;

    for (int i=0; i<prediction.size();i++) {
    for (int j=0; j<prediction.at(i).size();j++) {
        std::cout << " PCL_upd_DEMO::classCNN  << message >> : the prediction is :  " << prediction.at(i).at(j)
              << " at i = " << i << " and j = " << j
              << std::endl;
    }
    }
}

void
PCL_upd_DEMO::tryCNNfunctions()
{

    tiny_dnn::network<tiny_dnn::sequential> mlp = tiny_dnn::make_mlp<tiny_dnn::activation::tan_h>({2, 10, 2});

    std::vector<tiny_dnn::vec_t> data =  { {0.1, 0.9}, {0.9, 0.1}, {0.2, 0.5} };
    std::vector<tiny_dnn::label_t> labels =  { 1, 0, 0 };
/// TODO: check if empty and make the code more robust!



    tiny_dnn::adagrad optimizermlp;
    mlp.train<tiny_dnn::mse>(optimizermlp, data, labels); // T == label_t


#ifdef no
    {
    std::vector<tiny_dnn::label_t> train_labels, test_labels;
    std::vector<tiny_dnn::vec_t> train_images, test_images;
    std::vector<tiny_dnn::vec_t> train_data, test_data;

    // TODO:: load data completely missing !


    tiny_dnn::network<tiny_dnn::sequential> nn_s;
//    tiny_dnn::network<tiny_dnn::graph> nn_g;

//tiny_dnn::network<tiny_dnn::sequential> mynet = tiny_dnn::make_mlp<tiny_dnn::mse, tiny_dnn::gradient_descent, tiny_dnn::activation::tan_h>({ 32 * 32, 300, 10 });
std::vector<tiny_dnn::cnn_size_t> units ({ 32 * 32, 300, 10 });
tiny_dnn::network<tiny_dnn::sequential> mynet;
    mynet = tiny_dnn::make_mlp<tiny_dnn::activation::tan_h> (units.begin(), units.end());





    // construct nets   --- // Deconcolutional Auto-encoder
tiny_dnn::cnn_size_t in_width = 32;
tiny_dnn::cnn_size_t in_height= 32;
tiny_dnn::cnn_size_t window_size = 5;
tiny_dnn::cnn_size_t in_channels = 1; //grayscale=1, rgb=3
tiny_dnn::cnn_size_t out_channels = 6;

    nn_s << tiny_dnn::convolutional_layer<tiny_dnn::activation::tan_h>(in_width, in_height, window_size, in_channels, out_channels)
         << tiny_dnn::average_pooling_layer<tiny_dnn::activation::tan_h>(28, 28, 6, 2)
         << tiny_dnn::convolutional_layer<tiny_dnn::activation::tan_h>(14, 14, 3, 6, 16)
         << tiny_dnn::deconvolutional_layer<tiny_dnn::activation::tan_h>(12, 12, 3, 16, 6)
         << tiny_dnn::average_unpooling_layer<tiny_dnn::activation::tan_h>(14, 14, 6, 2)
         << tiny_dnn::deconvolutional_layer<tiny_dnn::activation::tan_h>(28, 28, 5, 6, 1);


    std::vector<tiny_dnn::vec_t> training_images_corrupted(train_images);

    for (auto& d : training_images_corrupted) {
        d = tiny_dnn::corrupt(std::move(d), tiny_dnn::float_t(0.1), tiny_dnn::float_t(0.0)); // corrupt 10% data
    }

    tiny_dnn::gradient_descent optimizer;

    // learning deconcolutional Auto-encoder
    nn_s.train<tiny_dnn::mse>(optimizer, training_images_corrupted, train_images);

    std::cout << "end training." << std::endl;

    }
#endif

    std::cout << " ciao " << std::endl;
}

void
PCL_upd_DEMO::getCNNoptimizationParams( )
{
    bool isNumeric;

    int epochs = 0;
    epochs = ui->lineEdit_cnnEphocs->text().toInt(&isNumeric);
    if(!isNumeric)
    {
        QMessageBox::warning(this, "Warning !", "alpha is not a valid number !");
        QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
        return;
    }
    m_cnn_epoch = epochs;

    int value = ui->comboBox_optimizationAlgorithms->currentIndex();

    switch (value) {
    case 0: //adagrad
    {
        double alpha = 0;
        alpha = ui->lineEdit_cnnAlpha->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "alpha is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_alpha_adagrad = alpha;
        //opt->alpha = m_alpha_adagrad;
        break;
     }
    case 1: //RMSprop
    {
        double alpha = 0;
        alpha = ui->lineEdit_cnnAlpha->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "alpha is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_alpha_RMSprop = alpha;
        //opt->alpha = m_alpha_RMSprop;

        double mu = 0;
        mu = ui->lineEdit_cnnMu->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "mu is not a valid number !");

            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_mu_RMSprop = mu;
        //opt->mu = m_mu_RMSprop;
        break;
    }
    case 2: //adam
    {
        double alpha = 0;
        alpha = ui->lineEdit_cnnAlpha->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "alpha is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_alpha_adam = alpha;
        //opt->alpha = m_alpha_adam;

        double b1 = 0;
        b1 = ui->lineEdit_cnnB1->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "b1 is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_b1_adam = b1;
        //opt->b1 = m_b1_adam;

        double b2 = 0;
        b2 = ui->lineEdit_cnnb2->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "b2 is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_b2_adam = b2;
        //opt->b2 = m_b2_adam;

        double b1_t = 0;
        b1_t = ui->lineEdit_cnnB1_t->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "b1_t is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_b1_t_adam = b1_t;
        //opt->b1_t = m_b1_t_adam;

        double b2_t = 0;
        b2_t = ui->lineEdit_cnnb2_t->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "b2_t is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_b2_t_adam = b2_t;
        //opt->b2_t = m_b2_t_adam;

        break;
    }
   case 3: //gradient descend SGD
    {
        double alpha = 0;
        alpha = ui->lineEdit_cnnAlpha->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "alpha is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_alpha_sgd = alpha;
        //opt->alpha = m_alpha_sgd;

        double lambda = 0;
        lambda = ui->lineEdit_cnnLambda->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "lambda is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_lambda_sgd = lambda;
        //opt->lambda = m_lambda_sgd;
        break;
    }
    case 4: //gradient descend with momentum
    {
        double alpha = 0;
        alpha = ui->lineEdit_cnnAlpha->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "alpha is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_alpha_sgdm = alpha;
        //opt->alpha = m_alpha_sgdm;

        double lambda = 0;
        lambda = ui->lineEdit_cnnLambda->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "lambda is not a valid number !");
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_lambda_sgdm = lambda;
        //opt->lambda = m_lambda_sgdm;


        double mu = 0;
        mu = ui->lineEdit_cnnMu->text().toDouble(&isNumeric);
        if(!isNumeric)
        {
            QMessageBox::warning(this, "Warning !", "lambda is not a valid number !");
            m_mu_sgdm = mu;
            QApplication::restoreOverrideCursor();    //close transform the cursor for waiting mode
            return;
        }
        m_mu_sgdm = mu;
        //opt->mu = m_mu_sgdm;



        break;
    }
    default:
        std::cout << "PCL_upd_DEMO::optimizationChanged  << message >> not recognized optimization function "
                  << std::endl;
  }

}


void
PCL_upd_DEMO::testCNN()
{
///TODO: if no network trained and bla bla bla
///
    std::vector<tiny_dnn::vec_t> data = m_cnn_training_set.dataVectors_t();
    std::vector<tiny_dnn::label_t> labels = m_cnn_training_set.dataLabels_t();

    if (data.size()<1 || labels.size()<1)
    {
     std::cout << " PCL_upd_DEMO::trainCNN  << error >> : no data to train !  " << std::endl;
        return;
    }

// perform a test on the training data
m_mlp.test(data, labels).print_detail(std::cout);


}


void
PCL_upd_DEMO::optimizationChanged(int _value)
{
//    if (opt == NULL)  {
//        opt = new tiny_dnn::adagrad; //default value
//    }

    switch (_value) {
            case 0: //adagrad
    {   ui->lineEdit_cnnAlpha->setEnabled(true);
        ui->lineEdit_cnnAlpha->setText(QString::number(m_alpha_adagrad));
        ui->lineEdit_cnnB1->setEnabled(false);
        ui->lineEdit_cnnb2->setEnabled(false);
        ui->lineEdit_cnnB1_t->setEnabled(false);
        ui->lineEdit_cnnb2_t->setEnabled(false);
        ui->lineEdit_cnnLambda->setEnabled(false);
        ui->lineEdit_cnnMu->setEnabled(false);
//        if (opt != NULL)  {
//            delete opt;
//        }
//        opt = new tiny_dnn::adagrad ();


        break;
     }
    case 1: //RMSprop
    {
        ui->lineEdit_cnnAlpha->setEnabled(true);
        ui->lineEdit_cnnAlpha->setText(QString::number(m_alpha_RMSprop)); ///TODO this must go in a settable parameter
        ui->lineEdit_cnnB1->setEnabled(false);
        ui->lineEdit_cnnb2->setEnabled(false);
        ui->lineEdit_cnnB1_t->setEnabled(false);
        ui->lineEdit_cnnb2_t->setEnabled(false);
        ui->lineEdit_cnnLambda->setEnabled(false);
        ui->lineEdit_cnnMu->setEnabled(true);
        ui->lineEdit_cnnMu->setText(QString::number(m_mu_RMSprop));
//        if (opt != NULL)  {
//         delete opt;
//        }
//         opt = new tiny_dnn::RMSprop();

        break;
    }
    case 2: //adam
    {
        ui->lineEdit_cnnAlpha->setEnabled(true);
        ui->lineEdit_cnnAlpha->setText(QString::number(m_alpha_adam));
        ui->lineEdit_cnnB1->setEnabled(true);
        ui->lineEdit_cnnB1->setText(QString::number(m_b1_adam));
        ui->lineEdit_cnnb2->setEnabled(true);
        ui->lineEdit_cnnb2->setText(QString::number(m_b2_adam));
        ui->lineEdit_cnnB1_t->setEnabled(true);
        ui->lineEdit_cnnB1_t->setText(QString::number(m_b1_t_adam));
        ui->lineEdit_cnnb2_t->setEnabled(true);
        ui->lineEdit_cnnb2_t->setText(QString::number(m_b2_t_adam));
        ui->lineEdit_cnnLambda->setEnabled(false);
        ui->lineEdit_cnnMu->setEnabled(false);
//        if (opt != NULL)  {
//            delete opt;
//        }
//        opt = new tiny_dnn::adam;
        break;
    }
   case 3: //gradient descend SGD
    {
        ui->lineEdit_cnnAlpha->setEnabled(true);
        ui->lineEdit_cnnAlpha->setText(QString::number(m_alpha_sgd));
        ui->lineEdit_cnnB1->setEnabled(false);
        ui->lineEdit_cnnb2->setEnabled(false);
        ui->lineEdit_cnnB1_t->setEnabled(false);
        ui->lineEdit_cnnb2_t->setEnabled(false);
        ui->lineEdit_cnnLambda->setEnabled(true);
        ui->lineEdit_cnnLambda->setText(QString::number(m_lambda_sgd));
        ui->lineEdit_cnnMu->setEnabled(false);
//        if (opt != NULL)  {
//            delete opt;
//        }
//        opt = new tiny_dnn::gradient_descent;
        break;
    }
    case 4: //gradient descend with momentum
    {
        ui->lineEdit_cnnAlpha->setEnabled(true);
        ui->lineEdit_cnnAlpha->setText(QString::number(m_alpha_sgdm));
        ui->lineEdit_cnnB1->setEnabled(false);
        ui->lineEdit_cnnb2->setEnabled(false);
        ui->lineEdit_cnnB1_t->setEnabled(false);
        ui->lineEdit_cnnb2_t->setEnabled(false);
        ui->lineEdit_cnnLambda->setEnabled(true);
        ui->lineEdit_cnnLambda->setText(QString::number(m_lambda_sgdm));
        ui->lineEdit_cnnMu->setEnabled(true);
        ui->lineEdit_cnnMu->setText(QString::number(m_mu_sgdm));
//        if (opt != NULL)  {
//            delete opt;
//        }
//        opt = new tiny_dnn::momentum;
        break;
    }
    default:
        std::cout << "PCL_upd_DEMO::optimizationChanged  << message >> not recognized optimization function "
                  << std::endl;
  }

}
