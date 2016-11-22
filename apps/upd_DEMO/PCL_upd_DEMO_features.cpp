/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#include "PCL_upd_DEMO.h"


std::vector<float>
PCL_upd_DEMO::createAllFeatures()
{
    std::vector<float> features;
    if (ui->treeWidget_featuresSelection->topLevelItem(0)->checkState(0) )
    { //std::cout << "the item " <<  ui->treeWidget_featuresSelection->topLevelItem(0)->text(0).toStdString() << " is checked" << std::endl;
        std::vector<float> featuresShape = createFeaturesShape();
        features.insert(features.end(), featuresShape.begin(), featuresShape.end());
    }
    if (ui->treeWidget_featuresSelection->topLevelItem(1)->checkState(0) )
    { //std::cout << "the item " <<  ui->treeWidget_featuresSelection->topLevelItem(1)->text(0).toStdString() << " is checked" << std::endl;
        std::vector<float> featuresUPD = createFeaturesUPD();
        features.insert(features.end(), featuresUPD.begin(), featuresUPD.end());
    }
    if (ui->treeWidget_featuresSelection->topLevelItem(2)->checkState(0) )
    { std::cout << "the item " <<  ui->treeWidget_featuresSelection->topLevelItem(2)->text(0).toStdString() << " is checked" << std::endl;
        std::vector<float> featuresReina = createFeaturesReina();
        features.insert(features.end(), featuresReina.begin(), featuresReina.end());
    }
    if (ui->treeWidget_featuresSelection->topLevelItem(3)->checkState(0) )
    { std::cout << "the item " <<  ui->treeWidget_featuresSelection->topLevelItem(3)->text(0).toStdString() << " is checked" << std::endl;
        std::vector<float> featuresColor = createColorFeatures();
        features.insert(features.end(), featuresColor.begin(), featuresColor.end());
    }


    std::cout << " PCL_upd_DEMO::createAllFeatures size = " << features.size() << std::endl;


    return features;
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
    //std::cout << " covariance matrix is \n"<< covariance_matrix << std::endl;

    // singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> SVD (covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = SVD.matrixU();
    Eigen::MatrixXd V = SVD.matrixV();
    Eigen::MatrixXd singular_values = SVD.singularValues().asDiagonal();

    //std::cout << "SVD decomposition : \n"
    //          << " U = \n" << U << "\n"
    //          << " V = \n" << V << "\n"
    //          << " singular values: \n" << singular_values << std::endl;
    //std::cout << "verification : COV = U * sigma * V \n" << U * singular_values * V.transpose() << std::endl;

    // check for the rank !
    Eigen::FullPivLU<Eigen::Matrix3d> lu_decomp(covariance_matrix);
    //std::cout <<  " The rank of the covariance matrix is " << lu_decomp.rank() << std::endl;
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
    //std::cout << " product = " << product << "\n norm = " << product.norm() <<  std::endl;
    double slope = std::acos( product.norm());
    features.push_back(slope);
    //std::cout << " slope = " << slope << std::endl; //this must be a number so U is actually the eigenvector associated to the smallest eigenvalue

    // smallest singular value
    float small_singular_value = singular_values(2, 2);
    features.push_back(small_singular_value);
    //std::cout << " the smallest singular value is " << small_singular_value << std::endl;

    // mean along z
    float mean = 0.0;
    for (int i = 0; i< m_cloud_patch->size(); i++) {
        mean +=m_cloud_patch->at(i).y; ///todo: be carefull with this, it must be z but for the reference system in the application become y
    }
    mean = mean / m_cloud_patch->size();
    features.push_back(mean);
    //std::cout << " the mean is : " << mean << std::endl;

    // variance in the vertical direction
    float variance = 0.0;
    for (int i = 0; i< m_cloud_patch->size(); i++)  {
        variance += (m_cloud_patch->at(i).z - mean)*(m_cloud_patch->at(i).z - mean);
    }
    features.push_back(variance);
    //std::cout << " the variance is : " << variance << std::endl;

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
    features.push_back( UPD_cloud->points[m_patch_labelling_index].radius );  // upd value
    features.push_back( norm );
    double cloud_density = 100.0; //points per square meter
    double m_e = cloud_density * M_PI * std::pow(m_upd->getSearchRadius(), 2);
    std::cout << " the UPD cloud size for the patch is " << UPD_cloud->size()
              << " m_patch_labelling_index = " << m_patch_labelling_index
              << " upd value = " << UPD_cloud->points[ m_patch_labelling_index ].radius
              << " m_ e = " << m_e
              << " C = K/M_E " << UPD_cloud->size() / m_e << std::endl;
    features.push_back( UPD_cloud->size() / m_e );


    return features;

}

std::vector<float>
PCL_upd_DEMO::createFeaturesShape( )
{
    // prepare data for the classifier
    std::vector<float> features;

    // if we have no patch we cannot classify
    if ( m_cloud_patch->size() < 1 || m_cloud_patch->size() < m_patch_labelling_index ) {
        QMessageBox::information(this, "info !", " Patch size error !!");
        return features; }

    // covariance matrix
    Eigen::Matrix3d cov;
    pcl::computeCovarianceMatrix(*m_cloud_patch, cov);

    // compute the eigenvalues of the covariance
    Eigen::VectorXcd ev = cov.eigenvalues();
    //std::cout << " the eigenvalues are : " << ev << std::endl;
    Eigen::Vector3d lambdas (ev(0).real(), ev(1).real(), ev(2).real());
    lambdas.normalize();
    std::sort(lambdas.data(), lambdas.data()+lambdas.size());

    // assume l1 > l2 > l3
    // Linearity = L = (l1 - l2)/l1
    double linearity = (lambdas(0) - lambdas(1))/lambdas(0);
    /// TODO: if lambdas(0) = 0
    features.push_back(linearity);
    //std::cout << " the linearity is : " << linearity << std::endl;

    // Planarity = P = (l2 - l3)/l1
    float planarity = (lambdas(1) - lambdas(2))/lambdas(0);
    /// TODO: if lambdas(0) = 0
    features.push_back(planarity);
    //std::cout << " the planarity is : " << planarity << std::endl;

    // Sphericity = S = l3/l1
    float sphericity = lambdas(2)/lambdas(0);
    /// TODO: if lambdas(0) = 0
    features.push_back(sphericity);
    //std::cout << " the sphericity is : " << sphericity << std::endl;

    // Omnivariance O = (l1*l2*l3)^(1/3)
    float omnivariance = std::cbrt(lambdas(0) * lambdas(1) * lambdas(2));
    features.push_back(omnivariance);
    //std::cout << " the omnivariance is : " << omnivariance << std::endl;

    // Anisotropy A = (l1-l3)/l1
    /// TODO: if lambdas(0) = 0
    float anisotropy = (lambdas(0) - lambdas(2))/lambdas(0);
    features.push_back(anisotropy);
    //std::cout << " the anisotropy is : " << anisotropy << std::endl;

    // Eigenentropy E = -sum(li*log(li))
    /// TODO: if lambdas(0) > 0
    float eigenentropy = -( lambdas(0)*std::log(lambdas(0)) +
                            lambdas(1)*std::log(lambdas(1)) +
                            lambdas(2)*std::log(lambdas(2)) );
    features.push_back(eigenentropy);
    //std::cout << " the eigenentropy is : " << eigenentropy << std::endl;

    // Sum of lambdas SUM = l1+l2+l3
    float sum_of_lambdas = lambdas(0) + lambdas(1) + lambdas(2);
    features.push_back(sum_of_lambdas);
    //std::cout << " the sum_of_lambdas is : " << sum_of_lambdas << std::endl;

    // Change of curvature = l3/(SUM)
    float curvature = lambdas(2)/sum_of_lambdas;
    features.push_back(curvature);
    //std::cout << " the curvature is : " << curvature << std::endl;


    return features;
}

std::vector<float>
PCL_upd_DEMO::createColorFeatures( )
{
    // prepare data for the classifier
    std::vector<float> features;
    // if we have no patch we cannot classify
    if ( m_cloud_patch->size() < 1 || m_cloud_patch->size() < m_patch_labelling_index ) {
        QMessageBox::information(this, "info !", " Patch size error !!");
        return features; }

    //TODO: check if the patch have a color otherwise it is not possible to have color-based features



    float r = m_cloud_patch->points[m_patch_labelling_index].r / 255.0;
    float g = m_cloud_patch->points[m_patch_labelling_index].g / 255.0;
    float b = m_cloud_patch->points[m_patch_labelling_index].b / 255.0;
    float c1 = std::atan2(r, std::max(g,b));
    float c2 = std::atan2(g, std::max(r,b));
    float c3 = std::atan2(b, std::max(r,g));

    float hue = std::atan2 (std::sqrt(3)*(g-b), 2*r-g-b  );
    //float saturation =

    features.push_back( c1 );
    features.push_back( c2 );
    features.push_back( c3 );
    features.push_back( hue );
//    features.push_back( saturation );


    //std::cout << " R = " << features.at(0)
    //         << " G = " << features.at(1)
    //          << " B = " << features.at(2)
//              << " RGBA = " << features.at(3)
    //          << std::endl;

    return features;
}
