
#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>

#include <pcl/ml/svm.h>
#include <pcl/ml/svm_wrapper.h>

#include <ostream>


int	main (int argc, char** argv)
{
	std::cout<<"\n >>>  UPD cmdline demo 2 experimental acquisition - 2014  <<< \n\n"<<std::endl;

	pcl::SVMClassify my_svm_classifier;



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