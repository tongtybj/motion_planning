#include <hydrus_object_transportation/posture_optimization.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydrus_posture_optimization");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  PostureOptimization *posture_optimization = new PostureOptimization(nh,nhp);
  //posture_optimization->process();
  posture_optimization->check();
  ros::spin();
  ros::shutdown();
  delete posture_optimization;


 return 0;
}
