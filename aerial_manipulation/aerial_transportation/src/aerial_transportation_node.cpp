#include <aerial_transportation/aerial_transportation.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "aerial_transportation");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  AerialTransportation* AerialTransportationNode = new AerialTransportation(nh, nhp);

  ros::spin();

  delete AerialTransportationNode;

  return 0;
}

