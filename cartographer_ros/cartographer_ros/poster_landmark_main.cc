/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: poster_landmark_main.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-07-16 12:41:22
  * @last_modified_date: 2019-07-16 13:24:58
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <cartographer_ros/poster_landmark.hh>

//CODE
using namespace vn;
int main(int argc, char** argv)
{
  if(argc < 3)
  {
    std::cout << "USAGE: " << argv[0] << " " << "PATH_TO_RAW_LANDMARKS" << " PATH_TO_SAVE_POSTER_LANDMARKS" << std::endl;
    exit(1);
  }
  vn::LandmarkGenerator lm_gen;
  std::ifstream file_in;
  file_in.open(std::string(argv[1]));
  //std::cout << "file: " << std::string(argv[1]) << std::endl;
  double x,y,z;
  size_t count_landmark = 0;
  while(!file_in.eof())
  {
    file_in >> x >> y >> z;
    //std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
    if(file_in.good() == false)
    {
      //std::cout << "Meet EOF"<< std::endl;
      break;
    }
    lm_gen.CheckNewCluster(Landmark(x,y,z));
    ++count_landmark;
  }
  file_in.close();
  std::cout << "Load " << count_landmark << " landmarks." << std::endl;
  lm_gen.RemoveInvalidCluster();
  lm_gen.SavePosterLandmark(std::string(argv[2]));

  return 0;
}
