/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: poster_landmark.cc
  * @version: v0.0.1
  * @author: aliben@USERNAME.com
  * @create_date: 2019-07-16 10:52:02
  * @last_modified_date: 2019-07-16 13:48:24
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <cartographer_ros/poster_landmark.hh>

//CODE
namespace vn
{
  size_t LandmarkCluster::size()
  {
    return landmarks_.size();
  }

  void LandmarkCluster::CalCenter()
  {
    if(flag_drop_ == true && landmarks_.size() == 0)
    {
      return;
    }
    mean_position_ = Landmark(0.0, 0.0, 0.0);
    for(auto& landmark:landmarks_)
    {
      mean_position_ += landmark;
      //std::cout << "lm: " << landmark;
      //std::cout << "mean: " << mean_position_;
    }
    mean_position_/=landmarks_.size();
    for(auto& landmark:landmarks_)
    {
      err_xx_ += std::pow((landmark.x_ - mean_position_.x_),2);
      err_xy_ += (landmark.x_ - mean_position_.x_)*(landmark.y_ - mean_position_.y_);
      err_yx_ += err_xy_;
      err_yy_ += std::pow((landmark.y_-mean_position_.y_),2);
    }
    err_xx_ /= landmarks_.size();
    err_xy_ /= landmarks_.size();
    err_yx_ /= landmarks_.size();
    err_yy_ /= landmarks_.size();
  }

  void LandmarkGenerator::RemoveInvalidCluster()
  {
    for(auto& landmark_cluster:landmark_clusters_)
    {
      if(landmark_cluster.size() < 3)
      {
        landmark_cluster.SetInvalid();
        continue;
      }
      landmark_cluster.CalCenter();
    }
  }

  bool LandmarkGenerator::CheckNewCluster(const Landmark& new_landmark)
  {
    for(auto& landmark_cluster:landmark_clusters_)
    {
      for(auto const& landmark:landmark_cluster.landmarks_)
      {
        if(Landmark::CalDist(new_landmark, landmark) <= 0.2)
        {
          landmark_cluster.landmarks_.emplace_back(new_landmark);
          return false;
        }
      }
    }
    landmark_clusters_.emplace_back(LandmarkCluster(new_landmark));
    return true;
  }

  void LandmarkGenerator::SavePosterLandmark(const std::string& save_path)
  {
    std::ofstream out_file;
    out_file.open(save_path);

    for(auto const& landmark_cluster:landmark_clusters_)
    {
      if(landmark_cluster.flag_drop_ == true) continue;
      out_file << landmark_cluster.mean_position_.x_ << " "
               << landmark_cluster.mean_position_.y_ << " "
               << landmark_cluster.err_xx_ << " "
               << landmark_cluster.err_xy_ << " "
               << landmark_cluster.err_yx_ << " "
               << landmark_cluster.err_yy_ << std::endl;
    }

    std::cout << "Save Done!" << std::endl;
    out_file.close();
  }
}
