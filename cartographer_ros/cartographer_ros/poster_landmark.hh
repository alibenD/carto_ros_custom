#ifndef __POSTER_LANDMARK_HH__
#define __POSTER_LANDMARK_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: poster_landmark.hh
  * @version: v0.0.1
  * @author: aliben@USERNAME.com
  * @create_date: 2019-07-16 10:52:02
  * @last_modified_date: 2019-07-16 13:39:15
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

// Declaration
namespace vn
{
  struct Landmark
  {
    Landmark(double x, double y, double z): x_(x), y_(y), z_(z){};
    static double CalDist(const Landmark& ori, const Landmark& dst)
    {
      auto diff_x = ori.x_ - dst.x_;
      auto diff_y = ori.y_ - dst.y_;
      auto diff_z = ori.z_ - dst.z_;
      return double(std::sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z));
    }

    Landmark operator+(const Landmark& rhs)
    {
      return Landmark(rhs.x_+this->x_, rhs.y_+this->y_, rhs.z_+this->z_);
    }

    Landmark& operator+=(const Landmark& rhs)
    {
      //return Landmark(rhs.x_+this->x_, rhs.y_+this->y_, rhs.z_+this->z_);
      this->x_ += rhs.x_;
      this->y_ += rhs.y_;
      this->z_ += rhs.z_;
      return *this;
    }

    template<typename T>
    Landmark& operator/(const T divisor)
    {
      this->x_ /= double(divisor);
      this->y_ /= double(divisor);
      this->z_ /= double(divisor);
      return *this;
    }
    template<typename T>
    Landmark& operator/=(const T divisor)
    {
      this->x_ /= double(divisor);
      this->y_ /= double(divisor);
      this->z_ /= double(divisor);
      return *this;
    }
    double x_;
    double y_;
    double z_;
  };

  static std::ostream& operator<<(std::ostream& os, const Landmark& landmark)
  {
    os << "x: " << landmark.x_ << " y: " << landmark.y_ << " z: " << landmark.z_ << std::endl;
    return os;
  }

  class LandmarkGenerator;
  class LandmarkCluster
  {
    public:
      friend LandmarkGenerator;
      LandmarkCluster() = default;
      LandmarkCluster(const Landmark& landmark_first_appear)
      {
        landmarks_.clear();
        landmarks_.emplace_back(landmark_first_appear);
      }
      size_t size();
      void SetInvalid(){flag_drop_=true;}
      void CalCenter();
    private:
      std::vector<Landmark> landmarks_;
      Landmark mean_position_{0.0,0.0,0.0};
      bool flag_drop_{false};
      double err_xx_{0.0};
      double err_yy_{0.0};
      double err_xy_{0.0};
      double err_yx_{0.0};
  };

  class LandmarkGenerator
  {
    public:
      LandmarkGenerator() = default;
      bool CheckNewCluster(const Landmark& new_landmark);
      void RemoveInvalidCluster();
      size_t size() {return landmark_clusters_.size();}
      void SavePosterLandmark(const std::string& save_path);
    private:
      std::vector<LandmarkCluster> landmark_clusters_;
  };
}
#endif // __POSTER_LANDMARK_HH__
