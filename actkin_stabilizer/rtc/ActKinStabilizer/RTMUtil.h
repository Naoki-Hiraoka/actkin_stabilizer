#ifndef RTMUTIL_H
#define RTMUTIL_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

namespace rtmutil{
  inline bool isAllFinite(const _CORBA_Sequence<double>& v){
    for(int i=0;i<v.length();i++){
      if(!std::isfinite(v[i])) return false;
    }
    return true;
  }
  template<typename T>
  inline bool isAllFinite(const _CORBA_Sequence<T>& v){
    for(int i=0;i<v.length();i++){
      if(!isAllFinite(v[i])) return false;
    }
    return true;
  }
  inline bool isAllFinite(const RTC::Vector3D& v){
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  inline bool isAllFinite(const RTC::Point3D& v){
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  inline bool isAllFinite(const RTC::Orientation3D& v){
    return std::isfinite(v.r) && std::isfinite(v.p) && std::isfinite(v.y);
  }
  inline bool isAllFinite(const RTC::Pose3D& v){
    return isAllFinite(v.position) && isAllFinite(v.orientation);
  }
};

#endif
