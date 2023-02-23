#ifndef ACTTOGENFRAMECONVERTER_H
#define ACTTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include <cnoid/Body>

class ActToGenFrameConverter {
public:
  // ActToGenFrameConverterだけでつかうパラメータ
  cpp_filters::TwoPointInterpolator<cnoid::Vector3> rpyOffset{cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(), cpp_filters::HOFFARBIB}; // [roll, pitch, yaw]. rootLink Frame. Actual robotのrootLinkの姿勢に加えるオフセット. IMUの取り付け位置のオフセットを考慮するためのものではない(それはモデルファイルを変えれば良い). 全身のキャリブのずれなど次第に出てくるなにかしらのずれをごますためのもの. 本来このようなパラメータは必要ないのが望ましいが、実用上は確かに必要.

public:

protected:
  // 内部で変更されるパラメータ. startAutoBalancer時にリセットされる
  mutable bool isInitial = true;

public:
  void init(GaitParam& gaitParam){
  }
  void onExecute(double dt){
    rpyOffset.interpolate(dt);
  }
  void onStartAutoBalancer() {
    isInitial = true;
  }
  void onStartStabilizer() {
  }

public:
  // robotとobjectsの位置姿勢と速度を更新する. actRobotRawとcontactsを用いて
  bool convertFrame(const GaitParam& gaitParam, double dt, // input
                    std::shared_ptr<Object>& robot, std::vector<std::shared_ptr<Object> >& objects) const; // input & output
};

#endif
