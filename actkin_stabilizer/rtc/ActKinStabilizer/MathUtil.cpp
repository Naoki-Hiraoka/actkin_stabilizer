#include "MathUtil.h"
#include <limits>
#include <cddeigen/cddeigen.h>

namespace actkin_stabilizer{
  namespace mathutil {
    Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
      // axisとlocalaxisはノルムが1, mは回転行列でなければならない.
      // axisとlocalaxisがピッタリ180反対向きの場合、回転方向が定まらないので不安定
      Eigen::AngleAxisd m_ = Eigen::AngleAxisd(m); // Eigen::Matrix3dの空間で積算していると数値誤差によってだんたん回転行列ではなくなってくるので
      Eigen::Vector3d localaxisdir = m_ * localaxis;
      Eigen::Vector3d cross = localaxisdir.cross(axis);
      double dot = std::min(1.0,std::max(-1.0,localaxisdir.dot(axis))); // acosは定義域外のときnanを返す
      if(cross.norm()==0){
        if(dot == -1) return Eigen::Matrix3d(-m);
        else return Eigen::Matrix3d(m_);
      }else{
        double angle = std::acos(dot); // 0~pi
        Eigen::Vector3d axis = cross.normalized(); // include sign
        return Eigen::Matrix3d(Eigen::AngleAxisd(angle, axis) * m_);
      }
    }
    Eigen::Transform<double, 3, Eigen::AffineCompact> orientCoordToAxis(const Eigen::Transform<double, 3, Eigen::AffineCompact>& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
      Eigen::Transform<double, 3, Eigen::AffineCompact> ret = m;
      ret.linear() = mathutil::orientCoordToAxis(ret.linear(), axis, localaxis);
      return ret;
    }
    Eigen::AngleAxisd slerp(const Eigen::AngleAxisd& M1, const Eigen::AngleAxisd& M2, double r){
      // 0 <= r <= 1
      Eigen::AngleAxisd trans = Eigen::AngleAxisd(M1.inverse() * M2);
      return Eigen::AngleAxisd(M1 * Eigen::AngleAxisd(trans.angle() * r, trans.axis()));
    }

    Eigen::Vector3d calcMidPos(const std::vector<Eigen::Vector3d>& coords, const std::vector<double>& weights){
      // coordsとweightsのサイズは同じでなければならない
      double sumWeight = 0.0;
      Eigen::Vector3d midpos = Eigen::Vector3d::Zero();

      for(int i=0;i<coords.size();i++){
        if(weights[i]<=0) continue;
        midpos = ((midpos*sumWeight + coords[i]*weights[i])/(sumWeight+weights[i])).eval();
        sumWeight += weights[i];
      }
      return midpos;
    }
    Eigen::Matrix3d calcMidRot(const std::vector<Eigen::Matrix3d>& coords, const std::vector<double>& weights){
      // coordsとweightsのサイズは同じでなければならない
      double sumWeight = 0.0;
      Eigen::AngleAxisd midrot = Eigen::AngleAxisd::Identity();

      for(int i=0;i<coords.size();i++){
        if(weights[i]<=0) continue;
        midrot = mathutil::slerp(midrot, Eigen::AngleAxisd(coords[i]), weights[i]/(sumWeight+weights[i]));
        //midrot = midrot.slerp(weights[i]/(sumWeight+weights[i]),Eigen::Quaterniond(coords[i])); // quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない
        sumWeight += weights[i];
      }
      return midrot.toRotationMatrix();
    }
    Eigen::Transform<double, 3, Eigen::AffineCompact> calcMidCoords(const std::vector<Eigen::Transform<double, 3, Eigen::AffineCompact>>& coords, const std::vector<double>& weights){
      // coordsとweightsのサイズは同じでなければならない
      double sumWeight = 0.0;
      Eigen::Transform<double, 3, Eigen::AffineCompact> midCoords = Eigen::Transform<double, 3, Eigen::AffineCompact>::Identity();

      for(int i=0;i<coords.size();i++){
        if(weights[i]<=0) continue;
        midCoords.translation() = ((midCoords.translation()*sumWeight + coords[i].translation()*weights[i])/(sumWeight+weights[i])).eval();
        midCoords.linear() = mathutil::slerp(Eigen::AngleAxisd(midCoords.linear()), Eigen::AngleAxisd(coords[i].linear()),(weights[i]/(sumWeight+weights[i]))).toRotationMatrix();
        //midCoords.linear() = Eigen::Quaterniond(midCoords.linear()).slerp(weights[i]/(sumWeight+weights[i]),Eigen::Quaterniond(coords[i].linear())).toRotationMatrix(); // quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない
        sumWeight += weights[i];
      }
      return midCoords;
    }

    inline double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b){
      return a[0] * b[1] - a[1] * b[0];
    }

    inline bool isIntersect (Eigen::Vector2d& r, const Eigen::Vector2d& a0, const Eigen::Vector2d& a1, const Eigen::Vector2d& b0, const Eigen::Vector2d& b1){
      double D =  mathutil::cross(a1 - a0, b1 - b0);
      if (D == 0.0) return false;
      double t =  mathutil::cross(b0 - a0, b1 - b0) / D;
      double s = - mathutil::cross(a0 - b0, a1 - a0) / D;
      r = a0 + t * (a1 - a0);
      return (t >= 0.0 && t <= 1.0 && s >= 0.0 && s <= 1.0);
    }


    void calcConvexHull(const std::vector<Eigen::Vector2d>& contours, std::vector<Eigen::Vector2d>& hull) {
      std::vector<Eigen::Vector2d> tmpVertices = contours;
      if(tmpVertices.size() == 1) {
        hull = tmpVertices;
        return;
      }
      if(tmpVertices.size() == 2) {
        if(tmpVertices[0] != tmpVertices[1]) {
          hull = tmpVertices;
          return;
        }else{
          hull = std::vector<Eigen::Vector2d>{tmpVertices[0]};
          return;
        }
      }
      std::sort(tmpVertices.begin(), tmpVertices.end(), [](const Eigen::Vector2d& lv, const Eigen::Vector2d& rv){ return lv(0) < rv(0) || (lv(0) == rv(0) && lv(1) < rv(1));});
      std::vector<Eigen::Vector2d> convexHull(2*tmpVertices.size());
      int n_ch = 0;
      for (int i = 0; i < tmpVertices.size(); convexHull[n_ch++] = tmpVertices[i++])
        while (n_ch >= 2 && mathutil::cross(convexHull[n_ch-1] - convexHull[n_ch-2], tmpVertices[i] - convexHull[n_ch-2]) <= 0.0) n_ch--;
      for (int i = tmpVertices.size()-2, j = n_ch+1; i >= 0; convexHull[n_ch++] = tmpVertices[i--])
        while (n_ch >= j && mathutil::cross(convexHull[n_ch-1] - convexHull[n_ch-2], tmpVertices[i] - convexHull[n_ch-2]) <= 0.0) n_ch--;
      convexHull.resize(std::max(0,n_ch-1));
      hull = convexHull;
      return;

      // cv::convexHullはやや遅い.
    }

    bool isInsideHull(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& contours) {
      static const double eps = 1e-10; // edge上にある場合に浮動小数点の丸め誤差に対応

      if(contours.size() == 0) return false;
      else if(contours.size() == 1) return contours[0] == p;
      else if(contours.size() == 2) {
        Eigen::Vector2d a = contours[0] - p, b = contours[1] - p;
        return (std::abs(mathutil::cross(a,b)) < eps) && (a.dot(b) <= 0);
      }else {
        for (int i = 0; i < contours.size(); i++) {
          Eigen::Vector2d a = contours[i] - p, b = contours[(i+1)%contours.size()] - p;
          if(mathutil::cross(a,b) < -eps) return false;
        }
        return true;
      }

      // openCVのcv::pointPolygonTestは、凹形状に対応しているぶん低速である + edge上にある場合に浮動小数点の丸め誤差により誤判定する場合がある ので、使ってはならない
    }


    std::vector<Eigen::Vector2d> calcIntersectConvexHull(const std::vector<Eigen::Vector2d>& P, const std::vector<Eigen::Vector2d>& Q) {
      std::vector<Eigen::Vector2d> R;
      for(int i=0; i<P.size();i++){
        if(isInsideHull(P[i],Q)) R.push_back(P[i]);
      }
      for(int j=0; j<Q.size();j++){
        if(isInsideHull(Q[j],P)) R.push_back(Q[j]);
      }
      Eigen::Vector2d r;
      if(P.size()>1 && Q.size() > 1){
        for(int i=0; i<P.size();i++){
          for(int j=0; j<Q.size();j++){
            if(isIntersect(r, P[i], P[(i+1)%P.size()], Q[j], Q[(j+1)%Q.size()])) R.push_back(r);
          }
        }
      }
      calcConvexHull(R, R);
      return R;

      // OpenCVのcv::intersectConvexConvexは、一方が一方に内接する場合に計算に失敗するので使ってはいけない.
    }

    std::vector<Eigen::Vector2d> resizeHull(const std::vector<Eigen::Vector2d>& hull, double length){
      // cddlibは低速なので使うべきでない
      Eigen::MatrixXd V(2,hull.size());
      Eigen::MatrixXd R_nonneg(2,0);
      Eigen::MatrixXd R_free(2,0);
      for(int i=0;i<hull.size();i++){
        V.col(i) = hull[i];
      }
      Eigen::MatrixXd A_eq, A_ineq;
      Eigen::VectorXd b_eq, b_ineq;
      if(!cddeigen::VtoHgmp(V,R_nonneg,R_free,A_eq,b_eq,A_ineq,b_ineq)){
        return std::vector<Eigen::Vector2d>();
      }
      for(int i=0;i<A_eq.rows();i++){
        double norm = A_eq.row(i).norm();
        if(norm > 0){
          A_eq.row(i) /= norm;
          b_eq[i] /= norm;
        }
      }
      for(int i=0;i<A_ineq.rows();i++){
        double norm = A_ineq.row(i).norm();
        if(norm > 0){
          A_ineq.row(i) /= norm;
          b_ineq[i] /= norm;
        }
      }

      Eigen::MatrixXd A_eq2(0,2);
      Eigen::VectorXd b_eq2(0);
      Eigen::MatrixXd A_ineq2(A_eq.rows()*2+A_ineq.rows(),2);
      Eigen::VectorXd b_ineq2(A_eq.rows()*2+A_ineq.rows());
      int idx = 0;
      for(int i=0;i<A_eq.rows();i++){
        A_ineq2.row(idx) = A_eq.row(i);
        b_ineq2[idx] = b_eq[i] + length;
        idx++;
        A_ineq2.row(idx) = -A_eq.row(i);
        b_ineq2[idx] = -b_eq[i] + length;
        idx++;
      }
      for(int i=0;i<A_ineq.rows();i++){
        A_ineq2.row(idx) = A_ineq.row(i);
        b_ineq2[idx] = b_ineq[i] + length;
        idx++;
      }

      Eigen::MatrixXd V2;
      Eigen::MatrixXd R_nonneg2;
      Eigen::MatrixXd R_free2;
      if(!cddeigen::HtoVgmp(A_eq2,b_eq2,A_ineq2,b_ineq2,V2,R_nonneg2,R_free2)){
        return std::vector<Eigen::Vector2d>();
      }


      std::vector<Eigen::Vector2d> vertices(V2.cols());
      for(int i=0;i<V2.cols();i++) vertices[i] = V2.col(i);
      mathutil::calcConvexHull(vertices, vertices);
      return vertices;
    }

  };

};
