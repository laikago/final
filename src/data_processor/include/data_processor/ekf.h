

#ifndef ekf_H
#define ekf_H

#include <Eigen/Dense>
#include <cmath>
#include <string> 
#include <sstream>
class ekf{
public:
  Eigen::Vector<double, 11> x;//状态矩阵
  Eigen::MatrixXd P;//预测协方差矩阵
  double r,dt;

  
ekf();
void init();//const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);
void predict();
void update();// const Eigen::VectorXd & z);
bool observe(Eigen::Matrix3d R_armor2world, Eigen::Vector3d xyz_in_world);
  // 新增 eulers 函数声明（两个重载版本）
  Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic);
  Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic);

  std::string debug();
private:
double limit_rad(double angle);
 Eigen::MatrixXd I;//更新协方差用
 

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;


  Eigen::MatrixXd F;//预测矩阵
Eigen::MatrixXd Q;//过程噪声协方差矩阵
  
Eigen::Vector<double, 11> x_prior;//缓存旧状态
Eigen::MatrixXd K;//卡尔曼增益
Eigen::MatrixXd H;//观测矩阵
Eigen::VectorXd residual;//残差
Eigen::MatrixXd R;//测量方差
Eigen::Vector4d z;//观测向量
int armor_num_;//装甲板数量 
int id;
bool is_init;
bool is_jump;
int last_id;
//卡方检验相关
Eigen::MatrixXd S ;
  double nis ;
  double nees;
  //调试状态码
  int code;
  std::ostringstream oss;//调试字符输出
};

#endif 
