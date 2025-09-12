#include "data_processor/ekf.h"
#include <numeric>  // 用于std::accumulate
ekf::ekf(){is_init=false;}
void ekf::init(){
  
  
Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
P=P0_dig.asDiagonal();
x=Eigen::VectorXd {{0, 0, 0, 0, 0, 0, 0, 0, r, 0, 0}};
armor_num_=4;  //四个装甲板

I=Eigen::MatrixXd::Identity(x.rows(), x.rows());
is_init=true;

F=Eigen::MatrixXd {
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  
    v1 = 100;//100;  // 加速度方差
    v2 = 400;//400;  // 角加速度方差
  
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Q=Eigen::MatrixXd {
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on


}



void ekf::predict(){
P = F * P * F.transpose() + Q; //P 有问题?   打印维度
x=F*x;
x[6]=limit_rad(x[6]);//状态预测
code=code|1<<3;
}




void ekf::update(){

x_prior = x;

//算雅可比矩阵
auto angle =limit_rad(x[6] + id * 2 * M_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];


  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza({
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  });
  // clang-format on

  //计算H_armor_ypda 

  const double xy_sq = armor_x * armor_x + armor_y * armor_y;  // xy平面距离的平方
const double xy_sqrt = std::sqrt(xy_sq);                    // xy平面距离
const double xyz_sq = xy_sq + armor_z * armor_z;            // 三维距离的平方
const double xyz_sqrt = std::sqrt(xyz_sq);                  // 三维距离
const double z_sq=armor_z*armor_z;
// 简化后的导数计算
auto dyaw_dx = -armor_y / xy_sq;
auto dyaw_dy = armor_x / xy_sq;
auto dyaw_dz = 0.0;

auto dpitch_dx = -(armor_x * armor_z) / ((z_sq / xy_sq + 1) * std::pow(xy_sq, 1.5));
auto dpitch_dy = -(armor_y * armor_z) / ((z_sq / xy_sq + 1) * std::pow(xy_sq, 1.5));
auto dpitch_dz = 1 / ((z_sq / xy_sq + 1) * xy_sqrt);  // 用xy_sqrt替代pow(xy_sq, 0.5)

auto ddistance_dx = armor_x / xyz_sqrt;
auto ddistance_dy = armor_y / xyz_sqrt;
auto ddistance_dz = armor_z / xyz_sqrt;
 
  Eigen::MatrixXd H_armor_ypda({
    {dyaw_dx, dyaw_dy, dyaw_dz,0},
    {dpitch_dx, dpitch_dy, dpitch_dz,0},
    {ddistance_dx, ddistance_dy, ddistance_dz,0},
     {                0,                 0,                 0, 1}
  
  });
 



 
  // clang-format off

  //Eigen::MatrixXd H_armor_ypda{
   // {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
   // {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
   // {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
   // {                0,                 0,                 0, 1}
  //};
  // clang-format on

  H=H_armor_ypda * H_armor_xyza;



//卡尔曼增益计算
  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  //更新预测协方差
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();


 residual =z- Eigen::Vector4d {std::atan2(armor_y, armor_x), std::atan2(armor_z, xy_sqrt), xyz_sqrt, angle};
  residual[0] =limit_rad(residual[0]);
  residual[1] =limit_rad(residual[1]);
  residual[3] = limit_rad(residual[3]);
  x=x+K*residual;//核心融合方程
  x[6]=limit_rad(x[6]);
  /// 卡方检验
  //Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验

  
 S = H * P * H.transpose() + R;
 nis = residual.transpose() * S.inverse() * residual;
 nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  /*
  // 卡方检验阈值（自由度=4，取置信水平95%）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  */
 code=code|1<<4;
}





//ekf新增角度归一化
double ekf::limit_rad(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle <= -M_PI) angle += 2 * M_PI;
  return angle;
}




//输入观测值=========输入观测值===============输入观测值=====================输入观测值=====================输入观测值
bool ekf::observe(Eigen::Matrix3d R_armor2world,Eigen::Vector3d xyz_in_world){
//debug();
//ypr_in_world = eulers(R_armor2world, 2, 1, 0);  // 直接从相机坐标系旋转矩阵计算

//ypr[0]=
if (R_armor2world.hasNaN()) return false;
//oss<<"Z"<<R_armor2world<<xyz_in_world<<"Z";
auto yaw=eulers(R_armor2world, 2, 1, 0,false)[0];
auto x_in_world = xyz_in_world[0], y_in_world = xyz_in_world[1], z_in_world = xyz_in_world[2];
code=code|1;

//oss<<" d "<<F<<" ! "<<yaw<<" ! "<<x_in_world<<" !"<<x<<" c ";//调试字符输出代码
if(is_init){
//auto x_in_world = xyz_in_world[0], y_in_world = xyz_in_world[1], z_in_world = xyz_in_world[2];

//std::vector<Eigen::Vector4d> _armor_xyza_list;

 auto min_angle_error = 1e10;
  for (int i = 0; i < armor_num_; i++) {
    auto angle = limit_rad(x[6] + i * 2 * M_PI / armor_num_);
  

  auto use_l_h = (armor_num_ == 4) && (i == 1 || i == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];


    //_armor_xyza_list.push_back({armor_x, armor_y, armor_z, angle});



 auto angle_error = //std::abs(limit_rad(yaw - angle));//+
 std::sqrt((armor_x -x_in_world)+(armor_y -y_in_world)+(armor_z -z_in_world));// +
                       //std::abs(limit_rad(yaw -  std::atan2(armor_y, armor_x)));

    if (angle_error < min_angle_error) {
      id = i;
      min_angle_error = angle_error;
    }


    
  }
  if(id!=last_id){is_jump=1;}else {is_jump=0;}
  last_id=id;
z[0]=std::atan2(y_in_world, x_in_world);
z[1]=std::atan2(z_in_world, std::sqrt(x_in_world * x_in_world + y_in_world * y_in_world));//xyz2ypd
z[2]=std::sqrt(x_in_world * x_in_world + y_in_world * y_in_world + z_in_world * z_in_world);
z[3]=yaw;

  auto center_yaw = std::atan2(y_in_world, x_in_world);
  auto delta_angle = limit_rad(yaw - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
    log(std::abs(z_in_world) + 1) / 200 + 9e-2}};
 //Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  //测量过程噪声偏差的方差
  R = R_dig.asDiagonal();

//predict();
//update();

code=code|1<<1;
return true;
}
else{
  //oss<<"a"<<x_in_world<<y_in_world<<z_in_world;
  init();
  //oss<<"b"<<x_in_world<<y_in_world<<z_in_world;
  x[0]=x_in_world + r * std::cos(yaw);
  x[2]=y_in_world + r * std::sin(yaw);
  x[4]=z_in_world;
  x[6]=yaw;
is_jump=false;
  
  //
  id=1;
  code=code|1<<2;
  return false;
}


}

std::string ekf::debug(){
std::string debug;

    oss <<is_init<<" "<<id<<" "<<is_jump<<" "<<nis<<" "<<nees<<" "<<code<<" "<<x[0];//is_init<<" "<<x.rows() << " " << x.cols() << " " << F.rows()<<" "<<F.cols()<< " " << P.rows()<<" "<<P.cols()<< " " << Q.rows()<<" "<<Q.cols(); 
debug=oss.str();
code=0;
 oss.str("开始");    // 清空缓冲区内容
    oss.clear();    // 重置状态标志位
return debug;
//x 11 1
//F 11 11
//P 11 1  ！！！
//Q 11 11

//出现玄学bug 第一次收到传进来一堆nan碰上初始化 导致之后都是nan
}


Eigen::Vector3d ekf::eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
  Eigen::Quaterniond q(R);
  return eulers(q, axis0, axis1, axis2, extrinsic);
} 

Eigen::Vector3d ekf::eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
  if (!extrinsic) std::swap(axis0, axis2);

  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);
  if (is_proper) k = 3 - i - j;
  auto sign = (i - j) * (j - k) * (k - i) / 2;

  double a, b, c, d;
  Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  Eigen::Vector3d eulers;
  auto n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  auto half_sum = std::atan2(b, a);
  auto half_diff = std::atan2(-d, c);

  auto eps = 1e-7;
  auto safe1 = std::abs(eulers[1]) >= eps;
  auto safe2 = std::abs(eulers[1] - M_PI) >= eps;
  auto safe = safe1 && safe2;
  if (safe) {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;
  } else {
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
  }

  for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

  if (!is_proper) {
    eulers[2] *= sign;
    eulers[1] -= M_PI / 2;
  }

  if (!extrinsic) std::swap(eulers[0], eulers[2]);

  return eulers;
}