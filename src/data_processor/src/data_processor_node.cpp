#include "rclcpp/rclcpp.hpp"
#include "data_processor/msg/output.hpp"
#include "csv_publisher/msg/csv_data.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <fstream> 

#include "data_processor/ekf.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>  // 用于cv::cv2eigen

#include <yaml-cpp/yaml.h>  // 用于YAML::LoadFile

//后续增加读取功能
/*
//读取相机内参 需要完善
#include "yaml-cpp/yaml.h"  // 需要引入yaml-cpp库

void loadConfigFromYaml(const std::string& file_path) {
  YAML::Node config = YAML::LoadFile(file_path);
  // 读取参数
  double radius = config["armor_radius"].as<double>();
  int armor_num = config["armor_num"].as<int>();
  // 应用参数...
}

int main(int argc, char**argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("target_node");
  
  // 读取YAML配置文件
  std::string config_path = "/path/to/config.yaml";
  loadConfigFromYaml(config_path);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/





//svm mlp lenet lenet5 resnet  训练 验证  mnist








//重投影检验需要 取消注释 重投影代码、绘制代码、重投影结果变量声明
// 画布参数
/*
const int CANVAS_WIDTH = 1280;
const int CANVAS_HEIGHT = 1024;
const cv::Scalar BG_COLOR = cv::Scalar(255, 255, 255); // 白色背景
const cv::Scalar TEXT_COLOR = cv::Scalar(0, 0, 0);     // 黑色文字
const cv::Scalar LINE_COLOR = cv::Scalar(255, 0, 0);   // 蓝色线条
*/
class DataProcessor : public rclcpp::Node
{
public:
//构造函数
    DataProcessor() : Node("data_processor")//,ekf_()
    {
        RCLCPP_INFO(this->get_logger(), "Simple CSV Visualizer Started!");

        // 订阅/csv_data话题
        subscriber_ = this->create_subscription<csv_publisher::msg::CsvData>(
            "/csv_data",
            10,
            std::bind(&DataProcessor::data_callback, this, std::placeholders::_1)
        );
        //创建发布者
        publisher_ = this->create_publisher<data_processor::msg::Output>("output", 10);
        
        index_=0;




        // 1. 定义配置文件路径（与你的路径一致）
std::string config_path = "/home/r/ros2_1/src/data_processor/config/correct_camera_info.yaml";

// 2. 检查 YAML 文件是否存在（避免 "YAML::BadFile" 错误）
if (!std::ifstream(config_path)) {
    RCLCPP_ERROR(this->get_logger(), "相机配置文件不存在！路径：%s", config_path.c_str());
    rclcpp::shutdown(); // 文件缺失时主动退出，避免崩溃
    return;
}

// 3. 加载 YAML 文件
YAML::Node yaml_node;
try {
    yaml_node = YAML::LoadFile(config_path);
} catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML 文件解析失败！错误：%s", e.what());
    rclcpp::shutdown();
    return;
}

// 4. 读取相机内参矩阵（camera_matrix -> data）
camera_matrix=cv::Mat(3, 3, CV_64F); // 3x3 内参矩阵（CV_64F 对应 double 类型）
try {
    // 读取 camera_matrix 的 data 数组（YAML 中是 std::vector<double>）
    std::vector<double> cam_mat_data = yaml_node["camera_matrix"]["data"].as<std::vector<double>>();
    // 检查 data 数组长度是否为 9（3x3 矩阵共 9 个元素）
    if (cam_mat_data.size() != 9) {
        RCLCPP_ERROR(this->get_logger(), "相机内参 data 长度错误！应为9，实际为%d", cam_mat_data.size());
        rclcpp::shutdown();
        return;
    }
    // 将 vector 数据赋值给 cv::Mat（按行优先，与 YAML 中 data 顺序一致）
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camera_matrix.at<double>(i, j) = cam_mat_data[i * 3 + j];
        }
    }
    RCLCPP_INFO(this->get_logger(), "成功读取相机内参矩阵：");
    RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", camera_matrix.at<double>(0,0), camera_matrix.at<double>(0,1), camera_matrix.at<double>(0,2));
    RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", camera_matrix.at<double>(1,0), camera_matrix.at<double>(1,1), camera_matrix.at<double>(1,2));
    RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f", camera_matrix.at<double>(2,0), camera_matrix.at<double>(2,1), camera_matrix.at<double>(2,2));
} catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "读取相机内参失败！错误：%s", e.what());
    rclcpp::shutdown();
    return;
}

// 5. 读取畸变系数（distortion_coefficients -> data）
 dist_coeffs=cv::Mat(5, 1, CV_64F); // 5x1 畸变系数（k1, k2, p1, p2, k3）
try {
    // 读取 distortion_coefficients 的 data 数组
    std::vector<double> dist_data = yaml_node["distortion_coefficients"]["data"].as<std::vector<double>>();
    // 检查 data 数组长度是否为 5（5个畸变系数）
    if (dist_data.size() != 5) {
        RCLCPP_ERROR(this->get_logger(), "畸变系数 data 长度错误！应为5，实际为%d", dist_data.size());
        rclcpp::shutdown();
        return;
    }
    // 赋值给 5x1 矩阵（按顺序对应 k1, k2, p1, p2, k3）
    for (int i = 0; i < 5; ++i) {
        dist_coeffs.at<double>(i, 0) = dist_data[i];
    }
    RCLCPP_INFO(this->get_logger(), "成功读取畸变系数：[%.6f, %.6f, %.6f, %.6f, %.6f]",
                dist_coeffs.at<double>(0,0), dist_coeffs.at<double>(1,0),
                dist_coeffs.at<double>(2,0), dist_coeffs.at<double>(3,0),
                dist_coeffs.at<double>(4,0));
} catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "读取畸变系数失败！错误：%s", e.what());
    rclcpp::shutdown();
    return;
}

// （可选）读取图像宽高（若后续需要用到）
int image_width = yaml_node["image_width"].as<int>();
int image_height = yaml_node["image_height"].as<int>();
RCLCPP_INFO(this->get_logger(), "图像分辨率：%dx%d", image_width, image_height);




        /*
        std::string config_path = "/home/r/ros2_1/src/data_processor/config/correct_camera_info.yaml";
         auto yaml = YAML::LoadFile(config_path);
        // 4. 相机内参矩阵（使用提供的实际相机参数）
     camera_matrix = (cv::Mat_<double>(3, 3) <<
        1307.16695,    0.0,     648.51847,
        0.0,     1304.73248,  503.16415,
        0.0,        0.0,        1.0
    );

    // 5. 畸变系数（使用提供的实际畸变参数）
    dist_coeffs = (cv::Mat_<double>(5, 1) << 
        -0.046674,  // k1
        0.044424,   // k2
        -0.001226,  // p1
        0.002452,   // p2
        0.000000    // k3
    );
   
   */
   
    /*
世界坐标系（3D）       
       Y↑                    
       |                         
       |                         
       o———→ X                  
      /                           
     Z (向外) */   
     //单位米
      width = 0.0675;//总长0.135;   // 矩形宽度（X方向）
     height = 0.0275;//总高0.055;  //改成一半长度高度 坐标原点在装甲板中心
        object_points = {
        cv::Point3f(-width,      -height,       0), //左下
    cv::Point3f(-width,      height,  0),          // 左上角
             cv::Point3f(width,  height,  0)  ,// 右上角
    cv::Point3f(width,  -height,       0),          // 右下角
    };

    //初始化ekf
    ekf_.r=0.2;
    ekf_.dt=0.01;
 
    
   //构造函数完
     
     
     }

    //~DataProcessor()
    //{
    
    //}

private:
//收到信息执行====================收到信息执行========================收到信息执行===========================收到信息执行
    void data_callback(const csv_publisher::msg::CsvData::SharedPtr msg)
    {
 
        current_data_ = msg->data;
        
        process_data();  //其实是pnp解算

        if(ekf_.observe(R_armor2camera,xyz_in_world)){
        
            /*
	
	RCLCPP_INFO(this->get_logger(), "接收%d",index_);
    RCLCPP_INFO(this->get_logger(), "打印调试信息%s",ekf_.debug().c_str());
        // 显示窗口，按ESC键退出
        if (cv::waitKey(1) == 27) {
            RCLCPP_INFO(this->get_logger(), "Exit by ESC key.");
            rclcpp::shutdown();
        }
    */
        RCLCPP_INFO(this->get_logger(), "初始化成功");
    auto output_msg = data_processor::msg::Output();
    Eigen::Vector3d euler=ekf_.eulers(R_armor2camera, 2, 1, 0,false);
   double yaw=euler[0];
output_msg.x_input= xyz_in_world[0];
output_msg.y_input =xyz_in_world[1];
output_msg.z_input = xyz_in_world[2];
output_msg.pitch_input = euler[1];
output_msg.yaw_input =euler[2];




double x=xyz_in_world[0]+ekf_.r * std::cos(yaw);
double y=xyz_in_world[1]+ekf_.r * std::sin(yaw);
double z=xyz_in_world[2];
output_msg.x_measurement = x;
output_msg.y_measurement =y;
output_msg.z_measurement = z;
output_msg.pitch_measurement = std::atan2(z, std::sqrt(x * x + y * y));
output_msg.yaw_measurement = std::atan2(y, x);

ekf_.predict();

x=ekf_.x[0];
y=ekf_.x[2];
z=ekf_.x[4];
output_msg.x_pred = x;
output_msg.y_pred =y;
output_msg.z_pred = z;
output_msg.pitch_pred = std::atan2(z, std::sqrt(x * x + y * y));
output_msg.yaw_pred = std::atan2(y, x);


ekf_.update();



x=ekf_.x[0];
y=ekf_.x[2];
z=ekf_.x[4];
output_msg.x_est = x;
output_msg.y_est =y;
output_msg.z_est = z;
output_msg.pitch_est = std::atan2(z, std::sqrt(x * x + y * y));
output_msg.yaw_est = std::atan2(y, x);



    publisher_->publish(output_msg);


        }





	RCLCPP_INFO(this->get_logger(), "接收%d",index_);
    RCLCPP_INFO(this->get_logger(), "打印调试信息%s",ekf_.debug().c_str());

index_++;








    }
	
	
	void process_data(){
	// 3. 定义对应的2D图像坐标点（像素坐标，根据实际提取结果修改）
     image_points = {
        cv::Point2f(current_data_[0],current_data_[1]),
        cv::Point2f(current_data_[2],current_data_[3]),
        cv::Point2f(current_data_[4],current_data_[5]),
        cv::Point2f(current_data_[6],current_data_[7])
    };
	//pnp解算
    bool success = cv::solvePnP(
        object_points, image_points,
        camera_matrix, dist_coeffs,
        rvec, tvec,false,cv::SOLVEPNP_IPPE
    );
    
//cv::SOLVEPNP_IPPE 这个用了很准，基本不偏  EPNP 这个有时准确有时不准确      
    if (!success) {
    RCLCPP_INFO(this->get_logger(), "solvePnP failed!");
        
        
    }


  cv::cv2eigen(tvec, xyz_in_world);
//简化世界坐标转化
// 1. 旋转矩阵转换简化（省略云台和世界坐标系的冗余转换）

cv::Rodrigues(rvec, rmat);  // 旋转向量转旋转矩阵（相机坐标系下）

cv::cv2eigen(rmat, R_armor2camera);  // OpenCV矩阵转Eigen矩阵


    
}
/*
cv::projectPoints(
    object_points,     // 原始 3D 点
    rvec, tvec,       // 刚求得的姿态
    camera_matrix,
    dist_coeffs,
    reprojectedPoints // 输出：重投影后的 2D 点
);*/
	
    // 成员变量
    rclcpp::Subscription<csv_publisher::msg::CsvData>::SharedPtr subscriber_;
    rclcpp::Publisher<data_processor::msg::Output>::SharedPtr publisher_;//发布者
    
    std::vector<unsigned short> current_data_;
   
    size_t index_;
    
    //相机参数
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs ;
    //pnp相关
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;
    cv::Mat rvec, tvec;
    double width ;   // 矩形宽度（X方向）
    double height;
    //新增转换数据传入ekf
Eigen::Vector3d xyz_in_world;//世界坐标
cv::Mat rmat;//旋转矩阵
Eigen::Matrix3d R_armor2camera;//旋转矩阵
  
    //重投影后的二维点
    //std::vector<cv::Point2f> reprojectedPoints;
  
//ekf
ekf ekf_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataProcessor>());
    rclcpp::shutdown();
    return 0;
}
    
