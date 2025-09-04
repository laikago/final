#include "rclcpp/rclcpp.hpp"

#include "csv_publisher/msg/csv_data.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>

// 画布参数
const int CANVAS_WIDTH = 1280;
const int CANVAS_HEIGHT = 1024;
const cv::Scalar BG_COLOR = cv::Scalar(255, 255, 255); // 白色背景
const cv::Scalar TEXT_COLOR = cv::Scalar(0, 0, 0);     // 黑色文字
const cv::Scalar LINE_COLOR = cv::Scalar(255, 0, 0);   // 蓝色线条

class SimpleCsvVisualizer : public rclcpp::Node
{
public:
    SimpleCsvVisualizer() : Node("data_processor")
    {
        RCLCPP_INFO(this->get_logger(), "Simple CSV Visualizer Started!");

        // 订阅/csv_data话题
        subscriber_ = this->create_subscription<csv_publisher::msg::CsvData>(
            "/csv_data",
            10,
            std::bind(&SimpleCsvVisualizer::data_callback, this, std::placeholders::_1)
        );

        // 初始化画布
        canvas_ = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_8UC3, BG_COLOR);
        
        // 标记是否已接收表头
        
        index_=0;
        
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
    /*
世界坐标系（3D）       
       Y↑                    
       |                         
       |                         
       o———→ X                  
      /                           
     Z (向外) */   
      width = 0.135;   // 矩形宽度（X方向）
     height = 0.055;
        object_points = {
        cv::Point3f(0,      0,       0), //左下
    cv::Point3f(0,      height,  0),          // 左上角
             cv::Point3f(width,  height,  0)  ,// 右上角
    cv::Point3f(width,  0,       0),          // 右下角
             
};
    
   
     
     
     }

    ~SimpleCsvVisualizer()
    {
        cv::destroyAllWindows();
    }

private:
    void data_callback(const csv_publisher::msg::CsvData::SharedPtr msg)
    {
        // 首次接收数据时保存表头
      //  if (!received_header_) {
            //headers_ = msg->headers;
           // received_header_ = true;
            
            // 检查是否包含timestamp列
            //auto it = std::find(headers_.begin(), headers_.end(), "timestamp");
            //if (it == headers_.end()) {
               // RCLCPP_ERROR(this->get_logger(), "CSV must contain 'timestamp' column!");
                //rclcpp::shutdown();
                //return;
            //}
            //timestamp_index_ = std::distance(headers_.begin(), it);
        //}

        // 记录当前数据和接收时间
        current_data_ = msg->data;
        //current_receive_time_ = this->now();
        process_data();

        // 绘制当前数据
        draw_current_data();
	index_++;
	RCLCPP_INFO(this->get_logger(), "接收%d",index_);
        // 显示窗口，按ESC键退出
        if (cv::waitKey(1) == 27) {
            RCLCPP_INFO(this->get_logger(), "Exit by ESC key.");
            rclcpp::shutdown();
        }
    }
	
	
	void process_data(){
	// 3. 定义对应的2D图像坐标点（像素坐标，根据实际提取结果修改）
     image_points = {
        cv::Point2f(current_data_[0],current_data_[1]),
        cv::Point2f(current_data_[2],current_data_[3]),
        cv::Point2f(current_data_[4],current_data_[5]),
        cv::Point2f(current_data_[6],current_data_[7])
    };
	
    bool success = cv::solvePnP(
        object_points, image_points,
        camera_matrix, dist_coeffs,
        rvec, tvec,false,cv::SOLVEPNP_EPNP
    );
    
//cv::SOLVEPNP_IPPE  EPNP
    if (!success) {
    RCLCPP_INFO(this->get_logger(), "solvePnP failed!");
        
        
    }
    RCLCPP_INFO(this->get_logger(), "rvec values:");
    
    for (int i = 0; i < 3; ++i) {
    RCLCPP_INFO(this->get_logger(), "rvec[%d] =%f ",i,rvec.at<double>(i, 0));
        
    }

    // 逐个读取 tvec 的值
    RCLCPP_INFO(this->get_logger(), "tvec values:");
    for (int i = 0; i < 3; ++i) {
        RCLCPP_INFO(this->get_logger(), "tvec[%d] =%f ",i,tvec.at<double>(i, 0));
    }
    
    
	//RCLCPP_INFO(this->get_logger(), "solvePnP %f,%f,%f,%f,%f,%f",rvec.at<double>(0, 0),rvec[1],rvec[2],tvec[0],tvec[1],tvec[2]);
	
	
	}
    // 绘制当前数据
    void draw_current_data()
    {
        // 清空画布
        canvas_.setTo(BG_COLOR);

        // 绘制标题
        //cv::putText(canvas_, "CSV Data Visualization", 
                    //cv::Point(20, 30), 
                    //cv::FONT_HERSHEY_SIMPLEX, 1.0, TEXT_COLOR, 2);

        // 绘制发布时间（节点接收时间）
        //std::string receive_time_str = "Receive Time: " + 
            //current_receive_time_.seconds() < 1e9 ? 
            //current_receive_time_.to_msg().sec + "." + //std::to_string(current_receive_time_.to_msg().nanosec / 1e6) :
            //"N/A";
        //cv::putText(canvas_, receive_time_str, 
                   // cv::Point(20, 70), 
                    //cv::FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 1);

        // 绘制数据时间戳（CSV文件中的时间）
       // if (timestamp_index_ < current_data_.size()) {
        //    std::stringstream ss;
        //    ss << std::fixed << std::setprecision(3) << current_data_[timestamp_index_];
        //    std::string timestamp_str = "Data Timestamp: " + ss.str();
         //   cv::putText(canvas_, timestamp_str, 
        //                cv::Point(20, 100), 
         //               cv::FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 1);
       // }

        // 绘制分隔线
        
        /*
        cv::line(canvas_, 
                 cv::Point(20, 120), 
                 cv::Point(CANVAS_WIDTH - 20, 120), 
                 LINE_COLOR, 1);

        // 绘制数据列表
        int y_pos = 160;
        for (size_t i = 0;  i < current_data_.size(); ++i) {
            // 格式化数据，保留3位小数
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3) << current_data_[i];
            
            // 绘制每个数据项
            std::string data_str =  ss.str();
            cv::putText(canvas_, data_str, 
                        cv::Point(40, y_pos), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 1);
            
            y_pos += 30;
            // 防止超出画布
            
        }
	*/
	
												/*绘图结果看到的 
(0,0) ——————————→ x (宽度方向，列)
  |
  |
  ↓ y (高度方向，行)*/
	cv::circle(canvas_,cv::Point(current_data_[0],current_data_[1]),5,cv::Scalar(0,0,0),-1);//黑 左下
	cv::circle(canvas_,cv::Point(current_data_[2],current_data_[3]),5,cv::Scalar(255,0,0),-1);//蓝 左上
	cv::circle(canvas_,cv::Point(current_data_[4],current_data_[5]),5,cv::Scalar(0,255,0),-1);//绿 右上
	cv::circle(canvas_,cv::Point(current_data_[6],current_data_[7]),5,cv::Scalar(0,0,255),-1);//红 右下
  


        // 显示操作提示
        //cv::putText(canvas_, "Press ESC to exit", 
             //       cv::Point(20, CANVAS_HEIGHT - 20), 
               //     cv::FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 1);

        // 显示画布
        cv::imshow("Simple CSV Data Visualization", canvas_);
    }

    // 成员变量
    rclcpp::Subscription<csv_publisher::msg::CsvData>::SharedPtr subscriber_;
    cv::Mat canvas_;
    //std::vector<std::string> headers_;
    std::vector<unsigned short> current_data_;
    //rclcpp::Time current_receive_time_;
    //bool received_header_;
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleCsvVisualizer>());
    rclcpp::shutdown();
    return 0;
}
    
