#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "csv_publisher/msg/csv_data.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

class CsvPublisher : public rclcpp::Node
{
public:
    CsvPublisher() : Node("csv_publisher")
    {
        // 声明CSV文件路径参数
        this->declare_parameter<string>("csv_file_path", "");
        //RCLCPP_INFO(this->get_logger(),"%s", csv_file_path .c_str());
        // 获取参数值
        string csv_file_path = this->get_parameter("csv_file_path").as_string();
        RCLCPP_INFO(this->get_logger(),"你好%s", csv_file_path .c_str());
        if (csv_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "CSV file path is not specified!");
            return;
        }
        
        // 创建发布者
        publisher_ = this->create_publisher<csv_publisher::msg::CsvData>("csv_data", 10);
        
        // 读取CSV文件并发布数据
        if (readCsvFile(csv_file_path)) {
            timer_ = this->create_wall_timer(
                500ms, // 每500ms发布一次
                bind(&CsvPublisher::publishData, this)
            );
        }
    }

private:
    void publishData()
    {
        if (current_row_ >= data_rows_.size()) {
            RCLCPP_INFO(this->get_logger(), "All data published. Restarting...");
            current_row_ = 0;
        }
        
        auto msg = csv_publisher::msg::CsvData();
        
        msg.data = data_rows_[current_row_];
        
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing row %d", current_row_ + 1);
        
        current_row_++;
    }
    
    bool readCsvFile(const string& file_path)
    {
        ifstream file(file_path);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", file_path.c_str());
            return false;
        }
        
        string line;
        
        
        // 读取CSV文件内容
        while (getline(file, line)) {
            stringstream ss(line);
            string cell;
            vector<unsigned short> row_data;
            
            // 解析一行数据
            while (getline(ss, cell, ',')) {
                 
                    // 其他行作为数据
                    try {
                        row_data.push_back(stod(cell));
                        //RCLCPP_INFO(this->get_logger(), "哈哈%s",cell);
                    } catch (...) {
                        RCLCPP_WARN(this->get_logger(), "Invalid data format: %s", cell.c_str());
                        row_data.push_back(0.0);
                    }
                
            }
            
            if ( !row_data.empty()) {
                data_rows_.push_back(row_data);
            }
            
            
        }
        
        file.close();
        
        if (data_rows_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "CSV file is empty or invalid");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "哈哈%d,%d",data_rows_.size(),data_rows_[0].size());
        
        RCLCPP_INFO(this->get_logger(), "Successfully read CSV file. Rows: %d, Columns: %d", 
                   (int)data_rows_.size(), (int)data_rows_[0].size());
        return true;
    }
    
    rclcpp::Publisher<csv_publisher::msg::CsvData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    vector<vector<unsigned short>> data_rows_;
    size_t current_row_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<CsvPublisher>());
    rclcpp::shutdown();
    return 0;
}
