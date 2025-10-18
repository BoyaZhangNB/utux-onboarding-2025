#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::vector;

class Sonar_Subscriber: public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sonar_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dist_pub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr test_sub_;
        std::vector<float> prev_front;
        std::vector<float> prev_bottom;

        void sonar_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg){
            if (msg->data[0] == -1 or msg->data[1] == -1){
                RCLCPP_INFO(this->get_logger(), "Sensor timed out");
            }

            float front = msg->data[0] / 1.481; // [m]
            float bottom = msg->data[1] / 1.481; // [m]
            //RCLCPP_INFO(this->get_logger(), "front: %f m, bottom: %f m]", front, bottom);

            prev_front.push_back(front);
            prev_bottom.push_back(bottom);
            
            // Keep only the last 5 values
            if (prev_front.size() > 5) {
                prev_front.erase(prev_front.begin());
            }
            if (prev_bottom.size() > 5) {
                prev_bottom.erase(prev_bottom.begin());
            }
            
            if (prev_front.size() >= 5){
                std_msgs::msg::Float32MultiArray pub_msg;
                pub_msg.data = {filter_data(front, prev_front), filter_data(bottom, prev_bottom)};
                dist_pub_->publish(pub_msg);
                //RCLCPP_INFO(this->get_logger(), "Published filtered data: front=%f, bottom=%f", pub_msg.data[0], pub_msg.data[1]);
            }

        }

        float filter_data(float current_data, std::vector<float> prev_data){
            float data_sum = 0.0;
            float data_var = 0.0;

            for(float data : prev_data){ // calculate mean
                data_sum += data;
            }

            float average = data_sum / prev_data.size();

            for (float data : prev_data){
                data_var += pow((data - average), 2) / prev_data.size();
            }

            float data_sd = sqrt(data_var);

            if (current_data > average + data_sd){
                return average + data_sd;
            } 
            else if (current_data < average - data_sd){
                return average - data_sd;
            }
            else{
                return current_data;
            }
        }

        void test_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Receiving data: [front: %f m, bottom: %f m]", msg->data[0], msg->data[1]);
        }
    
    public:
        Sonar_Subscriber(): Node("sonar_subscriber"){
            sonar_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("sonar/data", 10, std::bind(&Sonar_Subscriber::sonar_callback, this, std::placeholders::_1));
            test_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("sonar/distance",10,std::bind(&Sonar_Subscriber::test_callback, this, std::placeholders::_1));
            dist_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar/distance",10);
        }
        
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sonar_Subscriber>());
    rclcpp::shutdown();
    return 0;
}