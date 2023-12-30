#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std;
using namespace chrono_literals;

class occupancygrid_pub : public rclcpp::Node
{
    public: occupancygrid_pub() : Node("occupancy_grid_pub")
        {
            occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occ_grid",10);
            occgrid_timer_ = this->create_wall_timer(500ms, bind(&occupancygrid_pub::occu_grid_callback,this));
        }
    private:

      rclcpp::TimerBase::SharedPtr occgrid_timer_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

      void occu_grid_callback()
      {
        auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
        std::vector<signed char> og_array(45); //5*9 = 45 (width*height)
        for(int i=0;i<45;i++){
            og_array[i] = i % 2 == 0 ? 100 : -1 ; 
            /* 
            The ternary operator ? : is used for a conditional expression. 
            If the condition i % 2 == 0 is true, then og_array[i] is assigned the value 100; otherwise, it is assigned the value 0.
             */
        }

        occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
        occupancy_grid_msg.header.frame_id = "map";

        occupancy_grid_msg.info.resolution = 1;

        occupancy_grid_msg.info.width = 5;
        occupancy_grid_msg.info.height = 9;

        occupancy_grid_msg.info.origin.position.x = 0.0;
        occupancy_grid_msg.info.origin.position.y = 0.0;
        occupancy_grid_msg.info.origin.position.z = 0.0;
        occupancy_grid_msg.info.origin.orientation.x = 0.0;
        occupancy_grid_msg.info.origin.orientation.y = 0.0;
        occupancy_grid_msg.info.origin.orientation.z = 0.0;
        occupancy_grid_msg.info.origin.orientation.w = 1.0;
        occupancy_grid_msg.data = og_array;

        occupancy_grid_pub_->publish(occupancy_grid_msg);
      }
      

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<occupancygrid_pub>());
  rclcpp::shutdown();
  return 0;
}