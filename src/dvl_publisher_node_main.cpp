#include "dvl_publisher_node.h"

int main(int argc, char * argv[])
{


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DVLPublisher>());
    rclcpp::shutdown();
    return 0;
}