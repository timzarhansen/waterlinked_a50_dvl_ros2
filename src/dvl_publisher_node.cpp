#include "dvl_publisher_node.h"


int sock;


void DVLPublisher::connectToDVL(){
    struct addrinfo hints, *res;

    memset(&hints, '\0', sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(IP_.c_str(), port_.c_str(), &hints, &res); // Move IP and PORT values to parameter server

    try{
        sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (connect(sock, res->ai_addr, res->ai_addrlen) < 0){
            RCLCPP_INFO(this->get_logger(), "Connection failed");
            throw connect(sock, res->ai_addr, res->ai_addrlen);
        }else{
            RCLCPP_INFO(this->get_logger(), "Connection succeeded");
        }
    }
    catch (int except) {
        RCLCPP_INFO(this->get_logger(), "Trying to connect again");
        connectToDVL();
    }
}

std::string DVLPublisher::getData(){
    // Adding '\0' such that only the data character is processed.
    char buf[2];
    buf[1] ='\0';
    int rec;

    std::string raw_data = "";

    while (raw_data.back() != '\n'){
        rec = recv(sock,buf,sizeof(char),0);
        if (rec == 0){
            RCLCPP_INFO(this->get_logger(), "Socket closed by the DVL, reopening");
            this->connectToDVL();
            // Add exception handling
        }
        raw_data = raw_data + buf;
    }
    return raw_data;
}

void DVLPublisher::publisher(){
    std::string raw_data = this->getData();

    nlohmann::json json_data = nlohmann::json::parse(raw_data);
    waterlinked_a50::msg::TransducerReportStamped dvlReport = waterlinked_a50::msg::TransducerReportStamped();

    if (json_data["type"] == "velocity"){
        dvlReport.report.time                 = json_data["time"];
       dvlReport.report.vx          = json_data["vx"];
       dvlReport.report.vy           = json_data["vy"];
       dvlReport.report.vz           = json_data["vz"];
       dvlReport.report.fom                  = json_data["fom"];

        for (int i = 0; i < 9; i++){
            dvlReport.report.covariance[i]    = json_data["covariance"][i/3][i%3];}

        dvlReport.report.altitude             = json_data["altitude"];
        dvlReport.report.velocity_valid       = json_data["velocity_valid"];
        dvlReport.report.status               = json_data["status"];
        dvlReport.report.time_of_validity     = json_data["time_of_validity"];
        dvlReport.report.time_of_transmission = json_data["time_of_transmission"];
        dvlReport.report.format                 = json_data["format"];
        dvlReport.report.type                 = json_data["type"];


        dvlReport.report.transducers[0].id                = json_data["transducers"][0]["id"];
        dvlReport.report.transducers[0].velocity          = json_data["transducers"][0]["velocity"];
        dvlReport.report.transducers[0].distance          = json_data["transducers"][0]["distance"];
        dvlReport.report.transducers[0].rssi              = json_data["transducers"][0]["rssi"];
        dvlReport.report.transducers[0].nsd               = json_data["transducers"][0]["nsd"];
        dvlReport.report.transducers[0].beam_valid             = json_data["transducers"][0]["beam_valid"];

        dvlReport.report.transducers[1].id                = json_data["transducers"][1]["id"];
        dvlReport.report.transducers[1].velocity          = json_data["transducers"][1]["velocity"];
        dvlReport.report.transducers[1].distance          = json_data["transducers"][1]["distance"];
        dvlReport.report.transducers[1].rssi              = json_data["transducers"][1]["rssi"];
        dvlReport.report.transducers[1].nsd               = json_data["transducers"][1]["nsd"];
        dvlReport.report.transducers[1].beam_valid             = json_data["transducers"][1]["beam_valid"];

        dvlReport.report.transducers[2].id                = json_data["transducers"][2]["id"];
        dvlReport.report.transducers[2].velocity          = json_data["transducers"][2]["velocity"];
        dvlReport.report.transducers[2].distance          = json_data["transducers"][2]["distance"];
        dvlReport.report.transducers[2].rssi              = json_data["transducers"][2]["rssi"];
        dvlReport.report.transducers[2].nsd               = json_data["transducers"][2]["nsd"];
        dvlReport.report.transducers[2].beam_valid             = json_data["transducers"][2]["beam_valid"];

        dvlReport.report.transducers[3].id                = json_data["transducers"][3]["id"];
        dvlReport.report.transducers[3].velocity          = json_data["transducers"][3]["velocity"];
        dvlReport.report.transducers[3].distance          = json_data["transducers"][3]["distance"];
        dvlReport.report.transducers[3].rssi              = json_data["transducers"][3]["rssi"];
        dvlReport.report.transducers[3].nsd               = json_data["transducers"][3]["nsd"];
        dvlReport.report.transducers[3].beam_valid             = json_data["transducers"][3]["beam_valid"];

//        theDVL.beams            = {beam0, beam1, beam2, beam3};
        dvlReport.timestamp     = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
        velocity_publisher_->publish(dvlReport);

    }else{
        waterlinked_a50::msg::PositionReportStamped dvlPositionReport = waterlinked_a50::msg::PositionReportStamped();

        dvlPositionReport.report.roll     = json_data["roll"];
        dvlPositionReport.report.pitch    = json_data["pitch"];
        dvlPositionReport.report.yaw      = json_data["yaw"];
        dvlPositionReport.report.x        = json_data["x"];
        dvlPositionReport.report.y        = json_data["y"];
        dvlPositionReport.report.z        = json_data["z"];
        dvlPositionReport.report.status   = json_data["status"];
        dvlPositionReport.report.std      = json_data["std"];

        dvlPositionReport.report.time       = json_data["ts"];
        dvlPositionReport.report.format     = json_data["format"];
        dvlPositionReport.report.type     = json_data["type"];
        dvlPositionReport.timestamp = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();

        odometry_publisher_->publish(dvlPositionReport);
    }
}