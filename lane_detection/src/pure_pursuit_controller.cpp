#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <cmath>
#include <chrono>
#include <sensor_msgs/msg/nav_sat_fix.hpp> 

class PurePursuitController : public rclcpp::Node {
public:
  PurePursuitController()
  : Node("pure_pursuit_controller"),
    in_turn_(false),
    target_lat_(0.167724),             // Hedef enlem
    target_lon_(0.104306),             // Hedef boylam
    gnss_tol_(5e-6)                    // Tolerans
  {
    
    declare_parameter<double>("base_throttle",    0.3);
    declare_parameter<double>("min_throttle",     0.05);
    declare_parameter<double>("turn_min_slope",   0.1);
    declare_parameter<double>("turn_max_slope",   0.38);
    declare_parameter<int>("entry_hold",         5);
    declare_parameter<int>("exit_hold",          5);
    declare_parameter<double>("max_turn_steer",   1.0);
    declare_parameter<double>("center_Kp",        0.0025);
    declare_parameter<int>("image_width",         800);

    get_parameter("base_throttle",  base_throttle_);
    get_parameter("min_throttle",   min_throttle_);
    get_parameter("turn_min_slope", turn_min_slope_);
    get_parameter("turn_max_slope", turn_max_slope_);
    get_parameter("entry_hold",     entry_hold_);
    get_parameter("exit_hold",      exit_hold_);
    get_parameter("max_turn_steer", max_turn_steer_);
    get_parameter("center_Kp",      center_Kp_);
    get_parameter("image_width",    image_width_);

    RCLCPP_INFO(get_logger(), "Parameters: turn_min=%.2f, turn_max=%.2f, entry_hold=%d, exit_hold=%d",
      turn_min_slope_, turn_max_slope_, entry_hold_, exit_hold_);

    center_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/lane_center", 10,
      std::bind(&PurePursuitController::center_callback, this, std::placeholders::_1)
    );
    left_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/left_slope",  10,
      std::bind(&PurePursuitController::left_callback, this, std::placeholders::_1)
    );
    right_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/right_slope", 10,
      std::bind(&PurePursuitController::right_callback, this, std::placeholders::_1)
    );

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/carla/ego_vehicle/gnss", 10,
      std::bind(&PurePursuitController::gnss_callback, this, std::placeholders::_1)
    );

    control_pub_ = create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
      "/carla/ego_vehicle/vehicle_control_cmd", 10
    );

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PurePursuitController::timer_callback, this)
    );

    RCLCPP_INFO(get_logger(), "Pure Pursuit Controller Started. ");
  }

private:
  
  void center_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    last_center_ = msg->data; has_center_ = true;
  }
  void left_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    last_left_slope_ = msg->data; has_left_ = true;
  }
  void right_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    last_right_slope_ = msg->data; has_right_ = true;
  }

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    double lat = msg->latitude;
    double lon = msg->longitude;
    if (std::fabs(lat - target_lat_) < gnss_tol_ && //fabs mutlak değeri alır
        std::fabs(lon - target_lon_) < gnss_tol_) 
    {
      RCLCPP_INFO(get_logger(),
        "GNSS target (%f, %f) reached. Sending brake command and shutting down node.",
       target_lat_, target_lon_);
     
     carla_msgs::msg::CarlaEgoVehicleControl stop_cmd;
      stop_cmd.throttle = 0.0;
      stop_cmd.brake    = 1.0;   
      stop_cmd.steer    = 0.0;
      control_pub_->publish(stop_cmd);
    
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::shutdown();

    }
  }

  void timer_callback() {
    double slope{0.0}; int cnt{0};
    if (has_left_ && !has_right_)  { slope = last_left_slope_;  cnt = 1; } // cnt=1 bilginin tek taraftan geldiğini gösterir
    if (has_right_ && !has_left_)  { slope = last_right_slope_; cnt = 1; }
    if (has_left_ && has_right_)   { slope = 0.5*(last_left_slope_+last_right_slope_); cnt = 2; } // sağ ve sol şeridin ort alıyor: daha doğru sonuç

    RCLCPP_INFO(get_logger(), "Current slope: %.3f", slope);

    bool in_range = (std::abs(slope)>=turn_min_slope_ && std::abs(slope)<=turn_max_slope_);
    static int entry_counter{0}, exit_counter{0};

    if (!in_turn_) {
      if (cnt>0 && in_range) entry_counter++;
      else entry_counter=0;

      if (entry_counter>=entry_hold_) {
        in_turn_=true; 
        exit_counter=0;
        turn_direction_ = (slope>0?-1.0:1.0);
        RCLCPP_INFO(get_logger(), "Entered turn mode (slope=%.2f)", slope);
      }
    }

    double steer{0.0}, throttle{base_throttle_}, brake{0.0};

    if (in_turn_) {
      steer = turn_direction_*max_turn_steer_;
      throttle = std::max(min_throttle_, base_throttle_*0.5);
      if (cnt>0 && !in_range)
     exit_counter++;
     else exit_counter=0;
      if (exit_counter>=exit_hold_) {
        in_turn_=false;
        entry_counter=0;
        RCLCPP_INFO(get_logger(), "Exited turn mode (slope=%.2f)", slope);
      }
    }
     else {
      if (has_left_ && !has_right_)       steer = -std::atan(last_left_slope_); // atan eğimi radyandan açıya çevirir
      else if (has_right_ && !has_left_)  steer = -std::atan(last_right_slope_);
      else if (has_left_&&has_right_&&has_center_) {
        double err = last_center_ - (image_width_/2.0); //err: görüntüdeki şerit merkezi ile kamera görüntüsünün ortası arasındaki fark 
        steer = -center_Kp_*err;
      }
    }

    auto cmd = carla_msgs::msg::CarlaEgoVehicleControl(); //cmd ros2 mesajıdır.
    cmd.throttle=throttle; cmd.steer=steer; cmd.brake=brake;
    control_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(),
      "[Control] steer=%.3f, throttle=%.3f, brake=%.3f, in_turn=%d",
      steer,throttle,brake,in_turn_);

    has_left_=has_right_=has_center_=false; // Her timer callback döngüsünde sıfıranır. Yeni veriler alır.
  }

 
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   center_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;  
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool in_turn_;
  double turn_direction_{0.0};
  bool has_center_{false},has_left_{false},has_right_{false};
  int last_center_{0},image_width_{800};
  double last_left_slope_{0.0},last_right_slope_{0.0};

  double base_throttle_,min_throttle_,k_throttle_;
  double turn_min_slope_,turn_max_slope_;
  int entry_hold_,exit_hold_;
  double max_turn_steer_,center_Kp_;

  const double target_lat_, target_lon_, gnss_tol_;
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
