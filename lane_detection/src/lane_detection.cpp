#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

class LaneFollowingNode : public rclcpp::Node {
public:
  LaneFollowingNode() : Node("lane_following_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/carla/ego_vehicle/rgb_view/image", 10,
      std::bind(&LaneFollowingNode::image_callback, this, std::placeholders::_1)
    );

    center_pub_ = this->create_publisher<std_msgs::msg::Int32>("/lane_center", 10);
    left_slope_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_slope", 10);
    right_slope_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_slope", 10);

    cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Lane Detection", 1280, 720);

    RCLCPP_INFO(this->get_logger(), "Lane Detection Node Started");
  }

private:
  double last_safe_cx_ = 400.0;  // Güvenli x
  double center_Kp_ = 0.0025;    // Hizalama katsayısı
  bool has_full_lane_ = false;   // Hem sol hem sağ şerit var mı?

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // Görüntü OpenCv ile işlenebilir.
    cv::Mat frame = cv_ptr->image;
    int width = frame.cols;
    int height = frame.rows;

    cv::Mat blurred, gray, edges;
    cv::GaussianBlur(frame, blurred, cv::Size(5,5), 0); //görüntüyü yumuşatır
    cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY); //gri tonlama
    cv::Canny(gray, edges, 50, 150); //kenarları algılar

    int roi_top = static_cast<int>(height * 2.0 / 3.0);
    int left_bound = static_cast<int>(width * 0.95 / 6.0);
    int right_bound = static_cast<int>(width * 3.15 / 5.0);
    cv::Mat mask = cv::Mat::zeros(edges.size(), edges.type());

    cv::Point roi_poly[4] = {
      {left_bound, height},
      {right_bound, height},
      {right_bound, roi_top},
      {left_bound, roi_top}
    };
    cv::fillConvexPoly(mask, roi_poly, 4, cv::Scalar(255));
    cv::Mat roi_edges;
    cv::bitwise_and(edges, mask, roi_edges);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(roi_edges, lines, 1, CV_PI/180, 20, 20, 10); // min oy, min çizgi uzunluğu, max çizgiler arası boşluk

    std::vector<cv::Vec4i> left_lines, right_lines;
    std::vector<double> right_centers, right_slopes;

    for (const auto& ln : lines) {
      int x1 = ln[0], y1 = ln[1], x2 = ln[2], y2 = ln[3]; //çizginin iki ucu (başlangıç ve bitiş noktaları) ayrıştırılıyor.
      double slope = (y2 - y1) / (double(x2 - x1) + 1e-6);
      int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
      if (dx < 15 || dy < 30) //çok kısa yatay ya da dikey çizgiler şerit değildir.
      continue;
      if (std::abs(slope) < 0.5 || std::abs(slope) > 2.0)  // eğimi çok yatık ya da dik çizgiler şerit değildir.
      continue;

      int cx = (x1 + x2) / 2; //cx, çizginin merkeze ne kadar yakın olduğunu belirlemek için kullanılır.
      if (slope < 0 && cx <= width/2) { //görüntünün sol yarısında yer alıyor.
        left_lines.push_back(ln);
        cv::line(frame, {x1,y1}, {x2,y2}, {128, 0, 0}, 2);
      }
      else if (slope >= 0 && cx > width/2 && cx <= right_bound) {
        right_lines.push_back(ln);
        right_centers.push_back(cx);
        right_slopes.push_back(slope);
        cv::line(frame, {x1,y1}, {x2,y2},  {0, 0, 255}, 2);
      }
    }

    double B = 0.0; // B en iyi seçilen sol çizginin ortası
    if (!left_lines.empty()) {
      int min_x_sum = 99999;
      int best_x1 = 0, best_x2 = 0;
      for (const auto& ln : left_lines) {
        int x1 = ln[0], x2 = ln[2];
        int x_sum = x1 + x2; // çizgi ne kadar sola yakınsa toplam o kadar küçük olur.
        if (x_sum < min_x_sum) {
          min_x_sum = x_sum; // en sola yatık çizgiyi saklar.
          best_x1 = x1;
          best_x2 = x2;
        }
      }
      B = (best_x1 + best_x2) / 2.0;
    }

    // Sağ şerit için: ortalama
    double A = 0.0;
    if (!right_centers.empty()) {
      A = std::accumulate(right_centers.begin(), right_centers.end(), 0.0) / right_centers.size();
    }

    // Lane center hesapla
    int lane_center = width/2;  // default orta nokta (hiç çizgi yoksa)
    if (A > 0 && B > 0) {
      lane_center = static_cast<int>(0.7 * A + 0.3 * B);
  }
  else if (A > 0) {
      lane_center = static_cast<int>(A);
  }
  else if (B > 0) {
      lane_center = static_cast<int>(B);
  }
  

    std_msgs::msg::Int32 center_msg;
    center_msg.data = lane_center;
    center_pub_->publish(center_msg);

    auto avg = [](const std::vector<double>& v) -> double {
      return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };
    std_msgs::msg::Float64 left_msg, right_msg;
    left_msg.data = 0.0; // sol eğim gönderilmiyor artık
    right_msg.data = avg(right_slopes);
    left_slope_pub_->publish(left_msg);
    right_slope_pub_->publish(right_msg);

    cv::circle(frame, {lane_center, height-30}, 8, {0, 255, 255}, -1);
   
   // Merkez hattı (araç ortası) ve lane_center arasındaki hata
int image_center = width / 2;
int alignment_error = lane_center - image_center;

// Alignment bilgisini ekrana yaz
std::string text = "Alignment: " + std::to_string(alignment_error);
cv::putText(frame,
            text,
            cv::Point(10, 30),                  
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,                                 
            cv::Scalar(255, 255, 255),           
            2);
   
   
    cv::rectangle(frame, {left_bound, roi_top}, {right_bound, height}, {0, 255, 255}, 2);
    cv::imshow("Lane Detection", frame);
    cv::waitKey(1);
}

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_slope_pub_, right_slope_pub_;
};
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneFollowingNode>());
  rclcpp::shutdown();
  return 0;
} 