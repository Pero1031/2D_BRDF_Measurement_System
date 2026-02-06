#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main() {
    cv::Mat img(200, 300, CV_8UC3, cv::Scalar(0, 255, 0));
    cv::putText(img, "opencv ok", { 20,100 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0,0,0 }, 2);
    cv::imshow("test", img);
    cv::waitKey(0);
}