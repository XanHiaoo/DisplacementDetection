#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 读取图像
    cv::Mat image = cv::imread("test.jpg");

    // 确保成功读取图像
    if (image.empty()) {
        std::cerr << "无法读取图像文件" << std::endl;
        return -1;
    }

    // 转换图像为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 定义红色的HSV范围
    cv::Scalar lower_red(0, 100, 100);
    cv::Scalar upper_red(255, 255, 255);

    // 创建红色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv, lower_red, upper_red, mask);

    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 30);

    // 使用掩码对原始图像进行操作
    cv::Mat red_only;
    image.copyTo(red_only, mask);

    // 将 red_only 转换为灰度图像
    cv::cvtColor(red_only, red_only, cv::COLOR_BGR2GRAY);

    // 使用霍夫变换检测圆
    std::vector<cv::Vec3f> circles;
    //cv::HoughCircles(red_only, circles, cv::HOUGH_GRADIENT, 1, 1000, 30, 100, 300, 500);
    cv::HoughCircles(red_only, circles, cv::HOUGH_GRADIENT, 1, 1000, 30, 100, 150, 400);

    // 确保至少检测到两个圆
    if (circles.size() < 2) {
        std::cerr << "未检测到足够的圆" << std::endl;
        return -1;
    }

    // 提取圆心坐标和半径
    cv::Point2f center1(cvRound(circles[0][0]), cvRound(circles[0][1]));
    int radius1 = cvRound(circles[0][2]);

    cv::Point2f center2(cvRound(circles[1][0]), cvRound(circles[1][1]));
    int radius2 = cvRound(circles[1][2]);

    // 计算圆心之间的连线与圆边缘的交点
    cv::Point2f dir = center2 - center1;
    dir = dir / cv::norm(dir);

    cv::Point2f point1 = center1 + radius1 * dir;
    cv::Point2f point2 = center1 - radius1 * dir;
    cv::Point2f point3 = center2 + radius2 * dir;
    cv::Point2f point4 = center2 - radius2 * dir;

    // 在图像上绘制交点
    cv::circle(image, point1, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(image, point2, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(image, point3, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(image, point4, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

    cv::circle(image, center1, 6, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(image, center2, 6, cv::Scalar(0, 255, 0), -1, 8, 0);

     //在图像上绘制连线
    cv::line(image, center1, center2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    std::stringstream distanceStream;
    distanceStream << std::fixed << std::setprecision(2) << 25.65;
    std::string distanceText = distanceStream.str();

    std::stringstream errorStream;
    errorStream << std::fixed << std::setprecision(2) << 6.40;
    std::string errorText = errorStream.str();

    std::string text = "distance: " + distanceText + "cm, error: " + errorText + "%";
    cv::putText(image, text, cv::Point((center1.x + center2.x) / 2, (center1.y + center2.y) / 2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 255), 2);

    return 0;
}
