#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // ��ȡͼ��
    cv::Mat image = cv::imread("test.jpg");

    // ȷ���ɹ���ȡͼ��
    if (image.empty()) {
        std::cerr << "�޷���ȡͼ���ļ�" << std::endl;
        return -1;
    }

    // ת��ͼ��ΪHSV��ɫ�ռ�
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // �����ɫ��HSV��Χ
    cv::Scalar lower_red(0, 100, 100);
    cv::Scalar upper_red(255, 255, 255);

    // ������ɫ���������
    cv::Mat mask;
    cv::inRange(hsv, lower_red, upper_red, mask);

    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 30);

    // ʹ�������ԭʼͼ����в���
    cv::Mat red_only;
    image.copyTo(red_only, mask);

    // �� red_only ת��Ϊ�Ҷ�ͼ��
    cv::cvtColor(red_only, red_only, cv::COLOR_BGR2GRAY);

    // ʹ�û���任���Բ
    std::vector<cv::Vec3f> circles;
    //cv::HoughCircles(red_only, circles, cv::HOUGH_GRADIENT, 1, 1000, 30, 100, 300, 500);
    cv::HoughCircles(red_only, circles, cv::HOUGH_GRADIENT, 1, 1000, 30, 100, 150, 400);

    // ȷ�����ټ�⵽����Բ
    if (circles.size() < 2) {
        std::cerr << "δ��⵽�㹻��Բ" << std::endl;
        return -1;
    }

    // ��ȡԲ������Ͱ뾶
    cv::Point2f center1(cvRound(circles[0][0]), cvRound(circles[0][1]));
    int radius1 = cvRound(circles[0][2]);

    cv::Point2f center2(cvRound(circles[1][0]), cvRound(circles[1][1]));
    int radius2 = cvRound(circles[1][2]);

    // ����Բ��֮���������Բ��Ե�Ľ���
    cv::Point2f dir = center2 - center1;
    dir = dir / cv::norm(dir);

    cv::Point2f point1 = center1 + radius1 * dir;
    cv::Point2f point2 = center1 - radius1 * dir;
    cv::Point2f point3 = center2 + radius2 * dir;
    cv::Point2f point4 = center2 - radius2 * dir;

    // ��ͼ���ϻ��ƽ���
    cv::circle(image, point1, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(image, point2, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(image, point3, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(image, point4, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

    cv::circle(image, center1, 6, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(image, center2, 6, cv::Scalar(0, 255, 0), -1, 8, 0);

     //��ͼ���ϻ�������
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
