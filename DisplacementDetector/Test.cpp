#include "DisplacementDetector.h"
#include <fmt/core.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

using json = nlohmann::json;

void testDisplacementDetector()
{
	DisplacementDetector detector;

	//std::string templateSettingsPath = "./Data/settings/samsungs23u/";
	std::string templateSettingsPath = "./Data/settings/scenes1/";

	cv::Mat template_image = cv::imread(templateSettingsPath + "template.jpg");
	if (template_image.empty())
	{
		std::cerr << "Failed to load image!" << std::endl;
		return;
	}
	detector.setDisplacementMarkerTemplate(template_image);

	cv::Vec3d point;
	MODELING_RESULT_CODE modeling_result_code;
	// 创建一些cv::Point对
	cv::Point point1_1(933, 936);
	cv::Point point1_2(1363, 1240);

	cv::Point point2_1(1907, 1624);
	cv::Point point2_2(2393, 1967);

	/*cv::Point point1_1(819, 1378);
	cv::Point point1_2(1541, 1430);

	cv::Point point2_1(2562, 1504);
	cv::Point point2_2(3334, 1560);*/

	// 创建std::vector<std::pair<cv::Point, cv::Point>>对象
	std::vector<std::pair<cv::Point, cv::Point>> intersectionPoints;
	intersectionPoints.push_back(std::make_pair(point1_1, point1_2));
	intersectionPoints.push_back(std::make_pair(point2_1, point2_2));

	std::pair<cv::Point, cv::Point > centers = std::make_pair(cv::Point(1148, 1088), cv::Point(2150, 1796));
	//std::pair<cv::Point, cv::Point > centers = std::make_pair(cv::Point(1180, 1404), cv::Point(2948, 1532));

	LOAD_RESULT_CODE load_result_code = detector.loadSettings(templateSettingsPath + "template.json");
	switch (load_result_code)
	{
	case LOAD_PENDING:

		//选取两组切点点对为intersectionOnMarkers_赋值
		detector.setIntersectionOnMarkers(intersectionPoints);
		//选取两组中心点为intersectionOnMarkers_赋值
		detector.setMarkerCenters(centers);
		modeling_result_code = detector.calculateMarkersCoordinates();
		fmt::print("标记建模完成\n");
		if (modeling_result_code == MODELING_SUCCESEE)
		{
			std::pair <cv::Vec3d, cv::Vec3d> marker_center_coordinates = detector.getMarkerCenterCoordinates();
			fmt::print("标记1中心坐标: ({:.2f}, {:.2f}, {:.2f})\n标记2中心坐标: ({:.2f}, {:.2f}, {:.2f})\n",
				marker_center_coordinates.first[0], marker_center_coordinates.first[1], marker_center_coordinates.first[2],
				marker_center_coordinates.second[0], marker_center_coordinates.second[1], marker_center_coordinates.second[2]
			);

			double distance = detector.calculateDistanceBetweenPoints(detector.getMarkerCenterCoordinates().first, detector.getMarkerCenterCoordinates().second);
			fmt::print("标记间距离:{:.2f}\n", distance);

			double errors = detector.calculationErrors();
			fmt::print("距离误差:±{:.2f}%", errors * 100);
		}
		break;

	case LOAD_MISSING_CAMERA_MARTIX:

	case LOAD_ERROR:
		fmt::print("配置加载失败\n");
		break;

	default:
		fmt::print("配置加载失败\n");
		break;
	}
}

int main()
{
	testDisplacementDetector();
	return 0;
}
