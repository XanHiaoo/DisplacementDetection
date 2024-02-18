#define _USE_MATH_DEFINES
#include "DisplacementDetectorImpl.h"

DisplacementDetectorImpl::DisplacementDetectorImpl()
{
    settingsLoaded = LOAD_ERROR;
    // Constructor implementation
}

DisplacementDetectorImpl::~DisplacementDetectorImpl()
{
    // Destructor implementation
}

void DisplacementDetectorImpl::setDisplacementMarkerTemplate(const cv::Mat &image)
{
    laneTemplateImage_ = image.clone();
}

void DisplacementDetectorImpl::setCameraMatrix(const cv::Mat &cameraMatrix)
{
    cameraMatrix_ = cameraMatrix.clone();
}

double DisplacementDetectorImpl::calculateDistanceBetweenPoints(const cv::Vec3d point1, const cv::Vec3d point2){
    cv::Vec3d diff = point1 - point2;
    double distance = cv::norm(diff);
    return distance;
}

LOAD_RESULT_CODE DisplacementDetectorImpl::loadSettings(const std::string &path)
{
    LOAD_RESULT_CODE result = settings.load(path);
    switch (result)
    {
    case LOAD_PENDING:
        cameraMatrix_ = settings.TEMPLATE_CAMERA_MATRIX_;
        markerRadius_ = settings.MARKER_RADIUS;
        settingsLoaded = LOAD_PENDING;
        break;

    case LOAD_MISSING_CAMERA_MARTIX:
        settingsLoaded = LOAD_MISSING_CAMERA_MARTIX;
        break;

    case LOAD_MISSING_MARKER_RADIUS:
        settingsLoaded = LOAD_MISSING_MARKER_RADIUS;
        break;

    case LOAD_ERROR:
        settingsLoaded = LOAD_ERROR;
        break;

    default:
        settingsLoaded = LOAD_ERROR;
        break;
    }
    return result;
}

std::pair<cv::Vec3d, cv::Vec3d> DisplacementDetectorImpl::calculateMarkerCenterDirections() {
    if (markerCenters_.first == cv::Point() && markerCenters_.second == cv::Point()) {
        // 若没有标记中心，返回空值
        return std::make_pair(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));
    }

    // Get the camera matrix values
    double fx = cameraMatrix_.at<double>(0, 0);
    double fy = cameraMatrix_.at<double>(1, 1);
    double u = cameraMatrix_.at<double>(0, 2);
    double v = cameraMatrix_.at<double>(1, 2);

    const cv::Point& p1 = markerCenters_.first;
    const cv::Point& p2 = markerCenters_.second;

    // Calculate the direction vectors of the lines for the two point pairs
    cv::Vec3d dir1((p1.x - u) / fx, (p1.y - v) / fy, 1.0);
    cv::Vec3d dir2((p2.x - u) / fx, (p2.y - v) / fy, 1.0);
    std::pair<cv::Vec3d, cv::Vec3d> dir_pair(dir1, dir2);
    markerCenterDirs_ = dir_pair;

    return dir_pair;
}

std::vector<std::pair<cv::Vec3d, cv::Vec3d>> DisplacementDetectorImpl::calculateMarkersIntersectionDirections() {
    if (intersectionOnMarkers_.size() < 2)
    {
        return {};
    }

    // Get the camera matrix values
    double fx = cameraMatrix_.at<double>(0, 0);
    double fy = cameraMatrix_.at<double>(1, 1);
    double u = cameraMatrix_.at<double>(0, 2);
    double v = cameraMatrix_.at<double>(1, 2);

    // Retrieve the two sets of point pairs
    const std::pair<cv::Point, cv::Point>& pointPair1 = intersectionOnMarkers_[0];
    const std::pair<cv::Point, cv::Point>& pointPair2 = intersectionOnMarkers_[1];

    // Extract points from the point pairs
    const cv::Point& p1 = pointPair1.first;
    const cv::Point& p2 = pointPair1.second;
    const cv::Point& p3 = pointPair2.first;
    const cv::Point& p4 = pointPair2.second;

    // Calculate the direction vectors of the lines for the two point pairs
    cv::Vec3d dir1((p1.x - u) / fx, (p1.y - v) / fy, 1.0);
    cv::Vec3d dir2((p2.x - u) / fx, (p2.y - v) / fy, 1.0);
    cv::Vec3d dir3((p3.x - u) / fx, (p3.y - v) / fy, 1.0);
    cv::Vec3d dir4((p4.x - u) / fx, (p4.y - v) / fy, 1.0);

    std::vector<std::pair<cv::Vec3d, cv::Vec3d>> dir_vec = {};
    dir_vec.push_back(std::make_pair(dir1, dir2));
    dir_vec.push_back(std::make_pair(dir3, dir4));

    intersectionOnMarkersDirs_ = dir_vec;

    return dir_vec;
}

cv::Vec3d DisplacementDetectorImpl::calculateSphereCenters(const cv::Vec3d& markerCenterDir, const cv::Vec3d& intersectionDir, double distance)
{
    // 归一化向量
    cv::Vec3d normalizedMarkerCenterDir = markerCenterDir / cv::norm(markerCenterDir);
    cv::Vec3d normalizedIntersectionDir = intersectionDir / cv::norm(intersectionDir);

    // 计算夹角θ
    double cosTheta = normalizedMarkerCenterDir.dot(normalizedIntersectionDir);
    double theta = std::acos(cosTheta);

    // 计算球心坐标
    double sinTheta = std::sin(theta);
    double modulus_distance = distance / sinTheta;

    // 根据夹角和距离计算球心坐标
    cv::Vec3d sphereCenter = normalizedMarkerCenterDir * modulus_distance;

    return sphereCenter;
}

double DisplacementDetectorImpl::calculationErrors() {

    // Get the camera matrix values
    double fx = cameraMatrix_.at<double>(0, 0);
    double fy = cameraMatrix_.at<double>(1, 1);
    double u = cameraMatrix_.at<double>(0, 2);
    double v = cameraMatrix_.at<double>(1, 2);

    // Retrieve the two sets of point pairs
    const std::pair<cv::Point, cv::Point>& pointPair1 = intersectionOnMarkers_[0];
    const std::pair<cv::Point, cv::Point>& pointPair2 = intersectionOnMarkers_[1];

    // Extract points from the point pairs
    const cv::Point& p1 = pointPair1.first;
    const cv::Point& p2 = pointPair2.first;

    double maxImpact = 0.0;

    //像素偏离值(中心点)，用于计算距离误差
    int pixel_offset_value = 3;
    for (int i = -pixel_offset_value; i <= pixel_offset_value; i += pixel_offset_value * 2) {
        for (int j = -pixel_offset_value; j <= pixel_offset_value; j += pixel_offset_value * 2) {
            for (int k = -pixel_offset_value; k <= pixel_offset_value; k += pixel_offset_value * 2) {
                for (int l = -pixel_offset_value; l <= pixel_offset_value; l += pixel_offset_value * 2) {
                    cv::Point p1_diff = cv::Point(p1.x + i, p1.y + j); // 像素差异
                    cv::Vec3d dir1_diff((p1_diff.x - u) / fx, (p1_diff.y - v) / fy, 1.0); // 新的方向向量

                    cv::Point p2_diff = cv::Point(p2.x + k, p2.y + l); // 像素差异
                    cv::Vec3d dir2_diff((p2_diff.x - u) / fx, (p2_diff.y - v) / fy, 1.0); // 新的方向向量

                    // 计算新的球心坐标
                    cv::Vec3d c1_diff = calculateSphereCenters(dir1_diff, markerCenterDirs_.first, markerRadius_);
                    cv::Vec3d c2_diff = calculateSphereCenters(dir2_diff, markerCenterDirs_.second, markerRadius_);

                    // 计算新的距离
                    double distance_diff = calculateDistanceBetweenPoints(c1_diff, c2_diff);

                    // 计算差异对结果的影响
                    double impact = (distance_diff - detectedDistance_) / detectedDistance_;

                    if (fabs(impact) > maxImpact) {
                        maxImpact = fabs(impact);
                    }
                }
            }
        }
    }

    // 计算差异对结果的影响
    return maxImpact;
}

void DisplacementDetectorImpl::calculateMarkersCoordinates() {
    calculateMarkerCenterDirections();
    calculateMarkersIntersectionDirections();

    cv::Vec3d c1 = calculateSphereCenters(markerCenterDirs_.first, intersectionOnMarkersDirs_[0].first, markerRadius_);
    cv::Vec3d c2 = calculateSphereCenters(markerCenterDirs_.second, intersectionOnMarkersDirs_[1].first, markerRadius_);
    std::pair <cv::Vec3d, cv::Vec3d> markers_coordinate = std::make_pair(c1, c2);
    MarkerCenterCoordinates_ = markers_coordinate;

    double distance = calculateDistanceBetweenPoints(c1, c2);
    detectedDistance_ = distance;

    double errors = calculationErrors();
    
}
