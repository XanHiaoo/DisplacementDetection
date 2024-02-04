#include "DisplacementDetector.h"

DisplacementDetector::DisplacementDetector()
{
    detector_ = new DisplacementDetectorImpl();
}

DisplacementDetector::~DisplacementDetector()
{
    delete detector_;
}

void DisplacementDetector::setDisplacementMarkerTemplate(const cv::Mat &image)
{
    detector_->setDisplacementMarkerTemplate(image);
}

void DisplacementDetector::setMarkerCenters(const std::pair<cv::Point, cv::Point>& intersection)
{
    detector_->setMarkerCenters(intersection);
}

void DisplacementDetector::setIntersectionOnMarkers(const std::vector<std::pair<cv::Point, cv::Point>>& intersection)
{
    detector_->setIntersectionOnMarkers(intersection);
}

LOAD_RESULT_CODE DisplacementDetector::loadSettings(const std::string &path)
{
    return (detector_->loadSettings(path));
}

MODELING_RESULT_CODE DisplacementDetector::calculateMarkersCoordinates()
{
 
    if (detector_->settingsLoaded == LOAD_PENDING)
    {
        detector_->calculateMarkersCoordinates();
        return MODELING_SUCCESEE;
    }
    else
    {
        return MODELING_ERROR;
    }
}

double DisplacementDetector::calculateDistanceBetweenPoints(const cv::Vec3d point1, const cv::Vec3d point2)
{
    return (detector_->calculateDistanceBetweenPoints(point1, point2));
}

std::pair <cv::Vec3d, cv::Vec3d>DisplacementDetector::getMarkerCenterCoordinates()
{ 
    return detector_->getMarkerCenterCoordinates(); 
}

double DisplacementDetector::calculationErrors() {
    return detector_->calculationErrors();
}