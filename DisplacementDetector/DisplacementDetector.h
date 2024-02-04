#ifndef DisplacementDetector_DisplacementDetector_H
#define DisplacementDetector_DisplacementDetector_H

#include <opencv2/core.hpp>
#include <vector>

#include "DisplacementDetectorImpl.h"
#include "TemplateSettings.h"

enum MODELING_RESULT_CODE : int
{
    MODELING_ERROR = -1,
    MODELING_SUCCESEE = 0
};

class DisplacementDetectorImpl;

class DisplacementDetector
{
  public:
    DisplacementDetector();
    ~DisplacementDetector();

    void setDisplacementMarkerTemplate(const cv::Mat &image);
    void setMarkerCenters(const std::pair<cv::Point, cv::Point>& intersection);
    void setIntersectionOnMarkers(const std::vector<std::pair<cv::Point, cv::Point>>& intersection);
    LOAD_RESULT_CODE loadSettings(const std::string &path);
    MODELING_RESULT_CODE calculateMarkersCoordinates();
    std::pair <cv::Vec3d, cv::Vec3d> getMarkerCenterCoordinates();
    double calculateDistanceBetweenPoints(const cv::Vec3d point1, const cv::Vec3d point2);
    double calculationErrors();

  private:
    DisplacementDetectorImpl *detector_;
};

#endif // DisplacementDetector_DisplacementDetector_H
