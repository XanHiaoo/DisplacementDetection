#ifndef DisplacementDetector_DisplacementDetectorIMPL_H
#define DisplacementDetector_DisplacementDetectorIMPL_H

#include "TemplateSettings.h"
#include <opencv2/opencv.hpp>

class DisplacementDetectorImpl
{
  public:
    DisplacementDetectorImpl();  // Constructor
    ~DisplacementDetectorImpl(); // Destructor

    LOAD_RESULT_CODE settingsLoaded;

    void setDisplacementMarkerTemplate(const cv::Mat &image);
    void setCameraMatrix(const cv::Mat &cameraMatrix);
    void setMarkerRadius(const double markerRadius) { markerRadius_ = markerRadius; }
    void setIntersectionOnMarkers(const std::vector<std::pair<cv::Point, cv::Point>>& intersectionPoints){intersectionOnMarkers_ = intersectionPoints;}
    void setMarkerCenters(const std::pair<cv::Point, cv::Point>& markerCenter) { markerCenters_ = markerCenter; }

    std::vector<std::pair<cv::Point, cv::Point>> getIntersectionOnMarkers() const { return intersectionOnMarkers_; }
    std::vector<std::pair<cv::Vec3d, cv::Vec3d>> getIntersectionOnMarkersDir() const { return intersectionOnMarkersDirs_; }
    std::pair <cv::Vec3d, cv::Vec3d> getMarkerCenterCoordinates() const { return MarkerCenterCoordinates_; }

  public:
    LOAD_RESULT_CODE loadSettings(const std::string &path);
    void calculateMarkersCoordinates();
    double calculateDistanceBetweenPoints(const cv::Vec3d point1, const cv::Vec3d point2);
    double calculationErrors();

  private:
    TemplateSettings settings;
    cv::Mat laneTemplateImage_;
    cv::Mat detectImage_;
    cv::Mat cameraMatrix_;
    double markerRadius_{};
    double detectedDistance_{};

    std::pair<cv::Point, cv::Point> markerCenters_;
    std::pair<cv::Vec3d, cv::Vec3d> markerCenterDirs_ = std::make_pair(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));
    std::vector<std::pair<cv::Point, cv::Point>> intersectionOnMarkers_;
    std::vector<std::pair<cv::Vec3d, cv::Vec3d>> intersectionOnMarkersDirs_{ };
    std::pair <cv::Vec3d, cv::Vec3d> MarkerCenterCoordinates_;

  private:
    std::pair<cv::Vec3d, cv::Vec3d> calculateMarkerCenterDirections();
    std::vector < std::pair<cv::Vec3d, cv::Vec3d>>calculateMarkersIntersectionDirections();
    cv::Vec3d calculateSphereCenters(const cv::Vec3d& markerCenterDir, const cv::Vec3d& intersectionDir, double distance);

};

#endif // DisplacementDetector_DisplacementDetectorIMPL_H
