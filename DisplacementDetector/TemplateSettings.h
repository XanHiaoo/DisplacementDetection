#ifndef DisplacementDetector_TEMPLATESETTINGS_H
#define DisplacementDetector_TEMPLATESETTINGS_H

#include <opencv2/core.hpp>
enum LOAD_RESULT_CODE : int
{
    LOAD_ERROR = -1,
    LOAD_PENDING = 0,
    LOAD_MISSING_CAMERA_MARTIX = 1,
    LOAD_MISSING_MARKER_RADIUS = 2
};
struct TemplateSettings
{
    std::string TEMPLATE_SETTINGS_PATH_{};
    cv::Mat TEMPLATE_CAMERA_MATRIX_ = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
    double MARKER_RADIUS{};
    LOAD_RESULT_CODE load(const std::string &path);
};

#endif // DisplacementDetector_TEMPLATESETTINGS_H
