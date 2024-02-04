#include "TemplateSettings.h"
#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

LOAD_RESULT_CODE TemplateSettings::load(const std::string &path)
{
    TEMPLATE_SETTINGS_PATH_ = path;
    json jsonObject;
    // 读取template.json文件
    std::ifstream jsonFile(path);
    if (jsonFile.is_open())
    {
        jsonFile >> jsonObject;
        jsonFile.close();

        // 检查是否存在CameraMatrix参数
        if (jsonObject.contains("CameraMatrix"))
        {
            auto cameraMatrixObject = jsonObject["CameraMatrix"];
            if (cameraMatrixObject.is_array() && cameraMatrixObject.size() == 3)
            {
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        TEMPLATE_CAMERA_MATRIX_.at<double>(i, j) = cameraMatrixObject[i][j];
                    }
                }
                fmt::print("CameraMatrix loaded from template.json.\n");
            }
            else
            {
                fmt::print(stderr, "Invalid CameraMatrix data in template.json!\n");
                return LOAD_MISSING_CAMERA_MARTIX;
            }
        }
        else
        {
            fmt::print(stderr, "CameraMatrix parameter not found in template.json.\n");
            return LOAD_MISSING_CAMERA_MARTIX;
        }

        // 检查是否存在MarkerRadius参数
        if (jsonObject.contains("MarkerRadius"))
        {
                MARKER_RADIUS = jsonObject["MarkerRadius"];
                fmt::print("MarkerRadius loaded from template.json.\n");
        }
        else
        {
            fmt::print(stderr, "MarkerRadius parameter not found in template.json.\n");
            return LOAD_MISSING_MARKER_RADIUS;
        }
    }
    else
    {
        fmt::print(stderr, "Failed to open template.json for reading.\n");
        return LOAD_ERROR;
    }
    return LOAD_PENDING;
}

