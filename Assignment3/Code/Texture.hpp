//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        auto u1 = std::floor(u_img);
        auto u2 = std::ceil(u_img);
        auto v1 = std::floor(v_img);
        auto v2 = std::ceil(v_img);
        auto du = u_img - u1;
        auto dv = v_img - v1;

        auto lerp = [*](Eigen::Vector3f v0, Eigen::Vector3f v1, float a) {
            return v0 + a * (v1 - v0);
        };

        return lerp(
            // 两次垂直插值, 一次水平插值
            lerp(getColor(u1, v1), getColor(u1, v2), du),
            lerp(getColor(u2, v1), getColor(u2, v2), du),
            dv,
        );
    }

};
#endif //RASTERIZER_TEXTURE_H
