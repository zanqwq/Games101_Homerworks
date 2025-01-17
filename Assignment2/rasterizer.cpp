// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


// static bool insideTriangle(int x, int y, const Vector3f* _v)
// {   
//     Eigen::Vector2f point(x, y);


//     Eigen::Vector2f AB = _v[1].head(2) - _v[0].head(2);
//     Eigen::Vector2f BC = _v[2].head(2) - _v[1].head(2);
//     Eigen::Vector2f CA = _v[0].head(2) - _v[2].head(2);

//     Eigen::Vector2f AP = point - _v[0].head(2);
//     Eigen::Vector2f BP = point - _v[1].head(2);
//     Eigen::Vector2f CP = point - _v[2].head(2);

//     return AB[0] * AP[1] - AB[1] * AP[0] > 0
//         && BC[0] * BP[1] - BC[1] * BP[0] > 0
//         && CA[0] * CP[1] - CA[1] * CP[0] > 0;
// }

static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // Note: cross product must be 3d vector !!!
    Eigen::Vector3d a(_v[0][0], _v[0][1], 0);
    Eigen::Vector3d b(_v[1][0], _v[1][1], 0);
    Eigen::Vector3d c(_v[2][0], _v[2][1], 0);

    Eigen::Vector3d p(x, y, 0);

    // check cross(ab, ap), cross(bc, bp). cross(ca, cp)

    auto ab = b - a;
    auto ap = p - a;

    auto bc = c - b;
    auto bp = p - b;

    auto ca = a - c;
    auto cp = p - c;

    auto ab_x_ap = (ab.cross(ap)).normalized();
    auto bc_x_bp = (bc.cross(bp)).normalized();
    auto ca_x_cp = (ca.cross(cp)).normalized();

    return (ab_x_ap == bc_x_bp && bc_x_bp == ca_x_cp);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        // Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }

        // Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// Screen space (after doing mvp + viewport) rasterization
// Note: these triangles has performed model-view-projection transform
// they are not equal to the original triangles we defined
// TODO: MSAA + understanding barycentric z interpolation
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    auto x_list = { v[0][0], v[1][0], v[2][0] };
    float x_min = std::min(x_list);
    float x_max = std::max(x_list);

    auto y_list = { v[0][1], v[1][1], v[2][1] };
    float y_min = std::min(y_list);
    float y_max = std::max(y_list);

    int int_x_min = (int)std::ceil(x_min), int_x_max = (int)std::floor(x_max);
    int int_y_min = (int)std::ceil(y_min), int_y_max = (int)std::floor(y_max);

    // use msaa_num * msaa_num sample points inside one pixel to do sample
    float interval = 1.0f / msaa_num;

    auto get_percentage_color = [&](const Eigen::Vector3f color, float percentage) {
        std::vector<float> rgb{color[0], color[1], color[2]};
        const float total = rgb[0] + rgb[1] + rgb[2];

        // rgb is all 0
        if (fabs(total - 0) < 0.00000001) return color;

        auto diff = (percentage - 1) * total;
        for (int i = 0; i < 3; i++) {
            rgb[i] = rgb[i] + (rgb[i] / total) * diff;
            if (diff < 0) rgb[i] = std::max(rgb[i], 0.0f);
            else if (diff > 0) rgb[i] = std::min(rgb[i], 255.0f);
        }
        auto new_color = Eigen::Vector3f(rgb[0], rgb[1], rgb[2]);
        return new_color;
    };

    // for each pixel in aabb
    for (int x = int_x_min; x <= int_x_max; x++) {
        for (int y = int_y_min; y <= int_y_max; y++) {
            auto pixel_idx = get_index(x, y);

            /* do msaa */

            // the number of inside triangle sample points on this pixel
            int inside_count = 0;

            // for each sample point inside current pixel
            for (int i = 0; i < sample_count; i++) {
                auto j = i / msaa_num;
                auto k = i % msaa_num;
                float sample_x = x + (0.5f + j) * interval;
                float sample_y = y + (0.5f + k) * interval;

                // If so, use the following code to get the interpolated z value.
                // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                // z_interpolated *= w_reciprocal;

                // WTF? why should we interpolate this z value, rather than figure out the
                // equation of the triangle pane and use it to compute an extract z value ?
                auto [alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                // w_reciprocal 用来做透视矫正, 解决重心坐标在 screen space 和 view space 下不是 invariant 的问题
                // 解释详见 cg 文档笔记
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if (insideTriangle(sample_x, sample_y, t.v)) {
                    inside_count++;
                    if (sample_depth_buf[pixel_idx][i] > z_interpolated) {
                        sample_depth_buf[pixel_idx][i] = z_interpolated;
                        sample_frame_buf[pixel_idx][i] = t.getColor();
                    }
                }
            }

            if (inside_count) {
                // averaging each sample points inside current pixel
                auto color = Eigen::Vector3f(0, 0, 0);
                for (auto c: sample_frame_buf[pixel_idx]) {
                    color += get_percentage_color(c, 1.0f / sample_count);
                }
                set_pixel(Eigen::Vector3f(x, y, 0), color); // set frame_buf
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(sample_frame_buf.begin(), sample_frame_buf.end(), std::vector<Eigen::Vector3f>(sample_count, Eigen::Vector3f(0, 0, 0)));
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::vector<float>(sample_count, std::numeric_limits<float>::infinity()));
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    sample_frame_buf.resize(w * h);
    sample_depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::set_msaa(const int n) {
    msaa_num = n;
    sample_count = n * n;
}

// clang-format on