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
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
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

// Screen space rasterization
// Note: these triangles has performed model-view-projection transform
// they are not equal to the original triangles we defined
// TODO: MSAA + understanding barycentric z interpolation
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // std::cout << "rasterize triangle: " << std::endl;
    // std::cout << v[0] << std::endl << v[1] << std::endl << v[2] << std::endl;
    float x_min = std::min({ v[0][0], v[1][0], v[2][0] });
    float x_max = std::max({ v[0][0], v[1][0], v[2][0] });
    float y_min = std::min({ v[0][1], v[1][1], v[2][1] });
    float y_max = std::max({ v[0][1], v[1][1], v[2][1] });

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;

    // WTF? why should we interpolate this z value, rather than figure out the
    // equation of the triangle pane and use it to compute an extract z value ?

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    int int_x_min = (int)std::ceil(x_min), int_x_max = (int)std::floor(x_max);
    int int_y_min = (int)std::ceil(y_min), int_y_max = (int)std::floor(y_max);

    // use msaa_num * msaa_num sample points inside one pixel to do sample
    float msaa_step = 1.0f / (msaa_num + 1);
    int sample_count = msaa_num * msaa_num;

    auto rgba = [&](const Eigen::Vector3f rgb, float alpha) {
        Eigen::Vector3f new_color = Eigen::Vector3f(rgb);
        // do lerp(x, 255, 1 - alpha) on rgb
        new_color[0] = new_color[0] + (255 - new_color[0]) * (1 - alpha);
        new_color[1] = new_color[1] + (255 - new_color[1]) * (1 - alpha);
        new_color[2] = new_color[2] + (255 - new_color[2]) * (1 - alpha);
        return new_color;
    };

    for (int x = int_x_min; x <= int_x_max; x++) {
        for (int y = int_y_min; y <= int_y_max; y++) {
            // msaa
            int inside_count = 0;
            float min_depth = FLT_MAX;
            for (int i = 1; i <= msaa_num; i++) {
                for (int j = 1; j <= msaa_num; j++) {
                    float sample_x = x + i * msaa_step;
                    float sample_y = y + j * msaa_step;
                    if (insideTriangle(sample_x, sample_y, t.v)) {
                        inside_count++;
                        auto [alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        min_depth = std::min(min_depth, z_interpolated);
                    }
                }
            }

            if (!inside_count) continue;

            auto idx = get_index(x, y);
            if (depth_buf[idx] > min_depth) {
                // printf("%f < %f\n", z_interpolated, depth_buf[idx]);
                depth_buf[idx] = min_depth;

                auto color = t.getColor();
                auto a = inside_count * 1.0f / (sample_count);
                // auto new_color = rgba(color, a);
                auto new_color = color * a;

                // TODO: set_pixel don't need z value ?
                set_pixel(Eigen::Vector3f(x, y, min_depth), new_color);
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
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
}

// clang-format on