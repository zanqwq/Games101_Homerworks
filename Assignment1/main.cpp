#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// Move eye pos to (0 0 0)
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate <<
        1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

// TODO: only need to concern about ratating around z-axis
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto rad = rotation_angle / 180.0f * MY_PI;
    auto c = cos(rad);
    auto s = sin(rad);

    Eigen::Matrix4f rotate;
    rotate <<
        c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    model = rotate * model;

    return model;
}

float get_rad(float angle) {
    return angle / 180.0f * MY_PI;
}

// a rotate along z, b rotate along y, c rotate along x
Eigen::Matrix4f get_model_matrix_3d(float a, float b, float c) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    auto rad_a = get_rad(a), rad_b = get_rad(b), rad_c = get_rad(c);

    auto ca = cos(rad_a), sa = sin(rad_a);
    auto cb = cos(rad_b), sb = sin(rad_b);
    auto cc = cos(rad_c), sc = sin(rad_c);

    Eigen::Matrix4f rotate_z;
    rotate_z <<
        ca, -sa, 0, 0,
        sa, ca, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f rotate_y;
    rotate_y <<
        cb, 0, sb, 0,
        0, 1, 0, 0,
        -sb, 0, cb, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f rotate_x;
    rotate_x <<
        1, 0, 0, 0,
        0, cc, -sc, 0,
        0, sc, cc, 0,
        0, 0, 0, 1;

    model = rotate_z * rotate_y * rotate_x * model;

    return model;
}

Eigen::Matrix4f get_model_matrix_3d_along_axis(Eigen::Vector3f axis, float angle) {
    auto rad = get_rad(angle);
    auto model = Eigen::Matrix4f::Identity();
    return model;
}

// TODO: fov is an angle,  aspect_ratio = width/height
Eigen::Matrix4f get_projection_matrix(
    float eye_fovY, float aspect_ratio,
    float zNear, float zFar
) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float radian = eye_fovY / 2.0f / 180.0f * MY_PI;
    float n = zNear, f = zFar;
    float t = tan(radian) * n;
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;

    Eigen::Matrix4f translate;
    translate <<
        1, 0, 0, -(l + r)/2,
        0, 1, 0, -(t + b)/2,
        0, 0, 1, -(n + f)/2,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale <<
        2.0f / (r - l), 0, 0, 0,
        0, 2.0f / (t - b), 0, 0,
        0, 0, 2.0f / (n - f), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f ortho = scale * translate;

    Eigen::Matrix4f perspective2ortho;
    perspective2ortho <<
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    projection = ortho * perspective2ortho;

    return projection;
}

int main(int argc, const char** argv)
{
    // a rotate along z, b rotate along y, c rotate along x
    float a = 0, b = 0, c = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        a = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else {
            return 0;
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(a));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix_3d(a, b, c));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            a += 10;
        } else if (key == 'd') {
            a -= 10;
        } else if (key == 'q') {
            b += 10;
        } else if (key == 'e') {
            b -= 10;
        } else if (key == 'w') {
            c -= 10;
        } else if (key == 's') {
            c += 10;
        }
    }

    return 0;
}
