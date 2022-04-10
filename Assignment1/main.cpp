#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// model-view-projection-(viewport)

float get_rad(float angle) {
    return angle / 180.0f * MY_PI;
}

// only need to concern about ratating around z-axis
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    auto rad = get_rad(rotation_angle);
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

// TODO: rotate along x axis error
Eigen::Matrix4f get_model_matrix_3d(float rotation_z, float rotation_y, float rotation_x) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    auto a = get_rad(rotation_z), b = get_rad(rotation_y), c = get_rad(rotation_x);

    auto ca = cos(a), sa = sin(a);
    auto cb = cos(b), sb = sin(b);
    auto cc = cos(c), sc = sin(c);

    Eigen::Matrix4f rotate_z_matrix;
    rotate_z_matrix <<
        ca, -sa, 0, 0,
        sa, ca, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f rotate_y_matrix;
    rotate_y_matrix <<
        cb, 0, sb, 0,
        0, 1, 0, 0,
        -sb, 0, cb, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f rotate_x_matrix;
    rotate_x_matrix <<
        1, 0, 0, 0,
        0, cc, -sc, 0,
        0, sc, cc, 0,
        0, 0, 0, 1;

    model = rotate_z_matrix * rotate_y_matrix * rotate_x_matrix * model;

    return model;
}

// rotate along any axis across (0, 0, 0)
Eigen::Matrix4f get_model_matrix_along_axis(Eigen::Vector3f axis, float angle) {
    /*
    R = cos(a) * Identity + (1 - cos(a)) * n * n_trans
        + sin(a) * [
            [0, -zn, yn],
            [zn, 0, -xn],
            [-yn, xn, 0],
        ]
    */

    Eigen::Vector3f n = axis.normalized();
    auto n_trans = n.transpose();
    auto a = get_rad(angle);

    auto xn = n[0];
    auto yn = n[1];
    auto zn = n[2];
    auto sin_a = sin(a);
    auto cos_a = cos(a);

    Eigen::Matrix3f cross_matrix;
    cross_matrix <<
        0, -zn, yn,
        zn, 0, -xn,
        -yn, xn, 0;

    Eigen::Matrix3f rotate_matrix =
        cos_a * Eigen::Matrix3f::Identity() +
        (1 - cos_a) * n * n_trans +
        sin_a * cross_matrix;
    
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model.block<3, 3>(0, 0) = rotate_matrix;

    return model;
}

// rotate along any axis
Eigen::Matrix4f get_model_matrix_along_any_axis(Eigen::Vector3f start, Eigen::Vector3f axis, float angle) {
    Eigen::Matrix4f translate;
    translate <<
        1, 0, 0, -start[0],
        0, 1, 0, -start[1],
        0, 0, 1, -start[2],
        0, 0, 0, 1;

    Eigen::Matrix4f translate_back;
    translate_back <<
        1, 0, 0, start[0],
        0, 1, 0, start[1],
        0, 0, 1, start[2],
        0, 0, 0, 1;
    
    // return translate_back * translate;
    return translate_back * get_model_matrix_along_axis(axis, angle) * translate;
}

// Move eye pos to (0 0 0) along with other objects relatively
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

// fov is an angle,  aspect_ratio = width/height
Eigen::Matrix4f get_projection_matrix(
    float eye_fovY, float aspect_ratio,
    float zNear, float zFar
) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float radian = eye_fovY / 2.0f / 180.0f * MY_PI;
    // Note: negative n and f value
    float n = -zNear, f = -zFar;
    float t = tan(radian) * zNear;
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

    projection = ortho * perspective2ortho;

    return projection;
}

enum class RotationMode {
    Z,
    XYZ,
    AXIS,
    ANY_AXIS,
};

auto mode_map = std::unordered_map<std::string, RotationMode>({
    { "z", RotationMode::Z },
    { "xyz", RotationMode::XYZ },
    { "axis", RotationMode::AXIS },
    { "any_axis", RotationMode::ANY_AXIS }
});

std::vector<std::string> split(std::string src, char sep) {
    std::vector<std::string> res;
    std::string::size_type pos = 0;
    std::string::size_type found_pos = 0;
    for (auto i = 0; i < src.size(); i++) {
        int j = i;
        while (src[j] != sep && j < src.size()) j++;
        res.push_back(src.substr(i, j - i));
        i = j;
    }
    return res;
}

// arg_count: the length of argv
// argv[0]: program name
int main(int arg_count, const char** argv)
{
    RotationMode mode = RotationMode::Z;
    float rotation_z = 0, rotation_y = 0, rotation_x = 0;

    rst::rasterizer r(700, 700);
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    Eigen::Vector3f eye_pos = {0, 0, 5};
    Eigen::Vector3f start;
    Eigen::Vector3f axis;
    float rotation_angle = 0;

    bool command_line = false;
    std::string filename = "output.png";
    std::vector<std::string> splits;

    auto parse_argv = [&]() {
        if (arg_count < 2) return false;

        auto run_mode = std::string(argv[1]);
        if (run_mode == "--cmd") {
            command_line = true;
        } else if (run_mode == "--loop") {
            command_line = false;
        } else {
            return false;
        }

        auto mode_str = std::string(argv[2]);
        if (!mode_map.count(mode_str)) return false;

        mode = mode_map[mode_str];
        switch (mode)
        {
        // cmd arg: pname --cmd z 20 [filename]
        // cmd arg: pname --loop z
        case RotationMode::Z :
            if (!command_line) break;

            rotation_z = std::stof(argv[3]);
            if (arg_count == 5) filename = argv[4];
            break;
        // cmd arg: pname --cmd xyz 20,10,30 [filename]
        // cmd arg: pname --loop xyz
        case RotationMode::XYZ :
            if (!command_line) break;

            splits = split(argv[3], ',');
            rotation_z = std::stof(splits[0]);
            rotation_y = std::stof(splits[1]);
            rotation_x = std::stof(splits[2]);
            printf("%f %f %f", rotation_z, rotation_y, rotation_x);
            if (arg_count == 5) filename = argv[4];
            break;
        // cmd arg: pname --cmd axis x,y,z angle [filename]
        // cmd arg: pname --loop axis x,y,z
        case RotationMode::AXIS :
            splits = split(argv[3], ',');
            axis[0] = std::stof(splits[0]);
            axis[1] = std::stof(splits[1]);
            axis[2] = std::stof(splits[2]);

            if (!command_line) break;

            rotation_angle = std::stof(argv[4]);
            if (arg_count == 6) filename = argv[5];
            break;
        // cmd arg: panme --cmd any_axis x,y,z,xx,yy,zz angle [filename]
        // cmd arg: panme --loop any_axis x,y,z,xx,yy,zz
        case RotationMode::ANY_AXIS :
            splits = split(argv[3], ',');
            start[0] = std::stof(splits[0]);
            start[1] = std::stof(splits[1]);
            start[2] = std::stof(splits[2]);
            axis[0] = std::stof(splits[3]);
            axis[1] = std::stof(splits[4]);
            axis[2] = std::stof(splits[5]);

            if (!command_line) break;

            rotation_angle = std::stof(argv[4]);
            if (arg_count == 6) filename = argv[5];
            break;
        default:
            break;
        }

        return true;
    };

    if (!parse_argv()) return 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        switch (mode)
        {
        case RotationMode::Z :
            r.set_model(get_model_matrix(rotation_z));
            break;
        case RotationMode::XYZ :
            r.set_model(get_model_matrix_3d(rotation_z, rotation_y, rotation_x));
            break;
        case RotationMode::AXIS :
            r.set_model(get_model_matrix_along_axis(axis, rotation_angle));
            break;
        case RotationMode::ANY_AXIS :
            r.set_model(get_model_matrix_along_any_axis(start, axis, rotation_angle));
            break;
        default:
            break;
        }

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    int key = 0;
    int frame_count = 0;
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        switch (mode)
        {
        case RotationMode::Z :
            if (key == 'a') {
                rotation_z += 10;
            } else if (key == 'd') {
                rotation_z -= 10;
            }
            r.set_model(get_model_matrix(rotation_z));
            break;
        case RotationMode::XYZ :
            if (key == 'a') {
                rotation_z += 10;
            } else if (key == 'd') {
                rotation_z -= 10;
            } else if (key == 'q') {
                rotation_y += 10;
            } else if (key == 'e') {
                rotation_y -= 10;
            } else if (key == 'w') {
                rotation_x -= 10;
            } else if (key == 's') {
                rotation_x += 10;
            }
            r.set_model(get_model_matrix_3d(rotation_z, rotation_y, rotation_x));
            break;
        case RotationMode::AXIS :
            if (key == 'a') {
                rotation_angle += 10;
            } else if (key == 'd') {
                rotation_angle -= 10;
            }
            r.set_model(get_model_matrix_along_axis(axis, rotation_angle));
            break;
        case RotationMode::ANY_AXIS :
            if (key == 'a') {
                rotation_angle += 10;
            } else if (key == 'd') {
                rotation_angle -= 10;
            }
            r.set_model(get_model_matrix_along_any_axis(start, axis, rotation_angle));
            break;
        default:
            break;
        }

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
