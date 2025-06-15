// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;
  
    view = translate * view;
  
    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float radian = (rotation_angle / 360) * (2 * MY_PI);
  
    Eigen::Matrix4f translate;
    translate << std::cos(radian), -std::sin(radian), 0, 0, std::sin(radian),
        std::cos(radian), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  
    // std::cout << "get_model_matrix: \n" << translate << std::endl;
    return translate;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
  // Students will implement this function

  float radian = (eye_fov / 365) * (2 * MY_PI);
  // opencv 中
  // 右侧是x轴正方向，y轴正方向向下，
  // 这就导致 y轴和z轴是相反的和推到过程中
  // 修改分两点
  // 1：不用写，纠正z轴
  // zNear = -zNear;
  // zFar = -zFar;
  // 2：求yTop的时候乘以 -1，纠正Y轴

  float yTop = -1 * (std::tan(radian / 2) * zNear); // -1 兼容opencv
  float yBottom = -yTop;

  float xLeft = -1 * ((yTop - yBottom) * aspect_ratio / 2);
  float xRight = -xLeft;

  Eigen::Matrix4f scale_mat;
  scale_mat << 2 / (xRight - xLeft), 0, 0, 0, 0, 2 / (yTop - yBottom), 0, 0, 0,
      0, 2 / (zNear - zFar), 0, 0, 0, 0, 1;
  // std::cout << "scale_mat: \n" << scale_mat << std::endl;

  Eigen::Matrix4f move_mat;
  move_mat << 1, 0, 0, -(xLeft + xRight) / 2, 0, 1, 0, -(yTop + yBottom) / 2, 0,
      0, 1, -(zFar + zNear) / 2, 0, 0, 0, 1;

  // std::cout << "move_mat: \n" << move_mat << std::endl;

  Eigen::Matrix4f persp_mat;
  persp_mat << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, (zFar + zNear),
      -(zNear * zFar), 0, 0, 1, 0;

  // std::cout << "persp_mat: \n" << persp_mat << std::endl;

  Eigen::Matrix4f projection = scale_mat * move_mat * persp_mat;

  std::cout << "projection: \n" << projection << std::endl;

  return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on