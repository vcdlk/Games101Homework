// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <array>
#include <limits>
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


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    const auto& point_a = _v[0];
    const auto& point_b = _v[1];
    const auto& point_c = _v[2];



    Eigen::Vector3f point_p(x , y, point_a[2]);


    auto& vector_pa = point_a - point_p;
    auto& vector_pb = point_b - point_p;
    auto& vector_pc = point_c - point_p;

    auto& vector_ab = point_a - point_b;
    auto& vector_bc = point_b - point_c;
    auto& vector_ca = point_c - point_a;


    auto pa_ab = vector_pa.cross(vector_ab);
    auto pb_bc = vector_pb.cross(vector_bc);
    auto pc_ca = vector_pc.cross(vector_ca);

    // std::cout << "pa_ab: \n" << pa_ab << std::endl;
    // std::cout << "pb_bc: \n" << pb_bc << std::endl;
    // std::cout << "pc_ca: \n" << pc_ca << std::endl;

    bool res = (pa_ab[2] > 0 && pb_bc[2] > 0 &&pc_ca[2] > 0) || ( (pa_ab[2] < 0 && pb_bc[2] < 0 &&pc_ca[2] < 0));

    // std::cout << "res:" << res << std::endl;
    return res;
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

void rst::rasterizer::GetBoundingBox(const Triangle &t, Eigen::Vector2f *bounding_box_x,
    Eigen::Vector2f *bounding_box_y){
    float bounding_box_min = std::numeric_limits<float>::min();
    float bounding_box_max = std::numeric_limits<float>::max();

    // 初始化 bounding_box 的范围，初始化是无限
    (*bounding_box_x) =  Eigen::Vector2f(bounding_box_min, bounding_box_max); // {x_max, x_min}
    (*bounding_box_y) =  Eigen::Vector2f(bounding_box_min, bounding_box_max);  // {y_max, y_min}

    auto v = t.toVector4();
    for(const auto& item : v){    
        const auto& x = item[0];
        if(x > (*bounding_box_x)[0]){
            (*bounding_box_x)[0] = x;
        }
        if(x < (*bounding_box_x)[1]){
            (*bounding_box_x)[1] = x;
        }

        const auto& y = item[1];
        if(y > (*bounding_box_y)[0]){
            (*bounding_box_y)[0] = y;
        }
        if(y < (*bounding_box_y)[1]){
            (*bounding_box_y)[1] = y;
        }
    }

    // std::cout << "(*bounding_box_x): " << (*bounding_box_x) << std::endl;
    // std::cout << "(*bounding_box_y): " << (*bounding_box_y) << std::endl;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // for(const auto& item : v){
    //     std::cout << "rasterize_triangle: " << item << std::endl;
    // }


    Eigen::Vector2f bounding_box_x;
    Eigen::Vector2f bounding_box_y;
    GetBoundingBox(t, &bounding_box_x, &bounding_box_y);

    std::cout << "GetBoundingBox bounding_box_x: " << bounding_box_x << std::endl;
    std::cout << "GetBoundingBox bounding_box_y: " << bounding_box_y << std::endl;


    Vector3f triangle_list[3];
    triangle_list[0] = v[0].head<3>();
    triangle_list[1] = v[1].head<3>();
    triangle_list[2] = v[2].head<3>();

    for(int x =static_cast<int>(bounding_box_x[1]); x < static_cast<int>(bounding_box_x[0]); x++){
        for(int y = static_cast<int>(bounding_box_y[1]); y < static_cast<int>(bounding_box_y[0]); y++){
            std::cout << "TEST x:" << x << "TEST y: " << y << std::endl;
            bool res = insideTriangle(x, y, triangle_list);
            std::cout << "insideTriangle res: " << res << std::endl;
            if(res){
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    Eigen::Vector3f point(x, y, 1);
                    set_pixel(point, t.getColor());
            }
        }
    }




    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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

// clang-format on