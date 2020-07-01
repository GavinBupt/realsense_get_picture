// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <fstream>              // std::ifstream
#include <string>
#include <sstream>
#include <cmath>
#include <time.h>
#include <Windows.h>
#include <io.h>
#include<direct.h>
// Helper functions
#define CLOCKS_PER_SEC  ((clock_t)1000)
void register_glfw_callbacks(window& app, glfw_state& app_state);

float detR(float H[16]) {
    return H[0] * (H[5] * H[10] - H[9] * H[6]) - H[4] * (H[1] * H[10] - H[2] * H[9]) + H[8] * (H[1] * H[6] - H[5] * H[2]);
}

time_t t;
std::string time_string;
rs2_pose T265_pose;
std::ofstream T265fout;
std::string file_root = "G:/realsense/";
cv::Mat UpD435colorImg, UpD435depthImg;
cv::Mat DownD435colorImg, DownD435depthImg;
HANDLE T265Semaphore, T265hMutex;
HANDLE UpD435ColorSemaphore, UpD435DepthSemaphore, UpD435ColorhMutex, UpD435DepthhMutex;
HANDLE DownD435ColorSemaphore, DownD435DepthSemaphore, DownD435ColorhMutex, DownD435DepthhMutex;

DWORD WINAPI FunProcT265(LPVOID lpParameter);
DWORD WINAPI FunProcUpD435Color(LPVOID lpParameter);
DWORD WINAPI FunProcUpD435Depth(LPVOID lpParameter);
DWORD WINAPI FunProcDownD435Color(LPVOID lpParameter);
DWORD WINAPI FunProcDownD435Depth(LPVOID lpParameter);

int main(int argc, char* argv[]) try
{
    //创建线程

    HANDLE hThreadUpD435Color, hThreadUpD435Depth, hThreadDownD435Color, hThreadDownD435Depth, hThreadT265;

    T265Semaphore = CreateSemaphore(NULL, 0, 1, NULL);
    UpD435ColorSemaphore = CreateSemaphore(NULL, 0, 1, NULL);
    UpD435DepthSemaphore = CreateSemaphore(NULL, 0, 1, NULL);
    DownD435ColorSemaphore = CreateSemaphore(NULL, 0, 1, NULL);
    DownD435DepthSemaphore = CreateSemaphore(NULL, 0, 1, NULL);

    T265hMutex = CreateMutex(NULL, TRUE, "T265");
    UpD435ColorhMutex = CreateMutex(NULL, TRUE, "UpD435Color");
    UpD435DepthhMutex = CreateMutex(NULL, TRUE, "UpD435Depth");
    DownD435ColorhMutex = CreateMutex(NULL, TRUE, "DownD435Color");
    DownD435DepthhMutex = CreateMutex(NULL, TRUE, "DownD435Depth");

    hThreadT265 = CreateThread(NULL, 0, FunProcT265, NULL, 0, NULL);
    hThreadUpD435Color = CreateThread(NULL, 0, FunProcUpD435Color, NULL, 0, NULL);
    hThreadUpD435Depth = CreateThread(NULL, 0, FunProcUpD435Depth, NULL, 0, NULL);
    hThreadDownD435Color = CreateThread(NULL, 0, FunProcDownD435Color, NULL, 0, NULL);
    hThreadDownD435Depth = CreateThread(NULL, 0, FunProcDownD435Depth, NULL, 0, NULL);

    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Tracking and Depth Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // store pose and timestamp
    rs2::pose_frame pose_frame(nullptr);
    std::vector<rs2_vector> trajectory;

    rs2::context                ctx;            // Create librealsense context for managing devices
    std::vector<rs2::pipeline>  pipelines;

    // Start a streaming pipe per each connected device
    std::string D435i_up = "844212071638";
    std::string T265 = "852212110032";
    std::string D435i_down = "844212071615";
    int height = 720, width = 1280, framerate = 30;
    int i = 0, up_id = 0, down_id = 0;
    for (auto&& dev : ctx.query_devices())
    {
        std::cout << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == D435i_up)
        {
            up_id = i;
            cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, framerate);
            cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, framerate);
        }
        else if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == D435i_down)
        {
            down_id = i;
            cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, framerate);
            cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, framerate);
        }
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        i++;
    }

    rs2::align align_to(RS2_STREAM_COLOR);
    float dec_up = 1, dec_down = 1; // decimation magnitude
    rs2::decimation_filter dec_filter_up;  // Decimation - reduces depth frame density
    dec_filter_up.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_up);
    rs2::decimation_filter dec_filter_down;  // Decimation - reduces depth frame density
    dec_filter_down.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_down);
    rs2::hole_filling_filter hole_filter;
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
                                        // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform depth_to_disparity(true); //将深度图转化为视差
    rs2::disparity_transform disparity_to_depth(false);

    // extrinsics
    // depth w.r.t. tracking (column-major)

    float H_t265_d400[16] = { 1, 0, 0, 0,
                              0,-1, 0, 0,
                              0, 0,-1, 0,
                              0, 0, 0, 1 };
    std::string fn = "./H_t265_d400.cfg";
    std::ifstream ifs(fn);
    if (!ifs.is_open()) {
        std::cerr << "Couldn't open " << fn << std::endl;
        return -1;
    }
    else {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                ifs >> H_t265_d400[i + 4 * j];  // row-major to column-major
            }
        }
    }
    float det = detR(H_t265_d400);
    if (fabs(1 - det) > 1e-6) {
        std::cerr << "Invalid homogeneous transformation matrix input (det != 1)" << std::endl;
        return -1;
    }
    
    cv::VideoWriter writer_color, writer_depth;
    //writer_color.open("G:/realsense/color01.MP4", cv::CAP_OPENCV_MJPEG, 30, cv::Size(width,height), true);
    //writer_depth.open("G:/realsense/depth01.MP4", cv::CAP_OPENCV_MJPEG, 30, cv::Size(width, height), true);
    ReleaseMutex(T265hMutex);
    ReleaseMutex(UpD435ColorhMutex);
    ReleaseMutex(UpD435DepthhMutex);
    ReleaseMutex(DownD435ColorhMutex);
    ReleaseMutex(DownD435DepthhMutex);

    time(&t);
    time_string = std::to_string(t);
    std::cout << file_root + time_string + "/up_color/" << std::endl;
    _mkdir((file_root + time_string).c_str());
    std::cout << "??" << std::endl;
    T265fout.open(file_root + time_string + "/T265output.txt");
    _mkdir((file_root + time_string + "/up_color").c_str());
    _mkdir((file_root + time_string + "/up_depth").c_str());
    _mkdir((file_root + time_string + "/down_color").c_str());
    _mkdir((file_root + time_string + "/down_depth").c_str());
    
    while (app) // Application still alive?
    {
        i = 0;
        clock_t t1 = clock();

        for (auto&& pipe : pipelines) // loop over pipelines
        {
            
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();

            auto color = frames.get_color_frame();

            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            //if (!color)
            //    color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            //if (color)
            //    pc.map_to(color);

            auto depth = frames.get_depth_frame();

            // Generate the pointcloud and texture mappings
            //只有D435才有depth和color
            if (depth)
            {
                frames = frames.apply_filter(align_to);
                //frames = frames.apply_filter(dec_filter_up); //up缩小4倍，从1280*720变到320*180
                //frames = frames.apply_filter(depth_to_disparity);

                //frames = frames.apply_filter(hole_filter);
                //frames = frames.apply_filter(spat_filter);
                //frames = frames.apply_filter(temp_filter);
                //frames = frames.apply_filter(disparity_to_depth);
                auto depth_tmp = frames.get_depth_frame();
                auto color_tmp = frames.get_color_frame();
                int heightd = depth_tmp.get_height(); // number of rows 
                int widthd = depth_tmp.get_width(); // number of columns

                //writer_color.write(colorImg);
                //writer_depth.write(depthImg);
                if (up_id == i)
                {
                    
                    unsigned char* color_frame = (unsigned char*)(color_tmp.get_data());
                    cv::Mat colorImg(720, 1280, CV_8UC3, color_frame);
                    cv::cvtColor(colorImg, UpD435colorImg, cv::COLOR_RGB2BGR);

                    unsigned char* depth_frame = (unsigned char*)(depth_tmp.get_data());
                    cv::Mat depthImg(720, 1280, CV_16UC1, depth_frame);
                    UpD435depthImg = depthImg;

                    WaitForSingleObject(UpD435ColorhMutex, INFINITE);
                    ReleaseSemaphore(UpD435ColorSemaphore, 1, NULL);

                    WaitForSingleObject(UpD435DepthhMutex, INFINITE);
                    ReleaseSemaphore(UpD435DepthSemaphore, 1, NULL);

                }
                else if (down_id == i)
                {
                    unsigned char* color_frame = (unsigned char*)(color_tmp.get_data());
                    cv::Mat colorImg(720, 1280, CV_8UC3, color_frame);
                    cv::cvtColor(colorImg, DownD435colorImg, cv::COLOR_RGB2BGR);

                    unsigned char* depth_frame = (unsigned char*)(depth_tmp.get_data());
                    cv::Mat depthImg(720, 1280, CV_16UC1, depth_frame);
                    DownD435depthImg = depthImg;

                    WaitForSingleObject(DownD435ColorhMutex, INFINITE);
                    ReleaseSemaphore(DownD435ColorSemaphore, 1, NULL);
                    //ReleaseMutex(DownD435ColorhMutex);

                    WaitForSingleObject(DownD435DepthhMutex, INFINITE);
                    ReleaseSemaphore(DownD435DepthSemaphore, 1, NULL);
                    //ReleaseMutex(DownD435DepthhMutex);
                }


                //cv::imshow("color", depthImg);
                //cv::waitKey(1);

                // Apply color map for visualization of depth
                //data = data.apply_filter(color_map);

                //points = pc.calculate(depth);
            }

            // Upload the color frame to OpenGL
            //if (color)
            //    app_state.tex.upload(color);



            auto pose = frames.get_pose_frame();
            // pose，只有T265进来了     
            if (pose) {
                pose_frame = pose;

                // Print the x, y, z values of the translation, relative to initial position
                //auto pose_data = pose.get_pose_data();
                T265_pose = pose.get_pose_data();
                WaitForSingleObject(T265hMutex, INFINITE);
                ReleaseSemaphore(T265Semaphore, 1, NULL);
                //ReleaseMutex(T265hMutex);

                //pose_data是rs2_pose，里面有
                //（rs2_vector是一个结构体，有float x, y, z）
                //rs2_vector translation ——X，Y，Z转换值，以米为单位（相对于初始位置）
                //rs2_vector velocity——X，Y，Z速度值，以米 / 秒为单位
                //rs2_vector acceleration—— X，Y，Z加速度值，以米/秒^2为单位
                //rs2_quaternion  rotation——四元数旋转中表示的旋转的Qi，Qj，Qk，Qr分量（相对于初始位置）
                //rs2_vector angular_velocity——角速度的X，Y，Z值，以弧度/秒为单位
                //rs2_vector angular_acceleration——角加速度的X，Y，Z值，以弧度/秒^ 2为单位
                //unsigned int tracker_confidence;  姿态置信度0x0-失败，0x1-低，0x2-中，0x3-高
                //unsigned int mapper_confidence;   姿势贴图置信度0x0-失败，0x1-低，0x2-中，0x3-高   
                //std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters) " << i % 2;

                // add new point in the trajectory (if motion large enough to reduce size of traj. vector)
                //if (trajectory.size() == 0)
                //    trajectory.push_back(pose_data.translation);
                //else {
                //    rs2_vector prev = trajectory.back();
                //    rs2_vector curr = pose_data.translation;
                //    if (sqrt(pow((curr.x - prev.x), 2) + pow((curr.y - prev.y), 2) + pow((curr.z - prev.z), 2)) > 0.002)
                //        trajectory.push_back(pose_data.translation);
                //}
            }
            i++;
        }

        std::cout << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << std::endl;
        
        // Draw the pointcloud
        //if (points && pose_frame) {
        //    rs2_pose pose = pose_frame.get_pose_data();
        //    draw_pointcloud_wrt_world(app.width(), app.height(), app_state, points, pose, H_t265_d400, trajectory);
        //}

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

DWORD WINAPI FunProcT265(LPVOID lpParameter)
{
    
    while (true)
    {
        WaitForSingleObject(T265Semaphore, INFINITE);
        std::stringstream inter;
        std::string s;
        //inter << std::setw(5) << std::setfill('0') << std::to_string(frame_num);
        //inter >> s;
        //ReleaseMutex(UpD435ColorhMutex);
        //clock_t t1 = clock();
        
        T265fout << T265_pose.translation << ", " << T265_pose.velocity << ", " << T265_pose.acceleration \
            << T265_pose.rotation << ", " << T265_pose.angular_velocity << ", " << T265_pose.angular_acceleration << ", "\
            << T265_pose.tracker_confidence << ", " << T265_pose.mapper_confidence << "\n";

        //cv::imwrite("G:/realsense/up_color/" + s + ".png", UpD435colorImg);
        //std::cout << frame_num << "————" << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << std::endl;
        ReleaseMutex(T265hMutex);
    }
    return 0;
}

DWORD WINAPI FunProcUpD435Color(LPVOID lpParameter)//thread data
{
    int frame_num = 0;
    while (true)
    {
        WaitForSingleObject(UpD435ColorSemaphore, INFINITE);
        std::stringstream inter;
        std::string s;
        inter << std::setw(5) << std::setfill('0') << std::to_string(frame_num);
        inter >> s;
        //clock_t t1 = clock();
        
        cv::imwrite(file_root + time_string + "/up_color/" + s + ".png", UpD435colorImg);
        //std::cout << frame_num << "————" << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << std::endl;
        ReleaseMutex(UpD435ColorhMutex);
        frame_num++;
    }
    return 0;
}

DWORD WINAPI FunProcUpD435Depth(LPVOID lpParameter)//thread data
{
    int frame_num = 0;
    while (true)
    {
        WaitForSingleObject(UpD435DepthSemaphore, INFINITE);
        std::stringstream inter;
        std::string s;
        inter << std::setw(5) << std::setfill('0') << std::to_string(frame_num);
        inter >> s;
        cv::imwrite(file_root + time_string + "/up_depth/" + s + ".png", UpD435depthImg);
        ReleaseMutex(UpD435DepthhMutex);
        frame_num++;
    }
    return 0;
}

DWORD WINAPI FunProcDownD435Color(LPVOID lpParameter)
{
    int frame_num = 0;
    while (true)
    {
        WaitForSingleObject(DownD435ColorSemaphore, INFINITE);
        std::stringstream inter;
        std::string s;
        inter << std::setw(5) << std::setfill('0') << std::to_string(frame_num);
        inter >> s;
        //clock_t t1 = clock();
        cv::imwrite(file_root + time_string + "/down_color/" + s + ".png", DownD435colorImg);
        //std::cout << frame_num << "————" << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << std::endl;
        ReleaseMutex(DownD435ColorhMutex);
        frame_num++;
    }
    return 0;
}

DWORD WINAPI FunProcDownD435Depth(LPVOID lpParameter)
{
    int frame_num = 0;
    while (true)
    {
        WaitForSingleObject(DownD435DepthSemaphore, INFINITE);
        std::stringstream inter;
        std::string s;
        inter << std::setw(5) << std::setfill('0') << std::to_string(frame_num);
        inter >> s;  
        cv::imwrite(file_root + time_string + "/down_depth/" + s + ".png", DownD435depthImg);
        ReleaseMutex(DownD435DepthhMutex);
        frame_num++;
    }
    return 0;
}