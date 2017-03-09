// Copyright 2017 Umar Arshad

#include <openvr.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <experimental/filesystem>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

//opencv_core320.lib; opencv_highgui320.lib; opencv_imgproc320.lib; opencv_imgcodecs320.lib;

// The type of images to capture
static const vr::EVRTrackedCameraFrameType frame_type = vr::EVRTrackedCameraFrameType::VRTrackedCameraFrameType_Undistorted;

int main(int argc, char ** argv) {
    std::string args = R"(
    { n | 100 | Number of frames to capture}
    { p | vr_data | Path to data })";
    cv::CommandLineParser parse(argc, argv, args);

    const int n_frames = parse.get<int>("n");
    const std::string data_path = parse.get<std::string>("p");

    // Initialize OpenVR
    vr::EVRInitError err;
    auto system = vr::VR_Init(&err, vr::EVRApplicationType::VRApplication_Utility);
    if (err) { fprintf(stderr, "Error initializing OpenVR.\n"); return EXIT_FAILURE; }

    // Setup front facing camera
    auto camera = vr::VRTrackedCamera();
    vr::TrackedCameraHandle_t cam_handle;
    camera->AcquireVideoStreamingService(0, &cam_handle);

    // Create directory
    auto path = std::experimental::filesystem::path(data_path);

    if (path.is_relative()) {
        printf("relitive path %s", path.string().c_str());
        path = std::experimental::filesystem::current_path().append(data_path);
    }
    auto save_path = path;
    std::experimental::filesystem::create_directory(save_path);
    printf("saving in: %s\n", save_path.string().c_str());

    // Get width, height and camera intrensics of the image
    uint32_t w, h, buffer_size;
    camera->GetCameraFrameSize(0, frame_type, &w, &h, &buffer_size);
    vr::HmdVector2_t focal_length, image_center;
    camera->GetCameraIntrinsics(0, frame_type, &focal_length, &image_center);
    cv::Mat_<float> camera_intrensics(3, 3);
    camera_intrensics << focal_length.v[0], 0.f, image_center.v[0], 0.f, focal_length.v[1], image_center.v[1], 0.f, 0.f, 1.0f;

    printf("Width:\t%10d\nHeight:\t%10d\nSize:\t%10d\n", w, h, buffer_size);
    cv::Size_<int> image_size{ (int)w, (int)h };

    cv::FileStorage fs(save_path.string() + "/pose.yaml", cv::FileStorage::Mode::WRITE | cv::FileStorage::Mode::FORMAT_YAML);
    fs << "image_size" << image_size;
    fs << "camera_intrensics" << camera_intrensics;

    for (int i = 0; i < n_frames; i++) {
        std::vector<uint8_t> buffer(buffer_size);
        vr::CameraVideoStreamFrameHeader_t cam_header;
        camera->GetVideoStreamFrameBuffer(cam_handle, frame_type, buffer.data(), buffer_size, &cam_header, sizeof(vr::CameraVideoStreamFrameHeader_t));

        // Convert and display image
        auto type = (cam_header.nBytesPerPixel == 4) ? CV_8UC4 : CV_8UC3;
        cv::Mat frame(cam_header.nHeight, cam_header.nWidth, type, buffer.data());
        cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGBA);
        cv::imshow("image", frame);
        cv::waitKey(1);

        // Save image and image path
        std::stringstream image_path;
        std::stringstream image_key;
        image_path << save_path.string() << "/image" << i << ".jpg";
        image_key << "image_path" << i;
        cv::imwrite(image_path.str().c_str(), frame);
        fs << image_key.str() << std::experimental::filesystem::path(image_path.str()).filename().string();

        printf("Sequence Number: %d\n", cam_header.nFrameSequence);

        // Save pose
        cv::Mat pose(3, 4, CV_32F, cam_header.standingTrackedDevicePose.mDeviceToAbsoluteTracking.m);
        std::stringstream pose_key;
        pose_key << "pose" << i;
        fs << pose_key.str() << pose;
        //std::cout << pose << std::endl;
    }

    // Release video and shutdown OpenVR
    camera->ReleaseVideoStreamingService(cam_handle);
    vr::VR_Shutdown();
    return 0;
}