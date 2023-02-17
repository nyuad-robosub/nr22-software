
#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/Detection2DArray.h>

#include <cstdio>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
using namespace std;
std::tuple<dai::Pipeline, int, int> createMonoPipeline(bool syncNN, std::string nnPath, std::string calib_path)
{
    dai::CalibrationHandler calibData(calib_path);

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setCalibrationData(calibData);

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("previeww");
    nnOut->setStreamName("detectionss");

    colorCam->setPreviewSize(640, 400);
    colorCam->setPreviewKeepAspectRatio(false);

    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(15);

    // Convert the grayscale frame into the nn-acceptable form
    manip->initialConfig.setResize(300, 300);
    // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM PREVIEW(1920X1080) -> IMMANIP(300X300) -> NN -> XLINK
    colorCam->preview.link(manip->inputImage);
    manip->out.link(detectionNetwork->input);

    if (syncNN)
        detectionNetwork->passthrough.link(xlinkOut->input);
    else
        colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);
    int width = 1920;
    int height = 1080;
    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobilenet_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix;
    std::string resourceBaseFolder, nnPath;
    std::string nnName;
    std::string calibration_file;
    std::string mxid;

    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish, rgbPublish, rgbPublishOAK;
    std::unique_ptr<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>> detectionPublish, detectionPublishOAK;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter, rgbConverter, rgbConverterOAK;
    std::unique_ptr<dai::rosBridge::ImgDetectionConverter> detConverter, detConverterOAK;
    std::unique_ptr<dai::Device> _dev, _dev_mono, _dev_temp;

    bool syncNN;
    int badParams = 0;

    badParams += !pnh.getParam("calibration_file", calibration_file);
    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("sync_nn", syncNN);
    badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);
    badParams += !pnh.getParam("MXID", mxid);
    badParams += !pnh.getParam("nnName", nnName);

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    if (resourceBaseFolder.empty())
    {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    nnPath = resourceBaseFolder + "/" + nnName;

    dai::Pipeline pipeline;
    bool found;
    dai::DeviceInfo device_info, device_info2;
    int rgbWidth, rgbHeight;

    std::tie(pipeline, rgbWidth, rgbHeight) = createMonoPipeline(syncNN, nnPath, calibration_file);

    if (mxid.empty())
    {
        _dev_mono = std::make_unique<dai::Device>(pipeline);
    }
    else
    {
        std::tie(found, device_info2) = dai::Device::getDeviceByMxId("1844301081F1670F00");
        _dev_mono = std::make_unique<dai::Device>(pipeline, device_info2); // OAK-1 MXID: 1844301081F1670F00
    }

    std::shared_ptr<dai::DataOutputQueue> previewQueueRGB = _dev_mono->getOutputQueue("previeww", 15, false);
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueueRGB = _dev_mono->getOutputQueue("detectionss", 15, false);

    // OAK-1 PUBLISHERS
    rgbConverterOAK = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_OAK1_camera_optical_frame", true);
    rgbPublishOAK = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(previewQueueRGB,
                                                                                                         pnh,
                                                                                                         std::string("OAK1/image"),
                                                                                                         std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                                   rgbConverterOAK.get(), // since the converter has the same frame name
                                                                                                                                          // and image type is also same we can reuse it
                                                                                                                   std::placeholders::_1,
                                                                                                                   std::placeholders::_2),
                                                                                                         15,
                                                                                                         "",
                                                                                                         "OAK1");

    detConverterOAK = std::make_unique<dai::rosBridge::ImgDetectionConverter>(tfPrefix + "_OAK1_camera_optical_frame", rgbHeight, rgbWidth, false); // 640 by 400

    detectionPublishOAK = std::make_unique<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>>(
        nNetDataQueueRGB,
        pnh,
        std::string("OAK1/mobilenet_detections"),
        std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, detConverterOAK.get(), std::placeholders::_1, std::placeholders::_2),
        15);

    cout << "PUBLISHER CALLBACK" << endl;
    detectionPublishOAK->addPublisherCallback();
    rgbPublishOAK->addPublisherCallback();
    ros::spin();

    return 0;
}
