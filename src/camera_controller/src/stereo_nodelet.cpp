#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>
#include <tuple>
#include <iostream>
#include <vector>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

using namespace std;
namespace camera_controller
{

    class StereoNodelet : public nodelet::Nodelet
    {

        std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish, rgbPublish;
        std::unique_ptr<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>> detectionPublish;
        std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter, rgbConverter;
        std::unique_ptr<dai::rosBridge::ImgDetectionConverter> detConverter;
        std::unique_ptr<dai::Device> _dev;

    public:
        virtual void onInit() override
        {   

            auto &pnh = getPrivateNodeHandle();

            std::string tfPrefix, mode;
            std::string calibration_file;
            std::string monoResolution = "400p";
            int badParams = 0;
            bool lrcheck, extended, subpixel, enableDepth;
            std::string mxid;
            int confidence = 200;
            int LRchecktresh = 5;


            std::string resourceBaseFolder; 
            std::string nnPath;
            std::string nnName;
            bool syncNN(true);
            badParams += !pnh.getParam("calibration_file", calibration_file);
            badParams += !pnh.getParam("tf_prefix", tfPrefix);
            badParams += !pnh.getParam("mode", mode);
            badParams += !pnh.getParam("lrcheck", lrcheck);
            badParams += !pnh.getParam("extended", extended);
            badParams += !pnh.getParam("subpixel", subpixel);
            badParams += !pnh.getParam("confidence", confidence);
            badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
            badParams += !pnh.getParam("monoResolution", monoResolution);
            badParams += !pnh.getParam("tf_prefix", tfPrefix);
            badParams += !pnh.getParam("sync_nn", syncNN);
            badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);
            badParams += !pnh.getParam("MXID", mxid);

            if (badParams > 0)
            {
                std::cout << " Bad parameters -> " << badParams << std::endl;
                throw std::runtime_error("Couldn't find %d of the parameters");
            }

            if (mode == "depth")
            {
                enableDepth = true;
            }
            else
            {
                enableDepth = false;
            }

            dai::Pipeline pipeline;
            int monoWidth, monoHeight;

            // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
            if (pnh.hasParam("nnName"))
            {
                pnh.getParam("nnName", nnName);
            }

            if (resourceBaseFolder.empty())
            {
                throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
            }

            nnPath = resourceBaseFolder + "/" + nnName;
            bool found;
            dai::DeviceInfo device_info;

            std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, syncNN, nnPath, calibration_file);

            if (mxid.empty())
            {
                _dev = std::make_unique<dai::Device>(pipeline);
            }
            else
            {
                cout << "Connecting device with MXID: " << mxid << endl;
                std::tie(found, device_info) = dai::Device::getDeviceByMxId(mxid);
                _dev = std::make_unique<dai::Device>(pipeline, device_info, false); 
            }
            
            auto leftQueue = _dev->getOutputQueue("left", 15, false);
            auto rightQueue = _dev->getOutputQueue("right", 15, false);

            std::shared_ptr<dai::DataOutputQueue> stereoQueue;
            if (enableDepth)
            {
                stereoQueue = _dev->getOutputQueue("depth", 15, false);
            }
            else
            {
                stereoQueue = _dev->getOutputQueue("disparity", 15, false);
            }

            std::shared_ptr<dai::DataOutputQueue> previewQueue = _dev->getOutputQueue("preview", 15, false);
            std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = _dev->getOutputQueue("detections", 15, false);
            auto calibrationHandler = _dev->readCalibration();


            auto boardName = calibrationHandler.getEepromData().boardName;
            if (monoHeight > 480 && boardName == "OAK-D-LITE")
            {
                monoWidth = 640;
                monoHeight = 480;
            }

            leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_left_camera_optical_frame", true);
            auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight);

            leftPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(leftQueue,
                                                                                                               pnh,
                                                                                                               std::string("left/image"),
                                                                                                               std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                                         leftConverter.get(),
                                                                                                                         std::placeholders::_1,
                                                                                                                         std::placeholders::_2),
                                                                                                               15,
                                                                                                               leftCameraInfo,
                                                                                                               "left");

            // bridgePublish.startPublisherThread();
            leftPublish->addPublisherCallback();

            rightConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_right_camera_optical_frame", true);
            auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);

            rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(rightQueue,
                                                                                                                pnh,
                                                                                                                std::string("right/image"),
                                                                                                                std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                                          rightConverter.get(),
                                                                                                                          std::placeholders::_1,
                                                                                                                          std::placeholders::_2),
                                                                                                                15,
                                                                                                                rightCameraInfo,
                                                                                                                "right");

            rightPublish->addPublisherCallback();

            // dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame");
            depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(stereoQueue,
                                                                                                                pnh,
                                                                                                                std::string("stereo/depth"),
                                                                                                                std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                                          rightConverter.get(), // since the converter has the same frame name
                                                                                                                                                // and image type is also same we can reuse it
                                                                                                                          std::placeholders::_1,
                                                                                                                          std::placeholders::_2),
                                                                                                                15,
                                                                                                                rightCameraInfo,
                                                                                                                "stereo");

            depthPublish->addPublisherCallback();
        
            auto rgbCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, monoWidth, monoHeight);
            rgbConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", true);
            rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(previewQueue,
                                                                                                              pnh,
                                                                                                              std::string("color/image"),
                                                                                                              std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                                        rgbConverter.get(), // since the converter has the same frame name
                                                                                                                                            // and image type is also same we can reuse it
                                                                                                                        std::placeholders::_1,
                                                                                                                        std::placeholders::_2),
                                                                                                              15,
                                                                                                              rgbCameraInfo,
                                                                                                              "color");
            rgbPublish->addPublisherCallback(); // addPublisherCallback works only when the dataqueue is non blocking.

            detConverter = std::make_unique<dai::rosBridge::ImgDetectionConverter>(tfPrefix + "_rgb_camera_optical_frame", monoHeight, monoWidth, false);
            detectionPublish = std::make_unique<dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections>>(
                nNetDataQueue,
                pnh,
                std::string("color/mobilenet_detections"),
                std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, detConverter.get(), std::placeholders::_1, std::placeholders::_2),
                15);

            detectionPublish->addPublisherCallback();
        }

        std::tuple<dai::Pipeline, int, int> createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution, bool syncNN, std::string nnPath, std::string calib_path)
        {
            dai::CalibrationHandler calibData(calib_path);

            // Create pipeline
            dai::Pipeline pipeline;
            pipeline.setCalibrationData(calibData);

            auto monoLeft = pipeline.create<dai::node::MonoCamera>();
            auto monoRight = pipeline.create<dai::node::MonoCamera>();
            auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
            auto xoutRight = pipeline.create<dai::node::XLinkOut>();
            auto stereo = pipeline.create<dai::node::StereoDepth>();
            auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

            // XLinkOut
            xoutLeft->setStreamName("left");
            xoutRight->setStreamName("right");

            if (withDepth)
            {
                xoutDepth->setStreamName("depth");
            }
            else
            {
                xoutDepth->setStreamName("disparity");
            }

            int width, height;
            dai::node::MonoCamera::Properties::SensorResolution monoResolution;
            if (resolution == "720p")
            {
                monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
                width = 1280;
                height = 720;
            }
            else if (resolution == "400p")
            {
                monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
                width = 640;
                height = 400;
            }
            else if (resolution == "800p")
            {
                monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
                width = 1280;
                height = 800;
            }
            else if (resolution == "480p")
            {
                monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
                width = 640;
                height = 480;
            }
            else
            {
                ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
                throw std::runtime_error("Invalid mono camera resolution.");
            }

            // MonoCamera
            monoLeft->setResolution(monoResolution);
            monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
            monoRight->setResolution(monoResolution);
            monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

            // StereoDepth
            stereo->initialConfig.setConfidenceThreshold(confidence);
            stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
            stereo->setRectification(true);
            stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

            stereo->setLeftRightCheck(lrcheck);
            stereo->setExtendedDisparity(extended);
            stereo->setSubpixel(subpixel);

            // Link plugins CAM -> STEREO -> XLINK
            monoLeft->out.link(stereo->left);
            monoRight->out.link(stereo->right);

            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);

            if (withDepth)
            {
                stereo->depth.link(xoutDepth->input);
            }
            else
            {
                stereo->disparity.link(xoutDepth->input);
            }
            auto manip = pipeline.create<dai::node::ImageManip>();
            auto colorCam = pipeline.create<dai::node::ColorCamera>(); // returns dai::node:ColorCamera
            colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);

            auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
            auto xlinkIn = pipeline.create<dai::node::XLinkIn>();

            auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
            auto nnOut = pipeline.create<dai::node::XLinkOut>();

            xlinkIn->setStreamName("caminput");
            xlinkOut->setStreamName("preview");
            nnOut->setStreamName("detections");

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

            // Link plugins CAM -> IMMANIP -> NN -> XLINK
            colorCam->preview.link(manip->inputImage);
            manip->out.link(detectionNetwork->input);
            if (syncNN)
            {
                detectionNetwork->passthrough.link(xlinkOut->input);
            }
            else
            {
                colorCam->preview.link(xlinkOut->input);
            }

            detectionNetwork->out.link(nnOut->input);

            return std::make_tuple(pipeline, width, height);
        }
    };

    PLUGINLIB_EXPORT_CLASS(camera_controller::StereoNodelet, nodelet::Nodelet)
} // namespace camera_controller
