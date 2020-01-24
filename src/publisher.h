#pragma once

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <string>
#include <map>
#include <boost/algorithm/string.hpp>
#include <astra/astra.hpp>

#include "common.h"
#include "frameListener.h"

namespace astra_viewer {

    class Publisher {
        PUBLIC enum StreamType {
            NONE = 0,
            DEPTH = 1,
            COLOR = 2,
            IR16 = 3,
            IRRGB = 4
        };
        
        PUBLIC struct StreamConfiguration {
            StreamConfiguration() {
                this->streamType = Publisher::StreamType::NONE;
            }

            Publisher::StreamType streamType;
            std::string streamTypeString;
            bool image, pointcloud;
            bool enable_mirroring;
        };

        PRIVATE struct {
            uint height, width, fps;
            bool enable_registration;
        } _globalConfiguration;

        PUBLIC Publisher(astra::StreamReader& reader) {
            this->_publishers = new ros::Publisher[Publisher::NUMBER_OF_PUBLISHERS];
            this->_dataStreams = new astra::DataStream[Publisher::NUMBER_OF_STREAMS];
            this->_streamConfigurations = new Publisher::StreamConfiguration[Publisher::NUMBER_OF_STREAMS];
            _nh = ros::NodeHandle("~");
            this->_globalConfiguration.height = _nh.param("height", 480);
            this->_globalConfiguration.width = _nh.param("width", 640);
            this->_globalConfiguration.fps = _nh.param("fps", 30);
            this->_globalConfiguration.enable_registration = _nh.param("enable_registration", false);

            std::stringstream ss(_nh.param("streams", std::string("color")));
            std::string streamType;
            uint streamID = 0;
            while (std::getline(ss, streamType, ',') && streamID < Publisher::NUMBER_OF_STREAMS) {
                boost::to_lower(streamType);
                if (Publisher::streamType_enumMapper.count(streamType)) {
                    this->configureStream(streamType, reader, streamID++);
                }
            }
            this->configurePublishers();
        }

        PUBLIC ~Publisher() {
            std::cout << "Deconstructing Publisher" << std::endl;
            delete[] this->_publishers;
            delete[] this->_dataStreams;
            delete[] this->_streamConfigurations;
        }

#define GET_PARAMS(config, nodeHandle, streamTypeString, streamTypeEnum) \
config.streamType = streamTypeEnum; \
config.streamTypeString = streamTypeString; \
config.image = nodeHandle.param(streamTypeString + "/image", true); \
config.pointcloud = nodeHandle.param(streamTypeString + "/pointcloud", true); \
config.enable_mirroring = nodeHandle.param(streamTypeString + "/enable_mirroring", true);

#define SET_STREAM_CONFIG_AND_START(stream, config, mode, pixelFormat) \
mode.set_pixel_format(pixelFormat); \
stream.enable_mirroring(config.enable_mirroring); \
stream.set_mode(mode); \
std::cout << "Starting Stream: " << config.streamTypeString << std::endl; \
stream.start();

        PRIVATE void configureStream(const std::string streamTypeString, astra::StreamReader& reader, const uint& streamID) {
            Publisher::StreamType streamType = Publisher::streamType_enumMapper.at(streamTypeString);
            Publisher::StreamConfiguration config;
            astra::ImageStreamMode mode;
            mode.set_width(this->_globalConfiguration.width);
            mode.set_height(this->_globalConfiguration.height);
            mode.set_fps(this->_globalConfiguration.fps);
            switch(streamType) {
                case Publisher::StreamType::DEPTH: {
                    astra::DepthStream stream = reader.stream<astra::DepthStream>();
                    stream.enable_registration(this->_globalConfiguration.enable_registration);
                    GET_PARAMS(config, this->_nh, streamTypeString, streamType);
                    SET_STREAM_CONFIG_AND_START(stream, config, mode, astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
                    this->_dataStreams[streamID] = stream;
                    break;
                }
                case Publisher::StreamType::COLOR: {
                    astra::ColorStream stream = reader.stream<astra::ColorStream>();
                    GET_PARAMS(config, this->_nh, streamTypeString, streamType);
                    SET_STREAM_CONFIG_AND_START(stream, config, mode, astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
                    this->_dataStreams[streamID] = stream;
                    break;
                }
                case Publisher::StreamType::IR16: {
                    astra::InfraredStream stream = reader.stream<astra::InfraredStream>();
                    GET_PARAMS(config, this->_nh, streamTypeString, streamType);
                    SET_STREAM_CONFIG_AND_START(stream, config, mode, astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
                    this->_dataStreams[streamID] = stream;
                    break;
                }
                case Publisher::StreamType::IRRGB: {
                    astra::InfraredStream stream = reader.stream<astra::InfraredStream>();
                    GET_PARAMS(config, this->_nh, streamTypeString, streamType);
                    SET_STREAM_CONFIG_AND_START(stream, config, mode, astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
                    this->_dataStreams[streamID] = stream;
                    break;
                }
            }

            this->_streamConfigurations[streamID] = config;
        }

        PRIVATE void configurePublishers() {
            std::cout << "Configuring Publishers" << std::endl;
            uint currentStreamConfiguration = 0;
            uint currentPublisher = 0;
            while (currentStreamConfiguration < Publisher::NUMBER_OF_STREAMS
            && currentPublisher < Publisher::NUMBER_OF_PUBLISHERS) {
                Publisher::StreamConfiguration* config = &(this->_streamConfigurations[currentStreamConfiguration]);
                if (config->streamType) {
                    if (config->pointcloud) {
                        std::string topic = "/astra_camera/" + config->streamTypeString + "/pointcloud";
                        std::cout << "Creating Topic: " << topic << std::endl;
                        this->_publishers[currentPublisher++] 
                            = this->_nh.advertise<sensor_msgs::PointCloud2>(topic, 1000);
                    }
                    if (config->image) {
                        std::string topic = "/astra_camera/" + config->streamTypeString + "/image";
                        std::cout << "Creating Topic: " << topic << std::endl;
                        this->_publishers[currentPublisher++] 
                            = this->_nh.advertise<sensor_msgs::Image>(topic, 1000);
                    }
                }
                ++currentStreamConfiguration;
            }
        }

        PUBLIC void publishCameraFeed(FrameListener::Shared_Ptr listener) {
            uint currentStreamConfiguration = 0;
            uint currentPublisher = 0;
            while (currentStreamConfiguration < Publisher::NUMBER_OF_STREAMS
            && currentPublisher < Publisher::NUMBER_OF_PUBLISHERS) {
                Publisher::StreamConfiguration* config = &(this->_streamConfigurations[currentStreamConfiguration]);
                if (config->streamType) {
                    pcl::PointCloud<pcl::PointXYZRGBA>* pcloud;
                    switch(config->streamType) {
                        case Publisher::StreamType::DEPTH: {
                            pcloud = &(listener->_depthCloud);
                            break;
                        }
                        case Publisher::StreamType::COLOR: {
                            pcloud = &(listener->_colorCloud);
                            break;
                        }
                        case Publisher::StreamType::IR16: {
                            pcloud = &(listener->_ir16Cloud);
                            break;
                        }
                        case Publisher::StreamType::IRRGB: {
                            pcloud = &(listener->_irRgbCloud);
                            break;
                        }
                    }
                    if (config->pointcloud) {
                        sensor_msgs::PointCloud2 pcloud2;
                        pcl::toROSMsg((*pcloud), pcloud2);
                        pcloud2.header.stamp = listener->_time;
                        pcloud2.header.frame_id = "astra_camera";
                        this->_publishers[currentPublisher++].publish(pcloud2);
                    }
                    if (config->image) {
                        sensor_msgs::Image image;
                        uint height = pcloud->height;
                        uint width = pcloud->width;
                        for (uint h = 0; h < height; ++h) {
                            for (uint w = 0; w < width / 2; ++w) {
                                uint32_t tempRgba;
                                tempRgba = pcloud->operator[]((h*width)+w).rgba;
                                pcloud->operator[]((h*width)+w).rgba = pcloud->operator[]((h*width)+(width-w-1)).rgba;
                                pcloud->operator[]((h*width)+(width-w-1)).rgba = tempRgba;
                            }
                        }
                        pcl::toROSMsg((*pcloud), image);
                        image.header.stamp = listener->_time;
                        image.header.frame_id = "astra_camera";
                        this->_publishers[currentPublisher++].publish(image);
                    }
                }
                ++currentStreamConfiguration;
            }
        }

        PUBLIC static const std::map<std::string, Publisher::StreamType> streamType_enumMapper;
        PUBLIC const static int NUMBER_OF_PUBLISHERS = 8;
        PUBLIC const static int NUMBER_OF_STREAMS = 4;

        PRIVATE ros::NodeHandle _nh;
        PRIVATE ros::Publisher* _publishers;
        PRIVATE astra::DataStream* _dataStreams;
        PRIVATE Publisher::StreamConfiguration* _streamConfigurations;
    };

}