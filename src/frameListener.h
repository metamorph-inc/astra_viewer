#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <astra/astra.hpp>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <vector>

#include "common.h"

namespace astra_viewer {

	class FrameListener : public astra::FrameListener
	{
		C_PTR(FrameListener)
		SHARED_PTR(FrameListener)
		UNIQUE_PTR(FrameListener)

		PUBLIC FrameListener() {	}

#define SET_LWH_RESIZE(frame, pcloud) \
length = (frame).length(); \
width = (frame).width(); \
height = (frame).height(); \
(pcloud).resize(length);

		PUBLIC void getWorldDepthData(astra::StreamReader& reader, astra::Frame& frame) {
			const astra::CoordinateMapper cMapper = reader.stream<astra::DepthStream>().coordinateMapper();
			const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
			uint length, height, width;

			if (depthFrame.is_valid()) {
				SET_LWH_RESIZE(depthFrame, this->_depthData)

				for (uint h = 0; h < height; ++h) {
					for (uint w = 0; w < width; ++w) {
						int index = (h*width)+w;
						this->_depthData[index] = cMapper.convert_depth_to_world(astra::Vector3f(w, h, depthFrame.data()[index]));
					}
				}
			}

			else {
				this->_depthData.resize(0);
			}
		}

#define CONVERT_MM_TO_M(mm) (((float)(mm)) / 1000.0)

#define SET_LWH_RESIZE_2(frame, pcloud) \
length = (frame).length(); \
(pcloud).width = width = (frame).width(); \
(pcloud).height = height = (frame).height(); \
(pcloud).resize(length); \
(pcloud).header.frame_id = (frame).frame_index();

#define SET_XYZ(w, h, index, point, depthData) \
if (depthData.size() > 0) { \
	point.x = CONVERT_MM_TO_M(depthData[index].x); \
	point.y = CONVERT_MM_TO_M(depthData[index].y); \
	point.z = CONVERT_MM_TO_M(depthData[index].z); \
} \
else { \
	point.x = CONVERT_MM_TO_M(w); \
	point.y = -CONVERT_MM_TO_M(h); \
	point.z = 0x0; \
}

#define SET_RGBA(point, rColor, gColor, bColor, aColor) \
point.r = (rColor); \
point.g = (gColor); \
point.b = (bColor); \
point.a = (aColor);

		PUBLIC template<typename FrameType> void frameToPointCloud(astra::Frame& frame) {
			FrameType f = frame.get<FrameType>();
			uint length, height, width;

			if (std::is_base_of<astra::DepthFrame, FrameType>()) {
				astra::DepthFrame* framePtr = (astra::DepthFrame*)(&f);
				if (framePtr->is_valid()) {
					SET_LWH_RESIZE_2(*framePtr, this->_depthCloud)
					for (uint h = 0; h < height; ++h) {
						for (uint w = 0; w < width; ++w) {
							uint index = (h*width)+w;
							SET_XYZ(w, h, index, this->_depthCloud.operator[](index), this->_depthData)
							SET_RGBA(this->_depthCloud.operator[](index), 0xFF, 0xFF, 0xFF, 0xFF)
						}
					}
				}
				
				else {
					this->_depthCloud.resize(0);
				}
			}

#define SET_RGBA_2(point, pixel, aColor) \
point.r = pixel.r; \
point.g = pixel.g; \
point.b = pixel.b; \
point.a = (aColor);

			else if (std::is_base_of<astra::ColorFrame, FrameType>()) {
				astra::ColorFrame* framePtr = (astra::ColorFrame*)(&f);
				if (framePtr->is_valid()) {
					SET_LWH_RESIZE_2(*framePtr, this->_colorCloud)
					for (uint h = 0; h < height; ++h) {
						for (uint w = 0; w < width; ++w) {
							uint index = (h*width)+w;
							SET_XYZ(w, h, index, this->_colorCloud.operator[](index), this->_depthData)
							SET_RGBA_2(this->_colorCloud.operator[](index), framePtr->data()[index], 0xFF)
						}
					}
				}
				
				else {
					this->_colorCloud.resize(0);
				}
			}

			else if (std::is_base_of<astra::InfraredFrame16, FrameType>()) {
				astra::InfraredFrame16* framePtr = (astra::InfraredFrame16*)(&f);
				if (framePtr->is_valid()) {
					SET_LWH_RESIZE_2(*framePtr, this->_ir16Cloud)
					for (uint h = 0; h < height; ++h) {
						for (uint w = 0; w < width; ++w) {
							uint index = (h*width)+w;
							SET_XYZ(w, h, index, this->_ir16Cloud.operator[](index), this->_depthData)
							uint16_t color = framePtr->data()[index];
							uint8_t r = static_cast<uint8_t>(color >> 2);
							uint8_t b = 0x66 - r / 2;
							SET_RGBA(this->_ir16Cloud.operator[](index), r, 0, b, 0xFF)
						}
					}
				}
				
				else {
					this->_ir16Cloud.resize(0);
				}
			}

			else if (std::is_base_of<astra::InfraredFrameRgb, FrameType>()) {
				astra::InfraredFrameRgb* framePtr = (astra::InfraredFrameRgb*)(&f);
				if (framePtr->is_valid()) {
					SET_LWH_RESIZE_2(*framePtr, this->_irRgbCloud)
					for (uint h = 0; h < height; ++h) {
						for (uint w = 0; w < width; ++w) {
							uint index = (h*width)+w;
							SET_XYZ(w, h, index, this->_irRgbCloud.operator[](index), this->_depthData)
							SET_RGBA_2(this->_irRgbCloud.operator[](index), framePtr->data()[index], 0xFF)
						}
					}
				}
				
				else {
					this->_irRgbCloud.resize(0);
				}
			}
		}

		PUBLIC virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
		{
			this->getWorldDepthData(reader, frame);
			this->frameToPointCloud<astra::DepthFrame>(frame);
			this->frameToPointCloud<astra::PointFrame>(frame);
			this->frameToPointCloud<astra::ColorFrame>(frame);
			this->frameToPointCloud<astra::InfraredFrame16>(frame);
			this->frameToPointCloud<astra::InfraredFrameRgb>(frame);
			this->_time = ros::Time::now();
		}

		typedef pcl::PointCloud<pcl::PointXYZRGBA> FrameData;
		typedef std::vector<astra::Vector3f> DepthData;
		PUBLIC DepthData _depthData;
		PUBLIC FrameData _depthCloud;
		PUBLIC FrameData _colorCloud;
		PUBLIC FrameData _ir16Cloud;
		PUBLIC FrameData _irRgbCloud;
		PUBLIC ros::Time _time;
	};

}