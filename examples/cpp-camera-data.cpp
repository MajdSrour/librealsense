// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include "example.hpp"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <algorithm>
#include <memory>



int main(int argc, char * argv[]) try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if (ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    std::vector<rs::stream> supported_streams;

    for (int i = (int)rs::capabilities::depth; i <= (int)rs::capabilities::fish_eye; i++)
        if (dev.supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);
    // Configure all supported streams to run at 30 frames per second
    for (auto & stream : supported_streams)
        dev.enable_stream(stream, rs::preset::best_quality);
    // Compute field of view for each enabled stream
    for (auto & stream : supported_streams)
    {
        if (!dev.is_stream_enabled(stream)) continue;
        auto intrin = dev.get_stream_intrinsics(stream);
        std::cout << "Intrinsics for " << stream << " at Resolution: " << intrin.width << " x " << intrin.height << std::endl;
        std::cout << "Principal point (x,y) " << intrin.ppx <<" , " <<intrin.ppy << std::endl;
        std::cout << "focal Length (fx,fy) " << intrin.fx << " , "<< intrin.fy << std::endl;
        std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
        std::cout << std::endl;
    }
    
    for (auto & stream : supported_streams)
    {
        if (!dev.is_stream_enabled(stream)) continue;
        if( stream != rs::stream::fisheye)
        {
          auto extrin = dev.get_extrinsics(rs::stream::fisheye,stream );
          std::cout << "Extrinsics for FISHEYE to " << stream << std::endl;
          std::cout << "Rotation Matrix : " << std::endl;
          std::cout << extrin.rotation[0] << " " << extrin.rotation[3] << " " << extrin.rotation[6] <<std::endl;
          std::cout << extrin.rotation[1] << " " << extrin.rotation[4] << " " << extrin.rotation[7] <<std::endl;
          std::cout << extrin.rotation[2] << " " << extrin.rotation[5] << " " << extrin.rotation[8] <<std::endl;
          std::cout << std::endl;
          
          std::cout << "Translation Vector: "  << std::endl;
          std::cout << extrin.translation[0] << " " << extrin.translation[1] << " " << extrin.translation[2] << std::endl;
          std::cout << std::endl;
        }        
    }
    
    for (auto & stream : supported_streams)
    {
        if (!dev.is_stream_enabled(stream)) continue;
         if( stream != rs::stream::infrared && stream != rs::stream::infrared2)
         {
          auto motionExtrin = dev.get_motion_extrinsics_from(stream);
          std::cout << "MOTION extrinsics from " << stream << std::endl;
          std::cout << "Rotation Matrix : " << std::endl;
          std::cout << motionExtrin.rotation[0] << " " << motionExtrin.rotation[3] << " " << motionExtrin.rotation[6] <<std::endl;
          std::cout << motionExtrin.rotation[1] << " " << motionExtrin.rotation[4] << " " << motionExtrin.rotation[7] <<std::endl;
          std::cout << motionExtrin.rotation[2] << " " << motionExtrin.rotation[5] << " " << motionExtrin.rotation[8] <<std::endl;
          std::cout << std::endl;
          
          std::cout << "Translation Vector: "  << std::endl;
          std::cout << motionExtrin.translation[0] << " " << motionExtrin.translation[1] << " " << motionExtrin.translation[2] << std::endl;
          std::cout << std::endl;
          }
    }
    
    
    
    return EXIT_SUCCESS;
}
catch (const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
