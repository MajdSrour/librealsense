// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "image.h"
#include "lr200_mm.h"
#include "rapidjson/document.h"
#include <iostream>
#include <fstream>
#include <streambuf>
#include <motion/MotionDeviceDescriptor.h>

using namespace rsimpl;
using namespace rsimpl::ds;

namespace rsimpl
{
     

    lr200_mm_camera::lr200_mm_camera(std::shared_ptr<uvc::device> device, const static_device_info & info, motion_module_calibration in_fe_intrinsic)
        : ds_device(device, info, calibration_validator()),
          fe_intrinsic(in_fe_intrinsic)
    {
    }

    void lr200_mm_camera::start_fw_logger(char fw_log_op_code, int grab_rate_in_ms, std::timed_mutex& mutex)
    {
        throw std::logic_error("Not implemented");
    }

    void lr200_mm_camera::stop_fw_logger()
    {
        throw std::logic_error("Not implemented");
    }
    
    void lr200_mm_camera::get_option_range(rs_option option, double & min, double & max, double & step, double & def)
    {
        
        ds_device::get_option_range(option, min, max, step, def);
    
    }
    void lr200_mm_camera::set_options(const rs_option options[], size_t count, const double values[])
    {
        std::vector<rs_option>  base_opt;
        std::vector<double>     base_opt_val;

        auto & dev = get_device();

        // Handle LR200 specific options first
        for (size_t i = 0; i < count; ++i)
        {
           

            switch (options[i])
            {
            case RS_OPTION_FISHEYE_STROBE:                            
            case RS_OPTION_FISHEYE_EXTERNAL_TRIGGER:                  
            case RS_OPTION_FISHEYE_EXPOSURE:                          
            case RS_OPTION_FISHEYE_ENABLE_AUTO_EXPOSURE:              
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_MODE:                
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_ANTIFLICKER_RATE:    
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_PIXEL_SAMPLE_RATE:   
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_SKIP_FRAMES:         
            case RS_OPTION_HARDWARE_LOGGER_ENABLED:                   break; 

                // Default will be handled by parent implementation
            default: base_opt.push_back(options[i]); base_opt_val.push_back(values[i]); break;
            }
        }

        //Then handle the common options
        if (base_opt.size())
            ds_device::set_options(base_opt.data(), base_opt.size(), base_opt_val.data());
    }
    void lr200_mm_camera::get_options(const rs_option options[], size_t count, double values[])
    {
        std::vector<rs_option>  base_opt;
        std::vector<size_t>     base_opt_index;
        std::vector<double>     base_opt_val;

        auto & dev = get_device();

        // Acquire ZR300-specific options first
        for (size_t i = 0; i<count; ++i)
        {
          
            
            switch (options[i])
            {
            case RS_OPTION_FISHEYE_STROBE:                            
            case RS_OPTION_FISHEYE_EXTERNAL_TRIGGER:                  
            case RS_OPTION_FISHEYE_EXPOSURE:                          
            case RS_OPTION_FISHEYE_ENABLE_AUTO_EXPOSURE:              
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_MODE:                
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_ANTIFLICKER_RATE:    
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_PIXEL_SAMPLE_RATE:   
            case RS_OPTION_FISHEYE_AUTO_EXPOSURE_SKIP_FRAMES:         
            case RS_OPTION_HARDWARE_LOGGER_ENABLED:                   break; 

                // Default will be handled by parent implementation
            default: base_opt.push_back(options[i]); base_opt_index.push_back(i); break;
            }
        }

        //Then retrieve the common options
        if (base_opt.size())
        {
            base_opt_val.resize(base_opt.size());
            ds_device::get_options(base_opt.data(), base_opt.size(), base_opt_val.data());
        }

        // Merge the local data with values obtained by base class
        for (auto i : base_opt_index)
            values[i] = base_opt_val[i];
    }
    void lr200_mm_camera::initialize_motion() {

        if(motion_initialized)
            return;
        motion_initialized = true;
        std::cout << "starting motion!" << std::endl;

        motion::MotionDeviceManager *deviceManager = motion::MotionDeviceManager::instance();
        deviceManager->AddMotionDeviceDescriptor(motion::motionDevices[0]);

        motion_device = deviceManager->connect(this,0);


        profile.imuBufferDepth = 1;
        profile.fisheyeBufferDepth = 20;
        profile.syncIMU = false;
        profile.gyro.enable = true;
        profile.gyro.fps = motion::MotionProfile::Gyro::FPS_200;
        profile.gyro.range = motion::MotionProfile::Gyro::RANGE_1000;

        profile.accel.enable = true;
        profile.accel.fps = motion::MotionProfile::Accel::FPS_125;
        profile.accel.range = motion::MotionProfile::Accel::RANGE_4;

        profile.fisheye.enable = true;
        profile.fisheye.fps = motion::MotionProfile::Fisheye::FPS_30;

        profile.depth.enable = true;

        profile.ext1.enable = false;
        profile.ext2.enable = false;

        profile.multiControllerSync.enable = false;

        profile.multiControllerSync.master = false; 
        motion_initialized = true;

        motion_device->start(profile);


    }
    void lr200_mm_camera::enable_fisheye_stream() {
        if(fisheye_started)
                return;
        motion_device->startFisheyeCamera();
        fisheye_started = true;
    }

     void lr200_mm_camera::start_motion_tracking()
    {
        initialize_motion();
    
        data_acquisition_active = true;

     


    }

    // Power down Motion Module
    void lr200_mm_camera::stop_motion_tracking()
    {
        std::cout << "stop motion: " << motion_device->stop() << std::endl;
        motion::MotionDeviceManager *deviceManager = motion::MotionDeviceManager::instance();
        deviceManager->disconnect(motion_device);

        rs_device_base::stop_motion_tracking();
        motion_initialized = false;
        fisheye_started = false;
        data_acquisition_active = false;


    }
    void lr200_mm_camera::start(rs_source source)
    {  
      
        ds_device::start(source);
    }

    // Power off Fisheye camera
    void lr200_mm_camera::stop(rs_source source)
    {
        ds_device::stop(source);
       
    }
     rs_stream lr200_mm_camera::select_key_stream(const std::vector<rsimpl::subdevice_mode_selection> & selected_modes)
    {
        // When all streams are enabled at an identical framerate, R200 images are delivered in the order: Z -> Third -> L/R
        // To maximize the chance of being able to deliver coherent framesets, we want to wait on the latest image coming from
        // a stream running at the fastest framerate.
        int fps[RS_STREAM_NATIVE_COUNT] = {}, max_fps = 0;
        for(const auto & m : selected_modes)
        {
            for(const auto & output : m.get_outputs())
            {
                fps[output.first] = m.mode.fps;
                max_fps = std::max(max_fps, m.mode.fps);
            }
        }
        
        // Select the "latest arriving" stream which is running at the fastest framerate
        for(auto s : {RS_STREAM_COLOR, RS_STREAM_INFRARED2, RS_STREAM_INFRARED, RS_STREAM_FISHEYE})
        {
            if(fps[s] == max_fps) return s;
        }
        return RS_STREAM_DEPTH;
    }
    rs_motion_intrinsics lr200_mm_camera::get_motion_intrinsics() const
    {
        return  fe_intrinsic.calib.imu_intrinsic;
    }
    rs_extrinsics lr200_mm_camera::get_motion_extrinsics_from(rs_stream from) const
    {
        switch (from)
        {
        case RS_STREAM_DEPTH:
            return fe_intrinsic.calib.mm_extrinsic.depth_to_imu;
        case RS_STREAM_COLOR:
            return fe_intrinsic.calib.mm_extrinsic.rgb_to_imu;
        case RS_STREAM_FISHEYE:
            return fe_intrinsic.calib.mm_extrinsic.fe_to_imu;
        default:
            throw std::runtime_error(to_string() << "No motion extrinsics from "<<from );
        }
    }
     void lr200_mm_camera::start_video_streaming(bool is_mipi) {
         std::cout << "starting video!" << std::endl;

     
       rs_device_base::start_video_streaming(true);
     
     }
     void lr200_mm_camera::stop_video_streaming()
     {
         if(!capturing) throw std::runtime_error("cannot stop device without first starting device");

         rs_device_base::stop_video_streaming();
     }




     motion_module_calibration read_fisheye_intrinsic(std::string calibrationFile)
     {

        motion_module_calibration intrinsic;

        std::ifstream cal("/intel/euclid/config/calibrationResult.json");
        std::string str((std::istreambuf_iterator<char>(cal)),std::istreambuf_iterator<char>());
        const char *json = str.c_str();

        rapidjson::Document d;
        d.Parse(json);
       
        double cx = d["Cx"].GetDouble();
        double cy = d["Cy"].GetDouble();
        double fx = d["Fx"].GetDouble();
        double fy = d["Fy"].GetDouble();

        double distortion = d["Kw"].GetDouble();

        intrinsic.calib.fe_intrinsic.kf[0] = fx;
        intrinsic.calib.fe_intrinsic.kf[1] = 0;
        intrinsic.calib.fe_intrinsic.kf[2] = cx;
        intrinsic.calib.fe_intrinsic.kf[3] = 0;
        intrinsic.calib.fe_intrinsic.kf[4] = fy;
        intrinsic.calib.fe_intrinsic.kf[5] = cy;
        intrinsic.calib.fe_intrinsic.kf[6] = 0;
        intrinsic.calib.fe_intrinsic.kf[7] = 0;
        intrinsic.calib.fe_intrinsic.kf[8] = 1;
        intrinsic.calib.fe_intrinsic.distf[0] = distortion;
        intrinsic.calib.fe_intrinsic.distf[1] = intrinsic.calib.fe_intrinsic.distf[2]
                = intrinsic.calib.fe_intrinsic.distf[3] =intrinsic.calib.fe_intrinsic.distf[4] = 0;


        {

            auto& scale = d["accelerometerTransform"];
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[0][0] = scale[0].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[1][0] = scale[1].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[2][0] = scale[2].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[0][1] = scale[3].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[1][1] = scale[4].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[2][1] = scale[5].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[0][2] = scale[6].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[1][2] = scale[7].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[2][2] = scale[8].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[0][3] = d["abias0"].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[1][3] = d["abias1"].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_intrinsic.val[2][3] = d["abias2"].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_bias_variance[0] = d["abiasvar0"].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_bias_variance[1] = d["abiasvar0"].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_bias_variance[2] = d["abiasvar0"].GetDouble();
            intrinsic.calib.imu_intrinsic.acc_noise_variance[0] =
                intrinsic.calib.imu_intrinsic.acc_noise_variance[1]=
                    intrinsic.calib.imu_intrinsic.acc_noise_variance[2] = d["aMeasVar"].GetDouble();


        }
        {
            auto& scale = d["gyroscopeTransform"];


            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[0][0] = scale[0].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[1][0] = scale[1].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[2][0] = scale[2].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[0][1] = scale[3].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[1][1] = scale[4].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[2][1] = scale[5].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[0][2] = scale[6].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[1][2] = scale[7].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[2][2] = scale[8].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[0][3] = d["wbias0"].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[1][3] = d["wbias1"].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_intrinsic.val[2][3] = d["wbias2"].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_bias_variance[0] = d["wbiasvar0"].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_bias_variance[1] = d["wbiasvar1"].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_bias_variance[2] = d["wbiasvar2"].GetDouble();
            intrinsic.calib.imu_intrinsic.gyro_noise_variance[0] =
                intrinsic.calib.imu_intrinsic.gyro_noise_variance[1]=
                    intrinsic.calib.imu_intrinsic.gyro_noise_variance[2] = d["wMeasVar"].GetDouble();

        }

        /* calculating extrinsics */
         pose imu_to_fe, imu_to_depth;
        {



            imu_to_fe = pose { {d["Rot0"].GetDouble(),d["Rot1"].GetDouble(),d["Rot2"].GetDouble(),
                                d["Rot3"].GetDouble(),d["Rot4"].GetDouble(),d["Rot5"].GetDouble(),
                                d["Rot6"].GetDouble(),d["Rot7"].GetDouble(),d["Rot8"].GetDouble()},
                               {d["Tc0"].GetDouble(),d["Tc1"].GetDouble(),d["Tc2"].GetDouble()} };


        }


        {
            auto& depth = d["depth"];



            imu_to_depth = pose  {{depth["Rot0"].GetDouble(),depth["Rot1"].GetDouble(),depth["Rot2"].GetDouble(),
                                   depth["Rot3"].GetDouble(),depth["Rot4"].GetDouble(),depth["Rot5"].GetDouble(),
                                   depth["Rot6"].GetDouble(),depth["Rot7"].GetDouble(),depth["Rot8"].GetDouble()},
                                  {depth["Tc0"].GetDouble(),depth["Tc1"].GetDouble(),depth["Tc2"].GetDouble()} };
        }


        pose fe_to_imu = inverse(imu_to_fe);
        pose fe_to_depth = inverse(imu_to_fe)*imu_to_depth;
        pose depth_to_imu = inverse(imu_to_depth);
        memcpy(intrinsic.calib.mm_extrinsic.fe_to_imu.rotation ,reinterpret_cast<float(&)[9]>(fe_to_imu.orientation),sizeof(float)*9);
        memcpy(intrinsic.calib.mm_extrinsic.fe_to_imu.translation ,reinterpret_cast<float(&)[3]>(fe_to_imu.position),sizeof(float)*3);

        memcpy(intrinsic.calib.mm_extrinsic.fe_to_depth.rotation , reinterpret_cast<float(&)[9]>(fe_to_depth.orientation),sizeof(float)*9);
        memcpy(intrinsic.calib.mm_extrinsic.fe_to_depth.translation , reinterpret_cast<float(&)[3]>(fe_to_depth.position),sizeof(float)*3);

        memcpy(intrinsic.calib.mm_extrinsic.depth_to_imu.rotation , reinterpret_cast<float(&)[9]>(depth_to_imu.orientation),sizeof(float)*9);
        memcpy(intrinsic.calib.mm_extrinsic.depth_to_imu.translation ,reinterpret_cast<float(&)[3]>(depth_to_imu.position),sizeof(float)*3);

        memcpy(intrinsic.calib.mm_extrinsic.rgb_to_imu.rotation,reinterpret_cast<float(&)[9]>(depth_to_imu.orientation),sizeof(float)*9);
        memcpy(intrinsic.calib.mm_extrinsic.rgb_to_imu.translation , reinterpret_cast<float(&)[3]>(depth_to_imu.position),sizeof(float)*3);


        return intrinsic;
     }



    std::shared_ptr<rs_device> make_lr200_mm_device(std::shared_ptr<uvc::device> device)
    {
        LOG_INFO("Connecting to Intel RealSense LR200 Motion");

        static_device_info info;
        info.name = { "Intel RealSense LR200M" };
        
        
        
        auto cam_info = ds::read_camera_info(*device);
        
        ds_device::set_common_ds_config(device, info, cam_info);
        motion_module_calibration fisheye_intrinsic = read_fisheye_intrinsic("/intel/euclid/config/calibrationResult.json");
        std::cout <<"Intrinsics: " << fisheye_intrinsic.calib.imu_intrinsic.acc_bias_variance[2] << std::endl;
        auto fe_extrinsic = fisheye_intrinsic.calib.mm_extrinsic;
        pose fisheye_to_depth = { reinterpret_cast<float3x3 &>(fe_extrinsic.fe_to_depth.rotation), reinterpret_cast<float3&>(fe_extrinsic.fe_to_depth.translation) };
        auto depth_to_fisheye = inverse(fisheye_to_depth);
        info.stream_poses[RS_STREAM_FISHEYE] = depth_to_fisheye;

        rs_intrinsics rs_intrinsics = fisheye_intrinsic.calib.fe_intrinsic;


        info.subdevice_modes.push_back({ 3, { 640, 480 }, pf_raw8, 30, rs_intrinsics, { /*TODO:ask if we need rect_modes*/ }, { 0 } });



        info.stream_subdevices[RS_STREAM_FISHEYE] = 3;
        info.presets[RS_STREAM_FISHEYE][RS_PRESET_BEST_QUALITY] = 
        info.presets[RS_STREAM_FISHEYE][RS_PRESET_LARGEST_IMAGE] =
        info.presets[RS_STREAM_FISHEYE][RS_PRESET_HIGHEST_FRAMERATE] = { true, 640, 480, RS_FORMAT_RAW8,   30 };
        
        info.options.push_back({ RS_OPTION_FISHEYE_GAIN                                             });
        info.options.push_back({ RS_OPTION_FISHEYE_STROBE,                          0,  1,   1,  0  });
        info.options.push_back({ RS_OPTION_FISHEYE_EXTERNAL_TRIGGER,                0,  1,   1,  0  });
        info.options.push_back({ RS_OPTION_FISHEYE_ENABLE_AUTO_EXPOSURE,            0,  1,   1,  1  });
        info.options.push_back({ RS_OPTION_FISHEYE_AUTO_EXPOSURE_MODE,              0,  2,   1,  0  }); 
        info.options.push_back({ RS_OPTION_FISHEYE_AUTO_EXPOSURE_ANTIFLICKER_RATE,  50, 60,  10, 60 });
        info.options.push_back({ RS_OPTION_FISHEYE_AUTO_EXPOSURE_PIXEL_SAMPLE_RATE, 1,  3,   1,  1  });
        info.options.push_back({ RS_OPTION_FISHEYE_AUTO_EXPOSURE_SKIP_FRAMES,       0,  3,   1,  2  });
        info.options.push_back({ RS_OPTION_HARDWARE_LOGGER_ENABLED,                 0,  1,   1,  0  });
        
        info.capabilities_vector.push_back({ RS_CAPABILITIES_FISH_EYE, { 1, 15, 5, 0 }, firmware_version::any(), RS_CAMERA_INFO_MOTION_MODULE_FIRMWARE_VERSION });
        info.capabilities_vector.push_back({ RS_CAPABILITIES_MOTION_EVENTS, { 1, 15, 5, 0 }, firmware_version::any(), RS_CAMERA_INFO_MOTION_MODULE_FIRMWARE_VERSION });
        info.camera_info[RS_CAMERA_INFO_MOTION_MODULE_FIRMWARE_VERSION] = "14.0.2";
        info.supported_metadata_vector.push_back(RS_FRAME_METADATA_ACTUAL_EXPOSURE);
        info.subdevice_modes.push_back({ 2,{ 1920, 1080 }, pf_rw16, 30,  fisheye_intrinsic.calib.fe_intrinsic,{ cam_info.calibration.modesThird[0][0] },{ 0 } });

        return std::make_shared<lr200_mm_camera>(device, info,fisheye_intrinsic);
    }
    
}
