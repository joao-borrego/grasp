/*!
    \file plugins/RGBDCameraPlugin.hh
    \brief RGBD camera Gazebo plugin

    // TODO

    \author João Borrego : jsbruglie
*/

#ifndef _RGBD_CAMERA_PLUGIN_HH_
#define _RGBD_CAMERA_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>

namespace RGBDCameraPlugin {

    // Communication topics

    /// Topic for RGB image stream
    #define RGB_TOPIC   "~/grasp/camera/rgb"
    /// Topic for depth image stream
    #define DEPTH_TOPIC "~/grasp/camera/depth"
    /// RGB stream update rate in Hertz
    #define RGB_PUB_FREQ_HZ 60
    /// Depth stream update rate in Hertz
    #define DEPTH_PUB_FREQ_HZ 60

    // SDF Parameters

    /// TODO
    #define PARAM_RGB   "rgb"
    /// TODO
    #define PARAM_DEPTH "depth"
    
}

namespace gazebo {

    // Forward declaration of private data class
    class RGBDCameraPluginPrivate;

    // TODO
    class RGBDCameraPlugin : public ModelPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<RGBDCameraPluginPrivate> data_ptr; 

        /// Camera model
        private: physics::ModelPtr model;
        /// World
        private: physics::WorldPtr world;

        /// Pointer to rgb camera renderer
        private: rendering::CameraPtr rgb_camera;
        /// Pointer to depth camera renderer
        private: rendering::DepthCameraPtr depth_camera;
        /// Pointer to RGB camera callback connection
        private: event::ConnectionPtr newRGBFrameConn;
        /// Pointer to depth camera callback connection
        private: event::ConnectionPtr newDepthFrameConn;

        // Public methods

        /// \brief Constructs the object
        public: RGBDCameraPlugin();

        /// \brief Destroys the object
        public: virtual ~RGBDCameraPlugin();

        /// \brief Loads the plugin
        /// \param _model The model pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Callback function for handling RGB frame updates
        /// \param _image   Image data
        /// \param _width   Image width
        /// \param _height  Image height
        /// \param _depth   Image depth
        /// \param _format  Image format
        public: void onNewRGBFrame(
            const unsigned char *_image,
            unsigned int _width,
            unsigned int _height,
            unsigned int _depth,
            const std::string &_format);

        /// \brief Callback function for handling depth frame updates
        /// \param _image   Image data
        /// \param _width   Image width
        /// \param _height  Image height
        /// \param _depth   Image depth
        /// \param _format  Image format
        public: void onNewDepthFrame(
            const float *_image,
            unsigned int _width,
            unsigned int _height,
            unsigned int _depth,
            const std::string &_format);
    };
}

#endif
