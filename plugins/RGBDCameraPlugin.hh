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
#include <gazebo/common/Events.hh>
// OGRE
#include "gazebo/rendering/ogre_gazebo.h"

// TODO - Profiling
#include <chrono>

// Queue for threaded access
#include "ConcurrentQueue.hh"

namespace RGBDCameraPlugin {

    // Communication topics

    // SDF parameters

    /// SDF depth camera name parameter
    #define PARAM_CAMERA        "cameraName"
    /// SDF rendering queue maximum size parameter
    #define PARAM_QUEUE_SIZE    "renderQueueSize"
    /// SDF output directory parameter
    #define PARAM_OUTPUT_DIR    "outputDir"
    /// SDF output frame extension parameter
    #define PARAM_EXTENSION     "imageFormat"

    // Default values for SDF parameters

    /// Default output directory
    #define DEFAULT_OUTPUT_DIR  "/tmp/RGBDCameraPlugin"
    /// Default output extension
    #define DEFAULT_EXTENSION   "png"
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

        /// Pointer to depth camera renderer
        private: rendering::DepthCameraPtr camera;
        /// Pointer to RGB camera callback connection
        private: event::ConnectionPtr newRGBFrameConn;
        /// Pointer to depth camera callback connection
        private: event::ConnectionPtr newDepthFrameConn;

        /// Auxiliar thread for saving frames to disk
        private: std::thread thread;
        /// Flag to stop thread execution
        private: bool stop_thread {false};
        /// Multithread safe queue
        private: ConcurrentQueue<unsigned char*> *rgb_queue;

        /// Render output directory
        private: std::string output_dir;
        /// Rendered output format
        private: std::string output_ext;

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

        /// TODO
        private: void saveRender();

        /// \brief Get the OGRE image pixel format
        /// As seen in gazebo/rendering/Camera.cc
        /// \param[in] _format The Gazebo image format
        /// \return Integer representation of the Ogre image format
        private: static int OgrePixelFormat(const std::string &_format);
    };
}

#endif
