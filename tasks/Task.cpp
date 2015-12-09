/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <GL/freeglut.h>
#include <GLFW/glfw3.h>
#include <libfreenect2/registration.h>

using namespace kinect2;
using namespace libfreenect2;

Task::Task(std::string const& name)
    : TaskBase(name)
    , undistorted(512, 424, 4)
    , registered(512, 424, 4)
{
    glfwInit();
    driver = new Freenect2();
    color_frame[0] = 0;
    color_frame[1] = 0;
    depth_frame = 0;
    ir_frame = 0;
    depth = 0;
    ir = 0;
    rgb = 0;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , undistorted(512, 424, 4)
    , registered(512, 424, 4)
{
    glfwInit();
    driver = new Freenect2();
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook()) return false;
    /*
    int argc=0;
    char **argv=0;
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_RGBA);
    glutCreateWindow("test");
    */

    int device_count = driver->enumerateDevices();
    if (device_count < 1) {
        LOG_FATAL_S << "Could not open Kinect";
        return false;
    }

    //    device = driver->openDefaultDevice();
    device = driver->openDevice(0);  // TODO handle more than 1 device
    if (!device) {
        LOG_FATAL_S << "Could not open Kinect";
        return false;
    } else {
        LOG_DEBUG_S << "Device opened successfull";
    }
    device->setIrAndDepthFrameListener(this);
    device->setColorFrameListener(this);
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook()) return false;

    device->start();
    registration = new libfreenect2::Registration(device->getIrCameraParams(), device->getColorCameraParams());
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    device->stop();
    delete registration;
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete device;
}

bool Task::writeColorFrame(libfreenect2::Frame* frame, bool orginal)
{
    size_t& width = frame->width;
    size_t& height = frame->height;
    size_t bpp = frame->bytes_per_pixel;
    unsigned char* data = frame->data;

    // The kinect guy desicded to use BGRX (with alpha channel and alpha always zero so we handle alpha as non-existend in the following)
    if (bpp == 4 || bpp == 3) {
        size_t bpp_internal = 3;

        if (!color_frame[orginal]) {
            color_frame[orginal] = new base::samples::frame::Frame(width, height, 8, base::samples::frame::MODE_RGB);
        }

        color_frame[orginal]->time = base::Time::now();
        char data_new[width * height * bpp_internal];
        for (size_t x = 0; x < width; x++) {
            for (size_t y = 0; y < height; y++) {
                data_new[0 + x * bpp_internal + y * bpp_internal * width] = data[2 + (width - x) * bpp + y * bpp * width];
                data_new[1 + x * bpp_internal + y * bpp_internal * width] = data[1 + (width - x) * bpp + y * bpp * width];
                data_new[2 + x * bpp_internal + y * bpp_internal * width] = data[0 + (width - x) * bpp + y * bpp * width];
            }
        }
        color_frame[orginal]->setImage(data_new, width * height * bpp_internal);
        color_frame_p[orginal].reset(color_frame[orginal]);
        if (orginal) {
            _color_frame_orginal.write(color_frame_p[orginal]);
        } else {
            _color_frame.write(color_frame_p[orginal]);
        }
    } else {
        LOG_ERROR_S << "Color Image: Unssuported bits per pixel size of " << bpp;
        return false;
    }
    return true;
}

bool Task::onNewFrame(Frame::Type type, Frame* frame)
{
    switch (type) {
        case Frame::Color:
            if (rgb) delete rgb;
            rgb_updated = true;
            rgb = frame;
            break;
        case Frame::Ir:
            if (ir) delete ir;
            ir = frame;
            break;
        case Frame::Depth:
            if (depth) delete depth;
            depth = frame;
            depth_updated = true;
            break;
    }

    // Only process the frame if we have a depth image
    // In this case we also write the color image (in a scaled-down version (not the orginal size))
    if (type == Frame::Depth) {
        if (depth && rgb && depth_updated && rgb_updated) {
            registration->apply(rgb, depth, &undistorted, &registered);
            size_t& width = undistorted.width;
            size_t& height = undistorted.height;
            size_t bpp = undistorted.bytes_per_pixel;
            unsigned char* data = undistorted.data;
            if (bpp == 4) {
                if (!depth_frame) {
                    depth_frame = new base::samples::DistanceImage(width, height);
                    depth_frame->data.resize(width * height);
                    Freenect2Device::IrCameraParams p = device->getIrCameraParams();
                    depth_frame->setIntrinsic(p.fx, p.fy, p.cx, p.cy);
                }
                depth_frame->time = base::Time::now();
                float* depth_data = (float*)data;
                for (size_t x = 0; x < width; x++) {
                    for (size_t y = 0; y < height; y++) {
                        float value = depth_data[(width-x)+y*width] / 1000.0f;  // to meters
                        if (isnan(value) || value <= 0.001) {
                            value = 0.0;
                        }
                        depth_frame->data[x+y*width] = value;
                    }
                }
                depth_frame_p.reset(depth_frame);
                _depth_frame.write(depth_frame_p);
            } else {
                LOG_ERROR_S << "Depth Image: Unssuported bits per pixel size of " << bpp;
                return false;
            }
            writeColorFrame(&registered, false);
            rgb_updated =false;
            depth_updated = false;
        }
    } else if (type == Frame::Color) {
        // Okay we write a colored frame imidiealty in his orginal size
        writeColorFrame(frame, true);
    } else if (type == Frame::Ir) {
        size_t& width = frame->width;
        size_t& height = frame->height;
        size_t bpp = frame->bytes_per_pixel;
        unsigned char* data = frame->data;
        if (bpp == 4) {
            if (!ir_frame) {
                ir_frame = new base::samples::frame::Frame(width, height, 32, base::samples::frame::MODE_GRAYSCALE);
            }
            ir_frame->setImage(data, width * height * bpp);
            ir_frame->time = base::Time::now();
            ir_frame_p.reset(ir_frame);
            _ir_frame.write(ir_frame_p);
        } else {
            LOG_ERROR_S << "IR Image: Unssuported bits per pixel size of " << bpp;
            return false;
        }
    } else {
        LOG_ERROR_S << "Unsupported Frame format";
        return false;
    }
    return true;
}
