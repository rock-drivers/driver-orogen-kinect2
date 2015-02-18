/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <libfreenect2/opengl.h>
#include <GL/freeglut.h>
//#include <libfreenect2/opengl.h>

using namespace kinect2;
using namespace libfreenect2;

Task::Task(std::string const& name)
    : TaskBase(name)
{
 glfwInit();
 driver = new Freenect2();
 color_frame = 0;
 depth_frame = 0;
 ir_frame = 0;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
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
    if (! TaskBase::configureHook())
        return false;
    /*
    int argc=0;
    char **argv=0;
    glutInit(&argc,argv);   
    glutInitDisplayMode(GLUT_RGBA);
    glutCreateWindow("test");
    */

    int device_count = driver->enumerateDevices();
    if(device_count < 1){
        LOG_FATAL_S << "Could not open Kinect"; 
        return false; 
    }

//    device = driver->openDefaultDevice();
    device = driver->openDevice(0); //TODO handle more than 1 device
    if(!device){
        LOG_FATAL_S << "Could not open Kinect"; 
        return false;
    }else{
        LOG_DEBUG_S << "Device opened successfull";
    }
    device->setIrAndDepthFrameListener(this);
    device->setColorFrameListener(this);
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    device->start();
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
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete device;
}
        
bool Task::onNewFrame(Frame::Type type, Frame *frame){
    size_t &width = frame->width;
    size_t &height = frame->height;
    size_t bpp = frame->bytes_per_pixel;
    unsigned char* data = frame->data;

    if(type == Frame::Color){
        if(bpp == 3){
            if(!color_frame){
                color_frame = new base::samples::frame::Frame(width,height,8,base::samples::frame::MODE_RGB);
            }

            color_frame->time = base::Time::now();
            char data_new[width*height*bpp];
            for(size_t x=0;x<width;x++){
                for(size_t y=0;y<height;y++){
                    data_new[0 + x*bpp + y*bpp*width] =  data[2 + (width-x)*bpp + y*bpp*width];
                    data_new[1 + x*bpp + y*bpp*width] =  data[1 + (width-x)*bpp + y*bpp*width];
                    data_new[2 + x*bpp + y*bpp*width] =  data[0 + (width-x)*bpp + y*bpp*width];
                }
            }
            color_frame->setImage(data_new,width*height*bpp);
            color_frame_p.reset(color_frame);
            _color_frame.write(color_frame_p);
        }else{
            LOG_ERROR_S << "Color Image: Unssuported bits per pixel size of " << bpp;
            delete frame;
            return false;
        }
    }else if(type == Frame::Ir){
        if(bpp == 4){
            if(!ir_frame){
                ir_frame = new base::samples::frame::Frame(width,height,32,base::samples::frame::MODE_GRAYSCALE);
            }
            char data_new[width*height*bpp];
            for(size_t x=0;x<width;x++){
                for(size_t y=0;y<height;y++){
                    data_new[0 + x*bpp + y*bpp*width] =  data[0 + (width-x)*bpp + y*bpp*width];
                    data_new[1 + x*bpp + y*bpp*width] =  data[1 + (width-x)*bpp + y*bpp*width];
                    data_new[2 + x*bpp + y*bpp*width] =  data[2 + (width-x)*bpp + y*bpp*width];
                    data_new[3 + x*bpp + y*bpp*width] =  data[3 + (width-x)*bpp + y*bpp*width];
                }
            }
            ir_frame->setImage(data,width*height*bpp);
            ir_frame->time = base::Time::now();
            ir_frame_p.reset(ir_frame);
            _ir_frame.write(ir_frame_p);
        }else{
            LOG_ERROR_S << "IR Image: Unssuported bits per pixel size of " << bpp;
            delete frame;
            return false;
        }
    }else if(type == Frame::Depth){
        if(bpp == 4){
            if(!depth_frame){
                depth_frame = new base::samples::DistanceImage(width,height); 
                depth_frame->data.resize(width*height);
                Freenect2Device::IrCameraParams p = device->getIrCameraParams();
                depth_frame->setIntrinsic(p.fx,p.fy,p.cx,p.cy);
            }
            depth_frame->time = base::Time::now();
            float *depth_data = (float*)data;

            for(size_t x=0;x<width;x++){
                for(size_t y=0;y<height;y++){
                    depth_frame->data[x+y*width] = depth_data[(width-x)+y*width]/1000.0; //to meters
                }
            }

            depth_frame_p.reset(depth_frame);
            _depth_frame.write(depth_frame_p);
        }else{
            LOG_ERROR_S << "Depth Image: Unssuported bits per pixel size of " << bpp;
            delete frame;
            return false;
        }
    }else{
        LOG_ERROR_S << "Unsupported Frame format";
        delete frame;
        return false;
    }
    delete frame;
    return true;
}
