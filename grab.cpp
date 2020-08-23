#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>

#include "webcam.h"

#define XRES 640
#define YRES 480
//#define FPS 60

using namespace std;

int main(int argc, char** argv)
{

    //std::cout << cv::getBuildInformation() << std::endl;

    Webcam webcam("/dev/video0", XRES, YRES);
    auto frame = webcam.frame();

    auto *pt = frame.data;
    //cv::Mat img(YRES, XRES, CV_8UC3,  frame.data);
    cv::Mat img(YRES, XRES, CV_8UC3,  pt);
    
    
    auto buffer = webcam.get_buffer();
    auto *buffer_ptr = buffer.data;

    cv::Mat yuyv(YRES, XRES, CV_8UC2, buffer_ptr);
    
    cv::namedWindow("img", 1);

    
    cv::Mat out;
    
    cv::cvtColor(yuyv, out, cv::COLOR_YUV2BGR_YUYV);
    /*
    cv::imshow("img", out);
    cv::waitKey(0);
    return 0;
    */

    /* working code
    cv::cuda::GpuMat src, dst;
    src.upload(out);
    //cv::cuda::cvtColor(src, dst, cv::COLOR_YUV2BGR);
    cv::cuda::resize(src, dst, cv::Size(XRES*2, YRES*2));
    dst.download(out);
    */
    

    size_t cnt;
    while (int key = cv::waitKey(1) != 'q') {

      //frame = webcam.frame();
      //pt = frame.data;
      //cv::imshow("img", img);

      buffer = webcam.get_buffer();
      buffer_ptr = buffer.data;
      
      if (++cnt%1 == 0) {  
        cv::cvtColor(yuyv, out, cv::COLOR_YUV2BGR_YUYV);
        
        //src.upload(out);
        //cv::cuda::resize(src, dst, cv::Size(XRES/2, YRES/2));
        //dst.download(out);
       
        //if (++cnt%120 == 0)
          cv::imshow("img", out);
      }
      //ofstream image;
      //image.open("frame.ppm");
      // image << "P6\n" << XRES << " " << YRES << " 255\n";
      //image.write((char *) frame.data, frame.size);
      //image.close();
    }
    //cv::waitKey(0);

    return 0;

}
