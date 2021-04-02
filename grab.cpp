#include <chrono>
//#include <iomanip>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
//#include <opencv2/cudaimgproc.hpp>

#include "webcam.h"

#define XRES 640
#define YRES 480
#define FPS 30

using namespace std;

#define CLIP(color) (unsigned char)(((color) > 0xFF) ? 0xff : (((color) < 0) ? 0 : (color)))

void v4lconvert_yuyv_to_rgb24(const unsigned char *src, 
                                     unsigned char *dest,
                                     int width, int height, 
                                     int stride)
{
    int j;

    while (--height >= 0) {
        for (j = 0; j + 1 < width; j += 2) {
            int u = src[1];
            int v = src[3];
            int u1 = (((u - 128) << 7) +  (u - 128)) >> 6;
            int rg = (((u - 128) << 1) +  (u - 128) +
                    ((v - 128) << 2) + ((v - 128) << 1)) >> 3;
            int v1 = (((v - 128) << 1) +  (v - 128)) >> 1;

            *dest++ = CLIP(src[0] + u1);
            *dest++ = CLIP(src[0] - rg);
            *dest++ = CLIP(src[0] + v1);

            *dest++ = CLIP(src[2] + u1);
            *dest++ = CLIP(src[2] - rg);
            *dest++ = CLIP(src[2] + v1);

            //*dest++ = CLIP(src[0] + v1);
            //*dest++ = CLIP(src[0] - rg);
            //*dest++ = CLIP(src[0] + u1);

            //*dest++ = CLIP(src[2] + v1);
            //*dest++ = CLIP(src[2] - rg);
            //*dest++ = CLIP(src[2] + u1);
            src += 4;
        }
        src += stride - (width * 2);
    }
}

int main(int argc, char** argv)
{

    //std::cout << cv::getBuildInformation() << std::endl;

    Webcam webcam("/dev/video2", XRES, YRES);
    
    cv::namedWindow("img", 1);

    /* user the webcam.cpp implementation for color conversion */
    auto frame = webcam.frame();

    //auto *pt = frame.data;
    //cv::Mat img(YRES, XRES, CV_8UC3,  pt);
    cv::Mat img(YRES, XRES, CV_8UC3,  frame.data);
    
    std::cout << "using webcam.cpp implementation for color conversion\n";
    while (int k = cv::waitKey(1) != 'q') {
      auto start = chrono::high_resolution_clock::now();
      frame = webcam.frame();

      auto stop = chrono::high_resolution_clock::now();
      auto duration = chrono::duration_cast<chrono::microseconds>(stop-start);
      cout << "dt = " << std::fixed << std::setprecision(3) << duration.count()/1000.0f << endl;
      cv::imshow("img", img);
    }
    
    /* use image buffer and OpenCV color conversion */
    //
    // => opencv is much more efficient! (~0.5 x cpu compared to webcam.cpp)
    //
    auto buffer = webcam.get_buffer();
    //auto *buffer_ptr = buffer.data;
    char* buf = new char[buffer.size]; 

    cv::Mat out;
    //cv::Mat yuyv(YRES, XRES, CV_8UC2, buffer.data);
    cv::Mat yuyv(YRES, XRES, CV_8UC2, buf);
    
    //cv::cvtColor(yuyv, out, cv::COLOR_YUV2BGR_YUYV);

    std::cout << "using OpenCV implementation for color conversion\n";

    while (int k = cv::waitKey(1) != 'q') {
      auto start = chrono::high_resolution_clock::now();
      //frame = webcam.frame();
      buffer = webcam.get_buffer();
      memcpy(buf, buffer.data, buffer.size);

      cv::cvtColor(yuyv, out, cv::COLOR_YUV2BGR_YUYV);

      auto stop = chrono::high_resolution_clock::now();
      auto duration = chrono::duration_cast<chrono::microseconds>(stop-start);
      cout << "dt = " << std::fixed << std::setprecision(3) << duration.count()/1000.0f << endl;
      cv::imshow("img", out);
    }
    
    return 0;

    /* working code
    cv::cuda::GpuMat src, dst;
    src.upload(out);
    //cv::cuda::cvtColor(src, dst, cv::COLOR_YUV2BGR);
    cv::cuda::resize(src, dst, cv::Size(XRES*2, YRES*2));
    dst.download(out);
    */
    unsigned char rgb[640*480*3];
    while (true) { 
      buffer = webcam.get_buffer();
      auto buffer_ptr = buffer.data;
      auto start = chrono::high_resolution_clock::now();
      // conversion takes 9 ms
      v4lconvert_yuyv_to_rgb24((unsigned char *) buffer.data,
                             rgb,
                             640,
                             480,
                             3);
      
      auto stop = chrono::high_resolution_clock::now();
      auto duration = chrono::duration_cast<chrono::microseconds>(stop-start);
      cout << "dt = " << std::fixed << std::setprecision(3) << duration.count()/1000.0f << endl;
      auto pt = frame.data;
      //cv::cvtColor(img, out, cv::COLOR_RGB2BGR);
      //out = img;
      cv::Mat tmp;
      yuyv.copyTo(tmp);
    }
    while (true) { 
      auto start = chrono::high_resolution_clock::now();
      frame = webcam.frame();
      auto stop = chrono::high_resolution_clock::now();
      auto duration = chrono::duration_cast<chrono::milliseconds>(stop-start);
      cout << "dt = " << duration.count() << endl;
      auto pt = frame.data;
      //cv::cvtColor(img, out, cv::COLOR_RGB2BGR);
      //out = img;
      cv::Mat tmp;
      img.copyTo(tmp);
    }

    size_t cnt;
    while (int key = cv::waitKey(1) != 'q') {

      frame = webcam.frame();
      auto pt = frame.data;
      //cv::cvtColor(img, out, cv::COLOR_RGB2BGR);
      //out = img;
      cv::Mat tmp1, tmp2, tmp3;
      img.copyTo(tmp1);

      auto start = chrono::high_resolution_clock::now();
      frame = webcam.frame();
      auto stop = chrono::high_resolution_clock::now();
      auto duration = chrono::duration_cast<chrono::milliseconds>(stop-start);
      cout << "dt = " << duration.count() << endl;

      pt = frame.data;
      img.copyTo(tmp2);

      tmp3 = tmp2 - tmp1;
      if (++cnt%10 == 0)   
        cv::imshow("img", tmp3);
      continue;

      buffer = webcam.get_buffer();
      auto buffer_ptr = buffer.data;
      
      if (++cnt%3 == 0) {  
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
