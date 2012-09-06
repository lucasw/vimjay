#include "utility.h"

#include <glog/logging.h>

namespace bm {
/**
 * This is the function to demonstrate the conversion XImage to IplImage
 * @param _xImage the X image object
 * @param _xDisplay the X display, for init see main() below
 * @param _xScreen the X screen, for init see main() below
 * @return
 */
cv::Mat XImage2OpenCVImage(XImage& _xImage, Display& _xDisplay, Screen& _xScreen) 
{
    XColor color;
    //    cout << "WxH: " << imageData->width << "x" << imageData->height << ": " << imageData->data[0] << imageData->data[1] << imageData->data[2] << imageData->format << endl;
    //IplImage* ocvImage = cvCreateImage(cv::Size(_xImage.width, _xImage.height), IPL_DEPTH_8U, 3);
    cv::Mat tmp = cv::Mat(cv::Size(_xImage.width, _xImage.height), CV_8UC3);

    if (_xScreen.depths->depth == 24) {
        // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
        unsigned long rmask = _xScreen.root_visual->red_mask,
                gmask = _xScreen.root_visual->green_mask,
                bmask = _xScreen.root_visual->blue_mask;
        unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
        unsigned char colorChannel[3];

        rshift = 0;
        rbits = 0;
        while (!(rmask & 1)) {
            rshift++;
            rmask >>= 1;
        }
        while (rmask & 1) {
            rbits++;
            rmask >>= 1;
        }
        if (rbits > 8) {
            rshift += rbits - 8;
            rbits = 8;
        }

        gshift = 0;
        gbits = 0;
        while (!(gmask & 1)) {
            gshift++;
            gmask >>= 1;
        }
        while (gmask & 1) {
            gbits++;
            gmask >>= 1;
        }
        if (gbits > 8) {
            gshift += gbits - 8;
            gbits = 8;
        }

        bshift = 0;
        bbits = 0;
        while (!(bmask & 1)) {
            bshift++;
            bmask >>= 1;
        }
        while (bmask & 1) {
            bbits++;
            bmask >>= 1;
        }
        if (bbits > 8) {
            bshift += bbits - 8;
            bbits = 8;
        }

        for (unsigned int x = 0; x < _xImage.width; x++) {
            for (unsigned int y = 0; y < _xImage.height; y++) {
                color.pixel = XGetPixel(&_xImage, x, y);
                colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
                colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
                colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
                cv::Vec3b col = cv::Vec3b(colorChannel[0], colorChannel[1], colorChannel[0]);
                tmp.at<cv::Vec3b> (y,x) = col;
            }
        }
    } else { // Extremly slow alternative for non 24bit-depth
        Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
        for (unsigned int x = 0; x < _xImage.width; x++) {
            for (unsigned int y = 0; y < _xImage.height; y++) {
                color.pixel = XGetPixel(&_xImage, x, y);
                XQueryColor(&_xDisplay, colmap, &color);
                cv::Vec3b col = cv::Vec3b(color.blue, color.green, color.red);
                tmp.at<cv::Vec3b> (y,x) = col;
            }
        }
    }
    return tmp;
}


/**
 * This is the function to demonstrate the conversion cv::Mat to XImage
 * @param ximage the X image object
 * @param display the X display, for init see main() below
 * @param screen the X screen, for init see main() below
 * @return
 */
bool matToXImage(cv::Mat& im, XImage* ximage, Window& win, Display& display, Screen& screen) 
{
    XColor color;
    
    LOG_FIRST_N(INFO,1) << im.rows << " " << im.cols << ", " << ximage->width << " " << ximage->height;
    //cv::Mat tmp = cv::Mat(cv::Size(ximage.width, ximage.height), CV_8UC3);

    if (screen.depths->depth == 24) {
      LOG_FIRST_N(INFO,1) << "24-bit depth";
        // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
        unsigned long rmask = screen.root_visual->red_mask,
                gmask = screen.root_visual->green_mask,
                bmask = screen.root_visual->blue_mask;
        unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
        //unsigned char colorChannel[3];

        rshift = 0;
        rbits = 0;
        while (!(rmask & 1)) {
            rshift++;
            rmask >>= 1;
        }
        while (rmask & 1) {
            rbits++;
            rmask >>= 1;
        }
        if (rbits > 8) {
            rshift += rbits - 8;
            rbits = 8;
        }

        gshift = 0;
        gbits = 0;
        while (!(gmask & 1)) {
            gshift++;
            gmask >>= 1;
        }
        while (gmask & 1) {
            gbits++;
            gmask >>= 1;
        }
        if (gbits > 8) {
            gshift += gbits - 8;
            gbits = 8;
        }

        bshift = 0;
        bbits = 0;
        while (!(bmask & 1)) {
            bshift++;
            bmask >>= 1;
        }
        while (bmask & 1) {
            bbits++;
            bmask >>= 1;
        }
        if (bbits > 8) {
            bshift += bbits - 8;
            bbits = 8;
        }

        LOG_FIRST_N(INFO,1) 
            << "bshift " << bshift << " bbits " << bbits 
            << ", gshift " << gshift << " gbits " << gbits 
            << ", rshift " << rshift << " rbits " << rbits;
        for (unsigned int x = 0; x < ximage->width; x++) {
            for (unsigned int y = 0; y < ximage->height; y++) {
                color.pixel = XGetPixel(ximage, x, y);
                //colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
                //colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
                //colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
                //cv::Vec3b col = cv::Vec3b(colorChannel[0], colorChannel[1], colorChannel[0]);
                
                cv::Vec3b col = im.at<cv::Vec3b> (y,x);
                int b = col[0];
                int g = col[1];
                int r = col[2];
                 
                 color.pixel  = (r << rshift);
                 color.pixel |= (b << bshift);
                 color.pixel |= (g << gshift);

                //color.pixel = 0xf0f0f0;
                XPutPixel(ximage, x,y, color.pixel);
            }
        }
        LOG_FIRST_N(INFO,1) << "done copying mat";
    } else { // Extremly slow alternative for non 24bit-depth
        LOG_FIRST_N(INFO,1) <<" slow route TBD";
        Colormap colmap = DefaultColormap(&display, DefaultScreen(&display));
        for (unsigned int x = 0; x < ximage->width; x++) {
            for (unsigned int y = 0; y < ximage->height; y++) {
                color.pixel = XGetPixel(ximage, x, y);
                XQueryColor(&display, colmap, &color);
                cv::Vec3b col = cv::Vec3b(color.blue, color.green, color.red);
                im.at<cv::Vec3b> (y,x) = col;
            }
        }
    }
    return true;
}

}//bm
