#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctype.h>
#include <CoreGraphics/CGEvent.h>
#include <CoreGraphics/CGEventTypes.h>
#include <ApplicationServices/ApplicationServices.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitKeys.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <thread>
#include <queue>


using namespace cv;
using namespace std;



static struct termios gOriginalTTYAttrs;

char gID = 'N';
int fd=0;

IplImage* imgTracking;
IplImage *imageR;
IplImage *paintImg;
int lastX = -1, lastY = -1;
int prevX = -1, prevY = -1;
bool isTouched = false;
bool wasTouched = false;


double gapBtwTaps = 0;

std::queue<int> touchStat;
float status = 0;
bool drawingMode = true;
double area = 0;

int lastLiftUpT = 0;
int lastLiftUpT2 = 0;
bool printDT = false;



void getTouchStatus(){
    
    float val=0;
    
    for(int i=0;i<touchStat.size();i++){
        
        val += touchStat.front();
        
        touchStat.push(touchStat.front());
        
        touchStat.pop();
        
    }
    
    status = val/touchStat.size();
    
}

bool touchStatus(){
    
    int val[]={0,0,0};
    int output = -1;
    if(touchStat.size()>=3){
        for(int c=0,i=0;i<touchStat.size();i++){
            if(i>=touchStat.size()-3){
                val[c++]= touchStat.front();
            }
            touchStat.push(touchStat.front());
            touchStat.pop();
        }
        if(val[0] == val[1] && val[1] == val[2])
            output = val[0];
        
        if(output == -1)
            return isTouched;
        else{
            if(output ==1)
                return true;
            else
                return false;
        }
    }else{
        return isTouched;
    }
    
}


void play_swipe()

{
    
    vector<cv::Point> trace;
    
    trace.reserve(1000);
    bool touched = false;
    
    while(1){
        
        //if(speakOut || printConsole){
        
        while(isTouched || status>0.5)    {
            
            trace.push_back(cv::Point(lastX,lastY));
            touched = true;
        }
        
        if(touched){
            touched = false;
            
            if(trace.size() > 5  && abs(trace.at(0).x - lastX) >0){
                
                //printf("len:%d area:%f\n",abs(trace.at(0).x - lastX),sqrt(area));
                if(trace.at(trace.size()/3).x - lastX > std::max(10,(int)sqrt(area)/4)){
                    
                    //  if(speakOut)
                    
                    //    hr = pVoice->Speak(L"Left", SPF_DEFAULT, NULL);
                    
                    //if(printConsole)
                    gID = 'L';
                    std::printf("Left Swipe<<<<<<<\n");
                    //http://stackoverflow.com/questions/3202629/where-can-i-find-a-list-of-mac-virtual-key-codes
                    CGKeyCode keyCode = 123; //123-left, 124-right
                    CGEventRef eventRefD = CGEventCreateKeyboardEvent (NULL, keyCode, true);
                    CGEventRef eventRefU = CGEventCreateKeyboardEvent (NULL, keyCode, false);
                    CGEventPost(kCGSessionEventTap, eventRefD);
                    CGEventPost(kCGSessionEventTap, eventRefU);
                    CFRelease(eventRefD);
                    CFRelease(eventRefU);
                    
                }else if(/*lastX - trace.at(trace.size()/2).x > 0 && trace.at(trace.size()/2).x - trace.at(0).x> 0 &&*/
                         lastX - trace.at(trace.size()/3).x > std::max(10,(int)sqrt(area)/4)){
                    gID = 'R';
                    //if(speakOut)
                    
                    //   hr = pVoice->Speak(L"Right", SPF_DEFAULT, NULL);
                    
                    // if(printConsole)
                    
                    std::printf("	 >>>>>>Right Swipe\n");
                    
                    CGKeyCode keyCode = 124; //123-left, 124-right
                    CGEventRef eventRefD = CGEventCreateKeyboardEvent (NULL, keyCode, true);
                    CGEventRef eventRefU = CGEventCreateKeyboardEvent (NULL, keyCode, false);
                    
                    CGEventPost(kCGSessionEventTap, eventRefD);
                    CGEventPost(kCGSessionEventTap, eventRefU);
                    CFRelease(eventRefD);
                    CFRelease(eventRefU);
                }
            }
            else if(gID == 'D'){
                ///gID = 'D';
                
                /*CGKeyCode keyCode = 49;
                 
                 
                 CGEventFlags ctrl = kCGEventFlagMaskControl;
                 CGEventFlags alt = kCGEventFlagMaskAlternate;
                 CGEventRef ev;
                 CGEventSourceRef source = CGEventSourceCreate (kCGEventSourceStateCombinedSessionState);
                 
                 //press down
                 ev = CGEventCreateKeyboardEvent (source, keyCode, true);
                 CGEventSetFlags(ev,ctrl);
                 CGEventSetFlags(ev,alt | CGEventGetFlags(ev)); //combine flags
                 CGEventPost(kCGHIDEventTap,ev);
                 CFRelease(ev);
                 
                 //press up
                 ev = CGEventCreateKeyboardEvent (source, keyCode, false);
                 CGEventSetFlags(ev,ctrl);
                 CGEventSetFlags(ev, alt | CGEventGetFlags(ev)); //combine flags
                 CGEventPost(kCGHIDEventTap,ev);
                 CFRelease(ev);
                 
                 CFRelease(source);*/
            }else{
                //  gID = 'S';
                //  std::printf("[SINGLE TAP]\n");
            }
            
            
        }
        trace.clear();
    }
    
    //sleep(100);
    // }
    
}

int open_port() {
    int fd; //Descriptor for the port
    fd = open("/dev/tty.usbmodem1411", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        cout << "Unable to open port. \n";
    }
    else {
        cout << "Port opened.\n";
    }
    cout << "Descriptor in open:";
    cout << fd;
    cout << "\n";
    return fd;
}

int configure_port (int fd) {
    struct termios options;
    
    tcgetattr(fd, &options);
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);
    
    cout << "Port configured.\n";
    
    return (fd);
}

void read_data(int fd) {
    //cout << "Reading data from: ";
    //cout << fd;
    //cout << "\n";
    
    //  write (fd, "hello!\n", 7);           // send 7 character greeting
    
    char buf[1];
    //memset (&buf, '\0', sizeof buf);
    /* *** READ *** */
    int n = read( fd, &buf ,1);
    //
    
    if(n==1)
        printf("%c\n",buf[0]);
    
    if(touchStat.size()==5)
        touchStat.pop();
    
    //if(n>=8 &&u8y6 n<=9){
    if(n==1 && buf[0]=='0' ){
        // printf("not touched\n");
        if(isTouched){
            
            /*if(lastLiftUpT > 0){
             double tmpT = (double)(clock() - lastLiftUpT);
             double tmpT2 = (double)(lastLiftUpT-lastLiftUpT2);
             //        printf("%f:",tmpT);
             // gapBtwTaps = tmpT;
             if(tmpT > 100000 && tmpT < 2000000){
             if(!printDT && gID == 'S'){
             gID = 'D';
             printDT = true;
             std::printf("[DOUBLE TAP]\n");
             }else{
             gID = 'S';
             std::printf("[SINGLE TAP1]\n");
             }
             
             
             }else if(tmpT > 5000000){
             gID = 'L';
             std::printf("[LONG TAP]\n");
             }else{
             gID = 'S';
             std::printf("[SINGLE TAP2]\n");
             
             }
             
             
             }
             lastLiftUpT2 = lastLiftUpT;
             lastLiftUpT = clock();
             */
            
        }
        //getTouchStatus();
        isTouched = false;
        touchStat.push(0);
        /* isTouched = false;
         touchStat.push(0);
         if(hasTouchedT1){
         lastTouchedT1 = clock();
         hasLiftedT1 = true;
         }else if(hasTouchedT2){
         hasTouchedT1 = false;
         hasLiftedT1 = false;
         hasTouchedT2 = false;
         hasLiftedT2 = true;
         }*/
        
    }else if(n==1 && buf[0]=='1' ){
        //printf("touched\n");
        //printf("--------------------\n");
        isTouched = true;
        touchStat.push(1);
        /*if(!hasTouchedT1 && !hasTouchedT2){
         firstTouchedT1 = clock();
         hasTouchedT1 = true;
         }else if(hasLiftedT1){
         firstTouchedT2 = clock();
         gapBtwTaps = (double)(firstTouchedT2 - lastTouchedT1); ///CLOCKS_PER_SEC;
         hasTouchedT2 = true;
         if(gapBtwTaps < 500000 && gapBtwTaps > 150000){
         printf("T1:%f, T2:%f, Gap:%f\n",(double)lastTouchedT1,(double)firstTouchedT2,gapBtwTaps);
         gID = 'D';
         }else{
         gID = 'T';
         }
         }*/
    }
    //getTouchStatus();
    // if(status>0.5)
    // isTouched = true;
    // else
    // isTouched = false;
    //}
    /*else{
     if(isTouched)
     touchStat.push(1);
     else
     touchStat.push(0);
     
     }*/
    /* if(printDT && gID == 'D'){
     write(fd,&gID,1);
     gID = 'N';
     //         printf("===================write D\n");
     printDT = false;
     }else if(gID == 'L'){
     write(fd,&gID,1);
     gID='N';
     
     }*/
    //isTouched = touchStatus();
    /*if(touchStatus()>0.7)
     isTouched = true;
     else if(touchStatus()<0.3)
     isTouched = false;*/
    
}

void closeSerialPort(int fileDescriptor)
{
    // Block until all written output has been sent from the device.
    // Note that this call is simply passed on to the serial device driver.
    // See tcsendbreak(3) <x-man-page://3/tcsendbreak> for details.
    if (tcdrain(fileDescriptor) == -1) {
        printf("Error waiting for drain - %s(%d).\n",
               strerror(errno), errno);
    }
    
    // Traditionally it is good practice to reset a serial port back to
    // the state in which you found it. This is why the original termios struct
    // was saved.
    if (tcsetattr(fileDescriptor, TCSANOW, &gOriginalTTYAttrs) == -1) {
        printf("Error resetting tty attributes - %s(%d).\n",
               strerror(errno), errno);
    }
    
    close(fileDescriptor);
}



void PostKeyWithModifiers(CGKeyCode key, CGEventFlags modifiers)
{
    CGEventSourceRef source = CGEventSourceCreate(kCGEventSourceStateCombinedSessionState);
    
    CGEventRef keyDown = CGEventCreateKeyboardEvent(source, key, TRUE);
    CGEventSetFlags(keyDown, modifiers);
    CGEventRef keyUp = CGEventCreateKeyboardEvent(source, key, FALSE);
    
    CGEventPost(kCGAnnotatedSessionEventTap, keyDown);
    CGEventPost(kCGAnnotatedSessionEventTap, keyUp);
    
    CFRelease(keyUp);
    CFRelease(keyDown);
    CFRelease(source);
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        
    }
}

/* bool insideEllipse (double p_find_x, double p_find_y)
 {
 double width = activeArea.size.width;
 double length = activeArea.size.height;
 double c_x = activeArea.center.x;
 double c_y = activeArea.center.y;
 if (p_find_x >= c_x && p_find_x <= (width / 2))
 {
 if (p_find_y >= c_y && p_find_y <= (length / 2))
 return true;
 }
 if (p_find_x <= c_x && p_find_x >= (width / 2))
 {
 if (p_find_y <= c_y && p_find_y >= (length / 2))
 return true;
 }
 return false;
 }*/


IplImage* trackObject(IplImage* imgThresh,IplImage* imgTracking, IplImage* paintImg){
    
    // Calculate the moments of 'imgThresh'
    
    if (imgThresh == NULL)
        
        return imgTracking;
    
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    
    cvMoments(imgThresh, moments, 1);
    
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    
    area = cvGetCentralMoment(moments, 0, 0);
    
    
    
    if(area>20 ){
        
        
        // calculate the position
        
        int posX = moment10/area ;
        
        int posY = moment01/area;
        
        
        
        if(lastX ==-1 && lastY ==-1){
            
            cvCircle(imgTracking, cvPoint(posX, posY), std::sqrt(area), cvScalar(0,0,255), 1, 8, 0 );
            
            lastX = posX;
            
            lastY = posY;
            
            
        }
        
        else if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
        {
            if(lastX >=50 && lastX <=imgTracking->width-50 && lastY >= 20 && lastY <=imgTracking->height-20){
                
                // Draw a yellow line from the previous point to the current point
                
                cvCircle(imgTracking, cvPoint(posX, posY), 5, cvScalar(0,255,255), -1, 8, 0 );
                
                if(touchStat.front()==1){
                    if(!wasTouched){
                        cvCircle(imgTracking, cvPoint(posX, posY), std::sqrt(area), cvScalar(0,255,0), 2, 8, 0 );
                        /* cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,0), 4);
                         cvCircle(imgTracking, cvPoint(posX, posY), std::sqrt(area), cvScalar(0,255,0), 2, 8, 0 );*/
                        if(drawingMode){
                            //cvLine(paintImg, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,0), 1);
                            cvCircle(paintImg, cvPoint(posX, posY), 1, cvScalar(255,255,0), 2, 8, 0 );
                            cvShowImage("Drawing",paintImg);
                            //  cvCreateImage(cvGetSize(imageR),8,3);
                            //  cvZero(imgTracking); //covert the image, 'imgTracking' to black
                        }
                        
                    }else{
                        cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,0), 4);
                        cvCircle(imgTracking, cvPoint(posX, posY), std::sqrt(area), cvScalar(0,255,0), 2, 8, 0 );
                        if(drawingMode){
                            cvLine(paintImg, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,0), 1);
                            cvCircle(paintImg, cvPoint(posX, posY), 1, cvScalar(255,255,0), 2, 8, 0 );
                            cvShowImage("Drawing",paintImg);
                        }
                    }
                }else{
                    cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
                    cvCircle(imgTracking, cvPoint(posX, posY), std::sqrt(area), cvScalar(0,0,255), 2, 8, 0 );
                    cvZero(paintImg);
                    cvShowImage("Drawing",paintImg);
                }
            }
        }
        
        prevX = lastX;
        
        prevY = lastY;
        
        lastX = posX;
        
        lastY = posY;
        
        
        
    }
    
    free(moments);
    return imgTracking;
    
}

/*    static void draw_box(IplImage *image, CvBox2D box, CvScalar color) {
 CvPoint2D32f boxPoints[4];
 
 cvBoxPoints(box, boxPoints);
 cvLine(image,
 cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y),
 cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y),
 color);
 cvLine(image,
 cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y),
 cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y),
 color);
 cvLine(image,
 cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y),
 cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y),
 color);
 cvLine(image,
 cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y),
 cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y),
 color);
 }*/


IplImage* imfill(IplImage* src)

{
    
    if(src == NULL)
        
        return src;
    
    CvScalar white = CV_RGB( 255, 255, 255 );
    
    IplImage* dst = cvCreateImage( cvSize(src->width,src->height), 8, 3);
    
    CvMemStorage* storage = cvCreateMemStorage(0);
    
    CvSeq* contour = 0;
    
    CvSeq* largest = 0;
    float area = 0;
    
    
    cvFindContours(src, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    
    cvZero( dst );
    
    for( ; contour != 0; contour = contour->h_next )
        
    {
        
        float areaT = fabs(cvContourArea(contour, CV_WHOLE_SEQ));
        
        if(areaT > area){
            
            largest = contour;
            
            area = areaT;
            
        }
        
        
    }
    
    if(largest!=0){
        
        cvDrawContours( dst, largest, white, white, 0, CV_FILLED);
        
    }
    
    //IplImage* bin_imgFilled = cvCreateImage(cvGetSize(src), 8, 1);
    cvInRangeS(dst, white, white, src);
    
    if(storage){
        
        cvClearMemStorage(storage);
        cvReleaseMemStorage(&storage);
        
    }
    
    cvReleaseImage(&dst);
    return src;
    
}



//This function threshold the HSV image and create a binary image

IplImage* GetThresholdedImage(IplImage* imgHSV, IplImage* imgThresh){
    
    cvInRangeS(imgHSV, cvScalar(100, 100, 100), cvScalar(110, 255, 255), imgThresh); //b g r
    imgThresh = imfill(imgThresh);
    
    return imgThresh;
    
}


void serialIO(){
    int fd = open_port();
    configure_port(fd);
    while(true){
        read_data(fd);
    }
    closeSerialPort(fd);
}

void processImg(VideoCapture cap){
    Mat image;
    IplImage ipl;
    IplImage *imageB;
    IplImage *paintImgT;
    IplImage *imgTrackingT;
    
    namedWindow( "Testing", CV_WINDOW_AUTOSIZE );
    moveWindow("Testing",800,20);
    
    int filterSize = 3;
    IplConvKernel *convKernel = cvCreateStructuringElementEx(filterSize, filterSize, (filterSize - 1) / 2, (filterSize - 1) / 2, CV_SHAPE_RECT, NULL);
    
    
    bool flag = false;
    
    while(1){
        cap >> image;
        ipl = image;
        IplImage *destination = cvCreateImage ( cvSize(ipl.width/4 , ipl.height/4 ),8,3);
        cvResize(&ipl, destination);
        imageB = destination;
        
        if(!flag){
            paintImgT =cvCreateImage(cvGetSize(imageB),8,3);
            cvZero(paintImgT); //covert the image, 'imgTracking' to black
            flag = true;
        }
        
        
        imgTrackingT =cvCreateImage(cvGetSize(imageB),8,3);
        cvZero(imgTrackingT); //covert the image, 'imgTracking' to black
        IplImage* imgHSV = cvCreateImage(cvGetSize(imageB), IPL_DEPTH_8U, 3);
        cvCvtColor(imageB, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
        //Get filtered image
        IplImage* imgThresh =cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
        imgThresh = GetThresholdedImage(imgHSV,imgThresh);
        //cvDilate(imgThresh,imgThresh,convKernel,3);
        
        IplImage* imgThreshRef =cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
        
        imgTrackingT = trackObject(imgThresh,imgTrackingT,paintImgT);
        
        cvAdd(imageB, imgTrackingT, imageB);
        
        
        
        cvShowImage("Testing", imageB);
        char ch = waitKey(10);
        
        if(ch=='d'){
            drawingMode = !drawingMode;
            cvShowImage("Drawing",paintImgT);
            
        }else if(ch=='c'){
            cvZero(paintImg); //covert the image, 'imgTracking' to black
            cvShowImage("Drawing",paintImgT);
        }
        
        cvReleaseImage(&destination);
        cvReleaseImage(&imgHSV);
        cvReleaseImage(&imgThresh);
        cvReleaseImage(&imgThreshRef);
        
    }
    image.release();
    cvReleaseImage(&imageB);
    cvReleaseImage(&paintImgT);
    cvReleaseImage(&imgTrackingT);
}



int main( int argc, const char** argv )
{
    VideoCapture cap;
    cap.open(0);
    
    if( !cap.isOpened() )
    {
        cout << "***Could not initialize capturing...***\n";
        return -1;
    }
    
    fd = open_port();
    configure_port(fd);
    
    Mat image;
    IplImage ipl;
    IplImage *imageB;
    IplImage *paintImgT;
    IplImage *imgTrackingT;
    bool flag = false;
    
    namedWindow( "Testing", CV_WINDOW_AUTOSIZE );
    moveWindow("Testing",800,20);
    
    int filterSize = 3;
    IplConvKernel *convKernel = cvCreateStructuringElementEx(filterSize, filterSize, (filterSize - 1) / 2, (filterSize - 1) / 2, CV_SHAPE_RECT, NULL);
    
    // int counter = 0;
    // int rate = 5;
    while(1)
    {
        wasTouched = isTouched;
        read_data(fd);
        //   if(counter%rate==0) {
        cap >> image;
        
        
        ipl = image;
        IplImage *destination = cvCreateImage ( cvSize(ipl.width/4 , ipl.height/4 ),8,3);
        cvResize(&ipl, destination);
        imageB = destination;
        
        if(!flag){
            paintImgT =cvCreateImage(cvGetSize(imageB),8,3);
            cvZero(paintImgT); //covert the image, 'imgTracking' to black
            flag = true;
        }
        
        
        
        imgTrackingT =cvCreateImage(cvGetSize(imageB),8,3);
        cvZero(imgTrackingT); //covert the image, 'imgTracking' to black
        IplImage* imgHSV = cvCreateImage(cvGetSize(imageB), IPL_DEPTH_8U, 3);
        cvCvtColor(imageB, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
        //Get filtered image
        IplImage* imgThresh =cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
        imgThresh = GetThresholdedImage(imgHSV,imgThresh);
        //cvDilate(imgThresh,imgThresh,convKernel,3);
        
        IplImage* imgThreshRef =cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
        imgTrackingT = trackObject(imgThresh,imgTrackingT,paintImgT);
        
        cvAdd(imageB, imgTrackingT, imageB);
        
        
        
        cvShowImage("Testing", imageB);
        char ch = waitKey(1);
        
        if(ch=='d'){
            drawingMode = !drawingMode;
            cvShowImage("Drawing",paintImgT);
            
        }else if(ch=='r'){
            cvZero(paintImg); //covert the image, 'imgTracking' to black
            cvShowImage("Drawing",paintImgT);
        }
        
        cvReleaseImage(&destination);
        cvReleaseImage(&imgHSV);
        cvReleaseImage(&imgThresh);
        cvReleaseImage(&imgThreshRef);
        
    }
    // counter = (counter+1)%rate;
    
    //}
    image.release();
    cvReleaseImage(&imageB);
    cvReleaseImage(&paintImgT);
    cvReleaseImage(&imgTrackingT);
    
    
    
    closeSerialPort(fd);
    return 0;
}
