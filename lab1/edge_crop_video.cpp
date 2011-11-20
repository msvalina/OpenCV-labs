// konstruktori i fje poicinju malim slovima
// klase i strukture velikim
// Mat I;
// IplImage pI = I;
// Mat I;
// IplImage* pI = &I.operator IplImage();
// IplImage* ipl_img = new IplImage( mat_image );
#include <cv.h>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;

void onMouse( int event, int x, int y, int flags, void* param );
void draw_box( Mat& img, Rect rect );
void crop_image( Mat& img, Rect rect );

Rect box;
bool drawing_box = false;

int main(int argc, char** argv) {

    cout << "LoadImage is CC work, do what you like " << endl;
    
    if (argc < 2) {
        cout << " Usage: "<< argv[0] <<" <image> " << endl;
        return -1;
    }

    char* imageName = argv[1];
    Mat mat_image;
    mat_image = imread( imageName, CV_LOAD_IMAGE_COLOR);
 
    if( !mat_image.data ) {
        cout << " Could not open or find the image" << endl;
        return -1;
    }

    Mat gray_image;
    Mat temp;
    cvtColor( mat_image, gray_image, CV_RGB2GRAY );
    
    Canny(gray_image, gray_image, 50, 200, 3);

    namedWindow( imageName, CV_WINDOW_AUTOSIZE );
    namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
    
    setMouseCallback( imageName, onMouse, (void*)&mat_image );

    while ( 1 ){
        mat_image.copyTo(temp);
        if( drawing_box ){
            draw_box( temp, box );
        }
        imshow( imageName, temp );
        imshow( "Gray image", gray_image );

        char c;
        c = waitKey( 15 );
        if( c == 27 ) break;
    }

    VideoCapture cap;
    if (argc==1) {
    	cap.open(0); // open the default camera
    } else {
    	cap.open(argv[1]);
    }
    if(!cap.isOpened())  // check if we succeeded
    {
    	cerr << "Couldn't open capture." << endl;
        return -1;
    }

    Mat frame;
    while(1) {
    	cap >> frame;
   		if(!frame.data) break;
        imshow( "Example2_9", frame );
        char c = waitKey(10);
        if( c == 27 ) break;
    }

    return 0;
}

void crop_image( Mat& img, Rect rect ){
    Mat imgRoi = img(rect);
    namedWindow( "ImgROI", CV_WINDOW_AUTOSIZE );
    imshow( "ImgROI", imgRoi );
}



void draw_box( Mat& img, Rect rect ){
    rectangle( img, rect.tl(), rect.br(), Scalar(0,0,255));
}

void onMouse( int event, int x, int y, int flags, void* param ) {
    Mat& image = *(Mat*) param;
    switch( event )
    {
        case CV_EVENT_LBUTTONDOWN:{
            drawing_box = true;
            box = Rect(x, y, 0, 0);
            }
            break;
        case CV_EVENT_MOUSEMOVE: {
            if( drawing_box ) {
                box.width = x-box.x;
                box.height = y-box.y;
            }
            }
            break;
        case CV_EVENT_LBUTTONUP: {
            drawing_box = false;
            if( box.width<0 ) {
                box.x+=box.width;
                box.width *= -1;
            }
            if( box.height<0 ) {
                box.y+=box.height;
                box.height*=-1;
            }
            cout << "box coordinates \n" 
                << "x\t y\t height\t width\n"
                << box.x << "\t" << box.y << "\t" 
                << box.height << "\t" << box.width << "\n";
            draw_box( image, box );
            crop_image( image, box);
            }
            break;
    }
} 
