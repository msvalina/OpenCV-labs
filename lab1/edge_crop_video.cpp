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
bool croping_roi = false;
bool running_video = false;

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
    
    while ( 1 ){
        mat_image.copyTo(temp);
        namedWindow( imageName, CV_WINDOW_AUTOSIZE );
        imshow( imageName, temp );

        int c = waitKey(15);
        switch( (char)c )
        {
            case 27:
                cout << "Exiting ... \n ";
                return 0;
            case 'c':
                cout << "Calling canny... \n";
                Canny(gray_image, gray_image, 50, 200, 3);
                
                namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
                imshow( "Gray image", gray_image );
                break;
            case 'r':
                croping_roi = true;
                running_video = false;
                if( croping_roi ) {
                    cout << "Setting callback, Image ROI i crop mode ...\n";
                    setMouseCallback( imageName, onMouse, (void*)&mat_image );
                if( drawing_box ) {
                    draw_box( temp, box );
                }
                }
                break;
            case 'v':
                cout << "Camera mode... \n"
                    << "Destroying croping mode \n";
                if ( croping_roi ) {
                    destroyWindow( imageName );
                    croping_roi = false;
                }
                running_video = true;
                if( running_video ){
                    VideoCapture cap(0);
                    if( !cap.isOpened() ){
                        cerr << "fail preko default camere \n";
                        cout << "isprobavam argv2 " 
                            << argv[2] << endl;
                        cap.open( argv[2] );
                        if( !cap.isOpened() ){
                            cerr << "fail i preko argv[2] \n";
                            return -1;
                        }

                    }

                    Mat frame;
                    while( 1 ){
                        cap >> frame;
                        if(!frame.data) break;
                        namedWindow( "camera", CV_WINDOW_AUTOSIZE );
                        imshow( "camera", frame );
                        char c = waitKey(10);
                        if( c == 'q' ) break;
                    }

                }
                break;
        }
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
        case CV_EVENT_LBUTTONDOWN:
            drawing_box = true;
            box = Rect(x, y, 0, 0);
            break;
        case CV_EVENT_MOUSEMOVE: 
            if( drawing_box ) {
                box.width = x-box.x;
                box.height = y-box.y;
            }
            break;
        case CV_EVENT_LBUTTONUP: 
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
            //draw_box( image, box );
            crop_image( image, box);
            break;
    }
} 
