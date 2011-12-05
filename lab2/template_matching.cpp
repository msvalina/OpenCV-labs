#include <cv.h>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;

void help(){
    cout << "This is Crative Commons work, do what you like... " << endl;
    cout << "Usage: ./binary <image_name> " << endl;
    cout << "Hot keys: \n"
                "\tr for croping \n"
                "\tR to kill crop win\n"
                "\tv for video \n"
                "\tV to quit video \n"
                "\tc for canny edge detetction \n" 
                "\tC to kill canny win\n"
                "\tt for template matching\n";
                "\tT for to kill template matching windows\n";
    }

void onMouse( int event, int x, int y, int flags, void* param );
void draw_box( Mat& img, Rect rect );
void crop_image( Mat& img, Rect rect );
void canny_edge();
void init_camera();
void match_template_on_crop( int match_method, Mat& templ );

Mat gray, temp, mat_image, gray_image, frame;
Mat templ, result, imgRoi;

Rect box;
bool drawing_box = false;
char ** global_argv;
int match_method = 0;

int main(int argc, char** argv) {
    help();
    global_argv = argv;
    
    if (argc < 2) {
        cout << " Usage: "<< argv[0] <<" <image> " << endl;
        return -1;
    }

    char* imageName = argv[1];
    mat_image = imread( imageName, CV_LOAD_IMAGE_COLOR);
 
    if( !mat_image.data ) {
        cout << " Could not open or find the image" << endl;
        return -1;
    }

    while ( 1 ){
        namedWindow( imageName, CV_WINDOW_AUTOSIZE );
        imshow( imageName, mat_image );

        int c = waitKey(15);
        switch( (char)c )
        {
            case 27:
                cout << "Exiting ... \n ";
                return 0;

            case 'c':
                canny_edge();
                break;

            case 'C':
                destroyWindow("canny edge");
                break;

            case 'r':
                cout << "Setting callback, calling crop_image  ...\n";
                setMouseCallback( imageName, onMouse, (void*)&mat_image );
                break;

            case 'R':
                destroyWindow( "ImgROI" );
                break;

            case 'v':
                init_camera();
                break;

            case 't':
                match_template_on_crop( 2, imgRoi );
                break;

            case 'T':
                destroyWindow( "source" );
                destroyWindow( "result" );
                break;
        }
    }

    return 0;
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

void draw_box( Mat& img, Rect rect ){
    rectangle( img, rect.tl(), rect.br(), Scalar(0,0,255));
}

void crop_image( Mat& img, Rect rect ){
    imgRoi = img(rect);
    namedWindow( "ImgROI", CV_WINDOW_AUTOSIZE );
    imshow( "ImgROI", imgRoi );
    /* gornji kod kopira samo header u imgRoi
     * ako treba kopirat i sliku moze se ovako:
    imgRoi.copyTo(temp);
    namedWindow( "temp", CV_WINDOW_AUTOSIZE );
    imshow( "temp", temp );
    */
}

void canny_edge(){
    cout << "Calling canny... \n";
    Mat gray_image;
    cvtColor( mat_image, gray_image, CV_RGB2GRAY );
    Canny( gray_image, gray_image, 50, 100, 3 );
    namedWindow( "canny edge", CV_WINDOW_AUTOSIZE );
    imshow( "canny edge", gray_image );
}

void init_camera(){
    cout << "Starting camera mode... \n";
    VideoCapture cap(0);
    if( !cap.isOpened() ){
        cerr << "fail preko default camere \n";
        cout << "isprobavam argv2 " 
            << global_argv[2] << endl;
        cap.open( global_argv[2] );
        if( !cap.isOpened() ){
            cerr << "fail i preko argv[2] \n";
        }

    }
    while( 1 ){
        cap >> frame;
        if(!frame.data) break;
        namedWindow( "camera", CV_WINDOW_AUTOSIZE );
        imshow( "camera", frame );
        char c = waitKey(10);
        if( c == 'V' ){
            destroyWindow("camera");
            break;
        }
    }
}

void match_template_on_crop( int match_method, Mat& templ ){
    /// Source image to display
    Mat img_display;
    mat_image.copyTo( img_display );

    /// Create the result matrix
    int result_cols =  mat_image.cols - templ.cols + 1;
    int result_rows = mat_image.rows - templ.rows + 1;   

    result.create( result_cols, result_rows, CV_32FC1 );

    /// Do the Matching and Normalize
    matchTemplate( mat_image, templ, result, match_method );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );


    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else  
    { matchLoc = maxLoc; }

    /// Show me what you got
    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 ); 
    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 ); 

    namedWindow( "source", CV_WINDOW_AUTOSIZE );
    namedWindow( "result", CV_WINDOW_AUTOSIZE );
    imshow( "source", img_display );
    imshow( "result", result );
}

