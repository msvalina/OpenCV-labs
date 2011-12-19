#include <cv.h>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;

void help(){
    cout << "This is Crative Commons work, do what you like... " << endl;
    cout << "Usage: ./binary <image_name> " << endl;
    cout << "Hot keys: \n"
                "\tr/R toggle croping \n"
                "\tc/C toggle camera \n"
                "\ts/S toggle taking screenshoot\n"
                "\th/H toggle Hough Transform on screenshot\n"
                "\te/E toggle edge detetction \n" 
                "\tt/T toggle template matching\n";
    }

void canny_edge();
void init_camera();
void onMouse( int event, int x, int y, int flags, void* param );
void draw_box( Mat& img, Rect rect );
void crop_image( Mat& img, Rect rect );
void match_template_on_crop( int match_method, Mat& templ );
void getPoints( int event, int x, int y, int flags, void* param );
void savePoint( int x, int y );
void callHoughTransform( Mat& img, Rect rect );

Mat gray, temp, mat_image, gray_image, frame, ss;
Mat templ, result, imgRoi, imgRoi2;

Point pt1, pt2, pt3, pt4;
int n;
Rect box, box2;
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

        char c = waitKey(10);
        switch( c )
        {
            case 27:
                cout << "Exiting ... \n ";
                return 0;
            case 'e':
                canny_edge();
                break;
            case 'E':
                destroyWindow("canny edge");
                break;
            case 'r':
                cout << "Setting callback, calling crop_image  ...\n";
                setMouseCallback( imageName, onMouse, (void*)&mat_image );
                break;
            case 'R':
                destroyWindow( "ImgROI" );
                break;
            case 'c':
                init_camera();
                break;
            case 't':
                match_template_on_crop( 2, imgRoi );
                break;
            case 'T':
                destroyWindow( "source" );
                destroyWindow( "result" );
                break;
            case 'S':
                destroyWindow( "snapshot" );
                break;
            case 'h':
                callHoughTransform( ss, box2 );
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
            crop_image( image, box);
            draw_box( image, box );
            break;
    }
} 

void draw_box( Mat& img, Rect rect ){
    rectangle( img, rect.tl(), rect.br(), Scalar(0,0,255));
}

void crop_image( Mat& img, Rect rect ){
    imgRoi = img( rect );
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
    cout << "calling canny... \n";
    mat gray_image;
    cvtcolor( mat_image, gray_image, cv_rgb2gray );
    canny( gray_image, gray_image, 50, 100, 3 );
    namedwindow( "canny edge", cv_window_autosize );
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
        if( c == 'C' ){
            destroyWindow("camera");
            break;
        }
        switch ( c ) {
            case 's':
                frame.copyTo( ss );
                namedWindow( "snapshot", CV_WINDOW_AUTOSIZE );
                imshow( "snapshot", ss );
                cout << " Setting MouseCallback getPoints " << endl;
                setMouseCallback( "snapshot", getPoints, 0 );
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

void savePoint( int x, int y ){
    n++;
    if ( n == 1 ){
        pt1.x = x;
        pt1.y = y;
        cout << pt1.x << " " << pt1.y << endl;
    }
    if ( n == 2 ){ 
        pt2.x = x;
        pt2.y = y;
        cout << pt2.x << " " << pt2.y << endl;
    }
    if ( n == 3 ){ 
        pt3.x = x;
        pt3.y = y;
        cout << pt3.x << " " << pt3.y << endl;
    }
    if ( n == 4 ){ 
        pt4.x = x;
        pt4.y = y;
        box2.x = pt1.x;
        box2.y = pt1.y;
        box2.width = pt4.x - box2.x;
        box2.height = pt4.y - box2.y;
        cout << pt4.x << " " << pt4.y << endl;
        cout << box2.width << " " << box2.height << endl;
    }
}

void getPoints( int event, int x, int y, int flags, void* param ) {
    switch( event ) {
        case CV_EVENT_LBUTTONDOWN:
                break;
        case CV_EVENT_LBUTTONUP:
                savePoint( x, y );
                break;
         }
}

void callHoughTransform( Mat& img, Rect rect ){
    imgRoi2 = img( rect );
    cvtColor( imgRoi2, temp, CV_RGB2GRAY );
    Canny( temp, temp, 50, 100, 3 );
    namedWindow( "ImgRoi2", CV_WINDOW_AUTOSIZE );
    imshow( "ImgRoi2", temp );
    vector<Vec2f> lines;
    HoughLines(temp, lines, 1, CV_PI/180, 100, 0, 0 );
    /*
    temp: Output of the edge detector. 
    lines: A vector that will store the parameters (r,\theta) of the detected lines
    rho : The resolution of the parameter r in pixels. We use 1 pixel.
    theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    threshold: The minimum number of intersections to “detect” a line
    srn and stn: Default parameters to zero. Check OpenCV reference for more info.
    */ 
    // izvuci prvu linija, koja bi trebala vektor od dva elementa
    // ro'' i theta
    // prebacit ro u k.s. slike a ne ROI ro=ro'' +pt1.x*cos theta + 
    //                                          +pt1.y*sin theta
    // findExtrinsci, prdat xml, vraca R koji treba konvertirit u 3*3 pogledi rodrigez, 
    // pomonzit R i P(iz xml, camera matrix) jedanko A
    // pomnozit t i P dobijemo B
    // dobijemo lamde
    //  
    // 

}
