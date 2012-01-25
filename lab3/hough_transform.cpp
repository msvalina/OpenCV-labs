#include <cv.h>
#include <highgui.h>
#include <iostream>

using namespace cv;
using namespace std;

void help(){
    cout << "This code is in Public Domain, do what you like... " << endl;
    cout << "Usage: ./binary <image_name> " << endl;
    cout << "Hot keys: \n"
                "\tr/R toggle croping \n"
                "\tc/C toggle camera \n"
                "\ts/S toggle taking screenshoot\n"
                "\th/H toggle Hough Transform on screenshot\n"
                "\te/E toggle edge detetction \n" 
                "\tt/T toggle template matching\n";
    }

void cannyEdge( Mat& img, Rect rect );
void cannyTreshold( int, void* );
void init_camera();
void onMouse( int event, int x, int y, int flags, void* param );
void draw_box( Mat& img, Rect rect );
void crop_image( Mat& img, Rect rect );
void match_template_on_crop( int match_method, Mat& templ );
void getPoints( int event, int x, int y, int flags, void* param );
void savePoint( int x, int y );
void callHoughTransform( );

Mat loaded_img, ss_img;
Mat canny_roi, canny_gray, canny_detected_edges, canny_out; 
Mat temp, frame;
Mat templ, result, imgRoi;

Point pt1, pt2, pt3, pt4;
vector<Point2f> imagePoints(4);
vector<Point2i> matrixsize(1);
int n;
Size2i size;
Rect box, box2, box3;
bool drawing_box = false;
char ** global_argv;
int match_method = 0;
int lowThreshold;
int ratio = 3;
int const max_lowThreshold = 150;

int main(int argc, char** argv) {
    help();
    global_argv = argv;
    
    if (argc < 2) {
        cout << " Usage: "<< argv[0] <<" <image> " << endl;
        return -1;
    }

    char* imageName = argv[1];
    loaded_img = imread( imageName, CV_LOAD_IMAGE_COLOR);
 
    if( !loaded_img.data ) {
        cout << " Could not open or find the image" << endl;
        return -1;
    }
    // dohvat width i height matrice
    size = loaded_img.size();
    //cout << size.width << " " << size.height << endl;

    // postavljanje ROI na cijelu sliku
    box3.x = 0;
    box3.y = 0;
    box3.width = size.width;
    box3.height = size.height;

    while ( 1 ){
        namedWindow( imageName, CV_WINDOW_AUTOSIZE );
        imshow( imageName, loaded_img);

        char c = waitKey(10);
        switch( c )
        {
            case 27:
                cout << "Exiting ... \n ";
                return 0;
            case 'e':
                //canny_edge();
                cannyEdge( loaded_img, box3 );
                break;
            case 'E':
                destroyWindow("canny edge");
                break;
            case 'r':
                cout << "Setting callback, calling crop_image  ...\n";
                setMouseCallback( imageName, onMouse, (void*)&loaded_img );
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
                cannyEdge( ss_img, box2 );
                break;
            case 'H':
                callHoughTransform( );
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

void cannyTreshold( int, void* ){
    Canny( canny_gray, canny_detected_edges, lowThreshold, lowThreshold*ratio, 3 );
    canny_out = Scalar::all(0);
    canny_detected_edges.copyTo( canny_out, canny_detected_edges);
    imshow( "canny", canny_out );
}

void cannyEdge( Mat& img, Rect rect ){
    canny_roi = img( rect );
    cvtColor( canny_roi, canny_gray, CV_RGB2GRAY);
    namedWindow( "canny", CV_WINDOW_AUTOSIZE);
    createTrackbar( "Min Treshold: ", "canny", &lowThreshold, max_lowThreshold, cannyTreshold );
    cannyTreshold( 0, 0 );
}

void draw_box( Mat& img, Rect rect ){
    rectangle( img, rect.tl(), rect.br(), Scalar(0,0,255));
}

void crop_image( Mat& img, Rect rect ){
    imgRoi = img( rect );
    namedWindow( "ImgROI", CV_WINDOW_AUTOSIZE );
    imshow( "ImgROI", imgRoi );
    /* 
    gornji kod kopira samo header u imgRoi
    ako treba kopirat i sliku moze se ovako:
    imgRoi.copyTo(temp);
    namedWindow( "temp", CV_WINDOW_AUTOSIZE );
    imshow( "temp", temp );
    */
}

void init_camera(){
    cout << "Starting camera mode... \n";
    VideoCapture cap(0);
    if( !cap.isOpened() ){
        cerr << "fail preko default camere \n";
        cout << "isprobavam argv[2] " 
            << global_argv[2] << endl;
        cap.open( global_argv[2] );
        if( !cap.isOpened() ){
            cerr << "fail i preko argv[2] \n";
        }
    }
    bool camera=true;
    while( camera ){
        cap >> frame;
        if(!frame.data) break;
        namedWindow( "camera", CV_WINDOW_AUTOSIZE );
        imshow( "camera", frame );
        char c = waitKey(10);
        switch( c ) {
            case 'C':
                camera = false;
                break;
            case 's':
                frame.copyTo( ss_img );
                namedWindow( "snapshot", CV_WINDOW_AUTOSIZE );
                imshow( "snapshot", ss_img );
                cout << " Setting MouseCallback getPoints " << endl;
                setMouseCallback( "snapshot", getPoints, 0 );
                break;
        }
    }
    destroyWindow("camera");
}

void match_template_on_crop( int match_method, Mat& templ ){
    /// Source image to display
    Mat img_display;
    loaded_img.copyTo( img_display );

    /// Create the result matrix
    int result_cols =  loaded_img.cols - templ.cols + 1;
    int result_rows = loaded_img.rows - templ.rows + 1;   

    result.create( result_cols, result_rows, CV_32FC1 );

    /// Do the Matching and Normalize
    matchTemplate( loaded_img, templ, result, match_method );
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
        imagePoints[0] = Point2f( x, y );
        cout << pt1.x << " " << pt1.y << endl;
    }
    if ( n == 2 ){ 
        pt2.x = x;
        pt2.y = y;
        imagePoints[1] = Point2f( x, y );
        cout << pt2.x << " " << pt2.y << endl;
    }
    if ( n == 3 ){ 
        pt3.x = x;
        pt3.y = y;
        imagePoints[2] = Point2f( x, y );
        cout << pt3.x << " " << pt3.y << endl;
    }
    if ( n == 4 ){ 
        pt4.x = x;
        pt4.y = y;
        imagePoints[3] = Point2f( x, y );
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

void callHoughTransform( ){
    vector<Vec2f> lines;
    HoughLines( canny_out, lines, 1, CV_PI/180, 100, 0, 0 );
    // temp nije output edge detectora, treba izmjeniti
    //cout << "Lines = " << Mat( lines ) << endl;
    float rho, rho_roi, theta;
    rho_roi= lines[0][0];
    theta = lines[0][1];
    // prebacivanje u k.s. slike
    rho = rho_roi + pt1.x * cos( theta ) + pt1.y * sin( theta );
    //cout << "rho_roi = " << rho_roi << endl;
    //cout << "theta = " << theta << endl;
    
    // ucitavanje parametara kamere
    FileStorage fs("calib/cam.xml", FileStorage::READ);
    Mat intrinsics(3, 3, CV_32F ); 
    Mat distortion( 5, 1, CV_32F );
    fs["camera_matrix"] >> intrinsics; //3*3
    fs["distortion_coefficients"] >> distortion; //4*1, kod mene 5*1
    //cout << "intrinsics = " << intrinsics <<  endl;
    //cout << "distortion = " << distortion <<  endl;

    vector<Point3f> objectPoints(4);
    objectPoints[0] = Point3f( 0, 0, 0 );
    objectPoints[1] = Point3f( 0, 250, 0 );
    objectPoints[2] = Point3f( 190, 0, 0 );
    objectPoints[3] = Point3f( 190, 250, 0 );
    //cout << "A vector of 3D Object Points = " << objectPoints << endl << endl;
    //cout << "A vector of 2D Image Points = " << imagePoints << endl << endl;

    Mat rvec( 1, 3, CV_32F );
    Mat tvec( 1, 3, CV_32F );
    Mat R( 3, 3, CV_32F );
    Mat A( 3, 3, CV_32F );
    Mat B( 3, 1, CV_32F );

    //cvFindExtrinsicCameraParams2() je zamjenjen s solvePnP()
    solvePnP( Mat(objectPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false );
    // solvePnP mijenja Mat type u CV_32F te se elementima moram pristupat s .at<dobule>
    //cout << "rvec = " << rvec <<  endl;
    cout << "tvec = " << tvec <<  endl;
    
    Rodrigues( rvec, R );
    //Mat t = tvec.t(); // transponirana matrica tvec 
    //cout << "t = " << t << endl;

    //cout << "R = " << R <<  endl;
    //cout << "A = " << endl << " " << A << endl << endl;

    A = intrinsics * R; // A = P * R
    B = intrinsics * tvec; // B = P * t
    
    //cout << "B = " << B <<  endl;
    //cout << "A = " << endl << " " << A << endl << endl;
    //cout << "A[1] = " << A.at<double>(0,0) << " A[1] = " << A.at<double>(0,1) << endl;

    double lambdaX, lambdaY, lambdaRo, rho_crtano, theta_crtano;
    // lambdaX = a11*cos(theta) + a21*sin(theta) - ro*a31
    lambdaX = A.at<double>(0,0) * cos(theta) + A.at<double>(1,0) * sin(theta) - rho * (A.at<double>(2,0));
    // lambdaY = a12*cos(theta) + a22*sin(theta) - ro*a32
    lambdaY = A.at<double>(0,1) * cos(theta) + A.at<double>(1,1) * sin(theta) - rho * (A.at<double>(2,1));
    // lamdbaRo = b3*ro - b1*cos(theta) - b2*sin(theta) 
    lambdaRo = rho * (B.at<double>(2)) - B.at<double>(0) * cos(theta) - B.at<double>(1) * sin(theta); 

    theta_crtano = atan2( lambdaY, lambdaX );
    rho_crtano = lambdaRo / sqrt( lambdaX * lambdaX + lambdaY * lambdaY );

    cout << "Theta = " << theta_crtano*180/CV_PI << endl;
    cout << "Rho = " << rho_crtano << endl;

}
