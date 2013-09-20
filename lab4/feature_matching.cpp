#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
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
void initCamera( );
void onMouse( int event, int x, int y, int flags, void* param );
void drawBox( Mat& img, Rect rect );
void cropImage( Mat& img, Rect rect );
void matchTemplateTrackbar( );
void matchTemplateOnCrop( int, void* );
void getPoints( int event, int x, int y, int flags, void* param );
void savePoint( int x, int y );
void callHoughTransform( );
void surfFlannMatcher( );

Mat loaded_img, ss_img, cam_frame;
Mat canny_roi, canny_gray, canny_detected_edges, canny_out; 
Mat croped_roi;
Mat templ_img, result_img; 

Point pt1, pt2, pt3, pt4;
vector<Point2f> imagePoints(4);
int n;
Size2i size;
Rect crop_box, canny_box, ss_box;
bool drawing_box = false;
char ** global_argv;
int lowThreshold;
int max_lowThreshold = 200;
int ratio = 3;
int max_Trackbar = 5;
int match_method = 0;

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
    canny_box.x = 0;
    canny_box.y = 0;
    canny_box.width = size.width;
    canny_box.height = size.height;

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
                cannyEdge( loaded_img, canny_box );
                break;
            case 'E':
                destroyWindow("canny");
                break;
            case 'r':
                cout << "Setting callback, calling cropImage  ...\n";
                setMouseCallback( imageName, onMouse, (void*)&loaded_img );
                break;
            case 'R':
                destroyWindow( "croped" );
                break;
            case 'c':
                initCamera( );
                break;
            case 't':
                if(!croped_roi.data){
                    cout << "nisi cropao nista" << endl;
                    break;
                }
                matchTemplateTrackbar( );
                break;
            case 'T':
                destroyWindow( "source" );
                destroyWindow( "result" );
                break;
            case 'S':
                destroyWindow( "snapshot" );
                break;
            case 'h':
                cannyEdge( ss_img, ss_box );
                break;
            case 'H':
                if(!canny_out.data){
                    cout << "pozivi canny" << endl;
                    break;
                }
                callHoughTransform( );
                break;
            case 'l':
                surfFlannMatcher( );
                break;
        }
    }


    return 0;
}

void cannyTreshold( int, void* ){
    Canny( canny_gray, canny_detected_edges, lowThreshold, lowThreshold*ratio, 3 );
    canny_out = Scalar::all(0);
    canny_detected_edges.copyTo( canny_out );
    imshow( "canny", canny_out );
}

void cannyEdge( Mat& img, Rect rect ){
    canny_roi = img( rect );
    cvtColor( canny_roi, canny_gray, CV_RGB2GRAY);
    namedWindow( "canny", CV_WINDOW_AUTOSIZE);
    createTrackbar( "Min Treshold: ", "canny", &lowThreshold, max_lowThreshold, cannyTreshold );
    cannyTreshold( 0, 0 );
}

void onMouse( int event, int x, int y, int flags, void* param ) {
    Mat& image = *(Mat*) param;
    switch( event )
    {
        case CV_EVENT_LBUTTONDOWN:
            drawing_box = true;
            crop_box = Rect(x, y, 0, 0);
            break;
        case CV_EVENT_MOUSEMOVE: 
            if( drawing_box ) {
                crop_box.width = x-crop_box.x;
                crop_box.height = y-crop_box.y;
            }
            break;
        case CV_EVENT_LBUTTONUP: 
            drawing_box = false;
            if( crop_box.width<0 ) {
                crop_box.x+=crop_box.width;
                crop_box.width *= -1;
            }
            if( crop_box.height<0 ) {
                crop_box.y+=crop_box.height;
                crop_box.height*=-1;
            }
            cout << "box coordinates \n" 
                << "x\t y\t height\t width\n"
                << crop_box.x << "\t" << crop_box.y << "\t" 
                << crop_box.height << "\t" << crop_box.width << "\n";
            cropImage( image, crop_box);
            //drawBox( image, box );
            break;
    }
} 

void drawBox( Mat& img, Rect rect ){
    rectangle( img, rect.tl(), rect.br(), Scalar(0,0,255));
}

void cropImage( Mat& img, Rect rect ){
    croped_roi = img( rect );
    namedWindow( "croped", CV_WINDOW_AUTOSIZE );
    imshow( "croped", croped_roi );
}

void initCamera( ){
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
        cap >> cam_frame;
        if(!cam_frame.data) break;
        namedWindow( "camera", CV_WINDOW_AUTOSIZE );
        imshow( "camera", cam_frame );
        char c = waitKey(10);
        switch( c ) {
            case 'o':
                camera = false;
                break;
            case 's':
                cam_frame.copyTo( ss_img );
                namedWindow( "snapshot", CV_WINDOW_AUTOSIZE );
                imshow( "snapshot", ss_img );
                cout << " Setting MouseCallback getPoints " << endl;
                setMouseCallback( "snapshot", getPoints, 0 );
                break;
        }
    }
    destroyWindow("camera");
}
void matchTemplateTrackbar( ){
    namedWindow( "source", CV_WINDOW_AUTOSIZE );
    namedWindow( "result", CV_WINDOW_AUTOSIZE );

    char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
    createTrackbar( trackbar_label, "source", &match_method, max_Trackbar, matchTemplateOnCrop );

    matchTemplateOnCrop( 0, 0 );
}

void matchTemplateOnCrop( int, void* ){
    Mat source_img;
    loaded_img.copyTo( source_img );
    croped_roi.copyTo( templ_img );

    /// Create the result matrix
    int result_cols =  loaded_img.cols - templ_img.cols + 1;
    int result_rows = loaded_img.rows - templ_img.rows + 1;   

    result_img.create( result_cols, result_rows, CV_32FC1 );

    /// Do the Matching and Normalize
    matchTemplate( loaded_img, templ_img, result_img, match_method );
    normalize( result_img, result_img, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result_img, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else  
    { matchLoc = maxLoc; }

    rectangle( source_img, matchLoc, Point( matchLoc.x + templ_img.cols , matchLoc.y + templ_img.rows ), Scalar::all(0), 2, 8, 0 ); 
    rectangle( result_img, matchLoc, Point( matchLoc.x + templ_img.cols , matchLoc.y + templ_img.rows ), Scalar::all(1), 2, 8, 0 ); 

    imshow( "source", source_img );
    imshow( "result", result_img );
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
        imagePoints[1] = Point2f( x, y );
        cout << x << " " << y << endl;
    }
    if ( n == 3 ){ 
        imagePoints[2] = Point2f( x, y );
        cout << x << " " << y << endl;
    }
    if ( n == 4 ){ 
        pt4.x = x;
        pt4.y = y;
        imagePoints[3] = Point2f( x, y );
        ss_box.x = pt1.x;
        ss_box.y = pt1.y;
        ss_box.width = pt4.x - ss_box.x;
        ss_box.height = pt4.y - ss_box.y;
        cout << pt4.x << " " << pt4.y << endl;
        cout << ss_box.width << " " << ss_box.height << endl;
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
    cout << "Lines = " << Mat( lines ) << endl;
    float rho, rho_roi, theta;
    rho_roi= lines[0][0];
    theta = lines[0][1];
    // prebacivanje u k.s. slike
    rho = rho_roi + pt1.x * cos( theta ) + pt1.y * sin( theta );
    cout << "rho_roi = " << rho_roi << endl;
    cout << "theta = " << theta << endl;

    // ucitavanje parametara kamere
    FileStorage fs("calib/cam.xml", FileStorage::READ);
    Mat intrinsics(3, 3, CV_32F ); 
    Mat distortion( 5, 1, CV_32F );
    fs["camera_matrix"] >> intrinsics; //3*3
    fs["distortion_coefficients"] >> distortion; //4*1, kod mene 5*1
    cout << "intrinsics = " << intrinsics <<  endl;
    cout << "distortion = " << distortion <<  endl;

    vector<Point3f> objectPoints(4);
    objectPoints[0] = Point3f( 0, 0, 0 );
    objectPoints[1] = Point3f( 0, 250, 0 );
    objectPoints[2] = Point3f( 190, 0, 0 );
    objectPoints[3] = Point3f( 190, 250, 0 );
    cout << "A vector of 3D Object Points = " << objectPoints << endl << endl;
    cout << "A vector of 2D Image Points = " << imagePoints << endl << endl;

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

void surfFlannMatcher( ){
    Mat img_1 = imread( global_argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_2 = imread( global_argv[2], CV_LOAD_IMAGE_GRAYSCALE );
    if( !img_1.data || !img_2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector( minHessian );

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_1, descriptors_2;

    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance < 2*min_dist )
        { good_matches.push_back( matches[i]); }
    }  

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2, 
            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), 
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS ); 

    //-- Show detected matches
    imshow( "Good Matches", img_matches );

    for( int i = 0; i < good_matches.size(); i++ )
    { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

}
