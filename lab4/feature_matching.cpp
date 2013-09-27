#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void help ()
{
cout << 
"Usage: ./binary <image_name> \n"
"Hot keys: \n"
"\trun/kill \n"
"\te/E -- edge detetction \n" 
"\tr/R -- croping \n"
"\tc/C -- camera \n"
"\tt/T -- template matching\n"
"\ti/I -- take screenshoot\n"
"\th/H -- Hough Transform on screenshot\n"
"\ts/S -- SURF\n"
"\tq/ESC -- Exit \n \n";
}

void cannyEdge (Mat& img, Rect rect);
void cannyTreshold (int, void*);
void initCamera ();
void onMouse (int event, int x, int y, int flags, void* param);
void drawBox (Mat& img, Rect rect);
void showSelRoi (Mat& img, Rect rect);
void matchTemplateTrackbar ();
void matchTemplateOnCrop (int, void*);
void getPoints (int event, int x, int y, int flags, void* param);
void savePoint (int x, int y);
void callHoughTransform ();
void surfFlannMatcher ();

Mat loadedImg, ssImg, camFrame;
Mat cannyRoi, cannyGray, cannyDetectedEdges, cannyOut; 
Mat cropedRoi;
Mat templImg, resultImg; 

string imageName;
Point pt1, pt2, pt3, pt4;
vector<Point2f> imagePoints(4);
Size2i size;
Rect cropBox, cannyBox, ssBox;
bool drawingBox = false;
bool pointSelected = false;
bool imageTaken = false;
char ** globalArgv;
int lowThreshold;
int maxLowThreshold = 200;
int ratio = 3;
int maxTrackbar = 5;
int matchMethod= 5;
int n;

int main (int argc, char** argv)
{
    help ();
    globalArgv = argv;
    
    if (argc < 2) {
        cout << "Using default images/lena_color_256.tif image" << endl;
        imageName = "../images/lena_color_256.tif";
    }
    else 
    imageName = argv[1];

    loadedImg = imread( imageName, CV_LOAD_IMAGE_COLOR);
 
    if( !loadedImg.data ) {
        cout << " Could not open or find the image" << endl;
        return -1;
    }

    // set ROI on whole image
    cannyBox.x = 0;
    cannyBox.y = 0;
    size = loadedImg.size();
    cannyBox.width = size.width;
    cannyBox.height = size.height;

    while (1){
        namedWindow( imageName, CV_WINDOW_AUTOSIZE );
        imshow( imageName, loadedImg);

        char c = waitKey(10);
        switch (c) {
            case 27:
                cout << "Exiting \n "; return 0;
            case 'q':
                cout << "Exiting \n "; return 0;
            case 'e':
                cannyEdge (loadedImg, cannyBox);
                break;
            case 'E':
                destroyWindow ("canny");
                break;
            case 'r':
                cout << "Setting callback onMouse, calling showSelRoi()\n";
                setMouseCallback (imageName, onMouse, (void*)&loadedImg);
                break;
            case 'R':
                destroyWindow ("selectedRoi");
                break;
            case 'c':
                initCamera ();
                break;
            case 't':
                if (!cropedRoi.data){
                    cout << "First crop template with \"r\" " << endl;
                    break; 
                }
                matchTemplateTrackbar ();
                break;
            case 'T':
                destroyWindow ("source");
                destroyWindow ("result");
                destroyWindow ("selectedRoi");
                break;
            case 'I':
                destroyWindow ("snapshot");
                break;
            case 'h':
                if (!imageTaken) {
                    if (pointSelected) {
                        cannyEdge (loadedImg, ssBox);
                        cout << "Points?!1\n";
                    }
                    setMouseCallback (imageName, getPoints, 0);
                    cout << "No data from camera. Using argv[1] or \ default image\n";
                }
                else { 
                cannyEdge (ssImg, ssBox);
                }
                break;
            case 'H':
                callHoughTransform ();
                destroyWindow ("snapshot");
                break;
            case 's':
                surfFlannMatcher ();
                break;
        }
    }
    return 0;
}

void cannyTreshold (int, void*)
{
    Canny (cannyGray, cannyDetectedEdges, lowThreshold,
            lowThreshold*ratio, 3);
    cannyOut = Scalar::all(0);
    cannyDetectedEdges.copyTo (cannyOut);
    imshow ("canny", cannyOut);
}

void cannyEdge (Mat& img, Rect rect)
{
    cannyRoi = img (rect);
    cvtColor (cannyRoi, cannyGray, CV_RGB2GRAY);
    namedWindow ("canny", CV_WINDOW_AUTOSIZE);
    createTrackbar ("Min Treshold: ", "canny", &lowThreshold,
            maxLowThreshold, cannyTreshold);
    cannyTreshold (0, 0);
}

void onMouse (int event, int x, int y, int flags, void* param) 
{
    Mat& imageLink = *(Mat*) param;
    Mat image = imageLink.clone();
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            drawingBox = true;
            cropBox = Rect (x, y, 0, 0);
            break;
        case CV_EVENT_MOUSEMOVE: 
            if (drawingBox) {
                cropBox.width = x-cropBox.x;
                cropBox.height = y-cropBox.y;
            }
            break;
        case CV_EVENT_LBUTTONUP: 
            drawingBox = false;
            if (cropBox.width<0) {
                cropBox.x+=cropBox.width;
                cropBox.width *= -1;
            }
            if (cropBox.height<0) {
                cropBox.y+=cropBox.height;
                cropBox.height*=-1;
            }
            // cout << "box coordinates \n" 
            //     << "x\t y\t height\t width\n"
            //     << cropBox.x << "\t" << cropBox.y << "\t" 
            //     << cropBox.height << "\t" << cropBox.width << "\n";
            showSelRoi (image, cropBox);
            break;
    }
    if (drawingBox) {
    drawBox (image, cropBox);
    imshow (imageName, image);
    }
} 

void drawBox (Mat& img, Rect rect)
{
    rectangle( img, rect.tl(), rect.br(), Scalar(0,0,255));
    imshow (imageName, loadedImg);
}

void showSelRoi (Mat& img, Rect rect)
{
    cropedRoi = img (rect);
    namedWindow ("selectedRoi", CV_WINDOW_AUTOSIZE);
    imshow ("selectedRoi", cropedRoi);
}

void initCamera ()
{
    cout << "Starting camera \n";
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Default camera won't open\n";
        cout << "Trying argv[2] " << globalArgv[2] << endl;
        cap.open (globalArgv[2]);
        if (!cap.isOpened()) {
            cerr << "Default camere isn't working\n";
            cerr << "Try adding path to camera as argv[2]\n";
        }
    }
    bool camera = true;
    while (camera) {
        cap >> camFrame;
        if (!camFrame.data) break;
        namedWindow ("camera", CV_WINDOW_AUTOSIZE);
        imshow ("camera", camFrame);
        char c = waitKey(10);
        switch (c) {
            case 'C':
                camera = false;
                break;
            case 'i':
                camFrame.copyTo (ssImg);
                namedWindow ("snapshot", CV_WINDOW_AUTOSIZE);
                imshow ("snapshot", ssImg);
                imwrite ("snapshot.jpeg", ssImg);
                imageTaken = true;
                cout << " Setting MouseCallback on getPoints " << endl;
                setMouseCallback ("snapshot", getPoints, 0);
                break;
        }
    }
    destroyWindow ("camera");
}

void matchTemplateTrackbar ()
{
    namedWindow ("source", CV_WINDOW_AUTOSIZE);
    namedWindow ("result", CV_WINDOW_AUTOSIZE);

    string trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \
        \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \
        \n 5: TM COEFF NORMED";

    createTrackbar( trackbar_label, "source" , &matchMethod,
            maxTrackbar, matchTemplateOnCrop );

    matchTemplateOnCrop( 0, 0 );
}

void matchTemplateOnCrop (int, void*)
{
    Mat sourceImg;
    loadedImg.copyTo (sourceImg);
    cropedRoi.copyTo (templImg);

    Mat gsourceImg, gtemplImg;
    cvtColor (sourceImg, gsourceImg, CV_BGR2GRAY);
    cvtColor (templImg, gtemplImg, CV_BGR2GRAY);
    /// Create the result matrix
    int resultCols = sourceImg.cols - templImg.cols + 1;
    int resultRows = sourceImg.rows - templImg.rows + 1;   
    resultImg.create (resultRows, resultCols, CV_32FC1);

    /// Do the Matching and Normalize
    matchTemplate (gsourceImg, gtemplImg, resultImg, matchMethod);
    normalize (resultImg, resultImg, 0, 1., NORM_MINMAX, -1, Mat());
    // Remove non matching results with tresholding
    threshold (resultImg, resultImg, 0.8, 1., THRESH_BINARY);

    // Localizing the best match with minMaxLoc
    // Used only for testing purpose
    double minVal; double maxVal; double threshold=0.8;
    Point minLoc; Point maxLoc; Point matchLoc;
    minMaxLoc (resultImg, &minVal, &maxVal, &minLoc, &maxLoc);
    rectangle (sourceImg, maxLoc, Point (maxLoc.x + templImg.cols,
                maxLoc.y + templImg.rows), Scalar (0,0,255)); 

    // Find all best matches and paint them
    // Go through every pixel from top left corner to right, top to down
    for (int y = 1; y < resultImg.rows -1; y++) {
        for (int x = 1; x < resultImg.cols -1; x++) {
            // search postion (y,x) but draw at (x,y) 
            if (resultImg.at<float>(y,x) > 0) {
                // cout << y << "," << x << " = " <<
                // resultImg.at<float>(y,x) << " , ";
                rectangle (sourceImg, Point (x,y), Point
                        (x + templImg.cols, y + templImg.rows),
                        Scalar (0,255,0));  
            }
        }
    }
    // I think this is safest method for "scaning" images
    // MatIterator_<uchar> it, end;
    // for( it = I.begin<uchar>(), end = I.end<uchar>(); it != end; ++it)
    //*it = table[*it];
    // break;
    imshow ("source", sourceImg);
    imshow ("result", resultImg);
}

void savePoint (int x, int y)
{
    n++;
    if (n == 1) {
        pt1.x = x;
        pt1.y = y;
        imagePoints[0] = Point2f (x, y);
        cout << pt1.x << " " << pt1.y << endl;
    }
    if (n == 2) { 
        imagePoints[1] = Point2f (x, y);
        cout << x << " " << y << endl;
    }
    if (n == 3) { 
        imagePoints[2] = Point2f (x, y);
        cout << x << " " << y << endl;
    }
    if (n == 4) { 
        pt4.x = x;
        pt4.y = y;
        imagePoints[3] = Point2f (x, y);
        ssBox.x = pt1.x;
        ssBox.y = pt1.y;
        ssBox.width = pt4.x - ssBox.x;
        ssBox.height = pt4.y - ssBox.y;
        pointSelected = true;
        cout << pt4.x << " " << pt4.y << endl;
        cout << ssBox.width << " " << ssBox.height << endl;
    }
    else NULL;
}

void getPoints (int event, int x, int y, int flags, void* param) 
{
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            break;
        case CV_EVENT_LBUTTONUP:
            savePoint (x, y);
            break;
    }
}

void callHoughTransform ()
{
    /*
     * Find lines in edge point image using Hough Transform
     * HT is operating in polar coordinte system so our lines are
     * represented with: 
     * rho - vector from origin of c.s to line (perpendicular to line) 
     * theta - angle form positive x axis to rho (range -90-90)
     */
    vector<Vec2f> lines;
    HoughLines (cannyOut, lines, 1, CV_PI/180, 100, 0, 0);
    /* cout << "Lines = " << Mat( lines ) << endl; */
    float rho, rhoRoi, theta;
    rhoRoi = lines[0][0];
    theta = lines[0][1];

    // rho 
    rho = rhoRoi + pt1.x * cos(theta) + pt1.y * sin(theta);
    // cout << "rhoRoi = " << rhoRoi << endl;
    // cout << "theta = " << theta << endl;

    // Read calibration parameters
    FileStorage fs ("../lab3/calib/cam-c270.xml", FileStorage::READ);
    // Intrinsics/projection matrix with fx,fy,u,v intrinsics camera
    // parameters for unit conversion
    Mat intrinsics (3, 3, CV_32F); 
    fs ["camera_matrix"] >> intrinsics; 
    // Distortion matrix with k1,k2,p1,p2,k3 distortion coefficients for
    // correcting cameras radial and tangential distortion
    Mat distortion (5, 1, CV_32F);
    fs ["distortion_coefficients"] >> distortion; 
    // cout << "intrinsics = " << intrinsics <<  endl;
    // cout << "distortion = " << distortion <<  endl;

    vector<Point3f> objectPoints (4);
    objectPoints[0] = Point3f (0, 0, 0);
    objectPoints[1] = Point3f (0, 250, 0);
    objectPoints[2] = Point3f (190, 0, 0);
    objectPoints[3] = Point3f (190, 250, 0);
    // cout << "A vector of 3D Object Points = " << objectPoints << endl << endl;
    // cout << "A vector of 2D Image Points = " << imagePoints << endl << endl;

    // Rotation vector output from solvePnP
    Mat rvec (1, 3, CV_32F);
    // Translation vetor - descrabise position of object c.s. in regards
    // to camera c.s
    Mat tvec (1, 3, CV_32F);
    /* cout << "tvec = " << tvec <<  endl; */

    // Estimate object position from 3D-2D point correspondences.
    solvePnP (Mat (objectPoints), Mat (imagePoints), intrinsics,
            distortion, rvec, tvec, false);
    
    // Rotation matrix - describes orientation of object c.s. in regards
    // to camera c.s
    Mat R (3, 3, CV_32F);
    // Converts rotation vector to rotation matrix
    Rodrigues (rvec, R);
    //cout << "R = " << R <<  endl;

    // A matrix store unit converted rotation matrix
    Mat A (3, 3, CV_32F);
    A = intrinsics * R; 

    // B vector stores crorrected translation vector
    Mat B (3, 1, CV_32F);
    B = intrinsics * tvec;
    //cout << "B = " << B <<  endl;
    //cout << "A = " << endl << " " << A << endl << endl;

    double lambdaX, lambdaY, lambdaRo, rhoCrtano, thetaCrtano;
    // lambdaX = a11*cos(theta) + a21*sin(theta) - ro*a31
    lambdaX = A.at<double>(0,0) * cos(theta) + A.at<double>(1,0) *
        sin(theta) - rho * (A.at<double>(2,0));
    // lambdaY = a12*cos(theta) + a22*sin(theta) - ro*a32
    lambdaY = A.at<double>(0,1) * cos(theta) + A.at<double>(1,1) *
        sin(theta) - rho * (A.at<double>(2,1));
    // lamdbaRo = b3*ro - b1*cos(theta) - b2*sin(theta) 
    lambdaRo = rho * (B.at<double>(2)) - B.at<double>(0) * cos(theta) -
        B.at<double>(1) * sin(theta); 

    thetaCrtano = atan2 (lambdaY, lambdaX);
    rhoCrtano = lambdaRo / sqrt (lambdaX * lambdaX + lambdaY * lambdaY);
    cout << "Theta = " << thetaCrtano*180/CV_PI << endl;
    cout << "Rho = " << rhoCrtano << endl;
}

void surfFlannMatcher ()
{
    Mat img1 = imread (globalArgv[1], CV_LOAD_IMAGE_GRAYSCALE);
    Mat img2 = imread (globalArgv[2], CV_LOAD_IMAGE_GRAYSCALE);
    if (!img1.data || !img2.data)  
        cout << " --(!) Error reading images " << endl;

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector(minHessian);

    std::vector<KeyPoint> keypoints1, keypoints2;

    detector.detect (img1, keypoints1);
    detector.detect (img2, keypoints2);

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors1, descriptors2;

    extractor.compute( img1, keypoints1, descriptors1 );
    extractor.compute( img2, keypoints2, descriptors2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match (descriptors1, descriptors2, matches);

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < descriptors1.rows; i++) { 
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf ("-- Max dist : %f \n", max_dist);
    printf ("-- Min dist : %f \n", min_dist);

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
    //-- PS.- radiusMatch can also be used here.
    std::vector<DMatch> goodMatches;

    for (int i = 0; i < descriptors1.rows; i++) { 
        if (matches[i].distance < 2*min_dist) 
            goodMatches.push_back( matches[i]); 
    }  

    //-- Draw only "good" matches
    Mat imgMatches;
    drawMatches (img1, keypoints1, img2, keypoints2, 
            goodMatches, imgMatches, Scalar::all(-1), Scalar::all(-1), 
            vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS); 

    //-- Show detected matches
    imshow ("Good Matches", imgMatches);

    for (int i = 0; i < goodMatches.size(); i++) {
        printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d \
                \n", i, goodMatches[i].queryIdx, 
                goodMatches[i].trainIdx); 
    }
}
