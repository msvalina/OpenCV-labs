#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

bool readImg (char*, Mat*);
bool isPlane (vector<float>, vector<float>, vector<float>* );
int planePoints (Mat*, vector<float>);

inline int rand (int min, int max)
{ 
    return (rand() % max) + min;
}

int main (int argc, char *argv[])
{
    string imgFile;
    char *dFile;
    if (argc < 2) {
        cout << "Using first default KincktPic" << endl;
        imgFile = "../assignments/kinecktPics/KinectPics\\sl-00001.bmp";
        dFile = "../assignments/kinecktPics/KinectPics\\sl-00001-D.txt";
    }
    else {
        imgFile = argv[1];
        dFile = argv[2];
    }

    // Load color image
    Mat img = imread(imgFile, CV_LOAD_IMAGE_COLOR);
    if (img.data == NULL) 
        return -1;

    // Load Kinect depth map
    Mat imgDepth(img.rows, img.cols, CV_8UC1);
    if (readImg(dFile, &imgDepth) == 0) 
        return -1;

    // Use RANSAC to determine dominant plane
    srand(time(NULL));
    int ransacRounds = 1000;
    int bestPlane = 0;
    vector<float> bPlane;

    for (int i = 0; i < ransacRounds; i++) {
        vector<float> points, depth, plane;
        points.clear(); depth.clear(); plane.clear();

        for (int j = 0; j < 3; j++) {
            int u = rand (0, img.cols);
            int v = rand (0, img.cols);
            int d = imgDepth.at<uchar>(v,u);

            points.push_back((float)u);
            points.push_back((float)v);
            points.push_back(1.0f);

            depth.push_back((float)d);
        }

        if (!isPlane (points, depth, &plane))
            continue; // Invalid point, skip iteration

        int num = planePoints (&imgDepth, plane);
        if (num > bestPlane) {
            bestPlane = num;
            bPlane.swap(plane);
        }

    }

    // Mark result
    int width = imgDepth.cols;
    int height = imgDepth.rows;
    cvtColor (imgDepth, imgDepth, COLOR_GRAY2RGB);

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            uchar d = imgDepth.at<Vec3b>(j,i)[0];
            float err = (float)d - (bPlane[0]*i + bPlane[1]*j + bPlane[2]);
            if (err < 0) err *= -1;
            if (err < 4.0f) {
                imgDepth.at<Vec3b>(j,i)[2] = 0; // remove red
                imgDepth.at<Vec3b>(j,i)[1] = 0; // remove green
            }
        }
    }

    // Output result
    imshow("Original", img);
    imshow("Depth", imgDepth);
    waitKey(0);

    return 0;
}

bool readImg (char *path, cv::Mat *depth) 
{ // Read Kinect image
    FILE *file = fopen(path, "r");
    if (file == NULL) 
        return 0;

    int width = depth->cols;
    int height = depth->rows;
    int map[width * height]; // temp field for reading
    int min = 2047, max = 0; // (min,max) -> (0,255)

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            int d;
            map[j * width + i] = 0; // clear memory

            while (fscanf (file, "%d ", &d) != 1);
            // if (fscanf(file, "%d ", &d) != 1) break;

            if (d==2047)
                d=-1; // Invalid depth (unable to measure)
            else {
                if (d>max) max=d;
                if (d<min) min=d;
            }

            map[j * width + i] = d; // temp storage
        }
    }

    fclose (file);

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {

            int d = map[j * width + i]; // read from temp storage
            if (d >= min)
                d = ((d - min) * 254 / (max - min)) + 1; // scale to fit
            else
                d = 0;
            depth->at<uchar>(j,i) = d; // write to output matrix
        }
    }

    return 1;
}

bool isPlane (vector<float> points, vector<float> depth, vector<float> *plane) 
{
    Mat A_(3, 3, CV_32FC1, &points[0]);
    Mat Z_(3, 1, CV_32FC1, &depth[0]);
    Mat p_(3, 1, CV_32FC1);

    // Solves linear system with two equations
    // if (u,v) are on the same plane as depth
    // save plane location
    if (solve(A_, Z_, p_)) {
        for (int i = 0; i < 3; i++) plane->push_back(p_.at<float>(i));
        return 1; // valid points
    } else
        return 0; // selected points are on the same line
}

int planePoints (Mat *depth, vector<float> plane) 
{
    float thr = 4.0f; // threshold
    int nMatch = 0; // number of matches

    // Calculate number of points on the sam plain/depth
    for (int j = 0; j < depth->rows; j++) {
        for (int i = 0; i < depth->cols; i++) {
            uchar d = depth->at<uchar>(j,i);
            float err = (float)d - (plane[0] * i + plane[1] * j + plane[2]);
            if (err < 0) err *= -1;
            if (err > thr)
                continue;
            else
                nMatch++;
        }
    }
    return nMatch;
}
