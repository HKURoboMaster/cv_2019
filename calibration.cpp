#include <iostream>
#include <cstdlib>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;
Mat intrinsic_matrix, distortion_coeff, img;
VideoCapture cap;
int camera_id, samples_count, samples_target, verbose;
vector<vector<Point2f>> image_points;
Size image_size, board_size;
double square_size;

void init_default_values()
{
    image_size = Size(640, 480);
    board_size = Size(8, 6);
    samples_target = 25;
    square_size = 20;
    verbose = 0;
}

bool parse(int argc, char *argv[])
{
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--width") == 0 && i+1 < argc)
        {
            image_size.width = atoi(argv[++i]);
        }
        else if(strcmp(argv[i], "--height") == 0 && i+1 < argc)
        {
            image_size.height = atoi(argv[++i]);
        }
        else if(strcmp(argv[i], "--board-width") == 0 && i+1 < argc)
        {
            board_size.width = atoi(argv[++i]);
        }
        else if(strcmp(argv[i], "--board-height") == 0 && i+1 < argc)
        {
            board_size.height = atoi(argv[++i]);
        }
        else if(strcmp(argv[i], "--square-size") == 0 && i+1 < argc)
        {
            square_size = static_cast<double>(atoi(argv[++i]));
        }
        else if(strcmp(argv[i], "--camera") == 0 && i+1 < argc)
        {
            camera_id = atoi(argv[++i]);
        }
        else if(strcmp(argv[i], "--samples") == 0 && i+1 < argc)
        {
            samples_target = atoi(argv[++i]);
        }
        else if(strcmp(argv[i], "--verbose") == 0 && i+1 < argc)
        {
            verbose = atoi(argv[++i]);
        }
        else
        {
            cout << "Unrecognized parameter " << argv[i] << endl;
            return false;
        }
    }
    return true;
}

void calibration()
{
    vector<Mat> rvecs, tvecs;
    vector<vector<Point3f>> object_points(1);
    vector<float> reprojection_err;
    double avg_err = 0.0;
    intrinsic_matrix = Mat::eye(3, 3, CV_64F);
    distortion_coeff = Mat::zeros(8, 1, CV_64F);
    
    for(int i = 0; i < board_size.height; i++)
    {
        for(int j = 0; j < board_size.width; j++)
        {
            object_points[0].push_back(Point3f(j * square_size, i * square_size, 0));
        }
    }
    object_points.resize(image_points.size(), object_points[0]);

    calibrateCamera(object_points, image_points, image_size, intrinsic_matrix, distortion_coeff, rvecs, tvecs, 0);

    ofstream fout("camera.txt");
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            fout << intrinsic_matrix.at<double>(i, j) << ' ';
        }
        fout << endl;
    }
    for(int i = 0; i < 8; i++)
        fout << distortion_coeff.at<double>(0, i) << ' ';
    fout << endl;
    fout.close();
    cout << "Calibration result saved to camera.txt" << endl;
}

int main(int argc, char *argv[])
{
    init_default_values();
    if(!parse(argc, argv)) return 1;
    int time_stamp = clock();
    if(!cap.open(camera_id))
    {
        cout << "Unable to open camera #" << camera_id << endl;
        return 1;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, image_size.width);
    cap.set(CAP_PROP_FRAME_HEIGHT, image_size.height);
    if(!cap.read(img))
    {
        cout << "Unable to read image from camera #" << camera_id << endl;
        return 1;
    }
    int samples_count = 0;
    vector<Point2f> points;
    while(cap.read(img))
    {
        if(samples_count >= samples_target)
        {
            calibration();
            break;
        }
        if(findChessboardCorners(img, board_size, points, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE) && (clock() - time_stamp) * 1.0 / CLOCKS_PER_SEC > 0.5)
        {
            ++samples_count;
            image_points.push_back(points);
            time_stamp = clock();
            cout << "Sample " << samples_count << "/" << samples_target << endl;
            points.clear();
        }
        if(verbose)
        {
            imshow("Camera", img);
            if(waitKey(10) == 27)
                break;
        }
    }
    return 0;
}
