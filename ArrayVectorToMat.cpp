#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[]){

    Mat imgin = imread("Utils/imgTeste.png", 0);

    vector<uchar> buffer;
    vector<int> param(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[0] = 95;

    imencode(".jpg", imgin, buffer, param);

    Mat imgout(imgin.rows, imgin.cols, CV_8UC1);

    cout    << "Img IN : " << imgin.size() << "\n"
            << "Img OUT: " << imgout.size() << "\n";


    for (register unsigned int x = 0; x < imgin.cols; x++)
        for (register unsigned int y = 0; y < imgin.rows; y++)
            imgout.at<uchar>(y,x) = imgin.at<uchar>(y,x); // buffer[x + y*imgout.cols];

    // ifstream ArrayFile("Utils/ArrayData.txt");

    // if (!ArrayFile.is_open())
    //     return -1;

    // // size 90240 width 376 height 240

    // string caracter;

    // while(ArrayFile.good()) {
    //     getline(ArrayFile, caracter);
    //     cout << caracter;
    //     cin.get();
    // }

    cv::namedWindow("IN", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("OUT", CV_WINDOW_AUTOSIZE);

    imshow("IN", imgin);
    imshow("OUT", imgout);

    waitKey();

    cout << endl;

    return 0;
}