#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "cvt.h"

namespace py = pybind11;

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <System.h>

using namespace std;

#include <windows.h>

static void usleep(__int64 usec) {
  HANDLE timer;
  LARGE_INTEGER ft;

  ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative
                              // value indicates relative time

  timer = CreateWaitableTimer(NULL, TRUE, NULL);
  SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
  WaitForSingleObject(timer, INFINITE);
  CloseHandle(timer);
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

//.\Monocular\Release\mono_euroc.exe
// ..\Vocabulary\ORBvoc.txt
//.\Monocular\EuRoC.yaml
// ..\data\MH_01_easy\
//./Monocular/EuRoC_TimeStamps/MH01.txt
// dataset-MH01_mono
#define ARG1 "extern/ORB_SLAM3/Vocabulary/ORBvoc.txt"
#define ARG2 "extern/ORB_SLAM3/Examples/Monocular/EuRoC.yaml"
#define ARG3 "extern/ORB_SLAM3/data/MH_01_easy"
#define ARG4 "extern/ORB_SLAM3/Examples/Monocular/EuRoC_TimeStamps/MH01.txt"
#define ARG5 "dataset-MH01_mono"

// int main(int argc, char **argv) {
int test(string arg1 = ARG1, string arg2 = ARG2, string arg3 = ARG3,
         string arg4 = ARG4, string arg5 = ARG5) {
  const int num_seq = 1;
  string file_name = arg5;
  int seq = 0;
  vector<vector<string>> vstrImageFilenames;
  vector<vector<double>> vTimestampsCam;
  vector<int> nImages;

  vstrImageFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  cout << "Loading images for sequence " << seq << "...";
  LoadImages(arg3 + "/mav0/cam0/data", arg4, vstrImageFilenames[seq],
             vTimestampsCam[seq]);
  cout << "LOADED!" << endl;

  nImages[seq] = vstrImageFilenames[seq].size();
  tot_images += nImages[seq];

  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(arg1, arg2, ORB_SLAM3::System::MONOCULAR, true);

  for (seq = 0; seq < num_seq; seq++) {
    // Main loop
    cv::Mat im;
    int proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {

      // Read image from file
      im = cv::imread(vstrImageFilenames[seq][ni], cv::IMREAD_UNCHANGED);
      double tframe = vTimestampsCam[seq][ni];

      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageFilenames[seq][ni]
             << endl;
        return 1;
      }

      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();

      // Pass the image to the SLAM system
      // cout << "tframe = " << tframe << endl;
      SLAM.TrackMonocular(im, tframe); // TODO change to monocular_inertial

      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[seq] - 1)
        T = vTimestampsCam[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[seq][ni - 1];

      if (ttrack < T)
        usleep((T - ttrack) * 1e6); // 1e6
    }

    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  //   if (bFileName) {
  //     const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
  //     const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
  //     SLAM.SaveTrajectoryEuRoC(f_file);
  //     SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  //   } else {
  //     SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
  //     SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  //   }

  return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps) {
  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e9);
    }
  }
}

class System {
public:
  // SLAM(string arg1 = ARG1, string arg2 = ARG2, string arg3 = ARG3) {
  System(const string arg1 = ARG1, const string arg2 = ARG2,
         const string arg3 = ARG3, const string arg4 = ARG4,
         const string arg5 = ARG5)

      : SLAM_System(arg1, arg2, ORB_SLAM3::System::MONOCULAR, true) {}

  ~System() { SLAM_System.Shutdown(); }

  float step(py::array_t<uint8_t> &input_img_array, double tframe) {
    cv::Mat im = CVT::nparray_to_mat(input_img_array);
    if (im.empty()) {
      cerr << endl << "input image array is empty" << endl;
      return -1;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    SLAM_System.TrackMonocular(im,
                               tframe); // TODO change to monocular_inertial

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();

    return ttrack;
  }

private:
  ORB_SLAM3::System SLAM_System;
  float last_timestamp = 0;

  // vector<vector<string>> vstrImageFilenames;
  // vector<vector<double>> vTimestampsCam;
  // vector<int> nImages;
  // vector<float> vTimesTrack;
};

void test_cvt(py::array_t<uint8_t> &input_img_array) {
  cv::Mat im = CVT::nparray_to_mat(input_img_array);
  // cv::imshow("test", im);
  cv::imwrite("j.jpg", im);
}

PYBIND11_MODULE(orbslam, m) {
  m.doc() = R"pbdoc(
        ORB_SLAM3 python binding
    )pbdoc";

  m.def("run_test", &test, R"pbdoc(
        ORB_SLAM3 test
    )pbdoc",
        py::arg("vac_file") = ARG1, py::arg("config_file") = ARG2,
        py::arg("data_path") = ARG3, py::arg("timestamp_file") = ARG4,
        py::arg("data_name") = ARG5);

  py::class_<System>(m, "System")
      .def(py::init<const string, const string, const string>(),
           py::arg("vac_file") = ARG1, py::arg("config_file") = ARG2,
           py::arg("data_path") = ARG3)
      // .def("step", &SLAM_Bind::step, py::arg("input_image"));
      .def("step", &System::step);
  m.def("test_cvt", &test_cvt);
  //     m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
  //         Subtract two numbers

  //         Some other explanation about the subtract function.
  //     )pbdoc");

  // #ifdef VERSION_INFO
  //     m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
  // #else
  //     m.attr("__version__") = "dev";
  // #endif
}
