#include <pybind11/pybind11.h>
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
  //   if (argc < 5) {
  //     cerr
  //         << endl
  //         << "Usage: ./mono_euroc path_to_vocabulary path_to_settings "
  //            "path_to_sequence_folder_1 path_to_times_file_1 "
  //            "(path_to_image_folder_2 path_to_times_file_2 ... "
  //            "path_to_image_folder_N path_to_times_file_N)
  //            (trajectory_file_name)"
  //         << endl;
  //     return 1;
  //   }

  //   const int num_seq = (argc - 3) / 2;
  const int num_seq = 1;
  //   cout << "num_seq = " << num_seq << endl;
  //   bool bFileName = (((argc - 3) % 2) == 1);
  string file_name = arg5;
  //   if (bFileName) {
  //   file_name = string(argv[argc - 1]);
  //   file_name = string(argv[argc - 1]);
  //   cout << "file name: " << file_name << endl;
  //   }

  // Load all sequences:
  int seq = 0;
  vector<vector<string>> vstrImageFilenames;
  vector<vector<double>> vTimestampsCam;
  vector<int> nImages;

  vstrImageFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  //   for (seq = 0; seq < num_seq; seq++) {
  cout << "Loading images for sequence " << seq << "...";
  // LoadImages(string(argv[(2 * seq) + 3]) + "/mav0/cam0/data",
  //            string(argv[(2 * seq) + 4]), vstrImageFilenames[seq],
  //            vTimestampsCam[seq]);
  LoadImages(arg3 + "/mav0/cam0/data", arg4, vstrImageFilenames[seq],
             vTimestampsCam[seq]);
  cout << "LOADED!" << endl;

  nImages[seq] = vstrImageFilenames[seq].size();
  tot_images += nImages[seq];
  //   }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  //   ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR,
  //   true);
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

      // #ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
      // #else
      //       std::chrono::monotonic_clock::time_point t1 =
      //           std::chrono::monotonic_clock::now();
      // #endif

      // Pass the image to the SLAM system
      // cout << "tframe = " << tframe << endl;
      SLAM.TrackMonocular(im, tframe); // TODO change to monocular_inertial

      // #ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
      // #else
      //       std::chrono::monotonic_clock::time_point t2 =
      //           std::chrono::monotonic_clock::now();
      // #endif

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
