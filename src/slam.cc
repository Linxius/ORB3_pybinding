#include "System.h"
#include "cvt.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

#define ARG1 "extern/ORB_SLAM3/Vocabulary/ORBvoc.txt"
#define ARG2 "extern/ORB_SLAM3/Examples/Monocular/EuRoC.yaml"

class System {
public:
  System(std::string arg1 = ARG1, std::string arg2 = ARG2)
      : SLAM_System(arg1, arg2, ORB_SLAM3::System::MONOCULAR, true) {}

  ~System() { SLAM_System.Shutdown(); }

  py::array step(py::array_t<uint8_t> &img, double tframe) {
    cv::Mat im = CVT::nparray_to_mat(img);

    if (im.empty()) {
      std::cerr << std::endl << "empty image" << std::endl;
      return py::array();
    }

    cv::Mat camera_pose = SLAM_System.TrackMonocular(im, tframe);

    return CVT::mat_to_nparray(camera_pose);
  }

  std::vector<std::pair<py::array, py::array>> get_map_points() {

    std::vector<ORB_SLAM3::MapPoint *> vMapPoints =
        SLAM_System.GetTrackedMapPoints();

    std::vector<std::pair<py::array, py::array>> mapPoints;
    cv::Mat pos_m;
    cv::Mat normal_m;
    py::array pos;
    py::array normal;
    for (int i = 0; i < vMapPoints.size(); i++) {
      if (vMapPoints[i] != NULL) {
        pos_m = (*(vMapPoints[i])).GetWorldPos();
        pos = CVT::mat_to_nparray(pos_m);
        normal_m = (*(vMapPoints[i])).GetNormal();
        normal = CVT::mat_to_nparray(normal_m);
        mapPoints.push_back(std::make_pair(pos, normal));
      }
    }
    return mapPoints;
  }

private:
  ORB_SLAM3::System SLAM_System;
};

PYBIND11_MODULE(orbslam, m) {
  m.doc() = R"pbdoc(
        ORB_SLAM3 python binding
    )pbdoc";

  py::class_<System>(m, "System")
      .def(py::init<const string, const string>(), py::arg("vacabulary") = ARG1,
           py::arg("config") = ARG2)
      .def("get_map_points", &System::get_map_points)
      .def("step", &System::step);

  // #ifdef VERSION_INFO
  //     m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
  // #else
  //     m.attr("__version__") = "dev";
  // #endif
}
