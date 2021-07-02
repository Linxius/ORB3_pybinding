#include "cvt.h"

namespace CVT {

py::dtype determine_np_dtype(int depth) {
  switch (depth) {
  case CV_8U:
    return py::dtype::of<uint8_t>();
  case CV_8S:
    return py::dtype::of<int8_t>();
  case CV_16U:
    return py::dtype::of<uint16_t>();
  case CV_16S:
    return py::dtype::of<int16_t>();
  case CV_32S:
    return py::dtype::of<int32_t>();
  case CV_32F:
    return py::dtype::of<float>();
  case CV_64F:
    return py::dtype::of<double>();
  default:
    throw std::invalid_argument("Unsupported data type.");
  }
}

std::vector<std::size_t> determine_shape(cv::Mat &m) {
  if (m.channels() == 1) {
    return {static_cast<size_t>(m.rows), static_cast<size_t>(m.cols)};
  }

  return {static_cast<size_t>(m.rows), static_cast<size_t>(m.cols),
          static_cast<size_t>(m.channels())};
}

py::capsule make_capsule(cv::Mat &m) {
  return py::capsule(new cv::Mat(m),
                     [](void *v) { delete reinterpret_cast<cv::Mat *>(v); });
}

py::array mat_to_nparray(cv::Mat &m) {
  if (!m.isContinuous()) {
    throw std::invalid_argument("Only continuous Mats supported.");
  }

  return py::array(determine_np_dtype(m.depth()), determine_shape(m), m.data,
                   make_capsule(m));
}

cv::Mat nparray_to_mat(py::array a) {
  auto rows = a.shape(0);
  auto cols = a.shape(1);
  auto type = CV_8UC3;
  cv::Mat m(rows, cols, type, (unsigned char *)a.data());
  // cv::imshow("test", m);
  return m;
}

} // namespace CVT