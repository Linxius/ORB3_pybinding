#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace py = pybind11;

namespace CVT {

py::dtype determine_np_dtype(int depth);

std::vector<std::size_t> determine_shape(cv::Mat &m);

py::capsule make_capsule(cv::Mat &m);

py::array mat_to_nparray(cv::Mat &m);

cv::Mat nparray_to_mat(py::array a);

} // namespace CVT