#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>
#include "_env.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_env, m) {
    m.doc() = "pybind11";
    py::class_<Map>(m, "Map")
        .def(py::init<>())
        .def("append_landmark", &Map::append_landmark)
        .def_property_readonly("landmarks", [](Map & m)
        {
            return m.landmarks;
        })
        ;
    py::class_<std::vector<Landmark>>(m, "landmarks")
        .def(py::init<>())
        .def("__iter__", [](std::vector<Landmark> & v) {
            return py::make_iterator(v.begin(), v.end());
        }, py::keep_alive<0, 1>())
        .def("__getitem__", [](std::vector<Landmark> & v, size_t i) {
            return v[i];
        })
        ;
    py::class_<Landmark>(m, "Landmark")
        .def(py::init<const float, const float>())
        .def_property_readonly("pos",[](const Landmark & lm)
		{
			return py::array_t<double>(
				{2},
				{sizeof(double)},
				lm.pos,
				py::cast(lm)
			);
		})
        .def_readwrite("id", & Landmark::id)
        ;
};
