// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file pybind_module.cpp
 * @brief Python bindings for the BMS driver library via pybind11.
 * @details Exposes BmsDriver and all its battery status accessors to Python
 *          as the bms_py module.
 */
 
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "bms_driver.hpp"

namespace py = pybind11;

PYBIND11_MODULE(bms_py, m) {
    m.doc() = "BMS Driver Python SDK";

    py::class_<BmsDriver, std::shared_ptr<BmsDriver>>(m, "BmsDriver")
        .def_static("create_bms", &BmsDriver::create_bms,
                     py::arg("bms_type"),
                     py::arg("socket_path") = "/tmp/bms.sock")
        .def("get_voltage", &BmsDriver::get_voltage)
        .def("get_current", &BmsDriver::get_current)
        .def("get_temperature", &BmsDriver::get_temperature)
        .def("get_percentage", &BmsDriver::get_percentage)
        .def("get_charge", &BmsDriver::get_charge)
        .def("get_capacity", &BmsDriver::get_capacity)
        .def("get_design_capacity", &BmsDriver::get_design_capacity)
        .def("get_protect_status", &BmsDriver::get_protect_status)
        .def("get_work_state", &BmsDriver::get_work_state)
        .def("get_max_cell_voltage", &BmsDriver::get_max_cell_voltage)
        .def("get_min_cell_voltage", &BmsDriver::get_min_cell_voltage)
        .def("get_soh", &BmsDriver::get_soh)
        .def("get_cycles", &BmsDriver::get_cycles)
        .def("is_connected", &BmsDriver::is_connected);
}
