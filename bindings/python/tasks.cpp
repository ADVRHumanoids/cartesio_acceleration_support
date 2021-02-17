#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "Force.h"
#include "ForceLimits.h"

namespace py = pybind11;

using namespace XBot;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::acceleration;

auto from_task = [](TaskDescription::Ptr task)
{
     auto ftask = std::dynamic_pointer_cast<ForceTask>(task);
     if(!ftask)
     {
         throw py::type_error("Task type does not match 'ForceTask'");
     }
     return ftask;
};

auto from_contraint = [](ConstraintDescription::Ptr constraint)
{
    auto flimit = std::dynamic_pointer_cast<ForceLimits>(constraint);
    if(!flimit)
    {
        throw py::type_error("Constrain type does not match 'ForceLimits'");
    }
    return flimit;
};

auto get_limits = [](ForceLimitsRosClient& self)
{
    Eigen::Vector6d fmin, fmax;
    self.getLimits(fmin, fmax);

    return std::make_tuple(fmin, fmax);
};

PYBIND11_MODULE(tasks, m)
{
    py::class_<ForceTask, TaskDescription, ForceTask::Ptr>
            (m, "ForceTask", py::multiple_inheritance())
            .def(py::init(from_task), py::return_value_policy::reference)
            .def("getForceValue", &ForceTask::getForceValue)
            .def("setForceReference", &ForceTask::setForceReference);

    py::class_<ForceTaskRosClient, ForceTask, ForceTaskRosClient::Ptr>
            (m, "ForceTaskRosClient", py::multiple_inheritance());

    py::class_<ForceLimits, TaskDescription, ForceLimits::Ptr>
            (m, "ForceLimits", py::multiple_inheritance())
            .def(py::init(from_contraint), py::return_value_policy::reference)
            .def("getLimits", get_limits)
            .def("setLimits", &ForceLimits::setLimits);

    py::class_<ForceLimitsRosClient, ForceLimits, ForceLimitsRosClient::Ptr>
            (m, "ForceLimitsRos", py::multiple_inheritance());
}

