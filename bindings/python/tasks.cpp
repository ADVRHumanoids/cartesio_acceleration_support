#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "Force.h"

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

PYBIND11_MODULE(tasks, m)
{
    py::class_<ForceTask, TaskDescription, ForceTask::Ptr>
            (m, "ForceTask", py::multiple_inheritance())
            .def_static("FromTask", from_task, py::return_value_policy::reference_internal)
            .def("getForceValue", &ForceTask::getForceValue)
            .def("setForceReference", &ForceTask::setForceReference);


    py::class_<ForceTaskRosClient, ForceTask, ForceTaskRosClient::Ptr>
            (m, "ForceTaskRosClient", py::multiple_inheritance());
}

