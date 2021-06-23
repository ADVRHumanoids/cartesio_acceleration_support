#ifndef DYNAMICFEASIBILITY_H
#define DYNAMICFEASIBILITY_H

#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian { namespace acceleration {

class DynamicFeasibility : public virtual TaskDescription
{
public:
    CARTESIO_DECLARE_SMART_PTR(DynamicFeasibility)

    virtual std::vector<std::string> getContactLinks() const = 0;
    virtual bool dynamicsEnabled() const = 0;
};

}}}

#endif // DYNAMICFEASIBILITY_H
