#include "learning_gem5/part2/hello_object.hh"

#include "base/trace.hh"
#include "debug/HelloExample.hh"

namespace gem5
{

    HelloObject::HelloObject(const HelloObjectParams &params) :
        SimObject(params)
    {
        //std::cout << "Hello World! From a SimObject!" << std::endl;
        DPRINTF(HelloExample, "Created this nice object!\n");
    }

} // namespace gem5

