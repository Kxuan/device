#include <iostream>
#include <boost/asio.hpp>
#include "Port.hpp"
#include "VideoBus.hpp"

using namespace boost::asio;

int main()
{
    VideoBus bus;
    io_context io;
    Port port{io, "debugdev", bus};
    io.run();
    return 0;
}