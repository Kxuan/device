#include <iostream>
#include <boost/asio.hpp>
#include "Port.hpp"

using namespace boost::asio;

int main()
{
    io_context io;
    Port port{io};
    io.run();
    return 0;
}