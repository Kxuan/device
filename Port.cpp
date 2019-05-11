/*
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Copyright (c) 2019 kXuan <kxuanobj@gmail.com>. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "config.h"
#include "Port.hpp"
#include <memory>
#include <boost/bind.hpp>
#include <iostream>

using boost::asio::ip::udp;

Port::Port(boost::asio::io_context &io_context, const std::string &name)
        : socket_(io_context, udp::endpoint(udp::v6(), DEFAULT_PORT_NUMBER)),
          name(name)
{
    start_receive();
}

void Port::start_receive()
{
    socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            boost::bind(&Port::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
}

void Port::handle_receive(const boost::system::error_code &error, std::size_t nbytes)
{
    if (!error) {

        std::shared_ptr<std::string> message(
                new std::string(remote_endpoint_.address().to_string()));

        socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
                              boost::bind(&Port::handle_send, this, message,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));

        start_receive();
    }
}

void Port::handle_send(const std::shared_ptr<std::string> &ptr, const boost::system::error_code &ec, std::size_t nbytes)
{
    std::cout << *ptr << ec.message() << nbytes << std::endl;
}
