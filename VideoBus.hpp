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
#ifndef DEVICE_VIDEOBUS_HPP
#define DEVICE_VIDEOBUS_HPP

#include <sys/types.h>
#include <boost/signals2.hpp>
#include <gstreamermm/appsink.h>
#include <gstreamermm/pipeline.h>
#include <glibmm/thread.h>
#include <glibmm/main.h>

class VideoBus : public boost::signals2::signal<void(const uint8_t *, size_t size)>
{
public:

	VideoBus();

	void start();

private:

	Gst::FlowReturn onNewSample();

	void run();

private:
	Glib::RefPtr<Glib::MainLoop> loop;
	Glib::Thread *worker = nullptr;
	Glib::RefPtr<Gst::AppSink> appsink;
};


#endif //DEVICE_VIDEOBUS_HPP
