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
#include "VideoBus.hpp"
#include <iostream>
#include <glibmm/thread.h>
#include <glibmm/main.h>
#include <gstreamermm/appsink.h>
#include <gstreamermm/pipeline.h>
#include <gstreamermm/element.h>
#include <gstreamermm/elementfactory.h>
#include <gstreamermm/caps.h>
#include <gstreamermm/init.h>

VideoBus::VideoBus()
{
	Glib::thread_init();
	Gst::init();

	loop = Glib::MainLoop::create();
}

Gst::FlowReturn VideoBus::onNewSample()
{
	auto sample = appsink->pull_sample();
	if (sample) {
		auto buf = sample->get_buffer();
		Gst::MapInfo mi;
		buf->map(mi, Gst::MapFlags::MAP_READ);

		(*this)(mi.get_data(), mi.get_size());
		buf->unmap(mi);
		std::cout << '.';
		std::flush(std::cout);
	}
	return Gst::FlowReturn::FLOW_OK;
}

void VideoBus::run()
{

	auto pipeline = Gst::Pipeline::create();
	auto v4l2src = Gst::ElementFactory::create_element("v4l2src");
	std::string devicename = "/dev/video0";
	v4l2src->set_property("device", devicename);

	appsink = Gst::AppSink::create();
	appsink->signal_new_sample().connect(sigc::mem_fun(*this, &VideoBus::onNewSample));
	appsink->set_property("emit-signals", true);

#ifdef __x86_64__
	auto videoconvert = Gst::VideoConvert::create();
	auto x264enc = Gst::ElementFactory::create_element("x264enc");
	auto rtph264pay = Gst::ElementFactory::create_element("rtph264pay");

	pipeline->add(v4l2src)->add(videoconvert)->add(x264enc)->add(rtph264pay)->add(appsink);
	v4l2src->link(videoconvert)->link(x264enc)->link(rtph264pay)->link(appsink);
#else
	auto rtph264pay = Gst::ElementFactory::create_element("rtph264pay");
	auto caps = Gst::Caps::create_from_string("video/x-h264,width=1280,height=720,framerate=30/1");

	pipeline->add(v4l2src)->add(rtph264pay)->add(appsink);
	v4l2src->link(rtph264pay, caps)->link(appsink);
#endif

	pipeline->set_state(Gst::State::STATE_PLAYING);
	loop->run();
	pipeline->set_state(Gst::State::STATE_NULL);
	appsink.reset();
}

void VideoBus::start()
{
	if (worker) {
		return;
	}
	worker = Glib::Thread::create(sigc::mem_fun(*this, &VideoBus::run));
}
