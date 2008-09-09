/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* \author Ioan Sucan */

#include "ompl/base/util/output.h"
#include <iostream>
#include <cstdlib>

static ompl::ma::OutputHandlerSTD defaultOutputHandler;
static ompl::ma::ActionHandlerSTD defaultActionHandler;

static ompl::ma::OutputHandler *OUTPUT_HANDLER = static_cast<ompl::ma::OutputHandler*>(&defaultOutputHandler);
static ompl::ma::ActionHandler *ACTION_HANDLER = static_cast<ompl::ma::ActionHandler*>(&defaultActionHandler);

void ompl::ma::useOutputHandler(OutputHandler *oh)
{
    OUTPUT_HANDLER = oh;
}

void ompl::ma::useActionHandler(ActionHandler *ah)
{
    ACTION_HANDLER = ah;
}

ompl::ma::Interface::Interface(void)
{
}

ompl::ma::Interface::~Interface(void)
{
}

void ompl::ma::Interface::message(const std::string &text)
{
    if (OUTPUT_HANDLER)
	OUTPUT_HANDLER->message(text);
}

void ompl::ma::Interface::message(const char *msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    message(combine(msg, ap));
    va_end(ap);
}

void ompl::ma::Interface::inform(const std::string &text)
{
    if (OUTPUT_HANDLER)
	OUTPUT_HANDLER->inform(text);
}

void ompl::ma::Interface::inform(const char *msg, ...)
{
    va_list ap; 
    va_start(ap, msg);
    inform(combine(msg, ap));
    va_end(ap);
}

void ompl::ma::Interface::warn(const std::string &text)
{
    if (OUTPUT_HANDLER)
	OUTPUT_HANDLER->warn(text);
}

void ompl::ma::Interface::warn(const char *msg, ...)
{
    va_list ap; 
    va_start(ap, msg);
    warn(combine(msg, ap));
    va_end(ap);
}

void ompl::ma::Interface::error(const std::string &text)
{
    if (OUTPUT_HANDLER)
	OUTPUT_HANDLER->error(text);
}

void ompl::ma::Interface::error(const char *msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    error(combine(msg, ap));
    va_end(ap);
}

void ompl::ma::Interface::act(int action)
{
    if (ACTION_HANDLER)
    {
	if (action & EA_DIE)
	    ACTION_HANDLER->die(action & EA_DIE_I);
	else
	    if (action & EA_DIE_I)
		ACTION_HANDLER->die(true);  
    }
}

void ompl::ma::Interface::messageAndAct(int action, int mt, const char *msg, ...)
{
    va_list ap; 
    va_start(ap, msg);
    messageAndAct(action, mt, combine(msg, ap));
    va_end(ap);
}

void ompl::ma::Interface::messageAndAct(int action, int mt, const std::string &text)
{ 
    switch (mt)
    {
    case MT_WARNING:
	warn(text);
	break;
    case MT_ERROR:
	error(text);
	break;
    case MT_INFORMATION:
	inform(text);
	break;
    case MT_NONE:
	message(text);
	break;
    default:
	break;
    }
    
    act(action);
}

std::string ompl::ma::Interface::combine(const char *msg, va_list va)
{
    va_list ap;
    va_copy(ap, va);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), msg, ap);
    va_end(ap);
    return buf;
}

void ompl::ma::OutputHandlerSTD::error(const std::string &text)
{
    std::cerr << "Error:   " << text << std::endl;
    std::cerr.flush();
}

void ompl::ma::OutputHandlerSTD::warn(const std::string &text)
{
    std::cerr << "Warning: " << text << std::endl;
    std::cerr.flush();
}

void ompl::ma::OutputHandlerSTD::inform(const std::string &text)
{ 
    std::cout << "Info:    " << text << std::endl;
    std::cout.flush();
}

void ompl::ma::OutputHandlerSTD::message(const std::string &text)
{
    std::cout << text;
    std::cout.flush();
}

void ompl::ma::ActionHandlerSTD::die(bool immediate)
{
    if (immediate)
	_Exit(1);
    else
	exit(1);
}
