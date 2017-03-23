/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan */

#include "ompl/util/PPM.h"
#include "ompl/util/Exception.h"
#include <cstdio>

void ompl::PPM::loadFile(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    if (fp == nullptr)
        throw Exception("Unable to load '" + std::string(filename) + "'");
    struct AutoClose
    {
        AutoClose(FILE *f) : f_(f)
        {
        }
        ~AutoClose()
        {
            fclose(f_);
        }
        FILE *f_;
    };
    AutoClose _(fp);  // close fp when this variable goes out of scope.

    char p6[2] = {0};
    if (fscanf(fp, "%c", &p6[0]) != 1 || fscanf(fp, "%c", &p6[1]) != 1 || p6[0] != 'P' || p6[1] != '6')
        throw Exception("Invalid format for file '" + std::string(filename) + "'. PPM is expected to start with the "
                                                                              "characters 'P6'.");
    int nc = fgetc(fp);
    while ((char)nc != '#' && ((char)nc > '9' || (char)nc < '0'))
        nc = fgetc(fp);
    if ((char)nc == '#')
        while ((char)nc != '\n')
            nc = fgetc(fp);
    else
        ungetc(nc, fp);
    if (fscanf(fp, "%d", &width_) != 1 || fscanf(fp, "%d", &height_) != 1)
        throw Exception("Unable to parse width and height for '" + std::string(filename) + "'");
    if (width_ <= 0 || height_ <= 0)
        throw Exception("Invalid image dimensions for '" + std::string(filename) + "'");
    if (fscanf(fp, "%d", &nc) != 1 || nc != 255 /* RGB component color marker*/)
        throw Exception("Invalid RGB component for '" + std::string(filename) + "'");
    fgetc(fp);
    nc = width_ * height_ * 3;
    pixels_.resize(width_ * height_);
    if ((int)fread(&pixels_[0], sizeof(unsigned char), nc, fp) != nc)
        throw Exception("Unable to load image data from '" + std::string(filename) + "'");
}

void ompl::PPM::saveFile(const char *filename)
{
    if (pixels_.size() != width_ * height_)
        throw Exception("Number of pixels is " + std::to_string(pixels_.size()) +
                        " but the set width and height require " + std::to_string(width_ * height_) + " pixels.");
    FILE *fp;
    fp = fopen(filename, "wb");
    if (fp == nullptr)
        throw Exception("Unable to open '" + std::string(filename) + "' for writing");
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", width_, height_);
    fprintf(fp, "%d\n", 255);  // RGB marker
    fwrite(&pixels_[0], 3 * width_, height_, fp);
    fclose(fp);
}
