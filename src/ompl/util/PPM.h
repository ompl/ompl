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

#ifndef OMPL_UTIL_PPM_
#define OMPL_UTIL_PPM_

#include <vector>

namespace ompl
{
    /** \brief Load and save .ppm files - "portable pixmap format" an image file formats designed
        to be easily exchanged between platforms. */
    class PPM
    {
    public:
        struct Color
        {
            unsigned char red, green, blue;

            // needed for Python bindings
            bool operator==(const Color c)
            {
                return red == c.red && green == c.green && blue == c.blue;
            }
        };

        PPM() = default;

        /* \brief Load a .ppm file. Throw an exception in case of an error. */
        PPM(const char *filename)
        {
            loadFile(filename);
        }

        /** \brief Load a .ppm file. Throw an exception in case of an error. */
        void loadFile(const char *filename);

        /** \brief Save image data to a .ppm file. Throw an exception in case of an error. */
        void saveFile(const char *filename);

        /** \brief Get the width of the loaded image. */
        unsigned int getWidth() const
        {
            return width_;
        }

        /** \brief Get the height of the loaded image. */
        unsigned int getHeight() const
        {
            return height_;
        }

        /** \brief Set the width for the loaded image. This must
        eventually match the number of pixels, if saveFile() gets called. */
        void setWidth(unsigned int width)
        {
            width_ = width;
        }

        /** \brief Set the height for the loaded image. This must
        eventually match the number of pixels, if saveFile() gets called. */
        void setHeight(unsigned int height)
        {
            height_ = height;
        }

        /** \brief Get read-only access to the pixels in the image. To access a pixel at coordinate (row,col),
        use getPixels()[row * getWidth() + col]. */
        const std::vector<Color> &getPixels() const
        {
            return pixels_;
        }
        /** \brief Get write access to the pixels in the image. To access a pixel at coordinate (row,col),
        use getPixels()[row * getWidth() + col].  This must eventually match the width & height set
        by setWidth() and setHeight(). */
        std::vector<Color> &getPixels()
        {
            return pixels_;
        }

        /** \brief Directly access a pixel in the image */
        const Color &getPixel(const int row, const int col) const
        {
            return pixels_[row * width_ + col];
        }

        /** \brief Directly access a pixel in the image */
        Color &getPixel(const int row, const int col)
        {
            return pixels_[row * width_ + col];
        }

    private:
        std::vector<Color> pixels_;
        unsigned int width_{0};
        unsigned int height_{0};
    };
}

#endif
