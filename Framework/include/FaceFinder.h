/*
 * FaceFinder.h
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#ifndef FACEFINDER_H_
#define FACEFINDER_H_

#include <string>

#include "Point.h"
#include "Image.h"
#include "minIni.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core.hpp"

#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;
using namespace cv;

namespace Robot
{
    class FaceFinder
    {
    private:
        Point2D m_center_point;

    public:
        String face_cascade_name;
        int count;
        FaceFinder();
        virtual ~FaceFinder();
        void LoadINISettings();
        Point2D& GetPosition(Image* rgb_img);
    };
}

#endif /* FaceFinder_H_ */
