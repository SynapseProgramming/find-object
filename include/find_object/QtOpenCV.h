/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef QTOPENCV_H
#define QTOPENCV_H

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <QtGui/QImage>
#include <opencv2/core/core.hpp>

// Convert OpenCV matrix to QImage
FINDOBJECT_EXP QImage cvtCvMat2QImage(const cv::Mat & image, bool isBgr = true);

// Convert QImage to OpenCV matrix
FINDOBJECT_EXP cv::Mat cvtQImage2CvMat(const QImage & image);

// Convert IplImage to QImage
FINDOBJECT_EXP QImage cvtIplImage2QImage(const IplImage * image);

// Convert QImage to IplImage
FINDOBJECT_EXP IplImage * cvtQImage2IplImage(const QImage & image);

#endif // QTOPENCV_H