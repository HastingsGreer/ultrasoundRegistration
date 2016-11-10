#ifndef PrintTransform_h
#define PrintTransform_h

#include "itkTransform.h"

void printTransform(const char * inputFilename, const char * outputFilename, itk::Transform<double, 2, 2>::ConstPointer transform);


#endif
