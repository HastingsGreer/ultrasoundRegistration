#ifndef PrintTransform_h
#define PrintTransform_h

#include "itkTransform.h"

void printTransform(char * inputFilename, char * outputFilename, itk::Transform<double, 2, 2>::ConstPointer transform);


#endif
