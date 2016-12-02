/*=========================================================================

Library:   MultipassUltrasound

Copyright 2010 Kitware Inc. 28 Corporate Drive,
Clifton Park, NY, 12065, USA.
All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

=========================================================================*/

#ifndef AngleSkewInvTransform_hxx
#define AngleSkewInvTransform_hxx

#include "AngleSkewInvTransform.h"
#include <math.h>

namespace itk {

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
AngleSkewInvTransform<TParametersValueType, NDimensions>
::AngleSkewInvTransform() :
  Superclass(NDimensions)
{}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
AngleSkewInvTransform<TParametersValueType, NDimensions>
::~AngleSkewInvTransform()
{}

template< 
  typename TParametersValueType,
  unsigned int NDimensions> 
const typename AngleSkewInvTransform<TParametersValueType, NDimensions>::ParametersType &
AngleSkewInvTransform<TParametersValueType, NDimensions>
::GetParameters() const
{ 
  MatrixType matrix = this->GetMatrix();
  double cosTheta = 1 / matrix[0][0];
  double tanTheta = -matrix[1][0] / matrix[1][1];
  double sinTheta = tanTheta * cosTheta;
  
  this->m_Parameters[0] = atan2(sinTheta, cosTheta);
  this->m_Parameters[1] = 1 / matrix[1][1];
  return this->m_Parameters;
}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
void 
AngleSkewInvTransform<TParametersValueType, NDimensions>
::SetParameters(const ParametersType & parameters)
{
  if( parameters.Size() < 2)
  {
    itkExceptionMacro
      (<< "Error setting parameters: parameters array size ("
       << parameters.Size() << ") is less than expected "
       << " NDimensions "
       << " (" << NDimensions << ")"
      );
  }
  if( &parameters != &(this->m_Parameters) )
  {
    this->m_Parameters = parameters;
  }
  
  MatrixType matrix = this->GetMatrix();
  matrix[0][0] = 1/cos(parameters[0]);
  matrix[0][1] = 0;
  matrix[1][0] = -tan(parameters[0]) / parameters[1];
  matrix[1][1] = 1/parameters[1];
  //this->matrixMTime.Modified();
  this->SetMatrix(matrix);
  //std::cout << this->GetMatrix() << std::endl;
  this->Modified();
}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
void 
AngleSkewInvTransform<TParametersValueType, NDimensions>
::ComputeJacobianWithRespectToParameters(const InputPointType  & p, JacobianType & jacobian) const
{
  jacobian.SetSize( NDimensions, NDimensions );
  jacobian.Fill(0.0);
  double cosTheta = cos(this->m_Parameters[0]);
  double tanTheta = tan(this->m_Parameters[0]);
  jacobian(0, 0) = tanTheta / cosTheta * p[0]; 
  jacobian(0, 1) = 0;
  jacobian(1, 0) = -1 / (cosTheta * cosTheta * this->m_Parameters[1]) * p[0];
  jacobian(1, 1) = -(-p[0] * tanTheta + p[1]) / (this->m_Parameters[1] * this->m_Parameters[1] );

  //std::cout<< "jacobian:" << std::endl << jacobian << std::endl;
  
  
}
} //namespace itk

#endif

