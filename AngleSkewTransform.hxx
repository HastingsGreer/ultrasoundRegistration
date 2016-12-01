#ifndef AngleSkewTransform_hxx
#define AngleSkewTransform_hxx

#include "AngleSkewTransform.h"
#include <math.h>
template< 
  typename TParametersValueType,
  unsigned int NDimensions>
AngleSkewTransform<TParametersValueType, NDimensions>
::AngleSkewTransform() :
  Superclass(NDimensions)
{}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
AngleSkewTransform<TParametersValueType, NDimensions>
::~AngleSkewTransform()
{}

template< 
  typename TParametersValueType,
  unsigned int NDimensions> 
const typename AngleSkewTransform<TParametersValueType, NDimensions>::ParametersType &
AngleSkewTransform<TParametersValueType, NDimensions>
::GetParameters() const
{
  //Not sure if I need to verify m_Parameters?
  
  return this->m_Parameters;
}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
void 
AngleSkewTransform<TParametersValueType, NDimensions>
::SetParameters(const ParametersType & parameters)
{
  if( parameters.Size() < 3)
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
  matrix[0][0] = cos(parameters[0]);
  matrix[0][1] = 0;
  matrix[1][0] = sin(parameters[0]);
  matrix[1][1] = parameters[1];
  //this->matrixMTime.Modified();
  this->SetMatrix(matrix);
  //std::cout << this->GetMatrix() << std::endl;
  this->Modified();
}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
void 
AngleSkewTransform<TParametersValueType, NDimensions>
::ComputeJacobianWithRespectToParameters(const InputPointType  & p, JacobianType & jacobian) const
{
  jacobian.SetSize( NDimensions, NDimensions );
  jacobian.Fill(0.0);
  /* if parameters = [i j k] and point = [x y z]
   * then T[point] = [x + iz, y + jz, kz]
   * so jacobian is 
   * |z 0 0|
   * |0 z 0|
   * |0 0 z| */
  for(unsigned int i = 0; i < NDimensions; i++){
    jacobian(i, i) = p[NDimensions - 1];
  }
  //std::cout<< "jacobian:" << std::endl << jacobian << std::endl;
  
  
}

#endif

