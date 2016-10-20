#ifndef SkewTransform_hxx
#define SkewTransform_hxx

#include "SkewTransform.h"

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
SkewTransform<TParametersValueType, NDimensions>
::SkewTransform() :
  Superclass(NDimensions)
{}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
SkewTransform<TParametersValueType, NDimensions>
::~SkewTransform()
{}

template< 
  typename TParametersValueType,
  unsigned int NDimensions> 
const typename SkewTransform<TParametersValueType, NDimensions>::ParametersType &
SkewTransform<TParametersValueType, NDimensions>
::GetParameters() const
{
  const MatrixType& matrix = this->GetMatrix();
  for( unsigned int row = 0; row < NDimensions; row++)
  {
    this->m_Parameters[row] = matrix[row][NDimensions - 1];
  }
  return this->m_Parameters;
}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
void 
SkewTransform<TParametersValueType, NDimensions>
::SetParameters(const ParametersType & parameters)
{
  if( parameters.Size() < NDimensions)
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
  for( unsigned int row = 0; row < NDimensions; row++)
  {
    matrix[row][NDimensions - 1] = this->m_Parameters[row];
  }
  //this->matrixMTime.Modified();
  this->SetMatrix(matrix);
  this->Modified();
}

template< 
  typename TParametersValueType,
  unsigned int NDimensions>
void 
SkewTransform<TParametersValueType, NDimensions>
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
    jacobian(i, i) = p[2];
  }
  
  
}

#endif

