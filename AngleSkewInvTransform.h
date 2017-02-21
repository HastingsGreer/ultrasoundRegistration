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

#ifndef itkAngleSkewInvTransform_h
#define itkAngleSkewInvTransform_h

#include <itkMatrixOffsetTransformBase.h>
#include <iostream>
namespace itk {

template< 
  typename TParametersValueType = double,
  unsigned int NDimensions=2>
class AngleSkewInvTransform:
  public itk::MatrixOffsetTransformBase< TParametersValueType, NDimensions, NDimensions>
{
public:
  /** Transform by the 2x2 matrix
       
      [cos(theta) 0     ]
      [sin(theta) vscale] */
     
  /** Standard typedefs   */
  typedef AngleSkewInvTransform Self;
  typedef itk::MatrixOffsetTransformBase< TParametersValueType,
                                     NDimensions,
                                     NDimensions >  Superclass;

  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  /** Run-time type information (and related methods).   */
  itkTypeMacro(AngleSkewInvTransform, itk::MatrixOffsetTransformBase);

  /** New macro for creation of through a Smart Pointer   */
  itkNewMacro(Self);

  /** Dimension of the domain space. */
  itkStaticConstMacro(InputSpaceDimension, unsigned int, NDimensions);
  itkStaticConstMacro(OutputSpaceDimension, unsigned int, NDimensions);
  itkStaticConstMacro(SpaceDimension, unsigned int, NDimensions);
  itkStaticConstMacro( ParametersDimension, unsigned int,
                       3 );

  /** Parameters Type   */
  typedef typename Superclass::ParametersType            ParametersType;
  typedef typename Superclass::FixedParametersType       FixedParametersType;
  typedef typename Superclass::JacobianType              JacobianType;
  typedef typename Superclass::ScalarType                ScalarType;
  typedef typename Superclass::InputPointType            InputPointType;
  typedef typename Superclass::OutputPointType           OutputPointType;
  typedef typename Superclass::InputVectorType           InputVectorType;
  typedef typename Superclass::OutputVectorType          OutputVectorType;
  typedef typename Superclass::InputVnlVectorType        InputVnlVectorType;
  typedef typename Superclass::OutputVnlVectorType       OutputVnlVectorType;
  typedef typename Superclass::InputCovariantVectorType  InputCovariantVectorType;
  typedef typename Superclass::OutputCovariantVectorType OutputCovariantVectorType;
  typedef typename Superclass::MatrixType                MatrixType;
  typedef typename Superclass::InverseMatrixType         InverseMatrixType;
  typedef typename Superclass::CenterType                CenterType;
  typedef typename Superclass::OffsetType                OffsetType;
  typedef typename Superclass::TranslationType           TranslationType;
  
  /** Base inverse transform type. This type should not be changed to the
   * concrete inverse transform type or inheritance would be lost.*/
  typedef typename Superclass::InverseTransformBaseType InverseTransformBaseType;
  typedef typename InverseTransformBaseType::Pointer    InverseTransformBasePointer;
  
  /** Set the transformation from a container of parameters
   * The parameters form the right column of the transform matrix
   *
   * \sa Transform::SetParameters()
   * \sa Transform::SetFixedParameters() */
  virtual void SetParameters(const ParametersType & parameters) ITK_OVERRIDE;

  /** Get the parameters that uniquely define the transform
   * This is typically used by optimizers.
   *
   * \sa Transform::GetParameters()
   * \sa Transform::GetFixedParameters() */
  virtual const ParametersType & GetParameters() const ITK_OVERRIDE;

  /** Compute the Jacobian Matrix of the transformation at one point,
   *  allowing for thread-safety. */
  virtual void ComputeJacobianWithRespectToParameters(const InputPointType  & p, JacobianType & jacobian) const ITK_OVERRIDE;
  

  AngleSkewInvTransform();

  virtual ~AngleSkewInvTransform();

private:
  //ITK_DISALLOW_COPY_AND_ASSIGN(SkewTransform);
};

} //namespace itk
#include "AngleSkewInvTransform.hxx"


#endif
