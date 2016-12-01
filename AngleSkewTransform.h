#ifndef itkAngleSkewTransform_h
#define itkAngleSkewTransform_h

#include "itkMatrixOffsetTransformBase.h"
#include <iostream>

template< 
  typename TParametersValueType = double,
  unsigned int NDimensions=2>
class AngleSkewTransform:
  public itk::MatrixOffsetTransformBase< TParametersValueType, NDimensions, NDimensions>
{
public:
  /** Transform by the 2x2 matrix
       
      [cos(theta) 0     ]
      [sin(theta) vscale] */
     
  /** Standard typedefs   */
  typedef AngleSkewTransform Self;
  typedef itk::MatrixOffsetTransformBase< TParametersValueType,
                                     NDimensions,
                                     NDimensions >  Superclass;

  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  /** Run-time type information (and related methods).   */
  itkTypeMacro(SkewTransform, itk::MatrixOffsetTransformBase);

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
   * The parameters form the right column of the transform matrix
   *
   * \sa Transform::GetParameters()
   * \sa Transform::GetFixedParameters() */
  virtual const ParametersType & GetParameters() const ITK_OVERRIDE;

  /** Compute the Jacobian Matrix of the transformation at one point,
   *  allowing for thread-safety. */
  virtual void ComputeJacobianWithRespectToParameters(const InputPointType  & p, JacobianType & jacobian) const ITK_OVERRIDE;
  

  AngleSkewTransform();

  virtual ~AngleSkewTransform();

private:
  //ITK_DISALLOW_COPY_AND_ASSIGN(SkewTransform);
};
#include "AngleSkewTransform.hxx"


#endif