#ifndef IterationUpdate_h
#define IterationUpdate_h

#include "PrintTransform.h"

#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkGradientDescentOptimizerv4.h"
#include "itkRegistrationParameterScalesFromPhysicalShift.h"

#include "itkLBFGSBOptimizerv4.h"
#include "itkAffineTransform.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"

#include "itkCommand.h"


class CommandIterationUpdate : public itk::Command
{
public:

  typedef  CommandIterationUpdate   Self;
  typedef  itk::Command             Superclass;
  typedef itk::SmartPointer<Self>  Pointer;
  
  typedef itk::Transform<double, 2, 2>::ConstPointer TransformCPtr;
  
  itkNewMacro( Self );
  itkSetMacro(infile, char*);
  itkSetMacro(outfile, char*);
  itkSetMacro(transform_ptr, TransformCPtr);
  
protected:
  CommandIterationUpdate() {};
public:
  typedef itk::LBFGSBOptimizerv4 OptimizerType;
  typedef const OptimizerType * OptimizerPointer;
  void Execute(itk::Object *caller,
               const itk::EventObject & event) ITK_OVERRIDE
    {
    Execute( (const itk::Object *)caller, event);
    }
  void Execute(const itk::Object * object,
               const itk::EventObject & event) ITK_OVERRIDE
    {
    
    OptimizerPointer optimizer =
                         static_cast< OptimizerPointer >( object );
    
    if( ! itk::IterationEvent().CheckEvent( &event ) )
      {
      return;
      }
    std::cout << optimizer->GetCurrentIteration() << " = ";
    std::cout << optimizer->GetValue() << " : ";
    std::cout << optimizer->GetCurrentPosition() << std::endl;
    //std::cout << optimizer->GetTransform() << std::endl;
    printTransform(this->m_infile, this->m_outfile, this->m_transform_ptr);
    
    }
private: 
  TransformCPtr m_transform_ptr;
  const char * m_infile;
  const char * m_outfile;

};
#endif
