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

template<typename RegistrationType>
class CommandIterationUpdate : public itk::Command
{
public:

  typedef  CommandIterationUpdate   Self;
  typedef  itk::Command             Superclass;
  typedef itk::SmartPointer<Self>  Pointer;
  
  itkNewMacro( Self );
  itkSetMacro(Infile, char*);
  itkSetMacro(Outfile, char*);
  itkSetMacro(Registration, typename RegistrationType::Pointer);
  
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
    printTransform(this->m_Infile, this->m_Outfile, this->m_Registration->GetTransform());
    
    }
private: 
  typename RegistrationType::Pointer m_Registration;
  const char * m_Infile;
  const char * m_Outfile;

};
#endif
