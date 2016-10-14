/*=========================================================================
 *
 *  Copyright Insight Software Consortium
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

// Software Guide : BeginLatex
//
// Given the numerous parameters involved in tuning a registration method for
// a particular application, it is not uncommon for a registration process to
// run for several minutes and still produce a useless result.  To avoid
// this situation it is quite helpful to track the evolution of the
// registration as it progresses. The following section illustrates the
// mechanisms provided in ITK for monitoring the activity of the
// ImageRegistrationMethodv4 class.
//
// Insight implements the \emph{Observer/Command} design pattern
// \cite{Gamma1995}.
// The classes involved in this implementation are the \doxygen{Object},
// \doxygen{Command} and \doxygen{EventObject} classes. The Object
// is the base class of most ITK objects. This class maintains a linked
// list of pointers to event observers. The role of observers is played by
// the Command class.  Observers register themselves with an
// Object, declaring that they are interested in receiving
// notification when a particular event happens. A set of events is
// represented by the hierarchy of the Event class. Typical events
// are \code{Start}, \code{End}, \code{Progress} and \code{Iteration}.
//
// Registration is controlled by an \doxygen{Optimizer}, which generally
// executes an iterative process. Most Optimizer classes invoke an
// \doxygen{IterationEvent} at the end of each iteration. When an event is
// invoked by an object, this object goes through its list of registered
// observers (Commands) and checks whether any one of them has expressed
// interest in the current event type. Whenever such an observer is found,
// its corresponding \code{Execute()} method is invoked.  In this context,
// \code{Execute()} methods should be considered \emph{callbacks}.  As such,
// some of the common sense rules of callbacks should be respected.  For
// example, \code{Execute()} methods should not perform heavy computational
// tasks.  They are expected to execute rapidly, for example, printing out a
// message or updating a value in a GUI.
//
// \index{itk::ImageRegistrationMethod!Monitoring}
//
//
// Software Guide : EndLatex

#include "SkewTransform.h"

#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkGradientDescentOptimizerv4.h"
#include "itkRegistrationParameterScalesFromPhysicalShift.h"

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
  itkNewMacro( Self );
protected:
  CommandIterationUpdate() {};
public:
  typedef itk::GradientDescentOptimizerv4 OptimizerType;
  typedef const OptimizerType *                              OptimizerPointer;
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
    
    }

};


int main( int argc, char *argv[] )
{
  if( argc < 4 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile ";
    std::cerr << "outputImagefile " << std::endl;
    return EXIT_FAILURE;
    }

  const    unsigned int    Dimension = 2;
  typedef  float           PixelType;

  typedef itk::Image< PixelType, Dimension >  FixedImageType;
  typedef itk::Image< PixelType, Dimension >  MovingImageType;

  typedef itk::AffineTransform< double, Dimension >      TransformType;

  typedef itk::GradientDescentOptimizerv4 OptimizerType;

  typedef itk::ImageRegistrationMethodv4<
                                    FixedImageType,
                                    MovingImageType,
                                    TransformType     >  RegistrationType;

  typedef itk::MeanSquaresImageToImageMetricv4<
                                      FixedImageType,
                                      MovingImageType >  MetricType;

  OptimizerType::Pointer      optimizer     = OptimizerType::New();
  RegistrationType::Pointer   registration  = RegistrationType::New();

  registration->SetOptimizer(     optimizer     );

  MetricType::Pointer         metric        = MetricType::New();

  registration->SetMetric( metric  );

  typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
  typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

  FixedImageReaderType::Pointer  fixedImageReader  = FixedImageReaderType::New();
  MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] );

  registration->SetFixedImage(    fixedImageReader->GetOutput()    );
  registration->SetMovingImage(   movingImageReader->GetOutput()   );

  // Set parameters of the optimizer
  //
  //optimizer->SetLearningRate( 1 );
  //optimizer->SetMinimumStepLength( 0.001 );
  //optimizer->SetRelaxationFactor( 0.75 );
  //optimizer->SetNumberOfIterations( 400 );
  typedef itk::RegistrationParameterScalesFromPhysicalShift < MetricType > RegistrationParameterScalesType; 
  
  RegistrationParameterScalesType::Pointer shiftScaleEstimator =  RegistrationParameterScalesType::New();
  shiftScaleEstimator->SetMetric(metric);
  
  
  optimizer->SetScalesEstimator(shiftScaleEstimator);
  optimizer->SetMaximumStepSizeInPhysicalUnits(.03);
  
  // Initialize the transform
  
  TransformType::Pointer initialTransform = TransformType::New();
  initialTransform->SetIdentity();
  registration->SetInitialTransform(initialTransform);

  // One level registration process without shrinking and smoothing.
  //
  const unsigned int numberOfLevels = 1;

  RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
  shrinkFactorsPerLevel.SetSize( 1 );
  shrinkFactorsPerLevel[0] = 1;

  RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
  smoothingSigmasPerLevel.SetSize( 1 );
  smoothingSigmasPerLevel[0] = 0;

  registration->SetNumberOfLevels ( numberOfLevels );
  registration->SetSmoothingSigmasPerLevel( smoothingSigmasPerLevel );
  registration->SetShrinkFactorsPerLevel( shrinkFactorsPerLevel );
  
  // Lock transform to only optimize skew
  
  RegistrationType::OptimizerWeightsType weights(6);
  weights.Fill(0);
  weights[1] = 1;
  weights[4] = 1;
  std::cout << weights << std::endl;
  registration->SetOptimizerWeights(weights);
  
  CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
  
  optimizer->AddObserver( itk::IterationEvent(), observer );
  
  try
    {
    registration->Update();
    std::cout << "Optimizer stop condition: "
              << registration->GetOptimizer()->GetStopConditionDescription()
              << std::endl;
    }
  catch( itk::ExceptionObject & err )
    {
    std::cout << "ExceptionObject caught !" << std::endl;
    std::cout << err << std::endl;
    return EXIT_FAILURE;
    }
  // Software Guide : EndCodeSnippet


  //  Software Guide : BeginLatex
  //
  //  The registration process is applied to the following images in \code{Examples/Data}:
  //
  //  \begin{itemize}
  //  \item \code{BrainProtonDensitySliceBorder20.png}
  //  \item \code{BrainProtonDensitySliceShifted13x17y.png}
  //  \end{itemize}
  //
  //  It produces the following output.
  //
  //  \begin{verbatim}
  //   0 = 4499.45 : [2.9286959512455857, 2.7244705953923805]
  //   1 = 3860.84 : [6.135143776902402, 5.115849348610004]
  //   2 = 3508.02 : [8.822660051952475, 8.078492808653918]
  //   3 = 3117.31 : [10.968558473732326, 11.454158663474674]
  //   4 = 2125.43 : [13.105290365964755, 14.835634202454191]
  //   5 = 911.308 : [12.75173580401588, 18.819978461140323]
  //   6 = 741.417 : [13.139053510563274, 16.857840597942413]
  //   7 = 16.8918 : [12.356787624301035, 17.480785285045815]
  //   8 = 233.714 : [12.79212443526829, 17.234854683011704]
  //   9 = 39.8027 : [13.167510875734614, 16.904574468172815]
  //   10 = 16.5731 : [12.938831371165355, 17.005597654570586]
  //   11 = 1.68763 : [13.063495692092735, 16.996443033457986]
  //   12 = 1.79437 : [13.001061362657559, 16.999307384689935]
  //   13 = 0.000762481 : [12.945418587211314, 17.0277701944711]
  //   14 = 1.74802 : [12.974454390534774, 17.01621663980765]
  //   15 = 0.430253 : [13.002439510423766, 17.002309966416835]
  //   16 = 0.00531816 : [12.989877586882951, 16.99301810428082]
  //   17 = 0.0721346 : [12.996759235073881, 16.996716492365685]
  //   18 = 0.00996773 : [13.00288423694971, 17.00156618393022]
  //   19 = 0.00516378 : [12.99928608126834, 17.000045636412015]
  //   20 = 0.000228075 : [13.00123653240422, 16.999943471681494]
  //  \end{verbatim}
  //  You can verify from the code in the \code{Execute()} method that the first
  //  column is the iteration number, the second column is the metric value and
  //  the third and fourth columns are the parameters of the transform, which
  //  is a $2D$ translation transform in this case. By tracking these values as
  //  the registration progresses, you will be able to determine whether the
  //  optimizer is advancing in the right direction and whether the step-length
  //  is reasonable or not.  That will allow you to interrupt the registration
  //  process and fine-tune parameters without having to wait until the
  //  optimizer stops by itself.
  //
  //  Software Guide : EndLatex


  TransformType::ParametersType finalParameters =
                            registration->GetOutput()->Get()->GetParameters();

  const double TranslationAlongX = finalParameters[0];
  const double TranslationAlongY = finalParameters[1];

  const unsigned int numberOfIterations = optimizer->GetCurrentIteration();

  const double bestValue = optimizer->GetValue();

  std::cout << "Registration done !" << std::endl;
  std::cout << "Number of iterations = " << numberOfIterations << std::endl;
  std::cout << "Translation along X  = " << TranslationAlongX << std::endl;
  std::cout << "Translation along Y  = " << TranslationAlongY << std::endl;
  std::cout << "Optimal metric value = " << bestValue << std::endl;


  // Prepare the resampling filter in order to map the moving image.
  //
  typedef itk::ResampleImageFilter<
                            MovingImageType,
                            FixedImageType >    ResampleFilterType;

  ResampleFilterType::Pointer resample = ResampleFilterType::New();

  resample->SetTransform( registration->GetTransform() );
  resample->SetInput( movingImageReader->GetOutput() );

  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  resample->SetSize(    fixedImage->GetLargestPossibleRegion().GetSize() );
  resample->SetOutputOrigin(  fixedImage->GetOrigin() );
  resample->SetOutputSpacing( fixedImage->GetSpacing() );
  resample->SetOutputDirection( fixedImage->GetDirection() );
  resample->SetDefaultPixelValue( 100 );


  // Prepare a writer and caster filters to send the resampled moving image to
  // a file
  //
  typedef  unsigned char  OutputPixelType;

  typedef itk::Image< OutputPixelType, Dimension > OutputImageType;

  typedef itk::CastImageFilter<
                        FixedImageType,
                        OutputImageType > CastFilterType;

  typedef itk::ImageFileWriter< OutputImageType >  WriterType;

  WriterType::Pointer      writer =  WriterType::New();
  CastFilterType::Pointer  caster =  CastFilterType::New();


  writer->SetFileName( argv[3] );

  caster->SetInput( resample->GetOutput() );
  writer->SetInput( caster->GetOutput()   );
  writer->Update();


  return EXIT_SUCCESS;
}
