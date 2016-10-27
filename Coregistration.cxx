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

#include "SkewTransform.h"
#include "PrintTransform.h"

#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkGradientDescentOptimizerv4.h"
#include "itkRegistrationParameterScalesFromPhysicalShift.h"

#include "itkLBFGSBOptimizerv4.h"
#include "itkRigid2DTransform.h"
#include "itkCompositeTransform.h"

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
  typedef itk::LBFGSBOptimizerv4 OptimizerType;
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
    //std::cout << optimizer->GetTransform() << std::endl;
    
    }

};

itk::CompositeTransform<double, 2>::Pointer makeInitialTransform(){
  const    unsigned int    Dimension = 2;
  typedef itk::CompositeTransform<double, Dimension> CompositeType;
  typedef itk::Rigid2DTransform<double>              RigidType;
  typedef SkewTransform<double, 2>                   SkewType;



  CompositeType::Pointer initialTransform = CompositeType::New();
  SkewType::Pointer skA = SkewType::New();
  skA->SetIdentity();
  SkewType::Pointer skB = SkewType::New();
  skB->SetIdentity();
/*  TransformType::ParametersType p;
  p.SetSize(2);
  p[0] = 0;
  p[1] = 0.7;
  initialTransform->SetParameters(p); */
  
  SkewType::FixedParametersType fp;
  fp.SetSize(2);
  fp[0] = -20   ;
  fp[1] = 20;
  skA->SetFixedParameters(fp);
  skB->SetFixedParameters(fp);
  
  RigidType::Pointer R = RigidType::New();
  
  RigidType::FixedParametersType Rfp;
  Rfp.SetSize(2);
  Rfp[0] = 20   ;
  Rfp[1] = 20;

  R->SetFixedParameters(Rfp);
  
  RigidType::ParametersType Rp;
  Rp.SetSize(3);
  Rp[0] = -3.1415 / 2;
  Rp[1] = 0;
  Rp[1] = 0;
  
  
  R->SetParameters(Rp);
  
  initialTransform->AddTransform(skA);
  initialTransform->AddTransform(R);
  initialTransform->AddTransform(skB);
  initialTransform->SetNthTransformToOptimizeOff(1);
  
  return initialTransform;
}

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

  typedef itk::CompositeTransform< double, Dimension >      TransformType;

  typedef itk::LBFGSBOptimizerv4 OptimizerType;

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
  optimizer->SetNumberOfIterations( 400 );
  //typedef itk::RegistrationParameterScalesFromPhysicalShift < MetricType > RegistrationParameterScalesType; 
  
  //RegistrationParameterScalesType::Pointer shiftScaleEstimator =  RegistrationParameterScalesType::New();
  //shiftScaleEstimator->SetMetric(metric);
  
  
  //optimizer->SetScalesEstimator(shiftScaleEstimator);
  //optimizer->SetMaximumStepSizeInPhysicalUnits(.03);
  
  registration->SetInitialTransform(makeInitialTransform());
 
 // One level registration process without shrinking and smoothing.
  //
  const unsigned int numberOfLevels = 1;

  RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
  smoothingSigmasPerLevel.SetSize( 1 );
  smoothingSigmasPerLevel[0] = 0;

  registration->SetNumberOfLevels ( numberOfLevels );
  registration->SetSmoothingSigmasPerLevel( smoothingSigmasPerLevel );

  
  // Lock transform to only optimize skew
  /*
  RegistrationType::OptimizerWeightsType weights(6);
  weights.Fill(0);
  weights[1] = 1;
  weights[4] = 1;
  std::cout << weights << std::endl;
  registration->SetOptimizerWeights(weights);
  */
  
  /* copied from example 12*/
  const unsigned int numParameters = 4;
  OptimizerType::BoundSelectionType boundSelect( numParameters );
  OptimizerType::BoundValueType upperBound( numParameters );
  OptimizerType::BoundValueType lowerBound( numParameters );
  boundSelect.Fill( 0 );
  upperBound.Fill( 2 );
  lowerBound.Fill( -2 );
  optimizer->SetBoundSelection( boundSelect );
  optimizer->SetUpperBound( upperBound );
  optimizer->SetLowerBound( lowerBound );
  /*end copypasta*/
  
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
  
  
  printTransform(argv[2], argv[3], registration->GetTransform());
  
  
  return EXIT_SUCCESS;
}
