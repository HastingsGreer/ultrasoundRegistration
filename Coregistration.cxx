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

#include "SkewTransform.h"
#include "PrintTransform.h"
#include "IterationUpdate.h"

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


itk::CompositeTransform<double, 2>::Pointer makeInitialTransform(){
  const    unsigned int    Dimension = 2;
  typedef itk::CompositeTransform<double, Dimension> CompositeType;
  typedef itk::Rigid2DTransform<double>              RigidType;
  typedef itk::SkewTransform<double, 2>              SkewType;
  typedef itk::TranslationTransform<double, 2>       TranslationType;

  CompositeType::Pointer initialTransform = CompositeType::New();
  SkewType::Pointer skA = SkewType::New();
  skA->SetIdentity();
  SkewType::Pointer skB = SkewType::New();
  skB->SetIdentity();
  
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
  Rp[1] = 20;
  Rp[2] = 20;
  
  
  R->SetParameters(Rp);
  
  TranslationType::Pointer T = TranslationType::New();
  T->SetIdentity();
  initialTransform->AddTransform(T);
  initialTransform->AddTransform(skA);
  initialTransform->AddTransform(R);
  initialTransform->AddTransform(skB);



  //initialTransform->SetNthTransformToOptimizeOff(1);
  initialTransform->SetNthTransformToOptimizeOff(2);
  //initialTransform->SetNthTransformToOptimizeOff(3);
  
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
  
  optimizer->SetNumberOfIterations( 400 );
  
  registration->SetInitialTransform(makeInitialTransform());
 
 // One level registration process without shrinking and smoothing.
  //
  const unsigned int numberOfLevels = 1;

  RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
  smoothingSigmasPerLevel.SetSize( 1 );
  smoothingSigmasPerLevel[0] = 0;

  registration->SetNumberOfLevels ( numberOfLevels );
  registration->SetSmoothingSigmasPerLevel( smoothingSigmasPerLevel );

  /* copied from example 12*/
  const unsigned int numParameters = 9;
  OptimizerType::BoundSelectionType boundSelect( numParameters );
  OptimizerType::BoundValueType upperBound( numParameters );
  OptimizerType::BoundValueType lowerBound( numParameters );
  boundSelect.Fill(2 );
  upperBound[0] = .2;
  upperBound[1] = 1;
  upperBound[2] = .2;
  upperBound[3] = 1;
  upperBound[4] = 3;
  upperBound[5] = 3;
  lowerBound[0] = -.2;
  lowerBound[1] = 1;
  lowerBound[2] = -.2;
  lowerBound[3] =1;
  lowerBound[4] = -3;
  lowerBound[5] = -3;

  optimizer->SetBoundSelection( boundSelect );
  optimizer->SetUpperBound( upperBound );
  optimizer->SetLowerBound( lowerBound );
  /*end copypasta*/
  
  CommandIterationUpdate<RegistrationType>::Pointer observer = CommandIterationUpdate<RegistrationType>::New();
  observer->SetInfile(argv[2]);
  observer->SetOutfile(argv[3]);
  observer->SetRegistration(registration);
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
