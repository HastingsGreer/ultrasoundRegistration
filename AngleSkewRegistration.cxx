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

#include "AngleSkewTransform.h"
#include "AngleSkewInvTransform.h"
#include "PrintTransform.h"

#include "IterationUpdate.h"
#include "ConvertTransform.h"


#include <itkImageRegistrationMethodv4.h>
#include <itkTranslationTransform.h>
#include <itkMeanSquaresImageToImageMetricv4.h>
#include <itkGradientDescentOptimizerv4.h>
#include <itkRegistrationParameterScalesFromPhysicalShift.h>
#include <itkLBFGSBOptimizerv4.h>
#include <itkAffineTransform.h>
#include <itkCompositeTransform.h>

#include <itkImageFileReader.h> 

#include <itkResampleImageFilter.h>
#include <itkCastImageFilter.h>

#include <itkCommand.h>
#include <string>

itk::CompositeTransform<double, 3>::Pointer makeInitialTransform(double width)
{
  const    unsigned int    Dimension = 3;
  typedef itk::CompositeTransform< double, Dimension >       CompositeType;
  typedef itk::AngleSkewInvTransform< double, Dimension >  RealToMovingType;
  typedef itk::AngleSkewTransform< double, Dimension >     FixedToRealType;

  CompositeType::Pointer initialTransform = CompositeType::New();
  
  RealToMovingType::Pointer A = RealToMovingType::New();
  A->SetIdentity();
  RealToMovingType::ParametersType pA;
  pA.SetSize( 2 );
  pA[0] = -3.14 / 6;
  pA[1] = 2.0;
  A->SetParameters(pA);
  
  FixedToRealType::Pointer B = FixedToRealType::New();
  B->SetIdentity();
  FixedToRealType::ParametersType pB;
  pB.SetSize( 2 );
  pB[0] = 3.14 / 6;
  pB[1] = 2.0;
  B->SetParameters(pB);
  
  RealToMovingType::FixedParametersType fp;
  fp.SetSize( 3 );
  fp[0] = width/2;
  fp[1] = 0;
  fp[2] = 0;
  A->SetFixedParameters( fp );
  B->SetFixedParameters( fp );
  
  initialTransform->AddTransform( A );
  initialTransform->AddTransform( B );
  
  //Fixed(x)  $= Moving(A(B(x))
  
  return initialTransform;
}

int main( int argc, char *argv[] )
{
  if( argc < 4 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile ";
    std::cerr << "outputImagefile fixedVolumeFile movingVolumeFile" 
      << std::endl;
    return EXIT_FAILURE;
    }
  
  const    unsigned int    Dimension = 3;
  typedef double                                             PixelType;

  typedef itk::Image< PixelType, Dimension >                 FixedImageType;
  typedef itk::Image< PixelType, Dimension >                 MovingImageType;

  typedef itk::CompositeTransform< double, Dimension >       TransformType;
  typedef itk::LBFGSBOptimizerv4                             OptimizerType;
  typedef itk::ImageRegistrationMethodv4< FixedImageType,
    MovingImageType, TransformType >                         RegistrationType;

  typedef itk::MeanSquaresImageToImageMetricv4< FixedImageType,
    MovingImageType >                                        MetricType;

  OptimizerType::Pointer      optimizer     = OptimizerType::New();
  RegistrationType::Pointer   registration  = RegistrationType::New();

  registration->SetOptimizer( optimizer );

  MetricType::Pointer         metric        = MetricType::New();

  registration->SetMetric( metric );

  typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
  typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

  FixedImageReaderType::Pointer  fixedImageReader  = 
    FixedImageReaderType::New();
  MovingImageReaderType::Pointer movingImageReader = 
    MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] );
  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();
  fixedImageReader->Update();
  
  registration->SetFixedImage( fixedImage );
  registration->SetMovingImage( movingImageReader->GetOutput() );
  double width = fixedImage->GetLargestPossibleRegion().GetSize()[0];
  std::cout << "width:" << width <<std::endl;

  
  TransformType::Pointer initialTransform = makeInitialTransform( width );
  std::cout << initialTransform->GetParameters() << std::endl;
  
  
  registration->SetInitialTransform( initialTransform );
  
  // One level registration process without shrinking and smoothing.
  //
  const unsigned int numberOfLevels = 1;

  RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
  smoothingSigmasPerLevel.SetSize( 1 );
  smoothingSigmasPerLevel[0] = 0;

  registration->SetNumberOfLevels ( numberOfLevels );
  registration->SetSmoothingSigmasPerLevel( smoothingSigmasPerLevel );
  
  /*bounds for the optimizer */
  const unsigned int numParameters = 4;
  OptimizerType::BoundSelectionType boundSelect( numParameters );
  OptimizerType::BoundValueType upperBound( numParameters );
  OptimizerType::BoundValueType lowerBound( numParameters );
  boundSelect.Fill( 2 ); //2 is a flag for "respect bounds"
  upperBound[0] = 3.14 / 3;
  upperBound[1] = 4.5;
  lowerBound[0] = 3.14 / 6;
  lowerBound[1] = 1.5;
  upperBound[2] = -3.14 / 6;
  upperBound[3] = 4.5;
  lowerBound[2] = -3.14 / 3;
  lowerBound[3] = 1.5;
  optimizer->SetBoundSelection( boundSelect );
  optimizer->SetUpperBound( upperBound );
  optimizer->SetLowerBound( lowerBound );
  
  
  CommandIterationUpdate< RegistrationType >::Pointer observer = 
    CommandIterationUpdate< RegistrationType >::New();
  observer->SetInfile( argv[2] );
  observer->SetOutfile( argv[3] );
  observer->SetRegistration( registration );
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

  const double Angle = finalParameters[0];
  const double YScale = finalParameters[1];

  const unsigned int numberOfIterations = optimizer->GetCurrentIteration();

  const double bestValue = optimizer->GetValue();

  std::cout << "Registration done !" << std::endl;
  std::cout << "Number of iterations = " << numberOfIterations << std::endl;
  std::cout << "Angle  = " << Angle << std::endl;
  std::cout << "Scale along Y  = " << YScale << std::endl;
  std::cout << "Optimal metric value = " << bestValue << std::endl;
  
  printTransform<3>( argv[2], argv[3], registration->GetTransform() );
  
  std::string movingOutput = std::string("moving") + argv[3];
  std::string fixedOutput = std::string("fixed") + argv[3];
  printTransform<3>(argv[2], movingOutput.c_str(), registration->GetTransform()->GetNthTransform(0));
  
  printTransform<3>(argv[1], fixedOutput.c_str(), registration->GetTransform()->GetNthTransform(1)->GetInverseTransform());

  return EXIT_SUCCESS;
}
