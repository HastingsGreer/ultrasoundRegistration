#ifndef PrintTransform_cxx
#define PrintTransform_cxx

#include "PrintTransform.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"

void printTransform(const char * inputFilename, const char * outputFilename, itk::Transform<double, 2, 2>::ConstPointer transform)
{
  const    unsigned int    Dimension = 2;
  typedef  float           PixelType;

  typedef itk::Image< PixelType, Dimension > ImageType;
  typedef itk::ImageFileReader< ImageType  > ImageReaderType;
  
  ImageReaderType::Pointer  imageReader  = ImageReaderType::New();

  imageReader->SetFileName(inputFilename);
  imageReader->Update();
  
  typedef itk::ResampleImageFilter<
                            ImageType,
                            ImageType >    ResampleFilterType;

  ResampleFilterType::Pointer resample = ResampleFilterType::New();

  resample->SetTransform(transform);
  ImageType::Pointer image = imageReader->GetOutput();
  resample->SetInput(image);

  

  resample->SetSize(    image->GetLargestPossibleRegion().GetSize() );
  resample->SetOutputOrigin(  image->GetOrigin() );
  resample->SetOutputSpacing( image->GetSpacing() );
  resample->SetOutputDirection( image->GetDirection() );
  resample->SetDefaultPixelValue( 100 );


  // Prepare a writer and caster filters to send the resampled moving image to
  // a file
  //
  typedef  unsigned char  OutputPixelType;

  typedef itk::Image< OutputPixelType, Dimension > OutputImageType;

  typedef itk::CastImageFilter<
                        ImageType,
                        OutputImageType > CastFilterType;

  typedef itk::ImageFileWriter< OutputImageType >  WriterType;

  WriterType::Pointer      writer =  WriterType::New();
  CastFilterType::Pointer  caster =  CastFilterType::New();


  writer->SetFileName( outputFilename );

  caster->SetInput( resample->GetOutput() );
  writer->SetInput( caster->GetOutput()   );
  writer->Update();
}

#endif
