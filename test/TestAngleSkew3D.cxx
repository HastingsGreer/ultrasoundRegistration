#include "AngleSkewTransform.h"
#include "AngleSkewInvTransform.h"
#include "PrintTransform.h"

#include <itkCompositeTransform.h>

int main( int argc, char *argv[] )
{
  /*
  typedef itk::AngleSkewInvTransform< double, 2 > TransformType;
  
  TransformType::Pointer trans = TransformType::New();
  
  TransformType::ParametersType param;
  
  param.SetSize(2);
  param[0] = .01   ;
  param[1] = 1.05;
  trans->SetParameters(param);
  
  std::cout << trans->GetParameters() << std::endl;
  
  
  
  printTransform<2>("GroundTruthCat.jpg", "outAngle.png", trans->GetInverseTransform());
  */
  
  const    unsigned int    Dimension = 3;
  typedef itk::CompositeTransform<double, Dimension>       CompositeType;
  typedef itk::AngleSkewInvTransform< double, Dimension >  MovingToRealType;
  typedef itk::AngleSkewTransform< double, Dimension >     RealToFixedType;
  float width = 800; //REMOVE ME 
  CompositeType::Pointer initialTransform = CompositeType::New();
  
  MovingToRealType::Pointer A = MovingToRealType::New();
  A->SetIdentity();
  MovingToRealType::ParametersType pA;
  pA.SetSize(2);
  pA[0] = -3.14 / 6;
  pA[1] = 2.0;
  A->SetParameters(pA);
  
  RealToFixedType::Pointer B = RealToFixedType::New();
  B->SetIdentity();
  RealToFixedType::ParametersType pB;
  pB.SetSize(2);
  pB[0] = 3.14 / 6;
  pB[1] = 2.0;
  B->SetParameters(pB);
  
  MovingToRealType::FixedParametersType fp;
  fp.SetSize(2);
  fp[0] = width/2;
  fp[1] = 0;
  A->SetFixedParameters(fp);
  B->SetFixedParameters(fp);
  
  initialTransform->AddTransform(A);
  initialTransform->AddTransform(B);
  
  printTransform<3>("Dino1Raw.mha", "FixedDino1.mha", B->GetInverseTransform());
  printTransform<3>("Dino2Raw.mha", "MovingDino2.mha", A);
  printTransform<3>("Dino2Raw.mha", "Dino2OverlaidDino1.mha", initialTransform);
  
}
