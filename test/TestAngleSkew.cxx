#include "AngleSkewTransform.h"
#include "AngleSkewInvTransform.h"
#include "SkewTransform.h"
#include "PrintTransform.h"

int main( int argc, char *argv[] )
{
  typedef itk::AngleSkewInvTransform< double, 2 > TransformType;
  
  TransformType::Pointer trans = TransformType::New();
  
  TransformType::ParametersType param;
  
  param.SetSize(2);
  param[0] = .01   ;
  param[1] = 1.05;
  trans->SetParameters(param);
  
  std::cout << trans->GetParameters() << std::endl;
  
  
  
  printTransform("GroundTruthCat.jpg", "outAngle.png", trans->GetInverseTransform());
}
