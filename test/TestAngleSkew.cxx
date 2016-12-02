#include "AngleSkewTransform.h"
#include "SkewTransform.h"
#include "PrintTransform.h"

int main( int argc, char *argv[] )
{
  typedef itk::AngleSkewTransform< double, 2 > TransformType;
  
  TransformType::Pointer trans = TransformType::New();
  
  TransformType::ParametersType param;
  
  param.SetSize(3);
  param[0] = 3.14/4   ;
  param[1] = 1;
  param[2] = 0;
  trans->SetParameters(param);
  
  
  
  printTransform("GroundTruthCat.jpg", "outAngle.png", trans->GetInverseTransform());
}
