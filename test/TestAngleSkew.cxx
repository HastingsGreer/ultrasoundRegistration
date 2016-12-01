#include "AngleSkewTransform.h"
#include "SkewTransform.h"
#include "PrintTransform.h"

int main( int argc, char *argv[] )
{
  typedef SkewTransform< double, 2 > TransformType;
  
  TransformType::Pointer trans = TransformType::New();
  
  TransformType::ParametersType param;
  
  param.SetSize(3);
  param[0] = 10   ;
  param[1] = 1;
  param[2] = 0;
  trans->SetFixedParameters(param);
  
  
  
  printTransform("GroundTruthCat.jpg", "outAngle.png", trans.GetPointer());
}
