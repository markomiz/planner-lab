struct point2d
{
  float x;
  float y;
};
struct pose2d 
{
  point2d x;
  float theta;

  pose2d(float xx,float yy,float th)
  {
    x.x = xx;
    x.y = yy;
    theta = th;
  };
};
struct ksigns {
  int l[3];
  ksigns(int l1, int l2, int l3){

  l[0] = l1;
  l[1] = l2;
  l[2] = l3;

  };
};
struct dubins_params
{
  float s[3];
  float L;
  ksigns k = ksigns(0,0,0);
  bool valid;
};
struct transformedVars
{
  float th0;
  float th1;
  float lambda;
};
