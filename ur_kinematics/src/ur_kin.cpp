#include <ur_kinematics/ur_kin.h>

#include <math.h>
#include <stdio.h>


namespace ur_kinematics {

  namespace {
    const double ZERO_THRESH = 0.00000001;
    int SIGN(double x) {
      return (x > 0) - (x < 0);
    }
    const double PI = M_PI;

    //#define UR30_PARAMS
    #ifdef UR30_PARAMS
    const double d1 =  0.2363;
    const double a2 = -0.6370;
    const double a3 = -0.5037;
    const double d4 =  0.2010;
    const double d5 =  0.1593;
    const double d6 =  0.1543;
    #endif

    //#define UR20_PARAMS
    #ifdef UR20_PARAMS
    const double d1 =  0.2363;
    const double a2 = -0.8620;
    const double a3 = -0.7287;
    const double d4 =  0.2010;
    const double d5 =  0.1593;
    const double d6 =  0.1543;
    #endif

    //#define UR16e_PARAMS
    #ifdef UR16e_PARAMS
    const double d1 =  0.1807;
    const double a2 = -0.4784;
    const double a3 = -0.36;
    const double d4 =  0.17415;
    const double d5 =  0.11985;
    const double d6 =  0.11655;
    #endif

    //#define UR10_PARAMS
    #ifdef UR10_PARAMS
    const double d1 =  0.1273;
    const double a2 = -0.612;
    const double a3 = -0.5723;
    const double d4 =  0.163941;
    const double d5 =  0.1157;
    const double d6 =  0.0922;
    #endif

    //#define UR10e_PARAMS
    #ifdef UR10e_PARAMS
    const double d1 =  0.1807;
    const double a2 = -0.6127;
    const double a3 = -0.57155;
    const double d4 =  0.17415;
    const double d5 =  0.11985;
    const double d6 =  0.11655;
    #endif

    //#define UR5_PARAMS
    #ifdef UR5_PARAMS
    const double d1 =  0.089159;
    const double a2 = -0.42500;
    const double a3 = -0.39225;
    const double d4 =  0.10915;
    const double d5 =  0.09465;
    const double d6 =  0.0823;
    #endif

    //#define UR5e_PARAMS
    #ifdef UR5e_PARAMS
    const double d1 =  0.1625;
    const double a2 = -0.425;
    const double a3 = -0.3922;
    const double d4 =  0.1333;
    const double d5 =  0.0997;
    const double d6 =  0.0996;
    #endif

    //#define UR3_PARAMS
    #ifdef UR3_PARAMS
    const double d1 =  0.1519;
    const double a2 = -0.24365;
    const double a3 = -0.21325;
    const double d4 =  0.11235;
    const double d5 =  0.08535;
    const double d6 =  0.0819;
    #endif

    //#define UR3e_PARAMS
    #ifdef UR3e_PARAMS
    const double d1 =  0.15185;
    const double a2 = -0.24355;
    const double a3 = -0.2132;
    const double d4 =  0.13105;
    const double d5 =  0.08535;
    const double d6 =  0.0921;
    #endif
  }

  void forward(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
    double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q); 
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
    *T = c234*c1*s5 - c5*s1; T++;
    *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
    *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
    *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
    *T = c1*c5 + c234*s1*s5; T++;
    *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
    *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
    *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
    *T = -s234*s5; T++;
    *T = -c234*s6 - s234*c5*c6; T++;
    *T = s234*c5*s6 - c234*c6; T++;
    *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  }

  void forward_all(const double* q, double* T1, double* T2, double* T3, 
                                    double* T4, double* T5, double* T6) {
    double s1 = sin(*q), c1 = cos(*q); q++; // q1
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
    q234 += *q; q++; // q4
    double s5 = sin(*q), c5 = cos(*q); q++; // q5
    double s6 = sin(*q), c6 = cos(*q); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);

    if(T1 != NULL) {
      *T1 = c1; T1++;
      *T1 = 0; T1++;
      *T1 = s1; T1++;
      *T1 = 0; T1++;
      *T1 = s1; T1++;
      *T1 = 0; T1++;
      *T1 = -c1; T1++;
      *T1 = 0; T1++;
      *T1 =       0; T1++;
      *T1 = 1; T1++;
      *T1 = 0; T1++;
      *T1 =d1; T1++;
      *T1 =       0; T1++;
      *T1 = 0; T1++;
      *T1 = 0; T1++;
      *T1 = 1; T1++;
    }

    if(T2 != NULL) {
      *T2 = c1*c2; T2++;
      *T2 = -c1*s2; T2++;
      *T2 = s1; T2++;
      *T2 =a2*c1*c2; T2++;
      *T2 = c2*s1; T2++;
      *T2 = -s1*s2; T2++;
      *T2 = -c1; T2++;
      *T2 =a2*c2*s1; T2++;
      *T2 =         s2; T2++;
      *T2 = c2; T2++;
      *T2 = 0; T2++;
      *T2 =   d1 + a2*s2; T2++;
      *T2 =               0; T2++;
      *T2 = 0; T2++;
      *T2 = 0; T2++;
      *T2 =                 1; T2++;
    }

    if(T3 != NULL) {
      *T3 = c23*c1; T3++;
      *T3 = -s23*c1; T3++;
      *T3 = s1; T3++;
      *T3 =c1*(a3*c23 + a2*c2); T3++;
      *T3 = c23*s1; T3++;
      *T3 = -s23*s1; T3++;
      *T3 = -c1; T3++;
      *T3 =s1*(a3*c23 + a2*c2); T3++;
      *T3 =         s23; T3++;
      *T3 = c23; T3++;
      *T3 = 0; T3++;
      *T3 =     d1 + a3*s23 + a2*s2; T3++;
      *T3 =                    0; T3++;
      *T3 = 0; T3++;
      *T3 = 0; T3++;
      *T3 =                                     1; T3++;
    }

    if(T4 != NULL) {
      *T4 = c234*c1; T4++;
      *T4 = s1; T4++;
      *T4 = s234*c1; T4++;
      *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
      *T4 = c234*s1; T4++;
      *T4 = -c1; T4++;
      *T4 = s234*s1; T4++;
      *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
      *T4 =         s234; T4++;
      *T4 = 0; T4++;
      *T4 = -c234; T4++;
      *T4 =                  d1 + a3*s23 + a2*s2; T4++;
      *T4 =                         0; T4++;
      *T4 = 0; T4++;
      *T4 = 0; T4++;
      *T4 =                                                  1; T4++;
    }

    if(T5 != NULL) {
      *T5 = s1*s5 + c234*c1*c5; T5++;
      *T5 = -s234*c1; T5++;
      *T5 = c5*s1 - c234*c1*s5; T5++;
      *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
      *T5 = c234*c5*s1 - c1*s5; T5++;
      *T5 = -s234*s1; T5++;
      *T5 = - c1*c5 - c234*s1*s5; T5++;
      *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
      *T5 =                           s234*c5; T5++;
      *T5 = c234; T5++;
      *T5 = -s234*s5; T5++;
      *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
      *T5 =                                                   0; T5++;
      *T5 = 0; T5++;
      *T5 = 0; T5++;
      *T5 =                                                                                 1; T5++;
    }

    if(T6 != NULL) {
      *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
      *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
      *T6 = c5*s1 - c234*c1*s5; T6++;
      *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
      *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
      *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
      *T6 = - c1*c5 - c234*s1*s5; T6++;
      *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
      *T6 =                                       c234*s6 + s234*c5*c6; T6++;
      *T6 = c234*c6 - s234*c5*s6; T6++;
      *T6 = -s234*s5; T6++;
      *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
      *T6 =                                                                                                   0; T6++;
      *T6 = 0; T6++;
      *T6 = 0; T6++;
      *T6 =                                                                                                                                            1; T6++;
    }
  }

  int inverse(const double* T, double* q_sols, double q6_des) {
    int num_sols = 0;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++; 
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++; 
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;
            q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k]; 
            q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k]; 
            q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6; 
            num_sols++;
          }

        }
      }
    }
    return num_sols;
  }
};


#define IKFAST_HAS_LIBRARY
#include <ur_kinematics/ikfast.h>
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==61);

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

void to_mat44(double * mat4_4, const IkReal* eetrans, const IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        mat4_4[i*4+0] = eerot[i*3+0];
        mat4_4[i*4+1] = eerot[i*3+1];
        mat4_4[i*4+2] = eerot[i*3+2];
        mat4_4[i*4+3] = eetrans[i];
    }
    mat4_4[3*4+0] = 0;
    mat4_4[3*4+1] = 0;
    mat4_4[3*4+2] = 0;
    mat4_4[3*4+3] = 1;
}

void from_mat44(const double * mat4_4, IkReal* eetrans, IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        eerot[i*3+0] = mat4_4[i*4+0];
        eerot[i*3+1] = mat4_4[i*4+1];
        eerot[i*3+2] = mat4_4[i*4+2];
        eetrans[i] = mat4_4[i*4+3];
    }
}


IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
  if(!pfree) return false;

  int n = GetNumJoints();
  double q_sols[8*6];
  double T[16];

  to_mat44(T, eetrans, eerot);

  int num_sols = ur_kinematics::inverse(T, q_sols,pfree[0]);

  std::vector<int> vfree(0);

  for (int i=0; i < num_sols; ++i){
    std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(n);
    for (int j=0; j < n; ++j) vinfos[j].foffset = q_sols[i*n+j];
    solutions.AddSolution(vinfos,vfree);
  }
  return num_sols > 0;
}

IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot)
{
    double T[16];
    ur_kinematics::forward(j,T);
    from_mat44(T,eetrans,eerot);
}

IKFAST_API int GetNumFreeParameters() { return 1; }
IKFAST_API int* GetFreeParameters() { static int freeparams[] = {5}; return freeparams; }
IKFAST_API int GetNumJoints() { return 6; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN

using namespace std;
using namespace ur_kinematics;

int main(int argc, char* argv[])
{
  double q[6] = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
  double* T = new double[16];
  forward(q, T);
  for(int i=0;i<4;i++) {
    for(int j=i*4;j<(i+1)*4;j++)
      printf("%1.3f ", T[j]);
    printf("\n");
  }
  double q_sols[8*6];
  int num_sols;
  num_sols = inverse(T, q_sols);
  for(int i=0;i<num_sols;i++) 
    printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
       q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
  for(int i=0;i<=4;i++)
    printf("%f ", PI/2.0*i);
  printf("\n");
  return 0;
}
#endif
