

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.03) // 5D IK has some crazy degenerate cases, but can rely on jacobian refinment to make better, just need good starting point
#endif


inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2Simple(float fy, float fx) {
    return atan2f(fy,fx);
}
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2Simple(double fy, double fx) {
    return atan2(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

template <typename T>
struct CheckValue
{
    T value;
    bool valid;
};

template <typename T>
inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
{
    CheckValue<T> ret;
    ret.valid = false;
    ret.value = 0;
    if( !isnan(fy) && !isnan(fx) ) {
        if( IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH ) {
            ret.value = IKatan2Simple(fy,fx);
            ret.valid = true;
        }
    }
    return ret;
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

template <typename T>
inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
{
    CheckValue<T> ret;
    ret.valid = true;
    if( n == 0 ) {
        ret.value = 1.0;
        return ret;
    }
    else if( n == 1 )
    {
        ret.value = f;
        return ret;
    }
    else if( n < 0 )
    {
        if( f == 0 )
        {
            ret.valid = false;
            ret.value = (T)1.0e30;
            return ret;
        }
        if( n == -1 ) {
            ret.value = T(1.0)/f;
            return ret;
        }
    }

    int num = n > 0 ? n : -n;
    if( num == 2 ) {
        ret.value = f*f;
    }
    else if( num == 3 ) {
        ret.value = f*f*f;
    }
    else {
        ret.value = 1.0;
        while(num>0) {
            if( num & 1 ) {
                ret.value *= f;
            }
            num >>= 1;
            f *= f;
        }
    }
    
    if( n < 0 ) {
        ret.value = T(1.0)/ret.value;
    }
    return ret;
}

template <typename T> struct ComplexLess
{
    bool operator()(const complex<T>& lhs, const complex<T>& rhs) const
    {
        if (real(lhs) < real(rhs)) {
            return true;
        }
        if (real(lhs) > real(rhs)) {
            return false;
        }
        return imag(lhs) < imag(rhs);
    }
};

/// solves the forward kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {
IkReal x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37,x38,x39,x40,x41,x42,x43,x44,x45,x46,x47,x48,x49,x50,x51,x52,x53,x54,x55,x56,x57,x58,x59,x60,x61,x62,x63;
x0=IKcos(j[0]);
x1=IKcos(j[1]);
x2=IKcos(j[2]);
x3=IKsin(j[0]);
x4=IKsin(j[2]);
x5=IKsin(j[3]);
x6=IKcos(j[3]);
x7=IKsin(j[1]);
x8=IKcos(j[4]);
x9=IKsin(j[4]);
x10=IKsin(j[6]);
x11=IKsin(j[5]);
x12=IKcos(j[5]);
x13=IKcos(j[6]);
x14=((0.365572010101202)*x2);
x15=((0.300908770238559)*x9);
x16=((1.0)*x2);
x17=((0.300908770238559)*x8);
x18=((1.0)*x5);
x19=((1.0)*x6);
x20=((1.0)*x4);
x21=((0.300908770238559)*x5);
x22=((0.365572010101202)*x4);
x23=((1.0)*x11);
x24=((1.0)*x12);
x25=((0.365572010101202)*x6);
x26=((0.300908770238559)*x6);
x27=(x0*x1);
x28=(x5*x7);
x29=(x3*x7);
x30=(x4*x7);
x31=(x1*x3);
x32=(x6*x7);
x33=(x0*x7);
x34=(x20*x3);
x35=(x1*x19);
x36=(x18*x33);
x37=(x19*x33);
x38=(x18*x29);
x39=(x19*x29);
x40=((((-1.0)*x34))+((x2*x27)));
x41=(((x0*x4))+((x2*x31)));
x42=(((x0*x2))+(((-1.0)*x20*x31)));
x43=((((-1.0)*x35))+((x2*x28)));
x44=((((-1.0)*x16*x27))+x34);
x45=((((-1.0)*x16*x3))+(((-1.0)*x20*x27)));
x46=((((-1.0)*x0*x20))+(((-1.0)*x16*x31)));
x47=(((x1*x18))+((x16*x32)));
x48=((-1.0)*x47);
x49=(x11*x43);
x50=(x40*x6);
x51=(x46*x5);
x52=((((-1.0)*x36))+x50);
x53=((((-1.0)*x38))+((x41*x6)));
x54=((((-1.0)*x37))+((x44*x5)));
x55=((((-1.0)*x39))+x51);
x56=(((x47*x9))+((x30*x8)));
x57=(((x30*x9))+((x48*x8)));
x58=(x11*x55);
x59=(((x53*x8))+((x42*x9)));
x60=(((x42*x8))+((x9*(((((-1.0)*x19*x41))+x38)))));
x61=(((x52*x8))+((x45*x9)));
x62=(x12*x59);
x63=(((x9*((x36+(((-1.0)*x50))))))+((x45*x8)));
eerot[0]=(((x10*x63))+((x13*((((x12*x61))+((x11*x54)))))));
eerot[1]=(((x13*x63))+((x10*(((((-1.0)*x23*x54))+(((-1.0)*x24*x61)))))));
eerot[2]=(((x12*(((((-1.0)*x18*x44))+x37))))+((x11*x61)));
eetrans[0]=(((x5*((((x14*x27))+(((-1.0)*x22*x3))))))+(((0.392938370078201)*x33))+((x11*((((x17*x52))+((x15*x45))))))+((x12*((((x26*x33))+(((-1.0)*x21*x44))))))+((x25*x33)));
eerot[3]=(((x13*((x58+x62))))+((x10*x60)));
eerot[4]=(((x13*x60))+((x10*(((((-1.0)*x23*x55))+(((-1.0)*x24*x59)))))));
eerot[5]=(((x12*(((((-1.0)*x18*x46))+x39))))+((x11*x59)));
eetrans[1]=(((x11*((((x17*x53))+((x15*x42))))))+((x12*(((((-1.0)*x21*x46))+((x26*x29))))))+((x5*((((x0*x22))+((x14*x31))))))+(((0.392938370078201)*x29))+((x25*x29)));
eerot[6]=(((x13*((((x12*x57))+x49))))+((x10*x56)));
eerot[7]=(((x13*x56))+((x10*(((((-1.0)*x23*x43))+(((-1.0)*x24*x57)))))));
eerot[8]=(((x11*x57))+((x12*(((((-1.0)*x16*x28))+x35)))));
eetrans[2]=((0.342604869774656)+((x1*x25))+(((0.392938370078201)*x1))+((x11*((((x17*x48))+((x15*x30))))))+(((-1.0)*x14*x28))+((x12*((((x1*x26))+(((-1.0)*x2*x21*x7)))))));
}

IKFAST_API int GetNumFreeParameters() { return 1; }
IKFAST_API const int* GetFreeIndices() { static const int freeindices[] = {2}; return freeindices; }
IKFAST_API int GetNumJoints() { return 7; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

IKFAST_API int GetIkType() { return 0x67000001; }

class IKSolver {
public:
IkReal j4,cj4,sj4,htj4,j4mul,j5,cj5,sj5,htj5,j5mul,j7,cj7,sj7,htj7,j7mul,j8,cj8,sj8,htj8,j8mul,j9,cj9,sj9,htj9,j9mul,j10,cj10,sj10,htj10,j10mul,j6,cj6,sj6,htj6,new_r00,r00,rxp0_0,new_r01,r01,rxp0_1,new_r02,r02,rxp0_2,new_r10,r10,rxp1_0,new_r11,r11,rxp1_1,new_r12,r12,rxp1_2,new_r20,r20,rxp2_0,new_r21,r21,rxp2_1,new_r22,r22,rxp2_2,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij4[2], _nj4,_ij5[2], _nj5,_ij7[2], _nj7,_ij8[2], _nj8,_ij9[2], _nj9,_ij10[2], _nj10,_ij6[2], _nj6;

IkReal j100, cj100, sj100;
unsigned char _ij100[2], _nj100;
bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
j4=numeric_limits<IkReal>::quiet_NaN(); _ij4[0] = -1; _ij4[1] = -1; _nj4 = -1; j5=numeric_limits<IkReal>::quiet_NaN(); _ij5[0] = -1; _ij5[1] = -1; _nj5 = -1; j7=numeric_limits<IkReal>::quiet_NaN(); _ij7[0] = -1; _ij7[1] = -1; _nj7 = -1; j8=numeric_limits<IkReal>::quiet_NaN(); _ij8[0] = -1; _ij8[1] = -1; _nj8 = -1; j9=numeric_limits<IkReal>::quiet_NaN(); _ij9[0] = -1; _ij9[1] = -1; _nj9 = -1; j10=numeric_limits<IkReal>::quiet_NaN(); _ij10[0] = -1; _ij10[1] = -1; _nj10 = -1;  _ij6[0] = -1; _ij6[1] = -1; _nj6 = 0; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    solutions.Clear();
j6=pfree[0]; cj6=cos(pfree[0]); sj6=sin(pfree[0]), htj6=tan(pfree[0]*0.5);
r00 = eerot[0*3+0];
r01 = eerot[0*3+1];
r02 = eerot[0*3+2];
r10 = eerot[1*3+0];
r11 = eerot[1*3+1];
r12 = eerot[1*3+2];
r20 = eerot[2*3+0];
r21 = eerot[2*3+1];
r22 = eerot[2*3+2];
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_r00=r00;
new_r01=r01;
new_r02=r02;
new_px=((((-0.300908770238559)*r02))+px);
new_r10=r10;
new_r11=r11;
new_r12=r12;
new_py=(py+(((-0.300908770238559)*r12)));
new_r20=r20;
new_r21=r21;
new_r22=r22;
new_pz=((-0.342604869774656)+pz+(((-0.300908770238559)*r22)));
r00 = new_r00; r01 = new_r01; r02 = new_r02; r10 = new_r10; r11 = new_r11; r12 = new_r12; r20 = new_r20; r21 = new_r21; r22 = new_r22; px = new_px; py = new_py; pz = new_pz;
IkReal x64=((1.0)*px);
IkReal x65=((1.0)*pz);
IkReal x66=((1.0)*py);
pp=((px*px)+(py*py)+(pz*pz));
npx=(((px*r00))+((py*r10))+((pz*r20)));
npy=(((px*r01))+((py*r11))+((pz*r21)));
npz=(((px*r02))+((py*r12))+((pz*r22)));
rxp0_0=((((-1.0)*r20*x66))+((pz*r10)));
rxp0_1=(((px*r20))+(((-1.0)*r00*x65)));
rxp0_2=((((-1.0)*r10*x64))+((py*r00)));
rxp1_0=((((-1.0)*r21*x66))+((pz*r11)));
rxp1_1=(((px*r21))+(((-1.0)*r01*x65)));
rxp1_2=((((-1.0)*r11*x64))+((py*r01)));
rxp2_0=(((pz*r12))+(((-1.0)*r22*x66)));
rxp2_1=(((px*r22))+(((-1.0)*r02*x65)));
rxp2_2=((((-1.0)*r12*x64))+((py*r02)));
{
IkReal j7array[2], cj7array[2], sj7array[2];
bool j7valid[2]={false};
_nj7 = 2;
cj7array[0]=((-1.00260679391769)+(((3.48074836864103)*pp)));
if( cj7array[0] >= -1-IKFAST_SINCOS_THRESH && cj7array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j7valid[0] = j7valid[1] = true;
    j7array[0] = IKacos(cj7array[0]);
    sj7array[0] = IKsin(j7array[0]);
    cj7array[1] = cj7array[0];
    j7array[1] = -j7array[0];
    sj7array[1] = -sj7array[0];
}
else if( isnan(cj7array[0]) )
{
    // probably any value will work
    j7valid[0] = true;
    cj7array[0] = 1; sj7array[0] = 0; j7array[0] = 0;
}
for(int ij7 = 0; ij7 < 2; ++ij7)
{
if( !j7valid[ij7] )
{
    continue;
}
_ij7[0] = ij7; _ij7[1] = -1;
for(int iij7 = ij7+1; iij7 < 2; ++iij7)
{
if( j7valid[iij7] && IKabs(cj7array[ij7]-cj7array[iij7]) < IKFAST_SOLUTION_THRESH && IKabs(sj7array[ij7]-sj7array[iij7]) < IKFAST_SOLUTION_THRESH )
{
    j7valid[iij7]=false; _ij7[1] = iij7; break; 
}
}
j7 = j7array[ij7]; cj7 = cj7array[ij7]; sj7 = sj7array[ij7];

{
IkReal j4eval[2];
j4eval[0]=((px*px)+(py*py));
j4eval[1]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
j5eval[0]=((((2.73543918125233)*(IKabs(((-0.392938370078201)+(((-0.365572010101202)*cj7)))))))+(IKabs((cj6*sj7))));
j5eval[1]=((1.15532189853532)+(cj7*cj7)+(((2.14971802665867)*cj7))+(((cj6*cj6)*(sj7*sj7))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j4eval[2];
IkReal x67=py*py;
IkReal x68=cj6*cj6;
IkReal x69=sj6*sj6;
IkReal x70=px*px;
IkReal x71=py*py*py*py;
IkReal x72=sj6*sj6*sj6*sj6;
IkReal x73=cj6*cj6*cj6*cj6;
IkReal x74=((1.0)*px*py);
IkReal x75=(x67*x70);
IkReal x76=((2.0)*x68*x69);
j4eval[0]=((IKabs(((((-1.0)*x69*x74))+(((-1.0)*x68*x74)))))+(IKabs((((x67*x68))+((x67*x69))))));
j4eval[1]=(((x75*x76))+((x72*x75))+((x73*x75))+((x71*x76))+((x71*x72))+((x71*x73)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(cj6))+(IKabs(((-1.0)+(IKsign(sj6)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[2];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j4eval[0]=((px*px)+(py*py));
j4eval[1]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[1];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((-1.07485901332934)+(((-1.0)*cj7)));
if( IKabs(j5eval[0]) < 0.0000010000000000  )
{
continue; // no branches [j4, j5]

} else
{
{
IkReal j5array[2], cj5array[2], sj5array[2];
bool j5valid[2]={false};
_nj5 = 2;
CheckValue<IkReal> x77=IKPowWithIntegerCheck(((-0.392938370078201)+(((-0.365572010101202)*cj7))),-1);
if(!x77.valid){
continue;
}
cj5array[0]=((-1.0)*pz*(x77.value));
if( cj5array[0] >= -1-IKFAST_SINCOS_THRESH && cj5array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j5valid[0] = j5valid[1] = true;
    j5array[0] = IKacos(cj5array[0]);
    sj5array[0] = IKsin(j5array[0]);
    cj5array[1] = cj5array[0];
    j5array[1] = -j5array[0];
    sj5array[1] = -sj5array[0];
}
else if( isnan(cj5array[0]) )
{
    // probably any value will work
    j5valid[0] = true;
    cj5array[0] = 1; sj5array[0] = 0; j5array[0] = 0;
}
for(int ij5 = 0; ij5 < 2; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 2; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
IkReal x78=pz*pz;
IkReal x79=(cj5*pp);
IkReal x80=(cj5*x78);
IkReal x81=((32389191142403.0)*cj5*sj7);
IkReal x82=((88598662500000.0)*pz*sj5);
j4eval[0]=(x79+(((-1.0)*x80)));
j4eval[1]=((IKabs((((py*x82))+(((-1.0)*px*x81)))))+(IKabs((((py*x81))+((px*x82))))));
j4eval[2]=IKsign(((((88598662500000.0)*x79))+(((-88598662500000.0)*x80))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
IkReal x83=pz*pz;
IkReal x84=((32389191142403.0)*py);
IkReal x85=((34813814033858.6)*sj5);
IkReal x86=(cj7*sj5);
IkReal x87=((32389191142403.0)*px);
j4eval[0]=(pp+(((-1.0)*x83)));
j4eval[1]=((IKabs((((sj7*x84))+((x86*x87))+((px*x85)))))+(IKabs((((x84*x86))+(((-1.0)*sj7*x87))+((py*x85))))));
j4eval[2]=IKsign(((((-88598662500000.0)*x83))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
IkReal x88=pz*pz;
IkReal x89=(pp*sj5);
IkReal x90=((32389191142403.0)*cj7);
IkReal x91=((88598662500000.0)*cj5*pz);
IkReal x92=(sj5*x88);
IkReal x93=((32389191142403.0)*sj5*sj7);
j4eval[0]=(x89+(((-1.0)*x92)));
j4eval[1]=((IKabs(((((34813814033858.6)*py))+((py*x90))+(((-1.0)*px*x93))+(((-1.0)*py*x91)))))+(IKabs(((((34813814033858.6)*px))+((py*x93))+((px*x90))+(((-1.0)*px*x91))))));
j4eval[2]=IKsign(((((88598662500000.0)*x89))+(((-88598662500000.0)*x92))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[2];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=0;
cj5=1.0;
j5=0;
IkReal x94=pz*pz;
j4eval[0]=((((-1.0)*pp))+x94);
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((88598662500000.0)*x94))+(((-88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x96 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x96.valid){
continue;
}
IkReal x95=x96.value;
j4array[0]=((-1.0)*x95);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x95)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x639 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x639.valid){
continue;
}
IkReal x638=x639.value;
j4array[0]=((-1.0)*x638);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x638)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x640 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x640.valid){
continue;
}
CheckValue<IkReal> x641=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*pp))+(((88598662500000.0)*(pz*pz))))),-1);
if(!x641.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x640.value)+(((1.5707963267949)*(x641.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x642=IKsin(j4);
IkReal x643=IKcos(j4);
IkReal x644=((1.0)*py);
evalcond[0]=((((-1.0)*px*x643))+(((-1.0)*x642*x644)));
evalcond[1]=(((px*x642))+(((-1.0)*x643*x644))+(((0.365572010101202)*sj7)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
IkReal x645=pz*pz;
j4eval[0]=((((-1.0)*pp))+x645);
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((88598662500000.0)*x645))+(((-88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x647 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x647.valid){
continue;
}
IkReal x646=x647.value;
j4array[0]=((-1.0)*x646);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x646)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x649 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x649.valid){
continue;
}
IkReal x648=x649.value;
j4array[0]=((-1.0)*x648);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x648)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x650 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x650.valid){
continue;
}
CheckValue<IkReal> x651=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*pp))+(((88598662500000.0)*(pz*pz))))),-1);
if(!x651.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x650.value)+(((1.5707963267949)*(x651.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x652=IKsin(j4);
IkReal x653=IKcos(j4);
IkReal x654=((1.0)*py);
evalcond[0]=((((-1.0)*px*x653))+(((-1.0)*x652*x654)));
evalcond[1]=(((px*x652))+(((0.365572010101202)*sj7))+(((-1.0)*x653*x654)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j5)))), 6.28318530717959)));
evalcond[1]=pz;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=1.0;
cj5=0;
j5=1.5707963267949;
IkReal x655=pz*pz;
IkReal x656=((32389191142403.0)*sj7);
IkReal x657=((32389191142403.0)*cj7);
j4eval[0]=((((-1.0)*pp))+x655);
j4eval[1]=((IKabs(((((-34813814033858.6)*py))+((px*x656))+(((-1.0)*py*x657)))))+(IKabs(((((-1.0)*px*x657))+(((-34813814033858.6)*px))+(((-1.0)*py*x656))))));
j4eval[2]=IKsign(((((88598662500000.0)*x655))+(((-88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=1.0;
cj5=0;
j5=1.5707963267949;
IkReal x658=pz*pz;
IkReal x659=((1.11282777406866e+28)*pp);
IkReal x660=((3.1970934299486e+27)*sj7);
j4eval[0]=((((-1.0)*pp))+x658);
j4eval[1]=((IKabs(((((-2.30997095980183e+26)*px))+(((-1.0)*px*x659))+(((-1.0)*py*x660)))))+(IKabs(((((-2.30997095980183e+26)*py))+((px*x660))+(((-1.0)*py*x659))))));
j4eval[2]=IKsign(((((8.7454546344058e+27)*x658))+(((-8.7454546344058e+27)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x661=((1.11282777406866e+28)*pp);
IkReal x662=((3.1970934299486e+27)*sj7);
CheckValue<IkReal> x663=IKPowWithIntegerCheck(IKsign(((((8.7454546344058e+27)*(pz*pz)))+(((-8.7454546344058e+27)*pp)))),-1);
if(!x663.valid){
continue;
}
CheckValue<IkReal> x664 = IKatan2WithCheck(IkReal(((((-2.30997095980183e+26)*py))+((px*x662))+(((-1.0)*py*x661)))),IkReal(((((-2.30997095980183e+26)*px))+(((-1.0)*py*x662))+(((-1.0)*px*x661)))),IKFAST_ATAN2_MAGTHRESH);
if(!x664.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x663.value)))+(x664.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x665=IKcos(j4);
IkReal x666=IKsin(j4);
IkReal x667=(px*x665);
IkReal x668=(py*x666);
evalcond[0]=((((-1.0)*py*x665))+((px*x666))+(((0.365572010101202)*sj7)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+x668+x667);
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+(((0.785876740156401)*x668))+(((0.785876740156401)*x667)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x669=((32389191142403.0)*sj7);
IkReal x670=((32389191142403.0)*cj7);
CheckValue<IkReal> x671=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*pp))+(((88598662500000.0)*(pz*pz))))),-1);
if(!x671.valid){
continue;
}
CheckValue<IkReal> x672 = IKatan2WithCheck(IkReal((((px*x669))+(((-1.0)*py*x670))+(((-34813814033858.6)*py)))),IkReal(((((-34813814033858.6)*px))+(((-1.0)*py*x669))+(((-1.0)*px*x670)))),IKFAST_ATAN2_MAGTHRESH);
if(!x672.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x671.value)))+(x672.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x673=IKcos(j4);
IkReal x674=IKsin(j4);
IkReal x675=(px*x673);
IkReal x676=(py*x674);
evalcond[0]=((((-1.0)*py*x673))+(((0.365572010101202)*sj7))+((px*x674)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+x676+x675);
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+(((0.785876740156401)*x676))+(((0.785876740156401)*x675)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j5)))), 6.28318530717959)));
evalcond[1]=pz;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=-1.0;
cj5=0;
j5=-1.5707963267949;
IkReal x677=pz*pz;
IkReal x678=((32389191142403.0)*sj7);
IkReal x679=((32389191142403.0)*cj7);
j4eval[0]=(pp+(((-1.0)*x677)));
j4eval[1]=((IKabs((((py*x678))+(((-34813814033858.6)*px))+(((-1.0)*px*x679)))))+(IKabs(((((-1.0)*py*x679))+(((-34813814033858.6)*py))+(((-1.0)*px*x678))))));
j4eval[2]=IKsign(((((-88598662500000.0)*x677))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
sj5=-1.0;
cj5=0;
j5=-1.5707963267949;
IkReal x680=pz*pz;
IkReal x681=((1.11282777406866e+28)*pp);
IkReal x682=((3.1970934299486e+27)*sj7);
j4eval[0]=(pp+(((-1.0)*x680)));
j4eval[1]=IKsign(((((-8.7454546344058e+27)*x680))+(((8.7454546344058e+27)*pp))));
j4eval[2]=((IKabs(((((-2.30997095980183e+26)*py))+(((-1.0)*px*x682))+(((-1.0)*py*x681)))))+(IKabs(((((-2.30997095980183e+26)*px))+(((-1.0)*px*x681))+((py*x682))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x683=((1.11282777406866e+28)*pp);
IkReal x684=((3.1970934299486e+27)*sj7);
CheckValue<IkReal> x685=IKPowWithIntegerCheck(IKsign(((((-8.7454546344058e+27)*(pz*pz)))+(((8.7454546344058e+27)*pp)))),-1);
if(!x685.valid){
continue;
}
CheckValue<IkReal> x686 = IKatan2WithCheck(IkReal(((((-2.30997095980183e+26)*py))+(((-1.0)*px*x684))+(((-1.0)*py*x683)))),IkReal(((((-2.30997095980183e+26)*px))+(((-1.0)*px*x683))+((py*x684)))),IKFAST_ATAN2_MAGTHRESH);
if(!x686.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x685.value)))+(x686.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x687=IKsin(j4);
IkReal x688=IKcos(j4);
IkReal x689=(px*x688);
IkReal x690=(py*x687);
evalcond[0]=(((px*x687))+(((-1.0)*py*x688))+(((0.365572010101202)*sj7)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+(((-1.0)*x689))+(((-1.0)*x690)));
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+(((-0.785876740156401)*x690))+(((-0.785876740156401)*x689)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x691=((32389191142403.0)*sj7);
IkReal x692=((32389191142403.0)*cj7);
CheckValue<IkReal> x693 = IKatan2WithCheck(IkReal(((((-34813814033858.6)*py))+(((-1.0)*px*x691))+(((-1.0)*py*x692)))),IkReal((((py*x691))+(((-34813814033858.6)*px))+(((-1.0)*px*x692)))),IKFAST_ATAN2_MAGTHRESH);
if(!x693.valid){
continue;
}
CheckValue<IkReal> x694=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x694.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x693.value)+(((1.5707963267949)*(x694.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x695=IKsin(j4);
IkReal x696=IKcos(j4);
IkReal x697=(px*x696);
IkReal x698=(py*x695);
evalcond[0]=((((-1.0)*py*x696))+((px*x695))+(((0.365572010101202)*sj7)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+(((-1.0)*x698))+(((-1.0)*x697)));
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+(((-0.785876740156401)*x698))+(((-0.785876740156401)*x697)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(((-3.14159265358979)+(IKfmod(((3.14159265358979)+j7), 6.28318530717959)))))+(IKabs(pz)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
IkReal x699=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x699);
rxp0_1=(px*r20);
rxp1_0=(r21*x699);
rxp1_1=(px*r21);
rxp2_0=(r22*x699);
rxp2_1=(px*r22);
IkReal x700=(((sj5*(px*px)))+((sj5*(py*py))));
j4eval[0]=x700;
j4eval[1]=IKsign(x700);
j4eval[2]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[4];
IkReal x701=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x701);
rxp0_1=(px*r20);
rxp1_0=(r21*x701);
rxp1_1=(px*r21);
rxp2_0=(r22*x701);
rxp2_1=(px*r22);
IkReal x702=py*py;
IkReal x703=px*px;
j4eval[0]=(x702+x703);
j4eval[1]=IKsign(((((1.78052443850985e+21)*x703))+(((1.78052443850985e+21)*x702))));
j4eval[2]=1.8239752240962e+42;
j4eval[3]=sj5;
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x704=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x704);
rxp0_1=(px*r20);
rxp1_0=(r21*x704);
rxp1_1=(px*r21);
rxp2_0=(r22*x704);
rxp2_1=(px*r22);
sj5=0;
cj5=1.0;
j5=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x706 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x706.valid){
continue;
}
IkReal x705=x706.value;
j4array[0]=((-1.0)*x705);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x705)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x707=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x707);
rxp0_1=(px*r20);
rxp1_0=(r21*x707);
rxp1_1=(px*r21);
rxp2_0=(r22*x707);
rxp2_1=(px*r22);
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x709 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x709.valid){
continue;
}
IkReal x708=x709.value;
j4array[0]=((-1.0)*x708);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x708)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x710=((1.35054626877282e+21)*sj5);
CheckValue<IkReal> x711=IKPowWithIntegerCheck(IKsign(((((1.78052443850985e+21)*(py*py)))+(((1.78052443850985e+21)*(px*px))))),-1);
if(!x711.valid){
continue;
}
CheckValue<IkReal> x712 = IKatan2WithCheck(IkReal((py*x710)),IkReal((px*x710)),IKFAST_ATAN2_MAGTHRESH);
if(!x712.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x711.value)))+(x712.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x713=IKsin(j4);
IkReal x714=IKcos(j4);
IkReal x715=((0.785876740156401)*sj5);
IkReal x716=(px*x714);
IkReal x717=(py*x713);
evalcond[0]=((((-1.0)*py*x714))+((px*x713)));
evalcond[1]=((-0.758510380179403)+((sj5*x716))+((sj5*x717)));
evalcond[2]=((-0.596095664950182)+((x715*x717))+((x715*x716)));
evalcond[3]=((((-1.0)*x717))+(((-1.0)*x716))+(((0.758510380179403)*sj5)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x718=IKPowWithIntegerCheck(IKsign((((sj5*(px*px)))+((sj5*(py*py))))),-1);
if(!x718.valid){
continue;
}
CheckValue<IkReal> x719 = IKatan2WithCheck(IkReal(((0.758510380179403)*py)),IkReal(((0.758510380179403)*px)),IKFAST_ATAN2_MAGTHRESH);
if(!x719.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x718.value)))+(x719.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x720=IKsin(j4);
IkReal x721=IKcos(j4);
IkReal x722=((0.785876740156401)*sj5);
IkReal x723=(px*x721);
IkReal x724=(py*x720);
evalcond[0]=((((-1.0)*py*x721))+((px*x720)));
evalcond[1]=((-0.758510380179403)+((sj5*x723))+((sj5*x724)));
evalcond[2]=((-0.596095664950182)+((x722*x723))+((x722*x724)));
evalcond[3]=((((0.758510380179403)*sj5))+(((-1.0)*x723))+(((-1.0)*x724)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(pz))+(IKabs(((-3.14159265358979)+(IKfmod(j7, 6.28318530717959))))));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
IkReal x725=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x725);
rxp0_1=(px*r20);
rxp1_0=(r21*x725);
rxp1_1=(px*r21);
rxp2_0=(r22*x725);
rxp2_1=(px*r22);
IkReal x726=(((sj5*(px*px)))+((sj5*(py*py))));
j4eval[0]=x726;
j4eval[1]=IKsign(x726);
j4eval[2]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[4];
IkReal x727=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x727);
rxp0_1=(px*r20);
rxp1_0=(r21*x727);
rxp1_1=(px*r21);
rxp2_0=(r22*x727);
rxp2_1=(px*r22);
IkReal x728=py*py;
IkReal x729=px*px;
j4eval[0]=(x728+x729);
j4eval[1]=9.49707657964976e+37;
j4eval[2]=sj5;
j4eval[3]=IKsign(((((3.5610488770197e+20)*x728))+(((3.5610488770197e+20)*x729))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x730=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x730);
rxp0_1=(px*r20);
rxp1_0=(r21*x730);
rxp1_1=(px*r21);
rxp2_0=(r22*x730);
rxp2_1=(px*r22);
sj5=0;
cj5=1.0;
j5=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x732 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x732.valid){
continue;
}
IkReal x731=x732.value;
j4array[0]=((-1.0)*x731);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x731)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x733=((-1.0)*py);
sj6=1.0;
cj6=0;
j6=1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x733);
rxp0_1=(px*r20);
rxp1_0=(r21*x733);
rxp1_1=(px*r21);
rxp2_0=(r22*x733);
rxp2_1=(px*r22);
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x735 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x735.valid){
continue;
}
IkReal x734=x735.value;
j4array[0]=((-1.0)*x734);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x734)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x736=((9.74529454642073e+18)*sj5);
CheckValue<IkReal> x737 = IKatan2WithCheck(IkReal((py*x736)),IkReal((px*x736)),IKFAST_ATAN2_MAGTHRESH);
if(!x737.valid){
continue;
}
CheckValue<IkReal> x738=IKPowWithIntegerCheck(IKsign(((((3.5610488770197e+20)*(py*py)))+(((3.5610488770197e+20)*(px*px))))),-1);
if(!x738.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x737.value)+(((1.5707963267949)*(x738.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x739=IKsin(j4);
IkReal x740=IKcos(j4);
IkReal x741=((0.785876740156401)*sj5);
IkReal x742=(px*x740);
IkReal x743=(py*x739);
evalcond[0]=((((-1.0)*py*x740))+((px*x739)));
evalcond[1]=((-0.0273663599769985)+((sj5*x742))+((sj5*x743)));
evalcond[2]=((-0.0215065857686702)+((x741*x743))+((x741*x742)));
evalcond[3]=((((0.0273663599769985)*sj5))+(((-1.0)*x743))+(((-1.0)*x742)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x744 = IKatan2WithCheck(IkReal(((0.0273663599769985)*py)),IkReal(((0.0273663599769985)*px)),IKFAST_ATAN2_MAGTHRESH);
if(!x744.valid){
continue;
}
CheckValue<IkReal> x745=IKPowWithIntegerCheck(IKsign((((sj5*(px*px)))+((sj5*(py*py))))),-1);
if(!x745.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x744.value)+(((1.5707963267949)*(x745.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x746=IKsin(j4);
IkReal x747=IKcos(j4);
IkReal x748=((0.785876740156401)*sj5);
IkReal x749=(px*x747);
IkReal x750=(py*x746);
evalcond[0]=((((-1.0)*py*x747))+((px*x746)));
evalcond[1]=((-0.0273663599769985)+((sj5*x750))+((sj5*x749)));
evalcond[2]=((-0.0215065857686702)+((x748*x750))+((x748*x749)));
evalcond[3]=((((0.0273663599769985)*sj5))+(((-1.0)*x750))+(((-1.0)*x749)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x751=((32389191142403.0)*cj7);
IkReal x752=((88598662500000.0)*sj5);
IkReal x753=((88598662500000.0)*cj5*pz);
IkReal x754=((32389191142403.0)*sj5*sj7);
CheckValue<IkReal> x755 = IKatan2WithCheck(IkReal((((py*x751))+(((34813814033858.6)*py))+(((-1.0)*px*x754))+(((-1.0)*py*x753)))),IkReal((((py*x754))+(((34813814033858.6)*px))+(((-1.0)*px*x753))+((px*x751)))),IKFAST_ATAN2_MAGTHRESH);
if(!x755.valid){
continue;
}
CheckValue<IkReal> x756=IKPowWithIntegerCheck(IKsign(((((-1.0)*x752*(pz*pz)))+((pp*x752)))),-1);
if(!x756.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x755.value)+(((1.5707963267949)*(x756.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[5];
IkReal x757=IKsin(j4);
IkReal x758=IKcos(j4);
IkReal x759=((0.365572010101202)*cj7);
IkReal x760=(cj5*pz);
IkReal x761=(py*x757);
IkReal x762=(px*x758);
IkReal x763=(sj5*x761);
evalcond[0]=(((px*x757))+(((0.365572010101202)*sj7))+(((-1.0)*py*x758)));
evalcond[1]=(((cj5*x761))+((cj5*x762))+(((-1.0)*pz*sj5)));
evalcond[2]=((-0.392938370078201)+x760+x763+(((-1.0)*x759))+((sj5*x762)));
evalcond[3]=(((sj5*x759))+(((-1.0)*x761))+(((-1.0)*x762))+(((0.392938370078201)*sj5)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x760))+(((0.785876740156401)*x763))+(((-1.0)*pp))+(((0.785876740156401)*sj5*x762)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x764=((32389191142403.0)*py);
IkReal x765=((34813814033858.6)*sj5);
IkReal x766=(cj7*sj5);
IkReal x767=((32389191142403.0)*px);
CheckValue<IkReal> x768 = IKatan2WithCheck(IkReal(((((-1.0)*sj7*x767))+((py*x765))+((x764*x766)))),IkReal((((px*x765))+((sj7*x764))+((x766*x767)))),IKFAST_ATAN2_MAGTHRESH);
if(!x768.valid){
continue;
}
CheckValue<IkReal> x769=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x769.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x768.value)+(((1.5707963267949)*(x769.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[5];
IkReal x770=IKsin(j4);
IkReal x771=IKcos(j4);
IkReal x772=((0.365572010101202)*cj7);
IkReal x773=(cj5*pz);
IkReal x774=(py*x770);
IkReal x775=(px*x771);
IkReal x776=(sj5*x774);
evalcond[0]=((((-1.0)*py*x771))+((px*x770))+(((0.365572010101202)*sj7)));
evalcond[1]=((((-1.0)*pz*sj5))+((cj5*x775))+((cj5*x774)));
evalcond[2]=((-0.392938370078201)+x773+x776+((sj5*x775))+(((-1.0)*x772)));
evalcond[3]=((((0.392938370078201)*sj5))+((sj5*x772))+(((-1.0)*x774))+(((-1.0)*x775)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x773))+(((0.785876740156401)*x776))+(((0.785876740156401)*sj5*x775))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x777=((88598662500000.0)*cj5);
IkReal x778=((32389191142403.0)*cj5*sj7);
IkReal x779=((88598662500000.0)*pz*sj5);
CheckValue<IkReal> x780=IKPowWithIntegerCheck(IKsign((((pp*x777))+(((-1.0)*x777*(pz*pz))))),-1);
if(!x780.valid){
continue;
}
CheckValue<IkReal> x781 = IKatan2WithCheck(IkReal(((((-1.0)*px*x778))+((py*x779)))),IkReal((((px*x779))+((py*x778)))),IKFAST_ATAN2_MAGTHRESH);
if(!x781.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x780.value)))+(x781.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[5];
IkReal x782=IKsin(j4);
IkReal x783=IKcos(j4);
IkReal x784=((0.365572010101202)*cj7);
IkReal x785=(cj5*pz);
IkReal x786=(py*x782);
IkReal x787=(px*x783);
IkReal x788=(sj5*x786);
evalcond[0]=(((px*x782))+(((0.365572010101202)*sj7))+(((-1.0)*py*x783)));
evalcond[1]=(((cj5*x786))+((cj5*x787))+(((-1.0)*pz*sj5)));
evalcond[2]=((-0.392938370078201)+((sj5*x787))+(((-1.0)*x784))+x785+x788);
evalcond[3]=(((sj5*x784))+(((0.392938370078201)*sj5))+(((-1.0)*x786))+(((-1.0)*x787)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*sj5*x787))+(((0.785876740156401)*x785))+(((0.785876740156401)*x788))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x791 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x791.valid){
continue;
}
IkReal x789=((1.0)*(x791.value));
if((((px*px)+(py*py))) < -0.00001)
continue;
CheckValue<IkReal> x792=IKPowWithIntegerCheck(IKabs(IKsqrt(((px*px)+(py*py)))),-1);
if(!x792.valid){
continue;
}
if( (((0.365572010101202)*sj7*(x792.value))) < -1-IKFAST_SINCOS_THRESH || (((0.365572010101202)*sj7*(x792.value))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x790=IKasin(((0.365572010101202)*sj7*(x792.value)));
j4array[0]=((((-1.0)*x789))+(((-1.0)*x790)));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x789))+x790);
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j5eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((1.07485901332934)+cj7);
j5eval[1]=((IKabs(pz))+(IKabs((((cj4*px))+((py*sj4))))));
j5eval[2]=IKsign(((0.392938370078201)+(((0.365572010101202)*cj7))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((-1.07485901332934)+(((-1.0)*cj7)));
j5eval[1]=((IKabs(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))))+(IKabs(pz)));
j5eval[2]=IKsign(((-0.392938370078201)+(((-0.365572010101202)*cj7))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
IkReal x793=((1.0)*cj7);
IkReal x794=(py*sj4);
IkReal x795=(cj4*px);
j5eval[0]=((((-1.0)*x793*x794))+(((-1.0)*x793*x795))+(((-1.07485901332934)*x794))+(((-1.07485901332934)*x795)));
j5eval[1]=((-1.07485901332934)+(((-1.0)*x793)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x796=(cj4*px);
IkReal x797=((0.365572010101202)*cj7);
IkReal x798=(py*sj4);
CheckValue<IkReal> x799=IKPowWithIntegerCheck(((((-1.0)*x796*x797))+(((-1.0)*x797*x798))+(((-0.392938370078201)*x798))+(((-0.392938370078201)*x796))),-1);
if(!x799.valid){
continue;
}
CheckValue<IkReal> x800=IKPowWithIntegerCheck(((-0.392938370078201)+(((-1.0)*x797))),-1);
if(!x800.valid){
continue;
}
if( IKabs(((x799.value)*(((-0.154400562679713)+(pz*pz)+(((-0.133642894569433)*(cj7*cj7)))+(((-0.287294539590756)*cj7)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*pz*(x800.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x799.value)*(((-0.154400562679713)+(pz*pz)+(((-0.133642894569433)*(cj7*cj7)))+(((-0.287294539590756)*cj7))))))+IKsqr(((-1.0)*pz*(x800.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j5array[0]=IKatan2(((x799.value)*(((-0.154400562679713)+(pz*pz)+(((-0.133642894569433)*(cj7*cj7)))+(((-0.287294539590756)*cj7))))), ((-1.0)*pz*(x800.value)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x801=IKcos(j5);
IkReal x802=IKsin(j5);
IkReal x803=((0.365572010101202)*cj7);
IkReal x804=(cj4*px);
IkReal x805=(py*sj4);
IkReal x806=((0.785876740156401)*x802);
IkReal x807=(pz*x801);
evalcond[0]=((((-0.392938370078201)*x801))+pz+(((-1.0)*x801*x803)));
evalcond[1]=((((-1.0)*pz*x802))+((x801*x804))+((x801*x805)));
evalcond[2]=((((-1.0)*x805))+(((-1.0)*x804))+(((0.392938370078201)*x802))+((x802*x803)));
evalcond[3]=((-0.392938370078201)+x807+(((-1.0)*x803))+((x802*x805))+((x802*x804)));
evalcond[4]=((-0.0207576681102795)+((x804*x806))+((x805*x806))+(((-1.0)*pp))+(((0.785876740156401)*x807)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x808 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))),IkReal(((-1.0)*pz)),IKFAST_ATAN2_MAGTHRESH);
if(!x808.valid){
continue;
}
CheckValue<IkReal> x809=IKPowWithIntegerCheck(IKsign(((-0.392938370078201)+(((-0.365572010101202)*cj7)))),-1);
if(!x809.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x808.value)+(((1.5707963267949)*(x809.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x810=IKcos(j5);
IkReal x811=IKsin(j5);
IkReal x812=((0.365572010101202)*cj7);
IkReal x813=(cj4*px);
IkReal x814=(py*sj4);
IkReal x815=((0.785876740156401)*x811);
IkReal x816=(pz*x810);
evalcond[0]=((((-1.0)*x810*x812))+pz+(((-0.392938370078201)*x810)));
evalcond[1]=(((x810*x813))+((x810*x814))+(((-1.0)*pz*x811)));
evalcond[2]=((((-1.0)*x813))+(((-1.0)*x814))+((x811*x812))+(((0.392938370078201)*x811)));
evalcond[3]=((-0.392938370078201)+(((-1.0)*x812))+((x811*x813))+((x811*x814))+x816);
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x816))+((x813*x815))+((x814*x815))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x817=IKPowWithIntegerCheck(IKsign(((0.392938370078201)+(((0.365572010101202)*cj7)))),-1);
if(!x817.valid){
continue;
}
CheckValue<IkReal> x818 = IKatan2WithCheck(IkReal((((cj4*px))+((py*sj4)))),IkReal(pz),IKFAST_ATAN2_MAGTHRESH);
if(!x818.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x817.value)))+(x818.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x819=IKcos(j5);
IkReal x820=IKsin(j5);
IkReal x821=((0.365572010101202)*cj7);
IkReal x822=(cj4*px);
IkReal x823=(py*sj4);
IkReal x824=((0.785876740156401)*x820);
IkReal x825=(pz*x819);
evalcond[0]=((((-1.0)*x819*x821))+pz+(((-0.392938370078201)*x819)));
evalcond[1]=(((x819*x822))+((x819*x823))+(((-1.0)*pz*x820)));
evalcond[2]=((((0.392938370078201)*x820))+((x820*x821))+(((-1.0)*x823))+(((-1.0)*x822)));
evalcond[3]=((-0.392938370078201)+((x820*x823))+((x820*x822))+(((-1.0)*x821))+x825);
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x825))+(((-1.0)*pp))+((x822*x824))+((x823*x824)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(cj6))+(IKabs(((1.0)+(IKsign(sj6)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[2];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j4eval[0]=((px*px)+(py*py));
j4eval[1]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[1];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((-1.07485901332934)+(((-1.0)*cj7)));
if( IKabs(j5eval[0]) < 0.0000010000000000  )
{
continue; // no branches [j4, j5]

} else
{
{
IkReal j5array[2], cj5array[2], sj5array[2];
bool j5valid[2]={false};
_nj5 = 2;
CheckValue<IkReal> x826=IKPowWithIntegerCheck(((-0.392938370078201)+(((-0.365572010101202)*cj7))),-1);
if(!x826.valid){
continue;
}
cj5array[0]=((-1.0)*pz*(x826.value));
if( cj5array[0] >= -1-IKFAST_SINCOS_THRESH && cj5array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j5valid[0] = j5valid[1] = true;
    j5array[0] = IKacos(cj5array[0]);
    sj5array[0] = IKsin(j5array[0]);
    cj5array[1] = cj5array[0];
    j5array[1] = -j5array[0];
    sj5array[1] = -sj5array[0];
}
else if( isnan(cj5array[0]) )
{
    // probably any value will work
    j5valid[0] = true;
    cj5array[0] = 1; sj5array[0] = 0; j5array[0] = 0;
}
for(int ij5 = 0; ij5 < 2; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 2; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
IkReal x827=pz*pz;
IkReal x828=(cj5*pp);
IkReal x829=(cj5*x827);
IkReal x830=((32389191142403.0)*cj5*sj7);
IkReal x831=((88598662500000.0)*pz*sj5);
j4eval[0]=((((-1.0)*x829))+x828);
j4eval[1]=((IKabs(((((-1.0)*py*x830))+((px*x831)))))+(IKabs((((py*x831))+((px*x830))))));
j4eval[2]=IKsign(((((-88598662500000.0)*x829))+(((88598662500000.0)*x828))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
IkReal x832=pz*pz;
IkReal x833=((32389191142403.0)*sj7);
IkReal x834=((34813814033858.6)*sj5);
IkReal x835=((32389191142403.0)*cj7*sj5);
j4eval[0]=(pp+(((-1.0)*x832)));
j4eval[1]=((IKabs(((((-1.0)*py*x833))+((px*x834))+((px*x835)))))+(IKabs((((py*x835))+((py*x834))+((px*x833))))));
j4eval[2]=IKsign(((((-88598662500000.0)*x832))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
IkReal x836=pz*pz;
IkReal x837=(pp*sj5);
IkReal x838=((32389191142403.0)*cj7);
IkReal x839=((88598662500000.0)*cj5*pz);
IkReal x840=(sj5*x836);
IkReal x841=((32389191142403.0)*sj5*sj7);
j4eval[0]=((((-1.0)*x840))+x837);
j4eval[1]=IKsign(((((-88598662500000.0)*x840))+(((88598662500000.0)*x837))));
j4eval[2]=((IKabs(((((34813814033858.6)*py))+(((-1.0)*py*x839))+((py*x838))+((px*x841)))))+(IKabs(((((-1.0)*py*x841))+(((34813814033858.6)*px))+(((-1.0)*px*x839))+((px*x838))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[2];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=0;
cj5=1.0;
j5=0;
IkReal x842=pz*pz;
j4eval[0]=(pp+(((-1.0)*x842)));
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((-88598662500000.0)*x842))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x844 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x844.valid){
continue;
}
IkReal x843=x844.value;
j4array[0]=((-1.0)*x843);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x843)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x846 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x846.valid){
continue;
}
IkReal x845=x846.value;
j4array[0]=((-1.0)*x845);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x845)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x847 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x847.valid){
continue;
}
CheckValue<IkReal> x848=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x848.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x847.value)+(((1.5707963267949)*(x848.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x849=IKsin(j4);
IkReal x850=IKcos(j4);
IkReal x851=((1.0)*py);
evalcond[0]=((((-1.0)*x849*x851))+(((-1.0)*px*x850)));
evalcond[1]=((((-1.0)*x850*x851))+(((-0.365572010101202)*sj7))+((px*x849)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
IkReal x852=pz*pz;
j4eval[0]=((((-1.0)*x852))+pp);
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((-88598662500000.0)*x852))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x854 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x854.valid){
continue;
}
IkReal x853=x854.value;
j4array[0]=((-1.0)*x853);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x853)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x856 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x856.valid){
continue;
}
IkReal x855=x856.value;
j4array[0]=((-1.0)*x855);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x855)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x857 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x857.valid){
continue;
}
CheckValue<IkReal> x858=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x858.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x857.value)+(((1.5707963267949)*(x858.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x859=IKsin(j4);
IkReal x860=IKcos(j4);
IkReal x861=((1.0)*py);
evalcond[0]=((((-1.0)*px*x860))+(((-1.0)*x859*x861)));
evalcond[1]=((((-1.0)*x860*x861))+((px*x859))+(((-0.365572010101202)*sj7)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j5)))), 6.28318530717959)));
evalcond[1]=pz;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=1.0;
cj5=0;
j5=1.5707963267949;
IkReal x862=pz*pz;
IkReal x863=((32389191142403.0)*sj7);
IkReal x864=((32389191142403.0)*cj7);
j4eval[0]=((((-1.0)*x862))+pp);
j4eval[1]=((IKabs(((((34813814033858.6)*py))+((px*x863))+((py*x864)))))+(IKabs(((((34813814033858.6)*px))+(((-1.0)*py*x863))+((px*x864))))));
j4eval[2]=IKsign(((((88598662500000.0)*pp))+(((-88598662500000.0)*x862))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=1.0;
cj5=0;
j5=1.5707963267949;
IkReal x865=pz*pz;
IkReal x866=((1.11282777406866e+28)*pp);
IkReal x867=((3.1970934299486e+27)*sj7);
j4eval[0]=((((-1.0)*x865))+pp);
j4eval[1]=IKsign(((((-8.7454546344058e+27)*x865))+(((8.7454546344058e+27)*pp))));
j4eval[2]=((IKabs(((((2.30997095980183e+26)*py))+((px*x867))+((py*x866)))))+(IKabs(((((2.30997095980183e+26)*px))+(((-1.0)*py*x867))+((px*x866))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x868=((1.11282777406866e+28)*pp);
IkReal x869=((3.1970934299486e+27)*sj7);
CheckValue<IkReal> x870=IKPowWithIntegerCheck(IKsign(((((-8.7454546344058e+27)*(pz*pz)))+(((8.7454546344058e+27)*pp)))),-1);
if(!x870.valid){
continue;
}
CheckValue<IkReal> x871 = IKatan2WithCheck(IkReal(((((2.30997095980183e+26)*py))+((px*x869))+((py*x868)))),IkReal(((((2.30997095980183e+26)*px))+(((-1.0)*py*x869))+((px*x868)))),IKFAST_ATAN2_MAGTHRESH);
if(!x871.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x870.value)))+(x871.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x872=IKcos(j4);
IkReal x873=IKsin(j4);
IkReal x874=(px*x872);
IkReal x875=(py*x873);
evalcond[0]=((((-1.0)*py*x872))+(((-0.365572010101202)*sj7))+((px*x873)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+x874+x875);
evalcond[2]=((-0.0207576681102795)+(((0.785876740156401)*x875))+(((0.785876740156401)*x874))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x876=((32389191142403.0)*sj7);
IkReal x877=((32389191142403.0)*cj7);
CheckValue<IkReal> x878 = IKatan2WithCheck(IkReal(((((34813814033858.6)*py))+((px*x876))+((py*x877)))),IkReal(((((34813814033858.6)*px))+(((-1.0)*py*x876))+((px*x877)))),IKFAST_ATAN2_MAGTHRESH);
if(!x878.valid){
continue;
}
CheckValue<IkReal> x879=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x879.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x878.value)+(((1.5707963267949)*(x879.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x880=IKcos(j4);
IkReal x881=IKsin(j4);
IkReal x882=(px*x880);
IkReal x883=(py*x881);
evalcond[0]=(((px*x881))+(((-0.365572010101202)*sj7))+(((-1.0)*py*x880)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+x883+x882);
evalcond[2]=((-0.0207576681102795)+(((0.785876740156401)*x882))+(((0.785876740156401)*x883))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j5)))), 6.28318530717959)));
evalcond[1]=pz;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=-1.0;
cj5=0;
j5=-1.5707963267949;
IkReal x884=pz*pz;
IkReal x885=((32389191142403.0)*sj7);
IkReal x886=((32389191142403.0)*cj7);
j4eval[0]=((((-1.0)*x884))+pp);
j4eval[1]=((IKabs((((px*x885))+(((-1.0)*py*x886))+(((-34813814033858.6)*py)))))+(IKabs(((((-1.0)*py*x885))+(((-34813814033858.6)*px))+(((-1.0)*px*x886))))));
j4eval[2]=IKsign(((((88598662500000.0)*pp))+(((-88598662500000.0)*x884))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
sj5=-1.0;
cj5=0;
j5=-1.5707963267949;
IkReal x887=pz*pz;
IkReal x888=((1.11282777406866e+28)*pp);
IkReal x889=((3.1970934299486e+27)*sj7);
j4eval[0]=((((-1.0)*x887))+pp);
j4eval[1]=((IKabs(((((-2.30997095980183e+26)*py))+((px*x889))+(((-1.0)*py*x888)))))+(IKabs(((((-2.30997095980183e+26)*px))+(((-1.0)*py*x889))+(((-1.0)*px*x888))))));
j4eval[2]=IKsign(((((-8.7454546344058e+27)*x887))+(((8.7454546344058e+27)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x890=((1.11282777406866e+28)*pp);
IkReal x891=((3.1970934299486e+27)*sj7);
CheckValue<IkReal> x892=IKPowWithIntegerCheck(IKsign(((((-8.7454546344058e+27)*(pz*pz)))+(((8.7454546344058e+27)*pp)))),-1);
if(!x892.valid){
continue;
}
CheckValue<IkReal> x893 = IKatan2WithCheck(IkReal(((((-2.30997095980183e+26)*py))+((px*x891))+(((-1.0)*py*x890)))),IkReal(((((-2.30997095980183e+26)*px))+(((-1.0)*py*x891))+(((-1.0)*px*x890)))),IKFAST_ATAN2_MAGTHRESH);
if(!x893.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x892.value)))+(x893.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x894=IKsin(j4);
IkReal x895=IKcos(j4);
IkReal x896=(px*x895);
IkReal x897=(py*x894);
evalcond[0]=(((px*x894))+(((-0.365572010101202)*sj7))+(((-1.0)*py*x895)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+(((-1.0)*x897))+(((-1.0)*x896)));
evalcond[2]=((-0.0207576681102795)+(((-0.785876740156401)*x896))+(((-0.785876740156401)*x897))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x898=((32389191142403.0)*sj7);
IkReal x899=((32389191142403.0)*cj7);
CheckValue<IkReal> x900=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x900.valid){
continue;
}
CheckValue<IkReal> x901 = IKatan2WithCheck(IkReal((((px*x898))+(((-1.0)*py*x899))+(((-34813814033858.6)*py)))),IkReal(((((-1.0)*py*x898))+(((-34813814033858.6)*px))+(((-1.0)*px*x899)))),IKFAST_ATAN2_MAGTHRESH);
if(!x901.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x900.value)))+(x901.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x902=IKsin(j4);
IkReal x903=IKcos(j4);
IkReal x904=(px*x903);
IkReal x905=(py*x902);
evalcond[0]=((((-1.0)*py*x903))+((px*x902))+(((-0.365572010101202)*sj7)));
evalcond[1]=((-0.392938370078201)+(((-0.365572010101202)*cj7))+(((-1.0)*x904))+(((-1.0)*x905)));
evalcond[2]=((-0.0207576681102795)+(((-0.785876740156401)*x904))+(((-0.785876740156401)*x905))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(((-3.14159265358979)+(IKfmod(((3.14159265358979)+j7), 6.28318530717959)))))+(IKabs(pz)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
IkReal x906=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x906);
rxp0_1=(px*r20);
rxp1_0=(r21*x906);
rxp1_1=(px*r21);
rxp2_0=(r22*x906);
rxp2_1=(px*r22);
IkReal x907=(((sj5*(px*px)))+((sj5*(py*py))));
j4eval[0]=x907;
j4eval[1]=IKsign(x907);
j4eval[2]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[4];
IkReal x908=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x908);
rxp0_1=(px*r20);
rxp1_0=(r21*x908);
rxp1_1=(px*r21);
rxp2_0=(r22*x908);
rxp2_1=(px*r22);
IkReal x909=py*py;
IkReal x910=px*px;
j4eval[0]=(x910+x909);
j4eval[1]=IKsign(((((1.78052443850985e+21)*x910))+(((1.78052443850985e+21)*x909))));
j4eval[2]=1.8239752240962e+42;
j4eval[3]=sj5;
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x911=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x911);
rxp0_1=(px*r20);
rxp1_0=(r21*x911);
rxp1_1=(px*r21);
rxp2_0=(r22*x911);
rxp2_1=(px*r22);
sj5=0;
cj5=1.0;
j5=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x913 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x913.valid){
continue;
}
IkReal x912=x913.value;
j4array[0]=((-1.0)*x912);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x912)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x914=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=0;
sj7=0;
cj7=1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x914);
rxp0_1=(px*r20);
rxp1_0=(r21*x914);
rxp1_1=(px*r21);
rxp2_0=(r22*x914);
rxp2_1=(px*r22);
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x916 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x916.valid){
continue;
}
IkReal x915=x916.value;
j4array[0]=((-1.0)*x915);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x915)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x917=((1.35054626877282e+21)*sj5);
CheckValue<IkReal> x918=IKPowWithIntegerCheck(IKsign(((((1.78052443850985e+21)*(py*py)))+(((1.78052443850985e+21)*(px*px))))),-1);
if(!x918.valid){
continue;
}
CheckValue<IkReal> x919 = IKatan2WithCheck(IkReal((py*x917)),IkReal((px*x917)),IKFAST_ATAN2_MAGTHRESH);
if(!x919.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x918.value)))+(x919.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x920=IKsin(j4);
IkReal x921=IKcos(j4);
IkReal x922=((0.785876740156401)*sj5);
IkReal x923=(px*x921);
IkReal x924=(py*x920);
evalcond[0]=((((-1.0)*py*x921))+((px*x920)));
evalcond[1]=((-0.758510380179403)+((sj5*x923))+((sj5*x924)));
evalcond[2]=((-0.596095664950182)+((x922*x923))+((x922*x924)));
evalcond[3]=((((0.758510380179403)*sj5))+(((-1.0)*x923))+(((-1.0)*x924)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x925=IKPowWithIntegerCheck(IKsign((((sj5*(px*px)))+((sj5*(py*py))))),-1);
if(!x925.valid){
continue;
}
CheckValue<IkReal> x926 = IKatan2WithCheck(IkReal(((0.758510380179403)*py)),IkReal(((0.758510380179403)*px)),IKFAST_ATAN2_MAGTHRESH);
if(!x926.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x925.value)))+(x926.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x927=IKsin(j4);
IkReal x928=IKcos(j4);
IkReal x929=((0.785876740156401)*sj5);
IkReal x930=(px*x928);
IkReal x931=(py*x927);
evalcond[0]=((((-1.0)*py*x928))+((px*x927)));
evalcond[1]=((-0.758510380179403)+((sj5*x930))+((sj5*x931)));
evalcond[2]=((-0.596095664950182)+((x929*x930))+((x929*x931)));
evalcond[3]=((((0.758510380179403)*sj5))+(((-1.0)*x931))+(((-1.0)*x930)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(pz))+(IKabs(((-3.14159265358979)+(IKfmod(j7, 6.28318530717959))))));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
IkReal x932=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x932);
rxp0_1=(px*r20);
rxp1_0=(r21*x932);
rxp1_1=(px*r21);
rxp2_0=(r22*x932);
rxp2_1=(px*r22);
IkReal x933=(((sj5*(px*px)))+((sj5*(py*py))));
j4eval[0]=x933;
j4eval[1]=IKsign(x933);
j4eval[2]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[4];
IkReal x934=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x934);
rxp0_1=(px*r20);
rxp1_0=(r21*x934);
rxp1_1=(px*r21);
rxp2_0=(r22*x934);
rxp2_1=(px*r22);
IkReal x935=py*py;
IkReal x936=px*px;
j4eval[0]=(x935+x936);
j4eval[1]=9.49707657964976e+37;
j4eval[2]=sj5;
j4eval[3]=IKsign(((((3.5610488770197e+20)*x935))+(((3.5610488770197e+20)*x936))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x937=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x937);
rxp0_1=(px*r20);
rxp1_0=(r21*x937);
rxp1_1=(px*r21);
rxp2_0=(r22*x937);
rxp2_1=(px*r22);
sj5=0;
cj5=1.0;
j5=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x939 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x939.valid){
continue;
}
IkReal x938=x939.value;
j4array[0]=((-1.0)*x938);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x938)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
IkReal x940=((-1.0)*py);
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
pz=0;
j7=3.14159265358979;
sj7=0;
cj7=-1.0;
pp=((px*px)+(py*py));
npx=(((px*r00))+((py*r10)));
npy=(((px*r01))+((py*r11)));
npz=(((px*r02))+((py*r12)));
rxp0_0=(r20*x940);
rxp0_1=(px*r20);
rxp1_0=(r21*x940);
rxp1_1=(px*r21);
rxp2_0=(r22*x940);
rxp2_1=(px*r22);
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x942 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x942.valid){
continue;
}
IkReal x941=x942.value;
j4array[0]=((-1.0)*x941);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x941)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x943=((9.74529454642073e+18)*sj5);
CheckValue<IkReal> x944 = IKatan2WithCheck(IkReal((py*x943)),IkReal((px*x943)),IKFAST_ATAN2_MAGTHRESH);
if(!x944.valid){
continue;
}
CheckValue<IkReal> x945=IKPowWithIntegerCheck(IKsign(((((3.5610488770197e+20)*(py*py)))+(((3.5610488770197e+20)*(px*px))))),-1);
if(!x945.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x944.value)+(((1.5707963267949)*(x945.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x946=IKsin(j4);
IkReal x947=IKcos(j4);
IkReal x948=((0.785876740156401)*sj5);
IkReal x949=(px*x947);
IkReal x950=(py*x946);
evalcond[0]=(((px*x946))+(((-1.0)*py*x947)));
evalcond[1]=((-0.0273663599769985)+((sj5*x950))+((sj5*x949)));
evalcond[2]=((-0.0215065857686702)+((x948*x949))+((x948*x950)));
evalcond[3]=((((-1.0)*x950))+(((-1.0)*x949))+(((0.0273663599769985)*sj5)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x951 = IKatan2WithCheck(IkReal(((0.0273663599769985)*py)),IkReal(((0.0273663599769985)*px)),IKFAST_ATAN2_MAGTHRESH);
if(!x951.valid){
continue;
}
CheckValue<IkReal> x952=IKPowWithIntegerCheck(IKsign((((sj5*(px*px)))+((sj5*(py*py))))),-1);
if(!x952.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x951.value)+(((1.5707963267949)*(x952.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x953=IKsin(j4);
IkReal x954=IKcos(j4);
IkReal x955=((0.785876740156401)*sj5);
IkReal x956=(px*x954);
IkReal x957=(py*x953);
evalcond[0]=(((px*x953))+(((-1.0)*py*x954)));
evalcond[1]=((-0.0273663599769985)+((sj5*x956))+((sj5*x957)));
evalcond[2]=((-0.0215065857686702)+((x955*x956))+((x955*x957)));
evalcond[3]=((((-1.0)*x957))+(((-1.0)*x956))+(((0.0273663599769985)*sj5)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x958=((32389191142403.0)*cj7);
IkReal x959=((88598662500000.0)*sj5);
IkReal x960=((88598662500000.0)*cj5*pz);
IkReal x961=((32389191142403.0)*sj5*sj7);
CheckValue<IkReal> x962 = IKatan2WithCheck(IkReal((((px*x961))+(((-1.0)*py*x960))+(((34813814033858.6)*py))+((py*x958)))),IkReal((((px*x958))+(((-1.0)*py*x961))+(((34813814033858.6)*px))+(((-1.0)*px*x960)))),IKFAST_ATAN2_MAGTHRESH);
if(!x962.valid){
continue;
}
CheckValue<IkReal> x963=IKPowWithIntegerCheck(IKsign(((((-1.0)*x959*(pz*pz)))+((pp*x959)))),-1);
if(!x963.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x962.value)+(((1.5707963267949)*(x963.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[5];
IkReal x964=IKsin(j4);
IkReal x965=IKcos(j4);
IkReal x966=((1.0)*cj5);
IkReal x967=((0.365572010101202)*cj7);
IkReal x968=((0.785876740156401)*sj5);
IkReal x969=(cj5*pz);
IkReal x970=(py*x964);
IkReal x971=(px*x965);
evalcond[0]=(((px*x964))+(((-0.365572010101202)*sj7))+(((-1.0)*py*x965)));
evalcond[1]=((((-1.0)*x966*x971))+(((-1.0)*x966*x970))+((pz*sj5)));
evalcond[2]=((-0.392938370078201)+((sj5*x971))+((sj5*x970))+x969+(((-1.0)*x967)));
evalcond[3]=((((-1.0)*x971))+(((-1.0)*x970))+(((0.392938370078201)*sj5))+((sj5*x967)));
evalcond[4]=((-0.0207576681102795)+((x968*x970))+((x968*x971))+(((-1.0)*pp))+(((0.785876740156401)*x969)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x972=((32389191142403.0)*sj7);
IkReal x973=((34813814033858.6)*sj5);
IkReal x974=((32389191142403.0)*cj7*sj5);
CheckValue<IkReal> x975 = IKatan2WithCheck(IkReal((((py*x974))+((py*x973))+((px*x972)))),IkReal(((((-1.0)*py*x972))+((px*x973))+((px*x974)))),IKFAST_ATAN2_MAGTHRESH);
if(!x975.valid){
continue;
}
CheckValue<IkReal> x976=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x976.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x975.value)+(((1.5707963267949)*(x976.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[5];
IkReal x977=IKsin(j4);
IkReal x978=IKcos(j4);
IkReal x979=((1.0)*cj5);
IkReal x980=((0.365572010101202)*cj7);
IkReal x981=((0.785876740156401)*sj5);
IkReal x982=(cj5*pz);
IkReal x983=(py*x977);
IkReal x984=(px*x978);
evalcond[0]=((((-0.365572010101202)*sj7))+(((-1.0)*py*x978))+((px*x977)));
evalcond[1]=(((pz*sj5))+(((-1.0)*x979*x984))+(((-1.0)*x979*x983)));
evalcond[2]=((-0.392938370078201)+(((-1.0)*x980))+((sj5*x984))+((sj5*x983))+x982);
evalcond[3]=((((-1.0)*x983))+(((-1.0)*x984))+(((0.392938370078201)*sj5))+((sj5*x980)));
evalcond[4]=((-0.0207576681102795)+((x981*x983))+((x981*x984))+(((-1.0)*pp))+(((0.785876740156401)*x982)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x985=((88598662500000.0)*cj5);
IkReal x986=((32389191142403.0)*cj5*sj7);
IkReal x987=((88598662500000.0)*pz*sj5);
CheckValue<IkReal> x988 = IKatan2WithCheck(IkReal((((py*x987))+((px*x986)))),IkReal(((((-1.0)*py*x986))+((px*x987)))),IKFAST_ATAN2_MAGTHRESH);
if(!x988.valid){
continue;
}
CheckValue<IkReal> x989=IKPowWithIntegerCheck(IKsign((((pp*x985))+(((-1.0)*x985*(pz*pz))))),-1);
if(!x989.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x988.value)+(((1.5707963267949)*(x989.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[5];
IkReal x990=IKsin(j4);
IkReal x991=IKcos(j4);
IkReal x992=((1.0)*cj5);
IkReal x993=((0.365572010101202)*cj7);
IkReal x994=((0.785876740156401)*sj5);
IkReal x995=(cj5*pz);
IkReal x996=(py*x990);
IkReal x997=(px*x991);
evalcond[0]=(((px*x990))+(((-1.0)*py*x991))+(((-0.365572010101202)*sj7)));
evalcond[1]=((((-1.0)*x992*x997))+(((-1.0)*x992*x996))+((pz*sj5)));
evalcond[2]=((-0.392938370078201)+(((-1.0)*x993))+((sj5*x997))+((sj5*x996))+x995);
evalcond[3]=((((-1.0)*x997))+(((-1.0)*x996))+((sj5*x993))+(((0.392938370078201)*sj5)));
evalcond[4]=((-0.0207576681102795)+((x994*x996))+((x994*x997))+(((-1.0)*pp))+(((0.785876740156401)*x995)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1000 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1000.valid){
continue;
}
IkReal x998=((1.0)*(x1000.value));
if((((px*px)+(py*py))) < -0.00001)
continue;
CheckValue<IkReal> x1001=IKPowWithIntegerCheck(IKabs(IKsqrt(((px*px)+(py*py)))),-1);
if(!x1001.valid){
continue;
}
if( (((0.365572010101202)*sj7*(x1001.value))) < -1-IKFAST_SINCOS_THRESH || (((0.365572010101202)*sj7*(x1001.value))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x999=IKasin(((0.365572010101202)*sj7*(x1001.value)));
j4array[0]=((((-1.0)*x998))+x999);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x999))+(((-1.0)*x998)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j5eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((1.07485901332934)+cj7);
j5eval[1]=((IKabs(pz))+(IKabs((((cj4*px))+((py*sj4))))));
j5eval[2]=IKsign(((0.392938370078201)+(((0.365572010101202)*cj7))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((-1.07485901332934)+(((-1.0)*cj7)));
j5eval[1]=((IKabs(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))))+(IKabs(pz)));
j5eval[2]=IKsign(((-0.392938370078201)+(((-0.365572010101202)*cj7))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
IkReal x1002=((1.0)*cj7);
IkReal x1003=(py*sj4);
IkReal x1004=(cj4*px);
j5eval[0]=((((-1.07485901332934)*x1003))+(((-1.07485901332934)*x1004))+(((-1.0)*x1002*x1003))+(((-1.0)*x1002*x1004)));
j5eval[1]=((-1.07485901332934)+(((-1.0)*x1002)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1005=(cj4*px);
IkReal x1006=((0.365572010101202)*cj7);
IkReal x1007=(py*sj4);
CheckValue<IkReal> x1008=IKPowWithIntegerCheck(((((-1.0)*x1006*x1007))+(((-1.0)*x1005*x1006))+(((-0.392938370078201)*x1005))+(((-0.392938370078201)*x1007))),-1);
if(!x1008.valid){
continue;
}
CheckValue<IkReal> x1009=IKPowWithIntegerCheck(((-0.392938370078201)+(((-1.0)*x1006))),-1);
if(!x1009.valid){
continue;
}
if( IKabs(((x1008.value)*(((-0.154400562679713)+(pz*pz)+(((-0.133642894569433)*(cj7*cj7)))+(((-0.287294539590756)*cj7)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*pz*(x1009.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1008.value)*(((-0.154400562679713)+(pz*pz)+(((-0.133642894569433)*(cj7*cj7)))+(((-0.287294539590756)*cj7))))))+IKsqr(((-1.0)*pz*(x1009.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j5array[0]=IKatan2(((x1008.value)*(((-0.154400562679713)+(pz*pz)+(((-0.133642894569433)*(cj7*cj7)))+(((-0.287294539590756)*cj7))))), ((-1.0)*pz*(x1009.value)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1010=IKcos(j5);
IkReal x1011=IKsin(j5);
IkReal x1012=((0.365572010101202)*cj7);
IkReal x1013=(py*sj4);
IkReal x1014=(cj4*px);
IkReal x1015=((0.785876740156401)*x1011);
IkReal x1016=(pz*x1010);
IkReal x1017=((1.0)*x1010);
evalcond[0]=((((-1.0)*x1010*x1012))+pz+(((-0.392938370078201)*x1010)));
evalcond[1]=((((0.392938370078201)*x1011))+((x1011*x1012))+(((-1.0)*x1013))+(((-1.0)*x1014)));
evalcond[2]=((((-1.0)*x1014*x1017))+((pz*x1011))+(((-1.0)*x1013*x1017)));
evalcond[3]=((-0.392938370078201)+x1016+(((-1.0)*x1012))+((x1011*x1014))+((x1011*x1013)));
evalcond[4]=((-0.0207576681102795)+((x1014*x1015))+(((0.785876740156401)*x1016))+((x1013*x1015))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1018 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))),IkReal(((-1.0)*pz)),IKFAST_ATAN2_MAGTHRESH);
if(!x1018.valid){
continue;
}
CheckValue<IkReal> x1019=IKPowWithIntegerCheck(IKsign(((-0.392938370078201)+(((-0.365572010101202)*cj7)))),-1);
if(!x1019.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1018.value)+(((1.5707963267949)*(x1019.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1020=IKcos(j5);
IkReal x1021=IKsin(j5);
IkReal x1022=((0.365572010101202)*cj7);
IkReal x1023=(py*sj4);
IkReal x1024=(cj4*px);
IkReal x1025=((0.785876740156401)*x1021);
IkReal x1026=(pz*x1020);
IkReal x1027=((1.0)*x1020);
evalcond[0]=((((-1.0)*x1020*x1022))+pz+(((-0.392938370078201)*x1020)));
evalcond[1]=((((-1.0)*x1023))+(((-1.0)*x1024))+(((0.392938370078201)*x1021))+((x1021*x1022)));
evalcond[2]=((((-1.0)*x1023*x1027))+(((-1.0)*x1024*x1027))+((pz*x1021)));
evalcond[3]=((-0.392938370078201)+x1026+(((-1.0)*x1022))+((x1021*x1024))+((x1021*x1023)));
evalcond[4]=((-0.0207576681102795)+((x1023*x1025))+(((-1.0)*pp))+((x1024*x1025))+(((0.785876740156401)*x1026)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1028=IKPowWithIntegerCheck(IKsign(((0.392938370078201)+(((0.365572010101202)*cj7)))),-1);
if(!x1028.valid){
continue;
}
CheckValue<IkReal> x1029 = IKatan2WithCheck(IkReal((((cj4*px))+((py*sj4)))),IkReal(pz),IKFAST_ATAN2_MAGTHRESH);
if(!x1029.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1028.value)))+(x1029.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1030=IKcos(j5);
IkReal x1031=IKsin(j5);
IkReal x1032=((0.365572010101202)*cj7);
IkReal x1033=(py*sj4);
IkReal x1034=(cj4*px);
IkReal x1035=((0.785876740156401)*x1031);
IkReal x1036=(pz*x1030);
IkReal x1037=((1.0)*x1030);
evalcond[0]=(pz+(((-1.0)*x1030*x1032))+(((-0.392938370078201)*x1030)));
evalcond[1]=(((x1031*x1032))+(((-1.0)*x1034))+(((-1.0)*x1033))+(((0.392938370078201)*x1031)));
evalcond[2]=((((-1.0)*x1033*x1037))+(((-1.0)*x1034*x1037))+((pz*x1031)));
evalcond[3]=((-0.392938370078201)+((x1031*x1034))+((x1031*x1033))+(((-1.0)*x1032))+x1036);
evalcond[4]=((-0.0207576681102795)+(((-1.0)*pp))+((x1034*x1035))+((x1033*x1035))+(((0.785876740156401)*x1036)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4, j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
IkReal x1038=py*py;
IkReal x1039=cj6*cj6;
IkReal x1040=sj6*sj6;
IkReal x1041=((1.0)*px*py);
IkReal x1042=(((x1038*x1039))+((x1038*x1040)));
IkReal x1043=((((-1.0)*x1039*x1041))+(((-1.0)*x1040*x1041)));
CheckValue<IkReal> x1046 = IKatan2WithCheck(IkReal(x1042),IkReal(x1043),IKFAST_ATAN2_MAGTHRESH);
if(!x1046.valid){
continue;
}
IkReal x1044=((1.0)*(x1046.value));
if((((x1042*x1042)+(x1043*x1043))) < -0.00001)
continue;
CheckValue<IkReal> x1047=IKPowWithIntegerCheck(IKabs(IKsqrt(((x1042*x1042)+(x1043*x1043)))),-1);
if(!x1047.valid){
continue;
}
if( (((0.365572010101202)*py*sj6*sj7*(x1047.value))) < -1-IKFAST_SINCOS_THRESH || (((0.365572010101202)*py*sj6*sj7*(x1047.value))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x1045=IKasin(((0.365572010101202)*py*sj6*sj7*(x1047.value)));
j4array[0]=(x1045+(((-1.0)*x1044)));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1045))+(((-1.0)*x1044)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x1048=IKcos(j4);
IkReal x1049=IKsin(j4);
IkReal x1050=cj6*cj6;
IkReal x1051=px*px;
IkReal x1052=sj6*sj6;
IkReal x1053=(px*py);
IkReal x1054=((1.0)*x1051);
IkReal x1055=((0.365572010101202)*sj6*sj7);
evalcond[0]=((((-1.0)*py*x1048))+x1055+((px*x1049)));
evalcond[1]=((((-1.0)*px*x1055))+((x1048*((((x1052*x1053))+((x1050*x1053))))))+((x1049*(((((-1.0)*x1052*x1054))+(((-1.0)*x1050*x1054)))))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j5eval[3];
IkReal x1056=(cj6*sj7);
IkReal x1057=(cj4*px);
IkReal x1058=(py*sj4);
IkReal x1059=((88598662500000.0)*pz);
IkReal x1060=(cj7*x1058);
j5eval[0]=(x1060+(((1.07485901332934)*x1057))+(((1.07485901332934)*x1058))+((cj7*x1057))+((pz*x1056)));
j5eval[1]=((IKabs(((((11840581711480.3)*cj7*x1056))+((x1057*x1059))+((x1058*x1059))+(((12726955975647.1)*x1056)))))+(IKabs(((13679683342670.0)+(((11840581711480.3)*(cj7*cj7)))+(((25453911951294.3)*cj7))+(((-1.0)*pz*x1059))))));
j5eval[2]=IKsign(((((32389191142403.0)*x1060))+(((32389191142403.0)*cj7*x1057))+(((34813814033858.6)*x1058))+(((34813814033858.6)*x1057))+(((32389191142403.0)*pz*x1056))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
IkReal x1061=(py*sj4);
IkReal x1062=(cj4*px);
IkReal x1063=((8.7454546344058e+27)*pz);
IkReal x1064=((4.06818686262725e+27)*pp);
IkReal x1065=((3.1970934299486e+27)*cj7);
IkReal x1066=(cj6*sj7);
j5eval[0]=(((cj7*x1061))+((cj7*x1062))+(((1.07485901332934)*x1062))+(((1.07485901332934)*x1061))+((pz*x1066)));
j5eval[1]=((IKabs((((x1061*x1063))+(((8.44460727050157e+25)*x1066))+((x1062*x1063))+((x1064*x1066)))))+(IKabs(((9.07676223872507e+25)+((cj7*x1064))+(((8.44460727050157e+25)*cj7))+(((4.3727273172029e+27)*pp))+(((-1.0)*pz*x1063))))));
j5eval[2]=IKsign(((((3.1970934299486e+27)*pz*x1066))+((x1061*x1065))+((x1062*x1065))+(((3.43642468963626e+27)*x1061))+(((3.43642468963626e+27)*x1062))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
IkReal x1067=cj6*cj6;
IkReal x1068=cj7*cj7;
IkReal x1069=((2.86963901467375e+27)*cj7);
IkReal x1070=(py*sj4);
IkReal x1071=(cj4*px);
IkReal x1072=((1.04905970285912e+27)*x1067);
IkReal x1073=((2.86963901467375e+27)*cj6*sj7);
j5eval[0]=((1.15532189853532)+x1068+x1067+(((-1.0)*x1067*x1068))+(((2.14971802665867)*cj7)));
j5eval[1]=((IKabs((((x1069*x1071))+((x1069*x1070))+(((3.0844573599236e+27)*x1071))+(((3.0844573599236e+27)*x1070))+(((-1.0)*pz*x1073)))))+(IKabs((((x1070*x1073))+((x1071*x1073))+((pz*x1069))+(((3.0844573599236e+27)*pz))))));
j5eval[2]=IKsign(((1.21200164758409e+27)+x1072+(((-1.0)*x1068*x1072))+(((1.04905970285912e+27)*x1068))+(((2.25518255427744e+27)*cj7))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(cj6))+(IKabs(((-1.0)+(IKsign(sj6)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((1.0)+(((48.1749681460985)*pp)));
j5eval[1]=IKsign(((0.026413389084589)+(((1.27246417778058)*pp))));
j5eval[2]=((IKabs(pz))+(IKabs((((cj4*px))+((py*sj4))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((-1.0)+(((-48.1749681460985)*pp)));
j5eval[1]=((IKabs(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))))+(IKabs(pz)));
j5eval[2]=IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
IkReal x1074=(cj4*px);
IkReal x1075=((48.1749681460985)*pp);
IkReal x1076=(py*sj4);
j5eval[0]=((((-1.0)*x1075*x1076))+(((-1.0)*x1074*x1075))+(((-1.0)*x1076))+(((-1.0)*x1074)));
j5eval[1]=((-1.0)+(((-1.0)*x1075)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1077=(cj4*px);
IkReal x1078=((1.27246417778058)*pp);
IkReal x1079=(py*sj4);
CheckValue<IkReal> x1080=IKPowWithIntegerCheck(((((-1.0)*x1077*x1078))+(((-1.0)*x1078*x1079))+(((-0.026413389084589)*x1077))+(((-0.026413389084589)*x1079))),-1);
if(!x1080.valid){
continue;
}
CheckValue<IkReal> x1081=IKPowWithIntegerCheck(((-0.026413389084589)+(((-1.0)*x1078))),-1);
if(!x1081.valid){
continue;
}
if( IKabs(((x1080.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*pz*(x1081.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1080.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))))+IKsqr(((-1.0)*pz*(x1081.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j5array[0]=IKatan2(((x1080.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))), ((-1.0)*pz*(x1081.value)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1082=IKcos(j5);
IkReal x1083=IKsin(j5);
IkReal x1084=(py*sj4);
IkReal x1085=(cj4*px);
IkReal x1086=((1.27246417778058)*pp);
IkReal x1087=((0.785876740156401)*x1083);
IkReal x1088=(pz*x1082);
evalcond[0]=(pz+(((-1.0)*x1082*x1086))+(((-0.026413389084589)*x1082)));
evalcond[1]=((((-1.0)*pz*x1083))+((x1082*x1084))+((x1082*x1085)));
evalcond[2]=((((-1.0)*x1085))+(((-1.0)*x1084))+(((0.026413389084589)*x1083))+((x1083*x1086)));
evalcond[3]=((-0.026413389084589)+x1088+(((-1.0)*x1086))+((x1083*x1084))+((x1083*x1085)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x1088))+(((-1.0)*pp))+((x1085*x1087))+((x1084*x1087)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1089=IKPowWithIntegerCheck(IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp)))),-1);
if(!x1089.valid){
continue;
}
CheckValue<IkReal> x1090 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))),IkReal(((-1.0)*pz)),IKFAST_ATAN2_MAGTHRESH);
if(!x1090.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1089.value)))+(x1090.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1091=IKcos(j5);
IkReal x1092=IKsin(j5);
IkReal x1093=(py*sj4);
IkReal x1094=(cj4*px);
IkReal x1095=((1.27246417778058)*pp);
IkReal x1096=((0.785876740156401)*x1092);
IkReal x1097=(pz*x1091);
evalcond[0]=((((-1.0)*x1091*x1095))+(((-0.026413389084589)*x1091))+pz);
evalcond[1]=(((x1091*x1094))+((x1091*x1093))+(((-1.0)*pz*x1092)));
evalcond[2]=((((-1.0)*x1093))+(((-1.0)*x1094))+(((0.026413389084589)*x1092))+((x1092*x1095)));
evalcond[3]=((-0.026413389084589)+x1097+(((-1.0)*x1095))+((x1092*x1094))+((x1092*x1093)));
evalcond[4]=((-0.0207576681102795)+((x1094*x1096))+(((-1.0)*pp))+((x1093*x1096))+(((0.785876740156401)*x1097)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1098 = IKatan2WithCheck(IkReal((((cj4*px))+((py*sj4)))),IkReal(pz),IKFAST_ATAN2_MAGTHRESH);
if(!x1098.valid){
continue;
}
CheckValue<IkReal> x1099=IKPowWithIntegerCheck(IKsign(((0.026413389084589)+(((1.27246417778058)*pp)))),-1);
if(!x1099.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1098.value)+(((1.5707963267949)*(x1099.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1100=IKcos(j5);
IkReal x1101=IKsin(j5);
IkReal x1102=(py*sj4);
IkReal x1103=(cj4*px);
IkReal x1104=((1.27246417778058)*pp);
IkReal x1105=((0.785876740156401)*x1101);
IkReal x1106=(pz*x1100);
evalcond[0]=(pz+(((-1.0)*x1100*x1104))+(((-0.026413389084589)*x1100)));
evalcond[1]=(((x1100*x1102))+((x1100*x1103))+(((-1.0)*pz*x1101)));
evalcond[2]=((((0.026413389084589)*x1101))+(((-1.0)*x1102))+(((-1.0)*x1103))+((x1101*x1104)));
evalcond[3]=((-0.026413389084589)+x1106+((x1101*x1102))+((x1101*x1103))+(((-1.0)*x1104)));
evalcond[4]=((-0.0207576681102795)+((x1102*x1105))+(((0.785876740156401)*x1106))+(((-1.0)*pp))+((x1103*x1105)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(cj6))+(IKabs(((1.0)+(IKsign(sj6)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((1.0)+(((48.1749681460985)*pp)));
j5eval[1]=IKsign(((0.026413389084589)+(((1.27246417778058)*pp))));
j5eval[2]=((IKabs(pz))+(IKabs((((cj4*px))+((py*sj4))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((-1.0)+(((-48.1749681460985)*pp)));
j5eval[1]=((IKabs(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))))+(IKabs(pz)));
j5eval[2]=IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
IkReal x1107=(cj4*px);
IkReal x1108=((48.1749681460985)*pp);
IkReal x1109=(py*sj4);
j5eval[0]=((((-1.0)*x1109))+(((-1.0)*x1107))+(((-1.0)*x1108*x1109))+(((-1.0)*x1107*x1108)));
j5eval[1]=((-1.0)+(((-1.0)*x1108)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1110=(cj4*px);
IkReal x1111=((1.27246417778058)*pp);
IkReal x1112=(py*sj4);
CheckValue<IkReal> x1113=IKPowWithIntegerCheck(((((-0.026413389084589)*x1112))+(((-0.026413389084589)*x1110))+(((-1.0)*x1110*x1111))+(((-1.0)*x1111*x1112))),-1);
if(!x1113.valid){
continue;
}
CheckValue<IkReal> x1114=IKPowWithIntegerCheck(((-0.026413389084589)+(((-1.0)*x1111))),-1);
if(!x1114.valid){
continue;
}
if( IKabs(((x1113.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*pz*(x1114.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1113.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))))+IKsqr(((-1.0)*pz*(x1114.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j5array[0]=IKatan2(((x1113.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))), ((-1.0)*pz*(x1114.value)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1115=IKcos(j5);
IkReal x1116=IKsin(j5);
IkReal x1117=((1.27246417778058)*pp);
IkReal x1118=(py*sj4);
IkReal x1119=(cj4*px);
IkReal x1120=((0.785876740156401)*x1116);
IkReal x1121=(pz*x1115);
IkReal x1122=((1.0)*x1115);
evalcond[0]=((((-0.026413389084589)*x1115))+pz+(((-1.0)*x1115*x1117)));
evalcond[1]=((((0.026413389084589)*x1116))+(((-1.0)*x1119))+(((-1.0)*x1118))+((x1116*x1117)));
evalcond[2]=(((pz*x1116))+(((-1.0)*x1119*x1122))+(((-1.0)*x1118*x1122)));
evalcond[3]=((-0.026413389084589)+x1121+((x1116*x1119))+((x1116*x1118))+(((-1.0)*x1117)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x1121))+((x1119*x1120))+(((-1.0)*pp))+((x1118*x1120)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1123=IKPowWithIntegerCheck(IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp)))),-1);
if(!x1123.valid){
continue;
}
CheckValue<IkReal> x1124 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))),IkReal(((-1.0)*pz)),IKFAST_ATAN2_MAGTHRESH);
if(!x1124.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1123.value)))+(x1124.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1125=IKcos(j5);
IkReal x1126=IKsin(j5);
IkReal x1127=((1.27246417778058)*pp);
IkReal x1128=(py*sj4);
IkReal x1129=(cj4*px);
IkReal x1130=((0.785876740156401)*x1126);
IkReal x1131=(pz*x1125);
IkReal x1132=((1.0)*x1125);
evalcond[0]=((((-0.026413389084589)*x1125))+pz+(((-1.0)*x1125*x1127)));
evalcond[1]=((((0.026413389084589)*x1126))+(((-1.0)*x1129))+(((-1.0)*x1128))+((x1126*x1127)));
evalcond[2]=(((pz*x1126))+(((-1.0)*x1128*x1132))+(((-1.0)*x1129*x1132)));
evalcond[3]=((-0.026413389084589)+x1131+(((-1.0)*x1127))+((x1126*x1129))+((x1126*x1128)));
evalcond[4]=((-0.0207576681102795)+((x1129*x1130))+(((0.785876740156401)*x1131))+(((-1.0)*pp))+((x1128*x1130)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1133 = IKatan2WithCheck(IkReal((((cj4*px))+((py*sj4)))),IkReal(pz),IKFAST_ATAN2_MAGTHRESH);
if(!x1133.valid){
continue;
}
CheckValue<IkReal> x1134=IKPowWithIntegerCheck(IKsign(((0.026413389084589)+(((1.27246417778058)*pp)))),-1);
if(!x1134.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1133.value)+(((1.5707963267949)*(x1134.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1135=IKcos(j5);
IkReal x1136=IKsin(j5);
IkReal x1137=((1.27246417778058)*pp);
IkReal x1138=(py*sj4);
IkReal x1139=(cj4*px);
IkReal x1140=((0.785876740156401)*x1136);
IkReal x1141=(pz*x1135);
IkReal x1142=((1.0)*x1135);
evalcond[0]=((((-0.026413389084589)*x1135))+pz+(((-1.0)*x1135*x1137)));
evalcond[1]=((((0.026413389084589)*x1136))+((x1136*x1137))+(((-1.0)*x1139))+(((-1.0)*x1138)));
evalcond[2]=(((pz*x1136))+(((-1.0)*x1139*x1142))+(((-1.0)*x1138*x1142)));
evalcond[3]=((-0.026413389084589)+x1141+((x1136*x1138))+((x1136*x1139))+(((-1.0)*x1137)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x1141))+(((-1.0)*pp))+((x1139*x1140))+((x1138*x1140)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1143=cj6*cj6;
IkReal x1144=cj7*cj7;
IkReal x1145=((2.86963901467375e+27)*cj7);
IkReal x1146=(py*sj4);
IkReal x1147=(cj4*px);
IkReal x1148=((1.04905970285912e+27)*x1143);
IkReal x1149=((2.86963901467375e+27)*cj6*sj7);
CheckValue<IkReal> x1150=IKPowWithIntegerCheck(IKsign(((1.21200164758409e+27)+x1148+(((1.04905970285912e+27)*x1144))+(((2.25518255427744e+27)*cj7))+(((-1.0)*x1144*x1148)))),-1);
if(!x1150.valid){
continue;
}
CheckValue<IkReal> x1151 = IKatan2WithCheck(IkReal((((x1145*x1146))+((x1145*x1147))+(((3.0844573599236e+27)*x1146))+(((3.0844573599236e+27)*x1147))+(((-1.0)*pz*x1149)))),IkReal((((x1146*x1149))+((pz*x1145))+(((3.0844573599236e+27)*pz))+((x1147*x1149)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1151.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1150.value)))+(x1151.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[6];
IkReal x1152=IKcos(j5);
IkReal x1153=IKsin(j5);
IkReal x1154=(py*sj4);
IkReal x1155=((0.365572010101202)*cj7);
IkReal x1156=(px*sj4);
IkReal x1157=((0.365572010101202)*sj7);
IkReal x1158=(cj4*px);
IkReal x1159=((1.0)*sj6);
IkReal x1160=(cj4*py);
IkReal x1161=((0.785876740156401)*x1153);
IkReal x1162=(sj6*x1152);
IkReal x1163=(pz*x1152);
IkReal x1164=(cj6*x1152);
IkReal x1165=(pz*x1153);
evalcond[0]=(((cj6*x1153*x1157))+pz+(((-1.0)*x1152*x1155))+(((-0.392938370078201)*x1152)));
evalcond[1]=((-0.392938370078201)+x1163+(((-1.0)*x1155))+((x1153*x1158))+((x1153*x1154)));
evalcond[2]=((-0.0207576681102795)+((x1154*x1161))+(((-1.0)*pp))+((x1158*x1161))+(((0.785876740156401)*x1163)));
evalcond[3]=(((x1157*x1164))+((x1153*x1155))+(((0.392938370078201)*x1153))+(((-1.0)*x1154))+(((-1.0)*x1158)));
evalcond[4]=((((-1.0)*x1159*x1165))+(((-1.0)*cj6*x1160))+((cj6*x1156))+((x1154*x1162))+((x1158*x1162)));
evalcond[5]=(x1157+(((-1.0)*x1154*x1164))+(((-1.0)*x1159*x1160))+((cj6*x1165))+(((-1.0)*x1158*x1164))+((sj6*x1156)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1166=(py*sj4);
IkReal x1167=(cj4*px);
IkReal x1168=((8.7454546344058e+27)*pz);
IkReal x1169=((4.06818686262725e+27)*pp);
IkReal x1170=((3.1970934299486e+27)*cj7);
IkReal x1171=(cj6*sj7);
CheckValue<IkReal> x1172 = IKatan2WithCheck(IkReal(((9.07676223872507e+25)+(((8.44460727050157e+25)*cj7))+(((-1.0)*pz*x1168))+(((4.3727273172029e+27)*pp))+((cj7*x1169)))),IkReal((((x1166*x1168))+(((8.44460727050157e+25)*x1171))+((x1167*x1168))+((x1169*x1171)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1172.valid){
continue;
}
CheckValue<IkReal> x1173=IKPowWithIntegerCheck(IKsign(((((3.1970934299486e+27)*pz*x1171))+((x1166*x1170))+(((3.43642468963626e+27)*x1167))+(((3.43642468963626e+27)*x1166))+((x1167*x1170)))),-1);
if(!x1173.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1172.value)+(((1.5707963267949)*(x1173.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[6];
IkReal x1174=IKcos(j5);
IkReal x1175=IKsin(j5);
IkReal x1176=(py*sj4);
IkReal x1177=((0.365572010101202)*cj7);
IkReal x1178=(px*sj4);
IkReal x1179=((0.365572010101202)*sj7);
IkReal x1180=(cj4*px);
IkReal x1181=((1.0)*sj6);
IkReal x1182=(cj4*py);
IkReal x1183=((0.785876740156401)*x1175);
IkReal x1184=(sj6*x1174);
IkReal x1185=(pz*x1174);
IkReal x1186=(cj6*x1174);
IkReal x1187=(pz*x1175);
evalcond[0]=(((cj6*x1175*x1179))+(((-0.392938370078201)*x1174))+pz+(((-1.0)*x1174*x1177)));
evalcond[1]=((-0.392938370078201)+x1185+((x1175*x1180))+((x1175*x1176))+(((-1.0)*x1177)));
evalcond[2]=((-0.0207576681102795)+(((0.785876740156401)*x1185))+((x1180*x1183))+((x1176*x1183))+(((-1.0)*pp)));
evalcond[3]=(((x1179*x1186))+((x1175*x1177))+(((-1.0)*x1180))+(((0.392938370078201)*x1175))+(((-1.0)*x1176)));
evalcond[4]=((((-1.0)*x1181*x1187))+(((-1.0)*cj6*x1182))+((x1180*x1184))+((x1176*x1184))+((cj6*x1178)));
evalcond[5]=((((-1.0)*x1181*x1182))+((cj6*x1187))+x1179+(((-1.0)*x1180*x1186))+((sj6*x1178))+(((-1.0)*x1176*x1186)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1188=(cj6*sj7);
IkReal x1189=(py*sj4);
IkReal x1190=((32389191142403.0)*cj7);
IkReal x1191=(cj4*px);
IkReal x1192=((88598662500000.0)*pz);
CheckValue<IkReal> x1193=IKPowWithIntegerCheck(IKsign((((x1189*x1190))+(((34813814033858.6)*x1191))+((x1190*x1191))+(((34813814033858.6)*x1189))+(((32389191142403.0)*pz*x1188)))),-1);
if(!x1193.valid){
continue;
}
CheckValue<IkReal> x1194 = IKatan2WithCheck(IkReal(((13679683342670.0)+(((11840581711480.3)*(cj7*cj7)))+(((25453911951294.3)*cj7))+(((-1.0)*pz*x1192)))),IkReal((((x1189*x1192))+(((12726955975647.1)*x1188))+((x1191*x1192))+(((11840581711480.3)*cj7*x1188)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1194.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1193.value)))+(x1194.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[6];
IkReal x1195=IKcos(j5);
IkReal x1196=IKsin(j5);
IkReal x1197=(py*sj4);
IkReal x1198=((0.365572010101202)*cj7);
IkReal x1199=(px*sj4);
IkReal x1200=((0.365572010101202)*sj7);
IkReal x1201=(cj4*px);
IkReal x1202=((1.0)*sj6);
IkReal x1203=(cj4*py);
IkReal x1204=((0.785876740156401)*x1196);
IkReal x1205=(sj6*x1195);
IkReal x1206=(pz*x1195);
IkReal x1207=(cj6*x1195);
IkReal x1208=(pz*x1196);
evalcond[0]=((((-1.0)*x1195*x1198))+(((-0.392938370078201)*x1195))+pz+((cj6*x1196*x1200)));
evalcond[1]=((-0.392938370078201)+x1206+(((-1.0)*x1198))+((x1196*x1197))+((x1196*x1201)));
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+((x1197*x1204))+(((0.785876740156401)*x1206))+((x1201*x1204)));
evalcond[3]=((((0.392938370078201)*x1196))+((x1200*x1207))+((x1196*x1198))+(((-1.0)*x1201))+(((-1.0)*x1197)));
evalcond[4]=(((cj6*x1199))+(((-1.0)*x1202*x1208))+((x1197*x1205))+((x1201*x1205))+(((-1.0)*cj6*x1203)));
evalcond[5]=(x1200+(((-1.0)*x1197*x1207))+(((-1.0)*x1202*x1203))+((sj6*x1199))+(((-1.0)*x1201*x1207))+((cj6*x1208)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

} else
{
{
IkReal j5array[2], cj5array[2], sj5array[2];
bool j5valid[2]={false};
_nj5 = 2;
IkReal x1209=((-0.392938370078201)+(((-0.365572010101202)*cj7)));
CheckValue<IkReal> x1212 = IKatan2WithCheck(IkReal(x1209),IkReal(((0.365572010101202)*cj6*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x1212.valid){
continue;
}
IkReal x1210=((1.0)*(x1212.value));
if((((x1209*x1209)+(((0.133642894569433)*(cj6*cj6)*(sj7*sj7))))) < -0.00001)
continue;
CheckValue<IkReal> x1213=IKPowWithIntegerCheck(IKabs(IKsqrt(((x1209*x1209)+(((0.133642894569433)*(cj6*cj6)*(sj7*sj7)))))),-1);
if(!x1213.valid){
continue;
}
if( ((pz*(x1213.value))) < -1-IKFAST_SINCOS_THRESH || ((pz*(x1213.value))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x1211=IKasin((pz*(x1213.value)));
j5array[0]=((((-1.0)*x1210))+(((-1.0)*x1211)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
j5array[1]=((3.14159265358979)+x1211+(((-1.0)*x1210)));
sj5array[1]=IKsin(j5array[1]);
cj5array[1]=IKcos(j5array[1]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
if( j5array[1] > IKPI )
{
    j5array[1]-=IK2PI;
}
else if( j5array[1] < -IKPI )
{    j5array[1]+=IK2PI;
}
j5valid[1] = true;
for(int ij5 = 0; ij5 < 2; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 2; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
IkReal j4eval[3];
IkReal x1214=pz*pz;
IkReal x1215=(pp*sj5);
IkReal x1216=((32389191142403.0)*cj7);
IkReal x1217=((88598662500000.0)*cj5*pz);
IkReal x1218=(sj5*x1214);
IkReal x1219=((32389191142403.0)*sj5*sj6*sj7);
j4eval[0]=(x1215+(((-1.0)*x1218)));
j4eval[1]=IKsign(((((88598662500000.0)*x1215))+(((-88598662500000.0)*x1218))));
j4eval[2]=((IKabs(((((34813814033858.6)*px))+((px*x1216))+(((-1.0)*px*x1217))+((py*x1219)))))+(IKabs(((((34813814033858.6)*py))+(((-1.0)*px*x1219))+((py*x1216))+(((-1.0)*py*x1217))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
IkReal x1220=pz*pz;
IkReal x1221=((1.11282777406866e+28)*pp);
IkReal x1222=(pp*sj5);
IkReal x1223=((8.7454546344058e+27)*cj5*pz);
IkReal x1224=(sj5*x1220);
IkReal x1225=((3.1970934299486e+27)*sj5*sj6*sj7);
j4eval[0]=(x1222+(((-1.0)*x1224)));
j4eval[1]=((IKabs(((((-1.0)*px*x1223))+((py*x1225))+(((2.30997095980183e+26)*px))+((px*x1221)))))+(IKabs(((((-1.0)*px*x1225))+((py*x1221))+(((2.30997095980183e+26)*py))+(((-1.0)*py*x1223))))));
j4eval[2]=IKsign(((((-8.7454546344058e+27)*x1224))+(((8.7454546344058e+27)*x1222))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
IkReal x1226=pz*pz;
IkReal x1227=(py*sj5);
IkReal x1228=((32389191142403.0)*cj7);
IkReal x1229=(cj5*cj6);
IkReal x1230=(px*sj5);
IkReal x1231=((32389191142403.0)*py*sj7);
IkReal x1232=((32389191142403.0)*px*sj7);
j4eval[0]=((((-1.0)*x1226))+pp);
j4eval[1]=((IKabs((((sj6*x1231))+(((34813814033858.6)*x1230))+((x1228*x1230))+((x1229*x1232)))))+(IKabs(((((-1.0)*sj6*x1232))+(((34813814033858.6)*x1227))+((x1227*x1228))+((x1229*x1231))))));
j4eval[2]=IKsign(((((-88598662500000.0)*x1226))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j5))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
sj5=0;
cj5=1.0;
j5=0;
IkReal x1233=pz*pz;
IkReal x1234=(cj6*sj7);
IkReal x1235=(sj6*sj7);
j4eval[0]=((((-1.0)*x1233))+pp);
j4eval[1]=((IKabs(((((-1.0)*px*x1235))+((py*x1234)))))+(IKabs((((py*x1235))+((px*x1234))))));
j4eval[2]=IKsign(((((-88598662500000.0)*x1233))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj5=0;
cj5=1.0;
j5=0;
IkReal x1236=pz*pz;
IkReal x1237=cj6*cj6;
IkReal x1238=((1.0)*cj6);
IkReal x1239=(py*sj7);
IkReal x1240=(px*sj7);
IkReal x1241=(cj6*x1236);
IkReal x1242=((1.0)*x1237);
j4eval[0]=(x1241+(((-1.0)*pp*x1238)));
j4eval[1]=IKsign(((((88598662500000.0)*x1241))+(((-88598662500000.0)*cj6*pp))));
j4eval[2]=((IKabs(((((-1.0)*x1239*x1242))+((cj6*sj6*x1240)))))+(IKabs(((((-1.0)*sj6*x1238*x1239))+(((-1.0)*x1240*x1242))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj5=0;
cj5=1.0;
j5=0;
IkReal x1243=cj6*cj6;
IkReal x1244=pz*pz;
IkReal x1245=(py*sj7);
IkReal x1246=(cj6*pp);
IkReal x1247=(cj6*sj6);
IkReal x1248=(px*sj7);
IkReal x1249=(cj6*x1244);
j4eval[0]=(x1246+(((-1.0)*x1249)));
j4eval[1]=((IKabs(((((-1.0)*x1247*x1248))+((x1243*x1245)))))+(IKabs((((x1245*x1247))+((x1243*x1248))))));
j4eval[2]=IKsign(((((88598662500000.0)*x1246))+(((-88598662500000.0)*x1249))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j6)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj5=0;
cj5=1.0;
j5=0;
sj6=1.0;
cj6=0;
j6=1.5707963267949;
IkReal x1250=pz*pz;
j4eval[0]=(x1250+(((-1.0)*pp)));
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((88598662500000.0)*x1250))+(((-88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj6=1.0;
cj6=0;
j6=1.5707963267949;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1252 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1252.valid){
continue;
}
IkReal x1251=x1252.value;
j4array[0]=((-1.0)*x1251);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1251)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj6=1.0;
cj6=0;
j6=1.5707963267949;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1254 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1254.valid){
continue;
}
IkReal x1253=x1254.value;
j4array[0]=((-1.0)*x1253);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1253)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x1255 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x1255.valid){
continue;
}
CheckValue<IkReal> x1256=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*pp))+(((88598662500000.0)*(pz*pz))))),-1);
if(!x1256.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1255.value)+(((1.5707963267949)*(x1256.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x1257=IKsin(j4);
IkReal x1258=IKcos(j4);
IkReal x1259=((1.0)*py);
evalcond[0]=((((-1.0)*px*x1258))+(((-1.0)*x1257*x1259)));
evalcond[1]=((((-1.0)*x1258*x1259))+(((0.365572010101202)*sj7))+((px*x1257)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j6)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj5=0;
cj5=1.0;
j5=0;
sj6=-1.0;
cj6=0;
j6=-1.5707963267949;
IkReal x1260=pz*pz;
j4eval[0]=(pp+(((-1.0)*x1260)));
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((-88598662500000.0)*x1260))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj6=-1.0;
cj6=0;
j6=-1.5707963267949;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1262 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1262.valid){
continue;
}
IkReal x1261=x1262.value;
j4array[0]=((-1.0)*x1261);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1261)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj6=-1.0;
cj6=0;
j6=-1.5707963267949;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1264 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1264.valid){
continue;
}
IkReal x1263=x1264.value;
j4array[0]=((-1.0)*x1263);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1263)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x1265 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x1265.valid){
continue;
}
CheckValue<IkReal> x1266=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x1266.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1265.value)+(((1.5707963267949)*(x1266.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x1267=IKsin(j4);
IkReal x1268=IKcos(j4);
IkReal x1269=((1.0)*py);
evalcond[0]=((((-1.0)*x1267*x1269))+(((-1.0)*px*x1268)));
evalcond[1]=((((-1.0)*x1268*x1269))+(((-0.365572010101202)*sj7))+((px*x1267)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs((((px*sj6))+(((-1.0)*cj6*py)))))+(IKabs((((cj6*px))+((py*sj6))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=1.0;
j7=0;
IkReal x1270=((1.0)*cj6);
j4eval[0]=((IKabs(((((-1.0)*py*x1270))+((px*sj6)))))+(IKabs(((((-1.0)*px*x1270))+(((-1.0)*py*sj6))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
IkReal x1271=((1.0)*cj6);
CheckValue<IkReal> x1273 = IKatan2WithCheck(IkReal(((((-1.0)*px*x1271))+(((-1.0)*py*sj6)))),IkReal(((((-1.0)*py*x1271))+((px*sj6)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1273.valid){
continue;
}
IkReal x1272=x1273.value;
j4array[0]=((-1.0)*x1272);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1272)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1274=IKsin(j4);
IkReal x1275=IKcos(j4);
IkReal x1276=(px*x1274);
IkReal x1277=(py*x1274);
IkReal x1278=(px*x1275);
IkReal x1279=((1.0)*py*x1275);
evalcond[0]=(x1276+(((-1.0)*x1279)));
evalcond[1]=((((-1.0)*x1277))+(((-1.0)*x1278)));
evalcond[2]=((((-1.0)*cj6*x1279))+((cj6*x1276))+((sj6*x1278))+((sj6*x1277)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1281 = IKatan2WithCheck(IkReal((((px*sj6))+(((-1.0)*cj6*py)))),IkReal((((cj6*px))+((py*sj6)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1281.valid){
continue;
}
IkReal x1280=x1281.value;
j4array[0]=((-1.0)*x1280);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1280)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1282=IKsin(j4);
IkReal x1283=IKcos(j4);
IkReal x1284=((1.0)*py);
IkReal x1285=(px*x1282);
IkReal x1286=((1.0)*px*x1283);
evalcond[0]=(x1285+(((-1.0)*x1283*x1284)));
evalcond[1]=((((-1.0)*x1286))+(((-1.0)*x1282*x1284)));
evalcond[2]=((((-1.0)*sj6*x1283*x1284))+(((-1.0)*cj6*x1286))+(((-1.0)*cj6*x1282*x1284))+((sj6*x1285)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1288 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1288.valid){
continue;
}
IkReal x1287=x1288.value;
j4array[0]=((-1.0)*x1287);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1287)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1289=IKsin(j4);
IkReal x1290=IKcos(j4);
IkReal x1291=((1.0)*py);
IkReal x1292=(sj6*x1289);
IkReal x1293=(sj6*x1290);
IkReal x1294=(cj6*x1289);
IkReal x1295=((1.0)*px*x1290);
evalcond[0]=((((-1.0)*x1295))+(((-1.0)*x1289*x1291)));
evalcond[1]=(((py*x1292))+(((-1.0)*cj6*x1290*x1291))+((px*x1293))+((px*x1294)));
evalcond[2]=((((-1.0)*cj6*x1295))+(((-1.0)*x1291*x1293))+(((-1.0)*x1291*x1294))+((px*x1292)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs((((px*sj6))+(((-1.0)*cj6*py)))))+(IKabs((((cj6*px))+((py*sj6))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=1.0;
j5=0;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
IkReal x1296=((1.0)*cj6);
j4eval[0]=((IKabs(((((-1.0)*py*x1296))+((px*sj6)))))+(IKabs(((((-1.0)*py*sj6))+(((-1.0)*px*x1296))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
IkReal x1297=((1.0)*cj6);
CheckValue<IkReal> x1299 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj6))+(((-1.0)*px*x1297)))),IkReal(((((-1.0)*py*x1297))+((px*sj6)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1299.valid){
continue;
}
IkReal x1298=x1299.value;
j4array[0]=((-1.0)*x1298);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1298)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1300=IKsin(j4);
IkReal x1301=IKcos(j4);
IkReal x1302=(px*x1300);
IkReal x1303=(py*x1300);
IkReal x1304=(px*x1301);
IkReal x1305=((1.0)*py*x1301);
evalcond[0]=(x1302+(((-1.0)*x1305)));
evalcond[1]=((((-1.0)*x1303))+(((-1.0)*x1304)));
evalcond[2]=((((-1.0)*cj6*x1305))+((cj6*x1302))+((sj6*x1304))+((sj6*x1303)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1307 = IKatan2WithCheck(IkReal((((px*sj6))+(((-1.0)*cj6*py)))),IkReal((((cj6*px))+((py*sj6)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1307.valid){
continue;
}
IkReal x1306=x1307.value;
j4array[0]=((-1.0)*x1306);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1306)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1308=IKsin(j4);
IkReal x1309=IKcos(j4);
IkReal x1310=((1.0)*py);
IkReal x1311=(px*x1308);
IkReal x1312=((1.0)*px*x1309);
evalcond[0]=(x1311+(((-1.0)*x1309*x1310)));
evalcond[1]=((((-1.0)*x1308*x1310))+(((-1.0)*x1312)));
evalcond[2]=((((-1.0)*cj6*x1312))+(((-1.0)*sj6*x1309*x1310))+((sj6*x1311))+(((-1.0)*cj6*x1308*x1310)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1314 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1314.valid){
continue;
}
IkReal x1313=x1314.value;
j4array[0]=((-1.0)*x1313);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1313)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1315=IKsin(j4);
IkReal x1316=IKcos(j4);
IkReal x1317=((1.0)*py);
IkReal x1318=(sj6*x1315);
IkReal x1319=(sj6*x1316);
IkReal x1320=(cj6*x1315);
IkReal x1321=((1.0)*px*x1316);
evalcond[0]=((((-1.0)*x1315*x1317))+(((-1.0)*x1321)));
evalcond[1]=(((py*x1318))+(((-1.0)*cj6*x1316*x1317))+((px*x1319))+((px*x1320)));
evalcond[2]=((((-1.0)*x1317*x1319))+((px*x1318))+(((-1.0)*cj6*x1321))+(((-1.0)*x1317*x1320)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1322=cj6*cj6;
IkReal x1323=((32389191142403.0)*sj7);
IkReal x1324=(cj6*sj6);
IkReal x1325=((88598662500000.0)*cj6);
CheckValue<IkReal> x1326 = IKatan2WithCheck(IkReal(((((-1.0)*px*x1323*x1324))+((py*x1322*x1323)))),IkReal((((py*x1323*x1324))+((px*x1322*x1323)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1326.valid){
continue;
}
CheckValue<IkReal> x1327=IKPowWithIntegerCheck(IKsign((((pp*x1325))+(((-1.0)*x1325*(pz*pz))))),-1);
if(!x1327.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1326.value)+(((1.5707963267949)*(x1327.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1328=IKsin(j4);
IkReal x1329=IKcos(j4);
IkReal x1330=((0.365572010101202)*sj7);
IkReal x1331=(py*sj6);
IkReal x1332=(px*x1328);
IkReal x1333=((1.0)*x1329);
IkReal x1334=((1.0)*py*x1328);
evalcond[0]=(x1332+(((-1.0)*py*x1333))+((sj6*x1330)));
evalcond[1]=((((-1.0)*px*x1333))+((cj6*x1330))+(((-1.0)*x1334)));
evalcond[2]=((((-1.0)*cj6*py*x1333))+((cj6*x1332))+((px*sj6*x1329))+((x1328*x1331)));
evalcond[3]=(x1330+((sj6*x1332))+(((-1.0)*x1331*x1333))+(((-1.0)*cj6*px*x1333))+(((-1.0)*cj6*x1334)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1335=cj6*cj6;
IkReal x1336=((32389191142403.0)*sj7);
IkReal x1337=(cj6*sj6);
IkReal x1338=((88598662500000.0)*cj6);
CheckValue<IkReal> x1339=IKPowWithIntegerCheck(IKsign(((((-1.0)*pp*x1338))+((x1338*(pz*pz))))),-1);
if(!x1339.valid){
continue;
}
CheckValue<IkReal> x1340 = IKatan2WithCheck(IkReal((((px*x1336*x1337))+(((-1.0)*py*x1335*x1336)))),IkReal(((((-1.0)*px*x1335*x1336))+(((-1.0)*py*x1336*x1337)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1340.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1339.value)))+(x1340.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1341=IKsin(j4);
IkReal x1342=IKcos(j4);
IkReal x1343=((0.365572010101202)*sj7);
IkReal x1344=(py*sj6);
IkReal x1345=(px*x1341);
IkReal x1346=((1.0)*x1342);
IkReal x1347=((1.0)*py*x1341);
evalcond[0]=(x1345+((sj6*x1343))+(((-1.0)*py*x1346)));
evalcond[1]=((((-1.0)*x1347))+(((-1.0)*px*x1346))+((cj6*x1343)));
evalcond[2]=(((px*sj6*x1342))+((x1341*x1344))+(((-1.0)*cj6*py*x1346))+((cj6*x1345)));
evalcond[3]=((((-1.0)*cj6*px*x1346))+x1343+((sj6*x1345))+(((-1.0)*cj6*x1347))+(((-1.0)*x1344*x1346)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1348=((32389191142403.0)*sj7);
CheckValue<IkReal> x1349=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x1349.valid){
continue;
}
CheckValue<IkReal> x1350 = IKatan2WithCheck(IkReal(((((-1.0)*px*sj6*x1348))+((cj6*py*x1348)))),IkReal((((py*sj6*x1348))+((cj6*px*x1348)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1350.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1349.value)))+(x1350.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1351=IKsin(j4);
IkReal x1352=IKcos(j4);
IkReal x1353=((0.365572010101202)*sj7);
IkReal x1354=(py*sj6);
IkReal x1355=(px*x1351);
IkReal x1356=((1.0)*x1352);
IkReal x1357=((1.0)*py*x1351);
evalcond[0]=(x1355+((sj6*x1353))+(((-1.0)*py*x1356)));
evalcond[1]=((((-1.0)*x1357))+(((-1.0)*px*x1356))+((cj6*x1353)));
evalcond[2]=((((-1.0)*cj6*py*x1356))+((x1351*x1354))+((px*sj6*x1352))+((cj6*x1355)));
evalcond[3]=(x1353+(((-1.0)*cj6*px*x1356))+(((-1.0)*x1354*x1356))+(((-1.0)*cj6*x1357))+((sj6*x1355)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j5)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[3];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
IkReal x1358=pz*pz;
IkReal x1359=(cj6*sj7);
IkReal x1360=(sj6*sj7);
j4eval[0]=(x1358+(((-1.0)*pp)));
j4eval[1]=((IKabs((((px*x1360))+((py*x1359)))))+(IKabs(((((-1.0)*py*x1360))+((px*x1359))))));
j4eval[2]=IKsign(((((88598662500000.0)*x1358))+(((-88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
IkReal x1361=pz*pz;
IkReal x1362=cj6*cj6;
IkReal x1363=(py*sj7);
IkReal x1364=((1.0)*cj6);
IkReal x1365=(px*sj7);
IkReal x1366=(cj6*x1361);
j4eval[0]=(x1366+(((-1.0)*pp*x1364)));
j4eval[1]=IKsign(((((88598662500000.0)*x1366))+(((-88598662500000.0)*cj6*pp))));
j4eval[2]=((IKabs(((((-1.0)*sj6*x1363*x1364))+((x1362*x1365)))))+(IKabs((((cj6*sj6*x1365))+((x1362*x1363))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[3];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
IkReal x1367=pz*pz;
IkReal x1368=cj6*cj6;
IkReal x1369=(py*sj7);
IkReal x1370=(cj6*sj6);
IkReal x1371=(px*sj7);
IkReal x1372=(cj6*pp);
IkReal x1373=((1.0)*x1368);
IkReal x1374=(cj6*x1367);
j4eval[0]=(x1372+(((-1.0)*x1374)));
j4eval[1]=IKsign(((((88598662500000.0)*x1372))+(((-88598662500000.0)*x1374))));
j4eval[2]=((IKabs(((((-1.0)*x1371*x1373))+((x1369*x1370)))))+(IKabs(((((-1.0)*x1370*x1371))+(((-1.0)*x1369*x1373))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j6)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj6=1.0;
cj6=0;
j6=1.5707963267949;
IkReal x1375=pz*pz;
j4eval[0]=(x1375+(((-1.0)*pp)));
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((88598662500000.0)*x1375))+(((-88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj6=1.0;
cj6=0;
j6=1.5707963267949;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1377 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1377.valid){
continue;
}
IkReal x1376=x1377.value;
j4array[0]=((-1.0)*x1376);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1376)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj6=1.0;
cj6=0;
j6=1.5707963267949;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1379 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1379.valid){
continue;
}
IkReal x1378=x1379.value;
j4array[0]=((-1.0)*x1378);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1378)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x1380 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x1380.valid){
continue;
}
CheckValue<IkReal> x1381=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*pp))+(((88598662500000.0)*(pz*pz))))),-1);
if(!x1381.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1380.value)+(((1.5707963267949)*(x1381.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x1382=IKsin(j4);
IkReal x1383=IKcos(j4);
IkReal x1384=((1.0)*py);
evalcond[0]=((((-1.0)*x1382*x1384))+(((-1.0)*px*x1383)));
evalcond[1]=((((-1.0)*x1383*x1384))+(((0.365572010101202)*sj7))+((px*x1382)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j6)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[4];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj6=-1.0;
cj6=0;
j6=-1.5707963267949;
IkReal x1385=pz*pz;
j4eval[0]=(pp+(((-1.0)*x1385)));
j4eval[1]=1.04905970285912e+27;
j4eval[2]=sj7;
j4eval[3]=IKsign(((((-88598662500000.0)*x1385))+(((88598662500000.0)*pp))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  || IKabs(j4eval[3]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj6=-1.0;
cj6=0;
j6=-1.5707963267949;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1387 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1387.valid){
continue;
}
IkReal x1386=x1387.value;
j4array[0]=((-1.0)*x1386);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1386)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj6=-1.0;
cj6=0;
j6=-1.5707963267949;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
continue; // 3 cases reached

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1389 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1389.valid){
continue;
}
IkReal x1388=x1389.value;
j4array[0]=((-1.0)*x1388);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1388)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[1];
evalcond[0]=((((-1.0)*py*(IKsin(j4))))+(((-1.0)*px*(IKcos(j4)))));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
CheckValue<IkReal> x1390 = IKatan2WithCheck(IkReal(((32389191142403.0)*px*sj7)),IkReal(((-32389191142403.0)*py*sj7)),IKFAST_ATAN2_MAGTHRESH);
if(!x1390.valid){
continue;
}
CheckValue<IkReal> x1391=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x1391.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1390.value)+(((1.5707963267949)*(x1391.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[2];
IkReal x1392=IKsin(j4);
IkReal x1393=IKcos(j4);
IkReal x1394=((1.0)*py);
evalcond[0]=((((-1.0)*x1392*x1394))+(((-1.0)*px*x1393)));
evalcond[1]=(((px*x1392))+(((-1.0)*x1393*x1394))+(((-0.365572010101202)*sj7)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j7))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=1.0;
j7=0;
j4eval[0]=((IKabs((((cj6*py))+((px*sj6)))))+(IKabs((((cj6*px))+(((-1.0)*py*sj6))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=1.0;
j7=0;
IkReal x1395=((1.0)*sj6);
j4eval[0]=((IKabs(((((-1.0)*px*x1395))+(((-1.0)*cj6*py)))))+(IKabs((((cj6*px))+(((-1.0)*py*x1395))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
IkReal x1396=((1.0)*py);
CheckValue<IkReal> x1398 = IKatan2WithCheck(IkReal(((((-1.0)*cj6*x1396))+(((-1.0)*px*sj6)))),IkReal(((((-1.0)*sj6*x1396))+((cj6*px)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1398.valid){
continue;
}
IkReal x1397=x1398.value;
j4array[0]=((-1.0)*x1397);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1397)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1399=IKsin(j4);
IkReal x1400=IKcos(j4);
IkReal x1401=((1.0)*py);
IkReal x1402=(px*x1399);
IkReal x1403=(px*x1400);
evalcond[0]=(x1402+(((-1.0)*x1400*x1401)));
evalcond[1]=((((-1.0)*x1403))+(((-1.0)*x1399*x1401)));
evalcond[2]=((((-1.0)*sj6*x1400*x1401))+((sj6*x1402))+((cj6*x1403))+((cj6*py*x1399)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1405 = IKatan2WithCheck(IkReal((((cj6*px))+(((-1.0)*py*sj6)))),IkReal((((cj6*py))+((px*sj6)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1405.valid){
continue;
}
IkReal x1404=x1405.value;
j4array[0]=((-1.0)*x1404);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1404)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1406=IKsin(j4);
IkReal x1407=IKcos(j4);
IkReal x1408=((1.0)*py);
IkReal x1409=(px*x1406);
IkReal x1410=((1.0)*px*x1407);
evalcond[0]=(x1409+(((-1.0)*x1407*x1408)));
evalcond[1]=((((-1.0)*x1410))+(((-1.0)*x1406*x1408)));
evalcond[2]=((((-1.0)*cj6*x1407*x1408))+((cj6*x1409))+(((-1.0)*sj6*x1406*x1408))+(((-1.0)*sj6*x1410)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1412 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1412.valid){
continue;
}
IkReal x1411=x1412.value;
j4array[0]=((-1.0)*x1411);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1411)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1413=IKsin(j4);
IkReal x1414=IKcos(j4);
IkReal x1415=((1.0)*py);
IkReal x1416=(px*x1413);
IkReal x1417=(px*x1414);
evalcond[0]=((((-1.0)*x1413*x1415))+(((-1.0)*x1417)));
evalcond[1]=(((cj6*x1417))+(((-1.0)*sj6*x1414*x1415))+((sj6*x1416))+((cj6*py*x1413)));
evalcond[2]=(((cj6*x1416))+(((-1.0)*cj6*x1414*x1415))+(((-1.0)*sj6*x1413*x1415))+(((-1.0)*sj6*x1417)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j7)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
j4eval[0]=((IKabs((((cj6*py))+((px*sj6)))))+(IKabs((((cj6*px))+(((-1.0)*py*sj6))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
sj5=0;
cj5=-1.0;
j5=3.14159265358979;
sj7=0;
cj7=-1.0;
j7=3.14159265358979;
IkReal x1418=((1.0)*sj6);
j4eval[0]=((IKabs((((cj6*px))+(((-1.0)*py*x1418)))))+(IKabs(((((-1.0)*px*x1418))+(((-1.0)*cj6*py))))));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(px))+(IKabs(py)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[4], cj4array[4], sj4array[4];
bool j4valid[4]={false};
_nj4 = 4;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=1.5707963267949;
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
j4array[2]=3.14159265358979;
sj4array[2]=IKsin(j4array[2]);
cj4array[2]=IKcos(j4array[2]);
j4array[3]=-1.5707963267949;
sj4array[3]=IKsin(j4array[3]);
cj4array[3]=IKcos(j4array[3]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
if( j4array[2] > IKPI )
{
    j4array[2]-=IK2PI;
}
else if( j4array[2] < -IKPI )
{    j4array[2]+=IK2PI;
}
j4valid[2] = true;
if( j4array[3] > IKPI )
{
    j4array[3]-=IK2PI;
}
else if( j4array[3] < -IKPI )
{    j4array[3]+=IK2PI;
}
j4valid[3] = true;
for(int ij4 = 0; ij4 < 4; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 4; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
IkReal x1419=((1.0)*py);
CheckValue<IkReal> x1421 = IKatan2WithCheck(IkReal(((((-1.0)*cj6*x1419))+(((-1.0)*px*sj6)))),IkReal((((cj6*px))+(((-1.0)*sj6*x1419)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1421.valid){
continue;
}
IkReal x1420=x1421.value;
j4array[0]=((-1.0)*x1420);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1420)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1422=IKsin(j4);
IkReal x1423=IKcos(j4);
IkReal x1424=((1.0)*py);
IkReal x1425=(px*x1422);
IkReal x1426=(px*x1423);
evalcond[0]=((((-1.0)*x1423*x1424))+x1425);
evalcond[1]=((((-1.0)*x1426))+(((-1.0)*x1422*x1424)));
evalcond[2]=(((cj6*py*x1422))+(((-1.0)*sj6*x1423*x1424))+((cj6*x1426))+((sj6*x1425)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1428 = IKatan2WithCheck(IkReal((((cj6*px))+(((-1.0)*py*sj6)))),IkReal((((cj6*py))+((px*sj6)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1428.valid){
continue;
}
IkReal x1427=x1428.value;
j4array[0]=((-1.0)*x1427);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1427)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1429=IKsin(j4);
IkReal x1430=IKcos(j4);
IkReal x1431=((1.0)*py);
IkReal x1432=(px*x1429);
IkReal x1433=((1.0)*px*x1430);
evalcond[0]=(x1432+(((-1.0)*x1430*x1431)));
evalcond[1]=((((-1.0)*x1429*x1431))+(((-1.0)*x1433)));
evalcond[2]=((((-1.0)*cj6*x1430*x1431))+(((-1.0)*sj6*x1429*x1431))+((cj6*x1432))+(((-1.0)*sj6*x1433)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1435 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1435.valid){
continue;
}
IkReal x1434=x1435.value;
j4array[0]=((-1.0)*x1434);
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1434)));
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[3];
IkReal x1436=IKsin(j4);
IkReal x1437=IKcos(j4);
IkReal x1438=((1.0)*py);
IkReal x1439=(px*x1436);
IkReal x1440=(px*x1437);
evalcond[0]=((((-1.0)*x1436*x1438))+(((-1.0)*x1440)));
evalcond[1]=(((sj6*x1439))+((cj6*x1440))+((cj6*py*x1436))+(((-1.0)*sj6*x1437*x1438)));
evalcond[2]=((((-1.0)*cj6*x1437*x1438))+(((-1.0)*sj6*x1436*x1438))+(((-1.0)*sj6*x1440))+((cj6*x1439)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1441=cj6*cj6;
IkReal x1442=((32389191142403.0)*sj7);
IkReal x1443=(cj6*sj6);
IkReal x1444=((88598662500000.0)*cj6);
CheckValue<IkReal> x1445 = IKatan2WithCheck(IkReal(((((-1.0)*px*x1442*x1443))+(((-1.0)*py*x1441*x1442)))),IkReal(((((-1.0)*px*x1441*x1442))+((py*x1442*x1443)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1445.valid){
continue;
}
CheckValue<IkReal> x1446=IKPowWithIntegerCheck(IKsign((((pp*x1444))+(((-1.0)*x1444*(pz*pz))))),-1);
if(!x1446.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1445.value)+(((1.5707963267949)*(x1446.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1447=IKsin(j4);
IkReal x1448=IKcos(j4);
IkReal x1449=((0.365572010101202)*sj7);
IkReal x1450=(px*x1447);
IkReal x1451=(py*x1447);
IkReal x1452=((1.0)*x1448);
evalcond[0]=(((sj6*x1449))+x1450+(((-1.0)*py*x1452)));
evalcond[1]=((((-1.0)*x1451))+(((-1.0)*px*x1452))+(((-1.0)*cj6*x1449)));
evalcond[2]=((((-1.0)*sj6*x1451))+(((-1.0)*px*sj6*x1452))+(((-1.0)*cj6*py*x1452))+((cj6*x1450)));
evalcond[3]=(((sj6*x1450))+x1449+((cj6*px*x1448))+(((-1.0)*py*sj6*x1452))+((cj6*x1451)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1453=cj6*cj6;
IkReal x1454=((32389191142403.0)*sj7);
IkReal x1455=(cj6*sj6);
IkReal x1456=((88598662500000.0)*cj6);
CheckValue<IkReal> x1457 = IKatan2WithCheck(IkReal((((py*x1453*x1454))+((px*x1454*x1455)))),IkReal((((px*x1453*x1454))+(((-1.0)*py*x1454*x1455)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1457.valid){
continue;
}
CheckValue<IkReal> x1458=IKPowWithIntegerCheck(IKsign(((((-1.0)*pp*x1456))+((x1456*(pz*pz))))),-1);
if(!x1458.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1457.value)+(((1.5707963267949)*(x1458.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1459=IKsin(j4);
IkReal x1460=IKcos(j4);
IkReal x1461=((0.365572010101202)*sj7);
IkReal x1462=(px*x1459);
IkReal x1463=(py*x1459);
IkReal x1464=((1.0)*x1460);
evalcond[0]=((((-1.0)*py*x1464))+((sj6*x1461))+x1462);
evalcond[1]=((((-1.0)*cj6*x1461))+(((-1.0)*px*x1464))+(((-1.0)*x1463)));
evalcond[2]=(((cj6*x1462))+(((-1.0)*sj6*x1463))+(((-1.0)*px*sj6*x1464))+(((-1.0)*cj6*py*x1464)));
evalcond[3]=((((-1.0)*py*sj6*x1464))+((cj6*x1463))+((cj6*px*x1460))+((sj6*x1462))+x1461);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1465=((32389191142403.0)*sj7);
CheckValue<IkReal> x1466=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*pp))+(((88598662500000.0)*(pz*pz))))),-1);
if(!x1466.valid){
continue;
}
CheckValue<IkReal> x1467 = IKatan2WithCheck(IkReal((((px*sj6*x1465))+((cj6*py*x1465)))),IkReal(((((-1.0)*py*sj6*x1465))+((cj6*px*x1465)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1467.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1466.value)))+(x1467.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1468=IKsin(j4);
IkReal x1469=IKcos(j4);
IkReal x1470=((0.365572010101202)*sj7);
IkReal x1471=(px*x1468);
IkReal x1472=(py*x1468);
IkReal x1473=((1.0)*x1469);
evalcond[0]=(((sj6*x1470))+(((-1.0)*py*x1473))+x1471);
evalcond[1]=((((-1.0)*px*x1473))+(((-1.0)*x1472))+(((-1.0)*cj6*x1470)));
evalcond[2]=((((-1.0)*px*sj6*x1473))+((cj6*x1471))+(((-1.0)*cj6*py*x1473))+(((-1.0)*sj6*x1472)));
evalcond[3]=(((sj6*x1471))+((cj6*px*x1469))+((cj6*x1472))+(((-1.0)*py*sj6*x1473))+x1470);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1474=(py*sj5);
IkReal x1475=((32389191142403.0)*cj7);
IkReal x1476=((32389191142403.0)*sj7);
IkReal x1477=(px*sj5);
IkReal x1478=(cj5*cj6);
CheckValue<IkReal> x1479 = IKatan2WithCheck(IkReal(((((-1.0)*px*sj6*x1476))+((x1474*x1475))+((py*x1476*x1478))+(((34813814033858.6)*x1474)))),IkReal(((((34813814033858.6)*x1477))+((py*sj6*x1476))+((px*x1476*x1478))+((x1475*x1477)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1479.valid){
continue;
}
CheckValue<IkReal> x1480=IKPowWithIntegerCheck(IKsign(((((-88598662500000.0)*(pz*pz)))+(((88598662500000.0)*pp)))),-1);
if(!x1480.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1479.value)+(((1.5707963267949)*(x1480.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[6];
IkReal x1481=IKsin(j4);
IkReal x1482=IKcos(j4);
IkReal x1483=((0.365572010101202)*cj7);
IkReal x1484=(py*sj6);
IkReal x1485=(cj5*pz);
IkReal x1486=((0.365572010101202)*sj7);
IkReal x1487=(cj6*py);
IkReal x1488=(cj5*cj6);
IkReal x1489=(pz*sj5);
IkReal x1490=((0.785876740156401)*sj5);
IkReal x1491=(px*x1481);
IkReal x1492=((1.0)*x1482);
IkReal x1493=(px*x1482);
IkReal x1494=(py*x1481);
IkReal x1495=(cj5*x1481);
evalcond[0]=(((sj6*x1486))+x1491+(((-1.0)*py*x1492)));
evalcond[1]=((-0.392938370078201)+((sj5*x1493))+((sj5*x1494))+(((-1.0)*x1483))+x1485);
evalcond[2]=((-0.0207576681102795)+(((0.785876740156401)*x1485))+(((-1.0)*pp))+((x1490*x1494))+((x1490*x1493)));
evalcond[3]=((((-1.0)*px*x1492))+(((-1.0)*x1494))+((x1486*x1488))+(((0.392938370078201)*sj5))+((sj5*x1483)));
evalcond[4]=((((-1.0)*x1487*x1492))+((cj5*sj6*x1493))+((x1484*x1495))+((cj6*x1491))+(((-1.0)*sj6*x1489)));
evalcond[5]=((((-1.0)*x1487*x1495))+(((-1.0)*x1484*x1492))+x1486+((sj6*x1491))+(((-1.0)*px*x1488*x1492))+((cj6*x1489)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1496=((1.11282777406866e+28)*pp);
IkReal x1497=((8.7454546344058e+27)*sj5);
IkReal x1498=((8.7454546344058e+27)*cj5*pz);
IkReal x1499=((3.1970934299486e+27)*sj5*sj6*sj7);
CheckValue<IkReal> x1500=IKPowWithIntegerCheck(IKsign((((pp*x1497))+(((-1.0)*x1497*(pz*pz))))),-1);
if(!x1500.valid){
continue;
}
CheckValue<IkReal> x1501 = IKatan2WithCheck(IkReal((((py*x1496))+(((-1.0)*px*x1499))+(((2.30997095980183e+26)*py))+(((-1.0)*py*x1498)))),IkReal((((py*x1499))+(((-1.0)*px*x1498))+(((2.30997095980183e+26)*px))+((px*x1496)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1501.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1500.value)))+(x1501.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[6];
IkReal x1502=IKsin(j4);
IkReal x1503=IKcos(j4);
IkReal x1504=((0.365572010101202)*cj7);
IkReal x1505=(py*sj6);
IkReal x1506=(cj5*pz);
IkReal x1507=((0.365572010101202)*sj7);
IkReal x1508=(cj6*py);
IkReal x1509=(cj5*cj6);
IkReal x1510=(pz*sj5);
IkReal x1511=((0.785876740156401)*sj5);
IkReal x1512=(px*x1502);
IkReal x1513=((1.0)*x1503);
IkReal x1514=(px*x1503);
IkReal x1515=(py*x1502);
IkReal x1516=(cj5*x1502);
evalcond[0]=(((sj6*x1507))+x1512+(((-1.0)*py*x1513)));
evalcond[1]=((-0.392938370078201)+(((-1.0)*x1504))+((sj5*x1514))+((sj5*x1515))+x1506);
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+((x1511*x1515))+((x1511*x1514))+(((0.785876740156401)*x1506)));
evalcond[3]=(((sj5*x1504))+(((-1.0)*x1515))+((x1507*x1509))+(((-1.0)*px*x1513))+(((0.392938370078201)*sj5)));
evalcond[4]=(((cj6*x1512))+((cj5*sj6*x1514))+(((-1.0)*sj6*x1510))+((x1505*x1516))+(((-1.0)*x1508*x1513)));
evalcond[5]=((((-1.0)*px*x1509*x1513))+((cj6*x1510))+((sj6*x1512))+x1507+(((-1.0)*x1508*x1516))+(((-1.0)*x1505*x1513)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1517=((88598662500000.0)*sj5);
IkReal x1518=((32389191142403.0)*px);
IkReal x1519=((32389191142403.0)*py);
IkReal x1520=((88598662500000.0)*cj5*pz);
IkReal x1521=(sj5*sj6*sj7);
CheckValue<IkReal> x1522 = IKatan2WithCheck(IkReal(((((34813814033858.6)*py))+(((-1.0)*x1518*x1521))+(((-1.0)*py*x1520))+((cj7*x1519)))),IkReal((((x1519*x1521))+(((34813814033858.6)*px))+(((-1.0)*px*x1520))+((cj7*x1518)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1522.valid){
continue;
}
CheckValue<IkReal> x1523=IKPowWithIntegerCheck(IKsign(((((-1.0)*x1517*(pz*pz)))+((pp*x1517)))),-1);
if(!x1523.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(x1522.value)+(((1.5707963267949)*(x1523.value))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[6];
IkReal x1524=IKsin(j4);
IkReal x1525=IKcos(j4);
IkReal x1526=((0.365572010101202)*cj7);
IkReal x1527=(py*sj6);
IkReal x1528=(cj5*pz);
IkReal x1529=((0.365572010101202)*sj7);
IkReal x1530=(cj6*py);
IkReal x1531=(cj5*cj6);
IkReal x1532=(pz*sj5);
IkReal x1533=((0.785876740156401)*sj5);
IkReal x1534=(px*x1524);
IkReal x1535=((1.0)*x1525);
IkReal x1536=(px*x1525);
IkReal x1537=(py*x1524);
IkReal x1538=(cj5*x1524);
evalcond[0]=(x1534+((sj6*x1529))+(((-1.0)*py*x1535)));
evalcond[1]=((-0.392938370078201)+((sj5*x1536))+((sj5*x1537))+(((-1.0)*x1526))+x1528);
evalcond[2]=((-0.0207576681102795)+(((0.785876740156401)*x1528))+(((-1.0)*pp))+((x1533*x1536))+((x1533*x1537)));
evalcond[3]=((((-1.0)*x1537))+((sj5*x1526))+((x1529*x1531))+(((-1.0)*px*x1535))+(((0.392938370078201)*sj5)));
evalcond[4]=((((-1.0)*x1530*x1535))+((cj6*x1534))+((x1527*x1538))+((cj5*sj6*x1536))+(((-1.0)*sj6*x1532)));
evalcond[5]=((((-1.0)*x1530*x1538))+(((-1.0)*px*x1531*x1535))+((cj6*x1532))+x1529+((sj6*x1534))+(((-1.0)*x1527*x1535)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x1541 = IKatan2WithCheck(IkReal(((-1.0)*py)),IkReal(px),IKFAST_ATAN2_MAGTHRESH);
if(!x1541.valid){
continue;
}
IkReal x1539=((1.0)*(x1541.value));
if((((px*px)+(py*py))) < -0.00001)
continue;
CheckValue<IkReal> x1542=IKPowWithIntegerCheck(IKabs(IKsqrt(((px*px)+(py*py)))),-1);
if(!x1542.valid){
continue;
}
if( (((0.365572010101202)*sj6*sj7*(x1542.value))) < -1-IKFAST_SINCOS_THRESH || (((0.365572010101202)*sj6*sj7*(x1542.value))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x1540=IKasin(((0.365572010101202)*sj6*sj7*(x1542.value)));
j4array[0]=((((-1.0)*x1540))+(((-1.0)*x1539)));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
j4array[1]=((3.14159265358979)+(((-1.0)*x1539))+x1540);
sj4array[1]=IKsin(j4array[1]);
cj4array[1]=IKcos(j4array[1]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
if( j4array[1] > IKPI )
{
    j4array[1]-=IK2PI;
}
else if( j4array[1] < -IKPI )
{    j4array[1]+=IK2PI;
}
j4valid[1] = true;
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j5eval[3];
IkReal x1543=(cj6*sj7);
IkReal x1544=(cj4*px);
IkReal x1545=(py*sj4);
IkReal x1546=((88598662500000.0)*pz);
IkReal x1547=(cj7*x1545);
j5eval[0]=((((1.07485901332934)*x1545))+(((1.07485901332934)*x1544))+x1547+((cj7*x1544))+((pz*x1543)));
j5eval[1]=((IKabs(((((11840581711480.3)*cj7*x1543))+((x1544*x1546))+(((12726955975647.1)*x1543))+((x1545*x1546)))))+(IKabs(((13679683342670.0)+(((11840581711480.3)*(cj7*cj7)))+(((25453911951294.3)*cj7))+(((-1.0)*pz*x1546))))));
j5eval[2]=IKsign(((((32389191142403.0)*x1547))+(((32389191142403.0)*pz*x1543))+(((32389191142403.0)*cj7*x1544))+(((34813814033858.6)*x1544))+(((34813814033858.6)*x1545))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
IkReal x1548=(py*sj4);
IkReal x1549=(cj4*px);
IkReal x1550=((8.7454546344058e+27)*pz);
IkReal x1551=((4.06818686262725e+27)*pp);
IkReal x1552=((3.1970934299486e+27)*cj7);
IkReal x1553=(cj6*sj7);
j5eval[0]=((((1.07485901332934)*x1548))+(((1.07485901332934)*x1549))+((pz*x1553))+((cj7*x1549))+((cj7*x1548)));
j5eval[1]=((IKabs(((9.07676223872507e+25)+(((-1.0)*pz*x1550))+(((8.44460727050157e+25)*cj7))+(((4.3727273172029e+27)*pp))+((cj7*x1551)))))+(IKabs((((x1551*x1553))+((x1549*x1550))+(((8.44460727050157e+25)*x1553))+((x1548*x1550))))));
j5eval[2]=IKsign(((((3.43642468963626e+27)*x1548))+(((3.43642468963626e+27)*x1549))+((x1549*x1552))+(((3.1970934299486e+27)*pz*x1553))+((x1548*x1552))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
IkReal x1554=cj6*cj6;
IkReal x1555=cj7*cj7;
IkReal x1556=((2.86963901467375e+27)*cj7);
IkReal x1557=(py*sj4);
IkReal x1558=(cj4*px);
IkReal x1559=((1.04905970285912e+27)*x1554);
IkReal x1560=((2.86963901467375e+27)*cj6*sj7);
j5eval[0]=((1.15532189853532)+x1555+x1554+(((2.14971802665867)*cj7))+(((-1.0)*x1554*x1555)));
j5eval[1]=((IKabs(((((3.0844573599236e+27)*pz))+((x1557*x1560))+((pz*x1556))+((x1558*x1560)))))+(IKabs((((x1556*x1558))+((x1556*x1557))+(((3.0844573599236e+27)*x1557))+(((3.0844573599236e+27)*x1558))+(((-1.0)*pz*x1560))))));
j5eval[2]=IKsign(((1.21200164758409e+27)+(((-1.0)*x1555*x1559))+x1559+(((2.25518255427744e+27)*cj7))+(((1.04905970285912e+27)*x1555))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(cj6))+(IKabs(((-1.0)+(IKsign(sj6)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((1.0)+(((48.1749681460985)*pp)));
j5eval[1]=IKsign(((0.026413389084589)+(((1.27246417778058)*pp))));
j5eval[2]=((IKabs(pz))+(IKabs((((cj4*px))+((py*sj4))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
j5eval[0]=((-1.0)+(((-48.1749681460985)*pp)));
j5eval[1]=((IKabs(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))))+(IKabs(pz)));
j5eval[2]=IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj6=1.0;
cj6=0;
j6=1.57079633263667;
IkReal x1561=(cj4*px);
IkReal x1562=((48.1749681460985)*pp);
IkReal x1563=(py*sj4);
j5eval[0]=((((-1.0)*x1562*x1563))+(((-1.0)*x1561))+(((-1.0)*x1563))+(((-1.0)*x1561*x1562)));
j5eval[1]=((-1.0)+(((-1.0)*x1562)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1564=(cj4*px);
IkReal x1565=((1.27246417778058)*pp);
IkReal x1566=(py*sj4);
CheckValue<IkReal> x1567=IKPowWithIntegerCheck(((((-1.0)*x1565*x1566))+(((-0.026413389084589)*x1564))+(((-0.026413389084589)*x1566))+(((-1.0)*x1564*x1565))),-1);
if(!x1567.valid){
continue;
}
CheckValue<IkReal> x1568=IKPowWithIntegerCheck(((-0.026413389084589)+(((-1.0)*x1565))),-1);
if(!x1568.valid){
continue;
}
if( IKabs(((x1567.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*pz*(x1568.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1567.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))))+IKsqr(((-1.0)*pz*(x1568.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j5array[0]=IKatan2(((x1567.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))), ((-1.0)*pz*(x1568.value)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1569=IKcos(j5);
IkReal x1570=IKsin(j5);
IkReal x1571=(py*sj4);
IkReal x1572=(cj4*px);
IkReal x1573=((1.27246417778058)*pp);
IkReal x1574=((0.785876740156401)*x1570);
IkReal x1575=(pz*x1569);
evalcond[0]=((((-1.0)*x1569*x1573))+(((-0.026413389084589)*x1569))+pz);
evalcond[1]=(((x1569*x1572))+((x1569*x1571))+(((-1.0)*pz*x1570)));
evalcond[2]=(((x1570*x1573))+(((0.026413389084589)*x1570))+(((-1.0)*x1572))+(((-1.0)*x1571)));
evalcond[3]=((-0.026413389084589)+((x1570*x1572))+((x1570*x1571))+x1575+(((-1.0)*x1573)));
evalcond[4]=((-0.0207576681102795)+((x1571*x1574))+((x1572*x1574))+(((0.785876740156401)*x1575))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1576=IKPowWithIntegerCheck(IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp)))),-1);
if(!x1576.valid){
continue;
}
CheckValue<IkReal> x1577 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))),IkReal(((-1.0)*pz)),IKFAST_ATAN2_MAGTHRESH);
if(!x1577.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1576.value)))+(x1577.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1578=IKcos(j5);
IkReal x1579=IKsin(j5);
IkReal x1580=(py*sj4);
IkReal x1581=(cj4*px);
IkReal x1582=((1.27246417778058)*pp);
IkReal x1583=((0.785876740156401)*x1579);
IkReal x1584=(pz*x1578);
evalcond[0]=((((-1.0)*x1578*x1582))+pz+(((-0.026413389084589)*x1578)));
evalcond[1]=((((-1.0)*pz*x1579))+((x1578*x1580))+((x1578*x1581)));
evalcond[2]=((((0.026413389084589)*x1579))+(((-1.0)*x1581))+(((-1.0)*x1580))+((x1579*x1582)));
evalcond[3]=((-0.026413389084589)+(((-1.0)*x1582))+x1584+((x1579*x1581))+((x1579*x1580)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x1584))+(((-1.0)*pp))+((x1580*x1583))+((x1581*x1583)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1585 = IKatan2WithCheck(IkReal((((cj4*px))+((py*sj4)))),IkReal(pz),IKFAST_ATAN2_MAGTHRESH);
if(!x1585.valid){
continue;
}
CheckValue<IkReal> x1586=IKPowWithIntegerCheck(IKsign(((0.026413389084589)+(((1.27246417778058)*pp)))),-1);
if(!x1586.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1585.value)+(((1.5707963267949)*(x1586.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1587=IKcos(j5);
IkReal x1588=IKsin(j5);
IkReal x1589=(py*sj4);
IkReal x1590=(cj4*px);
IkReal x1591=((1.27246417778058)*pp);
IkReal x1592=((0.785876740156401)*x1588);
IkReal x1593=(pz*x1587);
evalcond[0]=((((-0.026413389084589)*x1587))+pz+(((-1.0)*x1587*x1591)));
evalcond[1]=(((x1587*x1589))+((x1587*x1590))+(((-1.0)*pz*x1588)));
evalcond[2]=(((x1588*x1591))+(((-1.0)*x1589))+(((0.026413389084589)*x1588))+(((-1.0)*x1590)));
evalcond[3]=((-0.026413389084589)+(((-1.0)*x1591))+((x1588*x1590))+((x1588*x1589))+x1593);
evalcond[4]=((-0.0207576681102795)+((x1590*x1592))+(((0.785876740156401)*x1593))+((x1589*x1592))+(((-1.0)*pp)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(cj6))+(IKabs(((1.0)+(IKsign(sj6)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((1.0)+(((48.1749681460985)*pp)));
j5eval[1]=IKsign(((0.026413389084589)+(((1.27246417778058)*pp))));
j5eval[2]=((IKabs(pz))+(IKabs((((cj4*px))+((py*sj4))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
j5eval[0]=((-1.0)+(((-48.1749681460985)*pp)));
j5eval[1]=((IKabs(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))))+(IKabs(pz)));
j5eval[2]=IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj6=-1.0;
cj6=0;
j6=-1.57079633263667;
IkReal x1594=(cj4*px);
IkReal x1595=((48.1749681460985)*pp);
IkReal x1596=(py*sj4);
j5eval[0]=((((-1.0)*x1594*x1595))+(((-1.0)*x1595*x1596))+(((-1.0)*x1594))+(((-1.0)*x1596)));
j5eval[1]=((-1.0)+(((-1.0)*x1595)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1597=(cj4*px);
IkReal x1598=((1.27246417778058)*pp);
IkReal x1599=(py*sj4);
CheckValue<IkReal> x1600=IKPowWithIntegerCheck(((((-0.026413389084589)*x1597))+(((-0.026413389084589)*x1599))+(((-1.0)*x1598*x1599))+(((-1.0)*x1597*x1598))),-1);
if(!x1600.valid){
continue;
}
CheckValue<IkReal> x1601=IKPowWithIntegerCheck(((-0.026413389084589)+(((-1.0)*x1598))),-1);
if(!x1601.valid){
continue;
}
if( IKabs(((x1600.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*pz*(x1601.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1600.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))))+IKsqr(((-1.0)*pz*(x1601.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j5array[0]=IKatan2(((x1600.value)*(((-0.000697667122933887)+(((-1.61916508373482)*(pp*pp)))+(((-0.0672201828478404)*pp))+(pz*pz)))), ((-1.0)*pz*(x1601.value)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1602=IKcos(j5);
IkReal x1603=IKsin(j5);
IkReal x1604=((1.27246417778058)*pp);
IkReal x1605=(py*sj4);
IkReal x1606=(cj4*px);
IkReal x1607=((0.785876740156401)*x1603);
IkReal x1608=(pz*x1602);
IkReal x1609=((1.0)*x1602);
evalcond[0]=((((-0.026413389084589)*x1602))+pz+(((-1.0)*x1602*x1604)));
evalcond[1]=((((-1.0)*x1606))+(((-1.0)*x1605))+(((0.026413389084589)*x1603))+((x1603*x1604)));
evalcond[2]=((((-1.0)*x1605*x1609))+(((-1.0)*x1606*x1609))+((pz*x1603)));
evalcond[3]=((-0.026413389084589)+(((-1.0)*x1604))+x1608+((x1603*x1605))+((x1603*x1606)));
evalcond[4]=((-0.0207576681102795)+(((-1.0)*pp))+((x1605*x1607))+(((0.785876740156401)*x1608))+((x1606*x1607)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1610=IKPowWithIntegerCheck(IKsign(((-0.026413389084589)+(((-1.27246417778058)*pp)))),-1);
if(!x1610.valid){
continue;
}
CheckValue<IkReal> x1611 = IKatan2WithCheck(IkReal(((((-1.0)*py*sj4))+(((-1.0)*cj4*px)))),IkReal(((-1.0)*pz)),IKFAST_ATAN2_MAGTHRESH);
if(!x1611.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1610.value)))+(x1611.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1612=IKcos(j5);
IkReal x1613=IKsin(j5);
IkReal x1614=((1.27246417778058)*pp);
IkReal x1615=(py*sj4);
IkReal x1616=(cj4*px);
IkReal x1617=((0.785876740156401)*x1613);
IkReal x1618=(pz*x1612);
IkReal x1619=((1.0)*x1612);
evalcond[0]=((((-0.026413389084589)*x1612))+pz+(((-1.0)*x1612*x1614)));
evalcond[1]=((((-1.0)*x1615))+(((-1.0)*x1616))+(((0.026413389084589)*x1613))+((x1613*x1614)));
evalcond[2]=(((pz*x1613))+(((-1.0)*x1616*x1619))+(((-1.0)*x1615*x1619)));
evalcond[3]=((-0.026413389084589)+x1618+(((-1.0)*x1614))+((x1613*x1616))+((x1613*x1615)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x1618))+((x1615*x1617))+(((-1.0)*pp))+((x1616*x1617)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x1620 = IKatan2WithCheck(IkReal((((cj4*px))+((py*sj4)))),IkReal(pz),IKFAST_ATAN2_MAGTHRESH);
if(!x1620.valid){
continue;
}
CheckValue<IkReal> x1621=IKPowWithIntegerCheck(IKsign(((0.026413389084589)+(((1.27246417778058)*pp)))),-1);
if(!x1621.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1620.value)+(((1.5707963267949)*(x1621.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[5];
IkReal x1622=IKcos(j5);
IkReal x1623=IKsin(j5);
IkReal x1624=((1.27246417778058)*pp);
IkReal x1625=(py*sj4);
IkReal x1626=(cj4*px);
IkReal x1627=((0.785876740156401)*x1623);
IkReal x1628=(pz*x1622);
IkReal x1629=((1.0)*x1622);
evalcond[0]=((((-0.026413389084589)*x1622))+(((-1.0)*x1622*x1624))+pz);
evalcond[1]=((((-1.0)*x1625))+(((-1.0)*x1626))+((x1623*x1624))+(((0.026413389084589)*x1623)));
evalcond[2]=(((pz*x1623))+(((-1.0)*x1625*x1629))+(((-1.0)*x1626*x1629)));
evalcond[3]=((-0.026413389084589)+x1628+((x1623*x1625))+((x1623*x1626))+(((-1.0)*x1624)));
evalcond[4]=((-0.0207576681102795)+(((0.785876740156401)*x1628))+((x1626*x1627))+(((-1.0)*pp))+((x1625*x1627)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1630=cj6*cj6;
IkReal x1631=cj7*cj7;
IkReal x1632=((2.86963901467375e+27)*cj7);
IkReal x1633=(py*sj4);
IkReal x1634=(cj4*px);
IkReal x1635=((1.04905970285912e+27)*x1630);
IkReal x1636=((2.86963901467375e+27)*cj6*sj7);
CheckValue<IkReal> x1637=IKPowWithIntegerCheck(IKsign(((1.21200164758409e+27)+x1635+(((1.04905970285912e+27)*x1631))+(((2.25518255427744e+27)*cj7))+(((-1.0)*x1631*x1635)))),-1);
if(!x1637.valid){
continue;
}
CheckValue<IkReal> x1638 = IKatan2WithCheck(IkReal(((((-1.0)*pz*x1636))+(((3.0844573599236e+27)*x1634))+(((3.0844573599236e+27)*x1633))+((x1632*x1633))+((x1632*x1634)))),IkReal((((pz*x1632))+(((3.0844573599236e+27)*pz))+((x1634*x1636))+((x1633*x1636)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1638.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1637.value)))+(x1638.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[6];
IkReal x1639=IKcos(j5);
IkReal x1640=IKsin(j5);
IkReal x1641=(py*sj4);
IkReal x1642=((0.365572010101202)*cj7);
IkReal x1643=(px*sj4);
IkReal x1644=((0.365572010101202)*sj7);
IkReal x1645=(cj4*px);
IkReal x1646=((1.0)*sj6);
IkReal x1647=(cj4*py);
IkReal x1648=((0.785876740156401)*x1640);
IkReal x1649=(sj6*x1639);
IkReal x1650=(pz*x1639);
IkReal x1651=(cj6*x1639);
IkReal x1652=(pz*x1640);
evalcond[0]=((((-0.392938370078201)*x1639))+(((-1.0)*x1639*x1642))+pz+((cj6*x1640*x1644)));
evalcond[1]=((-0.392938370078201)+((x1640*x1641))+((x1640*x1645))+x1650+(((-1.0)*x1642)));
evalcond[2]=((-0.0207576681102795)+((x1645*x1648))+((x1641*x1648))+(((-1.0)*pp))+(((0.785876740156401)*x1650)));
evalcond[3]=(((x1640*x1642))+((x1644*x1651))+(((-1.0)*x1641))+(((-1.0)*x1645))+(((0.392938370078201)*x1640)));
evalcond[4]=((((-1.0)*cj6*x1647))+(((-1.0)*x1646*x1652))+((x1645*x1649))+((cj6*x1643))+((x1641*x1649)));
evalcond[5]=((((-1.0)*x1646*x1647))+x1644+(((-1.0)*x1641*x1651))+((cj6*x1652))+(((-1.0)*x1645*x1651))+((sj6*x1643)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1653=(py*sj4);
IkReal x1654=(cj4*px);
IkReal x1655=((8.7454546344058e+27)*pz);
IkReal x1656=((4.06818686262725e+27)*pp);
IkReal x1657=((3.1970934299486e+27)*cj7);
IkReal x1658=(cj6*sj7);
CheckValue<IkReal> x1659=IKPowWithIntegerCheck(IKsign((((x1654*x1657))+(((3.43642468963626e+27)*x1653))+(((3.43642468963626e+27)*x1654))+(((3.1970934299486e+27)*pz*x1658))+((x1653*x1657)))),-1);
if(!x1659.valid){
continue;
}
CheckValue<IkReal> x1660 = IKatan2WithCheck(IkReal(((9.07676223872507e+25)+(((-1.0)*pz*x1655))+(((8.44460727050157e+25)*cj7))+(((4.3727273172029e+27)*pp))+((cj7*x1656)))),IkReal((((x1656*x1658))+((x1654*x1655))+(((8.44460727050157e+25)*x1658))+((x1653*x1655)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1660.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1659.value)))+(x1660.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[6];
IkReal x1661=IKcos(j5);
IkReal x1662=IKsin(j5);
IkReal x1663=(py*sj4);
IkReal x1664=((0.365572010101202)*cj7);
IkReal x1665=(px*sj4);
IkReal x1666=((0.365572010101202)*sj7);
IkReal x1667=(cj4*px);
IkReal x1668=((1.0)*sj6);
IkReal x1669=(cj4*py);
IkReal x1670=((0.785876740156401)*x1662);
IkReal x1671=(sj6*x1661);
IkReal x1672=(pz*x1661);
IkReal x1673=(cj6*x1661);
IkReal x1674=(pz*x1662);
evalcond[0]=((((-1.0)*x1661*x1664))+(((-0.392938370078201)*x1661))+pz+((cj6*x1662*x1666)));
evalcond[1]=((-0.392938370078201)+x1672+((x1662*x1663))+((x1662*x1667))+(((-1.0)*x1664)));
evalcond[2]=((-0.0207576681102795)+(((0.785876740156401)*x1672))+((x1667*x1670))+(((-1.0)*pp))+((x1663*x1670)));
evalcond[3]=(((x1666*x1673))+(((-1.0)*x1663))+(((-1.0)*x1667))+((x1662*x1664))+(((0.392938370078201)*x1662)));
evalcond[4]=((((-1.0)*cj6*x1669))+(((-1.0)*x1668*x1674))+((cj6*x1665))+((x1667*x1671))+((x1663*x1671)));
evalcond[5]=((((-1.0)*x1668*x1669))+(((-1.0)*x1663*x1673))+x1666+((cj6*x1674))+(((-1.0)*x1667*x1673))+((sj6*x1665)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1675=(cj6*sj7);
IkReal x1676=(py*sj4);
IkReal x1677=((32389191142403.0)*cj7);
IkReal x1678=(cj4*px);
IkReal x1679=((88598662500000.0)*pz);
CheckValue<IkReal> x1680=IKPowWithIntegerCheck(IKsign(((((34813814033858.6)*x1676))+(((34813814033858.6)*x1678))+((x1676*x1677))+(((32389191142403.0)*pz*x1675))+((x1677*x1678)))),-1);
if(!x1680.valid){
continue;
}
CheckValue<IkReal> x1681 = IKatan2WithCheck(IkReal(((13679683342670.0)+(((11840581711480.3)*(cj7*cj7)))+(((25453911951294.3)*cj7))+(((-1.0)*pz*x1679)))),IkReal((((x1676*x1679))+((x1678*x1679))+(((12726955975647.1)*x1675))+(((11840581711480.3)*cj7*x1675)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1681.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1680.value)))+(x1681.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[6];
IkReal x1682=IKcos(j5);
IkReal x1683=IKsin(j5);
IkReal x1684=(py*sj4);
IkReal x1685=((0.365572010101202)*cj7);
IkReal x1686=(px*sj4);
IkReal x1687=((0.365572010101202)*sj7);
IkReal x1688=(cj4*px);
IkReal x1689=((1.0)*sj6);
IkReal x1690=(cj4*py);
IkReal x1691=((0.785876740156401)*x1683);
IkReal x1692=(sj6*x1682);
IkReal x1693=(pz*x1682);
IkReal x1694=(cj6*x1682);
IkReal x1695=(pz*x1683);
evalcond[0]=(((cj6*x1683*x1687))+(((-1.0)*x1682*x1685))+pz+(((-0.392938370078201)*x1682)));
evalcond[1]=((-0.392938370078201)+(((-1.0)*x1685))+x1693+((x1683*x1684))+((x1683*x1688)));
evalcond[2]=((-0.0207576681102795)+(((-1.0)*pp))+(((0.785876740156401)*x1693))+((x1688*x1691))+((x1684*x1691)));
evalcond[3]=((((-1.0)*x1688))+(((-1.0)*x1684))+(((0.392938370078201)*x1683))+((x1687*x1694))+((x1683*x1685)));
evalcond[4]=(((cj6*x1686))+(((-1.0)*cj6*x1690))+((x1688*x1692))+((x1684*x1692))+(((-1.0)*x1689*x1695)));
evalcond[5]=(x1687+(((-1.0)*x1684*x1694))+((sj6*x1686))+(((-1.0)*x1688*x1694))+(((-1.0)*x1689*x1690))+((cj6*x1695)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}
}
}
}
return solutions.GetNumSolutions()>0;
}
inline void rotationfunction0(IkSolutionListBase<IkReal>& solutions) {
for(int rotationiter = 0; rotationiter < 1; ++rotationiter) {
IkReal x97=(sj5*sj6);
IkReal x98=(cj6*cj7);
IkReal x99=(cj4*sj6);
IkReal x100=((1.0)*cj5);
IkReal x101=((1.0)*sj5);
IkReal x102=((1.0)*sj4);
IkReal x103=(cj7*sj6);
IkReal x104=(cj6*sj7);
IkReal x105=((((-1.0)*sj7*x101))+((cj5*x98)));
IkReal x106=((((-1.0)*sj4*sj6*x100))+((cj4*cj6)));
IkReal x107=(((cj5*x104))+((cj7*sj5)));
IkReal x108=((((-1.0)*x101*x104))+((cj5*cj7)));
IkReal x109=(cj4*x105);
IkReal x110=((((-1.0)*sj7*x100))+(((-1.0)*x101*x98)));
IkReal x111=((((-1.0)*cj6*x102))+(((-1.0)*x100*x99)));
IkReal x112=(((cj7*x99))+((sj4*x105)));
IkReal x113=((((-1.0)*sj6*sj7*x102))+((cj4*x107)));
IkReal x114=(((sj7*x99))+((sj4*x107)));
IkReal x115=((((-1.0)*sj4*x103))+x109);
new_r00=(((r10*x112))+((r20*x110))+((r00*x115)));
new_r01=(((r21*x110))+((r11*x112))+((r01*x115)));
new_r02=(((r22*x110))+((r02*((x109+(((-1.0)*x102*x103))))))+((r12*x112)));
new_r10=(((r10*x106))+((r20*x97))+((r00*x111)));
new_r11=(((r21*x97))+((r11*x106))+((r01*x111)));
new_r12=(((r02*x111))+((r12*x106))+((r22*x97)));
new_r20=(((r10*x114))+((r20*x108))+((r00*x113)));
new_r21=(((r21*x108))+((r11*x114))+((r01*x113)));
new_r22=(((r02*x113))+((r22*x108))+((r12*x114)));
{
IkReal j9array[2], cj9array[2], sj9array[2];
bool j9valid[2]={false};
_nj9 = 2;
cj9array[0]=new_r22;
if( cj9array[0] >= -1-IKFAST_SINCOS_THRESH && cj9array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j9valid[0] = j9valid[1] = true;
    j9array[0] = IKacos(cj9array[0]);
    sj9array[0] = IKsin(j9array[0]);
    cj9array[1] = cj9array[0];
    j9array[1] = -j9array[0];
    sj9array[1] = -sj9array[0];
}
else if( isnan(cj9array[0]) )
{
    // probably any value will work
    j9valid[0] = true;
    cj9array[0] = 1; sj9array[0] = 0; j9array[0] = 0;
}
for(int ij9 = 0; ij9 < 2; ++ij9)
{
if( !j9valid[ij9] )
{
    continue;
}
_ij9[0] = ij9; _ij9[1] = -1;
for(int iij9 = ij9+1; iij9 < 2; ++iij9)
{
if( j9valid[iij9] && IKabs(cj9array[ij9]-cj9array[iij9]) < IKFAST_SOLUTION_THRESH && IKabs(sj9array[ij9]-sj9array[iij9]) < IKFAST_SOLUTION_THRESH )
{
    j9valid[iij9]=false; _ij9[1] = iij9; break; 
}
}
j9 = j9array[ij9]; cj9 = cj9array[ij9]; sj9 = sj9array[ij9];

{
IkReal j8eval[3];
j8eval[0]=sj9;
j8eval[1]=((IKabs(new_r12))+(IKabs(new_r02)));
j8eval[2]=IKsign(sj9);
if( IKabs(j8eval[0]) < 0.0000010000000000  || IKabs(j8eval[1]) < 0.0000010000000000  || IKabs(j8eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[3];
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j8eval[2];
j8eval[0]=new_r12;
j8eval[1]=sj9;
if( IKabs(j8eval[0]) < 0.0000010000000000  || IKabs(j8eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
IkReal j10mul = 1;
j10=0;
j8mul=-1.0;
if( IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r01))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8=IKatan2(((-1.0)*new_r01), new_r00);
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].fmul = j8mul;
vinfos[4].freeind = 0;
vinfos[4].maxsolutions = 0;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].fmul = j10mul;
vinfos[6].freeind = 0;
vinfos[6].maxsolutions = 0;
std::vector<int> vfree(1);
vfree[0] = 6;
solutions.AddSolution(vinfos,vfree);
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
IkReal j10mul = 1;
j10=0;
j8mul=1.0;
if( IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r01))+IKsqr(((-1.0)*new_r00))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8=IKatan2(((-1.0)*new_r01), ((-1.0)*new_r00));
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].fmul = j8mul;
vinfos[4].freeind = 0;
vinfos[4].maxsolutions = 0;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].fmul = j10mul;
vinfos[6].freeind = 0;
vinfos[6].maxsolutions = 0;
std::vector<int> vfree(1);
vfree[0] = 6;
solutions.AddSolution(vinfos,vfree);
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r12))+(IKabs(new_r02)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j8eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
IkReal x116=new_r22*new_r22;
IkReal x117=((16.0)*new_r10);
IkReal x118=((16.0)*new_r01);
IkReal x119=((16.0)*new_r22);
IkReal x120=((8.0)*new_r11);
IkReal x121=((8.0)*new_r00);
IkReal x122=(x116*x117);
IkReal x123=(x116*x118);
j8eval[0]=((IKabs((((new_r22*x120))+(((-1.0)*x121)))))+(IKabs(((((32.0)*new_r11))+(((-1.0)*new_r00*x119))+(((-16.0)*new_r11*x116)))))+(IKabs((((new_r11*x119))+(((16.0)*new_r00))+(((-32.0)*new_r00*x116)))))+(IKabs(((((-1.0)*x118))+x123)))+(IKabs(((((-1.0)*x122))+x117)))+(IKabs(((((-1.0)*x123))+x118)))+(IKabs(((((-1.0)*x117))+x122)))+(IKabs((((x116*x120))+(((-1.0)*new_r22*x121))))));
if( IKabs(j8eval[0]) < 0.0000000100000000  )
{
continue; // no branches [j8, j10]

} else
{
IkReal op[4+1], zeror[4];
int numroots;
IkReal j8evalpoly[1];
IkReal x124=new_r22*new_r22;
IkReal x125=((16.0)*new_r10);
IkReal x126=(new_r11*new_r22);
IkReal x127=(x124*x125);
IkReal x128=((((-8.0)*new_r00))+(((8.0)*x126)));
op[0]=x128;
op[1]=((((-1.0)*x127))+x125);
op[2]=((((16.0)*x126))+(((16.0)*new_r00))+(((-32.0)*new_r00*x124)));
op[3]=((((-1.0)*x125))+x127);
op[4]=x128;
polyroots4(op,zeror,numroots);
IkReal j8array[4], cj8array[4], sj8array[4], tempj8array[1];
int numsolutions = 0;
for(int ij8 = 0; ij8 < numroots; ++ij8)
{
IkReal htj8 = zeror[ij8];
tempj8array[0]=((2.0)*(atan(htj8)));
for(int kj8 = 0; kj8 < 1; ++kj8)
{
j8array[numsolutions] = tempj8array[kj8];
if( j8array[numsolutions] > IKPI )
{
    j8array[numsolutions]-=IK2PI;
}
else if( j8array[numsolutions] < -IKPI )
{
    j8array[numsolutions]+=IK2PI;
}
sj8array[numsolutions] = IKsin(j8array[numsolutions]);
cj8array[numsolutions] = IKcos(j8array[numsolutions]);
numsolutions++;
}
}
bool j8valid[4]={true,true,true,true};
_nj8 = 4;
for(int ij8 = 0; ij8 < numsolutions; ++ij8)
    {
if( !j8valid[ij8] )
{
    continue;
}
    j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
htj8 = IKtan(j8/2);

IkReal x129=new_r22*new_r22;
IkReal x130=((16.0)*new_r01);
IkReal x131=(new_r00*new_r22);
IkReal x132=((8.0)*x131);
IkReal x133=(new_r11*x129);
IkReal x134=((8.0)*x133);
IkReal x135=(x129*x130);
j8evalpoly[0]=(((htj8*((x130+(((-1.0)*x135))))))+x134+(((-1.0)*x132))+(((htj8*htj8*htj8*htj8)*((x134+(((-1.0)*x132))))))+(((htj8*htj8)*(((((32.0)*new_r11))+(((-16.0)*x131))+(((-16.0)*x133))))))+(((htj8*htj8*htj8)*((x135+(((-1.0)*x130)))))));
if( IKabs(j8evalpoly[0]) > 0.0000001000000000  )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < numsolutions; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
{
IkReal j10eval[3];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
IkReal x136=cj8*cj8;
IkReal x137=(cj8*new_r22);
IkReal x138=((-1.0)+(((-1.0)*x136*(new_r22*new_r22)))+x136);
j10eval[0]=x138;
j10eval[1]=IKsign(x138);
j10eval[2]=((IKabs((((new_r01*x137))+((new_r00*sj8)))))+(IKabs((((new_r01*sj8))+(((-1.0)*new_r00*x137))))));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j10eval[0]=new_r22;
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10eval[2];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
IkReal x139=new_r22*new_r22;
j10eval[0]=(((cj8*x139))+(((-1.0)*cj8)));
j10eval[1]=((((-1.0)*sj8))+((sj8*x139)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j8)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r01))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r01));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x140=IKsin(j10);
IkReal x141=IKcos(j10);
evalcond[0]=x140;
evalcond[1]=((-1.0)*x141);
evalcond[2]=((((-1.0)*x140))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*x141))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j8)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(new_r01)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r00, new_r01);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x142=IKsin(j10);
IkReal x143=IKcos(j10);
evalcond[0]=x142;
evalcond[1]=((-1.0)*x143);
evalcond[2]=((((-1.0)*x142))+new_r00);
evalcond[3]=((((-1.0)*x143))+new_r01);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j8))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r10)+IKsqr(new_r11)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r10, new_r11);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x144=IKsin(j10);
IkReal x145=IKcos(j10);
evalcond[0]=x144;
evalcond[1]=((-1.0)*x145);
evalcond[2]=((((-1.0)*x144))+new_r10);
evalcond[3]=((((-1.0)*x145))+new_r11);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j8)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r10), ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x146=IKsin(j10);
IkReal x147=IKcos(j10);
evalcond[0]=x146;
evalcond[1]=((-1.0)*x147);
evalcond[2]=((((-1.0)*x146))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x147))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x148=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x148.valid){
continue;
}
if((x148.value) < -0.00001)
continue;
IkReal gconst12=((-1.0)*(IKsqrt(x148.value)));
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs((cj8+(((-1.0)*gconst12)))))+(IKabs(((-1.0)+(IKsign(sj8)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst12*gconst12))))) < -0.00001)
continue;
sj8=IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12)))));
cj8=gconst12;
if( (gconst12) < -1-IKFAST_SINCOS_THRESH || (gconst12) > 1+IKFAST_SINCOS_THRESH )
    continue;
j8=IKacos(gconst12);
CheckValue<IkReal> x149=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x149.valid){
continue;
}
if((x149.value) < -0.00001)
continue;
IkReal gconst12=((-1.0)*(IKsqrt(x149.value)));
j10eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if((((1.0)+(((-1.0)*(gconst12*gconst12))))) < -0.00001)
continue;
CheckValue<IkReal> x150=IKPowWithIntegerCheck(gconst12,-1);
if(!x150.valid){
continue;
}
if( IKabs(((((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12))))))))+((gconst12*new_r10)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x150.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12))))))))+((gconst12*new_r10))))+IKsqr((new_r11*(x150.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12))))))))+((gconst12*new_r10))), (new_r11*(x150.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x151=IKcos(j10);
IkReal x152=IKsin(j10);
IkReal x153=((1.0)*gconst12);
if((((1.0)+(((-1.0)*gconst12*x153)))) < -0.00001)
continue;
IkReal x154=IKsqrt(((1.0)+(((-1.0)*gconst12*x153))));
IkReal x155=((1.0)*x154);
evalcond[0]=x152;
evalcond[1]=((-1.0)*x151);
evalcond[2]=((((-1.0)*x151*x153))+new_r11);
evalcond[3]=((((-1.0)*x152*x153))+new_r10);
evalcond[4]=(((x151*x154))+new_r01);
evalcond[5]=(((x152*x154))+new_r00);
evalcond[6]=((((-1.0)*x152))+(((-1.0)*new_r00*x155))+((gconst12*new_r10)));
evalcond[7]=((((-1.0)*x151))+(((-1.0)*new_r01*x155))+((gconst12*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x156 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x156.valid){
continue;
}
CheckValue<IkReal> x157=IKPowWithIntegerCheck(IKsign(gconst12),-1);
if(!x157.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x156.value)+(((1.5707963267949)*(x157.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x158=IKcos(j10);
IkReal x159=IKsin(j10);
IkReal x160=((1.0)*gconst12);
if((((1.0)+(((-1.0)*gconst12*x160)))) < -0.00001)
continue;
IkReal x161=IKsqrt(((1.0)+(((-1.0)*gconst12*x160))));
IkReal x162=((1.0)*x161);
evalcond[0]=x159;
evalcond[1]=((-1.0)*x158);
evalcond[2]=((((-1.0)*x158*x160))+new_r11);
evalcond[3]=((((-1.0)*x159*x160))+new_r10);
evalcond[4]=(new_r01+((x158*x161)));
evalcond[5]=(new_r00+((x159*x161)));
evalcond[6]=((((-1.0)*new_r00*x162))+(((-1.0)*x159))+((gconst12*new_r10)));
evalcond[7]=((((-1.0)*x158))+(((-1.0)*new_r01*x162))+((gconst12*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x163=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x163.valid){
continue;
}
if((x163.value) < -0.00001)
continue;
IkReal gconst12=((-1.0)*(IKsqrt(x163.value)));
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.0)+(IKsign(sj8)))))+(IKabs((cj8+(((-1.0)*gconst12)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst12*gconst12))))) < -0.00001)
continue;
sj8=((-1.0)*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12)))))));
cj8=gconst12;
if( (gconst12) < -1-IKFAST_SINCOS_THRESH || (gconst12) > 1+IKFAST_SINCOS_THRESH )
    continue;
j8=((-1.0)*(IKacos(gconst12)));
CheckValue<IkReal> x164=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x164.valid){
continue;
}
if((x164.value) < -0.00001)
continue;
IkReal gconst12=((-1.0)*(IKsqrt(x164.value)));
j10eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if((((1.0)+(((-1.0)*(gconst12*gconst12))))) < -0.00001)
continue;
CheckValue<IkReal> x165=IKPowWithIntegerCheck(gconst12,-1);
if(!x165.valid){
continue;
}
if( IKabs((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12))))))))+((gconst12*new_r10)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x165.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12))))))))+((gconst12*new_r10))))+IKsqr((new_r11*(x165.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst12*gconst12))))))))+((gconst12*new_r10))), (new_r11*(x165.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x166=IKcos(j10);
IkReal x167=IKsin(j10);
IkReal x168=((1.0)*gconst12);
IkReal x169=((1.0)*x166);
IkReal x170=((1.0)*x167);
if((((1.0)+(((-1.0)*gconst12*x168)))) < -0.00001)
continue;
IkReal x171=IKsqrt(((1.0)+(((-1.0)*gconst12*x168))));
evalcond[0]=x167;
evalcond[1]=((-1.0)*x166);
evalcond[2]=((((-1.0)*x166*x168))+new_r11);
evalcond[3]=(new_r10+(((-1.0)*x167*x168)));
evalcond[4]=(new_r01+(((-1.0)*x169*x171)));
evalcond[5]=((((-1.0)*x170*x171))+new_r00);
evalcond[6]=(((new_r00*x171))+(((-1.0)*x170))+((gconst12*new_r10)));
evalcond[7]=(((new_r01*x171))+(((-1.0)*x169))+((gconst12*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x172 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x172.valid){
continue;
}
CheckValue<IkReal> x173=IKPowWithIntegerCheck(IKsign(gconst12),-1);
if(!x173.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x172.value)+(((1.5707963267949)*(x173.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x174=IKcos(j10);
IkReal x175=IKsin(j10);
IkReal x176=((1.0)*gconst12);
IkReal x177=((1.0)*x174);
IkReal x178=((1.0)*x175);
if((((1.0)+(((-1.0)*gconst12*x176)))) < -0.00001)
continue;
IkReal x179=IKsqrt(((1.0)+(((-1.0)*gconst12*x176))));
evalcond[0]=x175;
evalcond[1]=((-1.0)*x174);
evalcond[2]=((((-1.0)*x174*x176))+new_r11);
evalcond[3]=((((-1.0)*x175*x176))+new_r10);
evalcond[4]=((((-1.0)*x177*x179))+new_r01);
evalcond[5]=((((-1.0)*x178*x179))+new_r00);
evalcond[6]=(((new_r00*x179))+(((-1.0)*x178))+((gconst12*new_r10)));
evalcond[7]=(((new_r01*x179))+(((-1.0)*x177))+((gconst12*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x180=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x180.valid){
continue;
}
if((x180.value) < -0.00001)
continue;
IkReal gconst13=IKsqrt(x180.value);
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.0)+(IKsign(sj8)))))+(IKabs((cj8+(((-1.0)*gconst13)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
sj8=IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))));
cj8=gconst13;
if( (gconst13) < -1-IKFAST_SINCOS_THRESH || (gconst13) > 1+IKFAST_SINCOS_THRESH )
    continue;
j8=IKacos(gconst13);
CheckValue<IkReal> x181=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x181.valid){
continue;
}
if((x181.value) < -0.00001)
continue;
IkReal gconst13=IKsqrt(x181.value);
j10eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
CheckValue<IkReal> x182=IKPowWithIntegerCheck(gconst13,-1);
if(!x182.valid){
continue;
}
if( IKabs((((gconst13*new_r10))+(((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x182.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((gconst13*new_r10))+(((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13))))))))))+IKsqr((new_r11*(x182.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((((gconst13*new_r10))+(((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13))))))))), (new_r11*(x182.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x183=IKcos(j10);
IkReal x184=IKsin(j10);
IkReal x185=((1.0)*x184);
IkReal x186=((1.0)*x183);
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
IkReal x187=IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))));
IkReal x188=((1.0)*x187);
evalcond[0]=x184;
evalcond[1]=((-1.0)*x183);
evalcond[2]=((((-1.0)*gconst13*x186))+new_r11);
evalcond[3]=((((-1.0)*gconst13*x185))+new_r10);
evalcond[4]=(new_r01+((x183*x187)));
evalcond[5]=(((x184*x187))+new_r00);
evalcond[6]=((((-1.0)*x185))+((gconst13*new_r10))+(((-1.0)*new_r00*x188)));
evalcond[7]=((((-1.0)*x186))+((gconst13*new_r11))+(((-1.0)*new_r01*x188)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x189 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x189.valid){
continue;
}
CheckValue<IkReal> x190=IKPowWithIntegerCheck(IKsign(gconst13),-1);
if(!x190.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x189.value)+(((1.5707963267949)*(x190.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x191=IKcos(j10);
IkReal x192=IKsin(j10);
IkReal x193=((1.0)*x192);
IkReal x194=((1.0)*x191);
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
IkReal x195=IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))));
IkReal x196=((1.0)*x195);
evalcond[0]=x192;
evalcond[1]=((-1.0)*x191);
evalcond[2]=(new_r11+(((-1.0)*gconst13*x194)));
evalcond[3]=(new_r10+(((-1.0)*gconst13*x193)));
evalcond[4]=(((x191*x195))+new_r01);
evalcond[5]=(((x192*x195))+new_r00);
evalcond[6]=((((-1.0)*x193))+((gconst13*new_r10))+(((-1.0)*new_r00*x196)));
evalcond[7]=((((-1.0)*x194))+(((-1.0)*new_r01*x196))+((gconst13*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x197=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x197.valid){
continue;
}
if((x197.value) < -0.00001)
continue;
IkReal gconst13=IKsqrt(x197.value);
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.0)+(IKsign(sj8)))))+(IKabs((cj8+(((-1.0)*gconst13)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
sj8=((-1.0)*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))))));
cj8=gconst13;
if( (gconst13) < -1-IKFAST_SINCOS_THRESH || (gconst13) > 1+IKFAST_SINCOS_THRESH )
    continue;
j8=((-1.0)*(IKacos(gconst13)));
CheckValue<IkReal> x198=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x198.valid){
continue;
}
if((x198.value) < -0.00001)
continue;
IkReal gconst13=IKsqrt(x198.value);
j10eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
CheckValue<IkReal> x199=IKPowWithIntegerCheck(gconst13,-1);
if(!x199.valid){
continue;
}
if( IKabs((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13))))))))+((gconst13*new_r10)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x199.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13))))))))+((gconst13*new_r10))))+IKsqr((new_r11*(x199.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13))))))))+((gconst13*new_r10))), (new_r11*(x199.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x200=IKcos(j10);
IkReal x201=IKsin(j10);
IkReal x202=((1.0)*x201);
IkReal x203=((1.0)*x200);
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
IkReal x204=IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))));
evalcond[0]=x201;
evalcond[1]=((-1.0)*x200);
evalcond[2]=((((-1.0)*gconst13*x203))+new_r11);
evalcond[3]=((((-1.0)*gconst13*x202))+new_r10);
evalcond[4]=((((-1.0)*x203*x204))+new_r01);
evalcond[5]=((((-1.0)*x202*x204))+new_r00);
evalcond[6]=(((new_r00*x204))+(((-1.0)*x202))+((gconst13*new_r10)));
evalcond[7]=(((new_r01*x204))+(((-1.0)*x203))+((gconst13*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x205 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x205.valid){
continue;
}
CheckValue<IkReal> x206=IKPowWithIntegerCheck(IKsign(gconst13),-1);
if(!x206.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x205.value)+(((1.5707963267949)*(x206.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x207=IKcos(j10);
IkReal x208=IKsin(j10);
IkReal x209=((1.0)*x208);
IkReal x210=((1.0)*x207);
if((((1.0)+(((-1.0)*(gconst13*gconst13))))) < -0.00001)
continue;
IkReal x211=IKsqrt(((1.0)+(((-1.0)*(gconst13*gconst13)))));
evalcond[0]=x208;
evalcond[1]=((-1.0)*x207);
evalcond[2]=((((-1.0)*gconst13*x210))+new_r11);
evalcond[3]=((((-1.0)*gconst13*x209))+new_r10);
evalcond[4]=((((-1.0)*x210*x211))+new_r01);
evalcond[5]=(new_r00+(((-1.0)*x209*x211)));
evalcond[6]=((((-1.0)*x209))+((new_r00*x211))+((gconst13*new_r10)));
evalcond[7]=(((gconst13*new_r11))+((new_r01*x211))+(((-1.0)*x210)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x212=new_r22*new_r22;
CheckValue<IkReal> x213=IKPowWithIntegerCheck((((cj8*x212))+(((-1.0)*cj8))),-1);
if(!x213.valid){
continue;
}
CheckValue<IkReal> x214=IKPowWithIntegerCheck(((((-1.0)*sj8))+((sj8*x212))),-1);
if(!x214.valid){
continue;
}
if( IKabs(((x213.value)*(((((-1.0)*new_r01*new_r22))+(((-1.0)*new_r10)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((x214.value)*((((new_r10*new_r22))+new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x213.value)*(((((-1.0)*new_r01*new_r22))+(((-1.0)*new_r10))))))+IKsqr(((x214.value)*((((new_r10*new_r22))+new_r01))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((x213.value)*(((((-1.0)*new_r01*new_r22))+(((-1.0)*new_r10))))), ((x214.value)*((((new_r10*new_r22))+new_r01))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[10];
IkReal x215=IKsin(j10);
IkReal x216=IKcos(j10);
IkReal x217=(new_r22*sj8);
IkReal x218=(cj8*new_r00);
IkReal x219=((1.0)*sj8);
IkReal x220=(cj8*new_r01);
IkReal x221=((1.0)*x216);
IkReal x222=(new_r22*x215);
IkReal x223=((1.0)*x215);
evalcond[0]=(((new_r11*sj8))+x220+x222);
evalcond[1]=(x215+((new_r22*x220))+((new_r11*x217)));
evalcond[2]=(((cj8*new_r10))+(((-1.0)*new_r00*x219))+(((-1.0)*x223)));
evalcond[3]=(((cj8*new_r11))+(((-1.0)*new_r01*x219))+(((-1.0)*x221)));
evalcond[4]=(((cj8*x222))+((sj8*x216))+new_r01);
evalcond[5]=(((new_r10*sj8))+x218+(((-1.0)*new_r22*x221)));
evalcond[6]=((((-1.0)*cj8*new_r22*x221))+((sj8*x215))+new_r00);
evalcond[7]=(((x215*x217))+(((-1.0)*cj8*x221))+new_r11);
evalcond[8]=(((new_r22*x218))+((new_r10*x217))+(((-1.0)*x221)));
evalcond[9]=((((-1.0)*cj8*x223))+(((-1.0)*x217*x221))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x224=((1.0)*new_r01);
CheckValue<IkReal> x225=IKPowWithIntegerCheck(new_r22,-1);
if(!x225.valid){
continue;
}
if( IKabs(((x225.value)*(((((-1.0)*cj8*x224))+(((-1.0)*new_r11*sj8)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*sj8*x224))+((cj8*new_r11)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x225.value)*(((((-1.0)*cj8*x224))+(((-1.0)*new_r11*sj8))))))+IKsqr(((((-1.0)*sj8*x224))+((cj8*new_r11))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((x225.value)*(((((-1.0)*cj8*x224))+(((-1.0)*new_r11*sj8))))), ((((-1.0)*sj8*x224))+((cj8*new_r11))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[10];
IkReal x226=IKsin(j10);
IkReal x227=IKcos(j10);
IkReal x228=(new_r22*sj8);
IkReal x229=(cj8*new_r00);
IkReal x230=((1.0)*sj8);
IkReal x231=(cj8*new_r01);
IkReal x232=((1.0)*x227);
IkReal x233=(new_r22*x226);
IkReal x234=((1.0)*x226);
evalcond[0]=(((new_r11*sj8))+x233+x231);
evalcond[1]=(((new_r22*x231))+x226+((new_r11*x228)));
evalcond[2]=(((cj8*new_r10))+(((-1.0)*x234))+(((-1.0)*new_r00*x230)));
evalcond[3]=(((cj8*new_r11))+(((-1.0)*x232))+(((-1.0)*new_r01*x230)));
evalcond[4]=(((cj8*x233))+((sj8*x227))+new_r01);
evalcond[5]=((((-1.0)*new_r22*x232))+((new_r10*sj8))+x229);
evalcond[6]=(((sj8*x226))+new_r00+(((-1.0)*cj8*new_r22*x232)));
evalcond[7]=((((-1.0)*cj8*x232))+((x226*x228))+new_r11);
evalcond[8]=((((-1.0)*x232))+((new_r22*x229))+((new_r10*x228)));
evalcond[9]=((((-1.0)*cj8*x234))+new_r10+(((-1.0)*x228*x232)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x235=cj8*cj8;
IkReal x236=(cj8*new_r22);
CheckValue<IkReal> x237 = IKatan2WithCheck(IkReal((((new_r01*x236))+((new_r00*sj8)))),IkReal(((((-1.0)*new_r00*x236))+((new_r01*sj8)))),IKFAST_ATAN2_MAGTHRESH);
if(!x237.valid){
continue;
}
CheckValue<IkReal> x238=IKPowWithIntegerCheck(IKsign(((-1.0)+(((-1.0)*x235*(new_r22*new_r22)))+x235)),-1);
if(!x238.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x237.value)+(((1.5707963267949)*(x238.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[10];
IkReal x239=IKsin(j10);
IkReal x240=IKcos(j10);
IkReal x241=(new_r22*sj8);
IkReal x242=(cj8*new_r00);
IkReal x243=((1.0)*sj8);
IkReal x244=(cj8*new_r01);
IkReal x245=((1.0)*x240);
IkReal x246=(new_r22*x239);
IkReal x247=((1.0)*x239);
evalcond[0]=(((new_r11*sj8))+x246+x244);
evalcond[1]=(((new_r22*x244))+x239+((new_r11*x241)));
evalcond[2]=((((-1.0)*new_r00*x243))+((cj8*new_r10))+(((-1.0)*x247)));
evalcond[3]=((((-1.0)*new_r01*x243))+((cj8*new_r11))+(((-1.0)*x245)));
evalcond[4]=(((sj8*x240))+((cj8*x246))+new_r01);
evalcond[5]=(((new_r10*sj8))+(((-1.0)*new_r22*x245))+x242);
evalcond[6]=((((-1.0)*cj8*new_r22*x245))+((sj8*x239))+new_r00);
evalcond[7]=((((-1.0)*cj8*x245))+((x239*x241))+new_r11);
evalcond[8]=(((new_r22*x242))+(((-1.0)*x245))+((new_r10*x241)));
evalcond[9]=((((-1.0)*cj8*x247))+(((-1.0)*x241*x245))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
    }

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j8, j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}

} else
{
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
CheckValue<IkReal> x249=IKPowWithIntegerCheck(sj9,-1);
if(!x249.valid){
continue;
}
IkReal x248=x249.value;
CheckValue<IkReal> x250=IKPowWithIntegerCheck(new_r12,-1);
if(!x250.valid){
continue;
}
if( IKabs((x248*(x250.value)*(((1.0)+(((-1.0)*(cj9*cj9)))+(((-1.0)*(new_r02*new_r02))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r02*x248)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x248*(x250.value)*(((1.0)+(((-1.0)*(cj9*cj9)))+(((-1.0)*(new_r02*new_r02)))))))+IKsqr((new_r02*x248))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8array[0]=IKatan2((x248*(x250.value)*(((1.0)+(((-1.0)*(cj9*cj9)))+(((-1.0)*(new_r02*new_r02)))))), (new_r02*x248));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[8];
IkReal x251=IKcos(j8);
IkReal x252=IKsin(j8);
IkReal x253=((1.0)*cj9);
IkReal x254=((1.0)*sj9);
IkReal x255=((1.0)*x252);
IkReal x256=(new_r12*x252);
IkReal x257=(new_r02*x251);
evalcond[0]=((((-1.0)*x251*x254))+new_r02);
evalcond[1]=((((-1.0)*x252*x254))+new_r12);
evalcond[2]=(((new_r12*x251))+(((-1.0)*new_r02*x255)));
evalcond[3]=(x256+x257+(((-1.0)*x254)));
evalcond[4]=(((cj9*x257))+((cj9*x256))+(((-1.0)*new_r22*x254)));
evalcond[5]=((((-1.0)*new_r10*x252*x254))+(((-1.0)*new_r20*x253))+(((-1.0)*new_r00*x251*x254)));
evalcond[6]=((((-1.0)*new_r21*x253))+(((-1.0)*new_r11*x252*x254))+(((-1.0)*new_r01*x251*x254)));
evalcond[7]=((1.0)+(((-1.0)*x254*x256))+(((-1.0)*x254*x257))+(((-1.0)*new_r22*x253)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j10eval[3];
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[2];
j10eval[0]=sj8;
j10eval[1]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  )
{
{
IkReal j10eval[3];
j10eval[0]=cj8;
j10eval[1]=cj9;
j10eval[2]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j8)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[3];
sj8=1.0;
cj8=0;
j8=1.5707963267949;
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[3];
sj8=1.0;
cj8=0;
j8=1.5707963267949;
j10eval[0]=cj9;
j10eval[1]=IKsign(cj9);
j10eval[2]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[1];
sj8=1.0;
cj8=0;
j8=1.5707963267949;
j10eval[0]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r11))+IKsqr(new_r10)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r11), new_r10);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x258=IKsin(j10);
IkReal x259=((1.0)*(IKcos(j10)));
evalcond[0]=(x258+new_r11);
evalcond[1]=(new_r10+(((-1.0)*x259)));
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x258)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x259)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r11)+IKsqr(((-1.0)*new_r10))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r11, ((-1.0)*new_r10));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x260=IKcos(j10);
IkReal x261=((1.0)*(IKsin(j10)));
evalcond[0]=(x260+new_r10);
evalcond[1]=(new_r11+(((-1.0)*x261)));
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x261)));
evalcond[3]=((((-1.0)*x260))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x262=IKcos(j10);
IkReal x263=((1.0)*(IKsin(j10)));
evalcond[0]=(x262+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x263)));
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x263)));
evalcond[3]=((((-1.0)*x262))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x264=IKsin(j10);
IkReal x265=((1.0)*(IKcos(j10)));
evalcond[0]=(x264+new_r21);
evalcond[1]=(new_r20+(((-1.0)*x265)));
evalcond[2]=((((-1.0)*x264))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x265)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r01))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r01));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[6];
IkReal x266=IKsin(j10);
IkReal x267=IKcos(j10);
IkReal x268=((-1.0)*x267);
evalcond[0]=x266;
evalcond[1]=(new_r22*x266);
evalcond[2]=x268;
evalcond[3]=(new_r22*x268);
evalcond[4]=((((-1.0)*x266))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x267))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x269=IKPowWithIntegerCheck(sj9,-1);
if(!x269.valid){
continue;
}
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x269.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r20*(x269.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r20*(x269.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x270=IKsin(j10);
IkReal x271=IKcos(j10);
IkReal x272=((1.0)*sj9);
IkReal x273=((1.0)*x271);
evalcond[0]=(((sj9*x271))+new_r20);
evalcond[1]=(((cj9*x270))+new_r11);
evalcond[2]=((((-1.0)*x270*x272))+new_r21);
evalcond[3]=((((-1.0)*cj9*x273))+new_r10);
evalcond[4]=((((-1.0)*x270))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x273))+(((-1.0)*new_r01)));
evalcond[6]=(((cj9*new_r11))+(((-1.0)*new_r21*x272))+x270);
evalcond[7]=(((cj9*new_r10))+(((-1.0)*new_r20*x272))+(((-1.0)*x273)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x274=IKPowWithIntegerCheck(IKsign(cj9),-1);
if(!x274.valid){
continue;
}
CheckValue<IkReal> x275 = IKatan2WithCheck(IkReal(((-1.0)*new_r11)),IkReal(new_r10),IKFAST_ATAN2_MAGTHRESH);
if(!x275.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(((1.5707963267949)*(x274.value)))+(x275.value));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x276=IKsin(j10);
IkReal x277=IKcos(j10);
IkReal x278=((1.0)*sj9);
IkReal x279=((1.0)*x277);
evalcond[0]=(((sj9*x277))+new_r20);
evalcond[1]=(((cj9*x276))+new_r11);
evalcond[2]=(new_r21+(((-1.0)*x276*x278)));
evalcond[3]=((((-1.0)*cj9*x279))+new_r10);
evalcond[4]=((((-1.0)*x276))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x279))+(((-1.0)*new_r01)));
evalcond[6]=(((cj9*new_r11))+(((-1.0)*new_r21*x278))+x276);
evalcond[7]=(((cj9*new_r10))+(((-1.0)*new_r20*x278))+(((-1.0)*x279)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x280 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x280.valid){
continue;
}
CheckValue<IkReal> x281=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x281.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x280.value)+(((1.5707963267949)*(x281.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x282=IKsin(j10);
IkReal x283=IKcos(j10);
IkReal x284=((1.0)*sj9);
IkReal x285=((1.0)*x283);
evalcond[0]=(((sj9*x283))+new_r20);
evalcond[1]=(((cj9*x282))+new_r11);
evalcond[2]=((((-1.0)*x282*x284))+new_r21);
evalcond[3]=((((-1.0)*cj9*x285))+new_r10);
evalcond[4]=((((-1.0)*x282))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x285))+(((-1.0)*new_r01)));
evalcond[6]=(((cj9*new_r11))+x282+(((-1.0)*new_r21*x284)));
evalcond[7]=(((cj9*new_r10))+(((-1.0)*x285))+(((-1.0)*new_r20*x284)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j8)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(new_r01)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r00, new_r01);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x286=IKcos(j10);
IkReal x287=IKsin(j10);
IkReal x288=((1.0)*sj9);
IkReal x289=((1.0)*new_r10);
IkReal x290=((1.0)*new_r11);
IkReal x291=((1.0)*x286);
evalcond[0]=(((sj9*x286))+new_r20);
evalcond[1]=((((-1.0)*x287))+new_r00);
evalcond[2]=((((-1.0)*x291))+new_r01);
evalcond[3]=((((-1.0)*x287*x288))+new_r21);
evalcond[4]=((((-1.0)*x290))+((cj9*x287)));
evalcond[5]=((((-1.0)*cj9*x291))+(((-1.0)*x289)));
evalcond[6]=((((-1.0)*cj9*x290))+x287+(((-1.0)*new_r21*x288)));
evalcond[7]=((((-1.0)*x291))+(((-1.0)*cj9*x289))+(((-1.0)*new_r20*x288)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x292=IKcos(j10);
IkReal x293=IKsin(j10);
IkReal x294=((1.0)*sj8);
IkReal x295=((1.0)*x293);
IkReal x296=((1.0)*x292);
evalcond[0]=(x292+new_r20);
evalcond[1]=((((-1.0)*x295))+new_r21);
evalcond[2]=(((sj8*x292))+new_r01);
evalcond[3]=(((sj8*x293))+new_r00);
evalcond[4]=((((-1.0)*cj8*x296))+new_r11);
evalcond[5]=((((-1.0)*cj8*x295))+new_r10);
evalcond[6]=((((-1.0)*x295))+((cj8*new_r10))+(((-1.0)*new_r00*x294)));
evalcond[7]=((((-1.0)*new_r01*x294))+(((-1.0)*x296))+((cj8*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x297=IKcos(j10);
IkReal x298=IKsin(j10);
IkReal x299=((1.0)*sj8);
IkReal x300=((1.0)*x297);
IkReal x301=((1.0)*x298);
evalcond[0]=(x298+new_r21);
evalcond[1]=(new_r20+(((-1.0)*x300)));
evalcond[2]=(((sj8*x297))+new_r01);
evalcond[3]=(((sj8*x298))+new_r00);
evalcond[4]=((((-1.0)*cj8*x300))+new_r11);
evalcond[5]=((((-1.0)*cj8*x301))+new_r10);
evalcond[6]=(((cj8*new_r10))+(((-1.0)*new_r00*x299))+(((-1.0)*x301)));
evalcond[7]=((((-1.0)*new_r01*x299))+((cj8*new_r11))+(((-1.0)*x300)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x302=((1.0)*new_r01);
if( IKabs(((((-1.0)*cj8*x302))+(((-1.0)*new_r00*sj8)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj8*new_r00))+(((-1.0)*sj8*x302)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*cj8*x302))+(((-1.0)*new_r00*sj8))))+IKsqr((((cj8*new_r00))+(((-1.0)*sj8*x302))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((((-1.0)*cj8*x302))+(((-1.0)*new_r00*sj8))), (((cj8*new_r00))+(((-1.0)*sj8*x302))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x303=IKsin(j10);
IkReal x304=IKcos(j10);
IkReal x305=((1.0)*sj8);
IkReal x306=((1.0)*x304);
IkReal x307=(sj8*x303);
IkReal x308=(cj8*x303);
IkReal x309=(cj8*x306);
evalcond[0]=(((new_r11*sj8))+x303+((cj8*new_r01)));
evalcond[1]=(((sj8*x304))+x308+new_r01);
evalcond[2]=(((new_r10*sj8))+((cj8*new_r00))+(((-1.0)*x306)));
evalcond[3]=((((-1.0)*x303))+((cj8*new_r10))+(((-1.0)*new_r00*x305)));
evalcond[4]=(((cj8*new_r11))+(((-1.0)*new_r01*x305))+(((-1.0)*x306)));
evalcond[5]=(x307+new_r00+(((-1.0)*x309)));
evalcond[6]=(x307+new_r11+(((-1.0)*x309)));
evalcond[7]=((((-1.0)*x308))+(((-1.0)*x304*x305))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x310=((1.0)*new_r00);
if( IKabs(((((-1.0)*sj8*x310))+((cj8*new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*new_r01*sj8))+(((-1.0)*cj8*x310)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*sj8*x310))+((cj8*new_r01))))+IKsqr(((((-1.0)*new_r01*sj8))+(((-1.0)*cj8*x310))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((((-1.0)*sj8*x310))+((cj8*new_r01))), ((((-1.0)*new_r01*sj8))+(((-1.0)*cj8*x310))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x311=IKsin(j10);
IkReal x312=IKcos(j10);
IkReal x313=((1.0)*sj8);
IkReal x314=((1.0)*x311);
IkReal x315=(sj8*x312);
IkReal x316=((1.0)*x312);
IkReal x317=(cj8*x314);
evalcond[0]=(((new_r10*sj8))+x312+((cj8*new_r00)));
evalcond[1]=(((new_r11*sj8))+((cj8*new_r01))+(((-1.0)*x314)));
evalcond[2]=(((cj8*x312))+((sj8*x311))+new_r00);
evalcond[3]=((((-1.0)*new_r00*x313))+((cj8*new_r10))+(((-1.0)*x314)));
evalcond[4]=(((cj8*new_r11))+(((-1.0)*x316))+(((-1.0)*new_r01*x313)));
evalcond[5]=(x315+(((-1.0)*x317))+new_r01);
evalcond[6]=(x315+(((-1.0)*x317))+new_r10);
evalcond[7]=(new_r11+(((-1.0)*cj8*x316))+(((-1.0)*x311*x313)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j8))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r10)+IKsqr(new_r11)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r10, new_r11);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x318=IKcos(j10);
IkReal x319=IKsin(j10);
IkReal x320=((1.0)*sj9);
IkReal x321=((1.0)*x318);
evalcond[0]=(new_r20+((sj9*x318)));
evalcond[1]=((((-1.0)*x319))+new_r10);
evalcond[2]=((((-1.0)*x321))+new_r11);
evalcond[3]=(((cj9*x319))+new_r01);
evalcond[4]=((((-1.0)*x319*x320))+new_r21);
evalcond[5]=((((-1.0)*cj9*x321))+new_r00);
evalcond[6]=(((cj9*new_r01))+x319+(((-1.0)*new_r21*x320)));
evalcond[7]=(((cj9*new_r00))+(((-1.0)*x321))+(((-1.0)*new_r20*x320)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j8)))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[3];
sj8=0;
cj8=-1.0;
j8=3.14159265358979;
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[1];
sj8=0;
cj8=-1.0;
j8=3.14159265358979;
j10eval[0]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10eval[2];
sj8=0;
cj8=-1.0;
j8=3.14159265358979;
j10eval[0]=cj9;
j10eval[1]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x322=IKcos(j10);
IkReal x323=((1.0)*(IKsin(j10)));
evalcond[0]=(x322+new_r20);
evalcond[1]=((((-1.0)*x323))+new_r21);
evalcond[2]=((((-1.0)*x323))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x322))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x324=IKsin(j10);
IkReal x325=((1.0)*(IKcos(j10)));
evalcond[0]=(x324+new_r21);
evalcond[1]=((((-1.0)*x325))+new_r20);
evalcond[2]=((((-1.0)*x324))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x325))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r01)+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r01, ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x326=IKsin(j10);
IkReal x327=((1.0)*(IKcos(j10)));
evalcond[0]=(x326+(((-1.0)*new_r01)));
evalcond[1]=((((-1.0)*x326))+(((-1.0)*new_r10)));
evalcond[2]=((((-1.0)*x327))+(((-1.0)*new_r11)));
evalcond[3]=((((-1.0)*x327))+(((-1.0)*new_r00)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r10), new_r00);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x328=IKcos(j10);
IkReal x329=((1.0)*(IKsin(j10)));
evalcond[0]=(x328+(((-1.0)*new_r00)));
evalcond[1]=((((-1.0)*x329))+(((-1.0)*new_r10)));
evalcond[2]=((((-1.0)*x328))+(((-1.0)*new_r11)));
evalcond[3]=((((-1.0)*x329))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r10), ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[6];
IkReal x330=IKsin(j10);
IkReal x331=IKcos(j10);
IkReal x332=((-1.0)*x331);
evalcond[0]=x330;
evalcond[1]=(new_r22*x330);
evalcond[2]=x332;
evalcond[3]=(new_r22*x332);
evalcond[4]=((((-1.0)*x330))+(((-1.0)*new_r10)));
evalcond[5]=((((-1.0)*x331))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x333=IKPowWithIntegerCheck(cj9,-1);
if(!x333.valid){
continue;
}
CheckValue<IkReal> x334=IKPowWithIntegerCheck(sj9,-1);
if(!x334.valid){
continue;
}
if( IKabs((new_r01*(x333.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x334.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r01*(x333.value)))+IKsqr(((-1.0)*new_r20*(x334.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((new_r01*(x333.value)), ((-1.0)*new_r20*(x334.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x335=IKsin(j10);
IkReal x336=IKcos(j10);
IkReal x337=((1.0)*sj9);
IkReal x338=((1.0)*new_r01);
IkReal x339=((1.0)*new_r00);
IkReal x340=((1.0)*x336);
evalcond[0]=(((sj9*x336))+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x335*x337)));
evalcond[2]=((((-1.0)*x335))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x340)));
evalcond[4]=(((cj9*x335))+(((-1.0)*x338)));
evalcond[5]=((((-1.0)*cj9*x340))+(((-1.0)*x339)));
evalcond[6]=((((-1.0)*new_r21*x337))+x335+(((-1.0)*cj9*x338)));
evalcond[7]=((((-1.0)*new_r20*x337))+(((-1.0)*cj9*x339))+(((-1.0)*x340)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x341=IKPowWithIntegerCheck(sj9,-1);
if(!x341.valid){
continue;
}
if( IKabs((new_r21*(x341.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r21*(x341.value)))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((new_r21*(x341.value)), ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x342=IKsin(j10);
IkReal x343=IKcos(j10);
IkReal x344=((1.0)*sj9);
IkReal x345=((1.0)*new_r01);
IkReal x346=((1.0)*new_r00);
IkReal x347=((1.0)*x343);
evalcond[0]=(((sj9*x343))+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x342*x344)));
evalcond[2]=((((-1.0)*new_r10))+(((-1.0)*x342)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x347)));
evalcond[4]=(((cj9*x342))+(((-1.0)*x345)));
evalcond[5]=((((-1.0)*cj9*x347))+(((-1.0)*x346)));
evalcond[6]=((((-1.0)*cj9*x345))+x342+(((-1.0)*new_r21*x344)));
evalcond[7]=((((-1.0)*cj9*x346))+(((-1.0)*new_r20*x344))+(((-1.0)*x347)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x348 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x348.valid){
continue;
}
CheckValue<IkReal> x349=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x349.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x348.value)+(((1.5707963267949)*(x349.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x350=IKsin(j10);
IkReal x351=IKcos(j10);
IkReal x352=((1.0)*sj9);
IkReal x353=((1.0)*new_r01);
IkReal x354=((1.0)*new_r00);
IkReal x355=((1.0)*x351);
evalcond[0]=(((sj9*x351))+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x350*x352)));
evalcond[2]=((((-1.0)*x350))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x355))+(((-1.0)*new_r11)));
evalcond[4]=(((cj9*x350))+(((-1.0)*x353)));
evalcond[5]=((((-1.0)*cj9*x355))+(((-1.0)*x354)));
evalcond[6]=((((-1.0)*cj9*x353))+x350+(((-1.0)*new_r21*x352)));
evalcond[7]=((((-1.0)*cj9*x354))+(((-1.0)*x355))+(((-1.0)*new_r20*x352)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[1];
new_r21=0;
new_r20=0;
new_r02=0;
new_r12=0;
j10eval[0]=1.0;
if( IKabs(j10eval[0]) < 0.0000000100000000  )
{
continue; // no branches [j10]

} else
{
IkReal op[2+1], zeror[2];
int numroots;
op[0]=1.0;
op[1]=0;
op[2]=-1.0;
polyroots2(op,zeror,numroots);
IkReal j10array[2], cj10array[2], sj10array[2], tempj10array[1];
int numsolutions = 0;
for(int ij10 = 0; ij10 < numroots; ++ij10)
{
IkReal htj10 = zeror[ij10];
tempj10array[0]=((2.0)*(atan(htj10)));
for(int kj10 = 0; kj10 < 1; ++kj10)
{
j10array[numsolutions] = tempj10array[kj10];
if( j10array[numsolutions] > IKPI )
{
    j10array[numsolutions]-=IK2PI;
}
else if( j10array[numsolutions] < -IKPI )
{
    j10array[numsolutions]+=IK2PI;
}
sj10array[numsolutions] = IKsin(j10array[numsolutions]);
cj10array[numsolutions] = IKcos(j10array[numsolutions]);
numsolutions++;
}
}
bool j10valid[2]={true,true};
_nj10 = 2;
for(int ij10 = 0; ij10 < numsolutions; ++ij10)
    {
if( !j10valid[ij10] )
{
    continue;
}
    j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
htj10 = IKtan(j10/2);

_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < numsolutions; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
    }

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x357=IKPowWithIntegerCheck(sj9,-1);
if(!x357.valid){
continue;
}
IkReal x356=x357.value;
CheckValue<IkReal> x358=IKPowWithIntegerCheck(cj8,-1);
if(!x358.valid){
continue;
}
CheckValue<IkReal> x359=IKPowWithIntegerCheck(cj9,-1);
if(!x359.valid){
continue;
}
if( IKabs((x356*(x358.value)*(x359.value)*((((new_r20*sj8))+(((-1.0)*new_r01*sj9)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x356)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x356*(x358.value)*(x359.value)*((((new_r20*sj8))+(((-1.0)*new_r01*sj9))))))+IKsqr(((-1.0)*new_r20*x356))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((x356*(x358.value)*(x359.value)*((((new_r20*sj8))+(((-1.0)*new_r01*sj9))))), ((-1.0)*new_r20*x356));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[12];
IkReal x360=IKsin(j10);
IkReal x361=IKcos(j10);
IkReal x362=(cj9*sj8);
IkReal x363=((1.0)*sj9);
IkReal x364=((1.0)*sj8);
IkReal x365=((1.0)*cj8);
IkReal x366=(cj8*new_r00);
IkReal x367=(cj8*new_r01);
IkReal x368=((1.0)*x361);
IkReal x369=(cj9*x360);
IkReal x370=(cj9*x368);
evalcond[0]=(((sj9*x361))+new_r20);
evalcond[1]=((((-1.0)*x360*x363))+new_r21);
evalcond[2]=(((new_r11*sj8))+x369+x367);
evalcond[3]=((((-1.0)*new_r00*x364))+(((-1.0)*x360))+((cj8*new_r10)));
evalcond[4]=((((-1.0)*x368))+((cj8*new_r11))+(((-1.0)*new_r01*x364)));
evalcond[5]=(((sj8*x361))+((cj8*x369))+new_r01);
evalcond[6]=(((new_r10*sj8))+(((-1.0)*x370))+x366);
evalcond[7]=(((sj8*x360))+(((-1.0)*cj9*x361*x365))+new_r00);
evalcond[8]=((((-1.0)*x361*x365))+new_r11+((x360*x362)));
evalcond[9]=((((-1.0)*x362*x368))+(((-1.0)*x360*x365))+new_r10);
evalcond[10]=(x360+((cj9*x367))+((new_r11*x362))+(((-1.0)*new_r21*x363)));
evalcond[11]=((((-1.0)*new_r20*x363))+(((-1.0)*x368))+((cj9*x366))+((new_r10*x362)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x372=IKPowWithIntegerCheck(sj9,-1);
if(!x372.valid){
continue;
}
IkReal x371=x372.value;
CheckValue<IkReal> x373=IKPowWithIntegerCheck(sj8,-1);
if(!x373.valid){
continue;
}
if( IKabs((x371*(x373.value)*(((((-1.0)*cj8*cj9*new_r20))+(((-1.0)*new_r00*sj9)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x371)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x371*(x373.value)*(((((-1.0)*cj8*cj9*new_r20))+(((-1.0)*new_r00*sj9))))))+IKsqr(((-1.0)*new_r20*x371))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((x371*(x373.value)*(((((-1.0)*cj8*cj9*new_r20))+(((-1.0)*new_r00*sj9))))), ((-1.0)*new_r20*x371));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[12];
IkReal x374=IKsin(j10);
IkReal x375=IKcos(j10);
IkReal x376=(cj9*sj8);
IkReal x377=((1.0)*sj9);
IkReal x378=((1.0)*sj8);
IkReal x379=((1.0)*cj8);
IkReal x380=(cj8*new_r00);
IkReal x381=(cj8*new_r01);
IkReal x382=((1.0)*x375);
IkReal x383=(cj9*x374);
IkReal x384=(cj9*x382);
evalcond[0]=(((sj9*x375))+new_r20);
evalcond[1]=((((-1.0)*x374*x377))+new_r21);
evalcond[2]=(((new_r11*sj8))+x383+x381);
evalcond[3]=((((-1.0)*new_r00*x378))+(((-1.0)*x374))+((cj8*new_r10)));
evalcond[4]=(((cj8*new_r11))+(((-1.0)*x382))+(((-1.0)*new_r01*x378)));
evalcond[5]=(((cj8*x383))+((sj8*x375))+new_r01);
evalcond[6]=(((new_r10*sj8))+x380+(((-1.0)*x384)));
evalcond[7]=(((sj8*x374))+(((-1.0)*cj9*x375*x379))+new_r00);
evalcond[8]=(new_r11+(((-1.0)*x375*x379))+((x374*x376)));
evalcond[9]=((((-1.0)*x376*x382))+(((-1.0)*x374*x379))+new_r10);
evalcond[10]=((((-1.0)*new_r21*x377))+((cj9*x381))+x374+((new_r11*x376)));
evalcond[11]=((((-1.0)*new_r20*x377))+((cj9*x380))+(((-1.0)*x382))+((new_r10*x376)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x385 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x385.valid){
continue;
}
CheckValue<IkReal> x386=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x386.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x385.value)+(((1.5707963267949)*(x386.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[12];
IkReal x387=IKsin(j10);
IkReal x388=IKcos(j10);
IkReal x389=(cj9*sj8);
IkReal x390=((1.0)*sj9);
IkReal x391=((1.0)*sj8);
IkReal x392=((1.0)*cj8);
IkReal x393=(cj8*new_r00);
IkReal x394=(cj8*new_r01);
IkReal x395=((1.0)*x388);
IkReal x396=(cj9*x387);
IkReal x397=(cj9*x395);
evalcond[0]=(((sj9*x388))+new_r20);
evalcond[1]=((((-1.0)*x387*x390))+new_r21);
evalcond[2]=(((new_r11*sj8))+x394+x396);
evalcond[3]=(((cj8*new_r10))+(((-1.0)*x387))+(((-1.0)*new_r00*x391)));
evalcond[4]=((((-1.0)*new_r01*x391))+(((-1.0)*x395))+((cj8*new_r11)));
evalcond[5]=(((cj8*x396))+((sj8*x388))+new_r01);
evalcond[6]=((((-1.0)*x397))+((new_r10*sj8))+x393);
evalcond[7]=((((-1.0)*cj9*x388*x392))+((sj8*x387))+new_r00);
evalcond[8]=(((x387*x389))+(((-1.0)*x388*x392))+new_r11);
evalcond[9]=((((-1.0)*x387*x392))+(((-1.0)*x389*x395))+new_r10);
evalcond[10]=((((-1.0)*new_r21*x390))+x387+((new_r11*x389))+((cj9*x394)));
evalcond[11]=((((-1.0)*x395))+((new_r10*x389))+((cj9*x393))+(((-1.0)*new_r20*x390)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x398 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x398.valid){
continue;
}
CheckValue<IkReal> x399=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x399.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x398.value)+(((1.5707963267949)*(x399.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[2];
evalcond[0]=(((sj9*(IKcos(j10))))+new_r20);
evalcond[1]=((((-1.0)*sj9*(IKsin(j10))))+new_r21);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j8eval[3];
j8eval[0]=sj9;
j8eval[1]=((IKabs(new_r12))+(IKabs(new_r02)));
j8eval[2]=IKsign(sj9);
if( IKabs(j8eval[0]) < 0.0000010000000000  || IKabs(j8eval[1]) < 0.0000010000000000  || IKabs(j8eval[2]) < 0.0000010000000000  )
{
{
IkReal j8eval[2];
j8eval[0]=cj10;
j8eval[1]=sj9;
if( IKabs(j8eval[0]) < 0.0000010000000000  || IKabs(j8eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j10)))), 6.28318530717959)));
evalcond[1]=new_r20;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(new_r10)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8array[0]=IKatan2(((-1.0)*new_r00), new_r10);
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[18];
IkReal x400=IKsin(j8);
IkReal x401=IKcos(j8);
IkReal x402=((1.0)*sj9);
IkReal x403=((1.0)*cj9);
IkReal x404=((1.0)*x400);
IkReal x405=(cj9*x400);
IkReal x406=(new_r01*x401);
IkReal x407=(cj9*x401);
IkReal x408=(new_r00*x401);
IkReal x409=(new_r02*x401);
evalcond[0]=(x400+new_r00);
evalcond[1]=(x407+new_r01);
evalcond[2]=(x405+new_r11);
evalcond[3]=(new_r10+(((-1.0)*x401)));
evalcond[4]=((((-1.0)*x401*x402))+new_r02);
evalcond[5]=((((-1.0)*x400*x402))+new_r12);
evalcond[6]=(x408+((new_r10*x400)));
evalcond[7]=(((new_r12*x401))+(((-1.0)*new_r02*x404)));
evalcond[8]=(((new_r11*x401))+(((-1.0)*new_r01*x404)));
evalcond[9]=(cj9+x406+((new_r11*x400)));
evalcond[10]=((-1.0)+((new_r10*x401))+(((-1.0)*new_r00*x404)));
evalcond[11]=(((new_r10*x405))+((new_r00*x407)));
evalcond[12]=(x409+((new_r12*x400))+(((-1.0)*x402)));
evalcond[13]=((((-1.0)*x402*x408))+(((-1.0)*new_r10*x400*x402)));
evalcond[14]=(((new_r12*x405))+(((-1.0)*new_r22*x402))+((new_r02*x407)));
evalcond[15]=((1.0)+(((-1.0)*new_r21*x402))+((new_r11*x405))+((cj9*x406)));
evalcond[16]=((((-1.0)*x402*x406))+(((-1.0)*new_r21*x403))+(((-1.0)*new_r11*x400*x402)));
evalcond[17]=((1.0)+(((-1.0)*new_r12*x400*x402))+(((-1.0)*x402*x409))+(((-1.0)*new_r22*x403)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j10)))), 6.28318530717959)));
evalcond[1]=new_r20;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(((-1.0)*new_r10))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8array[0]=IKatan2(new_r00, ((-1.0)*new_r10));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[18];
IkReal x410=IKcos(j8);
IkReal x411=IKsin(j8);
IkReal x412=((1.0)*cj9);
IkReal x413=((1.0)*sj9);
IkReal x414=((1.0)*x411);
IkReal x415=(new_r11*x411);
IkReal x416=(cj9*x411);
IkReal x417=(cj9*x410);
IkReal x418=(x410*x413);
evalcond[0]=(x410+new_r10);
evalcond[1]=(new_r00+(((-1.0)*x414)));
evalcond[2]=(new_r02+(((-1.0)*x418)));
evalcond[3]=((((-1.0)*x411*x413))+new_r12);
evalcond[4]=((((-1.0)*x410*x412))+new_r01);
evalcond[5]=((((-1.0)*x411*x412))+new_r11);
evalcond[6]=(((new_r10*x411))+((new_r00*x410)));
evalcond[7]=(((new_r12*x410))+(((-1.0)*new_r02*x414)));
evalcond[8]=(((new_r11*x410))+(((-1.0)*new_r01*x414)));
evalcond[9]=((1.0)+((new_r10*x410))+(((-1.0)*new_r00*x414)));
evalcond[10]=(((new_r10*x416))+((new_r00*x417)));
evalcond[11]=(((new_r12*x411))+(((-1.0)*x413))+((new_r02*x410)));
evalcond[12]=(x415+(((-1.0)*x412))+((new_r01*x410)));
evalcond[13]=((((-1.0)*new_r10*x411*x413))+(((-1.0)*new_r00*x418)));
evalcond[14]=(((new_r12*x416))+(((-1.0)*new_r22*x413))+((new_r02*x417)));
evalcond[15]=((-1.0)+(sj9*sj9)+((cj9*x415))+((new_r01*x417)));
evalcond[16]=((((-1.0)*new_r21*x412))+(((-1.0)*x413*x415))+(((-1.0)*new_r01*x418)));
evalcond[17]=((1.0)+(((-1.0)*new_r12*x411*x413))+(((-1.0)*new_r02*x418))+(((-1.0)*new_r22*x412)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
IkReal x419=((1.0)*sj10);
if( IKabs(((((-1.0)*cj10*new_r01))+(((-1.0)*new_r00*x419)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj10*new_r00))+(((-1.0)*new_r01*x419)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*cj10*new_r01))+(((-1.0)*new_r00*x419))))+IKsqr((((cj10*new_r00))+(((-1.0)*new_r01*x419))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8array[0]=IKatan2(((((-1.0)*cj10*new_r01))+(((-1.0)*new_r00*x419))), (((cj10*new_r00))+(((-1.0)*new_r01*x419))));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[8];
IkReal x420=IKsin(j8);
IkReal x421=IKcos(j8);
IkReal x422=((1.0)*cj10);
IkReal x423=((1.0)*sj10);
IkReal x424=(sj10*x420);
IkReal x425=((1.0)*x420);
IkReal x426=(x421*x422);
evalcond[0]=(((new_r11*x420))+sj10+((new_r01*x421)));
evalcond[1]=(((sj10*x421))+new_r01+((cj10*x420)));
evalcond[2]=((((-1.0)*x426))+x424+new_r00);
evalcond[3]=((((-1.0)*x426))+x424+new_r11);
evalcond[4]=((((-1.0)*x422))+((new_r10*x420))+((new_r00*x421)));
evalcond[5]=((((-1.0)*x421*x423))+(((-1.0)*x420*x422))+new_r10);
evalcond[6]=((((-1.0)*new_r00*x425))+(((-1.0)*x423))+((new_r10*x421)));
evalcond[7]=((((-1.0)*x422))+((new_r11*x421))+(((-1.0)*new_r01*x425)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
IkReal x427=((1.0)*new_r00);
if( IKabs(((((-1.0)*sj10*x427))+(((-1.0)*cj10*new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*cj10*x427))+((new_r01*sj10)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*sj10*x427))+(((-1.0)*cj10*new_r01))))+IKsqr(((((-1.0)*cj10*x427))+((new_r01*sj10))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8array[0]=IKatan2(((((-1.0)*sj10*x427))+(((-1.0)*cj10*new_r01))), ((((-1.0)*cj10*x427))+((new_r01*sj10))));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[8];
IkReal x428=IKcos(j8);
IkReal x429=IKsin(j8);
IkReal x430=((1.0)*sj10);
IkReal x431=((1.0)*cj10);
IkReal x432=(cj10*x429);
IkReal x433=((1.0)*x429);
IkReal x434=(x428*x430);
evalcond[0]=(((new_r10*x429))+cj10+((new_r00*x428)));
evalcond[1]=(((sj10*x429))+new_r00+((cj10*x428)));
evalcond[2]=((((-1.0)*x434))+x432+new_r01);
evalcond[3]=((((-1.0)*x434))+x432+new_r10);
evalcond[4]=((((-1.0)*x430))+((new_r11*x429))+((new_r01*x428)));
evalcond[5]=((((-1.0)*x429*x430))+new_r11+(((-1.0)*x428*x431)));
evalcond[6]=((((-1.0)*new_r00*x433))+(((-1.0)*x430))+((new_r10*x428)));
evalcond[7]=((((-1.0)*x431))+(((-1.0)*new_r01*x433))+((new_r11*x428)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r12))+(IKabs(new_r02)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j8eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j8eval[0]=((IKabs(new_r11))+(IKabs(new_r01)));
if( IKabs(j8eval[0]) < 0.0000010000000000  )
{
{
IkReal j8eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j8eval[0]=((IKabs(new_r10))+(IKabs(new_r00)));
if( IKabs(j8eval[0]) < 0.0000010000000000  )
{
{
IkReal j8eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j8eval[0]=((IKabs((new_r11*new_r22)))+(IKabs((new_r01*new_r22))));
if( IKabs(j8eval[0]) < 0.0000010000000000  )
{
continue; // no branches [j8]

} else
{
{
IkReal j8array[2], cj8array[2], sj8array[2];
bool j8valid[2]={false};
_nj8 = 2;
CheckValue<IkReal> x436 = IKatan2WithCheck(IkReal((new_r01*new_r22)),IkReal((new_r11*new_r22)),IKFAST_ATAN2_MAGTHRESH);
if(!x436.valid){
continue;
}
IkReal x435=x436.value;
j8array[0]=((-1.0)*x435);
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
j8array[1]=((3.14159265358979)+(((-1.0)*x435)));
sj8array[1]=IKsin(j8array[1]);
cj8array[1]=IKcos(j8array[1]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
if( j8array[1] > IKPI )
{
    j8array[1]-=IK2PI;
}
else if( j8array[1] < -IKPI )
{    j8array[1]+=IK2PI;
}
j8valid[1] = true;
for(int ij8 = 0; ij8 < 2; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 2; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[5];
IkReal x437=IKsin(j8);
IkReal x438=IKcos(j8);
IkReal x439=(new_r00*x438);
IkReal x440=((1.0)*x437);
IkReal x441=(new_r10*x437);
evalcond[0]=(((new_r01*x438))+((new_r11*x437)));
evalcond[1]=(x439+x441);
evalcond[2]=((((-1.0)*new_r00*x440))+((new_r10*x438)));
evalcond[3]=((((-1.0)*new_r01*x440))+((new_r11*x438)));
evalcond[4]=(((new_r22*x441))+((new_r22*x439)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j8array[2], cj8array[2], sj8array[2];
bool j8valid[2]={false};
_nj8 = 2;
CheckValue<IkReal> x443 = IKatan2WithCheck(IkReal(new_r00),IkReal(new_r10),IKFAST_ATAN2_MAGTHRESH);
if(!x443.valid){
continue;
}
IkReal x442=x443.value;
j8array[0]=((-1.0)*x442);
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
j8array[1]=((3.14159265358979)+(((-1.0)*x442)));
sj8array[1]=IKsin(j8array[1]);
cj8array[1]=IKcos(j8array[1]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
if( j8array[1] > IKPI )
{
    j8array[1]-=IK2PI;
}
else if( j8array[1] < -IKPI )
{    j8array[1]+=IK2PI;
}
j8valid[1] = true;
for(int ij8 = 0; ij8 < 2; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 2; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[5];
IkReal x444=IKcos(j8);
IkReal x445=IKsin(j8);
IkReal x446=(new_r01*x444);
IkReal x447=(new_r11*x445);
IkReal x448=((1.0)*x445);
evalcond[0]=(x447+x446);
evalcond[1]=((((-1.0)*new_r00*x448))+((new_r10*x444)));
evalcond[2]=((((-1.0)*new_r01*x448))+((new_r11*x444)));
evalcond[3]=(((new_r22*x446))+((new_r22*x447)));
evalcond[4]=(((new_r10*new_r22*x445))+((new_r00*new_r22*x444)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j8array[2], cj8array[2], sj8array[2];
bool j8valid[2]={false};
_nj8 = 2;
CheckValue<IkReal> x450 = IKatan2WithCheck(IkReal(new_r01),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x450.valid){
continue;
}
IkReal x449=x450.value;
j8array[0]=((-1.0)*x449);
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
j8array[1]=((3.14159265358979)+(((-1.0)*x449)));
sj8array[1]=IKsin(j8array[1]);
cj8array[1]=IKcos(j8array[1]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
if( j8array[1] > IKPI )
{
    j8array[1]-=IK2PI;
}
else if( j8array[1] < -IKPI )
{    j8array[1]+=IK2PI;
}
j8valid[1] = true;
for(int ij8 = 0; ij8 < 2; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 2; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[5];
IkReal x451=IKcos(j8);
IkReal x452=IKsin(j8);
IkReal x453=(new_r22*x451);
IkReal x454=(new_r22*x452);
IkReal x455=((1.0)*x452);
evalcond[0]=(((new_r10*x452))+((new_r00*x451)));
evalcond[1]=(((new_r10*x451))+(((-1.0)*new_r00*x455)));
evalcond[2]=(((new_r11*x451))+(((-1.0)*new_r01*x455)));
evalcond[3]=(((new_r11*x454))+((new_r01*x453)));
evalcond[4]=(((new_r10*x454))+((new_r00*x453)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j8]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
CheckValue<IkReal> x457=IKPowWithIntegerCheck(sj9,-1);
if(!x457.valid){
continue;
}
IkReal x456=x457.value;
CheckValue<IkReal> x458=IKPowWithIntegerCheck(cj10,-1);
if(!x458.valid){
continue;
}
if( IKabs((x456*(x458.value)*(((((-1.0)*cj9*new_r02*sj10))+(((-1.0)*new_r01*sj9)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r02*x456)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x456*(x458.value)*(((((-1.0)*cj9*new_r02*sj10))+(((-1.0)*new_r01*sj9))))))+IKsqr((new_r02*x456))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j8array[0]=IKatan2((x456*(x458.value)*(((((-1.0)*cj9*new_r02*sj10))+(((-1.0)*new_r01*sj9))))), (new_r02*x456));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[18];
IkReal x459=IKcos(j8);
IkReal x460=IKsin(j8);
IkReal x461=((1.0)*cj10);
IkReal x462=((1.0)*new_r22);
IkReal x463=((1.0)*sj9);
IkReal x464=((1.0)*sj10);
IkReal x465=((1.0)*cj9);
IkReal x466=((1.0)*x460);
IkReal x467=(sj10*x460);
IkReal x468=(new_r11*x460);
IkReal x469=(cj9*x459);
IkReal x470=(new_r10*x460);
IkReal x471=(new_r12*x460);
IkReal x472=(x459*x463);
evalcond[0]=((((-1.0)*x472))+new_r02);
evalcond[1]=((((-1.0)*x460*x463))+new_r12);
evalcond[2]=(((new_r12*x459))+(((-1.0)*new_r02*x466)));
evalcond[3]=(((cj10*x460))+((sj10*x469))+new_r01);
evalcond[4]=((((-1.0)*x463))+x471+((new_r02*x459)));
evalcond[5]=(((new_r01*x459))+((cj9*sj10))+x468);
evalcond[6]=(x467+(((-1.0)*x461*x469))+new_r00);
evalcond[7]=(((cj9*x467))+(((-1.0)*x459*x461))+new_r11);
evalcond[8]=(((new_r10*x459))+(((-1.0)*x464))+(((-1.0)*new_r00*x466)));
evalcond[9]=(((new_r11*x459))+(((-1.0)*new_r01*x466))+(((-1.0)*x461)));
evalcond[10]=(((new_r00*x459))+(((-1.0)*cj9*x461))+x470);
evalcond[11]=((((-1.0)*x459*x464))+(((-1.0)*cj9*x460*x461))+new_r10);
evalcond[12]=(((cj9*x471))+((new_r02*x469))+(((-1.0)*sj9*x462)));
evalcond[13]=((((-1.0)*new_r21*x463))+((new_r01*x469))+((cj9*x468))+sj10);
evalcond[14]=((((-1.0)*new_r20*x465))+(((-1.0)*new_r00*x472))+(((-1.0)*x463*x470)));
evalcond[15]=((((-1.0)*new_r21*x465))+(((-1.0)*x463*x468))+(((-1.0)*new_r01*x472)));
evalcond[16]=((1.0)+(((-1.0)*new_r02*x472))+(((-1.0)*cj9*x462))+(((-1.0)*x463*x471)));
evalcond[17]=(((new_r00*x469))+(((-1.0)*new_r20*x463))+(((-1.0)*x461))+((cj9*x470)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
CheckValue<IkReal> x473 = IKatan2WithCheck(IkReal(new_r12),IkReal(new_r02),IKFAST_ATAN2_MAGTHRESH);
if(!x473.valid){
continue;
}
CheckValue<IkReal> x474=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x474.valid){
continue;
}
j8array[0]=((-1.5707963267949)+(x473.value)+(((1.5707963267949)*(x474.value))));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[18];
IkReal x475=IKcos(j8);
IkReal x476=IKsin(j8);
IkReal x477=((1.0)*cj10);
IkReal x478=((1.0)*new_r22);
IkReal x479=((1.0)*sj9);
IkReal x480=((1.0)*sj10);
IkReal x481=((1.0)*cj9);
IkReal x482=((1.0)*x476);
IkReal x483=(sj10*x476);
IkReal x484=(new_r11*x476);
IkReal x485=(cj9*x475);
IkReal x486=(new_r10*x476);
IkReal x487=(new_r12*x476);
IkReal x488=(x475*x479);
evalcond[0]=((((-1.0)*x488))+new_r02);
evalcond[1]=((((-1.0)*x476*x479))+new_r12);
evalcond[2]=(((new_r12*x475))+(((-1.0)*new_r02*x482)));
evalcond[3]=(((sj10*x485))+((cj10*x476))+new_r01);
evalcond[4]=((((-1.0)*x479))+((new_r02*x475))+x487);
evalcond[5]=(((new_r01*x475))+((cj9*sj10))+x484);
evalcond[6]=(x483+(((-1.0)*x477*x485))+new_r00);
evalcond[7]=((((-1.0)*x475*x477))+((cj9*x483))+new_r11);
evalcond[8]=((((-1.0)*x480))+(((-1.0)*new_r00*x482))+((new_r10*x475)));
evalcond[9]=((((-1.0)*new_r01*x482))+((new_r11*x475))+(((-1.0)*x477)));
evalcond[10]=(((new_r00*x475))+x486+(((-1.0)*cj9*x477)));
evalcond[11]=((((-1.0)*x475*x480))+(((-1.0)*cj9*x476*x477))+new_r10);
evalcond[12]=(((new_r02*x485))+((cj9*x487))+(((-1.0)*sj9*x478)));
evalcond[13]=((((-1.0)*new_r21*x479))+sj10+((cj9*x484))+((new_r01*x485)));
evalcond[14]=((((-1.0)*new_r00*x488))+(((-1.0)*new_r20*x481))+(((-1.0)*x479*x486)));
evalcond[15]=((((-1.0)*new_r21*x481))+(((-1.0)*new_r01*x488))+(((-1.0)*x479*x484)));
evalcond[16]=((1.0)+(((-1.0)*new_r02*x488))+(((-1.0)*cj9*x478))+(((-1.0)*x479*x487)));
evalcond[17]=((((-1.0)*x477))+((cj9*x486))+((new_r00*x485))+(((-1.0)*new_r20*x479)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}

} else
{
{
IkReal j8array[1], cj8array[1], sj8array[1];
bool j8valid[1]={false};
_nj8 = 1;
CheckValue<IkReal> x489 = IKatan2WithCheck(IkReal(new_r12),IkReal(new_r02),IKFAST_ATAN2_MAGTHRESH);
if(!x489.valid){
continue;
}
CheckValue<IkReal> x490=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x490.valid){
continue;
}
j8array[0]=((-1.5707963267949)+(x489.value)+(((1.5707963267949)*(x490.value))));
sj8array[0]=IKsin(j8array[0]);
cj8array[0]=IKcos(j8array[0]);
if( j8array[0] > IKPI )
{
    j8array[0]-=IK2PI;
}
else if( j8array[0] < -IKPI )
{    j8array[0]+=IK2PI;
}
j8valid[0] = true;
for(int ij8 = 0; ij8 < 1; ++ij8)
{
if( !j8valid[ij8] )
{
    continue;
}
_ij8[0] = ij8; _ij8[1] = -1;
for(int iij8 = ij8+1; iij8 < 1; ++iij8)
{
if( j8valid[iij8] && IKabs(cj8array[ij8]-cj8array[iij8]) < IKFAST_SOLUTION_THRESH && IKabs(sj8array[ij8]-sj8array[iij8]) < IKFAST_SOLUTION_THRESH )
{
    j8valid[iij8]=false; _ij8[1] = iij8; break; 
}
}
j8 = j8array[ij8]; cj8 = cj8array[ij8]; sj8 = sj8array[ij8];
{
IkReal evalcond[8];
IkReal x491=IKcos(j8);
IkReal x492=IKsin(j8);
IkReal x493=((1.0)*cj9);
IkReal x494=((1.0)*sj9);
IkReal x495=((1.0)*x492);
IkReal x496=(new_r12*x492);
IkReal x497=(new_r02*x491);
evalcond[0]=((((-1.0)*x491*x494))+new_r02);
evalcond[1]=(new_r12+(((-1.0)*x492*x494)));
evalcond[2]=((((-1.0)*new_r02*x495))+((new_r12*x491)));
evalcond[3]=((((-1.0)*x494))+x496+x497);
evalcond[4]=((((-1.0)*new_r22*x494))+((cj9*x496))+((cj9*x497)));
evalcond[5]=((((-1.0)*new_r20*x493))+(((-1.0)*new_r10*x492*x494))+(((-1.0)*new_r00*x491*x494)));
evalcond[6]=((((-1.0)*new_r11*x492*x494))+(((-1.0)*new_r21*x493))+(((-1.0)*new_r01*x491*x494)));
evalcond[7]=((1.0)+(((-1.0)*x494*x496))+(((-1.0)*x494*x497))+(((-1.0)*new_r22*x493)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j10eval[3];
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[2];
j10eval[0]=sj8;
j10eval[1]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  )
{
{
IkReal j10eval[3];
j10eval[0]=cj8;
j10eval[1]=cj9;
j10eval[2]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j8)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[3];
sj8=1.0;
cj8=0;
j8=1.5707963267949;
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[3];
sj8=1.0;
cj8=0;
j8=1.5707963267949;
j10eval[0]=cj9;
j10eval[1]=IKsign(cj9);
j10eval[2]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[1];
sj8=1.0;
cj8=0;
j8=1.5707963267949;
j10eval[0]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r11))+IKsqr(new_r10)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r11), new_r10);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x498=IKsin(j10);
IkReal x499=((1.0)*(IKcos(j10)));
evalcond[0]=(x498+new_r11);
evalcond[1]=((((-1.0)*x499))+new_r10);
evalcond[2]=((((-1.0)*x498))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*x499))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r11)+IKsqr(((-1.0)*new_r10))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r11, ((-1.0)*new_r10));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x500=IKcos(j10);
IkReal x501=((1.0)*(IKsin(j10)));
evalcond[0]=(x500+new_r10);
evalcond[1]=((((-1.0)*x501))+new_r11);
evalcond[2]=((((-1.0)*x501))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x500)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x502=IKcos(j10);
IkReal x503=((1.0)*(IKsin(j10)));
evalcond[0]=(x502+new_r20);
evalcond[1]=((((-1.0)*x503))+new_r21);
evalcond[2]=((((-1.0)*x503))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x502)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x504=IKsin(j10);
IkReal x505=((1.0)*(IKcos(j10)));
evalcond[0]=(x504+new_r21);
evalcond[1]=((((-1.0)*x505))+new_r20);
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x504)));
evalcond[3]=((((-1.0)*x505))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r01))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r01));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[6];
IkReal x506=IKsin(j10);
IkReal x507=IKcos(j10);
IkReal x508=((-1.0)*x507);
evalcond[0]=x506;
evalcond[1]=(new_r22*x506);
evalcond[2]=x508;
evalcond[3]=(new_r22*x508);
evalcond[4]=((((-1.0)*new_r00))+(((-1.0)*x506)));
evalcond[5]=((((-1.0)*new_r01))+(((-1.0)*x507)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x509=IKPowWithIntegerCheck(sj9,-1);
if(!x509.valid){
continue;
}
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x509.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r20*(x509.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r20*(x509.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x510=IKsin(j10);
IkReal x511=IKcos(j10);
IkReal x512=((1.0)*sj9);
IkReal x513=((1.0)*x511);
evalcond[0]=(((sj9*x511))+new_r20);
evalcond[1]=(((cj9*x510))+new_r11);
evalcond[2]=(new_r21+(((-1.0)*x510*x512)));
evalcond[3]=((((-1.0)*cj9*x513))+new_r10);
evalcond[4]=((((-1.0)*x510))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*new_r01))+(((-1.0)*x513)));
evalcond[6]=(((cj9*new_r11))+x510+(((-1.0)*new_r21*x512)));
evalcond[7]=(((cj9*new_r10))+(((-1.0)*new_r20*x512))+(((-1.0)*x513)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x514=IKPowWithIntegerCheck(IKsign(cj9),-1);
if(!x514.valid){
continue;
}
CheckValue<IkReal> x515 = IKatan2WithCheck(IkReal(((-1.0)*new_r11)),IkReal(new_r10),IKFAST_ATAN2_MAGTHRESH);
if(!x515.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(((1.5707963267949)*(x514.value)))+(x515.value));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x516=IKsin(j10);
IkReal x517=IKcos(j10);
IkReal x518=((1.0)*sj9);
IkReal x519=((1.0)*x517);
evalcond[0]=(((sj9*x517))+new_r20);
evalcond[1]=(((cj9*x516))+new_r11);
evalcond[2]=((((-1.0)*x516*x518))+new_r21);
evalcond[3]=((((-1.0)*cj9*x519))+new_r10);
evalcond[4]=((((-1.0)*x516))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*new_r01))+(((-1.0)*x519)));
evalcond[6]=(((cj9*new_r11))+x516+(((-1.0)*new_r21*x518)));
evalcond[7]=(((cj9*new_r10))+(((-1.0)*new_r20*x518))+(((-1.0)*x519)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x520 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x520.valid){
continue;
}
CheckValue<IkReal> x521=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x521.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x520.value)+(((1.5707963267949)*(x521.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x522=IKsin(j10);
IkReal x523=IKcos(j10);
IkReal x524=((1.0)*sj9);
IkReal x525=((1.0)*x523);
evalcond[0]=(((sj9*x523))+new_r20);
evalcond[1]=(((cj9*x522))+new_r11);
evalcond[2]=((((-1.0)*x522*x524))+new_r21);
evalcond[3]=((((-1.0)*cj9*x525))+new_r10);
evalcond[4]=((((-1.0)*x522))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x525))+(((-1.0)*new_r01)));
evalcond[6]=(((cj9*new_r11))+(((-1.0)*new_r21*x524))+x522);
evalcond[7]=((((-1.0)*new_r20*x524))+((cj9*new_r10))+(((-1.0)*x525)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j8)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(new_r01)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r00, new_r01);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x526=IKcos(j10);
IkReal x527=IKsin(j10);
IkReal x528=((1.0)*sj9);
IkReal x529=((1.0)*new_r10);
IkReal x530=((1.0)*new_r11);
IkReal x531=((1.0)*x526);
evalcond[0]=(((sj9*x526))+new_r20);
evalcond[1]=((((-1.0)*x527))+new_r00);
evalcond[2]=((((-1.0)*x531))+new_r01);
evalcond[3]=((((-1.0)*x527*x528))+new_r21);
evalcond[4]=((((-1.0)*x530))+((cj9*x527)));
evalcond[5]=((((-1.0)*cj9*x531))+(((-1.0)*x529)));
evalcond[6]=((((-1.0)*cj9*x530))+(((-1.0)*new_r21*x528))+x527);
evalcond[7]=((((-1.0)*cj9*x529))+(((-1.0)*new_r20*x528))+(((-1.0)*x531)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x532=IKcos(j10);
IkReal x533=IKsin(j10);
IkReal x534=((1.0)*sj8);
IkReal x535=((1.0)*x533);
IkReal x536=((1.0)*x532);
evalcond[0]=(x532+new_r20);
evalcond[1]=((((-1.0)*x535))+new_r21);
evalcond[2]=(((sj8*x532))+new_r01);
evalcond[3]=(((sj8*x533))+new_r00);
evalcond[4]=((((-1.0)*cj8*x536))+new_r11);
evalcond[5]=((((-1.0)*cj8*x535))+new_r10);
evalcond[6]=((((-1.0)*new_r00*x534))+(((-1.0)*x535))+((cj8*new_r10)));
evalcond[7]=((((-1.0)*x536))+((cj8*new_r11))+(((-1.0)*new_r01*x534)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x537=IKcos(j10);
IkReal x538=IKsin(j10);
IkReal x539=((1.0)*sj8);
IkReal x540=((1.0)*x537);
IkReal x541=((1.0)*x538);
evalcond[0]=(x538+new_r21);
evalcond[1]=((((-1.0)*x540))+new_r20);
evalcond[2]=(((sj8*x537))+new_r01);
evalcond[3]=(((sj8*x538))+new_r00);
evalcond[4]=((((-1.0)*cj8*x540))+new_r11);
evalcond[5]=((((-1.0)*cj8*x541))+new_r10);
evalcond[6]=((((-1.0)*new_r00*x539))+(((-1.0)*x541))+((cj8*new_r10)));
evalcond[7]=((((-1.0)*x540))+((cj8*new_r11))+(((-1.0)*new_r01*x539)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x542=((1.0)*new_r01);
if( IKabs(((((-1.0)*cj8*x542))+(((-1.0)*new_r00*sj8)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj8*new_r00))+(((-1.0)*sj8*x542)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*cj8*x542))+(((-1.0)*new_r00*sj8))))+IKsqr((((cj8*new_r00))+(((-1.0)*sj8*x542))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((((-1.0)*cj8*x542))+(((-1.0)*new_r00*sj8))), (((cj8*new_r00))+(((-1.0)*sj8*x542))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x543=IKsin(j10);
IkReal x544=IKcos(j10);
IkReal x545=((1.0)*sj8);
IkReal x546=((1.0)*x544);
IkReal x547=(sj8*x543);
IkReal x548=(cj8*x543);
IkReal x549=(cj8*x546);
evalcond[0]=(((new_r11*sj8))+((cj8*new_r01))+x543);
evalcond[1]=(x548+new_r01+((sj8*x544)));
evalcond[2]=(((new_r10*sj8))+(((-1.0)*x546))+((cj8*new_r00)));
evalcond[3]=((((-1.0)*x543))+(((-1.0)*new_r00*x545))+((cj8*new_r10)));
evalcond[4]=((((-1.0)*new_r01*x545))+(((-1.0)*x546))+((cj8*new_r11)));
evalcond[5]=((((-1.0)*x549))+x547+new_r00);
evalcond[6]=((((-1.0)*x549))+x547+new_r11);
evalcond[7]=((((-1.0)*x544*x545))+(((-1.0)*x548))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
IkReal x550=((1.0)*new_r00);
if( IKabs(((((-1.0)*sj8*x550))+((cj8*new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*new_r01*sj8))+(((-1.0)*cj8*x550)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*sj8*x550))+((cj8*new_r01))))+IKsqr(((((-1.0)*new_r01*sj8))+(((-1.0)*cj8*x550))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((((-1.0)*sj8*x550))+((cj8*new_r01))), ((((-1.0)*new_r01*sj8))+(((-1.0)*cj8*x550))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x551=IKsin(j10);
IkReal x552=IKcos(j10);
IkReal x553=((1.0)*sj8);
IkReal x554=((1.0)*x551);
IkReal x555=(sj8*x552);
IkReal x556=((1.0)*x552);
IkReal x557=(cj8*x554);
evalcond[0]=(((new_r10*sj8))+((cj8*new_r00))+x552);
evalcond[1]=(((new_r11*sj8))+(((-1.0)*x554))+((cj8*new_r01)));
evalcond[2]=(((sj8*x551))+((cj8*x552))+new_r00);
evalcond[3]=(((cj8*new_r10))+(((-1.0)*new_r00*x553))+(((-1.0)*x554)));
evalcond[4]=((((-1.0)*new_r01*x553))+((cj8*new_r11))+(((-1.0)*x556)));
evalcond[5]=((((-1.0)*x557))+x555+new_r01);
evalcond[6]=((((-1.0)*x557))+x555+new_r10);
evalcond[7]=((((-1.0)*cj8*x556))+(((-1.0)*x551*x553))+new_r11);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j8))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r10)+IKsqr(new_r11)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r10, new_r11);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x558=IKcos(j10);
IkReal x559=IKsin(j10);
IkReal x560=((1.0)*sj9);
IkReal x561=((1.0)*x558);
evalcond[0]=(((sj9*x558))+new_r20);
evalcond[1]=((((-1.0)*x559))+new_r10);
evalcond[2]=(new_r11+(((-1.0)*x561)));
evalcond[3]=(((cj9*x559))+new_r01);
evalcond[4]=((((-1.0)*x559*x560))+new_r21);
evalcond[5]=((((-1.0)*cj9*x561))+new_r00);
evalcond[6]=(((cj9*new_r01))+x559+(((-1.0)*new_r21*x560)));
evalcond[7]=((((-1.0)*new_r20*x560))+((cj9*new_r00))+(((-1.0)*x561)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j8)))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[3];
sj8=0;
cj8=-1.0;
j8=3.14159265358979;
j10eval[0]=sj9;
j10eval[1]=IKsign(sj9);
j10eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  || IKabs(j10eval[2]) < 0.0000010000000000  )
{
{
IkReal j10eval[1];
sj8=0;
cj8=-1.0;
j8=3.14159265358979;
j10eval[0]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  )
{
{
IkReal j10eval[2];
sj8=0;
cj8=-1.0;
j8=3.14159265358979;
j10eval[0]=cj9;
j10eval[1]=sj9;
if( IKabs(j10eval[0]) < 0.0000010000000000  || IKabs(j10eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x562=IKcos(j10);
IkReal x563=((1.0)*(IKsin(j10)));
evalcond[0]=(x562+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x563)));
evalcond[2]=((((-1.0)*new_r10))+(((-1.0)*x563)));
evalcond[3]=((((-1.0)*x562))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j9)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x564=IKsin(j10);
IkReal x565=((1.0)*(IKcos(j10)));
evalcond[0]=(x564+new_r21);
evalcond[1]=(new_r20+(((-1.0)*x565)));
evalcond[2]=((((-1.0)*x564))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x565)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j9))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r01)+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(new_r01, ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x566=IKsin(j10);
IkReal x567=((1.0)*(IKcos(j10)));
evalcond[0]=(x566+(((-1.0)*new_r01)));
evalcond[1]=((((-1.0)*x566))+(((-1.0)*new_r10)));
evalcond[2]=((((-1.0)*new_r11))+(((-1.0)*x567)));
evalcond[3]=((((-1.0)*new_r00))+(((-1.0)*x567)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j9)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r10), new_r00);
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[4];
IkReal x568=IKcos(j10);
IkReal x569=((1.0)*(IKsin(j10)));
evalcond[0]=(x568+(((-1.0)*new_r00)));
evalcond[1]=((((-1.0)*new_r10))+(((-1.0)*x569)));
evalcond[2]=((((-1.0)*x568))+(((-1.0)*new_r11)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x569)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2(((-1.0)*new_r10), ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[6];
IkReal x570=IKsin(j10);
IkReal x571=IKcos(j10);
IkReal x572=((-1.0)*x571);
evalcond[0]=x570;
evalcond[1]=(new_r22*x570);
evalcond[2]=x572;
evalcond[3]=(new_r22*x572);
evalcond[4]=((((-1.0)*x570))+(((-1.0)*new_r10)));
evalcond[5]=((((-1.0)*x571))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x573=IKPowWithIntegerCheck(cj9,-1);
if(!x573.valid){
continue;
}
CheckValue<IkReal> x574=IKPowWithIntegerCheck(sj9,-1);
if(!x574.valid){
continue;
}
if( IKabs((new_r01*(x573.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x574.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r01*(x573.value)))+IKsqr(((-1.0)*new_r20*(x574.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((new_r01*(x573.value)), ((-1.0)*new_r20*(x574.value)));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x575=IKsin(j10);
IkReal x576=IKcos(j10);
IkReal x577=((1.0)*sj9);
IkReal x578=((1.0)*new_r01);
IkReal x579=((1.0)*new_r00);
IkReal x580=((1.0)*x576);
evalcond[0]=(((sj9*x576))+new_r20);
evalcond[1]=((((-1.0)*x575*x577))+new_r21);
evalcond[2]=((((-1.0)*x575))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x580)));
evalcond[4]=((((-1.0)*x578))+((cj9*x575)));
evalcond[5]=((((-1.0)*x579))+(((-1.0)*cj9*x580)));
evalcond[6]=((((-1.0)*cj9*x578))+x575+(((-1.0)*new_r21*x577)));
evalcond[7]=((((-1.0)*cj9*x579))+(((-1.0)*new_r20*x577))+(((-1.0)*x580)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x581=IKPowWithIntegerCheck(sj9,-1);
if(!x581.valid){
continue;
}
if( IKabs((new_r21*(x581.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r21*(x581.value)))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((new_r21*(x581.value)), ((-1.0)*new_r11));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x582=IKsin(j10);
IkReal x583=IKcos(j10);
IkReal x584=((1.0)*sj9);
IkReal x585=((1.0)*new_r01);
IkReal x586=((1.0)*new_r00);
IkReal x587=((1.0)*x583);
evalcond[0]=(((sj9*x583))+new_r20);
evalcond[1]=((((-1.0)*x582*x584))+new_r21);
evalcond[2]=((((-1.0)*x582))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x587)));
evalcond[4]=((((-1.0)*x585))+((cj9*x582)));
evalcond[5]=((((-1.0)*cj9*x587))+(((-1.0)*x586)));
evalcond[6]=((((-1.0)*new_r21*x584))+x582+(((-1.0)*cj9*x585)));
evalcond[7]=((((-1.0)*new_r20*x584))+(((-1.0)*cj9*x586))+(((-1.0)*x587)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x588 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x588.valid){
continue;
}
CheckValue<IkReal> x589=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x589.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x588.value)+(((1.5707963267949)*(x589.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[8];
IkReal x590=IKsin(j10);
IkReal x591=IKcos(j10);
IkReal x592=((1.0)*sj9);
IkReal x593=((1.0)*new_r01);
IkReal x594=((1.0)*new_r00);
IkReal x595=((1.0)*x591);
evalcond[0]=(((sj9*x591))+new_r20);
evalcond[1]=((((-1.0)*x590*x592))+new_r21);
evalcond[2]=((((-1.0)*x590))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x595))+(((-1.0)*new_r11)));
evalcond[4]=(((cj9*x590))+(((-1.0)*x593)));
evalcond[5]=((((-1.0)*cj9*x595))+(((-1.0)*x594)));
evalcond[6]=((((-1.0)*new_r21*x592))+(((-1.0)*cj9*x593))+x590);
evalcond[7]=((((-1.0)*new_r20*x592))+(((-1.0)*cj9*x594))+(((-1.0)*x595)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j10eval[1];
new_r21=0;
new_r20=0;
new_r02=0;
new_r12=0;
j10eval[0]=1.0;
if( IKabs(j10eval[0]) < 0.0000000100000000  )
{
continue; // no branches [j10]

} else
{
IkReal op[2+1], zeror[2];
int numroots;
op[0]=1.0;
op[1]=0;
op[2]=-1.0;
polyroots2(op,zeror,numroots);
IkReal j10array[2], cj10array[2], sj10array[2], tempj10array[1];
int numsolutions = 0;
for(int ij10 = 0; ij10 < numroots; ++ij10)
{
IkReal htj10 = zeror[ij10];
tempj10array[0]=((2.0)*(atan(htj10)));
for(int kj10 = 0; kj10 < 1; ++kj10)
{
j10array[numsolutions] = tempj10array[kj10];
if( j10array[numsolutions] > IKPI )
{
    j10array[numsolutions]-=IK2PI;
}
else if( j10array[numsolutions] < -IKPI )
{
    j10array[numsolutions]+=IK2PI;
}
sj10array[numsolutions] = IKsin(j10array[numsolutions]);
cj10array[numsolutions] = IKcos(j10array[numsolutions]);
numsolutions++;
}
}
bool j10valid[2]={true,true};
_nj10 = 2;
for(int ij10 = 0; ij10 < numsolutions; ++ij10)
    {
if( !j10valid[ij10] )
{
    continue;
}
    j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
htj10 = IKtan(j10/2);

_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < numsolutions; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
    }

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j10]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}
}
}
}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x597=IKPowWithIntegerCheck(sj9,-1);
if(!x597.valid){
continue;
}
IkReal x596=x597.value;
CheckValue<IkReal> x598=IKPowWithIntegerCheck(cj8,-1);
if(!x598.valid){
continue;
}
CheckValue<IkReal> x599=IKPowWithIntegerCheck(cj9,-1);
if(!x599.valid){
continue;
}
if( IKabs((x596*(x598.value)*(x599.value)*((((new_r20*sj8))+(((-1.0)*new_r01*sj9)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x596)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x596*(x598.value)*(x599.value)*((((new_r20*sj8))+(((-1.0)*new_r01*sj9))))))+IKsqr(((-1.0)*new_r20*x596))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((x596*(x598.value)*(x599.value)*((((new_r20*sj8))+(((-1.0)*new_r01*sj9))))), ((-1.0)*new_r20*x596));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[12];
IkReal x600=IKsin(j10);
IkReal x601=IKcos(j10);
IkReal x602=(cj9*sj8);
IkReal x603=((1.0)*sj9);
IkReal x604=((1.0)*sj8);
IkReal x605=((1.0)*cj8);
IkReal x606=(cj8*new_r00);
IkReal x607=(cj8*new_r01);
IkReal x608=((1.0)*x601);
IkReal x609=(cj9*x600);
IkReal x610=(cj9*x608);
evalcond[0]=(((sj9*x601))+new_r20);
evalcond[1]=((((-1.0)*x600*x603))+new_r21);
evalcond[2]=(((new_r11*sj8))+x607+x609);
evalcond[3]=((((-1.0)*x600))+((cj8*new_r10))+(((-1.0)*new_r00*x604)));
evalcond[4]=(((cj8*new_r11))+(((-1.0)*x608))+(((-1.0)*new_r01*x604)));
evalcond[5]=(((sj8*x601))+((cj8*x609))+new_r01);
evalcond[6]=(((new_r10*sj8))+(((-1.0)*x610))+x606);
evalcond[7]=((((-1.0)*cj9*x601*x605))+((sj8*x600))+new_r00);
evalcond[8]=(((x600*x602))+new_r11+(((-1.0)*x601*x605)));
evalcond[9]=((((-1.0)*x602*x608))+(((-1.0)*x600*x605))+new_r10);
evalcond[10]=(((new_r11*x602))+((cj9*x607))+x600+(((-1.0)*new_r21*x603)));
evalcond[11]=((((-1.0)*new_r20*x603))+((new_r10*x602))+((cj9*x606))+(((-1.0)*x608)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x612=IKPowWithIntegerCheck(sj9,-1);
if(!x612.valid){
continue;
}
IkReal x611=x612.value;
CheckValue<IkReal> x613=IKPowWithIntegerCheck(sj8,-1);
if(!x613.valid){
continue;
}
if( IKabs((x611*(x613.value)*(((((-1.0)*cj8*cj9*new_r20))+(((-1.0)*new_r00*sj9)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x611)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x611*(x613.value)*(((((-1.0)*cj8*cj9*new_r20))+(((-1.0)*new_r00*sj9))))))+IKsqr(((-1.0)*new_r20*x611))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j10array[0]=IKatan2((x611*(x613.value)*(((((-1.0)*cj8*cj9*new_r20))+(((-1.0)*new_r00*sj9))))), ((-1.0)*new_r20*x611));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[12];
IkReal x614=IKsin(j10);
IkReal x615=IKcos(j10);
IkReal x616=(cj9*sj8);
IkReal x617=((1.0)*sj9);
IkReal x618=((1.0)*sj8);
IkReal x619=((1.0)*cj8);
IkReal x620=(cj8*new_r00);
IkReal x621=(cj8*new_r01);
IkReal x622=((1.0)*x615);
IkReal x623=(cj9*x614);
IkReal x624=(cj9*x622);
evalcond[0]=(((sj9*x615))+new_r20);
evalcond[1]=((((-1.0)*x614*x617))+new_r21);
evalcond[2]=(((new_r11*sj8))+x621+x623);
evalcond[3]=(((cj8*new_r10))+(((-1.0)*x614))+(((-1.0)*new_r00*x618)));
evalcond[4]=((((-1.0)*x622))+((cj8*new_r11))+(((-1.0)*new_r01*x618)));
evalcond[5]=(((sj8*x615))+((cj8*x623))+new_r01);
evalcond[6]=(((new_r10*sj8))+(((-1.0)*x624))+x620);
evalcond[7]=(((sj8*x614))+(((-1.0)*cj9*x615*x619))+new_r00);
evalcond[8]=(((x614*x616))+new_r11+(((-1.0)*x615*x619)));
evalcond[9]=((((-1.0)*x614*x619))+new_r10+(((-1.0)*x616*x622)));
evalcond[10]=((((-1.0)*new_r21*x617))+((new_r11*x616))+x614+((cj9*x621)));
evalcond[11]=((((-1.0)*x622))+((new_r10*x616))+(((-1.0)*new_r20*x617))+((cj9*x620)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j10array[1], cj10array[1], sj10array[1];
bool j10valid[1]={false};
_nj10 = 1;
CheckValue<IkReal> x625 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x625.valid){
continue;
}
CheckValue<IkReal> x626=IKPowWithIntegerCheck(IKsign(sj9),-1);
if(!x626.valid){
continue;
}
j10array[0]=((-1.5707963267949)+(x625.value)+(((1.5707963267949)*(x626.value))));
sj10array[0]=IKsin(j10array[0]);
cj10array[0]=IKcos(j10array[0]);
if( j10array[0] > IKPI )
{
    j10array[0]-=IK2PI;
}
else if( j10array[0] < -IKPI )
{    j10array[0]+=IK2PI;
}
j10valid[0] = true;
for(int ij10 = 0; ij10 < 1; ++ij10)
{
if( !j10valid[ij10] )
{
    continue;
}
_ij10[0] = ij10; _ij10[1] = -1;
for(int iij10 = ij10+1; iij10 < 1; ++iij10)
{
if( j10valid[iij10] && IKabs(cj10array[ij10]-cj10array[iij10]) < IKFAST_SOLUTION_THRESH && IKabs(sj10array[ij10]-sj10array[iij10]) < IKFAST_SOLUTION_THRESH )
{
    j10valid[iij10]=false; _ij10[1] = iij10; break; 
}
}
j10 = j10array[ij10]; cj10 = cj10array[ij10]; sj10 = sj10array[ij10];
{
IkReal evalcond[12];
IkReal x627=IKsin(j10);
IkReal x628=IKcos(j10);
IkReal x629=(cj9*sj8);
IkReal x630=((1.0)*sj9);
IkReal x631=((1.0)*sj8);
IkReal x632=((1.0)*cj8);
IkReal x633=(cj8*new_r00);
IkReal x634=(cj8*new_r01);
IkReal x635=((1.0)*x628);
IkReal x636=(cj9*x627);
IkReal x637=(cj9*x635);
evalcond[0]=(((sj9*x628))+new_r20);
evalcond[1]=((((-1.0)*x627*x630))+new_r21);
evalcond[2]=(((new_r11*sj8))+x636+x634);
evalcond[3]=((((-1.0)*new_r00*x631))+((cj8*new_r10))+(((-1.0)*x627)));
evalcond[4]=((((-1.0)*new_r01*x631))+(((-1.0)*x635))+((cj8*new_r11)));
evalcond[5]=(((cj8*x636))+((sj8*x628))+new_r01);
evalcond[6]=(((new_r10*sj8))+(((-1.0)*x637))+x633);
evalcond[7]=((((-1.0)*cj9*x628*x632))+((sj8*x627))+new_r00);
evalcond[8]=((((-1.0)*x628*x632))+new_r11+((x627*x629)));
evalcond[9]=((((-1.0)*x629*x635))+(((-1.0)*x627*x632))+new_r10);
evalcond[10]=((((-1.0)*new_r21*x630))+((new_r11*x629))+x627+((cj9*x634)));
evalcond[11]=((((-1.0)*x635))+(((-1.0)*new_r20*x630))+((new_r10*x629))+((cj9*x633)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j4;
vinfos[0].indices[0] = _ij4[0];
vinfos[0].indices[1] = _ij4[1];
vinfos[0].maxsolutions = _nj4;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j5;
vinfos[1].indices[0] = _ij5[0];
vinfos[1].indices[1] = _ij5[1];
vinfos[1].maxsolutions = _nj5;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j6;
vinfos[2].indices[0] = _ij6[0];
vinfos[2].indices[1] = _ij6[1];
vinfos[2].maxsolutions = _nj6;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j7;
vinfos[3].indices[0] = _ij7[0];
vinfos[3].indices[1] = _ij7[1];
vinfos[3].maxsolutions = _nj7;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j8;
vinfos[4].indices[0] = _ij8[0];
vinfos[4].indices[1] = _ij8[1];
vinfos[4].maxsolutions = _nj8;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j9;
vinfos[5].indices[0] = _ij9[0];
vinfos[5].indices[1] = _ij9[1];
vinfos[5].maxsolutions = _nj9;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j10;
vinfos[6].indices[0] = _ij10[0];
vinfos[6].indices[1] = _ij10[1];
vinfos[6].maxsolutions = _nj10;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}
}
}
}
}static inline void polyroots3(IkReal rawcoeffs[3+1], IkReal rawroots[3], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        polyroots2(&rawcoeffs[1], &rawroots[0], numroots);
        return;
    }
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[3];
    const int maxsteps = 110;
    for(int i = 0; i < 3; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[3];
    IkReal err[3];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < 3; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < 3; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < 3; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < 3; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    // sort roots hoping that it solution indices become more robust to slight change in coeffs
    std::sort(roots, roots+3, ComplexLess<IkReal>());

    numroots = 0;
    bool visited[3] = {false};
    for(int i = 0; i < 3; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < 3; ++j) {
                // care about error in real much more than imaginary
                if( abs(real(roots[i])-real(roots[j])) < tolsqrt && (abs(imag(roots[i])-imag(roots[j])) < 0.002 || abs(imag(roots[i])+imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( IKabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}
static inline void polyroots2(IkReal rawcoeffs[2+1], IkReal rawroots[2], int& numroots) {
    IkReal det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
    if( det < 0 ) {
        numroots=0;
    }
    else if( det == 0 ) {
        rawroots[0] = -0.5*rawcoeffs[1]/rawcoeffs[0];
        numroots = 1;
    }
    else {
        det = IKsqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]);//rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}
static inline void polyroots4(IkReal rawcoeffs[4+1], IkReal rawroots[4], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        polyroots3(&rawcoeffs[1], &rawroots[0], numroots);
        return;
    }
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[4];
    const int maxsteps = 110;
    for(int i = 0; i < 4; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[4];
    IkReal err[4];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < 4; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < 4; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < 4; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < 4; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    // sort roots hoping that it solution indices become more robust to slight change in coeffs
    std::sort(roots, roots+4, ComplexLess<IkReal>());

    numroots = 0;
    bool visited[4] = {false};
    for(int i = 0; i < 4; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < 4; ++j) {
                // care about error in real much more than imaginary
                if( abs(real(roots[i])-real(roots[j])) < tolsqrt && (abs(imag(roots[i])-imag(roots[j])) < 0.002 || abs(imag(roots[i])+imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( IKabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}
};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "<robot:GenericRobot - r300 (37dca05bfcc6586f17a19d28f444da97)>"; }

IKFAST_API const char* GetIkFastVersion() { return "0x1000004b"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
    return 0;
}

#endif
