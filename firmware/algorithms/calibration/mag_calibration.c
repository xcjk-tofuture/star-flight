#include "mag_calibration.h"
#include <math.h>
#include <string.h>
void uav_mag_correct(const float m[9], const float bias[3], const float raw[3], float out[3]) {
    float x[3]={raw[0]-bias[0],raw[1]-bias[1],raw[2]-bias[2]};
    for (unsigned i=0;i<3;i++) out[i]=m[3*i]*x[0]+m[3*i+1]*x[1]+m[3*i+2]*x[2];
}
void uav_mag_calibrator_init(uav_mag_calibrator_t *c) {
    memset(c,0,sizeof(*c)); c->rng=0x5a17c9e3u;
}

static int sphere_direction(const float matrix[9], const float bias[3], const float raw[3], float d[3]) {
    uav_mag_correct(matrix,bias,raw,d);
    float norm=sqrtf(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    if (!isfinite(norm) || norm<1e-5f) { memset(d,0,3*sizeof(float)); return 0; }
    for (unsigned i=0;i<3;i++) d[i]/=norm;
    return 1;
}
static unsigned sphere_bin(const float d[3]) {
    unsigned band=(unsigned)fmaxf(0,fminf(3,(d[2]+1)*2));
    float azimuth=atan2f(d[1],d[0])+3.14159265359f;
    return band*8+((unsigned)(azimuth*(8/6.28318530718f))&7u);
}
static void bin_center(unsigned bin, float d[3]) {
    float z=-.75f+.5f*(bin/8),angle=-3.14159265359f+((bin%8)+.5f)*(6.28318530718f/8);
    float radius=sqrtf(1-z*z); d[0]=radius*cosf(angle); d[1]=radius*sinf(angle); d[2]=z;
}
void uav_mag_calibrator_update_cursor(uav_mag_calibrator_t *c, const float current[3]) {
    c->sphere.cursor_valid=(uint8_t)sphere_direction(c->preview_matrix,c->preview_bias,current,c->sphere.cursor);
}
void uav_mag_calibrator_refresh_sphere(uav_mag_calibrator_t *c,
                                      const uav_mag_calibration_t *model, const float current[3], uint32_t ms) {
    uav_mag_calibration_t check={0};
    if (model) { check=*model; check.coverage=255; check.samples=UAV_MAG_CAL_MIN_SAMPLES; }
    int fitted=model && model->samples && uav_mag_calibration_valid(&check);
    float minimum[3]={0},maximum[3]={0};
    if (c->count) {
        memcpy(minimum,c->points[0],sizeof(minimum)); memcpy(maximum,minimum,sizeof(maximum));
        for (unsigned i=0;i<c->count;i++) for (unsigned j=0;j<3;j++) {
            if (c->points[i][j]<minimum[j]) minimum[j]=c->points[i][j];
            if (c->points[i][j]>maximum[j]) maximum[j]=c->points[i][j];
        }
    }
    if (fitted) {
        memcpy(c->preview_bias,model->bias,sizeof(c->preview_bias));
        memcpy(c->preview_matrix,model->matrix,sizeof(c->preview_matrix));
    } else {
        memset(c->preview_matrix,0,sizeof(c->preview_matrix));
        for (unsigned i=0;i<3;i++) {
            c->preview_bias[i]=.5f*(minimum[i]+maximum[i]);
            c->preview_matrix[3*i+i]=1/fmaxf(10,.5f*(maximum[i]-minimum[i]));
        }
    }
    memset(c->bin_count,0,sizeof(c->bin_count)); c->sphere.mask=0; c->sphere.covered=0;
    for (unsigned i=0;i<c->count;i++) {
        float d[3];
        if (sphere_direction(c->preview_matrix,c->preview_bias,c->points[i],d)) c->bin_count[sphere_bin(d)]++;
    }
    c->sphere.ready=c->count>=24;
    for (unsigned i=0;i<3;i++) if (maximum[i]-minimum[i]<20) c->sphere.ready=0;
    c->sphere.fitted=(uint8_t)fitted;
    for (unsigned i=0;i<UAV_MAG_SPHERE_BINS;i++) if (c->bin_count[i]>=3 && c->sphere.ready) {
        c->sphere.mask|=1u<<i; c->sphere.covered++;
    }
    uav_mag_calibrator_update_cursor(c,current);
    unsigned target=0; float best=-2;
    for (unsigned i=0;i<UAV_MAG_SPHERE_BINS;i++) {
        float center[3]; bin_center(i,center);
        float proximity=center[0]*c->sphere.cursor[0]+center[1]*c->sphere.cursor[1]+center[2]*c->sphere.cursor[2];
        if (c->bin_count[i]<c->bin_count[target] || (c->bin_count[i]==c->bin_count[target] && proximity>best)) {
            target=i; best=proximity;
        }
    }
    c->sphere.target=(uint8_t)target; bin_center(target,c->sphere.goal); c->sphere_ms=ms;
}

int uav_mag_calibrator_feed(uav_mag_calibrator_t *c, const float raw[3], uint32_t ms) {
    float change=0;
    for (unsigned i=0;i<3;i++) {
        if (!isfinite(raw[i]) || fabsf(raw[i])>1250) return 0;
        float d=raw[i]-c->last[i]; change+=d*d;
    }
    if (c->have_last && ((uint32_t)(ms-c->last_ms)<80u || change<4.0f)) return 0;
    c->last_ms=ms; c->have_last=1; memcpy(c->last,raw,sizeof(c->last)); c->seen++;
    unsigned index=c->count;
    if (c->count<UAV_MAG_CAL_CAPACITY) c->count++;
    else {
        /* Uniform reservoir avoids being permanently filled by the first poses. */
        c->rng=c->rng*1664525u+1013904223u; index=c->rng%c->seen;
        if (index>=UAV_MAG_CAL_CAPACITY) return 1;
    }
    memcpy(c->points[index],raw,3*sizeof(float));
    return 1;
}
/* Symmetric Jacobi eigensolver. Columns of v are the eigenvectors. */
static void eigen(double a[3][3], double v[3][3]) {
    memset(v,0,9*sizeof(double)); for (unsigned i=0;i<3;i++) v[i][i]=1;
    for (unsigned iter=0;iter<32;iter++) {
        unsigned p=0,q=1;
        if (fabs(a[0][2])>fabs(a[p][q])) { p=0; q=2; }
        if (fabs(a[1][2])>fabs(a[p][q])) { p=1; q=2; }
        if (fabs(a[p][q])<1e-12) break;
        double angle=.5*atan2(2*a[p][q],a[q][q]-a[p][p]);
        double cs=cos(angle),sn=sin(angle),ap=a[p][p],aq=a[q][q],cross=a[p][q];
        a[p][p]=cs*cs*ap-2*cs*sn*cross+sn*sn*aq;
        a[q][q]=sn*sn*ap+2*cs*sn*cross+cs*cs*aq;
        a[p][q]=a[q][p]=0;
        for (unsigned k=0;k<3;k++) {
            if (k!=p && k!=q) {
                double x=a[k][p],y=a[k][q];
                a[k][p]=a[p][k]=cs*x-sn*y; a[k][q]=a[q][k]=sn*x+cs*y;
            }
            double x=v[k][p],y=v[k][q]; v[k][p]=cs*x-sn*y; v[k][q]=sn*x+cs*y;
        }
    }
}
int uav_mag_calibration_valid(const uav_mag_calibration_t *c) {
    if (!c || !isfinite(c->field_ut) || c->field_ut<10 || c->field_ut>100 ||
        !isfinite(c->rms_fraction) || c->rms_fraction<0 || c->rms_fraction>.08f ||
        c->coverage!=255 || c->samples<UAV_MAG_CAL_MIN_SAMPLES || c->samples>UAV_MAG_CAL_CAPACITY) return 0;
    double a[3][3],v[3][3];
    for (unsigned i=0;i<3;i++) {
        if (!isfinite(c->bias[i]) || fabsf(c->bias[i])>500) return 0;
        for (unsigned j=0;j<3;j++) {
            float value=c->matrix[3*i+j];
            if (!isfinite(value) || fabsf(value)>5 || fabsf(value-c->matrix[3*j+i])>1e-4f) return 0;
            a[i][j]=value;
        }
    }
    eigen(a,v);
    double low=a[0][0],high=low,det=1;
    for (unsigned i=0;i<3;i++) {
        if (a[i][i]<.2 || a[i][i]>5) return 0;
        if (a[i][i]<low) low=a[i][i];
        if (a[i][i]>high) high=a[i][i];
        det*=a[i][i];
    }
    return high/low<=5 && det>.95 && det<1.05;
}
int uav_mag_calibrator_fit(uav_mag_calibrator_t *c, uav_mag_calibration_t *out) {
    if (c->count<UAV_MAG_CAL_MIN_SAMPLES) return UAV_MAG_CAL_SAMPLES;
    double mean[3]={0},lo[3],hi[3];
    for (unsigned j=0;j<3;j++) lo[j]=hi[j]=c->points[0][j];
    for (unsigned i=0;i<c->count;i++) for (unsigned j=0;j<3;j++) {
        double x=c->points[i][j]; mean[j]+=x;
        if (x<lo[j]) lo[j]=x;
        if (x>hi[j]) hi[j]=x;
    }
    for (unsigned j=0;j<3;j++) {
        mean[j]/=c->count;
        if (hi[j]-lo[j]<20) return UAV_MAG_CAL_SPAN;
    }
    memset(c->solve,0,sizeof(c->solve));
    /* Center and normalize before solving x^T A x + b^T x = 1. */
    for (unsigned i=0;i<c->count;i++) {
        double x=(c->points[i][0]-mean[0])/50,y=(c->points[i][1]-mean[1])/50,
               z=(c->points[i][2]-mean[2])/50;
        double f[9]={x*x,y*y,z*z,2*x*y,2*x*z,2*y*z,x,y,z};
        for (unsigned r=0;r<9;r++) {
            c->solve[r][9]+=f[r];
            for (unsigned k=0;k<9;k++) c->solve[r][k]+=f[r]*f[k];
        }
    }
    double scale=0;
    for (unsigned i=0;i<9;i++) if (c->solve[i][i]>scale) scale=c->solve[i][i];
    for (unsigned p=0;p<9;p++) {
        unsigned pivot=p;
        for (unsigned i=p+1;i<9;i++) if (fabs(c->solve[i][p])>fabs(c->solve[pivot][p])) pivot=i;
        if (fabs(c->solve[pivot][p])<scale*1e-9) return UAV_MAG_CAL_SINGULAR;
        for (unsigned k=p;k<10;k++) {
            double tmp=c->solve[p][k]; c->solve[p][k]=c->solve[pivot][k]; c->solve[pivot][k]=tmp;
        }
        double divisor=c->solve[p][p];
        for (unsigned k=p;k<10;k++) c->solve[p][k]/=divisor;
        for (unsigned r=0;r<9;r++) if (r!=p) {
            double factor=c->solve[r][p];
            for (unsigned k=p;k<10;k++) c->solve[r][k]-=factor*c->solve[p][k];
        }
    }
    double a[3][3]={{c->solve[0][9],c->solve[3][9],c->solve[4][9]},
                    {c->solve[3][9],c->solve[1][9],c->solve[5][9]},
                    {c->solve[4][9],c->solve[5][9],c->solve[2][9]}},v[3][3];
    eigen(a,v);
    double center[3]={0},radius=1;
    for (unsigned k=0;k<3;k++) {
        if (!isfinite(a[k][k]) || a[k][k]<=1e-8) return UAV_MAG_CAL_SHAPE;
        double projection=0;
        for (unsigned j=0;j<3;j++) projection+=v[j][k]*c->solve[6+j][9];
        for (unsigned j=0;j<3;j++) center[j]-=.5*v[j][k]*projection/a[k][k];
    }
    for (unsigned j=0;j<3;j++) radius-=.5*c->solve[6+j][9]*center[j];
    if (!isfinite(radius) || radius<=0) return UAV_MAG_CAL_SHAPE;
    double lambda[3],low=1e30,high=0,product=1;
    for (unsigned k=0;k<3;k++) {
        lambda[k]=a[k][k]/radius;
        if (lambda[k]<low) low=lambda[k];
        if (lambda[k]>high) high=lambda[k];
        product*=lambda[k];
    }
    if (high/low>25) return UAV_MAG_CAL_SHAPE;
    double field=pow(product,-1.0/6.0);
    uav_mag_calibration_t result={0}; result.field_ut=(float)(50*field); result.samples=c->count;
    for (unsigned i=0;i<3;i++) {
        result.bias[i]=(float)(mean[i]+50*center[i]);
        for (unsigned j=0;j<3;j++) {
            double entry=0;
            for (unsigned k=0;k<3;k++) entry+=v[i][k]*sqrt(lambda[k])*v[j][k];
            result.matrix[3*i+j]=(float)(field*entry);
        }
    }
    unsigned octants[8]={0}; float minimum[3]={0},maximum[3]={0}; double residual=0;
    for (unsigned i=0;i<c->count;i++) {
        float corrected[3]; uav_mag_correct(result.matrix,result.bias,c->points[i],corrected);
        float norm=sqrtf(corrected[0]*corrected[0]+corrected[1]*corrected[1]+corrected[2]*corrected[2]);
        double error=norm/result.field_ut-1; residual+=error*error;
        unsigned octant=0;
        for (unsigned j=0;j<3;j++) {
            if (corrected[j]>=0) octant|=1u<<j;
            if (corrected[j]<minimum[j]) minimum[j]=corrected[j];
            if (corrected[j]>maximum[j]) maximum[j]=corrected[j];
        }
        octants[octant]++;
    }
    result.rms_fraction=(float)sqrt(residual/c->count);
    for (unsigned i=0;i<8;i++) if (octants[i]>=3) result.coverage|=1u<<i;
    *out=result; /* Quality is available even if additional rotation is required. */
    if (result.coverage!=255) return UAV_MAG_CAL_COVERAGE;
    for (unsigned j=0;j<3;j++)
        if (minimum[j]>-.75f*result.field_ut || maximum[j]<.75f*result.field_ut) return UAV_MAG_CAL_COVERAGE;
    if (result.rms_fraction>.08f) return UAV_MAG_CAL_RESIDUAL;
    return uav_mag_calibration_valid(&result) ? UAV_MAG_CAL_OK:UAV_MAG_CAL_SHAPE;
}
