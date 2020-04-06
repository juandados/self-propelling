#include <math.h>
#include "matrix.h"
#include "mex.h"   //--This one is required
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

double sweepingLF(double**** phi, int N1, int N2, int N3, int N4, double***** xs, double* dx,\
        double Umax, double Dmax)
{
    int i,j,k,h;
    int s1,s2,s3,s4;
    double p, q, r, t;
    double c, H, phiTemp, phiOld, error;
    double sigma1, sigma2, sigma3, sigma4;
    
    double U1,U2,D1,D2;
    
    double tmp;
    
    int count;
    count = 0;
    error = 0;
   
    // mexPrintf("here in sweeping beginning place");
    for (s1=1; s1>=-1; s1-=2 )
    for (s2=1; s2>=-1; s2-=2 )
    for (s3=1; s3>=-1; s3-=2 )
    for (s4=1; s4>=-1; s4-=2 )
    {
        // LF sweeping module
        for ( i=(s1<0 ? (N1-2):1); (s1<0 ? i>=1:i<=(N1-2)); i+=s1 )
        for ( j=(s2<0 ? (N2-2):1); (s2<0 ? j>=1:j<=(N2-2)); j+=s2 ) 
        for ( k=(s3<0 ? (N3-2):1); (s3<0 ? k>=1:k<=(N3-2)); k+=s3 ) 
        for ( h=(s4<0 ? (N4-2):1); (s4<0 ? h>=1:h<=(N4-2)); h+=s4 )
        {
            phiOld = phi[i][j][k][h];
            // mexPrintf("phiold in front:%f", phiOld);
            
            count += 1;
            
            //mexPrintf("see here");
            p = (phi[i+1][j][k][h] - phi[i-1][j][k][h])/(2*dx[0]);
            q = (phi[i][j+1][k][h] - phi[i][j-1][k][h])/(2*dx[1]);
            r = (phi[i][j][k+1][h] - phi[i][j][k-1][h])/(2*dx[2]);
            t = (phi[i][j][k][h+1] - phi[i][j][k][h-1])/(2*dx[3]);
            
            //U1 = Umax*q/sqrt(q*q + t*t + 0.0000001);
            //U2 = Umax*t/sqrt(q*q + t*t + 0.0000001);
            //D1 = Dmax*q/sqrt(q*q + t*t + 0.0000001);
            //D2 = Dmax*t/sqrt(q*q + t*t + 0.0000001);


            sigma1 =  abs(xs[i][j][k][h][1]);
            //sigma2 =  abs(U1-D1);
            //sigma2 = abs(U1) + abs(D1);
            sigma2 = 0;
            sigma3 =  abs(xs[i][j][k][h][3]);
            //sigma4 =  abs(U2-D2);
            //sigma4 = abs(U2) + abs(D2);
            sigma4 = 0;

            c = (sigma1/dx[0] + sigma2/dx[1] + sigma3/dx[2] + sigma4/dx[3]);
        
            //H = (-1) * (p*(xs[i][j][k][h][1]) + q*U1 - q*D1 \
            //        + r*(xs[i][j][k][h][3]) + t*U2 - t*D2 \
            //        + 1);
            H = (-1) * (p*xs[i][j][k][h][1] \
                    + r*xs[i][j][k][h][3] \
                    + 1);
            //mexPrintf("H=%f\n", H);


            phiTemp = -H + sigma1*(phi[i+1][j][k][h] + phi[i-1][j][k][h])/(2*dx[0])\
                    + sigma2*(phi[i][j+1][k][h] + phi[i][j-1][k][h])/(2*dx[1])\
                    + sigma3*(phi[i][j][k+1][h] + phi[i][j][k-1][h])/(2*dx[2])\
                    + sigma4*(phi[i][j][k][h+1] + phi[i][j][k][h-1])/(2*dx[3]);
            //mexPrintf("phiTemp=%f\n", phiTemp);
            
            phi[i][j][k][h] = min(phiTemp/c, phiOld);
            
            
            error = max(error, abs(phiOld - phi[i][j][k][h]));   
            //mexPrintf("error=%f", error);
            

        }
    // Boundary conditions were removed 19 sept 2019
        for ( j = 0; j <= (N2-1); j++)
        for ( k = 0; k <= (N3-1); k++)
        for ( h = 0; h <= (N4-1); h++)
        {
            phiOld = phi[0][j][k][h]; 
            phi[0][j][k][h] = min(max(2*phi[1][j][k][h] - phi[2][j][k][h], phi[2][j][k][h]), phiOld); 
            error = max(error, phiOld - phi[0][j][k][h]);
            
            phiOld = phi[N1-1][j][k][h];
            phi[N1-1][j][k][h] = min(max(2*phi[N1-2][j][k][h] - phi[N1-3][j][k][h], phi[N1-3][j][k][h]), phiOld); 
            error = max(error, phiOld - phi[N1-1][j][k][h]);
        }

        for ( k = 0; k <= (N3-1); k++)
        for ( i = 0; i <= (N1-1); i++)
        for ( h = 0; h <= (N4-1); h++)
        {
            phiOld = phi[i][0][k][h]; 
            phi[i][0][k][h] = min(max(2*phi[i][1][k][h] - phi[i][2][k][h], phi[i][2][k][h]), phiOld); 
            error = max(error, phiOld - phi[i][0][k][h]);
            
            phiOld = phi[i][N2-1][k][h];
            phi[i][N2-1][k][h] = min(max(2*phi[i][N2-2][k][h] - phi[i][N2-3][k][h], phi[i][N2-3][k][h]), phiOld); 
            error = max(error, phiOld - phi[i][N2-1][k][h]);
            
        }
        
        for ( i = 0; i <= (N1-1); i++)
        for ( j = 0; j <= (N2-1); j++)
        for ( h = 0; h <= (N4-1); h++)
        {
            phiOld = phi[i][j][0][h]; 
            phi[i][j][0][h] = min(max(2*phi[i][j][1][h] - phi[i][j][2][h], phi[i][j][2][h]), phiOld); 
            error = max(error, phiOld - phi[i][j][0][h]);
            
            phiOld = phi[i][j][N3-1][h];
            phi[i][j][N3-1][h] = min(max(2*phi[i][j][N3-2][h] - phi[i][j][N3-3][h], phi[i][j][N3-3][h]), phiOld); 
            error = max(error, phiOld - phi[i][j][N3-1][h]);
        }
        
        for ( i = 0; i <= (N1-1); i++)
        for ( j = 0; j <= (N2-1); j++)
        for ( k = 0; k <= (N3-1); k++)
        {
            phiOld = phi[i][j][k][0]; 
            phi[i][j][k][0] = min(max(2*phi[i][j][k][1] - phi[i][j][k][2], phi[i][j][k][2]), phiOld); 
            error = max(error, phiOld - phi[i][j][k][0]);
            
            phiOld = phi[i][j][k][N4-1];
            phi[i][j][k][N4-1] = min(max(2*phi[i][j][k][N4-2] - phi[i][j][k][N4-3], phi[i][j][k][N4-3]), phiOld); 
            error = max(error, phiOld - phi[i][j][k][N4-1]);
        }
        // end of boundary conditions
    }
    mexPrintf("count = %d", count);
    return error;
}



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //---Inside mexFunction---

    
    double *phiValues, *xsValues, *dxValues;
    double ****phi, *****xs, *dx;
    //const int *N;
    const mwSize *N;
	int numIter;
    
    double Umax, Dmax, error,TOL; 
    int i,j,k,h,l,N1,N2,N3,N4;
    
    double numInfty = 1e6;
    
    //Get the input

    phiValues   = mxGetPr(prhs[0]);
    xsValues    = mxGetPr(prhs[1]);
    dxValues    = mxGetPr(prhs[2]);
    
    Umax = (double)mxGetScalar(prhs[3]);
    Dmax = (double)mxGetScalar(prhs[4]);
    
    
    numIter     = (int)mxGetScalar(prhs[5]);
    TOL         = (double)mxGetScalar(prhs[6]);
    
    N           = mxGetDimensions(prhs[0]);
    
    N1 = N[0]; N2 = N[1]; N3 = N[2]; N4 = N[3];
    // mexPrintf("here first place");
    
    // memory allocation & value assignment
	phi   = (double ****) malloc (N1 * sizeof(double***)); 
    dx    = (double *) malloc (4 * sizeof(double)); 
    xs    = (double *****) malloc (N1 * sizeof(double****)); 
    // mexPrintf("here bong place");
    
	for (i=0; i<N1; i++)
    {
		phi[i]   = (double ***) malloc ( N2 * sizeof(double**));
        xs[i]    = (double ****) malloc ( N2 * sizeof(double***));
        for (j=0; j<N2; j++)
        {
            phi[i][j]   = (double **) malloc ( N3 * sizeof(double*));
            xs[i][j]    = (double ***) malloc ( N3 * sizeof(double**));
            for (k=0; k<N3; k++)
            {
                phi[i][j][k] = (double *) malloc ( N4 * sizeof(double));
                xs[i][j][k]  = (double **) malloc ( N4 * sizeof(double*));
                for (h=0; h<N4; h++)
                {
                    phi[i][j][k][h] = phiValues[((h*N3+k)*N2+j)*N1+i];
                    xs[i][j][k][h]  = (double *) malloc (4 * sizeof(double));
                    for (l=0; l<4; l++)
                    {
                        xs[i][j][k][h][l] = xsValues[(((l*N4+h)*N3+k)*N2+j)*N1+i];
                    }
                }
                
            }
        }
    }
    // mexPrintf("here second place");  
    for (i=0; i<4; i++)
    {
        dx[i] = dxValues[i];
    }
   
    // run LF sweeping algorithm
    for(k=0; k<numIter; k++) 
    {
        error = sweepingLF(phi, N1,N2,N3,N4, xs, dx, Umax, Dmax);
        // mexPrintf("here third place");
        
        mexPrintf("Error = %g at iteration %i. \n", error, k);
        mexEvalString("drawnow;");
        if (error <= TOL) {
            mexPrintf("Stopped at iteration %i. \n", k);
            break;
        } 
        
    }
    
	// mexPrintf("here fourth place");	
  
    // send the processed phi to the output  
    for (i=0; i < N1; i++)
	for (j=0; j < N2; j++)
    for (k=0; k < N3; k++)
    for (h=0; h < N4; h++)
        phiValues[((h*N3+k)*N2+j)*N1+i] = phi[i][j][k][h]; 
        for(i=0; i<N1; i++)
        {
            for(j=0; j<N2; j++)
            {
                for(k=0; k<N3; k++)
                {
                    for(h=0; h<N4; h++)
                    {
                        free(xs[i][j][k][h]);
                    }
                    free(phi[i][j][k]);
                    free(xs[i][j][k]);
                }
                free(phi[i][j]);
                free(xs[i][j]);
            }
            free(phi[i]);
            free(xs[i]);
        }
        free(phi);
        free(xs);
        free(dx);
    
}
