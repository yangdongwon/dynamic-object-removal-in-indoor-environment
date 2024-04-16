#ifndef __ETT__
#define __ETT__
// Standard libraries
#include<iostream>
#include<vector>
#include<fstream>
#include<string>
#include<cmath>

// Additional libraries
#include "ros/ros.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Parameters
{
public:
    int nxt = 6;
    int ns = 5;
    int nz = 3;
    int dt = 0.1;
    double pi = 3.141592;

    // Covariance matrix of Kinematic states
    MatrixXd Phat = MatrixXd::Zero(nxt,nxt);
    double P1 = 0.1;
    double P2 = 0.1;
    double P3 = 0.1;
    double P4 = 0.1;
    double P5 = 0.1;
    double P6 = 0.1;

    // Covariance matrix of Shape parameters
    MatrixXd Pshape = MatrixXd::Zero(ns,ns);
    double Ps1 = 0.001*pi/180;
    double Ps2 = 0.001*pi/180;
    double Ps3 = 0.005;
    double Ps4 = 0.005;
    double Ps5 = 0.005;

    // Process noise of Kinematic states
    MatrixXd Qt = MatrixXd::Zero(nxt,nxt);
    double Qt1 = 0.01;
    double Qt2 = 0.01;
    double Qt3 = 0.01;
    double Qt4 = 0.05;
    double Qt5 = 0.05;
    double Qt6 = 0.01;

    // Process noise of Shape parameters
    MatrixXd Qs = MatrixXd::Zero(ns,ns);
    double Qs1 = 0.01*pi/180;
    double Qs2 = 0.01*pi/180;
    double Qs3 = 0.0001;
    double Qs4 = 0.0001;
    double Qs5 = 0.005;

    // Measurement noise    
    MatrixXd Re = MatrixXd::Zero(nz,nz);
    double Re1 = 0.000000001;
    double Re2 = 0.000000001;
    double Re3 = 0.000000001;

    // System matrix (Kinematic states)
    MatrixXd At = MatrixXd::Zero(nxt,nxt);

    // System matrix (Shape parameters)
    MatrixXd As = MatrixXd::Zero(ns,ns);

    // System matrix (Measurement to target)
    MatrixXd H = MatrixXd::Zero(nz,nxt);
    double ETJPDA_Gate = 7;
    
    double clambda = 50;
    double cp = 0.3;
    MatrixXd mlambda = {};
    MatrixXd Ch = MatrixXd::Zero(nz,nz);
    void initialize_ETT_param(Parameters &param);
};
void Parameters::initialize_ETT_param(Parameters &param)
{
    // Phat 초기화
    param.Phat(0,0) = pow(param.P1,2);
    param.Phat(1,1) = pow(param.P2,2);
    param.Phat(2,2) = pow(param.P3,2);
    param.Phat(3,3) = pow(param.P4,2);
    param.Phat(4,4) = pow(param.P5,2);
    param.Phat(5,5) = pow(param.P6,2);
    // Pshape 초기화
    param.Pshape(0,0) = pow(param.Ps1,2);
    param.Pshape(1,1) = pow(param.Ps2,2);
    param.Pshape(2,2) = pow(param.Ps3,2);
    param.Pshape(3,3) = pow(param.Ps4,2);
    param.Pshape(4,4) = pow(param.Ps5,2);
    // Qt 초기화
    param.Qt(0,0) = pow(param.Qt1,2);
    param.Qt(1,1) = pow(param.Qt2,2);
    param.Qt(2,2) = pow(param.Qt3,2);
    param.Qt(3,3) = pow(param.Qt4,2);
    param.Qt(4,4) = pow(param.Qt5,2);
    param.Qt(5,5) = pow(param.Qt6,2);
    // Qs 초기화
    param.Qs(0,0) = pow(param.Qs1,2);
    param.Qs(1,1) = pow(param.Qs2,2);
    param.Qs(2,2) = pow(param.Qs3,2);
    param.Qs(3,3) = pow(param.Qs4,2);
    param.Qs(4,4) = pow(param.Qs5,2);
    // Re 초기화
    param.Re(0,0) = pow(param.Re1,2);
    param.Re(1,1) = pow(param.Re2,2);
    param.Re(2,2) = pow(param.Re3,2);
    // System models
    param.At << 1,0,0,0.1,0,0,
                0,1,0,0,0.1,0,
                0,0,1,0,0,0.1,
                0,0,0,1,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1;

    param.As << 1,0,0,0,0,
                0,1,0,0,0,
                0,0,1,0,0,
                0,0,0,1,0,
                0,0,0,0,1;

    param.H << 1,0,0,0,0,0,
               0,1,0,0,0,0,
               0,0,1,0,0,0;    
    // Covariance of multiplicative noise
    param.Ch(0,0) = 0.1;
    param.Ch(1,1) = 0.1;
    param.Ch(2,2) = 0.1;
}
// 
class ETTJPDA
{
public:
    void ETT_predict(vector<MatrixXd> &xhat, vector<MatrixXd> &Phat, vector<MatrixXd> &Shape, vector<MatrixXd> &Pshape, Parameters &param);
    MatrixXd ETJPDA(vector<MatrixXd> xhat, vector<MatrixXd> Phat, vector<MatrixXd> Shape, vector<MatrixXd> Pshape, MatrixXd z, vector<double> mlambda, Parameters &param);
    void ETT_update(vector<MatrixXd> &xhat, vector<MatrixXd> &Phat, vector<MatrixXd> &Shape, vector<MatrixXd> &Pshape, MatrixXd z, MatrixXd beta, Parameters &param);
    // ETJPDA subfunction
    MatrixXd get_pred_meas_cov(MatrixXd Shape, MatrixXd Phat, MatrixXd Pshape, Parameters param);
    MatrixXd marginalAssociationProb(MatrixXd G, vector<double> mlambda, Parameters param);
    // ETT_update subfunction
    vector<int> find_idx(MatrixXd beta_n, double Threshold);
    MatrixXd matching_idx(MatrixXd z, vector<int> matched_ind);
    void measurement_update(MatrixXd H, MatrixXd &r, MatrixXd &p, MatrixXd &Cr, MatrixXd &Cp, MatrixXd y, MatrixXd beta_j, Parameters param);
    tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd> get_auxiliary_variables(MatrixXd Shape, MatrixXd Pshape, MatrixXd Ch);
    MatrixXd RotateMatrix(double alpha, double beta, double gamma);
    MatrixXd JacobianRotate1(double alpha, double beta, double l1, double l2, double l3);
    MatrixXd JacobianRotate2(double alpha, double beta, double l1, double l2, double l3);
    MatrixXd JacobianRotate3(double alpha, double beta, double l1, double l2, double l3);
    MatrixXd kroneckerProduct(MatrixXd A, MatrixXd B);
    MatrixXd Cvt_OneColumn(MatrixXd A);
    vector<double> Gen_mlanbda(vector<MatrixXd> Shape, Parameters param);
};
void ETTJPDA::ETT_predict(vector<MatrixXd> &xhat, vector<MatrixXd> &Phat, vector<MatrixXd> &Shape, vector<MatrixXd> &Pshape, Parameters &param)
{
    int Nt = xhat.size();
    MatrixXd At = param.At;
    MatrixXd As = param.As;
    MatrixXd Qt = param.Qt;
    MatrixXd Qs = param.Qs;
    for(int nid = 0; nid < Nt; nid++)
    {
        xhat[nid] = At*xhat[nid];
        Shape[nid] = As*Shape[nid];
        Phat[nid] = At*Phat[nid]*At.transpose()+Qt;
        Pshape[nid] = As*Pshape[nid]*As.transpose()+Qs;
    }
}
MatrixXd ETTJPDA::ETJPDA(vector<MatrixXd> xhat, vector<MatrixXd> Phat, vector<MatrixXd> Shape, vector<MatrixXd> Pshape, MatrixXd z, vector<double> mlambda, Parameters &param)
{
    int M = z.cols();
    int N = xhat.size();
    MatrixXd G = MatrixXd::Zero(N,M);
    for(int nid = 0; nid < N; nid++)
    {
        MatrixXd Cy = get_pred_meas_cov(Shape[nid], Phat[nid], Pshape[nid], param);
        for(int mid = 0; mid < M; mid++)
        {
            MatrixXd v = z.col(mid) -xhat[nid].block(0,0,3,1);
            MatrixXd d_ = v.transpose()*Cy.inverse()*v;
            double d = d_(0,0);
     
            if(d < param.ETJPDA_Gate)
            {
                double value = 1/(pow(2*param.pi,param.nz/2)*sqrt(Cy.determinant()))*exp(-0.5*d);
                G(nid,mid) = value;
            }
        }
    }
    MatrixXd beta = marginalAssociationProb(G, mlambda, param);
    return beta;
}
void ETTJPDA::ETT_update(vector<MatrixXd> &xhat, vector<MatrixXd> &Phat, vector<MatrixXd> &Shape, vector<MatrixXd> &Pshape, MatrixXd z, MatrixXd beta, Parameters &param)
{
    int N = xhat.size();
    MatrixXd H = param.H;
    // cout << "N?: " << N << endl;
    for(int n = 0; n < N; n++)
    {
        MatrixXd beta_n = beta.row(n);   
        vector<int> matched_ind = find_idx(beta_n,0.0);
        // cout << "Matched: " << matched_ind.size() << endl;
        MatrixXd obs_n = matching_idx(z,matched_ind);
        // cout << "obs_n: " << obs_n.rows() << ", " << obs_n.cols() << ", " << matched_ind.size() << endl;
        MatrixXd beta_n2 = matching_idx(beta_n,matched_ind);
        measurement_update(H,xhat[n],Shape[n],Phat[n],Pshape[n],obs_n,beta_n2,param);
    }
}
vector<int> ETTJPDA::find_idx(MatrixXd beta_n, double Threshold)
{
    vector<int> found_idx = {};
    for(int i = 0; i < beta_n.cols(); i++)
    {
        if(beta_n(0,i) > Threshold)
        {
            found_idx.push_back(i);
        }
    }
    return(found_idx);
}
MatrixXd ETTJPDA::matching_idx(MatrixXd z, vector<int> matched_ind)
{
    int Nz = matched_ind.size();
    MatrixXd obs_n = MatrixXd::Zero(z.rows(), matched_ind.size());
    for(int i = 0; i < Nz; i++)
    {
        obs_n.col(i) = z.col(matched_ind[i]);
        // cout << matched_ind[i] << endl;
    }
    //cout << obs_n.rows() << ", " << obs_n.cols() << endl;
    return(obs_n);
}
void ETTJPDA::measurement_update(MatrixXd H, MatrixXd &r, MatrixXd &p, MatrixXd &Cr, MatrixXd &Cp, MatrixXd y, MatrixXd beta_j, Parameters param)
{
    int nk = y.cols();
    MatrixXd Ch = param.Ch;
    MatrixXd Cv = param.Re;
    for(int i = 0; i < nk; i++)
    {
        MatrixXd CI, CII, M, F, Ftilde;
        tie(CI,CII,M,F,Ftilde) = get_auxiliary_variables(p,Cp,Ch);
        MatrixXd yi = y.col(i);
        MatrixXd pyM = beta_j.col(i);
        double py = pyM(0,0);
        // Calculate moments for the kinematic state update
        MatrixXd yibar = H*r;
        MatrixXd Cry = Cr*H.transpose();
        MatrixXd Cy = H*Cr*H.transpose()+CI+CII+Cv;
        MatrixXd L = Cry*(Cy.inverse());
        // Kinematic update
        r = r + py*L*(yi-yibar);
        // cout << r << endl;
        // cout << "Yi" << yi-yibar << endl;
        // cout << L << endl;
        // cout << CII << endl;
        // cout << CI << endl;
        Cr = Cr - py*L*Cry.transpose();
        Cr = (Cr + Cr.transpose())/2;

        // COnstruct pseudo-measurement for the shape update
        MatrixXd temp = (yi-yibar)*(yi-yibar).transpose();
        MatrixXd temp2 = Cvt_OneColumn(temp);
        MatrixXd Yi = F*temp2;
        MatrixXd Cy2 = Cvt_OneColumn(Cy);
        MatrixXd Yibar = F*Cy2;
        MatrixXd CpY = Cp*M.transpose();
        MatrixXd KronCy = kroneckerProduct(Cy,Cy);
        MatrixXd CY = F*KronCy*(F+Ftilde).transpose();
        // Update Shape paramters
        MatrixXd Lp = CpY*CY.inverse();
        p = p + py*Lp*(Yi-Yibar);
        Cp = Cp - py*Lp*CpY.transpose();
        Cp = (Cp + Cp.transpose())/2;
    }

}
tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd> ETTJPDA::get_auxiliary_variables(MatrixXd Shape, MatrixXd Pshape, MatrixXd Ch)
{
    MatrixXd CI, CII, M, F, Ftilde;
    double alpha = Shape(0,0);
    double beta = Shape(1,0);
    double gamma = 0;
    double l1 = Shape(2,0);
    double l2 = Shape(3,0);
    double l3 = Shape(4,0);

    MatrixXd D = MatrixXd::Zero(3,3);
    D << l1, 0, 0, 0, l2, 0, 0, 0, l3;

    MatrixXd S = RotateMatrix(alpha,beta,gamma)*D;
    MatrixXd S1 = S.row(0);
    MatrixXd S2 = S.row(1);
    MatrixXd S3 = S.row(2);

    // S1에 대한 Jacobian matrix
    MatrixXd J1 = JacobianRotate1(alpha, beta, l1, l2, l3);
    // S2에 대한 Jacobian matrix
    MatrixXd J2 = JacobianRotate2(alpha, beta, l1, l2, l3);
    // S3에 대한 Jacobian matrix
    MatrixXd J3 = JacobianRotate3(alpha, beta, l1, l2, l3);

    CI = S*Ch*S.transpose();
    CII = MatrixXd::Zero(3,3);
    CII(0,0) = (Pshape*J1.transpose()*Ch*J1).trace();
    CII(0,1) = (Pshape*J2.transpose()*Ch*J1).trace();
    CII(0,2) = (Pshape*J3.transpose()*Ch*J1).trace();
    CII(1,0) = (Pshape*J1.transpose()*Ch*J2).trace();
    CII(1,1) = (Pshape*J2.transpose()*Ch*J2).trace();
    CII(1,2) = (Pshape*J3.transpose()*Ch*J2).trace();
    CII(2,0) = (Pshape*J1.transpose()*Ch*J3).trace();
    CII(2,1) = (Pshape*J2.transpose()*Ch*J3).trace();
    CII(2,2) = (Pshape*J3.transpose()*Ch*J3).trace();
    
    M = MatrixXd::Zero(5,5);
    M.row(0) = 2*S1*Ch*J1;
    M.row(1) = 2*S2*Ch*J2;
    M.row(2) = 2*S3*Ch*J3;
    M.row(3) = S1*Ch*J1 + S2*Ch*J2;
    M.row(4) = S2*Ch*J2 + S3*Ch*J3;
    
    F = MatrixXd::Zero(5,9);
    Ftilde = MatrixXd::Zero(5,9);

    F << 1,0,0,0,0,0,0,0,0,
         0,0,0,0,1,0,0,0,0,
         0,0,0,0,0,0,0,0,1,
         0,1,0,0,0,0,0,0,0,
         0,0,0,0,0,1,0,0,0;
    
    Ftilde << 1,0,0,0,0,0,0,0,0,
              0,0,0,0,1,0,0,0,0,
              0,0,0,0,0,0,0,0,1,
              0,0,0,1,0,0,0,0,0,
              0,0,0,0,0,0,0,1,0;
    

    return make_tuple(CI, CII, M, F, Ftilde);
}
MatrixXd ETTJPDA::RotateMatrix(double alpha, double beta, double gamma)
{
    MatrixXd R = MatrixXd::Zero(3,3);
    R(0,0) = cos(alpha)*cos(beta);
    R(0,1) = cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
    R(0,2) = cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*sin(gamma);
    R(1,0) = sin(alpha)*cos(beta);
    R(1,1) = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
    R(1,2) = sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
    R(2,0) = -sin(beta);
    R(2,1) = cos(beta)*sin(gamma);
    R(2,2) = cos(beta)*cos(gamma);
    return(R);
}
MatrixXd ETTJPDA::JacobianRotate1(double alpha, double beta, double l1, double l2, double l3)
{
    MatrixXd J1 = MatrixXd::Zero(3,5);
    double gamma = 0;
    // J11 matrix
    J1(0,0) = -l1*cos(beta)*sin(alpha);
    J1(0,1) = -l1*cos(alpha)*sin(beta);
    J1(0,2) = cos(alpha)*cos(beta);
    J1(0,3) = 0;
    J1(0,4) = 0;
    // J12 matrix
    J1(1,0) = -l2*sin(beta)*sin(gamma)*sin(alpha)-cos(gamma)*cos(alpha);
    J1(1,1) = l2*cos(alpha)*sin(gamma)*cos(beta);
    J1(1,2) = 0;
    J1(1,3) = cos(alpha)*sin(beta)*sin(gamma);
    J1(1,4) = 0;
    // J13 matrix
    J1(2,0) = l3*(-sin(beta)*cos(gamma)*sin(alpha)+sin(gamma)*cos(alpha));
    J1(2,1) = l3*cos(alpha)*cos(gamma)*cos(beta);
    J1(2,2) = 0;
    J1(2,3) = 0;
    J1(2,4) = cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
    return(J1);
}
MatrixXd ETTJPDA::JacobianRotate2(double alpha, double beta, double l1, double l2, double l3)
{
    MatrixXd J2 = MatrixXd::Zero(3,5);
    double gamma = 0;
    // J21 matrix
    J2(0,0) = l1*cos(beta)*cos(alpha);
    J2(0,1) = -l1*sin(alpha)*sin(beta);
    J2(0,2) = sin(alpha)*cos(beta);
    J2(0,3) = 0;
    J2(0,4) = 0;
    // J22 matrix
    J2(1,0) = l2*(sin(alpha)*sin(gamma)*cos(beta)-cos(gamma)*sin(alpha));
    J2(1,1) = l2*sin(alpha)*sin(gamma)*cos(beta);
    J2(1,2) = 0;
    J2(1,3) = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
    J2(1,4) = 0;
    // J23 matrix
    J2(2,0) = l3*(sin(beta)*cos(gamma)*cos(alpha)+sin(gamma)*sin(alpha));
    J2(2,1) = l3*sin(alpha)*cos(gamma)*cos(beta);
    J2(2,2) = 0;
    J2(2,3) = 0;
    J2(2,4) = sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
    return(J2);
}
MatrixXd ETTJPDA::JacobianRotate3(double alpha, double beta, double l1, double l2, double l3)
{
    MatrixXd J3 = MatrixXd::Zero(3,5);
    double gamma = 0;
    // J31 matrix
    J3(0,0) = 0;
    J3(0,1) = -l1*cos(beta);
    J3(0,2) = -sin(beta);
    J3(0,3) = 0;
    J3(0,4) = 0;
    // J32 matrix
    J3(1,0) = 0;
    J3(1,1) = -l2*sin(gamma)*sin(beta);
    J3(1,2) = 0;
    J3(1,3) = cos(beta)*sin(gamma);
    J3(1,4) = 0;
    // J33 matrix
    J3(2,0) = 0;
    J3(2,1) = -l3*sin(beta)*cos(gamma);
    J3(2,2) = 0;
    J3(2,3) = 0;
    J3(2,4) = cos(beta)*cos(gamma);
    return(J3);
}
// ETT Update sub-function
MatrixXd ETTJPDA::get_pred_meas_cov(MatrixXd Shape, MatrixXd Phat, MatrixXd Pshape, Parameters param)
{
    MatrixXd Ch = param.Ch;
    MatrixXd H = param.H;
    MatrixXd R = param.Re;
    double alpha = Shape(0,0);
    double beta = Shape(1,0);
    double gamma = 0;
    double l1 = Shape(2,0);
    double l2 = Shape(3,0);
    double l3 = Shape(4,0);
    
    MatrixXd D = MatrixXd::Zero(3,3); 
    D << l1, 0, 0, 0, l2, 0, 0, 0, l3;

    MatrixXd S = RotateMatrix(alpha,beta,gamma)*D;
    // S1에 대한 Jacobian matrix
    MatrixXd J1 = JacobianRotate1(alpha, beta, l1, l2, l3);
    // S2에 대한 Jacobian matrix
    MatrixXd J2 = JacobianRotate2(alpha, beta, l1, l2, l3);
    // S3에 대한 Jacobian matrix
    MatrixXd J3 = JacobianRotate3(alpha, beta, l1, l2, l3);

    MatrixXd CI = S*Ch*S.transpose();
    MatrixXd CII = MatrixXd::Zero(3,3);
    CII(0,0) = (Pshape*J1.transpose()*Ch*J1).trace();
    CII(0,1) = (Pshape*J2.transpose()*Ch*J1).trace();
    CII(0,2) = (Pshape*J3.transpose()*Ch*J1).trace();
    CII(1,0) = (Pshape*J1.transpose()*Ch*J2).trace();
    CII(1,1) = (Pshape*J2.transpose()*Ch*J2).trace();
    CII(1,2) = (Pshape*J3.transpose()*Ch*J2).trace();
    CII(2,0) = (Pshape*J1.transpose()*Ch*J3).trace();
    CII(2,1) = (Pshape*J2.transpose()*Ch*J3).trace();
    CII(2,2) = (Pshape*J3.transpose()*Ch*J3).trace();

    MatrixXd P = H*Phat*H.transpose() + CI + CII + R;
    return(P);
}
MatrixXd ETTJPDA::marginalAssociationProb(MatrixXd G, vector<double> mlambda, Parameters param)
{
    double cp = param.cp;
    double clambda = param.clambda;
    int N = G.rows();
    int M = G.cols();
    MatrixXd beta = MatrixXd::Zero(N+1,M);
    MatrixXd betaTemp = MatrixXd::Zero(N+1,M);
    vector<double> S = {};
    for(int j = 0; j < M; j++)
    {
        S.push_back(clambda*cp);    
        for(int i = 0; i < N; i++)
        {
            S[j] = S[j] + mlambda[i]*G(i,j);
        }
    }
    for(int j = 0; j < M; j++)
    {
        for(int i = 0; i < N; i++)
        {
            betaTemp(i,j) = mlambda[i]*G(i,j)/S[j];
        }
        betaTemp(N,j) = clambda*cp/S[j];
    }
    for(int j = 0; j < M; j++)
    {
        beta.col(j) = betaTemp.col(j).array()/betaTemp.col(j).sum();
    }
    return(beta);
}
MatrixXd ETTJPDA::kroneckerProduct(MatrixXd A, MatrixXd B)
{
    int n1 = A.rows();    int m1 = A.cols();
    int n2 = B.rows();    int m2 = B.cols();
    MatrixXd C = MatrixXd::Zero(n1*n2,m1*m2);

    for(int i = 0; i < n1; i++)
    {
        for(int j = 0; j < m1; j++)
        {
            C.block(n1*i,m1*j,3,3) = A(i,j)*B;
        }
    }
    return(C);
}
MatrixXd ETTJPDA::Cvt_OneColumn(MatrixXd A)
{
    int n = A.rows();
    int m = A.cols();
    int cnt = 0;
    MatrixXd B = MatrixXd::Zero(n*m,1);
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < m; j++)
        {
            B(cnt,0) = A(i,j);
            cnt = cnt + 1;
        }
    }
    return(B);
}
vector<double> ETTJPDA::Gen_mlanbda(vector<MatrixXd> Shape, Parameters param)
{
    vector<double> mlambda = {};
    int Nt = Shape.size();

    for(int i = 0; i < Nt; i++)
    {
        MatrixXd Shape_i = Shape[i];
        double value = Shape_i(2,0) * Shape_i(3,0) * 100;
        value = 100;
        if(value < 1)
        {
            value = 1;
        }
        mlambda.push_back(value);
    }
    return(mlambda);
}
#endif
