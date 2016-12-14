#include "qpsolver.hpp"

VectorXd NormalOptimization(MatrixXd &N, MatrixXd &M, VectorXd &qdot, VectorXd &fi){
    real_1d_array x; // the array to store results
    minqpstate state;
    minqpreport rep;
    int numofconstrain= N.cols();
    minqpcreate(numofconstrain, state);
    real_2d_array a;
    MatrixXd a_matrix = 2*N.transpose()*M.inverse()*N;
    a= Eigen_2_2d_array(a_matrix);    
    minqpsetquadraticterm(state, a);
    real_1d_array b;
    VectorXd Z = -M*qdot-fi; 
    VectorXd b_vector = -2*N.transpose()*((M.inverse()).transpose())*Z;
    b= Eigen_2_1d_array(b_vector);
    minqpsetlinearterm(state, b);
    MatrixXd cmatrix_matrix(numofconstrain, numofconstrain+1);
    cmatrix_matrix.setZero();
    cmatrix_matrix.block(0,0,numofconstrain,numofconstrain)= MatrixXd::Identity(numofconstrain,numofconstrain);
    real_2d_array cmatrix= Eigen_2_2d_array(cmatrix_matrix);
    VectorXi ct_vector(numofconstrain);
    VectorXd s_vector(numofconstrain);
    ae_int_t temp =1;
    for(int i=0;i<numofconstrain;i++){
        ct_vector(i)=temp;
        s_vector(i)=1.0;
    }
    real_1d_array s;
    integer_1d_array ct;
    s = Eigen_2_1d_array(s_vector);
    ct = Eigen_2_1d_integer_array(ct_vector);
    minqpsetlc(state, cmatrix, ct);
    minqpsetscale(state, s);

    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    minqpoptimize(state);
    minqpresults(state, x, rep);

    VectorXd ResultsX(numofconstrain);
    ResultsX.setZero();
    for(int i=0;i<numofconstrain;i++){
        ResultsX(i)= x[i];
    }
    return ResultsX;
}


VectorXd FrictionOptimization(double mu, MatrixXd &N, MatrixXd &D, MatrixXd &M, 
                            VectorXd &qdot, VectorXd &alpha){
    // create solver, set quadratic/linear terms
    // take four directions of the Tk spanned space
    real_1d_array x; // the array to store results
    minqpstate state;
    minqpreport rep;
    int numofconstrain= N.cols();
    minqpcreate(4*numofconstrain, state);
    real_2d_array a;
    MatrixXd a_matrix = 2*D.transpose()*M.inverse()*D;

    a= Eigen_2_2d_array(a_matrix);
    minqpsetquadraticterm(state, a);
    real_1d_array b;
    VectorXd Z = -M*qdot-N*alpha; 
    VectorXd b_vector = -2*D.transpose()*((M.inverse()).transpose())*Z;
    b= Eigen_2_1d_array(b_vector);
    minqpsetlinearterm(state, b);

    MatrixXd cmatrix_linear(numofconstrain, 4*numofconstrain+1);  //Fix the hardcode later
    cmatrix_linear.setZero();
    for (int i=0; i<numofconstrain; i++){
        VectorXd constant_one(4);
        constant_one << 1.0,1.0,1.0,1.0;
        cmatrix_linear.block(i,i*4,1,4)= constant_one.transpose();
        cmatrix_linear(i,4*numofconstrain)= mu*alpha(i);
    }
    VectorXi ct_vector(numofconstrain);
    VectorXd s_vector(4*numofconstrain);
    ae_int_t temp =1;
    for(int i=0;i<numofconstrain;i++){ 
        ct_vector(i)=-1*temp;
    }
    for(int j=0;j<4*numofconstrain;j++){
        s_vector(j)=1.0;
    }
    real_1d_array s;
    integer_1d_array ct;
    s = Eigen_2_1d_array(s_vector);
    ct = Eigen_2_1d_integer_array(ct_vector);
    real_2d_array cmatrix = Eigen_2_2d_array(cmatrix_linear);
    minqpsetlc(state, cmatrix, ct);

    real_1d_array bndl;
    real_1d_array bndu;
    real_1d_array x0;// initial condition of boundary value problem
    VectorXd bndl_vector(4*numofconstrain);
    VectorXd bndu_vector(4*numofconstrain);
    VectorXd x0_vector(4*numofconstrain);
    bndl_vector.setZero();
    bndu_vector.setZero();
    x0_vector.setZero();
    for(int i=0;i<numofconstrain*4;i++){
        bndu_vector(i)= 650000.0;
        bndl_vector(i)=0.0;
        x0_vector(i)=i;
    }
    x0= Eigen_2_1d_array(x0_vector);
    bndl=Eigen_2_1d_array(bndl_vector);
    bndu=Eigen_2_1d_array(bndu_vector);
    
    minqpsetstartingpoint(state, x0);
    minqpsetbc(state, bndl, bndu);
    
    minqpsetscale(state, s);
    // solve problem with BLEIC-based QP solver
    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    //minqpsetalgoquickqp(state, 0.0, 0.0, 0.0, 0, true);
    minqpoptimize(state);
    minqpresults(state, x, rep);
    
    VectorXd ResultsX(numofconstrain*4);
    ResultsX.setZero();
    for(int i=0;i<numofconstrain*4;i++){
        ResultsX(i)= x[i];
    } 
    return ResultsX;
}

real_2d_array Eigen_2_2d_array(MatrixXd &M){
    int numofrow = M.rows();// num of rows of inertia matrix
    int numofcol = M.cols();// num of cols of inertia matrix
    double _r1[numofrow*numofcol]={0}; // create a double array;
    for(int i=0; i<numofrow; i++){
        for(int j=0; j<numofcol; j++){
            _r1[i*numofcol+j]= M(i,j);
        }
    }
    real_2d_array a;
    a.setcontent(numofrow, numofcol, _r1);
    return a;
}

real_1d_array Eigen_2_1d_array(VectorXd &V){
    real_1d_array b;
    int num_length= V.size(); 
    double _r2[num_length]={0};
    for(int i=0; i<num_length; i++){
        _r2[i]=V(i);
    }
    b.setcontent(num_length, _r2);
    return b;
}

integer_1d_array Eigen_2_1d_integer_array(VectorXi &V){
    integer_1d_array b;
    int num_length= V.size(); 
    ae_int_t _r2[num_length]={0};
    for(int i=0; i<num_length; i++){
        _r2[i]=V(i);
    }
    b.setcontent(num_length, _r2);
    return b;
}
