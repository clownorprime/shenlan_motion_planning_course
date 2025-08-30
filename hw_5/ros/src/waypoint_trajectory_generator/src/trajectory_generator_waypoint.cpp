#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment
    int derivate_num = 8;                         // the number of dervatives in each segment.

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    ROS_INFO_STREAM("path size: " << Path.rows() << " segment size: " << Time.size());
    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
    auto mappingMatrixGenerator = [p_num1d, p_order, derivate_num](double t) -> Eigen::MatrixXd {
        Eigen::MatrixXd mat = MatrixXd::Zero(derivate_num, p_num1d);
        mat(0, 0) = 1;
        mat(1, 1) = 1;
        mat(2, 2) = 2;
        mat(3, 3) = 6;
        //row5
        for (int i = 0; i < p_num1d; i++)
        {
            mat(4, p_num1d - 1 - i) = pow(t, p_order - i);
        }
        //row6
        for (int i = 0; i < p_num1d; i++)
        {
            mat(5, p_num1d - 1 - i) = (p_order - i) * pow(t, p_order - 1 - i);
        }
        //row7
        for (int i = 0; i < p_num1d; i++)
        {
            mat(6, p_num1d - 1 - i) = (p_order - i) * (p_order - 1 - i) * pow(t, p_order - 2 - i);
        }
        //row8
        for (int i = 0; i < p_num1d; i++)
        {
            mat(7, p_num1d - 1 - i) = (p_order - i) * (p_order -i - 1) * (p_order - i - 2) * pow(t, p_order - 3 - i);
        }
        return mat;
    };
    //generate Mapping matrix M
    MatrixXd M = MatrixXd::Zero(derivate_num * m, p_num1d * m);
    for (int i = 0; i < m; i++)
    {
        Eigen::MatrixXd mat = mappingMatrixGenerator(Time[i]);
        M.block(i * derivate_num, i * p_num1d, mat.rows(), mat.cols()) = mat;
    }
    ROS_INFO_STREAM("get mapping matrix: \n" << M);

    /*   Produce the dereivatives in X, Y and Z axis directly.  */


    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    /*   Generate entire Q matrix */
    auto qMatrixGenerator = [p_order, p_num1d](double t) {
        MatrixXd mat = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = 0; i < mat.rows(); i++)
        {
            for (int j = 0; j < mat.cols(); j++)
            {
                if (i >= 4 && j >= 4)
                {
                    mat(i, j) = pow(t, i + j - 7) * i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) * (j - 3) / (i + j - 7);
                }
            }
        }
        return mat;
    };

    MatrixXd Q = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    for (int i = 0; i < Time.size(); i++)
    {
        MatrixXd q_mat = qMatrixGenerator(Time[i]);
        Q.block(i * p_num1d, i * p_num1d, q_mat.rows(), q_mat.cols()) = q_mat;
    }
    ROS_INFO_STREAM("get Q matrix: \n" << Q);
    //build selection matrix.
    MatrixXd selectionMatrixT = MatrixXd::Zero(p_num1d * m, 4 * m + 4);
    //map df and dp to all derivatives coefficients.
    for (int i = 0; i < 4 * m + 4; i++)
    {
        //first points fixed constrain.
        if (i < 4)
        {
            selectionMatrixT(i, i) = 1;
        }
        //m-1 interpoints position fixed constrain.
        else if (i < 4 + m - 1)
        {
            //first map first segment end position.
            selectionMatrixT((i - 4) * p_num1d + 4, i) = 1;
            //then map second segment begin position.
            selectionMatrixT((i - 3) * p_num1d, i) = 1;
        }
        //last position fixed constrain.
        else if (i < p_num1d + m - 1)
        {
            //i = m + 3, map 8 * m - 4;
            //i = m + 6, map 8 * m - 1; 
            selectionMatrixT(i + 7 * m - 7,  i) = 1;
        }
        //then 3 * (m - 1) smooth free variables.
        else 
        {
            int segment = (i - m - 7) / 3;
            int d_order = (i - m - 7) % 3 + 1;
            //first map first segment end derivatives.
            selectionMatrixT(segment * p_num1d + 4 + d_order, i) = 1;
            //then map second segment begin derivatives;
            selectionMatrixT(segment * p_num1d + p_num1d + d_order, i) = 1;
        }
    }
    MatrixXd  selectionMatrix = selectionMatrixT.transpose();
    ROS_INFO_STREAM("get selectionMatrix: \n" << selectionMatrix);
    // next we get R matrix.
    MatrixXd R = selectionMatrix * ((M.inverse()).transpose()) * Q * (M.inverse()) * selectionMatrixT;
    ROS_INFO_STREAM("get R Matrix: \n" << R);
    MatrixXd Rpp = R.block(m + 7, m + 7, 3 * m - 3, 3 * m - 3);   //m + 7, fixed constrain size.
    MatrixXd Rfp = R.block(0, m + 7, m + 7, 3 * m - 3);
    //then we calculate x, y, z coefficent of the trajectory.
    VectorXd Dfx(m + 7), Dfy(m + 7), Dfz(m + 7);
    //m segment, and the number of the points is m + 1.
    Dfx(0) = Path(0, 0), Dfy(0) = Path(0, 1), Dfz(0) = Path(0, 2);  //Path(0), first waypoint.
    for (int i = 0; i < m; i++) //m - 1 interpoint and last point. 
    {
        Dfx(4 + i) = Path(i + 1, 0);
        Dfy(4 + i) = Path(i + 1, 1);
        Dfz(4 + i) = Path(i + 1, 2);
    }
    VectorXd Dpx = -Rpp.inverse() * Rfp.transpose() * Dfx;
    VectorXd Dpy = -Rpp.inverse() * Rfp.transpose() * Dfy;
    VectorXd Dpz = -Rpp.inverse() * Rfp.transpose() * Dfz;
    VectorXd Dx(4 * m + 4), Dy(4 * m + 4), Dz(4 * m + 4);
    ROS_INFO_STREAM("Rpp: \n" << Rpp);
    ROS_INFO_STREAM("Rpp.inverse: " << Rpp.inverse());
    ROS_INFO_STREAM("Dfx: \n" << Dfx);
    ROS_INFO_STREAM("Dpx: \n" << Dpx);
    Dx << Dfx, Dpx;
    Dy << Dfy, Dpy;
    Dz << Dfz, Dpz;
    Px = M.inverse() * selectionMatrixT * Dx;
    Py = M.inverse() * selectionMatrixT * Dy;
    Pz = M.inverse() * selectionMatrixT * Dz;
    for (int i = 0; i < Px.size(); i++)
    {
        //m, 3 * pnum1d
        int row = i / p_num1d;
        int col = i % p_num1d;
        PolyCoeff(row, col) = Px(i);
        PolyCoeff(row, col + p_num1d) = Py(i);
        PolyCoeff(row, col + 2 * p_num1d) = Pz(i);
    }
    ROS_INFO_STREAM("PolyCoeff: ");
    ROS_INFO_STREAM("Cofficient of x: ");
    for (int i = 0; i < m; i++)
    {
        std::cout << "segment " << i << ":\n";
        for (int j = 0; j < p_num1d; j++)
        {
            std::cout << PolyCoeff(i, j) << " ";
        }
        std::cout << std::endl;
    }
    ROS_INFO_STREAM("Cofficient of y: ");
    for (int i = 0; i < m; i++)
    {
        std::cout << "segment " << i << ":\n";
        for (int j = 0; j < p_num1d; j++)
        {
            std::cout << PolyCoeff(i, p_num1d + j) << " ";
        }
        std::cout << std::endl;
    }
    ROS_INFO_STREAM("Cofficient of z: ");
    for (int i = 0; i < m; i++)
    {
        std::cout << "segment " << i << ":\n";
        for (int j = 0; j < p_num1d; j++)
        {
            std::cout << PolyCoeff(i, 2 * p_num1d + j) << " ";
        }
        std::cout << std::endl;
    }
    return PolyCoeff;
}
