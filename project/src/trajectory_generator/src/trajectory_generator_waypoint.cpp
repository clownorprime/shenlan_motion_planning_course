#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <osqp/osqp.h>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

//written by doubao AI code tools.
Eigen::VectorXd generateTrajectoryPolyCoeff(
    const Eigen::MatrixXd& H, 
    const Eigen::VectorXd& f, 
    const Eigen::MatrixXd& Aeq, 
    const Eigen::VectorXd& beq)
{
    // （省略输入检查部分，与之前相同）
    const int n_vars = H.rows();
    if (H.cols() != n_vars || f.size() != n_vars) {
        return Eigen::VectorXd();
    }
    const int n_eq_constraints = Aeq.rows();
    if (Aeq.cols() != n_vars || beq.size() != n_eq_constraints) {
        return Eigen::VectorXd();
    }

    // -------------------------- 关键修改：只存储H的上三角部分 --------------------------
    std::vector<c_float> H_csc_data;
    std::vector<c_int>   H_csc_indices;
    std::vector<c_int>   H_csc_indptr;

    H_csc_indptr.push_back(0);
    for (int col = 0; col < n_vars; ++col) {
        // 仅保留上三角元素（row <= col），忽略下三角部分
        for (int row = 0; row <= col; ++row) { 
            if (std::abs(H(row, col)) > 1e-12) {
                H_csc_data.push_back(H(row, col));
                H_csc_indices.push_back(row);
            }
        }
        H_csc_indptr.push_back(H_csc_data.size());
    }

    // （Aeq转换、参数设置等部分与之前相同）
    std::vector<c_float> Aeq_csc_data;
    std::vector<c_int>   Aeq_csc_indices;
    std::vector<c_int>   Aeq_csc_indptr;

    Aeq_csc_indptr.push_back(0);
    for (int col = 0; col < n_vars; ++col) {
        for (int row = 0; row < n_eq_constraints; ++row) {
            if (std::abs(Aeq(row, col)) > 1e-12) {
                Aeq_csc_data.push_back(Aeq(row, col));
                Aeq_csc_indices.push_back(row);
            }
        }
        Aeq_csc_indptr.push_back(Aeq_csc_data.size());
    }

    OSQPWorkspace* work = nullptr;
    OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    if (!settings) return Eigen::VectorXd();
    osqp_set_default_settings(settings);
    settings->verbose = false;
    settings->max_iter = 10000;
    settings->eps_abs = 1e-6;
    settings->eps_rel = 1e-6;

    std::vector<c_float> l(n_eq_constraints), u(n_eq_constraints);
    for (int i = 0; i < n_eq_constraints; ++i) {
        l[i] = beq[i];
        u[i] = beq[i];
    }

    std::vector<c_float> f_c(n_vars);
    for (int i = 0; i < n_vars; ++i) {
        f_c[i] = f[i];
    }

    OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    if (!data) {
        c_free(settings);
        return Eigen::VectorXd();
    }
    data->n = n_vars;
    data->m = n_eq_constraints;
    data->P = csc_matrix(data->n, data->n, H_csc_data.size(),
                        H_csc_data.data(), H_csc_indices.data(), H_csc_indptr.data());
    data->q = f_c.data();
    data->A = csc_matrix(data->m, data->n, Aeq_csc_data.size(),
                        Aeq_csc_data.data(), Aeq_csc_indices.data(), Aeq_csc_indptr.data());
    data->l = l.data();
    data->u = u.data();

    int setup_ret = osqp_setup(&work, data, settings);
    if (setup_ret != 0 || !work) {
        c_free(data);
        c_free(settings);
        return Eigen::VectorXd();
    }

    osqp_solve(work);

    Eigen::VectorXd poly_coeff(n_vars);
    if (work->info && work->info->status_val == OSQP_SOLVED && work->solution) {
        for (int i = 0; i < n_vars; ++i) {
            poly_coeff[i] = work->solution->x[i];
        }
    } else {
        poly_coeff = Eigen::VectorXd();
    }

    osqp_cleanup(work);
    return poly_coeff;
}


TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment
  int p_derivate_num = 2 * (d_order + 1);

  int m = Time.size();
  std::cout << "d_order: " << d_order << " p_num1d: " << p_num1d << " m: " << m << std::endl;
  MatrixXd PolyCoeff(m, 3 * p_num1d);

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  auto generateQMatrix = [p_num1d](double t) -> MatrixXd
  {
    MatrixXd Q = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
    for (int i = 0; i < Q.rows(); i++)
    {
      for (int j = 0; j < Q.cols(); j++)
      {
        if (i >= 4 && j >= 4)
        {
          Q(i, j) = (i * (i - 1) * (i - 2) * (i - 3)) * (j * (j - 1) * (j - 2) * (j - 3)) * pow(t, i + j - 7) / (i + j - 7);
        }
      }
    }
    return Q;
  };

  auto generateMMatrix = [p_num1d, p_derivate_num](double t) -> MatrixXd
  {
    MatrixXd M = Eigen::MatrixXd::Zero(p_derivate_num, p_num1d);
    M(0, 0) = 1;
    M(1, 1) = 1;
    M(2, 2) = 2;
    M(3, 3) = 6;
    for (int i = 0; i < p_num1d; i++)
    {
      M(4, i) = pow(t, i);
    }
    for (int i = 0; i < p_num1d; i++)
    {
      M(5, i) = i * pow(t, i - 1);
    }
    for (int i = 0; i < p_num1d; i++)
    {
      M(6, i) = i * (i - 1) * pow(t, i - 2);
    }
    for (int i = 0; i < p_num1d; i++)
    {
      M(7, i) = i * (i - 1) * (i - 2) * pow(t, i - 3);
    }
    return M;
  };

  MatrixXd Q = Eigen::MatrixXd::Zero(m * p_num1d, m * p_num1d);
  for (int i = 0; i < m; i++)
  {
    MatrixXd Q_i = generateQMatrix(Time[i]);
    Q.block(i * p_num1d, i * p_num1d, Q_i.rows(), Q_i.cols()) = Q_i;
  }

  auto getPolyCoeff = [&](int axis) -> MatrixXd
  {
    MatrixXd polyCoeff_axis = MatrixXd::Zero(m, p_num1d);
    MatrixXd Aeq = MatrixXd::Zero(5 * m + 1, m * p_num1d);
    VectorXd beq = VectorXd::Zero(5 * m + 1);
    MatrixXd M_0 = generateMMatrix(Time[0]);
    //point 0 location, speed, accelerate constrain at axis.
    MatrixXd point0_constrain_Aeq = MatrixXd::Zero(3, p_num1d);
    for (int i = 0; i < 3; i++)
    {
      point0_constrain_Aeq.row(i) = M_0.row(i);
    }
    beq(0) = Path(0, axis);
    beq(1) = Vel(0, axis);
    beq(2) = Acc(0, axis);
    Aeq.block(0, 0, point0_constrain_Aeq.rows(), point0_constrain_Aeq.cols()) = point0_constrain_Aeq;
    //point m - 1 location, vel, accelerate constrain at axis.
    MatrixXd M_last = generateMMatrix(Time[m - 1]);
    MatrixXd last_point_constrain_Aeq = MatrixXd::Zero(3, p_num1d);
    for (int i = 0; i < 3; i++)
    {
      last_point_constrain_Aeq.row(i) = M_last.row(4 + i);
    }
    beq(3) = Path(m, axis);
    beq(4) = Vel(1, axis);
    beq(5) = Acc(1, axis);
    Aeq.block(3, (m - 1) * p_num1d, last_point_constrain_Aeq.rows(), last_point_constrain_Aeq.cols()) = last_point_constrain_Aeq;
    //(2*m - 2) location constrain.
    int start_index = 6;
    for (int i = 0; i < m - 1; i++)
    {
      MatrixXd M_first = generateMMatrix(Time[i]);
      MatrixXd first_Aeq = MatrixXd::Zero(1, p_num1d);
      first_Aeq = M_first.block(4, 0, first_Aeq.rows(), first_Aeq.cols());
      Aeq.block(start_index + i, i * p_num1d, first_Aeq.rows(), first_Aeq.cols()) = first_Aeq;
      beq(start_index + i) = Path(i + 1, axis);
    }
    start_index = m + 5;
    for (int i = 0; i < m - 1; i++)
    {
      MatrixXd M_second = generateMMatrix(Time[i + 1]);
      MatrixXd second_Aeq = MatrixXd::Zero(1, p_num1d);
      second_Aeq = M_second.block(0, 0, second_Aeq.rows(), second_Aeq.cols());
      Aeq.block(start_index + i, (i + 1) * p_num1d, second_Aeq.rows(), second_Aeq.cols()) = second_Aeq;
      beq(start_index + i) = Path(i + 1, axis);
    }

    // (m - 1) * location, speed, accelerate, jerk continuity constrain.
    start_index = 2 * m + 4;
    for (int i = 0; i < m - 1; i++)
    {
      MatrixXd M_first = generateMMatrix(Time[i]);
      MatrixXd M_second = generateMMatrix(Time[i + 1]);
      MatrixXd first_Aeq = MatrixXd::Zero(3, p_num1d);
      MatrixXd second_Aeq = MatrixXd::Zero(3, p_num1d);
      first_Aeq = M_first.block(5, 0, first_Aeq.rows(), first_Aeq.cols());
      second_Aeq = M_second.block(1, 0, second_Aeq.rows(), second_Aeq.cols());
      Aeq.block(start_index + i * 3, i * p_num1d, first_Aeq.rows(), first_Aeq.cols()) = first_Aeq;
      Aeq.block(start_index + i * 3, (i + 1) * p_num1d, second_Aeq.rows(), second_Aeq.cols()) = -second_Aeq;
    }

    VectorXd f = VectorXd::Zero(m * p_num1d);
    VectorXd poly_coeff = generateTrajectoryPolyCoeff(Q, f, Aeq, beq);
    for (int i = 0; i < polyCoeff_axis.rows(); i++)
    {
      for (int j = 0; j < polyCoeff_axis.cols(); j++)
      {
        polyCoeff_axis(i, j) = poly_coeff(i * polyCoeff_axis.cols() + j);
      }
    }
    std::cout << "axis : " << axis << " polycoeff." << std::endl;
    for (int i = 0; i < polyCoeff_axis.rows(); i++)
    {
      for (int j = 0; j < polyCoeff_axis.cols(); j++)
      {
        std::cout << polyCoeff_axis(i, j) << " ";
      }
      std::cout << std::endl;
    }
    return polyCoeff_axis;
  };

  for (int i = 0; i < 3; i++)
  {
    PolyCoeff.block(0, i * p_num1d, m, p_num1d) = getPolyCoeff(i);
  }
  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}