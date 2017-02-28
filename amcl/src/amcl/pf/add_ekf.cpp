

#include "../../../include/amcl/pf/add_ekf.h"
#include "../../../include/amcl/pf/pf.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

Eigen::MatrixXd bel_sigma(3,3);
Eigen::MatrixXd odom_sigma(3,3);
Eigen::MatrixXd yakobi(3,3);
Eigen::MatrixXd K(3,3);

Eigen::MatrixXd I(3,3);


double odom_x_error;
double odom_y_error;
double odom_theta_error;
//信念の分散
double bel_x_error;
double bel_y_error;
double bel_theta_error;

int reset_flag;

Kalman::Kalman(){
  odom_x_error = 0.4;
  odom_y_error = 0.4;
  odom_theta_error = 1.5;

  bel_x_error = 0.3;
  bel_y_error = 0.3;
  bel_theta_error = 0.3;

  I <<  1,0,0,
        0,1,0,
        0,0,1;

  reset_flag = 0;

  bel_sigma <<  bel_x_error * bel_x_error,0,0,
                0,bel_y_error * bel_y_error,0,
                0,0,1;
  odom_sigma << odom_x_error * odom_x_error,0,0,
                0,odom_y_error * odom_y_error,0,
                0,0,odom_theta_error * odom_theta_error;

}


void cal_K_gein(pf_vector_t odom, pf_vector_t highest_weight_pf, pf_vector_t *result_position, int check_reset, pf_vector_t reset_sigma){
  Eigen::MatrixXd kalman_gyouretu(3,3);
  if(check_reset){
    //リセットしたら，パーティクルの分散をカルマンフィルタの分散として使用
    bel_sigma(0,0) = reset_sigma.v[0] * reset_sigma.v[0];
    bel_sigma(1,1) = reset_sigma.v[1] * reset_sigma.v[1];
    bel_sigma(2,2) = reset_sigma.v[2] * reset_sigma.v[2];
    reset_flag = 0;
  }

  yakobi << 1,0,0,
            0,1,0,
            0,0,1;


  //カルマンゲインの計算
  kalman_gyouretu = yakobi * bel_sigma * yakobi.transpose() + odom_sigma;
  K = bel_sigma*yakobi * kalman_gyouretu.inverse();

  //共分散行列の計算

  result_position->v[0] = (1-K(0,0)) * highest_weight_pf.v[0] + K(0,0) * odom.v[0];
  result_position->v[1] = (1-K(1,1)) * highest_weight_pf.v[1] + K(1,1) * odom.v[1];
  result_position->v[2] = (1-K(1,1)) * highest_weight_pf.v[2] + K(2,2) * odom.v[2];

  bel_sigma = (I - K*yakobi) * bel_sigma;
  //std::cout << result_position->v[0];
}
