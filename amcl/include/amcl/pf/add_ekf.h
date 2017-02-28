#ifndef ADD_EKF_H_
#define ADD_EKF_H_


#include "pf.h"

//オドメトリ推定値、最尤位置，結果,リセットのフラグ,リセット時の分散
//.cはbool型が使えないためint型を引数にしている
//0:false それ以外:ture
void cal_K_gein(pf_vector_t odom, pf_vector_t highest_weight_pf, pf_vector_t *result_position, int check_reset,pf_vector_t reset_sigma);

class Kalman{
public:
  Kalman();
};

#endif
