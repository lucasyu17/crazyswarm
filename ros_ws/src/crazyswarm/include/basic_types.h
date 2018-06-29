#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H
#pragma anon_unions
#include <stdint.h>
#include <stdbool.h>
//#include "Eigen/Eigen/Eigen"
#include <tf_conversions/tf_eigen.h>
//#include "Eigen/Eigen/Geometry"
//using namespace Eigen;

struct Att {
    Eigen::Matrix3f R;
    Eigen::Vector4f Q;
    Eigen::Vector3f Euler;
  } ;


struct M_est_Vecs{
		Eigen::Vector3f m_pos_est, m_cfImuAcc, m_att_est;//, m_vel_est, m_att_est, m_lpos, m_latt;//, m_cfImuAcc_wd, m_cfImuVel, m_cfImuVel_wd, m_cfImuPos, m_cfImuPos_wd;
		Eigen::Matrix3f m_R_est;
		//rotation_t R_est;
};

struct M_sp_Vecs{
		Eigen::Vector4f v_posctrl_posSp;
		Eigen::Vector3f v_posctrl_velFF;
		Eigen::Vector3f v_posctrl_acc_sp;
		//Eigen::Vector3f v_vel_sp, v_acc_sp, v_att_sp, v_posctrl_velFF;
		//Eigen::Vector3f v_lposSp, v_lvelSp;
	};
struct M_recording
	{
		Eigen::Vector3f Rec_posEst;
		Eigen::Vector3f Rec_velEst;
		Eigen::Vector3f Rec_attEst;
		Eigen::Vector3f Rec_posSp;
		Eigen::Vector3f Rec_velSp;
		Eigen::Vector3f Rec_accSp;
		float Rec_yaw_sp;
		float Rec_roll_sp;
		float Rec_pitch_sp;
	};
	

#endif
