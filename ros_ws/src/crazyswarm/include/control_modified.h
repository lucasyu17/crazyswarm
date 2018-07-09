#include <math.h>
#include "type_methode.h"
//#include <tf_conversions/tf_eigen.h>
#include "commons.h"
#include "pid.h"

//using namespace Eigen;
class Controller
{
public:
	Controller()
	: loop_record(0)
	, m_group_index(0)
	, m_resFnameRoot("/home/walt/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/vehicle0/")
		,m_pidX(160.0, 0.5, 2.0, 2.5, 0.0, -1e6, 1e6, -0.3, 0.3) //kp, kd, ki, kpp, ff, minOutput, maxOutput, integratorMin, integratorMax;
		,m_pidY(160.0, 0.5, 2.0, 2.5, 0.0, -1e6, 1e6, -0.3, 0.3)//kp 22 kd 1.8 ki 2.0 kpp 7
		,m_pidZ(170.0, 1.65, 5.0, 5.0, 0.0, -1e6, 1e6, -2, 2)//kpp 3
		, l_yawSp(0.0f)
		, yaw_deriv(0.0f)
		, initAcc_IIR(true)
		, initGyro_IIR(true)
	{

	 vel_estIMU.setZero();
	 vel_Sp.setZero();
	 vel_estVicon.setZero();
	 acc_IMU_wd.setZero();
	 _acc_Sp_W.setZero();
	 _acc_Sp_B.setZero();
	 //lacc_est_IMU.setZero();

	 _Zb_des.setZero();
	 _Xc_des.setZero();
	 _Xb_des.setZero();
	 _Yb_des.setZero();
	 _R_des.setZero();
	 e_R.setZero();

	 pos_Sp.setZero();
	 pos_estIMU.setZero();
	 R_est.setZero();
	 l_possp.setZero();
	 l_velsp.setZero();
	 acc_deriv.setZero();
	 l_acc_Sp.setZero();

	 acc_Sp_net.setZero();

	 RPY_sp.setZero();
	 RPY_des.setZero();
	 m_pidX.reset();
	 m_pidY.reset();
	 m_pidZ.reset();
	 
	 printf("hello!! control.h\n");
	};

	const float w_Vicon = 1.0f;
	const float w_IMU = 0.0f;
	const float wv_Vicon = 1;
	const float wv_IMU = 0;
	const float m_cutoff_freq = 2.0f; //cutoff f
    const float m_sample_freq = 300.0f; //sampling f
    const float m_fc_gyro = 0.5f;
    const int num_redording = 4096;
    const float max_thrust = 0.5827*1.3;
    float Pitch_Sp;
	float Roll_Sp;
	char m_resFnameRoot[150];
	int loop_record;
	int m_group_index;
protected:
	//att:
	Eigen::Vector3f  e_R;
	Eigen::Vector3f _Zb_des, _Xc_des, _Yb_des, _Xb_des, Thrust_des;//, F_des
	std::vector<Eigen::MatrixXf> PID_v;
	Eigen::Vector3f RPY_sp, RPY_des;
	Eigen::Matrix3f _R_des, R_est;

	Eigen::Vector3f vel_Sp, _acc_Sp_W, _acc_Sp_B, vel_estVicon, vel_estIMU, acc_IMU_wd, acc_deriv, l_acc_Sp, acc_Sp_net, h_omega;

	float l_yawSp, yaw_deriv;

	Eigen::Vector3f l_posVicon, pos_Sp, l_possp, pos_estIMU, l_velsp;//, lacc_est_IMU
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	ros::Publisher m_pos_sppub, m_posEstPub, m_attSpPub;
	bool initAcc_IIR, initGyro_IIR;

	void resetposController(Eigen::Vector3f* pos_est_Vicon)
	{
		l_posVicon = *pos_est_Vicon;
		l_possp = *pos_est_Vicon;
		pos_estIMU = *pos_est_Vicon; //init of IMU position esti
	}

public:
    void control_nonLineaire(const Eigen::Vector3f& pos_est_Vicon, Eigen::Vector4f& Sp, Eigen::Vector3f& Vel_ff,
                                  Eigen::Vector3f& acc_Sp, Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output);

};