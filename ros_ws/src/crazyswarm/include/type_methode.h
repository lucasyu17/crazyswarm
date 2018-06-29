#ifndef TYPE_METHODS_H
#define TYPE_METHODS_H
#include <math.h>
#include "basic_types.h"
//#include "Eigen/Eigen/Eigen"
//#include "Eigen/Eigen/Geometry"
//using namespace Eigen;

#define _USE_MATH_DEFINES //PI

void body2earth(const Eigen::Matrix3f* R, const Eigen::Vector3f* body, Eigen::Vector3f* earth, short dimension);

void earth2body(const Eigen::Matrix3f* R, const Eigen::Vector3f* earth, Eigen::Vector3f* body, short dimension);//body=inv(R)*earth

void rotation2euler(const Eigen::Matrix3f* R, Eigen::Vector3f* Euler);

void euler2rotation(const Eigen::Vector3f* Euler, Eigen::Matrix3f* R);

float data_2_angle(float x, float y, float z);	 //in rad

void quaternion2rotation(const Eigen::Vector4f* Q, Eigen::Matrix3f* R);

void rotation2quaternion(const Eigen::Matrix3f* R, Eigen::Vector4f* Q);

void euler2quaternion(const Eigen::Vector3f* Euler, Eigen::Vector4f* Q);

void vec3f_norm(const Eigen::Vector3f* a, float* anwser);

//void quaternion_derivative(const Eigen::Vector4f* Q, Eigen::Vector4f* derQ, const Eigen::Vector3f* w);

float deriv_f(const	float f_now, const float f_past , const float dt);

void quaternion_normalize(Eigen::Vector4f* Q);


void vec3f_normalize(Eigen::Vector3f* v);

void vec3f_passnorm(const Eigen::Vector3f* v, Eigen::Vector3f* vec_des);

float vec3f_length(const Eigen::Vector3f* v);

float vec3f_dot(const Eigen::Vector3f* a, const Eigen::Vector3f* b);

void vec3f_cross(const Eigen::Vector3f* a, const Eigen::Vector3f* b, Eigen::Vector3f* d);

void vec3f_integration(Eigen::Vector3f* Integrated, Eigen::Vector3f* Origin, float dt);

void vec3f_derivative(Eigen::Vector3f* Deriv, Eigen::Vector3f* Origin, Eigen::Vector3f* l_Origin, float dt);

void vec3f_angle(Eigen::Vector3f* a, Eigen::Vector3f* b, float* angleInRad);
//void add_A2B(const Eigen::Vector3f* a, Eigen::Vector3f*b);

float degToRad(float deg);

float radToDeg(float deg);

void writeData_bin(const char* fname, Eigen::Vector3f* vec);

void writeData_binf(const char* fname, float number);

bool IsUAVForm(Eigen::Vector3f* a, Eigen::Vector3f*b);

Eigen::Vector3f CreateVectorFromTwoPts(Eigen::Vector3f* a, Eigen::Vector3f* b);

void number_times_vec3f(float* a, Eigen::Vector3f* v);

Eigen::Vector3f vec3f_minus(Eigen::Vector3f* a, Eigen::Vector3f* b);

float dist_two_pts(Eigen::Vector3f* a, Eigen::Vector3f* b);

float m_power_f(float a, int b);

Eigen::Vector3f vec3f_shift(Eigen::Vector3f* origin, Eigen::Vector3f* delta);


#endif