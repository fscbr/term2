/*
 * helper_functions.h
 *
 *  Created on: May 19, 2017
 *      Author: frank
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <iostream>


Eigen::VectorXd convertMapToCar(const Eigen::VectorXd &state, const Eigen::VectorXd &pos)
{
	Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd(3,3);
	Eigen::VectorXd new_pos = Eigen::VectorXd(3);
	Eigen::VectorXd car_pos = Eigen::VectorXd(3);

	double car_pos_x = state(0);
	double car_pos_y = state(1);
	double theta = state(2);

	car_pos << car_pos_x,car_pos_y, 0;

	rotation_matrix << cos(theta), sin(theta), 0,
			(-sin(theta)), cos(theta), 0,
			0, 0, 1;

	new_pos = rotation_matrix * (pos - car_pos);

	return new_pos;

}

Eigen::VectorXd convertCarToMap(const Eigen::VectorXd &state, const Eigen::VectorXd &pos)
{
	Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd(3,3);
	Eigen::VectorXd new_pos = Eigen::VectorXd(3);
	Eigen::VectorXd car_pos = Eigen::VectorXd(3);

	double car_pos_x = state(0);
	double car_pos_y = state(1);
	double theta = state(2);

	car_pos << car_pos_x,car_pos_y, 0;

	rotation_matrix << cos(theta), sin(theta), 0,
			(-sin(theta)), cos(theta), 0,
			0, 0, 1;

	new_pos = rotation_matrix.inverse() * pos + car_pos;

	return new_pos;

}


void convertVectorMapToCar(const Eigen::VectorXd state,
		const std::vector<double> map_x,
		const std::vector<double> map_y,
		std::vector<double> &car_x,
		std::vector<double> &car_y,
		bool filterNegX)
{
	Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd(3,3);

	double car_pos_x = state(0);
	double car_pos_y = state(1);
	double theta = state(2);
	Eigen::VectorXd car_pos = Eigen::VectorXd(3);
	car_pos << car_pos_x,car_pos_y, 0;

	rotation_matrix << cos(theta), sin(theta), 0,
			(-sin(theta)), cos(theta), 0,
			0, 0, 1;

	Eigen::VectorXd pos = Eigen::VectorXd(3);
	Eigen::VectorXd new_pos = Eigen::VectorXd(3);

	car_x.clear();
	car_y.clear();

//	std::cout << "rotation_matrix:" <<  rotation_matrix << endl;


	for(unsigned int i = 0; i < map_x.size();i++)
	{
		pos << map_x[i],map_y[i], 0;
		new_pos = rotation_matrix * (pos - car_pos);

//		std::cout << "map:" <<  pos.transpose() << endl;
//		std::cout << "car:" <<  new_pos.transpose() << endl;

		//ignore points behind
		if(filterNegX && new_pos(0) < 0)
			continue;

		car_x.push_back((double)new_pos(0));
		car_y.push_back((double)new_pos(1));
	}


}

void convertVectorCarToMap(const Eigen::VectorXd state,
		const std::vector<double> car_x,
		const std::vector<double> car_y,
		std::vector<double> &map_x,
		std::vector<double> &map_y)
{
	Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd(3,3);
	Eigen::MatrixXd rotation_matrix_inverse = Eigen::MatrixXd(3,3);

	double car_pos_x = state(0);
	double car_pos_y = state(1);
	double theta = state(2);

	Eigen::VectorXd car_pos = Eigen::VectorXd(3);
	car_pos << car_pos_x,car_pos_y, 0;

	rotation_matrix << cos(theta), sin(theta), 0,
			(-sin(theta)), cos(theta), 0,
			0, 0, 1;

	rotation_matrix_inverse = rotation_matrix.inverse();
	Eigen::VectorXd pos = Eigen::VectorXd(3);
	Eigen::VectorXd new_pos = Eigen::VectorXd(3);

	map_x.clear();
	map_y.clear();

//	std::cout << "rotation_matrix:" <<  rotation_matrix << endl;


	for(unsigned int i = 0; i < car_x.size();i++)
	{
		pos << car_x[i],car_y[i], 0;
		new_pos = rotation_matrix_inverse * pos + car_pos;

//		std::cout << "map:" <<  pos.transpose() << endl;
//		std::cout << "car:" <<  new_pos.transpose() << endl;

		map_x.push_back((double)new_pos(0));
		map_y.push_back((double)new_pos(1));
	}


}

Eigen::Map<Eigen::VectorXd> toVectorXd(std::vector<double> &vec)
{
	double* ptr = &vec.data()[0];
	Eigen::Map<Eigen::VectorXd> v(ptr, vec.size());
	return v;
}

void log_vector(const std::string name, std::vector<double> &vec)
{
	double* vptr = &vec.data()[0];
	Eigen::Map<Eigen::VectorXd> evec(vptr, vec.size());

	std::cout << name << ":" << evec.transpose() << endl;

}

double poly_eval_psi(Eigen::VectorXd &coeffs, double y0)
{
	return atan(coeffs[1]+ 2*coeffs[2]*y0);// + 3*coeffs[3]*y0*y0);
}

#endif /* HELPER_FUNCTIONS_H_ */
