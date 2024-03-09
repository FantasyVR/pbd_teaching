#include <igl/opengl/glfw/Viewer.h>
// Inline mesh of a cube
Eigen::MatrixXd positions = (Eigen::MatrixXd(3, 3) <<
	0.0, 0.0, 0.0,
	1.0, 0.0, 0.0,
	2.0, 0.0, 0.0).finished();
const Eigen::MatrixXd colors = (Eigen::MatrixXd(1, 3) <<
	1.0, 0.0, 0.0).finished();
const Eigen::MatrixXi edges = (Eigen::MatrixXi(2, 2) <<
	0, 1,
	1, 2
	).finished();

//质量
Eigen::Vector3d inv_mass(0.0, 1.0, 1.0);


//速度
Eigen::MatrixXd old_positions = positions;
Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(3, 3);

//约束相关
Eigen::Vector2d rest_length;
double h = 0.01;


void init_constriant() {
	for (int i = 0; i < 2; i++)
		rest_length[i] = (positions.row(i + 1) - positions.row(i)).norm();
}

void step(double h) {
	// prediction step
	Eigen::RowVector3d gravity(0.0, -9.8, 0.0);
	old_positions = positions;
	for (int i = 0; i < 3; i++) {
		if (inv_mass[i] == 0.0) continue;
		velocities.row(i) = velocities.row(i) + h * gravity;
		positions.row(i) = positions.row(i) + h * velocities.row(i);
	}



}


// 
bool pre_draw(igl::opengl::glfw::Viewer& viewer) {

	step(h);

	viewer.data().set_points(positions, colors);
	viewer.data().set_edges(positions, edges, colors);

	return false;
}


int main(int argc, char* argv[])
{
	// Plot the mesh
	igl::opengl::glfw::Viewer viewer;
	viewer.callback_pre_draw = &pre_draw;
	viewer.core().is_animating = true;
	viewer.launch();
}
