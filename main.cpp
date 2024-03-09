#include <igl/opengl/glfw/Viewer.h>

int main(int argc, char *argv[])
{
  // Inline mesh of a cube
  const Eigen::MatrixXd V= (Eigen::MatrixXd(3,3)<<
    0.0,0.0,0.0,
	1.0,0.0,0.0,
    2.0,0.0,0.0).finished();
  const Eigen::MatrixXd C = (Eigen::MatrixXd(1, 3) <<
      1.0, 0.0, 0.0).finished();
  const Eigen::MatrixXi E = (Eigen::MatrixXi(2, 2) <<
     0, 1,
     1, 2
      ).finished();

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_points(V, C);
  viewer.data().set_edges(V, E, C);
  viewer.launch();
}
