#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>

int main (int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  /* Declaration of reconstruction parameters */
  std::stringstream i_file_name(argv[1]),
    o_file_name(argv[2]);
  std::stringstream boolreader(argv[9]);
  std::string s;
  double mu(atof(argv[3])),
    search_radius(atof(argv[5])),
    maximum_angle(atof(argv[6])),
    maximum_surface_angle(atof(argv[7])),
    minimum_angle(atof(argv[8]));
  int maximum_nearest_neighbors(atoi(argv[4]));
  bool normal_consistency,
    consistent_vertex_ordering;

  boolreader >> std::boolalpha >> normal_consistency;
  boolreader.str(argv[10]);
  boolreader >> std::boolalpha >> consistent_vertex_ordering;

  // Load input file into a PointCloud<T> with an appropriate type
  s = i_file_name.str();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbCloudwithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  printf("Reading %s\n", s.c_str());
  pcl::io::loadPLYFile(s.c_str(), *rgbCloudwithNormals);
  printf("Stop reading\n");
  //* the data should be available in rgbCloudwithNormals

  /* Create search tree */
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud(rgbCloudwithNormals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh mesh;

  // Set values for the parameters
  gp3.setMu(mu);
  gp3.setMaximumNearestNeighbors(maximum_nearest_neighbors);
  gp3.setSearchRadius(search_radius);
  gp3.setMaximumAngle(maximum_angle);
  gp3.setMaximumSurfaceAngle(maximum_surface_angle);
  gp3.setMinimumAngle(minimum_angle);
  gp3.setNormalConsistency(normal_consistency);
  gp3.setConsistentVertexOrdering(consistent_vertex_ordering);

  // Get result
  gp3.setInputCloud(rgbCloudwithNormals);
  gp3.setSearchMethod(tree2);
  printf("%s\n", "Reconstruction ");
  gp3.reconstruct(mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary(s.c_str(), mesh);

  // Finish
  return (0);
}
