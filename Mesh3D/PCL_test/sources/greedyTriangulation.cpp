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
  /* Declaration of reconstruction parameters */
  std::stringstream i_file_name(argv[1]),
    o_file_name(argv[2]);
  std::stringstream boolreader(argv[3]);
  std::string s;
  double mu(atof(argv[5])),
    search_radius(atof(argv[7])),
    minimum_angle(atof(argv[8])),
    maximum_angle(atof(argv[9])),
    maximum_surface_angle(atof(argv[10]));
  int n_estimation(atoi(argv[4])),
    maximum_nearest_neighbors(atoi(argv[6]));
  bool ktree,
    normal_consistency,
    consistent_vertex_ordering;

  boolreader >> std::boolalpha >> ktree;
  boolreader.str(argv[11]);
  boolreader >> std::boolalpha >> normal_consistency;
  boolreader.str(argv[12]);
  boolreader >> std::boolalpha >> consistent_vertex_ordering;

  // Load input file into a PointCloud<T> with an appropriate type
  s = i_file_name.str();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  printf("Reading %s\n", s.c_str());
  pcl::io::loadPLYFile(s.c_str(), *rgbCloud);
  printf("Stop reading\n");
  //* the data should be available in cloud

  /* Normal estimation */
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normalEstimation.setViewPoint(0.0,0.0,60000.0);
  normalEstimation.setInputCloud(rgbCloud);
  if (ktree) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(rgbCloud);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setKSearch(n_estimation);
  } else {
    //normalEstimation.setInputCloud(rgbCloud);
    normalEstimation.setRadiusSearch(n_estimation);
  }
  printf("%s\n", "Estimation des normales");
  normalEstimation.compute(*normals);
  //* normals should not contain the point normals + surface curvatures

  /* Concatenation of XYZRGB with normals */
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbCloudwithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*rgbCloud, *normals, *rgbCloudwithNormals);
  //* cloud_with_normals = rgbCloud + normals

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
  gp3.reconstruct (mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), mesh);

  // Finish
  return (0);
}
