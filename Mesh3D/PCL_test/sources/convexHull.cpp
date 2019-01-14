#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <sstream>

int main(int argc, char const *argv[]) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  /* Declaration of reconstruction parameters */
  std::stringstream i_file_name(argv[1]),
    o_file_name(argv[2]);
  std::stringstream boolreader(argv[3]);
  std::string s;
  bool compute_area_volume;
  boolreader >> std::boolalpha >> compute_area_volume;

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

  /* Initialize objects */
  pcl::ConvexHull<pcl::PointXYZRGBNormal> convexHull;
  pcl::PolygonMesh mesh;

  convexHull.setComputeAreaVolume(compute_area_volume);

  convexHull.setInputCloud(rgbCloudwithNormals);
  convexHull.setSearchMethod (tree2);
  printf("%s\n", "Reconstruction ");
  convexHull.reconstruct(mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), mesh);

  return 0;
}
