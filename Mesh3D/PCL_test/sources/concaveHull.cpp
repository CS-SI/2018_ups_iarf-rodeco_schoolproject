#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
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
  std::stringstream boolreader(argv[4]);
  std::string s;
  double alpha(atoi(argv[3]));
  bool keep_information;
  boolreader.str(argv[6]);
  boolreader >> std::boolalpha >> keep_information;

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
  pcl::ConcaveHull<pcl::PointXYZRGBNormal> concaveHull;
  pcl::PolygonMesh mesh;

  concaveHull.setAlpha(alpha);
  concaveHull.setVoronoiCenters(rgbCloudwithNormals);
  concaveHull.setKeepInformation(keep_information);

  concaveHull.setInputCloud(rgbCloudwithNormals);
  concaveHull.setSearchMethod (tree2);
  printf("%s\n", "Reconstruction ");
  concaveHull.reconstruct(mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), mesh);

  return 0;
}
