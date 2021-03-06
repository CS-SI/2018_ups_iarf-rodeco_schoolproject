#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <sstream>

int main(int argc, char const *argv[]) {

  /* Declaration of reconstruction parameters */
  std::stringstream i_file_name(argv[1]),
    o_file_name(argv[2]);
  std::stringstream boolreader(argv[3]);
  std::string s;
  int n_estimation(atoi(argv[4]));
  bool ktree;

  boolreader >> std::boolalpha >> ktree;

  s = i_file_name.str();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  printf("Reading %s\n", s.c_str());
  pcl::io::loadPLYFile(s.c_str(), *rgbCloud);
  printf("Stop reading\n");

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

  /* Concatenation of XYZRGB with normals */
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbCloudwithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*rgbCloud, *normals, *rgbCloudwithNormals);
  //* cloud_with_normals = rgbCloud + normals

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), *rgbCloudwithNormals);

  return 0;
}
