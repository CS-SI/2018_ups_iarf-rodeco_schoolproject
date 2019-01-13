#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/reconstruction.h>
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
  std::string s;

  s = i_file_name.str();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  printf("Reading %s\n", s.c_str());
  pcl::io::loadPLYFile(s.c_str(), *rgbCloud);
  printf("Stop reading\n");

  /* Create search tree */
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree2->setInputCloud(rgbCloud);

  /* Initialize objects */
  pcl::OrganizedFastMesh<pcl::PointXYZRGB> organizedFastMesh;
  pcl::PolygonMesh mesh;

  /* Initialize reconstruction parameters */

  organizedFastMesh.setInputCloud(rgbCloud);
  organizedFastMesh.setSearchMethod (tree2);
  printf("%s\n", "Reconstruction ");
  organizedFastMesh.reconstruct(mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), mesh);

  return 0;
}
