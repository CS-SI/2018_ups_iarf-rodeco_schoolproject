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
  float point_weight(atof(argv[10])),
    scale(atof(argv[11])),
    samples_per_node(atof(argv[12]));
  int n_estimation(atoi(argv[4])),
    depth(atoi(argv[5])),
    min_depth(atoi(argv[6])),
    solver_divide(atoi(argv[7])),
    iso_divide(atoi(argv[8])),
    degree (atoi(argv[9]));
  bool ktree,
    confidence,
    output_polygon,
    manifold;

  boolreader >> std::boolalpha >> ktree;
  boolreader.str(argv[13]);
  boolreader >> std::boolalpha >> confidence;
  boolreader.str(argv[14]);
  boolreader >> std::boolalpha >> output_polygon;
  boolreader.str(argv[15]);
  boolreader >> std::boolalpha >> manifold;

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

  /* Create search tree */
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud(rgbCloudwithNormals);

  /* Initialize objects */
  pcl::Poisson<pcl::PointXYZRGBNormal> poissonreconstruction;
  pcl::PolygonMesh mesh;

  poissonreconstruction.setDepth(depth);
  poissonreconstruction.setMinDepth(min_depth);
  poissonreconstruction.setPointWeight(point_weight);
  poissonreconstruction.setScale(scale);
  poissonreconstruction.setSolverDivide(solver_divide);
  poissonreconstruction.setIsoDivide(iso_divide);
  poissonreconstruction.setSamplesPerNode(samples_per_node);
  poissonreconstruction.setConfidence(confidence);
  poissonreconstruction.setOutputPolygons(output_polygon);
  poissonreconstruction.setDegree(degree);
  poissonreconstruction.setManifold(manifold);

  poissonreconstruction.setInputCloud(rgbCloudwithNormals);
  poissonreconstruction.setSearchMethod (tree2);
  printf("%s\n", "Reconstruction ");
  poissonreconstruction.reconstruct(mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), mesh);

  return 0;
}
