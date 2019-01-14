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
  std::stringstream boolreader(argv[11]);
  std::string s;
  float point_weight(atof(argv[8])),
    scale(atof(argv[9])),
    samples_per_node(atof(argv[10]));
  int depth(atoi(argv[3])),
    min_depth(atoi(argv[4])),
    solver_divide(atoi(argv[5])),
    iso_divide(atoi(argv[6])),
    degree (atoi(argv[7]));
  bool confidence,
    output_polygon,
    manifold;

  boolreader >> std::boolalpha >> confidence;
  boolreader.str(argv[12]);
  boolreader >> std::boolalpha >> output_polygon;
  boolreader.str(argv[13]);
  boolreader >> std::boolalpha >> manifold;

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
