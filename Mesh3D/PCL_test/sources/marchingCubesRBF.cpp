#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

int main(int argc, char const *argv[]) {

  /* Declaration of reconstruction parameters */
  std::stringstream i_file_name(argv[1]),
    o_file_name(argv[2]);
  std::stringstream boolreader(argv[3]);
  std::string s;
  float iso_level(atoi(argv[5])),
    percentage_extend_grid(atoi(argv[9])),
    distance_ignore(atoi(argv[10]));
  int n_estimation(atoi(argv[4])),
    res_x(atoi(argv[6])),
    res_y(atoi(argv[7])),
    res_z(atoi(argv[8]));
  bool ktree;

  boolreader >> std::boolalpha >> ktree;

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
  pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal> marchingCubesRBF;
  pcl::PolygonMesh mesh;

  //MarchingCubesRBF parameters
  marchingCubesRBF.setIsoLevel(iso_level);
  marchingCubesRBF.setGridResolution(res_x, res_y, res_z);
  marchingCubesRBF.setPercentageExtendGrid(percentage_extend_grid);

  // Get result
  marchingCubesRBF.setInputCloud(rgbCloudwithNormals);
  marchingCubesRBF.setSearchMethod(tree2);
  printf("%s\n", "Reconstruction ");
  marchingCubesRBF.reconstruct(mesh);

  s = o_file_name.str();
  printf("Ecriture dans %s\n", s.c_str());
  pcl::io::savePLYFileBinary (s.c_str(), mesh);

  // Finish
  return (0);
}
