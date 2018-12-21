#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

int main(int argc, char const *argv[]) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  printf("Reading %s\n", argv[1]);
  pcl::io::loadPLYFile(argv[1], *rgbCloud);
  printf("Stop reading\n");

  /* Normal estimation */
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(rgbCloud);
  normalEstimation.setInputCloud(rgbCloud);
  normalEstimation.setSearchMethod(tree);
  //normalEstimation.setViewPoint(0.0,0.0,60000.0);
  normalEstimation.setKSearch(80);
  printf("%s\n", "Estimation des normales");
  normalEstimation.compute(*normals);

  /* Concatenation of XYZRGB with normals */
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbCloudwithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*rgbCloud, *normals, *rgbCloudwithNormals);

  /* Create search tree */
  //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGB>);
  //tree2->setInputCloud(rgbCloudwithNormals);

  /* Initialize objects */
  pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal> marchingCubesRBF;
  std::vector<pcl::Vertices> polygons;
  pcl::PolygonMesh mesh;

  marchingCubesRBF.setInputCloud(rgbCloudwithNormals);
  marchingCubesRBF.voxelizeData ();
  marchingCubesRBF.setOffSurfaceDisplacement (10.0);
  printf("%s\n", "Reconstruction ");
  marchingCubesRBF.reconstruct(mesh);

  pcl::io::savePLYFileBinary ("mesh.ply", mesh);

  return 0;
}
