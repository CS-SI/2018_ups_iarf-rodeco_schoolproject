#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;

int main(int argc , char ** argv)
{
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	ofstream ptsFile;
        ptsFile.open ("converted.pts");
	if(io::loadPLYFile<PointXYZ> (argv[1], *cloud) == -1)
	{
		std::cout << "ERROR: couldn't find file" << std::endl;
		return (1);
	}
	for (auto i: *cloud)
	 {
		ptsFile << i.x - cloud->points[0].x<< " " << i.y -cloud->points[0].y<< " " << i.z -cloud->points[0].z<< endl;
	}
	ptsFile.close();
	return 0;
}
