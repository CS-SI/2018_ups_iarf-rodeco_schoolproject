#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
	
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#define CGAL_USE_BASIC_VIEWER 1

#include <fstream>
#include <iostream>	

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_2<K>                            Triangulation;
typedef Triangulation::Point                                Point;

typedef CGAL::Projection_traits_xy_3<K>                     Gt;
typedef CGAL::Delaunay_triangulation_2<Gt>                  Delaunay;
typedef K::Point_3                                          Point_3;

void input_RIO (int &x1,int &x2,int &y1,int &y2);
void initialize_point(cv::Mat img, std::vector<Point_3> &points);
cv::Mat cannyDetector(cv::Mat src,int x1,int x2,int y1,int y2);

int main(int argc, char** argv)
{
  int x1,x2,y1,y2;
  cv::Mat original_image = cv::imread(argv[1]);
  cv::namedWindow( "original image", cv::WINDOW_NORMAL);
  cv::imshow("original image",original_image);
  cv::waitKey(1);
  input_RIO (x1,x2,y1,y2);
  cv::Mat detected_edges = cannyDetector(original_image,x1,x2,y1,y2);
  std::vector<Point_3> point;
  initialize_point(detected_edges, point);
  Delaunay dt(point.begin(), point.end());
  std::cout << dt.number_of_vertices() << std::endl;
  CGAL::draw(dt);
  return 0;
}


void initialize_point(cv::Mat img, std::vector<Point_3> &points)
{
 /* This function initialize the points to be triangulated whithin the RIO
   img : Edge rio image 
   points : vector of points to be triangulated 
 */
  for (int x = 0;x < img.rows; x++)//To loop through all the pixels in RIO 
        {
            for (int y = 0; y < img.cols; y++)
            {
               if(img.at<uint8_t>(x,y) !=0)
			points.push_back(Point_3(x,y,0));
            }
        }
}

cv::Mat cannyDetector(cv::Mat src,int x1,int x2,int y1,int y2)
{
 /* This function calculate the edges using canny filter 
   src : image 
   x1,x2,y1,y2 : points that define the RIO 
 */
  int lowthreshold;
  cv::Mat detected_edges,src_gray;
  if( src.empty() )
  {
    std::cout << "Could not open or find the image!\n" << std::endl; 
  }
  else
  { 
    cv::cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );
    cv::namedWindow( "edges", cv::WINDOW_NORMAL);
    cv::Canny(src_gray(cv::Range(x1,x2) , cv::Range(y1,y2)),detected_edges,20,60,3);
    cv::imshow("edges",detected_edges);
  }
  return detected_edges;
}

void input_RIO (int &x1,int &x2,int &y1,int &y2)
{
  /*  Function that inputs the RIO 
      x1, x2, y1, y2 : the points that define the RIO 
  */
  std::cout << "enter the RIO of the image to be triangulated" << std::endl;
  std::cout << "x1 : ";
  std::cin  >> x1;
  std::cout << "x2 : ";
  std::cin  >> x2;
  std::cout << "y1 : ";
  std::cin  >> y1;
  std::cout << "y2 : ";
  std::cin  >> y2; 
}







