#include <iostream>
#include <fstream>
#include <stdint.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree.h>
#include <cstdlib>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/console/parse.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/make_shared.hpp>
#include <pcl/PCLPointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>

bool savePointCloud (pcl::PCLPointCloud2::Ptr input, std::string output_file) {
    pcl::PolygonMesh mesh;
    mesh.cloud = *input;
    PCL_INFO ("Saving file %s as ASCII.\n", output_file.c_str ());
    if (pcl::io::saveOBJFile (output_file, mesh) != 0)
      return (false);
    return true;
}

void toPCD(){
  pcl::PointCloud<pcl::PointXYZRGB> cloud3;
  pcl::PointXYZRGB p;

  int num_points;
  int r, g, b;

  ifstream infile;
  ofstream outfile;

  infile.open("sarcofago_xyz.xyz");
  num_points = 0;

  if(!infile)
  {
          // Opening a file encounters a problem
          cerr << "Error: file could not be opened!" << endl;
          exit(1);
  }
  else
  {
      // Reading file
      while(!infile.eof())
      {
          infile >> p.x;
          infile >> p.y;
          infile >> p.z;
          infile >> r;
          infile >> g;
          infile >> b;
          p.r = (uint8_t)r;
          p.g = (uint8_t)g;
          p.b = (uint8_t)b;

          cloud3.points.push_back(p);
      }

      cloud3.width = cloud3.points.size();
      cloud3.height = 1;
      cloud3.points.resize(cloud3.width*cloud3.height);

      infile.close();
      pcl::io::savePCDFileASCII("sarcofago_cloud.pcd", cloud3);

  }

    std::cout << "End Convertion\n";

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> showPCD (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main (int argc, char** argv) {

  // INICIALIZACAO
  std::cout << "Inicializacao..." << '\n';
  float xxx = -36.21;
  float yyy = -50.82;
  float zzz = -57.43;
  float radius = 100;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  // CARREGANDO PCD
  std::cout << "Carregando PCD" << '\n';
  pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud);

  // SEGMENTANDO
  std::cout << "Sementando" << '\n';
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);

  pcl::PointXYZRGB searchPoint(1,2,3);
  searchPoint.x = xxx;
  searchPoint.y = yyy;
  searchPoint.z = zzz;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
          cloud_cluster->points.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
  }

  cloud = cloud_cluster;

  // VISUALIZACAO
  std::cout << "VISUALIZACAO" << '\n';
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = showPCD(cloud);
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
  savePointCloud (cloud2, argv[1]);
}
