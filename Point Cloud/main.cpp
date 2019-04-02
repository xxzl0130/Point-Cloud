#include "stdafx.h"

int main(int argc, char** argv)
{
    double t0 = clock();
    vtkObject::GlobalWarningDisplayOff();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dst(new pcl::PointCloud <pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("2.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points\n"
        << "Width:  " << cloud->width << std::endl
        << "Height: " << cloud->height<< std::endl;

    // create normal
    double t1 = clock();
    std::cout << "Time cost:" << (t1 - t0) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Calculating norm information. ";
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchSurface(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);
    double t2 = clock();
    std::cout << "Time cost:" << (t2 - t1) / CLOCKS_PER_SEC << std::endl;

    std::cout << "Calculating cylinder feature." ;
    std::vector<int> inliers;
    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr
        modelC(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(cloud));
    modelC->setInputNormals(cloud_normals);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelC);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();
    ransac.getInliers(inliers);
    pcl::copyPointCloud(*cloud, inliers, *dst);
    double t3 = clock();
    std::cout << "Time cost:" << (t3 - t2) / CLOCKS_PER_SEC << std::endl;

    std::cout << "Creating viewer.";
    auto viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(dst, "sample cloud",0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    double t4 = clock();
    std::cout << "Time cost:" << (t4 - t3) / CLOCKS_PER_SEC << std::endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}