#define _CRT_SECURE_NO_WARNINGS
typedef unsigned long long pop_t;
#pragma warning(disable: 4996) 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
using namespace std;
int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    char const *filename;
    string filenames;
    if(argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        cout << "Please input filename:";
        cin >> filenames;
        filename = filenames.c_str();
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }
    pcl::PCDWriter writer;
    if(writer.writeBinaryCompressed("binary.pcd",cloud) < 0)
    {
        {
            PCL_ERROR("Couldn't write file\n");
            return (-1);
        }
    }
}