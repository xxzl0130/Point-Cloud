#pragma once
#define _CRT_SECURE_NO_WARNINGS
typedef unsigned long long pop_t;
#pragma warning(disable: 4996) 
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d.h>

#include <vtkWin32OutputWindow.h>