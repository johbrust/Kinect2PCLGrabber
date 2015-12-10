//

/* // Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif */

#include "Kinect2Grabber.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <boost/thread/thread.hpp>
void help(const std::string &path)
{
	std::cout << path << " [options]" << std::endl
		<< "  mode: 'Depth', 'RGBDepth', 'RGBADepth', or 'IRDepth'" << std::endl;
}

int main(int argc, char* argv[])
{
	boost::mutex mutexVis;
	mtec::Streams selStream;
	if (argc == 1)
	{
		help(argv[0]);
		std::cout << "Using 'Depth' as default..." << std::endl;
		selStream = mtec::Depth;
		//return 0;
	}
	for (size_t i = 1; i < (size_t)argc; ++i)
	{
		std::string param(argv[i]);

		if (param == "-h" || param == "--help" || param == "-?" || param == "--?")
		{
			help(argv[0]);
			return 0;
		}
		else if (param == "Depth")
		{
			selStream = mtec::Depth;
		}
		else if (param == "RGBDepth")
		{
			selStream = mtec::RGBDepth;
		}
		else if (param == "RGBADepth")
		{
			selStream = mtec::RGBADepth;
		}
		//else if (param == "IRDepth")
		//{
		//	selStream = mtec::IRDepth;
		//}
		else if (param == "DepthFace")
		{
			selStream = mtec::DepthFace;
		}
		else if (param == "HDFace")
		{
			selStream = mtec::HDFace;
		}
		else
		{
			selStream = mtec::Depth;
		}
	}

	// Create Cloud Viewer
	//pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );
	pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	const std::string cloudName = "rendered";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->resize(1);
	cloud->at(0).x = 0; cloud->at(0).y = 0; cloud->at(0).z = 0;
	visualizer->addPointCloud(cloud, cloudName);

	const std::string faceNormName = "faceNorm";
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudFace(new pcl::PointCloud<pcl::PointNormal>());
	cloudFace->resize(1);
	cloudFace->at(0).x = 1; cloudFace->at(0).y = 1; cloudFace->at(0).z = 1;
	cloudFace->at(0).normal_x = 0.577350269189626; cloudFace->at(0).normal_y = 0.577350269189626; cloudFace->at(0).normal_z = 0.577350269189626;
	//visualizer->addPointCloudNormals<pcl::PointNormal>(cloudFace, 1, 0.5, faceNormName);
	pcl::PointXYZ p1(1, 1, 1);
	pcl::PointXYZ p2(2, 2, 2);
	visualizer->addArrow(p1, p2, 1.0, 0, 0, faceNormName);

	visualizer->setBackgroundColor(0, 0, 0);
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
	visualizer->initCameraParameters();
	visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
	visualizer->addCoordinateSystem(1.0);

	//visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

	bool first = true;
	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> functionXYZ =
		// lambda function capturing 'viewer' by reference
		[&visualizer, &cloudName](const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
		if (!visualizer->wasStopped()){
			std::cout << "Cloud size: " << cloud->size() << std::endl;
			visualizer->updatePointCloud(cloud, cloudName);
		}
	};

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> functionXYZRGB =
		// lambda function capturing 'viewer' by reference
		[&visualizer, &cloudName](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		if (!visualizer->wasStopped()){
			visualizer->updatePointCloud(cloud, cloudName);
		}
	};

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> functionXYZRGBA =
		// lambda function capturing 'viewer' by reference
		[&visualizer, &cloudName](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){
		if (!visualizer->wasStopped()){
			visualizer->updatePointCloud(cloud, cloudName);
		}
	};

	//// Callback Function to be called when Updating Data
	//boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> functionXYZI =
	//	// lambda function capturing 'viewer' by reference
	//	[&visualizer, &cloudName](const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
	//	if (!visualizer->wasStopped()){
	//		visualizer->updatePointCloud(cloud, cloudName);
	//	}
	//};

	bool save = true;

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&, const pcl::PointCloud < pcl::PointNormal >::Ptr&, const bool)> functionXYZFaceNormal =
		// lambda function capturing 'viewer' by reference
		[&visualizer, &cloudName, &faceNormName, &mutexVis](const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const pcl::PointCloud < pcl::PointNormal >::Ptr& pfn, const bool faceFound){
		boost::mutex::scoped_lock lock(mutexVis);
		if (!visualizer->wasStopped() && faceFound){
			visualizer->updatePointCloud(cloud, cloudName);
			//viewer.showCloud(cloud);
			//visualizer->removePointCloud(faceNormName);
			//visualizer->addPointCloudNormals<pcl::PointNormal>(pfn, 1, 0.5, faceNormName);
			visualizer->removeAllShapes();
			pcl::PointXYZ p1(pfn->at(0).x, pfn->at(0).y, pfn->at(0).z);
			pcl::PointXYZ p2(p1.x + 2 * (pfn->at(0).normal_x), p1.y + 2 * (pfn->at(0).normal_y), p1.z + 2 * (pfn->at(0).normal_z));
			visualizer->addArrow(p1, p2, 1.0, 0, 0, faceNormName);
			//if (save && faceFound)  {
			//	pcl::PCDWriter writer;
			//	writer.writeBinary("cloud.pcd", *cloud);
			//	writer.writeASCII("cloudFaceNorm.pcd", *pfn);
			//	//save = false;
			//}

		}
	};

	// Create Kinect2Grabber
	pcl::Grabber* grabber = new mtec::Kinect2Grabber(selStream);

	// Register Callback Function
	switch (selStream) {
	case mtec::RGBDepth:
		grabber->registerCallback(functionXYZRGB);
		break;
	case mtec::RGBADepth:
		grabber->registerCallback(functionXYZRGBA);
		break;
		//case mtec::IRDepth:
		//	grabber->registerCallback(functionXYZI);
		//	break;
	case mtec::HDFace:
	case mtec::DepthFace:
		grabber->registerCallback(functionXYZFaceNormal);
		break;
	default:
		grabber->registerCallback(functionXYZ);
	}

	// Start Retrieve Data
	grabber->start();

	while (!visualizer->wasStopped()){
		if (mutexVis.try_lock()){
			// Input Key ( Exit ESC key )
			if (GetKeyState(VK_ESCAPE) < 0){
				break;
			}
			visualizer->spinOnce(20);
			//boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
			mutexVis.unlock();
		}
	}

	// Stop Retrieve Data
	grabber->stop();

	visualizer->close();

	return 0;
}

