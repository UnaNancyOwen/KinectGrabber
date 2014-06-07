// Sample.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "kinect_grabber.h"
#include <pcl/visualization/cloud_viewer.h>


int _tmain(int argc, _TCHAR* argv[])
{
	pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );

	boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> function =
		[&viewer]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud ) {
		if( !viewer.wasStopped() ) {
			viewer.showCloud( cloud );
		}
	};

	pcl::Grabber* grabber = new pcl::KinectGrabber();
	grabber->registerCallback( function );
	grabber->start();

	while( !viewer.wasStopped() ) {
		if( GetKeyState( VK_ESCAPE ) < 0 ){
			break;
		}
	}

	grabber->stop();

	return 0;
}

