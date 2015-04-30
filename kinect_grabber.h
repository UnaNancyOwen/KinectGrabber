// KinectGrabber is pcl::Grabber to retrieve the point cloud data from Kinect v1 using Kinect for Windows SDK v1.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT_GRABBER
#define KINECT_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <NuiApi.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	template <typename T> class pcl::PointCloud;

	class KinectGrabber : public pcl::Grabber
	{
		public:
			KinectGrabber( const int index = 0 );
			virtual ~KinectGrabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

			typedef void ( signal_Kinect_PointXYZ )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
			typedef void ( signal_Kinect_PointXYZRGB )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );

		protected:
			boost::signals2::signal<signal_Kinect_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect_PointXYZRGB>* signal_PointXYZRGB;

			pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( NUI_LOCKED_RECT* depthLockedRect );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB( NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect );

			boost::thread thread;
			mutable boost::mutex mutex;

			void threadFunction();

			bool quit;
			bool running;

			HRESULT result;
			INuiSensor* sensor;
			INuiCoordinateMapper* mapper;
			HANDLE colorHandle;
			HANDLE depthHandle;

			int width;
			int height;
	};

	pcl::KinectGrabber::KinectGrabber( const int index )
		: sensor( nullptr )
		, mapper( nullptr )
		, result( S_OK )
		, colorHandle( INVALID_HANDLE_VALUE )
		, depthHandle( INVALID_HANDLE_VALUE )
		, width( 640 )
		, height( 480 )
		, running( false )
		, quit( false )
		, signal_PointXYZ( nullptr )
		, signal_PointXYZRGB( nullptr )
	{
		// Retrieved Sensor Count that is Connected to PC 
		int count = 0;
		result = NuiGetSensorCount( &count );
		if( FAILED( result ) ){
			throw std::exception( "Exception : NuiGetSensorCount" );
		}

		if( count > index ){
			// Create Sensor Instance
			result = NuiCreateSensorByIndex( index, &sensor );
			if( FAILED( result ) ){
				throw std::exception( "Exception : NuiCreateSensorByIndex" );
			}

			// Initialize Sensor
			result = sensor->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX );
			if( FAILED( result ) ){
				throw std::exception( "Exception : INuiSensor::NuiInitialize" );
			}

			// Retrieved Coordinate Mapper
			result = sensor->NuiGetCoordinateMapper( &mapper );
			if( FAILED( result ) ){
				throw std::exception( "Exception : INuiSensor::NuiGetCoordinateMapper" );
			}
		}
		else{
			throw std::exception( "Exception : Failed to Find a Kinect Sensor" );
		}

		// Retrieved Image Size from Stream Resolution
		unsigned long refWidth = 0;
		unsigned long refHeight = 0;
		NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, refWidth, refHeight );
		width = static_cast<int>( refWidth );
		height = static_cast<int>( refHeight );

		signal_PointXYZ = createSignal<signal_Kinect_PointXYZ>();
		signal_PointXYZRGB = createSignal<signal_Kinect_PointXYZRGB>();
	}

	pcl::KinectGrabber::~KinectGrabber() throw( )
	{
		stop();

		disconnect_all_slots<signal_Kinect_PointXYZ>();
		disconnect_all_slots<signal_Kinect_PointXYZRGB>();

		// End Processing
		sensor->NuiShutdown();
		mapper->Release();

		thread.join();
	}

	void pcl::KinectGrabber::start()
	{
		//  Open Color Stream
		result = sensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &colorHandle );
		if( FAILED( result ) ){
			throw std::exception( "Exception : INuiSensor::NuiImageStreamOpen( Color )" );
		}

		// Open Depth Stream
		result = sensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &depthHandle );
		if( FAILED( result ) ){
			throw std::exception( "Exception : INuiSensor::NuiImageStreamOpen( Depth )" );
		}

		running = true;

		thread = boost::thread( &KinectGrabber::threadFunction, this );
	}

	void pcl::KinectGrabber::stop()
	{
		boost::unique_lock<boost::mutex> lock( mutex );
		
		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::KinectGrabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock( mutex );

		return running;

		lock.unlock();
	}

	std::string pcl::KinectGrabber::getName() const{
		return std::string( "KinectGrabber" );
	}

	float pcl::KinectGrabber::getFramesPerSecond() const {
		return 30.0f;
	}

	void pcl::KinectGrabber::threadFunction()
	{
		while( !quit ){
			boost::unique_lock<boost::mutex> lock( mutex );

			// Retrieved Color Data from Kinect
			NUI_IMAGE_FRAME colorImageFrame = { 0 };
			result = sensor->NuiImageStreamGetNextFrame( colorHandle, INFINITE, &colorImageFrame );
			if( FAILED( result ) ){
				throw std::exception( "Exception : INuiSensor::NuiImageStreamGetNextFrame( Color )" );
			}

			INuiFrameTexture* colorFrameTexture = colorImageFrame.pFrameTexture;
			NUI_LOCKED_RECT colorLockedRect;
			colorFrameTexture->LockRect( 0, &colorLockedRect, nullptr, 0 );

			colorFrameTexture->UnlockRect( 0 );
			sensor->NuiImageStreamReleaseFrame( colorHandle, &colorImageFrame );

			// Retrieved Depth Data from Kinect
			NUI_IMAGE_FRAME depthImageFrame = { 0 };
			result = sensor->NuiImageStreamGetNextFrame( depthHandle, INFINITE, &depthImageFrame );
			if( FAILED( result ) ){
				throw std::exception( "Exception : INuiSensor::NuiImageStreamGetNextFrame( Depth )" );
			}

			BOOL nearMode = false;
			INuiFrameTexture* depthFrameTexture = nullptr;
			result = sensor->NuiImageFrameGetDepthImagePixelFrameTexture( depthHandle, &depthImageFrame, &nearMode, &depthFrameTexture );
			if( FAILED( result ) ){
				throw std::exception( "Exception : INuiSensor::NuiImageFrameGetDepthImagePixelFrameTexture" );
			}
			NUI_LOCKED_RECT depthLockedRect;
			depthFrameTexture->LockRect( 0, &depthLockedRect, nullptr, 0 );

			depthFrameTexture->UnlockRect( 0 );
			sensor->NuiImageStreamReleaseFrame( depthHandle, &depthImageFrame );

			lock.unlock();

			if( signal_PointXYZ->num_slots() > 0 ) {
				signal_PointXYZ->operator()( convertDepthToPointXYZ( &depthLockedRect ) );
			}

			if( signal_PointXYZRGB->num_slots() > 0 ) {
				signal_PointXYZRGB->operator()( convertRGBDepthToPointXYZRGB( &colorLockedRect, &depthLockedRect ) );
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::KinectGrabber::convertDepthToPointXYZ( NUI_LOCKED_RECT* depthLockedRect )
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

		cloud->width = static_cast<uint32_t>( width );
		cloud->height = static_cast<uint32_t>( height );
		cloud->is_dense = false;

		cloud->points.resize( cloud->height * cloud->width );

		NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>( depthLockedRect->pBits );
		pcl::PointXYZ* pt = &cloud->points[0];
		for( int y = 0; y < height; y++ ){
			for( int x = 0; x < width; x++, pt++ ){
				pcl::PointXYZ point;

				NUI_DEPTH_IMAGE_POINT depthPoint;
				depthPoint.x = x;
				depthPoint.y = y;
				depthPoint.depth = depthPixel[y * width + x].depth;

				// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
				Vector4 skeletonPoint;
				mapper->MapDepthPointToSkeletonPoint( NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint );

				point.x = skeletonPoint.x;
				point.y = skeletonPoint.y;
				point.z = skeletonPoint.z;

				*pt = point;
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::KinectGrabber::convertRGBDepthToPointXYZRGB( NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );

		cloud->width = static_cast<uint32_t>( width );
		cloud->height = static_cast<uint32_t>( height );
		cloud->is_dense = false;

		cloud->points.resize( cloud->height * cloud->width );

		NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>( depthLockedRect->pBits );
		pcl::PointXYZRGB* pt = &cloud->points[0];
		for( int y = 0; y < height; y++ ){
			for( int x = 0; x < width; x++, pt++ ){
				pcl::PointXYZRGB point;

				NUI_DEPTH_IMAGE_POINT depthPoint;
				depthPoint.x = x;
				depthPoint.y = y;
				depthPoint.depth = depthPixel[y * width + x].depth;

				// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
				Vector4 skeletonPoint;
				mapper->MapDepthPointToSkeletonPoint( NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint );

				point.x = skeletonPoint.x;
				point.y = skeletonPoint.y;
				point.z = skeletonPoint.z;

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				NUI_COLOR_IMAGE_POINT colorPoint;
				mapper->MapDepthPointToColorPoint( NUI_IMAGE_RESOLUTION_640x480, &depthPoint, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &colorPoint );

				if( 0 <= colorPoint.x && colorPoint.x < width && 0 <= colorPoint.y && colorPoint.y < height ){
					unsigned int index = colorPoint.y * colorLockedRect->Pitch + colorPoint.x * 4;
					point.b = colorLockedRect->pBits[index + 0];
					point.g = colorLockedRect->pBits[index + 1];
					point.r = colorLockedRect->pBits[index + 2];
				}

				*pt = point;
			}
		}

		return cloud;
	}
}

#endif KINECT_GRABBER

