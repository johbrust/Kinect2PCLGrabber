// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace mtec
{
	enum Streams { All, Depth, RGBDepth, RGBADepth, IRDepth, DepthFace, HDFace };

	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZRGBA;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease(Interface *& IRelease)
	{
		if (IRelease != NULL){
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class Kinect2Grabber : public pcl::Grabber
	{
	public:
		Kinect2Grabber(Streams = All);
		virtual ~Kinect2Grabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

		typedef void (slotKinect2PointXYZ)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (slotKinect2PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
		typedef void (slotKinect2PointXYZRGBA)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);
		typedef void (slotKinect2PointXYZI)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);
		typedef void (slotKinect2PointXYZFaceNormal)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&, const pcl::PointCloud < pcl::PointNormal >::Ptr&, const bool);

	protected:
		boost::signals2::signal<slotKinect2PointXYZ>* m_signalPointXYZ;
		boost::signals2::signal<slotKinect2PointXYZRGB>* m_signalPointXYZRGB;
		boost::signals2::signal<slotKinect2PointXYZRGBA>* m_signalPointXYZRGBA;
		boost::signals2::signal<slotKinect2PointXYZI>* m_signalPointXYZI;
		boost::signals2::signal<slotKinect2PointXYZFaceNormal>* m_signalPointXYZFaceNormal;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZI>::Ptr convertIRDepthToPointXYZI(UINT16* infraredBuffer, UINT16* depthBuffer);
		//pcl::PointNormal getFacePointNormal();

		boost::thread m_thread;
		mutable boost::mutex m_mutex;

		void threadFunction();

		Streams m_selectedStream;

		bool m_quit;
		bool m_running;

		HRESULT m_result;
		IKinectSensor* m_sensor;
		ICoordinateMapper* m_mapper;
		IMultiSourceFrameReader* m_multiSourceReader;

		// Depth Face
		IFaceFrameSource*					m_pFaceFrameSources[BODY_COUNT];
		IFaceFrameReader*					m_pFaceFrameReaders[BODY_COUNT];

		// High Definition Face
		IHighDefinitionFaceFrameSource*		m_pHDFaceFrameSources[BODY_COUNT];
		IHighDefinitionFaceFrameReader*		m_pHDFaceFrameReaders[BODY_COUNT];
		IFaceModel*							m_pFaceModel[BODY_COUNT];
		IFaceModelBuilder*					m_pFaceModelBuilder[BODY_COUNT];
		IFaceAlignment*						m_pFaceAlignment[BODY_COUNT];
		bool								produce[BODY_COUNT];
		float deformations[BODY_COUNT][FaceShapeDeformations::FaceShapeDeformations_Count];
		UINT32 nVertices;

		int m_colorWidth;
		int m_colorHeight;
		std::vector<RGBQUAD> m_colorBuffer;

		int m_depthWidth;
		int m_depthHeight;
		std::vector<UINT16> m_depthBuffer;

		bool m_faceFound;
		pcl::PointCloud < pcl::PointNormal >::Ptr m_facePointNormalPtr;
		bool m_HDFaceFound;
		pcl::PointCloud < pcl::PointNormal >::Ptr m_HDFacePointNormalPtr;

		int m_infraredWidth;
		int m_infraredHeight;
		std::vector<UINT16> m_infraredBuffer;
	};


}

#endif KINECT2_GRABBER
