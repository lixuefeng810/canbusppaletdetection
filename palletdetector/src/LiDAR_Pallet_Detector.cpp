#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pcl/common/common.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl/common/transforms.h>
#include <canbusrotor/rotorposservice.h>
#include <canbusrotor/rotorstatus.h>
#include <vector>
#include <algorithm>
#include <pcl/filters/passthrough.h>
#include <palletdetector/Capture3Dservice.h>
#include <visualization_msgs/Marker.h>
#include <string>     // std::string, std::to_string
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/centroid.h>
//#include <boost/thread.hpp>

using namespace std;
float StartSwipAngle = 90; // -100 degrees
float EndSwipAngle = 105; // +100 degrees
float MinCaptureAngle = 92; // from -91 degrees
float MaxCaptureAngle = +100; // to -91 degrees
float HomeAngle = +90; //
int HomeSpeed = 45;
int RotationSpeed = 254;
int RotationStepSize = 7;
float MinSquareLineLenght = 0.001; // Square of the min acceptable line length
float MaxSquareLineLenght = 0.250; // Square of the max acceptable line length
unsigned int MininPointsCount = 0; // Minimum Number of Points in a point cloud segment
float AllowedPointDistance = 0.01; // in meters, For extracting a line out of a point cloud segment. This determines the maximum allowed distance of points from the line we extracted from this point cloud segment
float ToleranceFromLongLine = 0.05; // in meters, For extracting a line out of a point cloud segment. This determines the maximum allowed distance of points from the line we extracted from this point cloud segment
float MinPointDistanceTolerance = 0.05; // in meters; Max distance between consequent points in a segment
float PalletpillarThicknessMax = 0.12; // In meters, max thickness of pillar
float ForkWindowWidth = 0.62; // In meters, Approx pillar to pillar distance
float ForkWindowWidthTolerance = 0.10;
float MidPilarPointFromSidePilarLineDistanceMax = 0.05;
float SensorElevation = 0.30;
float PalletWidthFor3DAnalysis = 1.2; //meters
float PalletWidthToleranceFor3DAnalysis = 0.1; //meters
float PalletAngleToleranceFor3DAnalysis = 10; //Degrees
float PalletCenterToleranceFor3DAnalysis = 0.10; //meters
bool Publish3D = true;
bool PublishRvizPalletLine = true;
ros::Subscriber LASERsub;
ros::Subscriber RotorStatussb;
ros::ServiceClient client;
ros::Publisher pub3D;
ros::Publisher vis_pub;
ros::Publisher Projectedlaserscan;
vector<float> PosVector ;
vector<sensor_msgs::LaserScan> Scans ;
bool NewPosAvailable = false;
float NewPos = 0;
//bool ShouldScan = false;
//rotor::rotor rotorcommand;

enum PalletDetectionState
{
  PDS_ServiceRequested = 0,
  PDS_GoingToHomePosition = 1,
  PDS_GoingStartSweepPosition = 2,
  PDS_Scanning = 3,
  PDS_ScanComplete = 4,
  PDS_Processing = 5,
  PDS_ProcessingComplete = 6,
  PDS_Responded = 7,
};

PalletDetectionState CurrentPalletDetectionState = PDS_Responded;

struct PointCloudSegmentStruct
{
    unsigned int StartIndex;
    unsigned int EndIndex;
};

struct LineSegmentStruct
{
    PointCloudSegmentStruct PointCloudSegment;
    float StartX;
    float StartY;
    float EndX;
    float EndY;
};

struct ConfirmLineStruct
{
    bool IsConfirmed;
    LineSegmentStruct ConfirmedLine;
};


template <typename ForwardIt> ForwardIt Next(ForwardIt it, typename std::iterator_traits<ForwardIt>::difference_type n = 1)
{
    std::advance(it, n);
    return it;
}

template <typename T> std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

/*boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  Eigen::Vector4f center;
  pcl::compute3DCentroid(*cloud, center);
  viewer->setCameraPosition(center[0]-2, center[1] , +1, -1, 0, 1);
  viewer->setSize (640, 240);
  return (viewer);
}

void ViewThread(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(cloud);
  while (ros::ok())
  {
    viewer->spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
}

void ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::thread NewViewerThread(ViewThread,cloud);
  NewViewerThread.join();
}*/

class DetectionClass
{
    public:
    DetectionClass();
    void LASERCallback (const sensor_msgs::LaserScan::ConstPtr&);
    void RotorStatusCallback(const canbusrotor::rotorstatus::ConstPtr&);
    std::list<PointCloudSegmentStruct> GetPointCloudSegments(const sensor_msgs::LaserScan& , float, unsigned int );
    geometry_msgs::Point ConvertToCartesian(int , float , float , float);
    geometry_msgs::Point ProjectPointOnLine(geometry_msgs::Point ,float ,float , geometry_msgs::Point );
    std::list <LineSegmentStruct> GetLineSegments(const sensor_msgs::LaserScan& , float , float ,unsigned int );
    float LineLenght(LineSegmentStruct );
    float DistanceFromPointToLine (geometry_msgs::Point , geometry_msgs::Point , geometry_msgs::Point );
    float PointToPointDistance(geometry_msgs::Point , geometry_msgs::Point );
    std::vector <LineSegmentStruct> DetectPalletFromThreePillarFrontFaceLines(std::list<LineSegmentStruct> );
    pcl::PointCloud<pcl::PointXYZ> ShowOutputCloud (vector<sensor_msgs::LaserScan> );
    void DrawLine(ros::Publisher* , LineSegmentStruct );
    void DrawLines(ros::Publisher* , std::vector <LineSegmentStruct> );
    ConfirmLineStruct Further_3Dto2D_Analyze(LineSegmentStruct, pcl::PointCloud<pcl::PointXYZ> );
    sensor_msgs::LaserScan PointCloud2LASERScan(sensor_msgs::PointCloud2);


    geometry_msgs::Pose2D DetectPallet();
    void SweepAndScan();

    vector <LineSegmentStruct> PalletLines;

 };

DetectionClass::DetectionClass()
{
}

void* RotorServiceCaller (void* Dummy)
{
  float PrevPos = 0;
  PosVector.clear();
  Scans.clear();


  canbusrotor::rotorposservice srv;
    srv.request.SetpointPosDegree = HomeAngle;
    srv.request.Speed = HomeSpeed;
    std::cout << "-------------------------------------------"<<std::endl;
    std::cout << "Going To Home Position ..." << std::endl;
    ROS_INFO(" Calling Rotor service, Pos = [%6.3f] degree, Speed = [%d]." , srv.request.SetpointPosDegree , srv.request.Speed);
    CurrentPalletDetectionState = PDS_GoingToHomePosition;
    if (client.call(srv))
    {
      ROS_INFO("Rotor Service Completed Successfully.");
      std::cout << "In Home Position." << std::endl;
    }
    else
    {
      ROS_ERROR("Failed to call rotor service.");
    }

  srv.request.SetpointPosDegree = StartSwipAngle;
  srv.request.Speed = HomeSpeed;
  std::cout << "-------------------------------------------"<<std::endl;
  std::cout << "Going to Start Sweep Position..." << std::endl;
  ROS_INFO(" Calling Rotor service, Pos = [%6.3f] degree, Speed = [%d]." , srv.request.SetpointPosDegree , srv.request.Speed);
  CurrentPalletDetectionState = PDS_GoingStartSweepPosition;
  if (client.call(srv))
  {
    ROS_INFO("Rotor Service Completed Successfully.");
    std::cout << "Ready to Scan." << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call rotor service.");
  }

  srv.request.SetpointPosDegree = MaxCaptureAngle;
  srv.request.Speed = RotationSpeed;
  std::cout << "-------------------------------------------"<<std::endl;
  std::cout << "Scanning..." << std::endl;
  ROS_INFO(" Calling Rotor service, Pos = [%6.3f] degree, Speed = [%d]." , srv.request.SetpointPosDegree , srv.request.Speed);
  CurrentPalletDetectionState = PDS_Scanning;
  if (client.call(srv))
  {
    ROS_INFO("Rotor Service Completed Successfully.");
  }
  else
  {
    ROS_ERROR("Failed to call rotor service.");
  }

  srv.request.SetpointPosDegree = HomeAngle;
  srv.request.Speed = HomeSpeed;
  std::cout << "-------------------------------------------"<<std::endl;
  std::cout << "Returning To Home Position ..." << std::endl;
  ROS_INFO(" Calling Rotor service, Pos = [%6.3f] degree, Speed = [%d]." , srv.request.SetpointPosDegree , srv.request.Speed);
  CurrentPalletDetectionState = PDS_GoingToHomePosition;
  if (client.call(srv))
  {
    ROS_INFO("Rotor Service Completed Successfully.");
    std::cout << "In Home Position." << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call rotor service.");
  }
  ROS_INFO("Current average resolution is: [%6.3f] degrees.", (*max_element(PosVector.begin(), PosVector.end())- *min_element(PosVector.begin(), PosVector.end()))/PosVector.size());
  CurrentPalletDetectionState = PDS_ScanComplete;

  return Dummy;
}

void DetectionClass::SweepAndScan()
{
  pthread_t RotorServiceThread;
  pthread_create (&RotorServiceThread, NULL, &RotorServiceCaller, (void *)NULL);
  while (ros::ok() && CurrentPalletDetectionState != PDS_ScanComplete)
  {
    usleep (1);
    ros::spinOnce();
  }
  usleep (100); // wait for pthread function to return
}

std::list<PointCloudSegmentStruct> DetectionClass::GetPointCloudSegments(const sensor_msgs::LaserScan& scan, float MinPtDistPoints, unsigned int MinNumberofPoints)
{
	unsigned int StartIndex = 0;
	unsigned int EndIndex = 1;
	unsigned int KeepIndex = 1;
	PointCloudSegmentStruct Segment;
	std::list <PointCloudSegmentStruct> Segments;
	Segment.StartIndex = StartIndex;
	for (size_t Counter = 1; Counter < scan.ranges.size(); Counter++)
	{
		if (scan.ranges[Counter] == 0)
			continue;
		if (fabs(scan.ranges[Counter] - scan.ranges[Counter - 1]) < MinPtDistPoints) //(Tolerance * fmax(scan.ranges[Counter], scan.ranges[Counter - 1])))
			EndIndex = Counter;
		else
		{
			if (KeepIndex - StartIndex > MinNumberofPoints)
			{
				Segment.EndIndex = KeepIndex;
				Segments.push_back(Segment);
			}
			StartIndex = Counter;
			Segment.StartIndex = StartIndex;
		}
		KeepIndex = EndIndex;
	}
	if (KeepIndex - StartIndex > MinNumberofPoints)
	{
		Segment.EndIndex = KeepIndex;
		Segments.push_back(Segment);
	}
	return Segments;
}

geometry_msgs::Point DetectionClass::ConvertToCartesian(int Index, float R, float angle_increment, float angle_min)
{
	geometry_msgs::Point retval;
  double Angle =  Index* angle_increment+angle_min;
	float X =  (R * cos(Angle));
	float Y =  (R * sin(Angle));
	retval.x = X;
	retval.y = Y;
	return retval;
}

geometry_msgs::Point DetectionClass::ProjectPointOnLine(geometry_msgs::Point OutPoint,float m,float d, geometry_msgs::Point APointOnLine)
{
	geometry_msgs::Point retval;
	float X = (OutPoint.x + m * OutPoint.y - m * d) / (m * m + 1);
	float Y = (m * OutPoint.x + m * m * OutPoint.y + d) / (m * m + 1);
	if (isinf(m))
	{
		X = APointOnLine.x;
		Y = OutPoint.y;
	}
	retval.x = X;
	retval.y = Y;
	return retval;
}

std::list <LineSegmentStruct> DetectionClass::GetLineSegments(const sensor_msgs::LaserScan& scan, float MinSqLineLenght, float MaxSqLineLenght,unsigned int MinNumberofPoints)
{
	std::list<PointCloudSegmentStruct> Segments ((std::list<PointCloudSegmentStruct>)GetPointCloudSegments(scan, MinPointDistanceTolerance,MininPointsCount));
	std::list <LineSegmentStruct> Lines;
	std::list<PointCloudSegmentStruct>::iterator SegmentIterator;
	float A = 0, B = 0, C = 0, X = 0, Y = 0;
	float StartX = 0, StartY = 0, EndX = 0, EndY = 0;
	float MaxDistance = 0;
	float Distance = 0;
	int MaxIndex = 0;
	for (size_t SegmentCounter = 0; SegmentCounter < Segments.size(); SegmentCounter++)
	{
		SegmentIterator = Segments.begin();
		for (size_t Counter=0;Counter<SegmentCounter;Counter++)
			SegmentIterator++;
    geometry_msgs::Point StartPoint = ConvertToCartesian(((PointCloudSegmentStruct)(*SegmentIterator)).StartIndex,scan.ranges[((PointCloudSegmentStruct)(*SegmentIterator)).StartIndex],scan.angle_increment, scan.angle_min);
    geometry_msgs::Point EndPoint = ConvertToCartesian(((PointCloudSegmentStruct)(*SegmentIterator)).EndIndex, scan.ranges[((PointCloudSegmentStruct)(*SegmentIterator)).EndIndex],scan.angle_increment, scan.angle_min);
		StartX = StartPoint.x;
		StartY = StartPoint.y;
		EndX = EndPoint.x;
		EndY = EndPoint.y;
		A = EndX - StartX;
		B = StartY - EndY;
		C = (StartX - EndX) * StartY + (EndY - StartY) * StartX;
		MaxDistance = 0;
		MaxIndex = 0;
		for (size_t Counter = ((PointCloudSegmentStruct)(*SegmentIterator)).StartIndex; Counter < ((PointCloudSegmentStruct)(*SegmentIterator)).EndIndex; Counter++)
		{
			if (scan.ranges[Counter] > 0)
			{
        geometry_msgs::Point PointToDet = ConvertToCartesian(Counter, scan.ranges[Counter],scan.angle_increment, scan.angle_min);
				X = PointToDet.x;
				Y = PointToDet.y;
				Distance = fabs(((A * Y + B * X + C) / sqrt(A * A + B * B)));
				if (Distance > MaxDistance)
				{
					MaxDistance = Distance;
					MaxIndex = Counter;
				}
			}
		}

		if (MaxDistance > AllowedPointDistance)
		{
			PointCloudSegmentStruct Segment;
			Segment.StartIndex = ((PointCloudSegmentStruct)(*SegmentIterator)).StartIndex;
			Segment.EndIndex = MaxIndex;
			if (Segment.EndIndex - Segment.StartIndex > MinNumberofPoints)
			{
				SegmentIterator++;
				Segments.insert(SegmentIterator, Segment);
				SegmentIterator--;
				SegmentIterator--;
				Segment.StartIndex = MaxIndex;
				Segment.EndIndex = ((PointCloudSegmentStruct)(*SegmentIterator)).EndIndex;
				if (Segment.EndIndex - Segment.StartIndex > MinNumberofPoints)
				{
					SegmentIterator++;
					SegmentIterator++;
					Segments.insert(SegmentIterator, Segment);
					SegmentIterator--;
					SegmentIterator--;
					SegmentIterator--;
				}
			}
			else
			{
				Segment.StartIndex = MaxIndex;
				Segment.EndIndex = ((PointCloudSegmentStruct)(*SegmentIterator)).EndIndex;
				if (Segment.EndIndex - Segment.StartIndex > MinNumberofPoints)
				{
					SegmentIterator++;
					Segments.insert(SegmentIterator, Segment);
					SegmentIterator--;
					SegmentIterator--;
				}
			}
			Segments.erase(SegmentIterator);
			SegmentCounter--;
		}
		else
		{
			if ((StartX - EndX) * (StartX - EndX) + (StartY - EndY) * (StartY - EndY) > MinSqLineLenght
					&& (StartX - EndX) * (StartX - EndX) + (StartY - EndY) * (StartY - EndY) < MaxSqLineLenght)
			{
				LineSegmentStruct line;
				PointCloudSegmentStruct Segment =(PointCloudSegmentStruct)(*SegmentIterator);
				double Xm = 0;
				double Ym = 0;
				int PointsCount = 0;
				for (size_t Counter = Segment.StartIndex; Counter < Segment.EndIndex; Counter++)
				{
					if (scan.ranges[Counter] > 0)
					{
            geometry_msgs::Point PointToDet = ConvertToCartesian(Counter, scan.ranges[Counter],scan.angle_increment, scan.angle_min);
						Xm += PointToDet.x;
						Ym += PointToDet.y;
						PointsCount++;
					}
				}
				Xm /= PointsCount;
				Ym /= PointsCount;
				double Num = 0;
				double Denum = 0;
				for (size_t Counter = Segment.StartIndex; Counter < Segment.EndIndex; Counter++)
				{
					if (scan.ranges[Counter] > 0)
					{
            geometry_msgs::Point PointToDet = ConvertToCartesian(Counter, scan.ranges[Counter],scan.angle_increment, scan.angle_min);
						Num += ((Xm - PointToDet.x) * (Ym - PointToDet.y));
						Denum += ((Ym - PointToDet.y) * (Ym - PointToDet.y) - (Xm - PointToDet.x) * (Xm - PointToDet.x));
					}
				}
				double LineLenght1 = 0;
				LineSegmentStruct SuspLine1;
				double Phi = M_PI / 4;
				if (Denum != 0)
					Phi = atan(-2 * Num / Denum) / 2;
				double Slope = std::numeric_limits<double>::quiet_NaN();
				if (Phi != 0 && !((Phi <M_PI/2+0.001 && Phi >M_PI/2-0.001) || (Phi >-M_PI/2-0.001 && Phi <-M_PI/2+0.001)))
					Slope = -1 / tan(Phi);
				double R = Xm * cos(Phi) + Ym * sin(Phi);
				double XL = R * cos(Phi);
				double YL = R * sin(Phi);
				if ((Phi <M_PI/2+0.001 && Phi >M_PI/2-0.001) || (Phi >-M_PI/2-0.001 && Phi <-M_PI/2+0.001))
				{
					line.StartX = (float)XL;
					line.StartY = (float)StartY;
					line.EndX = (float)XL;
					line.EndY = (float)EndY;
				}
				else
				{
					geometry_msgs::Point XYPoint;
					XYPoint.x = XL;
					XYPoint.y = YL;
          geometry_msgs::Point StartOutPoint = ConvertToCartesian(Segment.StartIndex, scan.ranges[Segment.StartIndex],scan.angle_increment, scan.angle_min);
          geometry_msgs::Point EndOutPoint = ConvertToCartesian(Segment.EndIndex, scan.ranges[Segment.EndIndex],scan.angle_increment, scan.angle_min);
					geometry_msgs::Point ProjectedStartPoint = ProjectPointOnLine(StartOutPoint, (float)Slope, (float)(YL - Slope * XL),XYPoint);
					geometry_msgs::Point ProjectedEndPoint = ProjectPointOnLine(EndOutPoint, (float)Slope, (float)(YL - Slope * XL), XYPoint);
					line.StartX = ProjectedStartPoint.x;
					line.StartY = ProjectedStartPoint.y;
					line.EndX = ProjectedEndPoint.x;
					line.EndY = ProjectedEndPoint.y;
				}
				LineLenght1 = sqrt((line.StartX - line.EndX) * (line.StartX - line.EndX) + (line.StartY - line.EndY) * (line.StartY - line.EndY));
				SuspLine1 = line;
				double LineLenght2 = 0;
				//                        if (Math.Sqrt((line.StartX - line.EndX) * (line.StartX - line.EndX) + (line.StartY - line.EndY) * (line.StartY - line.EndY)) < 100)
				{
					Phi -= M_PI / 2;
					Slope = std::numeric_limits<double>::quiet_NaN();
					if (Phi != 0 && !((Phi <M_PI/2+0.001 && Phi >M_PI/2-0.001) || (Phi >-M_PI/2-0.001 && Phi <-M_PI/2+0.001)))
						Slope = -1 / tan(Phi);
					R = Xm * cos(Phi) + Ym * sin(Phi);
					XL = R * cos(Phi);
					YL = R * sin(Phi);
					if ((Phi <M_PI/2+0.001 && Phi >M_PI/2-0.001) || (Phi >-M_PI/2-0.001 && Phi <-M_PI/2+0.001))
					{
						line.StartX = (float)XL;
						line.StartY = (float)StartY;
						line.EndX = (float)XL;
						line.EndY = (float)EndY;
					}
					else
					{
						geometry_msgs::Point XYPoint;
						XYPoint.x = XL;
						XYPoint.y = YL;
            geometry_msgs::Point StartOutPoint = ConvertToCartesian(Segment.StartIndex, scan.ranges[Segment.StartIndex],scan.angle_increment, scan.angle_min);
            geometry_msgs::Point EndOutPoint = ConvertToCartesian(Segment.EndIndex, scan.ranges[Segment.EndIndex],scan.angle_increment, scan.angle_min);
						geometry_msgs::Point ProjectedStartPoint = ProjectPointOnLine(StartOutPoint, (float)Slope, (float)(YL - Slope * XL), XYPoint);
						geometry_msgs::Point ProjectedEndPoint = ProjectPointOnLine(EndOutPoint, (float)Slope, (float)(YL - Slope * XL), XYPoint);
						line.StartX = ProjectedStartPoint.x;
						line.StartY = ProjectedStartPoint.y;
						line.EndX = ProjectedEndPoint.x;
						line.EndY = ProjectedEndPoint.y;
					}
					LineLenght2 = sqrt((line.StartX - line.EndX) * (line.StartX - line.EndX) + (line.StartY - line.EndY) * (line.StartY - line.EndY));
				}
				if (LineLenght1 > LineLenght2)
					line = SuspLine1;
				if (line.StartX == line.StartX && line.StartY == line.StartY && line.EndX == line.EndX && line.EndY == line.EndY) // checking nan and inf values
					Lines.push_back(line);
			}
		}
	}
	return Lines;
}

float DetectionClass::LineLenght(LineSegmentStruct inputline)
{
	return sqrt((inputline.StartX-inputline.EndX)*(inputline.StartX-inputline.EndX) + (inputline.StartY-inputline.EndY)*(inputline.StartY-inputline.EndY));
}

float DetectionClass::DistanceFromPointToLine (geometry_msgs::Point ThePoint, geometry_msgs::Point LineP1, geometry_msgs::Point LineP2)
{
	float x0=ThePoint.x, y0=ThePoint.y, x1=LineP1.x, y1=LineP1.y, x2=LineP2.x, y2 = LineP2.y;
	return fabs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1));
}

float DetectionClass::PointToPointDistance(geometry_msgs::Point Point1, geometry_msgs::Point Point2)
{
	return sqrt((Point1.x - Point2.x) * (Point1.x - Point2.x) + (Point1.y - Point2.y) * (Point1.y - Point2.y));
}

std::vector <LineSegmentStruct> DetectionClass::DetectPalletFromThreePillarFrontFaceLines(std::list<LineSegmentStruct> Lines)
{
	LineSegmentStruct Pallet;
  std::vector <LineSegmentStruct> Pallets;

  //std::vector<LineSegmentStruct> v{ std::list::begin(Lines), std::list::end(Lines) };

  std::vector<LineSegmentStruct> v;
  v.reserve(Lines.size());
  v.insert(v.end(),Lines.begin(), Lines.end());

	Pallet.StartX = 0;
	Pallet.StartY = 0;
	Pallet.EndX = 0;
	Pallet.EndY = 0;

	if (Lines.size()<3)
	{
    return Pallets;
	}

  DrawLines(&vis_pub,v);
  ros::spinOnce();
	LineSegmentStruct Line1;
	LineSegmentStruct Line2;
	LineSegmentStruct Line3;
	LineSegmentStruct TempLine;

	geometry_msgs::Point Line1MidPoint;
	geometry_msgs::Point Line2MidPoint;
	geometry_msgs::Point Line3MidPoint;
	geometry_msgs::Point TempLineP1;
	geometry_msgs::Point TempLineP2;

	std::list<LineSegmentStruct> Batch1;
	std::list<LineSegmentStruct> Batch2;
	std::list<LineSegmentStruct> AllIntermediateLines;

//	bool PointSetFound = false;
	bool DefetiveLinesegmentFound = false;

/*  int counter =1;

  for ( std::list <LineSegmentStruct>::iterator it1 = Lines.begin(); it1!=Lines.end(); ++it1)
	{
    std::cout << counter++ << "- ["<<((LineSegmentStruct)(*it1)).StartX << " ," <<((LineSegmentStruct)(*it1)).StartY << "] to [" <<((LineSegmentStruct)(*it1)).EndX << " ,"<<((LineSegmentStruct)(*it1)).EndY << "]." << " Len = [" << LineLenght((LineSegmentStruct)(*it1)) << "]."<< std::endl;
  }*/

	for ( std::list <LineSegmentStruct>::iterator it1 = Lines.begin(); it1!=Lines.end(); ++it1)
	{
    bool PointSetFound = false;
		Line1 = ((LineSegmentStruct)(*it1));
		if (LineLenght(Line1) > PalletpillarThicknessMax)
			continue;
		Line1MidPoint.x = (Line1.StartX + Line1.EndX)/2.0;
		Line1MidPoint.y = (Line1.StartY + Line1.EndY)/2.0;
		AllIntermediateLines.clear();
		Batch1.clear();
		for ( std::list <LineSegmentStruct>::iterator it2 = Next(it1,1); it2!=Lines.end(); ++it2)
		{
			Line2 = ((LineSegmentStruct)(*it2));
			if (LineLenght(Line2) > PalletpillarThicknessMax)
			{
//        ROS_INFO("Rejeted Since [%3.3f] is larger than PalletpillarThicknessMax [%3.3f]", LineLenght(Line2), PalletpillarThicknessMax);
				Batch1.push_back((LineSegmentStruct)(*it2));
				continue;
			}
			Line2MidPoint.x = (Line2.StartX + Line2.EndX)/2.0;
			Line2MidPoint.y = (Line2.StartY + Line2.EndY)/2.0;
			if (fabs(PointToPointDistance(Line2MidPoint, Line1MidPoint) - ForkWindowWidth) > ForkWindowWidthTolerance)
			{
 //       ROS_INFO("Rejeted Since [%3.3f] is larger than ForkWindowWidthTolerance [%3.3f]", PointToPointDistance(Line2MidPoint, Line1MidPoint) - ForkWindowWidth, ForkWindowWidthTolerance);
        Batch1.push_back((LineSegmentStruct)(*it2));
				continue;
			}
			Batch2.clear();
			for ( std::list <LineSegmentStruct>::iterator it3 = Next(it2,1); it3!=Lines.end(); ++it3)
			{
				Line3 = ((LineSegmentStruct)(*it3));
				if (LineLenght(Line3) > PalletpillarThicknessMax)
				{
 //         ROS_INFO("Rejeted Since [%3.3f] is larger than PalletpillarThicknessMax [%3.3f]", LineLenght(Line3), PalletpillarThicknessMax);
          Batch2.push_back((LineSegmentStruct)(*it3));
					continue;
				}
				Line3MidPoint.x = (Line3.StartX + Line3.EndX)/2.0;
				Line3MidPoint.y = (Line3.StartY + Line3.EndY)/2.0;
        if (fabs(PointToPointDistance(Line1MidPoint, Line3MidPoint) - 2*ForkWindowWidth) > ForkWindowWidthTolerance)
				{
 //         ROS_INFO("Rejeted Since [%3.3f] is larger than ForkWindowWidthTolerance [%3.3f]",
 //                  fabs(PointToPointDistance(Line1MidPoint, Line3MidPoint) - (2*ForkWindowWidth))
 //                  , ForkWindowWidthTolerance);
          Batch2.push_back((LineSegmentStruct)(*it3));
					continue;
				}
				if (fabs(PointToPointDistance(Line2MidPoint, Line3MidPoint) - ForkWindowWidth) > ForkWindowWidthTolerance)
				{
 //         ROS_INFO("Rejeted Since [%3.3f] is larger than ForkWindowWidthTolerance [%3.3f]",
  //                 fabs(PointToPointDistance(Line2MidPoint, Line3MidPoint) - ForkWindowWidth)
 //                  , ForkWindowWidthTolerance);
          Batch2.push_back((LineSegmentStruct)(*it3));
					continue;
				}
				if (DistanceFromPointToLine(Line2MidPoint, Line1MidPoint, Line3MidPoint) > MidPilarPointFromSidePilarLineDistanceMax)
				{
//          ROS_INFO("Rejeted Since [%3.3f] is larger than MidPilarPointFromSidePilarLineDistanceMax [%3.3f]",
//                   DistanceFromPointToLine(Line2MidPoint, Line1MidPoint, Line3MidPoint)
//                   , MidPilarPointFromSidePilarLineDistanceMax);
          Batch2.push_back((LineSegmentStruct)(*it3));
					continue;
				}

				AllIntermediateLines.insert( AllIntermediateLines.end(), Batch1.begin(), Batch1.end() );
				AllIntermediateLines.insert( AllIntermediateLines.end(), Batch2.begin(), Batch2.end() );

				DefetiveLinesegmentFound = false;
				for ( std::list <LineSegmentStruct>::iterator it4 = AllIntermediateLines.begin(); it4!=AllIntermediateLines.end(); ++it4)
				{
					TempLine = ((LineSegmentStruct)(*it4));
					TempLineP1.x = TempLine.StartX;
					TempLineP1.y = TempLine.StartY;
					TempLineP2.x = TempLine.EndX;
					TempLineP2.y = TempLine.EndY;
          if (DistanceFromPointToLine(TempLineP1,Line1MidPoint, Line3MidPoint) < ToleranceFromLongLine && DistanceFromPointToLine(TempLineP2,Line1MidPoint, Line3MidPoint) < ToleranceFromLongLine)
					{
						DefetiveLinesegmentFound = true;
						break;
					}
					if (TempLineP1.x < std::min(std::min(Line1MidPoint.x, Line2MidPoint.x), Line3MidPoint.x) || TempLineP2.x < std::min(std::min(Line1MidPoint.x, Line2MidPoint.x), Line3MidPoint.x))
					{
						DefetiveLinesegmentFound = true;
						break;
					}
				}
				if (DefetiveLinesegmentFound)
				{
					AllIntermediateLines.clear();
					Batch2.push_back((LineSegmentStruct)(*it3));
					Pallet.StartX = 0;
					Pallet.StartY = 0;
					Pallet.EndX = 0;
					Pallet.EndY = 0;
				}
				else
				{
					Pallet.StartX = Line1MidPoint.x;
					Pallet.StartY = Line1MidPoint.y;
					Pallet.EndX = Line3MidPoint.x;
					Pallet.EndY = Line3MidPoint.y;
          Pallets.push_back(Pallet);
          PointSetFound = true;
          break;
				}
			}
      if(PointSetFound)
        break;
			Batch1.push_back((LineSegmentStruct)(*it2));
		}
	}
  return Pallets;
}

pcl::PointCloud<pcl::PointXYZ> DetectionClass::ShowOutputCloud (vector<sensor_msgs::LaserScan> inputscans)
{
	pcl::PointCloud<pcl::PointXYZ> BigCloud;
	laser_geometry::LaserProjection projector_;
	sensor_msgs::PointCloud2 BigCloudMsg;
	tf::TransformListener listener_;

	for (uint Counter = 0; Counter < PosVector.size(); Counter++)
	{
        sensor_msgs::LaserScan scan_in = inputscans[Counter];
		// Convert laserscan message to RosPointCloud2 Message
//		listener_.waitForTransform(scan_in.header.frame_id, "/vertical_laser_link", scan_in.header.stamp + ros::Duration().fromSec(scan_in.ranges.size()*scan_in.time_increment),ros::Duration(1));
		sensor_msgs::PointCloud2 roscloud;
		projector_.transformLaserScanToPointCloud("/vertical_laser_link", scan_in, roscloud,listener_);
		//Convert RosPointCloud2 to PCLPointCloud2
		pcl::PCLPointCloud2 pcl_pointcloud2;
		pcl_conversions::toPCL(roscloud,pcl_pointcloud2);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::fromPCLPointCloud2(pcl_pointcloud2, *cloudXYZ);

		//Transform PCLPointCloud<T>
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate (Eigen::AngleAxisf (PosVector[Counter] * M_PI / 180.0 - M_PI/2.0, Eigen::Vector3f::UnitY()));
		pcl::transformPointCloud (*cloudXYZ, *transformed_cloud, transform);
		transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, SensorElevation;
		pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform);
//		ROS_INFO ("Appending New Scan ... [%d] @ [%6.3f] Degrees. ", Counter+1, PosVector[Counter]);

		//Append to Big PointCloud
		BigCloud+= (*transformed_cloud);
	}
	BigCloud.header.frame_id = "/vertical_laser_link";
	pcl::toROSMsg(BigCloud, BigCloudMsg);
	pub3D.publish(BigCloudMsg);
	ros::spinOnce();
  return BigCloud;
}

void DetectionClass::DrawLine(ros::Publisher * vispub, LineSegmentStruct TheLine)
{
	geometry_msgs::Point pt;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/vertical_laser_link";
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.id =0;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.0;
	marker.scale.z = 0.05;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
//	marker.lifetime = ros::Duration(4);

	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "/vertical_laser_link";
	marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.id =1;
	marker2.pose.orientation.x = 0.0;
	marker2.pose.orientation.y = 0.0;
	marker2.pose.orientation.z = 0.0;
	marker2.pose.orientation.w = 1.0;
	marker2.scale.x = 0.1;
	marker2.scale.y = 0.0;
	marker2.scale.z = 0.1;
	marker2.color.a = 1.0;
	marker2.color.r = 1.0;
	marker2.color.g = 0.0;
	marker2.color.b = 0.0;
//	marker2.lifetime = ros::Duration(4);

  float angle = (atan2(TheLine.EndY-TheLine.StartY,TheLine.EndX-TheLine.StartX)- M_PI/2.0) *180.0/M_PI;
  std::string s="";
  pt.x=TheLine.StartX ;
  pt.y=TheLine.StartY ;
  pt.z= 0.02;//0.34;
  marker.points.push_back(pt);
  pt.x=TheLine.EndX ;
  pt.y=TheLine.EndY ;
  pt.z= 0.02;//0.34;
  marker.points.push_back(pt);
  marker2.pose.position.x = (TheLine.StartX + TheLine.EndX)/2.0;
  marker2.pose.position.y = (TheLine.StartY + TheLine.EndY)/2.0;
  marker2.pose.position.z = 0.3; //0.3
  s.append("[");
  s.append(ToString((TheLine.StartY + TheLine.EndY)/-2.0));
  s.append(" , ");
  s.append(ToString((TheLine.StartX + TheLine.EndX)/2.0));
  s.append("] , Angle = [");
  s.append(ToString(angle));
  s.append("] degrees.");
  s.append("Pallet width = [");
  s.append(ToString(LineLenght(TheLine)));
  s.append("] meters.");
  marker2.text = s;

  if (!marker.points.empty())
	{
		vispub->publish( marker );
		ros::spinOnce();
		vispub->publish( marker2 );
		ros::spinOnce();
	}
}

void DetectionClass::DrawLines(ros::Publisher * vispub, std::vector <LineSegmentStruct> Lines)
{
  if (Lines.size()  == 0)
    return;
  geometry_msgs::Point pt;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/vertical_laser_link";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id =3;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.0;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
//	marker.lifetime = ros::Duration(4);

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "/vertical_laser_link";
  marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.id =4;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = 0.1;
  marker2.scale.y = 0.0;
  marker2.scale.z = 0.1;
  marker2.color.a = 1.0;
  marker2.color.r = 1.0;
  marker2.color.g = 0.0;
  marker2.color.b = 0.0;
//	marker2.lifetime = ros::Duration(4);

  int PalletCounter=0;
  for ( std::vector <LineSegmentStruct>::iterator it = Lines.begin(); it!=Lines.end(); ++it)
  {
    if (((LineSegmentStruct)(*it)).StartX != 0 || ((LineSegmentStruct)(*it)).StartY != 0)
    {
//      float angle = (atan2(((LineSegmentStruct)(*it)).EndY-((LineSegmentStruct)(*it)).StartY,((LineSegmentStruct)(*it)).EndX-((LineSegmentStruct)(*it)).StartX)- M_PI/2.0) *180.0/M_PI;
//      if (fabs(angle) <45)
      {
        std::string s="";
        pt.x=((LineSegmentStruct)(*it)).StartX ;
        pt.y=((LineSegmentStruct)(*it)).StartY ;
                pt.z= 0.1;//0.34;
        marker.points.push_back(pt);
        pt.x=((LineSegmentStruct)(*it)).EndX ;
        pt.y=((LineSegmentStruct)(*it)).EndY ;
                pt.z= 0.1;//0.34;
        marker.points.push_back(pt);
/*				marker2.pose.position.x = (((LineSegmentStruct)(*it)).StartX + ((LineSegmentStruct)(*it)).EndX)/2.0;
        marker2.pose.position.y = (((LineSegmentStruct)(*it)).StartY + ((LineSegmentStruct)(*it)).EndY)/2.0;
        marker2.pose.position.z = 0.3; //0.3
        s.append("[");
                s. append(ToString((((LineSegmentStruct)(*it)).StartY + ((LineSegmentStruct)(*it)).EndY)/-2.0));
                s.append(" , ");
                s. append(ToString((((LineSegmentStruct)(*it)).StartX + ((LineSegmentStruct)(*it)).EndX)/2.0));
        s.append("] , Angle = [");
        s.append(ToString(angle));
        s.append("] degrees");
        marker2.text = s;*/
        PalletCounter++;
      }
    }
  }
  if (!marker.points.empty())
  {
    vispub->publish( marker );
    ros::spinOnce();
    vispub->publish( marker2 );
    ros::spinOnce();
  }
}

LineSegmentStruct TransformLine (LineSegmentStruct BetterLine,LineSegmentStruct Origline)
{
  LineSegmentStruct TransformedLine;
  float Tx = (Origline.StartX + Origline.EndX)/2.0;
  float Ty = (Origline.StartY + Origline.EndY)/2.0;
  float Theta = atan2(Origline.EndY-Origline.StartY,Origline.EndX-Origline.StartX)- M_PI/2.0;

  std::cout << "Origin StartX:" << Origline.StartX << std::endl;
  std::cout << "Origin StartY:" << Origline.StartY << std::endl;
  std::cout << "Origin EndX:" << Origline.EndX << std::endl;
  std::cout << "Origin EndY:" << Origline.EndY << std::endl;
  //std::cout << "cos(TAng):" << cos(TAng) << std::endl;
//  std::cout << "sin(TAng):" << sin(TAng) << std::endl;
/*  std::cout << "Origline.StartX * cos(TAng):" << Origline.StartX * cos(TAng) << std::endl;
  std::cout << "Origline.StartY * sin(TAng):" << Origline.StartY * sin(TAng) << std::endl;
  std::cout << "Origline.EndX * cos(TAng):" << Origline.EndX * cos(TAng) << std::endl;
  std::cout << "Origline.EndY * cos(TAng):" << Origline.EndY * cos(TAng) << std::endl;*/
/*  TransformedLine.StartX = Origline.StartX * cos(TAng) - Origline.StartY * sin(TAng) + Tx;
  TransformedLine.StartY = Origline.StartX * sin(TAng) + Origline.StartY * cos(TAng) + Ty;
  TransformedLine.EndX = Origline.EndX * cos(TAng) - Origline.EndY * sin(TAng) + Tx;
  TransformedLine.EndY = Origline.EndX * sin(TAng) + Origline.EndY * cos(TAng) + Ty;*/
    TransformedLine.StartX = BetterLine.StartX * cos(Theta) - BetterLine.StartY * sin(Theta) + Tx;
    TransformedLine.StartY = BetterLine.StartX * sin(Theta) + BetterLine.StartY * cos(Theta) + Ty;
    TransformedLine.EndX = BetterLine.EndX * cos(Theta) - BetterLine.EndY * sin(Theta) + Tx;
    TransformedLine.EndY = BetterLine.EndX * sin(Theta) + BetterLine.EndY * cos(Theta) + Ty;
  return TransformedLine;
}

geometry_msgs::Pose2D DetectionClass::DetectPallet()
{
    geometry_msgs::Pose2D pose2d;
    bool FirstLineFound = false;
    for (int FOVCounter = 9; FOVCounter>-1; FOVCounter--)
    {
        PalletLines.clear();
        pose2d.x = 0;
        pose2d.y = 0;
        pose2d.theta = 0;
        vector<sensor_msgs::LaserScan> TrimmedScans (Scans);
        int StartTrimZone = FOVCounter * Scans[0].ranges.size()/20;
        int EndTrimZone = Scans[0].ranges.size() /2 + (10-FOVCounter) * Scans[0].ranges.size()/20;
        std::cout << "-------------------------------------------"<<std::endl;
        std::cout << "------------    2D Analysis  --------------"<<std::endl;
        ROS_INFO ("Searching For Pallets, FOV = [%3.3f] Degrees", (10-FOVCounter) * (Scans[0].angle_max- Scans[0].angle_min) /10 *180/M_PI);
        for (uint ScanCounter = 0; ScanCounter< Scans.size(); ScanCounter++)
        {
            for (uint RayCounter = 0; RayCounter< Scans[ScanCounter].ranges.size(); RayCounter++)
            {
                if (RayCounter < StartTrimZone || RayCounter > EndTrimZone)
                    TrimmedScans[ScanCounter].ranges[RayCounter] = 0;
            }
        }
        for (uint Counter = 0; Counter < PosVector.size(); Counter++)
        {
            sensor_msgs::LaserScan LASERscan = TrimmedScans[Counter];
            std::list<LineSegmentStruct> LineSegments = GetLineSegments(LASERscan, MinSquareLineLenght, MaxSquareLineLenght, MininPointsCount);

            std::vector <LineSegmentStruct> candidatelines = DetectPalletFromThreePillarFrontFaceLines(LineSegments);
            if (!candidatelines.empty())
            {
              for ( std::vector <LineSegmentStruct>::iterator it = candidatelines.begin(); it!=candidatelines.end(); ++it)
              {
                LineSegmentStruct candidateline = (LineSegmentStruct)(*it);
                float angle = (atan2(candidateline.EndY-candidateline.StartY,candidateline.EndX-candidateline.StartX)- M_PI/2.0) *180.0/M_PI;
                if (fabs(angle) < 45)
                {
                    ROS_INFO ("Pallet candidate from scan [%d] @ angle [%6.3f] degrees: ( X = [%6.3f] , Y = [%6.3f] Theta = [%6.3f] )",
                            Counter+1, PosVector[Counter],
                            (candidateline.StartY + candidateline.EndY)/2.0,
                            (candidateline.StartX + candidateline.EndX)/2.0, angle);
                    PalletLines.push_back(candidateline);
                }
              }
            }
        }
        pcl::PointCloud<pcl::PointXYZ> Cloud3d;
        if (Publish3D)
            Cloud3d = ShowOutputCloud(TrimmedScans);
        if (!PalletLines.empty())
        {
          if (PalletLines.size()>1)
            for ( std::vector <LineSegmentStruct>::iterator it = PalletLines.begin(); it!=PalletLines.end(); ++it)
              for ( std::vector <LineSegmentStruct>::iterator it2 = it+1; it2!=PalletLines.end(); ++it2)
              {
                LineSegmentStruct candidateline = (LineSegmentStruct)(*it);
                LineSegmentStruct candidateline2 = (LineSegmentStruct)(*it2);
                float cen1 = (candidateline.StartY+candidateline.EndY)/2.0;
                float cen2 = (candidateline2.StartY+candidateline2.EndY)/2.0;
                if (fabs(cen1)>fabs(cen2))
                  std::iter_swap (it,it2);
              }
          std::cout << "-------------------------------------------"<<std::endl;
          ROS_INFO ("Here is the sorted list of possible pallet lines:");
          int counter=1;
          for ( std::vector <LineSegmentStruct>::iterator it = PalletLines.begin(); it!=PalletLines.end(); ++it)
          {
            LineSegmentStruct candidateline = (LineSegmentStruct)(*it);
            float cen1 = (candidateline.StartY+candidateline.EndY)/2.0;
            std::cout << counter++ << "- ["<<((LineSegmentStruct)(*it)).StartX << " ," <<((LineSegmentStruct)(*it)).StartY <<
                         "] to [" <<((LineSegmentStruct)(*it)).EndX << " ,"<<((LineSegmentStruct)(*it)).EndY << "]." <<
                         " Len = [" << LineLenght((LineSegmentStruct)(*it)) << "];"<< "Centered at: [" << cen1 << "]." <<std::endl;

          }
          if (PublishRvizPalletLine)
            {
              ConfirmLineStruct ConfirmationThrough3DAnalysis;
                  ConfirmationThrough3DAnalysis.IsConfirmed= false;
              if (FirstLineFound)
              {
                std::cout << "--------------------------------------------"<<std::endl;
                std::cout << "------------    3D Analysis  ---------------"<<std::endl;
                for ( std::vector <LineSegmentStruct>::iterator it = PalletLines.begin(); it!=PalletLines.end(); ++it)
                {
                  LineSegmentStruct candidateline = (LineSegmentStruct)(*it);
                  std::cout << std::endl;
                  std::cout << "3D study of the pallet candidate: ["<<((LineSegmentStruct)(*it)).StartX << " ," <<((LineSegmentStruct)(*it)).StartY <<
                               "] to [" <<((LineSegmentStruct)(*it)).EndX << " ,"<<((LineSegmentStruct)(*it)).EndY << "]." <<std::endl;
                  ConfirmationThrough3DAnalysis = Further_3Dto2D_Analyze(candidateline, Cloud3d);
                  if (ConfirmationThrough3DAnalysis.IsConfirmed )
                  {
 /*                   pose2d.x = -(candidateline.StartY + candidateline.EndY)/2.0;
                    pose2d.y = (candidateline.StartX + candidateline.EndX)/2.0;
                    pose2d.theta = (atan2(candidateline.EndY-candidateline.StartY,candidateline.EndX-candidateline.StartX)- M_PI/2.0);
                    DrawLine(&vis_pub,candidateline);*/

                    LineSegmentStruct correctedline = TransformLine(ConfirmationThrough3DAnalysis.ConfirmedLine,candidateline);
                    pose2d.x = -(correctedline.StartY + correctedline.EndY)/2.0;
                    pose2d.y = (correctedline.StartX + correctedline.EndX)/2.0;
                    pose2d.theta = (atan2(correctedline.EndY-correctedline.StartY,correctedline.EndX-correctedline.StartX)- M_PI/2.0);
                    DrawLine(&vis_pub,correctedline);

                    float Linelen = LineLenght(correctedline);

                    std::cout <<std::endl;
                    std::cout <<"******************** >>>>>>>>>>>>>>>>>>>>>   Pallet Found and Confirmed! Here it is:" <<std::endl;;
                    std::cout << "["<<((LineSegmentStruct)(*it)).StartX << " ," <<((LineSegmentStruct)(*it)).StartY <<
                                 "] to [" <<((LineSegmentStruct)(*it)).EndX << " ,"<<((LineSegmentStruct)(*it)).EndY << "]." <<
                                 "Which means, after applying 3D correction, there is a pallet centered at [" << pose2d.x << ", " << pose2d.y <<
                                 "], at " << pose2d.theta<< " radians ("<< pose2d.theta *180/ M_PI <<
                                 " degrees). The lenght of the pallet is [" <<  Linelen << "] meters." << std::endl;
                    break;
                  }
                  else
                  {
                    ROS_INFO ("Checking Other Possibilities ...");
                    continue;
                  }
                }
              }
              if (ConfirmationThrough3DAnalysis.IsConfirmed)
              {
                break;
              }
            }
            if (!FirstLineFound)
            {
              FirstLineFound  = true; // but still broaden the FOV one more step
              if (FOVCounter>0)
              {
                ROS_INFO("However, let's increase the FOV one more step, and reconfirm...");
                std::cout << "-------------------------------------------"<<std::endl;
                continue;
              }
              else
                break;
            }
            else
            {
              break;
            }
        }
        else
        {
            ROS_INFO ("No Pallets Found At within this FOV, Broadening the FOV ...");
        }
    }
    if (PalletLines.empty())
    {
        ROS_INFO ("No Pallets Found At All !!!");
    }
    if (Publish3D)
        ShowOutputCloud(Scans);
    if (pose2d.x == 0 && pose2d.y == 0)
      ROS_INFO("Pallet not detected.");
    std::cout << "-------------------------------------------"<<std::endl;
    std::cout << "Waiting for the next request ..." << std::endl;
    return pose2d;
}

sensor_msgs::LaserScan DetectionClass::PointCloud2LASERScan(sensor_msgs::PointCloud2 cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header.frame_id = "/vertical_laser_link";

  output.angle_min = -M_PI / 2.0;
  output.angle_max = M_PI / 2.0;
  output.angle_increment = 1 * M_PI / 180.0;
  output.time_increment = 0.0;
  output.scan_time = 1 / 40.0;
  output.range_min = 0.50;
  output.range_max = 10;

  float max_height = 0.1;
  float min_height = -0.1;

  // determine amount of rays to create
  uint32_t ranges_size =
      std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // initialize with zero
  output.ranges.assign(ranges_size, 0);

  sensor_msgs::PointCloud2ConstPtr cloud_out(
      new sensor_msgs::PointCloud2(cloud_msg));

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"),
       iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {


/*    if (*iter_z > max_height || *iter_z < min_height)
    {
      ROS_INFO("rejected for height %f not in range (%f, %f)\n", *iter_z,
               min_height, max_height);
      continue;
    }*/

    double range = 100;
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
       range = 100;
    }
    else
    {
      range = hypot(*iter_x, *iter_y);
    }

    if (range < output.range_min)
    {
/*      ROS_INFO(
          "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
          range, output.range_min, *iter_x, *iter_y, *iter_z);*/
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
/*      ROS_INFO("rejected for angle %f not in range (%f, %f)\n", angle,
               output.angle_min, output.angle_max);*/
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (output.ranges[index] <0.1)
    {
      output.ranges[index] = range;
    }
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }

  for (int index = 1; index < output.ranges.size()-1 ; index++)
  {
    if (output.ranges[index]  <0.1 && output.ranges[index+1]>0.1 && output.ranges[index-1]>0.1)
      output.ranges[index] = (output.ranges[index+1]+output.ranges[index-1])/2.0;
  }

  Projectedlaserscan.publish(output);
  ros::spinOnce();

  sensor_msgs::LaserScan highresolutionoutput;
  highresolutionoutput.header.frame_id = "/vertical_laser_link";

  highresolutionoutput.angle_min = -M_PI / 2.0;
  highresolutionoutput.angle_max = M_PI / 2.0;
  highresolutionoutput.angle_increment = 0.25 * M_PI / 180.0;
  highresolutionoutput.time_increment = 0.0;
  highresolutionoutput.scan_time = 1 / 40.0;
  highresolutionoutput.range_min = 0.50;
  highresolutionoutput.range_max = 10;
  highresolutionoutput.header.frame_id = "/vertical_laser_link";

  // determine amount of rays to create
  ranges_size = std::ceil((highresolutionoutput.angle_max - highresolutionoutput.angle_min) / highresolutionoutput.angle_increment);

  // initialize with zero
  highresolutionoutput.ranges.assign(ranges_size, 0);

  for (int index = 0; index < output.ranges.size()-1 ; index++)
  {
    highresolutionoutput.ranges [4*index] = output.ranges[index];
    highresolutionoutput.ranges [4*index+1] = (3*output.ranges[index] + output.ranges[index+1])/4.0;
    highresolutionoutput.ranges [4*index+2] = (2*output.ranges[index] + 2*output.ranges[index+1])/4.0;
    highresolutionoutput.ranges [4*index+3] = (output.ranges[index] + 3*output.ranges[index+1])/4.0;
  }
  highresolutionoutput.ranges [highresolutionoutput.ranges.size()-1 ] = output.ranges [output.ranges.size()-1 ];
  Projectedlaserscan.publish(highresolutionoutput);
  ros::spinOnce();
  return (highresolutionoutput);
}

ConfirmLineStruct DetectionClass::Further_3Dto2D_Analyze(LineSegmentStruct candidateline, pcl::PointCloud<pcl::PointXYZ> Cloud3d)
{
  sensor_msgs::PointCloud2 BigCloudMsg;
/*  pcl::toROSMsg(Cloud3d, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();*/


  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud( new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -(candidateline.EndX + candidateline.StartX) / 2, -(candidateline.EndY + candidateline.StartY) / 2, 0.0;
  pcl::transformPointCloud(Cloud3d, *transformed_cloud, transform);

  transform = Eigen::Affine3f::Identity();
  float angle = (atan2(candidateline.EndY - candidateline.StartY, candidateline.EndX - candidateline.StartX) -  M_PI / 2.0);
  transform.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform);

  /*pcl::toROSMsg(*transformed_cloud, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();*/

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(transformed_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.8, +0.8);
  pass.filter(*transformed_cloud);
  pass.setInputCloud(transformed_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.2, +1.2);
  pass.filter(*transformed_cloud);

  /*pcl::toROSMsg(*transformed_cloud, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPlane_cloud( new pcl::PointCloud<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  coefficients->values.resize(4);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
  seg.setAxis(axis);
  seg.setEpsAngle(5 * M_PI / 180.0);
  seg.setInputCloud (transformed_cloud);
  seg.segment (*inliers, *coefficients);
  pcl::copyPointCloud<pcl::PointXYZ>( *transformed_cloud, inliers->indices, *GroundPlane_cloud );
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " "
                                    << coefficients->values[3] << std::endl;
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  /*pcl::toROSMsg(*transformed_cloud, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();*/

  if (fabs(coefficients->values[0] - 1) < 0.2 &&
      fabs(coefficients->values[1]) < 0.2 &&
      fabs(coefficients->values[2] ) < 0.2)
  {
    transformed_cloud = GroundPlane_cloud;
  }
  else   if (fabs(coefficients->values[0] ) < 0.1 &&
             fabs(coefficients->values[1]) < 0.1 &&
             fabs(coefficients->values[2] -1 ) < 0.1)
  {
  transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(-coefficients->values[0], Eigen::Vector3f::UnitY()));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform);
  pcl::transformPointCloud(*GroundPlane_cloud, *GroundPlane_cloud, transform);
  transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(coefficients->values[1], Eigen::Vector3f::UnitX()));
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform);
  pcl::transformPointCloud(*GroundPlane_cloud, *GroundPlane_cloud, transform);
  transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.0, 0.0, coefficients->values[3];
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform);
  pcl::transformPointCloud(*GroundPlane_cloud, *GroundPlane_cloud, transform);

  coefficients.reset(new pcl::ModelCoefficients);
  coefficients->values.resize(4);
  inliers.reset(new pcl::PointIndices);
  seg.setInputCloud (transformed_cloud);
  seg.segment (*inliers, *coefficients);
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " "
                                    << coefficients->values[3] << std::endl;

  /*pcl::toROSMsg(*transformed_cloud, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();*/

  pcl::ExtractIndices<pcl::PointXYZ> eifilter (true); // Initializing with true will allow us to extract the removed indices
  eifilter.setInputCloud (transformed_cloud);
  eifilter.setIndices (inliers);
  eifilter.setNegative (true);
  eifilter.filter (*transformed_cloud);
}

  /*pcl::toROSMsg(*transformed_cloud, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();*/

  coefficients.reset(new pcl::ModelCoefficients);
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;


  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(transformed_cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*transformed_cloud);

  transform = Eigen::Affine3f::Identity();
  transform.translation() << 1.0, 0.0, 0.0;
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform);


  pcl::toROSMsg(*transformed_cloud, BigCloudMsg);
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();
  sensor_msgs::LaserScan projectedscan = PointCloud2LASERScan(BigCloudMsg);
  ConfirmLineStruct CouldConfirmThePalletThrough3DAnalysis;
  CouldConfirmThePalletThrough3DAnalysis.IsConfirmed = false;
  float temp = AllowedPointDistance;
  AllowedPointDistance = 0.10;
  std::list<LineSegmentStruct> LineSegments = GetLineSegments(projectedscan, 0.5, 2, MininPointsCount);
  if (!LineSegments.empty())
  {
    std::vector<LineSegmentStruct> v;
    v.reserve(LineSegments.size());
    v.insert(v.end(),LineSegments.begin(), LineSegments.end());
    DrawLines(&vis_pub,v);
  }
  for ( std::list <LineSegmentStruct>::iterator it = LineSegments.begin(); it != LineSegments.end(); ++it)
  {
    if (((LineSegmentStruct)(*it)).StartX != 0 || ((LineSegmentStruct)(*it)).StartY != 0)
    {
      LineSegmentStruct LineToCheck = (LineSegmentStruct)(*it);
      LineToCheck.StartX -=1;
      LineToCheck.EndX -=1;
      float angle = (atan2(((LineSegmentStruct)(*it)).EndY-((LineSegmentStruct)(*it)).StartY,((LineSegmentStruct)(*it)).EndX-((LineSegmentStruct)(*it)).StartX)- M_PI/2.0) *180.0/M_PI;
      float centerx = (((LineSegmentStruct)(*it)).StartX + ((LineSegmentStruct)(*it)).EndX)/2.0;
      float centery = (((LineSegmentStruct)(*it)).StartY + ((LineSegmentStruct)(*it)).EndY)/2.0;
      float linelen = LineLenght((LineSegmentStruct)(*it));
//      ROS_INFO("There is a Line at: [X= %3.3f , Y = %3.3f] with angle: %3.3f Degrees. Line Lenght = %3.3f meters.", centery,centerx,angle,linelen);
      if (fabs(angle) < PalletAngleToleranceFor3DAnalysis &&
          fabs(centery) <PalletCenterToleranceFor3DAnalysis &&
          fabs(centerx-1) <PalletCenterToleranceFor3DAnalysis &&
          fabs(linelen - PalletWidthFor3DAnalysis) <PalletWidthToleranceFor3DAnalysis)
      {
        ROS_INFO("3D analysis confirms that there is a pallet at this spot.   V^V^V^V^V^V^V ");
        CouldConfirmThePalletThrough3DAnalysis.IsConfirmed = true;
        CouldConfirmThePalletThrough3DAnalysis.ConfirmedLine = LineToCheck;
        break;
      }
      else
      {
        ROS_INFO("3D analysis does not confirm the existance of a pallet at this spot.    XXXXXXXXXXXXX");
      }
    }
  }
  AllowedPointDistance = temp;
  pub3D.publish(BigCloudMsg);
  ros::spinOnce();
  return CouldConfirmThePalletThrough3DAnalysis;
}

void ReadParams(ros::NodeHandle nh)
{
	std::string param_name(ros::this_node::getName());
	param_name.append("/ShouldReportParams");
	bool ShouldReportParams = true;
	if (nh.hasParam(param_name))
	{
		nh.getParam(param_name, ShouldReportParams);
	}
	else
	{
		ROS_WARN("ShouldReportParams was not set as param. All parameter readings will be reported ...");
	}
	if (ShouldReportParams)
	{
		param_name = ros::this_node::getName();
		param_name.append("/StartSwipAngle");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, StartSwipAngle);
			ROS_INFO("StartSwipAngle is set as external parameter [%f]", StartSwipAngle);
		}
		else
		{
			ROS_WARN("StartSwipAngle is (!!!)NOT(!!!) set as external parameter. Default value  [%f] used.", StartSwipAngle);
		}

		param_name = ros::this_node::getName();
		param_name.append("/EndSwipAngle");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, EndSwipAngle);
			ROS_INFO("EndSwipAngle is set as external parameter [%f]", EndSwipAngle);
		}
		else
		{
			ROS_WARN("EndSwipAngle is (!!!)NOT(!!!) set as external parameter. Default value  [%f] used.", EndSwipAngle);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MinCaptureAngle");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, MinCaptureAngle);
			ROS_INFO("MinCaptureAngle is set as external parameter [%f]", MinCaptureAngle);
		}
		else
		{
			ROS_WARN("MinCaptureAngle is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", MinCaptureAngle);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MaxCaptureAngle");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, MaxCaptureAngle);
			ROS_INFO("MaxCaptureAngle is set as external parameter [%f]", MaxCaptureAngle);
		}
		else
		{
			ROS_WARN("MaxCaptureAngle is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", MaxCaptureAngle);
		}

		param_name = ros::this_node::getName();
		param_name.append("/HomeAngle");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, HomeAngle);
			ROS_INFO("HomeAngle is set as external parameter [%f]", HomeAngle);
		}
		else
		{
			ROS_WARN("HomeAngle is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", HomeAngle);
		}

		param_name = ros::this_node::getName();
		param_name.append("/HomeSpeed");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, HomeSpeed);
			ROS_INFO("HomeSpeed is set as external parameter [%d]", HomeSpeed);
		}
		else
		{
			ROS_WARN("HomeSpeed is (!!!)NOT(!!!) set as external parameter. Default value [%d] used.", HomeSpeed);
		}

		param_name = ros::this_node::getName();
		param_name.append("/RotationSpeed");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, RotationSpeed);
			ROS_INFO("RotationSpeed is set as external parameter [%d]", RotationSpeed);
		}
		else
		{
			ROS_WARN("RotationSpeed is (!!!)NOT(!!!) set as external parameter. Default value [%d] used.", RotationSpeed);
		}

		param_name = ros::this_node::getName();
		param_name.append("/RotationStepSize");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, RotationStepSize);
			ROS_INFO("RotationStepSize is set as external parameter [%d]", RotationStepSize);
		}
		else
		{
			ROS_WARN("RotationStepSize is (!!!)NOT(!!!) set as external parameter. Default value [%d] used.", RotationStepSize);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MinSquareLineLenght");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, MinSquareLineLenght);
			ROS_INFO("MinSquareLineLenght is set as external parameter [%f]", MinSquareLineLenght);
		}
		else
		{
			ROS_WARN("MinSquareLineLenght is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", MinSquareLineLenght);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MaxSquareLineLenght");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, MaxSquareLineLenght);
			ROS_INFO("MaxSquareLineLenght is set as external parameter [%f]", MaxSquareLineLenght);
		}
		else
		{
			ROS_WARN("MaxSquareLineLenght is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", MaxSquareLineLenght);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MininPointsCount");
		if (nh.hasParam(param_name))
		{
			int dummy;
			nh.getParam(param_name, dummy);
			MininPointsCount = dummy;
			ROS_INFO("MininPointsCount is set as external parameter [%d]", dummy);
		}
		else
		{
			ROS_WARN("MininPointsCount is (!!!)NOT(!!!) set as external parameter. Default value [%d] used.", MininPointsCount);
		}

		param_name = ros::this_node::getName();
		param_name.append("/AllowedPointDistance");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, AllowedPointDistance);
			ROS_INFO("AllowedPointDistance is set as external parameter [%f]", AllowedPointDistance);
		}
		else
		{
			ROS_WARN("AllowedPointDistance is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", AllowedPointDistance);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MinPointDistanceTolerance");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, MinPointDistanceTolerance);
			ROS_INFO("MinPointDistanceTolerance is set as external parameter [%f]", MinPointDistanceTolerance);
		}
		else
		{
			ROS_WARN("MinPointDistanceTolerance is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", MinPointDistanceTolerance);
		}

		param_name = ros::this_node::getName();
		param_name.append("/PalletpillarThicknessMax");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, PalletpillarThicknessMax);
			ROS_INFO("PalletpillarThicknessMax is set as external parameter [%f]", PalletpillarThicknessMax);
		}
		else
		{
			ROS_WARN("PalletpillarThicknessMax is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", PalletpillarThicknessMax);
		}

		param_name = ros::this_node::getName();
		param_name.append("/ForkWindowWidth");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, ForkWindowWidth);
			ROS_INFO("ForkWindowWidth is set as external parameter [%f]", ForkWindowWidth);
		}
		else
		{
			ROS_WARN("ForkWindowWidth is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", ForkWindowWidth);
		}

		param_name = ros::this_node::getName();
		param_name.append("/ForkWindowWidthTolerance");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, ForkWindowWidthTolerance);
			ROS_INFO("ForkWindowWidthTolerance is set as external parameter [%f]", ForkWindowWidthTolerance);
		}
		else
		{
			ROS_WARN("ForkWindowWidthTolerance is (!!!)NOT(!!!) set as external parameter. Default value [%f] used.", ForkWindowWidthTolerance);
		}

		param_name = ros::this_node::getName();
		param_name.append("/MidPilarPointFromSidePilarLineDistanceMax");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, MidPilarPointFromSidePilarLineDistanceMax);
			ROS_INFO("MidPilarPointFromSidePilarLineDistanceMax is set as external parameter [%f]", MidPilarPointFromSidePilarLineDistanceMax);
		}
		else
		{
			ROS_WARN("MidPilarPointFromSidePilarLineDistanceMax is (!!!)NOT(!!!) set as external parameter [%f]. Default value used.", MidPilarPointFromSidePilarLineDistanceMax);
		}


		param_name = ros::this_node::getName();
		param_name.append("/SensorElevation");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, SensorElevation);
			ROS_INFO("SensorElevation is set as external parameter [%f]", SensorElevation);
		}
		else
		{
			ROS_WARN("SensorElevation is (!!!)NOT(!!!) set as external parameter [%f]. Default value used.", SensorElevation);
		}

		param_name = ros::this_node::getName();
		param_name.append("/Publish3D");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, Publish3D);
			ROS_INFO("Publish3D is set as external parameter [%s]", Publish3D ? "true" : "false");
		}
		else
		{
			ROS_WARN("Publish3D is (!!!)NOT(!!!) set as external parameter. Default value [%s] used.", Publish3D ? "true" : "false");
		}

		param_name = ros::this_node::getName();
		param_name.append("/PublishRvizPalletLine");
		if (nh.hasParam(param_name))
		{
			nh.getParam(param_name, PublishRvizPalletLine);
			ROS_INFO("PublishRvizPalletLine is set as external parameter [%s]", PublishRvizPalletLine ? "true" : "false");
		}
		else
		{
			ROS_WARN("PublishRvizPalletLine is (!!!)NOT(!!!) set as external parameter. Default value [%s] used.", PublishRvizPalletLine ? "true" : "false");
		}

    param_name = ros::this_node::getName();
    param_name.append("/PalletWidthFor3DAnalysis");
    if (nh.hasParam(param_name))
    {
      nh.getParam(param_name, PalletWidthFor3DAnalysis);
      ROS_INFO("PalletWidthFor3DAnalysis is set as external parameter [%f]", PalletWidthFor3DAnalysis);
    }
    else
    {
      ROS_WARN("PalletWidthFor3DAnalysis is (!!!)NOT(!!!) set as external parameter [%f]. Default value used.", PalletWidthFor3DAnalysis);
    }

    param_name = ros::this_node::getName();
    param_name.append("/PalletWidthToleranceFor3DAnalysis");
    if (nh.hasParam(param_name))
    {
      nh.getParam(param_name, PalletWidthToleranceFor3DAnalysis);
      ROS_INFO("PalletWidthToleranceFor3DAnalysis is set as external parameter [%f]", PalletWidthToleranceFor3DAnalysis);
    }
    else
    {
      ROS_WARN("PalletWidthToleranceFor3DAnalysis is (!!!)NOT(!!!) set as external parameter [%f]. Default value used.", PalletWidthToleranceFor3DAnalysis);
    }

    param_name = ros::this_node::getName();
    param_name.append("/PalletAngleToleranceFor3DAnalysis");
    if (nh.hasParam(param_name))
    {
      nh.getParam(param_name, PalletAngleToleranceFor3DAnalysis);
      ROS_INFO("PalletAngleToleranceFor3DAnalysis is set as external parameter [%f]", PalletAngleToleranceFor3DAnalysis);
    }
    else
    {
      ROS_WARN("PalletAngleToleranceFor3DAnalysis is (!!!)NOT(!!!) set as external parameter [%f]. Default value used.", PalletAngleToleranceFor3DAnalysis);
    }

    param_name = ros::this_node::getName();
    param_name.append("/PalletCenterToleranceFor3DAnalysis");
    if (nh.hasParam(param_name))
    {
      nh.getParam(param_name, PalletCenterToleranceFor3DAnalysis);
      ROS_INFO("PalletCenterToleranceFor3DAnalysis is set as external parameter [%f]", PalletCenterToleranceFor3DAnalysis);
    }
    else
    {
      ROS_WARN("PalletCenterToleranceFor3DAnalysis is (!!!)NOT(!!!) set as external parameter [%f]. Default value used.", PalletCenterToleranceFor3DAnalysis);
    }

  }
}

bool RespondToService(palletdetector::Capture3Dservice::Request  &req, palletdetector::Capture3Dservice::Response &res)
{

  NewPosAvailable = false;
  NewPos = 0;
//  ShouldScan = false;
  PosVector.clear();
  Scans.clear();
//  rotor::rotor rotorcommand;
  CurrentPalletDetectionState = PDS_ServiceRequested;
  DetectionClass detectionobject;
  detectionobject.SweepAndScan();
  std::cout << "*** Scan Complete ***" << std::endl;
  geometry_msgs::Pose2D pose2d = detectionobject.DetectPallet();
  res.pose = pose2d;
  return true;
}

void LASERCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if (CurrentPalletDetectionState == PDS_Scanning)
  {
    if (NewPosAvailable)
    {
      float LocalNewPos = NewPos;
      NewPosAvailable =false;
      if (LocalNewPos>MinCaptureAngle && LocalNewPos < MaxCaptureAngle)
      {
        if (std::find(PosVector.begin(), PosVector.end(), LocalNewPos) == PosVector.end())
        {
          PosVector.push_back(LocalNewPos);
          Scans.push_back(*scan_in);
          ROS_INFO ("Appending Scan #[%d] @ [%6.3f] Degrees. ", (int) PosVector.size(), LocalNewPos);
        }
      }
    }
  }
}

void RotorStatusCallback(const canbusrotor::rotorstatus::ConstPtr& msg)
{
    NewPos = msg->EncoderPosDegree;
    NewPosAvailable =true;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "LiDAR_Pallet_Detector");

  ros::NodeHandle nh;
//  ros::NodeHandle nh_private( "~" );
  ReadParams(nh);

  ros::ServiceServer service = nh.advertiseService("/PalletFinderService", RespondToService);
  client = nh.serviceClient<canbusrotor::rotorposservice>("/rotor_service");

  LASERsub = nh.subscribe <sensor_msgs::LaserScan> ("/vertical_scan", 1, &LASERCallback);
  RotorStatussb = nh.subscribe <canbusrotor::rotorstatus>("/rotor_status", 1, &RotorStatusCallback);
  pub3D = nh.advertise <sensor_msgs::PointCloud2> ("output3D", 1);
//  rotor_pub = nh.advertise<canbusrotor::rotorstatus>("rotor_status", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 1 );
  Projectedlaserscan = nh.advertise<sensor_msgs::LaserScan>("projectedlaserscan", 1);

  ros::spin();

  return (0);
}
