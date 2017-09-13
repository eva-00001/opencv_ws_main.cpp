#include <ros/ros.h>   
#include <iostream> 
#include <cv_bridge/cv_bridge.h>   
#include <sensor_msgs/image_encodings.h>   
#include <image_transport/image_transport.h>  
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/aruco.hpp>
#include <zbar.h>
#include <iomanip>
#include <time.h>
#include <vector>
#include <math.h>

#include <sstream>
#include <string>

#define w 600 //the size of the map

const double MIN = -100000000.0;
const double MAX = 100000000.0;

static const std::string INPUT = "Input";
static const std::string OUTPUT="Output";
static const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
using namespace std;
using namespace zbar;
using namespace cv;

/// Function headers  

void MyFilledCircle( Mat img, Point center );  

static bool begin = false;

const double episilon = 0.001; // a small value, when the camera is near below the
							 // arcode on the ceiling, the translation vector from 
							 // the arcode to the camera's x and y components should be small
							 // if x^2+y^2 < epsilon, push the camera position in a vector
							 // the average of the positions(got when the camera is passing the arcode)
							 // is the estimated (x,y) location of the arcode
							 // valued still to be defined

struct Marker
{

  int marker_id;
  
  cv::Mat rotation_to_origin;
  cv::Mat translation_to_origin;
  std::vector<cv::Mat> locations;

}; 

struct Camera{

	cv::Mat position;
	cv::Mat rotation;
};

class ArucoMapping
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  zbar::ImageScanner scanner;


public:

  

  std::vector<Marker> all_markers;    
  std::vector<Camera> my_camera;    //my_camera.back() is the latest updated
//------------------------------------------------------------------------------------------------------------------------------
  ArucoMapping()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ArucoMapping::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(INPUT,CV_WINDOW_NORMAL);
    cv::namedWindow(OUTPUT,CV_WINDOW_NORMAL);
  }

  ~ArucoMapping()
  {
    cv::destroyWindow(INPUT);
    cv::destroyWindow(OUTPUT);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    cv::Mat img = cv_ptr->image;
    cv::Mat rs, imgcopy;
    rs=ArucoMapping::CameraCalibration(img);
    ArucoMapping::detectAruco(rs);

    cv::imshow(INPUT, img);
    cv::imshow(OUTPUT,rs);
    //cout<<CLOCKS_PER_SEC<<endl;
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }


  cv::Mat mat_average(std::vector<cv::Mat> a){      //get average mat from a vector of mats

	if(a.size() == 0){
		double la[1][1] = {9999999.0};
    	cv::Mat ka = Mat(1,1,CV_64FC1,la);
    	return ka;
	}

  	cv::Mat all = a[0];
  	for(int i=1;i<a.size();i++){
  		all+=a[i];
  	}
  	all = all/(double)a.size();
  	return all;
  }

  
private:
	
  cv::Mat CameraCalibration(cv::Mat img){

    cv::Mat t;
    Size image_size = img.size();

    float intrinsic[3][3] = {1821.653803,0,1225.830659,0,1817.080614,843.131985,0,0,1};
    float distortion[1][5] = {-0.298421 , 0.095408, -0.000272, 0.000384, 0};
    Mat intrinsic_matrix = Mat(3,3,CV_32FC1,intrinsic);
    Mat distortion_coeffs = Mat(1,5,CV_32FC1,distortion);

    Mat R = Mat::eye(3,3,CV_32FC1);       
    
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    
    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
    
    
    t = img.clone();
    cv::remap(img, t, mapx, mapy, INTER_LINEAR);
    return t;
  }

  void detectAruco(cv::Mat img){
  	std::vector<cv::Vec3d> rvecs, tvecs; 
    std::vector<int> ids;

    std::vector<std::vector<cv::Point2f> > corners;

    float intrinsic[3][3] = {1821.653803,0,1225.830659,0,1817.080614,843.131985,0,0,1};
    float distortion[1][5] = {-0.298421 , 0.095408, -0.000272, 0.000384, 0};
    Mat intrinsic_matrix = Mat(3,3,CV_32FC1,intrinsic);
    Mat distortion_coeffs = Mat(1,5,CV_32FC1,distortion); 

    cv::aruco::detectMarkers(img, dictionary, corners, ids);
    
    cv::aruco::drawDetectedMarkers(img, corners, ids);

    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, intrinsic_matrix, distortion_coeffs, rvecs, tvecs);

    //rvecs and tvecs are double
   if (ids.size() > 0) {
   	cout<<endl<<endl<<endl<<"begin------------------------------------"<<endl;
    
    if(!begin){                     

    	//the first time the camera detects arcode, initialize 

		double kakaka1[1][3] = {0,0,0};
		cv::Mat new_camera_position = Mat(1,3,CV_64FC1,kakaka1);

		double kakaka2[3][3] = {1,0,0,0,1,0,0,0,1};                //to be corrected
		cv::Mat new_camera_rotation = Mat(3,3,CV_64FC1,kakaka2);

		Camera newcamera;
		newcamera.position = new_camera_position;
		newcamera.rotation = new_camera_rotation;

		my_camera.push_back(newcamera);


		for (int i=0;i<rvecs.size();i++){              //push all detected arcodes

			Marker new_marker;
			new_marker.marker_id = ids[i];

			cv::Mat marker_rotation(3,3,CV_64FC1);
			cv::Rodrigues(rvecs[i], marker_rotation);
			new_marker.rotation_to_origin = marker_rotation;

			double lalala[1][3] = {tvecs[i][0],tvecs[i][1],tvecs[i][2]};

			double dis1 = sqrt(tvecs[i][0]*tvecs[i][0] + tvecs[i][1]*tvecs[i][1] + tvecs[i][2]*tvecs[i][2]);

  			cv::Mat newmat = Mat(1,3,CV_64FC1,lalala);
  			new_marker.translation_to_origin = newmat;


			//see whether the camera is near the place below the arcode
  			double x1 = tvecs[i][0];
			double y1 = tvecs[i][1];
			if ((x1*x1 + y1*y1)<episilon){
		  		new_marker.locations.push_back(my_camera.back().position);
	  		}
	  		//---------------------------------------------------------

			cout<<ids[i]<<"'s position_to_camera: "<<tvecs[i]<<endl<<endl;

			all_markers.push_back(new_marker);

			begin = true;
		}
    }

    else{

		int visited[ids.size()];
		for(int i=0;i<ids.size();i++){
			visited[i] = 0;
		}

		cv::Mat all_positions(3,3,CV_64FC1);
		cv::Mat all_rotations(3,3,CV_64FC1);

	    double kakaka3[1][3] = {0,0,0};
	    all_positions = Mat(1,3,CV_64FC1,kakaka3);

	    double kakaka4[3][3] = {0,0,0,0,0,0,0,0,0};
	    all_rotations = Mat(3,3,CV_64FC1,kakaka4);

	    int num = 0;

	    bool exist = false;



	    //scan all detected, use them to determine the camera position
    	for (int i=0;i<ids.size();i++){
    		for(int j=0;j<all_markers.size();j++){
    			if (ids[i] == all_markers[j].marker_id){
    				exist = true;
    				visited[i] = 1;
    				num++;
    				
    				cv::Mat r0 = all_markers[j].rotation_to_origin;
    				cv::Mat r1(3,3,CV_64FC1);
            
    				cv::Rodrigues(rvecs[i], r1);
    				cv::Mat t0 = all_markers[j].translation_to_origin;

					double lalala[1][3] = {tvecs[i][0],tvecs[i][1],tvecs[i][2]};

					double dis2 = sqrt(tvecs[i][0]*tvecs[i][0] + tvecs[i][1]*tvecs[i][1] + tvecs[i][2]*tvecs[i][2]);

					//see whether the camera is near the place below the arcode
					double x2 = tvecs[i][0];
					double y2 = tvecs[i][1];
					if ((x2*x2 + y2*y2)<episilon){
				  		all_markers[j].locations.push_back(my_camera.back().position);
			  		}
			  		//----------------------------------------------------------

					cout<<ids[i]<<"'s position_to_camera: "<<tvecs[i]<<endl<<endl;

  					cv::Mat t1 = Mat(1,3,CV_64FC1,lalala);

		    		cv::Mat n_translation(3,3,CV_64FC1);
		    		n_translation = t0-t1*r1.inv()*r0; //maybe to be corrected
		            
		    		cv::Mat n_rotation(3,3,CV_64FC1);
		    		n_rotation = r1.inv()*r0;          //maybe to be corrected
		    		 
		    		all_rotations += n_rotation;
		        
		    		all_positions += n_translation;
		   
		    		break;
    			}
    		}
        }

        if(exist){
        	cv::Mat new1_camera_position = all_positions/num;
    		cv::Mat new1_camera_rotation = all_rotations/num;

    		Camera new1camera;
    		new1camera.position = new1_camera_position;
    		new1camera.rotation = new1_camera_rotation;
    		my_camera.push_back(new1camera);
        }

        num = 0;
	    exist = false;

	    // using the estimated position of camera above
	    // to compute the new markers' rotation and translation to origin
	    // push all newcomers
    	for(int i=0;i<ids.size();i++){

    		if (!visited[i]){

    			cv::Mat r0 = my_camera.back().rotation;
    			cv::Mat t0 = my_camera.back().position;

    			cv::Mat r1(3,3,CV_64FC1);
    			cv::Rodrigues(rvecs[i], r1);

    			double lalala[1][3] = {tvecs[i][0],tvecs[i][1],tvecs[i][2]};

    			double dis3 = sqrt(tvecs[i][0]*tvecs[i][0] + tvecs[i][1]*tvecs[i][1] + tvecs[i][2]*tvecs[i][2]);

  				cv::Mat t1 = Mat(1,3,CV_64FC1,lalala);

    			Marker new_marker;


    			//see whether the camera is near the place below the arcode
				double x3 = tvecs[i][0];
				double y3 = tvecs[i][1];
				if ((x3*x3 + y3*y3)<episilon){
			  		new_marker.locations.push_back(my_camera.back().position);
		  		}
		  		//---------------------------------------------------------

    			new_marker.marker_id = ids[i];

    			new_marker.rotation_to_origin = r1*r0;
    			new_marker.translation_to_origin = t1*r0+t0;

    			cout<<ids[i]<<"'s position_to_camera: "<<tvecs[i]<<endl;

    			all_markers.push_back(new_marker);
    		}
    	}
    }



    Camera omg = my_camera.back();
    cout<<"camera position:"<<omg.position<<endl<<endl;

    cout<<"number of arcodes captured:"<<all_markers.size()<<endl<<endl;
    
   }
  }
};




int main(int argc, char** argv)
{
	cout<<"start"<<endl;
  ros::init(argc, argv, "image_converter");
  ArucoMapping ic;
  ros::spin();

  //display locations, and find the size of the map
  double min_x = MAX;
  double max_x = MIN;
  double min_y = MAX;
  double max_y = MIN;
  for (int i=0;i<ic.all_markers.size();i++){

	if(ic.all_markers[i].locations.size()==0){
		//cout<<"num"<<all_markers[i].marker_id<<": nothing"<<endl;
	}

	else{
		cout<<"id"<<ic.all_markers[i].marker_id<<": "<<ic.all_markers[i].locations.size()<<"datas"<<endl;    //the number of times each arcode was detected 
	    double x = ic.mat_average(ic.all_markers[i].locations).at<double>(0,0);
	    double y = ic.mat_average(ic.all_markers[i].locations).at<double>(0,1);
	    double z = ic.mat_average(ic.all_markers[i].locations).at<double>(0,2);

	    //change unit to centimeter
	    x*=254;
	    y*=254;
	    z*=254;

	    //update min and max values
	    if (x<min_x) min_x = x;
	    if(x>max_x)max_x = x;

	    if (y<min_y) min_y = y;
	    if(y>max_y)max_y = y;

		cout<<"location "<<ic.all_markers[i].marker_id<<": "<<x<<" "<<y<<" "<<z<<endl;
	}
  }
  

  //now draw all the markers

  /// Window name 
  char markers_window[] = "Marker Locations"; 
  
  /// Create black empty image

  Mat markers_image = Mat::zeros( w+200, w+200, CV_8UC3 );  

 

  for (int i=0;i<ic.all_markers.size();i++){

	if(ic.all_markers[i].locations.size()==0){
		//cout<<"num"<<all_markers[i].marker_id<<": nothing"<<endl;
	}
	else{
		//draw each marker

	    double x = ic.mat_average(ic.all_markers[i].locations).at<double>(0,0);
	    
	    double y = ic.mat_average(ic.all_markers[i].locations).at<double>(0,1);

	    x*=254;
	    y*=254;

	    //fit x,y into the map

	    x = w*(x - min_x) / (max_x - min_x) + 100.0;
	    y = (w*(max_y-min_y)/(max_x-min_x))*(y - min_y) / (max_y - min_y) + 100.0;

	    /// Creating circles
	    cout<<"id:"<<ic.all_markers[i].marker_id<<endl; 
	    cout<<"x "<<x<<endl;
	    cout<<"y "<<y<<endl<<endl;

	    MyFilledCircle( markers_image, Point(x,y));
	    //插入文字   
	    //参数为：承载的图片，插入的文字，文字的位置（文本框左下角），字体，大小，颜色 
	    stringstream ss;
    	string words;
    	ss<<"id:"<<ic.all_markers[i].marker_id<<endl;
    	ss>>words;
	    putText( markers_image, words, Point(x,y), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 0) ); 
	}
  }
  /// Display
  imshow( markers_window, markers_image );  
  moveWindow( markers_window, 0, w/2+100 );  //initially 200
  
  waitKey(0); 

  return 0;
}
void MyFilledCircle( Mat img, Point center )  
{  
  int thickness = -1;  
  int lineType = 8;  
  
  circle( img,  
      center,  
      w/32,  
      Scalar( 0, 0, 255 ),  
      thickness,  
      lineType );  
}
//change epsilon 
//maybe get data from every detection, camera_position+r=translation_to_camera should be the location,
//instead of using just near below
//calibration(maybe just change the value is fine) needs fixing
//the algorithm, every time uses the data_to_origin

