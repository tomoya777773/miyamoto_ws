#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <random>
#include <iomanip>

#define PI 3.14159265

class Hockey_predict {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  ros::Publisher _pub;
  ros::Publisher _pub_coor;
  ros::Publisher _pub_line;
  ros::Publisher _pub_pack_pixel_state;
  ros::Publisher _pub_pack_pixel_state2;
  
  std_msgs::Int16 publish_place;
  
  int pre_Point[4][2];
  cv::Point2d velocity;
  
  double ro_x = 200;
  double ro_y = 50;
  double coe1 = 120;
  double coe2 = 15;
  double n = 8;
  double m = 3;
  
  double fps = 99.0;
  double x_min = 110;
  double x_max = 230;
  double vx_min = 30;
  double vx_max = 500;
  double vy_max = 500;
  
  const int step = 3;
  int step_count;
  int record_count;
  bool predict_flag;
  int data[3][2];

  double time_init_left     = 0.90 / 2;
  double time_init_topleft  = 0.87 / 2;
  double time_init_straight = 0.93 / 2;
  double time_init_topright = 1.74 / 2;
  double time_init_right    = 1.90 / 2;
  double during_time;
  
  int wait_publish = 200 * 1000;
  
  std::chrono::system_clock::time_point start, end;
  int sec_th = 300;

  bool deathblow;
  int deathblow_count;

  std::random_device rnd;

  cv::Mat line_img;
  
  int count_notdetect;
  int count;
  
public:
  Hockey_predict() : count(0) {

    _sub = _nh.subscribe ("/hockey_2/coordinate", 1, &Hockey_predict::predict, this);
    _pub = _nh.advertise <std_msgs::Int16> ("/hockey_2/predict/place", 5);
    //_pub_coor = _nh.advertise <std_msgs::Float32MultiArray> ("/hockey/deathblow", 100);
    _pub_pack_pixel_state = _nh.advertise <std_msgs::Float32MultiArray> ("/hockey_2/pixel_state", 100);
    _pub_pack_pixel_state2 = _nh.advertise <std_msgs::Float32MultiArray> ("/hockey_2/pixel_state2", 100);
  
    ROS_INFO ("Listing for coming data on topic /hockey/coordinate ...");
    ROS_INFO ("Advertized on topic /hockey/predict/place");
    ROS_INFO ("Advertized on topic /hockey/deathblow");
    ROS_INFO ("Advertized on topic /hockey/pixel_state");
    ROS_INFO ("Advertized on topic /hockey/pixel_state2");

    deathblow = true;
    predict_flag = false;
    step_count = 0;
    record_count = 0;

  }
  ~Hockey_predict() {}

  void predict( const std_msgs::Int16MultiArray &coor )
  {

    publish_place.data = 1;
    static std::mt19937 mt(rnd());
    static std::uniform_int_distribution<> rand100(4, 9);

    if ( coor.data[0] == 0 && coor.data[1] == 0 && count == 0){
      return;
    }

    if ( count == 0 ) {
      pre_Point[0][0] = coor.data[0];
      pre_Point[0][1] = coor.data[1];
      start = std::chrono::system_clock::now();
      deathblow_count = 0;
      count_notdetect = 0;
      count++;
      return;
    }
    if ( count < 3 ) {
      pre_Point[count][0] = coor.data[0];
      pre_Point[count][1] = coor.data[1];
      count++;
      return;
    }

    if (coor.data[0] == 0 && coor.data[1] == 0 && pre_Point[(count - 1) % 4][0] > 500.0) {
      count_notdetect++;
      if (count_notdetect > 10) {
	do {
	  //publish_place.data = rand100(mt);
	  publish_place.data = 8;
	} while ( publish_place.data == 5 );
	
	_pub.publish(publish_place);
	usleep( 1000 * 500 );
	count++;
      }
      return;
    }

    else if ( coor.data[0] == 0 && coor.data[1] == 0 ) {
      return ;
    }
    
    count_notdetect = 0;
    
    velocity.x = ( ( -coor.data[0] + 8.0 * pre_Point[(count - 1) % 4][0] - 8.0 * pre_Point[(count - 3) % 4][0] + pre_Point[count % 4][0] ) / 12.0 * fps ) / 10.0;
    velocity.y = ( ( -coor.data[1] + 8.0 * pre_Point[(count - 1) % 4][1] - 8.0 * pre_Point[(count - 3) % 4][1] + pre_Point[count % 4][1] ) / 12.0 * fps ) / 10.0;

    //std::cout << "Velocity: " << std::fixed << std::setprecision(3) << velocity.x << ", " << std::fixed <<  std::setprecision(3) << velocity.y << std::endl;
    //printf("velocity: %3f, %3f\n", velocity.x, velocity.y);
    
    std_msgs::Float32MultiArray publish_pack_pixel_state2;
    /*** publish_pack_pixel_state ***/
    publish_pack_pixel_state2.data.push_back(coor.data[1]);
    publish_pack_pixel_state2.data.push_back(coor.data[0]);
    
    publish_pack_pixel_state2.data.push_back(velocity.y);
    publish_pack_pixel_state2.data.push_back(velocity.x);
    
    _pub_pack_pixel_state2.publish(publish_pack_pixel_state2);


    // reset
    if ( record_count >= step ) {
      predict_flag = false;
      step_count = 0;
      record_count = 0;
    }
    
    // predict
    if ( ( coor.data[0] > x_min && coor.data[0] < x_max && velocity.x > vx_min && velocity.y != 0 ) || ( predict_flag == true  && record_count < step ) ) {
      
      if ( predict_flag != true ) {
	predict_flag = true;
	step_count = 0;
	record_count = 0;
      }

      if ( step_count % step == 0) {
	
	data[record_count][0] = coor.data[0]; // width
	data[record_count][1] = coor.data[1]; // height

	// If present coordinate X < previous coordinate X, or difference of both is very small, player does not shoot a pack.
	if ( record_count > 0 ) {
	  //std::cout << "reject?" << std::endl;
	  //printf("sub: %3d\n", data[record_count][0] - data[record_count - 1][0]);
	  int sub = data[record_count][0] - data[record_count - 1][0];
	  if ( sub < 0 || std::abs(sub) < 5 ) {

	    //printf("rejected\n");
	    predict_flag = false;
	    step_count = 0;
	    record_count = 0;
	    return;

	  }
	  else if ( sub > 150 ) {

	    printf("Defence\n");
	    predict_flag = false;
	    step_count = 0;
	    record_count = 0;
	    
	    publish_place.data = 2;
	    _pub.publish(publish_place);
	    return;

	  }
	}

	record_count++;
	/// PUBLISH ///
	if ( record_count == step ) {
	  
	  std_msgs::Float32MultiArray publish_pack_pixel_state;

	  /*** publish_pack_pixel_state ***/
	  for ( int i = 0; i < step; i++ ) {
	    
	    publish_pack_pixel_state.data.push_back(data[i][1]);
	    publish_pack_pixel_state.data.push_back(data[i][0]);
	    
	  }
	
	  _pub_pack_pixel_state.publish(publish_pack_pixel_state);

	   std::cout << "Predicted" << std::endl;
	   //std::cout << "velocity: " << velocity.x << std::endl;
	   deathblow = true;
	   deathblow_count = 0;
	   
	   printf("Input: ");
	   for ( int i = 0; i < step - 1; i++ ) {
	     printf("(%3d, %3d), ", data[i][0], data[i][1]);	
	   }
	   printf("(%3d, %3d)\n", data[step - 1][0], data[step - 1][1]);	

	   usleep( 1000 * 100 );
	   
	}
      }
      
      step_count++;
      return;
      
    }
    
    /*
    // If velocity of pack is too fast, baxter would defence.
    else if ( coor.data[0] > 110 && coor.data[0] < 230 && velocity.x >= vx_max && predict_flag == false ) {
      std::cout << "Defence" << std::endl;
      deathblow = true;
      deathblow_count = 0;
      
      publish_place.data = 2;

      std::cout << "Velocity: " << velocity.x << ", " << velocity.y << std::endl;
      //std::cout << "Velocity: " << (coor.data[0] - pre_Point[(count - 1) % 4][0]) * fps << ", " << (coor.data[1] - pre_Point[(count - 1) % 4][1]) * fps << std::endl;
      std::cout << "Point_x: " << coor.data[0] << ", " << pre_Point[(count - 1) % 4][0] << ", " << pre_Point[(count - 2) % 4][0]
		<< ", " << pre_Point[(count - 3) % 4][0] << ", " << pre_Point[(count - 4) % 4][0] << std::endl;
      std::cout << "Point_y: " << coor.data[1] << ", " << pre_Point[(count - 1) % 4][1] << ", " << pre_Point[(count - 2) % 4][1]
		<< ", " << pre_Point[(count - 3) % 4][1] << ", " << pre_Point[(count - 4) % 4][1] << std::endl;
      //std::cout << "velocity: " << velocity.x << std::endl;
      _pub.publish(publish_place);
      
      usleep( wait_publish );
      }*/
    
    // Deathblow
    /*
      if (coor.data[0] > 550 && std::abs(velocity.x) < 7 * fps && std::abs(velocity.y) < 7 * fps && deathblow == true) {
      
      std_msgs::Float32MultiArray publish_deathblow_coor;
      deathblow_count++;
      if (deathblow_count < 300) {
      pre_Point[(count % 4)][0] = coor.data[0];
      pre_Point[(count % 4)][1] = coor.data[1];
      count++;
      return;
      }
      deathblow = false;
      std::cout << "Deathblow!!" << std::endl;
      
      publish_place.data = 0;

      
      /*if ( coor.data[1] >= 0 && coor.data[1] < 187 ) {
      a = 0.0007694410692588089;
      b = 0.0010944714459295269;
      c = 0.00002910360884749709;
      d = 0.0010171711292200232;
      }
    */
    /*
    //if ( coor.data[0] < 640 ) {
    if ( coor.data[1] < 187 ) {
    a = 0.000568923708263384;
    b = 0.001762730833799664;
    c = -0.0000339149751572807;
    d = 0.001236200844482881;
    }
    //}
    else {
    a = 0.0012968299711815562;
    b = -0.0006724303554274724;
    c = -0.00004311273981461517;
    d = 0.001250269454623842;
    }
      
    double baxter_x = coor.data[0] * a + coor.data[1] * b;
    double baxter_y = coor.data[0] * c + coor.data[1] * d;
    std::cout << "baxter: " << baxter_x << ", " << baxter_y << std::endl;
    if ( baxter_x < 0.58 ) {
    baxter_x = 0.58;
    }
    if ( baxter_y < 0 ) {
    baxter_y = 0.05;
    }
    else if ( baxter_y > 0.4 ) {
    baxter_y = 0.4;
    }

    std::cout << "baxter: " << baxter_x << ", " << baxter_y << std::endl;
    publish_deathblow_coor.data.push_back( baxter_x );
    publish_deathblow_coor.data.push_back( baxter_y );
      
    _pub.publish( publish_place );
    _pub_coor.publish( publish_deathblow_coor );
    usleep( wait_publish );
      
    }
    */
    // When a pack stay near the baxter, baxter command publish.

    else if ( coor.data[0] > 480 && coor.data[0] <= 620 && velocity.x >= 0 && velocity.x < 10 && velocity.y < 20 ) {
      //std::cout << "Near" << std::endl;
      deathblow_count = 0;
      deathblow = true;
      cv::Point pack_baxter = cv::Point( coor.data[0] - 614, coor.data[1] - 183);
      double angle = (std::atan( (double)pack_baxter.y / (double)pack_baxter.x ) * 180.0 / PI -90.0) * -1.0;

      //std::cout << "angle = " << angle << std::endl;
      //printf("Point: %.3f, %.3f\n", (double)pack_baxter.x, (double)pack_baxter.y);

      if ( angle >= 0 && angle < 45.0 ) {
	publish_place.data = 6;
      }
      else if ( angle >= 45.0 && angle < 67.5 ) {
	publish_place.data = 9;
      }
      else if ( angle >= 67.5 && angle < 112.5 ) {
	publish_place.data = 8;
      }
      else if (angle >= 112.5 && angle < 135.0) {
	publish_place.data = 7;
      }
      else if (angle >= 135.0 && angle <= 180) {
	publish_place.data = 4;
      }
      //std::cout << "place: " << publish_place.data << std::endl;
      _pub.publish(publish_place);
      usleep( wait_publish );

    }
    
    else if ( coor.data[0] > 500 && coor.data[0] <= 620 && velocity.x <= 0 && std::abs( velocity.x ) < 10 && velocity.y < 20 ) {
      std::cout << "Near" << std::endl;
      deathblow_count = 0;
      deathblow = true;
      cv::Point pack_baxter = cv::Point( coor.data[0] - 614, coor.data[1] - 183);
      double angle = (std::atan( (double)pack_baxter.y / (double)pack_baxter.x ) * 180.0 / PI -90.0) * -1.0;
      
      //std::cout << "angle = " << angle << std::endl;
      //printf("Point: %.3f, %.3f\n", (double)pack_baxter.x, (double)pack_baxter.y);

      if ( angle >= 0 && angle < 45.0 ) {
	publish_place.data = 6;
      }
      else if ( angle >= 45.0 && angle < 67.5 ) {
	publish_place.data = 9;
      }
      else if ( angle >= 67.5 && angle < 112.5 ) {
	publish_place.data = 8;
      }
      else if (angle >= 112.5 && angle < 135.0) {
	publish_place.data = 7;
      }
      else if (angle >= 135.0 && angle <= 180) {
	publish_place.data = 4;
      }
      //std::cout << "place: " << publish_place.data << std::endl;
      _pub.publish(publish_place);
      usleep( wait_publish );

    }
    

    if ( coor.data[0] == 0 && coor.data[1] == 0 ) {
      //std::cout << "" << std::endl;
      pre_Point[(count % 4)][0] = pre_Point[(count - 1) % 4][0] + ( pre_Point[(count - 1) % 4][0] - pre_Point[(count - 2) % 4][0] );
      pre_Point[(count % 4)][1] = pre_Point[(count - 1) % 4][1] + ( pre_Point[(count - 1) % 4][1] - pre_Point[(count - 2) % 4][1] );
    }
    else {
      pre_Point[(count % 4)][0] = coor.data[0];
      pre_Point[(count % 4)][1] = coor.data[1];
    }
    count++;
    
  }
  
};


int main ( int argc, char** argv )
{

  ros::init(argc, argv, "hockey_2_predict");
  Hockey_predict hockey_predict;

  ros::spin();

  return 0;

}
