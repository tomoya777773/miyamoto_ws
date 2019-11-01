
//ros includes
#include "ros/ros.h"

#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/callback_queue.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//cpp includes
#include <vector>
#include <string.h>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <ctime>

//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>






#include <iostream>
#include <fstream>
using namespace std;

/**
 * This tutorial demonstrates simple re
 * ceipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
ros::Publisher pointcloud2_publisher;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

int image_folder = 0;
int writeCount = 0;
int state;


// convert rgb to hsv
/* IN: rgb Ptr
 * OUT: hsv Ptr
 * */
pcl::PointCloud<pcl::PointXYZHSV>::Ptr rgb2hsv(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& rgbPtr)
{
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvPtr(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloudXYZRGBtoXYZHSV(*rgbPtr, *hsvPtr);
        hsvPtr->header = rgbPtr->header;

        //hsvPtr->
        //rgbPtr->
        return(hsvPtr);
}


///* passThrough *///
template <typename POINTTYP>
class ptClass
{
        std::string 			        FieldName;
        float 					MinValue;
        float					MaxValue;
        //pcl::PointIndices::Ptr 	IndexPtrOutput;  //output index

        typename pcl::PassThrough<POINTTYP> Ptfilter;

public:
        ptClass(const std::string &fieldName,const float minValue, const float maxValue){
                FieldName = fieldName;
                MinValue = minValue;
                MaxValue = maxValue;

        //	pcl::PointIndices::Ptr indexPtrtmp(new(pcl::PointIndices));
        //	IndexPtrOutput = indexPtrtmp;

                typename pcl::PassThrough<POINTTYP> ptfiltertmp(true);
                Ptfilter = ptfiltertmp;
        }

        pcl::PointIndices::Ptr
        ptFun(const typename pcl::PointCloud<POINTTYP>::Ptr &inputPtr)
        {
                 pcl::PointIndices::Ptr indexPtrOutput(new(pcl::PointIndices));
                 Ptfilter.setInputCloud (inputPtr);
                 Ptfilter.setFilterFieldName (FieldName);
                 Ptfilter.setFilterLimits (MinValue, MaxValue);
                 Ptfilter.filter (indexPtrOutput->indices);

                 return(indexPtrOutput);
        }
};

template <typename POINTTYP>
typename pcl::PointCloud<POINTTYP>::Ptr
extract(const typename pcl::PointCloud<POINTTYP>::Ptr pclInput, const pcl::PointIndices::Ptr indexPtrInput, bool remove){
          // Extract index point
        typename pcl::PointCloud<POINTTYP>::Ptr pclPtrOutput(new typename pcl::PointCloud<POINTTYP>);
        typename pcl::ExtractIndices<POINTTYP> extract;
    extract.setInputCloud (pclInput);
    extract.setIndices (indexPtrInput);

    extract.setNegative (remove);
    extract.filter (*pclPtrOutput);
    return(pclPtrOutput);


}


pcl::PointIndices::Ptr
indexCom(std::vector<pcl::PointIndices::Ptr> indexPtrVec){
         // sort index
         for(std::vector<pcl::PointIndices::Ptr>::iterator it= indexPtrVec.begin(); it != indexPtrVec.end(); ++it){
                 std::sort((*it)->indices.begin(),(*it)->indices.end());
         }

         // initial
         std::vector<int> indicesTmp,indicesSaveTmp;
         //indicesTmp.swap((*indexPtrVec.begin())->indices);
         indicesTmp=((*indexPtrVec.begin())->indices);

         // test
        // int i = 0;
         for(std::vector<pcl::PointIndices::Ptr>::iterator it= indexPtrVec.begin()+1; it != indexPtrVec.end(); ++it){
                 std::set_intersection((*it)->indices.begin(), (*it)->indices.end(),
                                 indicesTmp.begin(),indicesTmp.end(),std::back_inserter(indicesSaveTmp));

                 indicesTmp = indicesSaveTmp;
                 indicesSaveTmp.clear();
                 // test
                 //i++;
                 //printf("in ite %d size is %d\n",i,indicesTmp.size());
         }

         //return result
          pcl::PointIndices::Ptr indexPtrCom(new(pcl::PointIndices));
         // indexPtrCom->indices.swap(indicesTmp);
          indexPtrCom->indices= indicesTmp;
          return(indexPtrCom);
}
// float hMin = 0;          //degree
// float hMax = 360;
// float sMin = 0.0;          //percent
// float sMax = 10;
// float vMin = 0;          //percent
// float vMax = 10;

float hMin = 30;          //degree
float hMax = 60;
float sMin = 0.0;          //percent
float sMax = 0.8;
float vMin = 0.4;          //percent
float vMax = 1;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr hsvFilterFun(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbPtrInput,bool remove)
{
    
 
    /** Convert rgb to hsv **/
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvpclPtr;
    hsvpclPtr = rgb2hsv(rgbPtrInput);

    /** perform H S V passthrough filter separately to get index **/
    // H
    pcl::PointIndices::Ptr H_indexPtr(new pcl::PointIndices);
    ptClass<pcl::PointXYZHSV>H_ptfilter("h",hMin,hMax);
    H_indexPtr = H_ptfilter.ptFun(hsvpclPtr);

    // S
    pcl::PointIndices::Ptr S_indexPtr(new pcl::PointIndices);
    ptClass<pcl::PointXYZHSV>S_ptfilter("s",sMin,sMax);
    S_indexPtr = S_ptfilter.ptFun(hsvpclPtr);

    //V
    pcl::PointIndices::Ptr V_indexPtr(new pcl::PointIndices);
    ptClass<pcl::PointXYZHSV>V_ptfilter("v",vMin,vMax);
    V_indexPtr = V_ptfilter.ptFun(hsvpclPtr);





    pcl::PointIndices::Ptr indexVecArry[] = {H_indexPtr,S_indexPtr,V_indexPtr};

    std::vector<pcl::PointIndices::Ptr> indexVec(indexVecArry,indexVecArry+3);
    pcl::PointIndices::Ptr indexPtr = indexCom(indexVec);


    // Extract index point
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbPtrOutput;
    rgbPtrOutput = extract<pcl::PointXYZRGB>(rgbPtrInput,indexPtr,false);
    return(rgbPtrOutput);
}

// convert msg to pcl
/*IN: pointcloud2 ros message (const sensor_msgs::PointCloud2)
 * OUT: pcl data
 * */
template <typename PCL>
typename pcl::PointCloud<PCL>::Ptr
msg2pcl(const sensor_msgs::PointCloud2 & cloud_msg)
{
        typename pcl::PointCloud<PCL>::Ptr pointCloudPtr(new typename pcl::PointCloud<PCL>);
        pcl::fromROSMsg(cloud_msg,*pointCloudPtr);
        return(pointCloudPtr);
}

template <typename PCL>
typename pcl::PointCloud<PCL>::Ptr
msg2pcl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
        typename pcl::PointCloud<PCL>::Ptr pointCloudPtr(new typename pcl::PointCloud<PCL>);
        pcl::fromROSMsg(*cloud_msg,*pointCloudPtr);
        return(pointCloudPtr);
}


template <typename PCL>
sensor_msgs::PointCloud2
pcl2msg(const typename pcl::PointCloud<PCL>::Ptr& pointClouPtr)
{
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*pointClouPtr,output);
        return(output);
}

template <typename PCL>
sensor_msgs::PointCloud2
pcl2msg(const typename pcl::PointCloud<PCL> pointCloud)
{
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(pointCloud,output);
        return(output);
}



pcl::PointCloud<pcl::PointXYZRGB> crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbFilterPtr )
{
    // Eigen::Vector4f minPoint;
    // minPoint[0] = 0.15; // define minimum point x
    // minPoint[1] = -0.5; // define minimum point y
    // minPoint[2] = -0.5; // define minimum point z
    // Eigen::Vector4f maxPoint;
    // maxPoint[0] = 1; // define max point x
    // maxPoint[1] = 1; // define max point y
    // maxPoint[2] = 0.1; // define max point z
  
    // Eigen::Vector3f boxTranslatation;
    // boxTranslatation[0] = -0.03;
    // boxTranslatation[1] = 0.2;
    // boxTranslatation[2] = 0.5;

    // Eigen::Vector3f boxRotation;
    // boxRotation[0] = 70; // rotation around x-axis
    // boxRotation[1] = 30; // rotation around y-axis
    // boxRotation[2] = 40; //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    Eigen::Vector4f minPoint;
    minPoint[0] = -0.3; // define minimum point x
    minPoint[1] = -0.2; // define minimum point y
    minPoint[2] = 0.14; // define minimum point z
    Eigen::Vector4f maxPoint;
    maxPoint[0] = 0.1; // define max point x
    maxPoint[1] = 1; // define max point y
    maxPoint[2] = 0.5; // define max point z
  
    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0] = -0.03;
    boxTranslatation[1] = 0.1;
    boxTranslatation[2] = 0.7;

    Eigen::Vector3f boxRotation;
    boxRotation[0] = 0; // rotation around x-axis
    boxRotation[1] = 0; // rotation around y-axis
    boxRotation[2] = 0; //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    pcl::CropBox<pcl::PointXYZRGB> crop;

    crop.setInputCloud(rgbFilterPtr);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setTranslation(boxTranslatation);
    crop.setRotation(boxRotation);

    pcl::PointCloud<pcl::PointXYZRGB> cropOutput ;
    crop.filter(cropOutput);
    return cropOutput;
}

pcl::PointCloud<pcl::PointXYZRGB> cropforVoxels(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbFilterPtr )
{  
    Eigen::Vector4f minPoint;
    minPoint[0] = -0.3; // define minimum point x
    minPoint[1] = -0.2; // define minimum point y
    minPoint[2] = 0.14; // define minimum point z
    Eigen::Vector4f maxPoint;
    maxPoint[0] = 0.1; // define max point x
    maxPoint[1] = 1; // define max point y
    maxPoint[2] = 0.5; // define max point z
  
    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0] = -0.03;
    boxTranslatation[1] = 0.1;
    boxTranslatation[2] = 0.7;


     Eigen::Vector3f boxRotation;
    boxRotation[0] = 0; // rotation around x-axis
    boxRotation[1] = 0; // rotation around y-axis
    boxRotation[2] = 0; //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    // Eigen::Vector4f minPoint;
    // minPoint[0] = 0.15; // define minimum point x
    // minPoint[1] = -0.5; // define minimum point y
    // minPoint[2] = -0.2; // define minimum point z
    // Eigen::Vector4f maxPoint;
    // maxPoint[0] = 0.5; // define max point x
    // maxPoint[1] = 1; // define max point y
    // maxPoint[2] = 1; // define max point z
  
    // Eigen::Vector3f boxTranslatation;
    // boxTranslatation[0] = -0.03;
    // boxTranslatation[1] = 0.1;
    // boxTranslatation[2] = 0.7;

    // Eigen::Vector3f boxRotation;
    // boxRotation[0] = 70; // rotation around x-axis
    // boxRotation[1] = 30; // rotation around y-axis
    // boxRotation[2] = 40; //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    // Eigen::Vector4f minPoint;
    // minPoint[0] = -0.2; // define minimum point x
    // minPoint[1] = -0.2; // define minimum point y
    // minPoint[2] = 0; // define minimum point z
    // Eigen::Vector4f maxPoint;
    // maxPoint[0] = 0.1; // define max point x
    // maxPoint[1] = 0.5; // define max point y
    // maxPoint[2] = 0.3; // define max point z
  
    // Eigen::Vector3f boxTranslatation;
    // boxTranslatation[0] = -0.05;
    // boxTranslatation[1] = 0.1;
    // boxTranslatation[2] = 0.7;


    //  Eigen::Vector3f boxRotation;
    // boxRotation[0] = 0; // rotation around x-axis
    // boxRotation[1] = 0; // rotation around y-axis
    // boxRotation[2] = 0; //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    pcl::CropBox<pcl::PointXYZRGB> crop;

    crop.setInputCloud(rgbFilterPtr);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setTranslation(boxTranslatation);
    crop.setRotation(boxRotation);

    pcl::PointCloud<pcl::PointXYZRGB> cropOutput ;
    crop.filter(cropOutput);
    return cropOutput ;
}


void callback_deep(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  
    ros::Time begin = ros::Time::now();

    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl_conversions::toPCL(*cloud_msg, *cloud);     //converts incomeing Pointer of Sensor Msg to PCLPointCloud2 format 



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbFilterPtr;//pclFilter;
    /**  Convert to rgb PCL data type **/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbpclPtr;

    rgbpclPtr = msg2pcl<pcl::PointXYZRGB>(cloud_msg);
    
    /** initial for convinience **/
    rgbFilterPtr = rgbpclPtr;

    rgbFilterPtr = hsvFilterFun(rgbpclPtr,true);    //hsv Filter funktioniert 
                                                    //dannach Box machen
    pcl::PointCloud<pcl::PointXYZRGB> cropFilter;//pclFilter;  
    
    cropFilter = crop(rgbFilterPtr);  

    
    sensor_msgs::PointCloud2 output2;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropFilterPtr; 
    cropFilterPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB> (cropFilter));



    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cropFilterPtr);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);


    ne.compute (*cloud_normals); 
    
    
    

    //pcl::PointCloud<pcl::PointXYZRGB> m_copy2 = cropFilter;
    //pcl::PointCloud<pcl::PointXYZRGB> out;
 

    pcl::PointCloud<pcl::PointXYZRGB> cropForVoxels; 
    
    
    //std::cout <<"cropFilterPtr"<<*cropFilterPtr<< std::endl;
    cropForVoxels = cropforVoxels(cropFilterPtr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropForVoxelsPtr; 
    cropForVoxelsPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB> (cropForVoxels));
    
    //std::cout <<"cropForVoxelsPtr"<< *cropForVoxelsPtr<< std::endl;




    pcl::PointCloud<pcl::PointXYZRGB> voxelsForTouchpoints;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelsForTouchpointsPtr; 
    voxelsForTouchpointsPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB> (voxelsForTouchpoints));

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;

    sor.setInputCloud (cropForVoxelsPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter ( voxelsForTouchpoints);

    

    // std::cout << "transformed_cloud-> "<< voxelsForTouchpoints.width;
    for(int point_num = 0; point_num < voxelsForTouchpoints.size(); point_num ++){
        voxelsForTouchpoints.points[point_num].x += 0.30;
        voxelsForTouchpoints.points[point_num].y += 0.12;
        voxelsForTouchpoints.points[point_num].z -= 0.6;
        // std::cout << "point_num: " << point_num << " Y: " << voxelsForTouchpoints.points[point_num].y << std::endl;
    }

   
    

    
    /***********************************************************************************/


    /* Reminder: how transformation matrices work :

        |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error pronze !
    */
    Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_x2 = Eigen::Matrix4f::Identity();

    float theta_x = -2.15*M_PI/3; // The angle of rotation in radians
    transform_x (1,1) = cos (theta_x);
    transform_x (1,2) = -sin(theta_x);
    transform_x (2,1) = sin (theta_x);
    transform_x (2,2) = cos (theta_x);

    transform_x (2,3) = 0.5;

    float theta_y = -3*M_PI/180; // The angle of rotation in radians
    transform_y (0,0) = cos (theta_y);
    transform_y (0,2) = sin (theta_y);
    transform_y (2,0) = -sin(theta_y);
    transform_y (2,2) = cos (theta_y);

    float theta_z = 0.9*M_PI/4; // The angle of rotation in radians
    transform_z (0,0) = cos (theta_z);
    transform_z (0,1) = -sin(theta_z);
    transform_z (1,0) = sin (theta_z);
    transform_z (1,1) = cos (theta_z);
    //    (row, column)

    transform_z (0,3) = 0.3;
    transform_z (1,3) = 0.02;
    transform_z (2,3) = 0;

    float theta_x2 = 3*M_PI/180; // The angle of rotation in radians
    transform_x2 (1,1) = cos (theta_x2);
    transform_x2 (1,2) = -sin(theta_x2);
    transform_x2 (2,1) = sin (theta_x2);
    transform_x2 (2,2) = cos (theta_x2);

    // Define a translation of 2.5 meters on the x axis.
    // transform_1 (0,3) = 0.8;
    // transform_1 (1,3) = 0.12;
    // transform_1 (2,3) = -0.8;
    // transform_1 (3,3) = -1;





 

    pcl::transformPointCloud (voxelsForTouchpoints, voxelsForTouchpoints, transform_x);
    pcl::transformPointCloud (voxelsForTouchpoints, voxelsForTouchpoints, transform_y);
    pcl::transformPointCloud (voxelsForTouchpoints, voxelsForTouchpoints, transform_z);
    pcl::transformPointCloud (voxelsForTouchpoints, voxelsForTouchpoints, transform_x2);

    output2 =pcl2msg<pcl::PointXYZRGB>(voxelsForTouchpoints);





    pointcloud2_publisher.publish(output2);
    std::cout << "PCL2msg"<< std::endl;
}

void callback_record_state(std_msgs::Int32 state)
{
    // %EndTag(CALLBACK)%
    //std::cout << state << std::endl;
    image_folder = state.data;
    writeCount = 0;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "PCL");

    ros::NodeHandle n;
    std::cout << "PCL extract touching points" << std::endl;
    ros::Subscriber pointcloud2_subscriber = n.subscribe("/kinect2/sd/points", 5, callback_deep);

    pointcloud2_publisher = n.advertise<sensor_msgs::PointCloud2>("output", 1);


    ros::Rate r(100);
    std::cout << "Loopstart" << std::endl;
    
    int i=0;
    ros::spin();
    /*
    while (ros::ok())
    {
                //only if using a MESH_RESOURCE marker type:
       
        if (c == 'a')
        {
            hMin =+ i;
            std::cout << "hu_min=" << hMin << std::endl;    

        }
        
        //std::cin >> i;
        std::cout << "running"<< std::endl;
        char a = getchar();
        if ('q' == a)
            hMax ++;
        if ('a' == a)
            hMax --;

        if ('w' == a)
            hMin ++;
        if ('s' == a)
            hMin --;


        if ('e' == a)
            sMax = sMax + 0.02;
        if ('d' == a)
            sMax =  sMax - 0.02;
        if ('r' == a)
            sMin = sMin + 0.02;
        if ('f' == a)
            sMin =  sMin - 0.02;

        if ('t' == a)
            vMax = vMax+ 0.02;
        if ('g' == a)
            vMax = vMax- 0.02;
        if ('y' == a)
            vMin = vMin+ 0.02;
        if ('h' == a)
            vMin = vMin- 0.02;

        std::cout << "h=" << hMin<<"-" <<hMax<< std::endl;
        std::cout << "s=" << sMin<<"-" <<sMax<< std::endl;
        std::cout << "v=" << vMin<<"-" <<vMax<< std::endl<<std::endl;

        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1));
      
       

    }
    */
    std::cout << "ros is not okay"<< std::endl;
    return 0;
}

// %EndTag(FULLTEXT)%