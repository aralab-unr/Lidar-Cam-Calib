#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_correntropy_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/uniform_sampling.h>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>

#include <opencv2/opencv.hpp>

#include<vector>
using namespace cv;
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTColor;
typedef pcl::PointCloud<PointTColor> PointCloudTColor;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;
 float im_a=0.0,im_b=0.0,im_c=0.0,im_d=0.0;
float im_minx=999999,im_miny=999999,im_minz=999999;
float im_maxx=-999999,im_maxy=-999999,im_maxz=-999999;

int imidx=0;

int li_ctr_z=0;
int li_ctr_y=0;
 int nop_y=8;
 int nop_z=10;
 int nop_x=11;
int multiplier=1;
float st_fact_y=0;
float st_fact_z=0;
float st_fact_x=0;

float li_fact_y=0;
float li_fact_z=0;

float lmin_y=0;
float lmin_z=0;

float imlmin_y=0;
float imlmin_z=0;

std::vector<int> ridx;
std::vector<float> dlx;
std::vector<float> li3dpointsz;
 PointCloudT::Ptr cloud_from_image (new PointCloudT);  // Point cloud generated from Image.
 PointCloudT::Ptr min_max_li (new PointCloudT);  // Minmax points from lidar.
 PointCloudTColor::Ptr cam_color (new PointCloudTColor);  // Minmax points from lidar.
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_res(new pcl::visualization::PCLVisualizer("viewer_res"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_in(new pcl::visualization::PCLVisualizer("viewer_in"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_icp(new pcl::visualization::PCLVisualizer("viewer_icp"));

  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  PointCloudT::Ptr cloud_selected_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_selected_icp (new PointCloudT);  // ICP output point cloud

  PointCloudT::Ptr cloud_ransac_ran_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_ransac_ran_icp (new PointCloudT);  // ICP output point cloud

  PointCloudT::Ptr cloud_selected_in1 (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_selected_icp1 (new PointCloudT);  // ICP output point cloud

 

  
  PointCloudT::Ptr cloud_f_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_f_icp (new PointCloudT);  // ICP output point cloud

  PointCloudT::Ptr lidarp (new PointCloudT);  // Original point cloud

  
   int v1 (0);
  int v2 (1);
  int v3 (3);

   int num_in=0;
   int num_icp=0;

 float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

class Ve 
{
public:
    void ReadImage(cv::String pattern);
    //double checkCameraPose(std::vector<cv::Point3d> &modelPts, std::vector<cv::Point2f> &imagePts,cv::Mat &cameraMatrix,
      //                 cv::Mat &distCoeffs, cv::Mat &rvec,cv::Mat &tvec);
};

double checkCameraPose(std::vector<cv::Point3d> &modelPts, std::vector<cv::Point2f> &imagePts, cv::Mat &cameraMatrix,
                       cv::Mat &distCoeffs, cv::Mat &rvec, cv::Mat &tvec) {
  std::vector<cv::Point2d> projectedPts;
  cv::projectPoints(modelPts, rvec, tvec, cameraMatrix, distCoeffs, projectedPts);

  double rms = 0.0;
  std::cout<<"\nprojectedPts.size="<<projectedPts.size();
  for (size_t i = 0; i < projectedPts.size(); i++) {
    rms += (projectedPts[i].x-imagePts[i].x)*(projectedPts[i].x-imagePts[i].x) + (projectedPts[i].y-imagePts[i].y)*(projectedPts[i].y-imagePts[i].y);
  }

  return sqrt(rms / projectedPts.size());
}


cv::Point3f transformPoint( cv::Point3d &pt, cv::Mat &rvec, cv::Mat &tvec) {
  //Compute res = (R | T) . pt
  cv::Mat rotationMatrix;
  cv::Rodrigues(rvec, rotationMatrix);
  

  cv::Mat transformationMatrix = (cv::Mat_<double>(4, 4) << rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2), tvec.at<double>(0),
                                  rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2), tvec.at<double>(1),
                                  rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2), tvec.at<double>(2),
                                  0, 0, 0, 1);


  cv::Mat homogeneousPt = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
 
  cv::Mat transformedPtMat = transformationMatrix * homogeneousPt;


  cv::Point3f transformedPt(transformedPtMat.at<double>(0), transformedPtMat.at<double>(1), transformedPtMat.at<double>(2));
  

  return transformedPt;
}

void computePlaneEquation( cv::Point3f &p0, cv::Point3f &p1, cv::Point3f &p2, float &a, float &b, float &c, float &d) {
  //Vector p0_p1
  cv::Point3f p0_p1;
  p0_p1.x = p0.x - p1.x;
  p0_p1.y = p0.y - p1.y;
  p0_p1.z = p0.z - p1.z;

  //Vector p0_p2
  cv::Point3f p0_p2;
  p0_p2.x = p0.x - p2.x;
  p0_p2.y = p0.y - p2.y;
  p0_p2.z = p0.z - p2.z;

  //Normal vector
  cv::Point3f n = p0_p1.cross(p0_p2);

  a = n.x;
  b = n.y;
  c = n.z;
  d = -(a*p0.x + b*p0.y + c*p0.z);

  float norm =  sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
  d /= norm;
}

cv::Point3f transformPointInverse(cv::Point3f &pt,  cv::Mat &rvec, cv::Mat &tvec) {
  //Compute res = (R^t | -R^t . T) . pt
  cv::Mat rotationMatrix;
  cv::Rodrigues(rvec, rotationMatrix);
  rotationMatrix = rotationMatrix.t();

  cv::Mat translation = -rotationMatrix*tvec;

  cv::Mat transformationMatrix = (cv::Mat_<double>(4, 4) << rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2), translation.at<double>(0),
                                  rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2), translation.at<double>(1),
                                  rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2), translation.at<double>(2),
                                  0, 0, 0, 1);

  cv::Mat homogeneousPt = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
  cv::Mat transformedPtMat = transformationMatrix * homogeneousPt;

  cv::Point3f transformedPt(transformedPtMat.at<double>(0), transformedPtMat.at<double>(1), transformedPtMat.at<double>(2));
  return transformedPt;
}

cv::Point3f compute3DOnPlaneFrom2D(cv::Point2f &imagePt,cv::Mat &cameraMatrix, float a, float b, float c, float d) {
  double fx = cameraMatrix.at<double>(0,0);
  double fy = cameraMatrix.at<double>(1,1);
  double cx = cameraMatrix.at<double>(0,2);
  double cy = cameraMatrix.at<double>(1,2);

  cv::Point2f normalizedImagePt;
  normalizedImagePt.x = (imagePt.x - cx) / fx;
  normalizedImagePt.y = (imagePt.y - cy) / fy;

  float s = -d / (a*normalizedImagePt.x + b*normalizedImagePt.y + c);

  cv::Point3f pt;
  pt.x = s*normalizedImagePt.x;
  pt.y = s*normalizedImagePt.y;
  pt.z = s;

  return pt;
}
float roundF(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

void Ve::ReadImage(cv::String pattern) 
{
  std::cout<<"\n ........................IMAGE COMPUTATION................................................................\n";
    vector<cv::String> fn;
    glob(pattern, fn, false);
    //vector<Mat> images;
    size_t count = fn.size(); //number of png files in images folder
    for(int i=0;i<count;i++) std::cout<<"\n"<<fn[i]<<"-->"<<i<<"\n";
    //for (size_t i = 4;i <4; i++)
    {
        Mat view=cv::imread(fn[imidx]);
       // images.push_back(view);

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
  
    Mat camera_matrix = (cv::Mat_<double>(3,3) <<  762.72493,0.00000,640.50000,0.00000,762.72493,360.50000,0.00000,0.00000,1.00000);
    Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
    
  std::vector<Point2f> corners1;// for chessboard pattern
  Mat rotation_vector; // Rotation in axis-angle form
    Mat translation_vector;
    Mat extrinsicMatrix,projectionMatrix, homographyMatrix, inverseHomographyMatrix;

//  VideoCapture capture(0);
  
  Mat Img;
  
  vector<Point3d>  objectPoints;
  
  const string x= "X";
  const string y= "Y";
  const string z= "Z";
  bool found=false;
  vector<Point3d> point3D;
    vector<Point2d> point2D;

  int ChessboardPatternWidth=5;//Horrizonal Number of internal corners of pattern //change it accrounding to your pattern 
  int ChessboardPatternHight=7;//vertical Number of internal corners of pattern //change it accrounding to your pattern 
  Size patternSize(ChessboardPatternWidth,ChessboardPatternHight);
  float BoardBoxSize=0.108;//distance between 2 correns //change it accrounding to your pattern . megger it in cm or mm.
                       //your unit of meggerment will consider as object point units.

  std::cout<<"\nimlmin_y="<<imlmin_y<<" imlmin_z="<<imlmin_z;
  std::cout<<"\nli3dpointsz.size="<< li3dpointsz.size();
  int li3ctr=li3dpointsz.size();
  for (int i=0; i<patternSize.height;i++)
   {
     for( int j=0; j < patternSize.width;j++)
   {
     objectPoints.push_back(Point3f((i*BoardBoxSize),(j*BoardBoxSize),0));
     li3ctr--;
     }
   }
  // objectPoints.push_back(Point3f(-378,-270,0));
 
   //objectPoints.push_back(Point3f(-378+BoardBoxSize,-270+BoardBoxSize,0));
 
  // //elow are the 3d object point(world point) to drow x , y z axis.
//  point3D.push_back(Point3d(0,0,-500.0)); //-z this point represents 10( cm or mm accrounding to BoardBoxSize unit  ) 
//  point3D.push_back(Point3d(500.0,0,0));  //x
 // point3D.push_back(Point3d(0,500.0,0));  //y
  
  // ////elow are the 3d object point(world point) to drow Box.
  // point3D.push_back(Point3d(12, 0,-12.0));//(x,y,z)
  // point3D.push_back(Point3d(12, 8,-12.0));
  // point3D.push_back(Point3d(20, 8,-12.0));
  // point3D.push_back(Point3d(20, 0,-12.0));

  //while(1)

    //view=calib_image;

    
  if(view.empty()!=1)
  { 
  found=findChessboardCorners(view, patternSize,  corners1);//This will detect pattern
  }  

  if(found )

    {
      cvtColor(view,Img,COLOR_BGR2GRAY);
    
    cornerSubPix(Img, corners1, Size(11, 11), Size(-1, -1),
    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 300, 0.01));
    
      drawChessboardCorners(view, patternSize, corners1, found);
      
    // The following are two important funtion
    cv::solvePnP(objectPoints, corners1, camera_matrix, dist_coeffs, rotation_vector, translation_vector,false,cv::SOLVEPNP_ITERATIVE);//Gives you rotation_vector, translation_vector
     //Check camera pose
   double rms = checkCameraPose(objectPoints, corners1, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    cv::Mat rot_mat =cv::Mat::zeros(3, 1, CV_64FC1);;
    //cv::Vec3f trvec((float*)translation_vector.data);
    //cv::Vec3f rotvec((float*)rotation_vector.data);
    cv::Mat tm=cv::Mat::zeros(4,4, CV_64FC1);

   cv::Rodrigues(rotation_vector,rot_mat);

   tm.at<double>(0,0)=rot_mat.at<double>(0,0); tm.at<double>(0,1)=rot_mat.at<double>(0,1); tm.at<double>(0,2)=rot_mat.at<double>(0,2);tm.at<double>(0,3)=translation_vector.at<double>(0,0);
   tm.at<double>(1,0)=rot_mat.at<double>(1,0); tm.at<double>(1,1)=rot_mat.at<double>(1,1); tm.at<double>(1,2)=rot_mat.at<double>(1,2);tm.at<double>(1,3)=translation_vector.at<double>(0,1);
   tm.at<double>(2,0)=rot_mat.at<double>(2,0); tm.at<double>(2,1)=rot_mat.at<double>(2,1); tm.at<double>(1,2)=rot_mat.at<double>(2,2);tm.at<double>(2,3)=translation_vector.at<double>(0,2);
   tm.at<double>(3,0)=0; tm.at<double>(3,1)=0; tm.at<double>(3,2)=0;tm.at<double>(3,3)=1;
     // cv::Affine3f tm(rot_mat,trvec);  
   std::cout<<"\nTransformation Matrix=" << tm;
   std::cout<< "RMS error for camera pose=" << rms << std::endl;
   std::cout<<"\n Rotation vector="<<rotation_vector;
   std::cout<<"\n Rotation Matrix="<<rot_mat;
   std::cout<<"\n translation_vector="<<translation_vector;
   
    //Transform model point (in object frame) to the camera frame
   cv::Point3f pt0 = transformPoint(objectPoints[1], rotation_vector, translation_vector);
   cv::Point3f pt1 = transformPoint(objectPoints[5], rotation_vector, translation_vector);
   cv::Point3f pt2 = transformPoint(objectPoints[4], rotation_vector, translation_vector);
   
   float a, b, c, d;
  computePlaneEquation(pt0, pt1, pt2, a, b, c, d);
  std::cout << "Plane equation=" << a << " ; " << b << " ; " << c << " ; " << d << std::endl;
 
  //Compute 3D from 2D
  std::vector<cv::Point3f> pts3dCameraFrame, pts3dObjectFrame;
  double rms_3D = 0.0;
  im_minx=999999,im_miny=999999,im_minz=999999;
   im_maxx=-999999,im_maxy=-999999,im_maxz=-999999;


  for (size_t i = 0; i < corners1.size(); i++) 
  {
    cv::Point3f pt = compute3DOnPlaneFrom2D(corners1[i], camera_matrix, a, b, c, d);
    std::cout<<"\npt="<<pt;
    std::cout<<"\nCorners="<<corners1[i];
    pts3dCameraFrame.push_back(pt);

    cv::Point3f ptObjectFrame = transformPointInverse(pt, rotation_vector, translation_vector);
    std::cout<<"\nptobjectframe="<<ptObjectFrame;
    pts3dObjectFrame.push_back(ptObjectFrame);


    if(pt.x<im_minx) im_minx=pt.x;
    if(pt.y<im_miny) im_miny=pt.y;
    if(pt.z<im_minz) im_minz=pt.z;

    if(pt.x>im_maxx) im_maxx=pt.x;
    if(pt.y>im_maxy) im_maxy=pt.y;
    if(pt.z>im_maxz) im_maxz=pt.z;


    rms_3D += (objectPoints[i].x-ptObjectFrame.x)*(objectPoints[i].x-ptObjectFrame.x) + (objectPoints[i].y-ptObjectFrame.y)*(objectPoints[i].y-ptObjectFrame.y) +
        (objectPoints[i].z-ptObjectFrame.z)*(objectPoints[i].z-ptObjectFrame.z);

    std::cout << "modelPts[" << i << "]=" << objectPoints[i] << " ; calc=" << ptObjectFrame << std::endl;
  }



im_a=a;im_b=b;im_c=c;im_d=d;
   a=-im_c;
   b=im_a;
   c=im_b;
   d=-im_d;



 std::vector<Point2f> campts;// for chessboard pattern
  cam_color->width=view.rows*view.cols;
  cam_color->height=1;
  cam_color->resize(view.rows*view.cols*1);
  int dctr=0;

  for (size_t i = 0; i < view.rows; i++) 
  {
    for(size_t j=0;j<view.cols;j++)
    {
      cv::Point2f imgpt;
      imgpt.x=j;
      imgpt.y=i;

       cv::Point3f pt = compute3DOnPlaneFrom2D(imgpt, camera_matrix, a, b, c, d);
       cam_color->points[dctr].x=pt.x;
       cam_color->points[dctr].y=pt.y;
       cam_color->points[dctr].z=pt.z;
       
            cv::Vec3b intensity = view.at<Vec3b>(j, i);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            std::uint32_t rgb = ((std::uint32_t)red << 16 | (std::uint32_t)green << 8 | (std::uint32_t)blue);
           cam_color->points[dctr].r=red;
           cam_color->points[dctr].g=green;
           cam_color->points[dctr].b=blue;

       dctr++;
       

    }
  }
  
   


  float dim_maxx=im_maxx,dim_maxy=im_maxy,dim_maxz=im_maxz;
  float dim_minx=im_minx,dim_miny=im_miny,dim_minz=im_minz;

  im_maxx=dim_maxx;
  im_maxy=dim_maxy;
  im_maxz=dim_maxz;


  im_minx=dim_minx;
  im_miny=dim_miny;
  im_minz=dim_minz;

  std::cout<<"\nBefore dummy variable\n";
  std::cout << "Max imx: " << im_maxx<< std::endl;
  std::cout << "Max imy: " << im_maxy << std::endl;
  std::cout << "Max imz: " << im_maxz<< std::endl;
  std::cout << "Min imx: " << im_minx<< std::endl;
  std::cout << "Min imy: " << im_miny<< std::endl;
  std::cout << "Min imz: " << im_minz<< std::endl;
  

  im_maxx=-dim_maxz;
  im_maxy=dim_maxx;
  im_maxz=dim_maxy;


  im_minx=-dim_minz;
  im_miny=dim_minx;
  im_minz=dim_miny;

  std::cout<<"\nAfter dummy variable Step1\n";
  std::cout << "Max imx: " << im_maxx<< std::endl;
  std::cout << "Max imy: " << im_maxy<< std::endl;
  std::cout << "Max imz: " << im_maxz<< std::endl;
  std::cout << "Min imx: " << im_minx<< std::endl;
  std::cout << "Min imy: " << im_miny<< std::endl;
  std::cout << "Min imz: " << im_minz<< std::endl;

  // im_maxx=(-dim_maxz/1000)+0.108;
  // im_maxy=(dim_maxx/1000)+0.0108;
  // im_maxz=dim_maxy/1000;


  // im_minx=(-dim_minz);
 // im_miny=((dim_miny));
 // im_minz=((dim_minz));

  //im_miny=dim_miny;
  //im_minz=dim_minz;

  std::cout<<"\nAfter dummy variable\n";
  std::cout << "Max imx: " << im_maxx<< std::endl;
  std::cout << "Max imy: " << im_maxy << std::endl;
  std::cout << "Max imz: " << im_maxz<< std::endl;
  std::cout << "Min imx: " << im_minx<< std::endl;
  std::cout << "Min imy: " << im_miny<< std::endl;
  std::cout << "Min imz: " << im_minz<< std::endl;
  std::cout << "RMS error for model points=" << sqrt(rms_3D / corners1.size()) << std::endl;

  std::cout << "Final_Plane equation=" << a << " ; " << b << " ; " << c << " ; " << d << std::endl;



  ///////////////////add point cloud to the result//////////////////////////////////////////////////
  
float fact_y=roundF((im_maxy-im_miny)/nop_y);
float fact_z=roundF((im_maxz-im_minz)/nop_z);
float fact_x=roundF((im_maxx-im_minx)/nop_x);



pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_from_image,255, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow_cl_im(cloud_from_image,0, 255, 0);
 

  cloud_from_image->width=nop_y*nop_z;
  cloud_from_image->height=1;
  cloud_from_image->resize(nop_y*nop_z*1);
 int ctr=0;
 int ctr_y=0;
 int ctr_z=0;
 //std::cout<<"\nFACTY="<<fact_y<<" FACTZ="<<fact_z;
 std::cerr << "Model coefficients for IMage plane: " << a << " " 
                                      << b << " "
                                      << c << " " 
                                      << d<< std::endl;  
   fact_y=0.108;
   fact_z=0.108;
  // miny=-2.0;
  // minz=-2.0;
//im_miny*=1000;
//im_minz*=1000;

std::cout<<"\nLI_FACTY="<<fact_y<<" LI_FACTZ="<<fact_z<<std::endl;
std::cout<<"\nLIminyY="<<im_miny<<" LIminZ="<<im_minz<<std::endl;

  for(float i=im_miny-(2*0.108*0);;i+=fact_y)
  {
    ctr_z=0;
    for(float j=im_minz+(2*0.108*0);;j+=fact_z)
    {
      float dz=0;
      std::vector<int>::iterator it;
      it=find(ridx.begin(),ridx.end(),ctr);
      ctr_z++;
      //  if(i==miny || j==miny || ctr_z==nop_z || ctr_y==nop_y)
      // {
      //  dz=0.1;
      // }

      if(it!=ridx.end())
      {
       dz=0.3;
      }
     cloud_from_image->points[ctr].y=i+dz;
     cloud_from_image->points[ctr].z=j+dz;
     cloud_from_image->points[ctr].x=((-1/(a))*((b*(i+dz))+(c*(j+dz))+d))+dz;//+(5*i*(exp(-i*i-j*j)));
    
    if(ctr==0) dlx.push_back(0.0);
    if(ctr>0)
    {
      dlx.push_back(cloud_from_image->points[ctr].x-cloud_from_image->points[ctr-1].x);
    //  std::cout<<"dlx["<<ctr<<"]="<<dlx[ctr-1]<<"\n";
    }

     //((-1/(a))*((b*i)+(c*j)+d))+dz;
     //d+(-j*exp(-i*i-j*j));
    // 
     // std::cerr << "\nPoints: " << cloud_selected_in1->points[ctr].x << " " 
     //                                  << cloud_selected_in1->points[ctr].y << " "
     //                                  << cloud_selected_in1->points[ctr].z << std::endl;  
      ctr++;
      
      if(ctr_z>=nop_z) break;

    }
    ctr_y++;
    if(ctr_y>=nop_y) break;
  }

    
  //  viewer_in->addPointCloud(cloud_from_image,yellow_cl_im, "image_pts");
    pcl::io::savePCDFile("/media/ashu/25EA6BA0225EEBFD/ashu_ws/CoSM_ICP_v2/src/test_st_calib.pcd", *cloud_from_image);
    //viewer_res->addPlane(coefficients, p[0], p[1], p[2], "box_plane");
    viewer_res->addPointCloud(cloud_from_image, red, "image_points");
     viewer_res->addPointCloud(cam_color,"calib_points");
    
    // for (int i = 0; i < cloud_selected_in->points.size(); i++)
    //     std::cout << cloud_selected_in->points[i].x << std::endl;
    // /std::cout << "cloud_in" << cloud_selected_in->points.size() << std::endl;
  ///////////////////////////////////////////////////////////////////////////////////////////////////

    // following are just drowing funtion to drow object on output image.

   //  //Tp drow x,y z axis on image.
    // cv::line(view,corners1[0], point2D[0], cv::Scalar(0,0,255), 3);//z
    //  cv::line(view,corners1[0], point2D[1], cv::Scalar(255,0,0), 3);//x
    //  cv::line(view,corners1[0], point2D[2], cv::Scalar(0,255,0), 3);//y
   
  
   //  putText(view, x, Point(point2D[1].x-10,point2D[1].y-10),FONT_HERSHEY_COMPLEX_SMALL,1, cv::Scalar(255,0,0),2 );
   //  putText(view, y, Point(point2D[2].x-10,point2D[2].y-10),FONT_HERSHEY_COMPLEX_SMALL,1, cv::Scalar(0,255,0),2 );
   //  putText(view, z, Point(point2D[0].x-10,point2D[0].y-10),FONT_HERSHEY_COMPLEX_SMALL,1, cv::Scalar(0,0,255),2 );
   //      // circle(view,point2D[0], 3, cv::Scalar(0,0,255), 4, 8,0);
   //      // circle(view,point2D[1], 3, cv::Scalar(255,0,0), 4, 8,0);
   //      // circle(view,point2D[2], 3, cv::Scalar(0,255,0), 4, 8,0);
  
   //  // To drow box on image. It will writen for pattern size 9,6.
   // //If you are using diffrent change corners1 point and point2D point. 

  //   cv::line(view,corners1[2], point2D[0], cv::Scalar(0,255,0),6);
  //   cv::line(view,corners1[7], point2D[0],cv::Scalar(0,0,255), 6);
  // cv::line(view,corners1[23], point2D[5],cv::Scalar(0,0,255), 6);
  //   cv::line(view,corners1[5],  point2D[6], cv::Scalar(0,0,255),6);
             
  //   cv::line(view,corners1[3],corners1[5], cv::Scalar(0,255,0),  6);
  //   cv::line(view,corners1[5],corners1[23],cv::Scalar(0,255,0) ,6);
  // cv::line(view,corners1[23],corners1[21],cv::Scalar(0,255,0),  6);
  //   cv::line(view,corners1[21],corners1[3],cv::Scalar(0,255,0),   6);
  
  // cv::line(view,point2D[3], point2D[4], cv::Scalar(255,0,0), 6);
  // cv::line(view,point2D[4], point2D[5], cv::Scalar(255,0,0), 6);
  // cv::line(view,point2D[5], point2D[6], cv::Scalar(255,0,0), 6); 
  // cv::line(view,point2D[3], point2D[6], cv::Scalar(255,0,0), 6);
      

    }
    imshow("img", view);
    waitKey();
        
    }

    std::cout<<"\n ........................END OF IMAGE COMPUTATION................................................................\n";
    
    //return images;
}
 

 
  

  
pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h1 (cloud_icp, 20, 200, 20);


namespace pcl
{
    template<> struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
            operator () (const PointXYZ &p) const
        {
            return p.z;
        }
    };
}

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void*)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

void loadFile(const char* fileName,pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  pcl::PolygonMesh mesh;
  
  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  else
    pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}
void ppp_callback_in(const pcl::visualization::AreaPickingEvent& event, void *args)

{
  std::cout<<"\n ........................LIDAR COMPUTATION................................................................\n";
    struct callback_args * data = (struct callback_args *)args;//Point cloud data & visualization window
    std::vector<int > indices;
    if (event.getPointsIndices(indices) == -1)
        return;
    for (size_t i = 0; i < indices.size(); i++)
    {
        cloud_selected_in->points.push_back(cloud_in->points.at(indices[i]));
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_selected_in, 255, 0, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_selected_in, 255, 0, 0);
    
    std::stringstream ss;
    std::string cloudName;
    ss << num_in++;
    ss >> cloudName;
    cloudName += "_cloudName";
    viewer_in->removePointCloud("in_clicked_points");
    viewer_in->addPointCloud(cloud_selected_in, red, "in_clicked_points");
   // viewer_icp->addPointCloud(cloud_selected_in,green,"");
    pcl::copyPointCloud(*cloud_selected_in,*lidarp);
    viewer_in->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "in_clicked_points");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h1 (cloud_selected_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
     
     float leaf_s=0.3;
       pcl::UniformSampling<PointT> sor;
       sor.setRadiusSearch (0.1);
       sor.setInputCloud (cloud_selected_in);
       
       sor.filter(*cloud_selected_in);
             
    //viewer_res->addPointCloud (cloud_selected_in, cloud_in_color_h1, "cloud_in_v1", v1);
       // for (int i = 0; i < cloud_selected_in->points.size(); i++)
       //   std::cout <<"\n["<< cloud_selected_in->points[i].x <<","
       //             <<cloud_selected_in->points[i].y<<","
       //             <<cloud_selected_in->points[i].z<<"]"<<std::endl;

    /// Perform RANSAC Plane detection
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_selected_in);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    //return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;  

  pcl::PointXYZ minPt, maxPt;       
  float a=coefficients->values[0],b=coefficients->values[1],c=coefficients->values[2],d=coefficients->values[3];
  pcl::getMinMax3D(*cloud_selected_in,minPt,maxPt);      
   std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z<< std::endl;
  
  float ydiff=maxPt.y-minPt.y;
  float zdiff=maxPt.z-minPt.z;
  int ctr=0;

 std::setprecision(1);
float maxy=maxPt.y;
float maxz=maxPt.z;
float maxx=maxPt.x;

float miny=minPt.y;
float minz=minPt.z;
float minx=minPt.x;

// maxy=roundF(maxy);
// maxz=roundF(maxz);
// maxx=roundF(maxx);

// miny=roundF(miny);
// minz=roundF(minz);
// minx=roundF(minx);


float fact_y=roundF((maxy-miny)/nop_y);
float fact_z=roundF((maxz-minz)/nop_z);
float fact_x=roundF((maxx-minx)/nop_x);

st_fact_y=fact_y;
st_fact_z=fact_z;
st_fact_x=fact_x;

lmin_y=miny;
lmin_z=minz;
 
std::cout<<"\nydiff="<<ydiff<<" zdiff="<<zdiff;
std::cout<<"\nvals="<<maxy<<" "<<maxz<<" "<<miny<<" "<<minz;
std::cout<<"\nfacts_y="<<fact_y<<" fact_z="<<fact_z;

// maxy=-1.3;
// maxz=0.5;

//miny=-2.0;
//minz=-0.5;

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_selected_in1,0, 255, 0);

min_max_li->width=1;
min_max_li->height=1;
min_max_li->resize(min_max_li->width*min_max_li->height);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> minmaxgreen(min_max_li,0, 255, 0);
 min_max_li->points[0].y=minPt.y;
 min_max_li->points[0].x=minPt.x;
 min_max_li->points[0].z=minPt.z;

 // min_max_li->points[1].y=maxPt.y;
 // min_max_li->points[1].x=maxPt.x;
 // min_max_li->points[1].z=maxPt.z;

 viewer_in->addPointCloud(min_max_li,minmaxgreen, "minmaxpts");
  //    // cloud_selected_in1->points[ctr].z=j;
  //    // cloud_selected_in1->points[ctr].x=(-1/(c))*((a*i)+(b*j)+d);


  // for(float i=miny;i<maxy;i+=fact_y)
  // {
  //   li_ctr_z=0;
  //   for(float j=minz;j<maxz;j+=fact_z)
  //   {
  //    // cloud_selected_in1->points[ctr].y=i;
  //    // cloud_selected_in1->points[ctr].z=j;
  //    // cloud_selected_in1->points[ctr].x=(-1/(c))*((a*i)+(b*j)+d);
  //     ctr++;
  //     li_ctr_z++;

  //   }

  //   li_ctr_y++;
  // }
  // std::cout<<"\nnop="<<ctr<<"["<<li_ctr_y<<","<<li_ctr_z<<"]";

  cloud_selected_in1->width=nop_y*nop_z;
  cloud_selected_in1->height=1;
  cloud_selected_in1->resize(nop_y*nop_z*1);
 ctr=0;
 int ctr_y=0;
 int ctr_z=0;
 //std::cout<<"\nFACTY="<<fact_y<<" FACTZ="<<fact_z;
 std::cerr << "Model coefficients for Lidar plane: " << a << " " 
                                      << b << " "
                                      << c << " " 
                                      << d<< std::endl;  
   fact_y=0.108;
   fact_z=0.108;
  // miny=-2.0;
  // minz=-2.0;

std::cout<<"\nLI_FACTY="<<fact_y<<" LI_FACTZ="<<fact_z<<std::endl;
std::cout<<"\nLIminyY="<<miny<<" LIminZ="<<minz<<std::endl;
imlmin_y=miny;
imlmin_z=minz;
int cornerctr1=0,cornerctr2=0;


  for(float i=miny;;i+=fact_y)
  {
    ctr_z=0;
    for(float j=minz;;j+=fact_z)
    {

      float dz=0;
      std::vector<int>::iterator it;
      it=find(ridx.begin(),ridx.end(),ctr);
      ctr_z++;
      //  if(i==miny || j==miny || ctr_z==nop_z || ctr_y==nop_y)
      // {
      //  dz=0.1;
      // }


      if(it!=ridx.end())
      {
       dz=0.3;
      }
     cloud_selected_in1->points[ctr].y=i+dz;
     cloud_selected_in1->points[ctr].z=j+dz;
     cloud_selected_in1->points[ctr].x=((-1/(a))*((b*(i+dz))+(c*(j+dz))+d))+dz;//+(5*i*(exp(-i*i-j*j)));
    
    if(ctr==0) dlx.push_back(0.0);
    if(ctr>0)
    {
      dlx.push_back(cloud_selected_in1->points[ctr].x-cloud_selected_in1->points[ctr-1].x);
    //  std::cout<<"dlx["<<ctr<<"]="<<dlx[ctr-1]<<"\n";
    }
    if(cornerctr1>=2 && cornerctr1<=(nop_z-2) && cornerctr2>=2 && cornerctr2<=(nop_y-2) )
     li3dpointsz.push_back(cloud_selected_in1->points[ctr].x);

     //((-1/(a))*((b*i)+(c*j)+d))+dz;
     //d+(-j*exp(-i*i-j*j));
    // 
     // std::cerr << "\nPoints: " << cloud_selected_in1->points[ctr].x << " " 
     //                                  << cloud_selected_in1->points[ctr].y << " "
     //                                  << cloud_selected_in1->points[ctr].z << std::endl;  
      ctr++;
      cornerctr1++;
      
      if(ctr_z>=nop_z) break;

    }
    cornerctr1=0;
    cornerctr2++;
    ctr_y++;
    if(ctr_y>=nop_y) break;
  }

    
    pcl::io::savePCDFile("/media/ashu/25EA6BA0225EEBFD/ashu_ws/CoSM_ICP_v2/src/test_li_calib.pcd", *cloud_selected_in1);
    //viewer_res->addPlane(coefficients, p[0], p[1], p[2], "box_plane");
    viewer_res->addPointCloud(cloud_selected_in1, green, "in_clicked_points1");
    viewer_in->addPointCloud(cloud_selected_in1,green, "regeneratedpts");
    
    // for (int i = 0; i < cloud_selected_in->points.size(); i++)
    //     std::cout << cloud_selected_in->points[i].x << std::endl;
    std::cout << "clopud_in" << cloud_selected_in->points.size() << std::endl;
    std::cout<<"\n ........................END OF LIDAR COMPUTATION................................................................\n";

}

void ppp_callback_icp(const pcl::visualization::AreaPickingEvent& event, void *args)
{
    struct callback_args * data = (struct callback_args *)args;//Point cloud data & visualization window
    std::vector<int > indices;
    if (event.getPointsIndices(indices) == -1)
        return;
    for (size_t i = 0; i < indices.size(); i++)
    {
        cloud_selected_icp->points.push_back(cloud_icp->points.at(indices[i]));
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_selected_icp, 0, 0, 255);

    std::stringstream ss;
    std::string cloudName;
    ss << num_icp++;
    ss >> cloudName;
    cloudName += "_icp_cloudName";
    viewer_icp->removePointCloud("icp_clicked_points");
    viewer_icp->addPointCloud(cloud_selected_icp, blue, "icp_clicked_points");
    viewer_icp->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "icp_clicked_points");

       // float leaf_s=0.3;
       // pcl::UniformSampling<PointT> sor;
       // sor.setRadiusSearch (0.1);
       // sor.setInputCloud (cloud_selected_icp);
       
       // sor.filter(*cloud_selected_icp);


    /// Perform RANSAC Plane detection
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_selected_icp);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    //return (-1);
  }

  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                     << coefficients->values[1] << " "
  //                                     << coefficients->values[2] << " " 
  //                                     << coefficients->values[3] << std::endl;  

  pcl::PointXYZ minPt, maxPt;       
  float a=coefficients->values[0],b=coefficients->values[1],c=coefficients->values[2],d=coefficients->values[3];

  // a=-im_c;
  // b=im_a;
  // c=-im_b;
  // d=-im_d;

  float da=a,db=b,dc=c,dd=d;
  a=da;
  b=db;
  c=dc;
  d=dd;



  // std::cerr << "Model coefficients imx: " << a << " " 
  //                                     << b << " "
  //                                     << c << " " 
  //                                     << d<< std::endl;  
  pcl::getMinMax3D(*cloud_selected_icp,minPt,maxPt);      
   std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
 

  // std::cout << "Max imx: " << im_maxx/1000 << std::endl;
  // std::cout << "Max imy: " << im_maxy/1000 << std::endl;
  // std::cout << "Max imz: " << im_maxz/1000 << std::endl;
  // std::cout << "Min imx: " << im_minx/1000 << std::endl;
  // std::cout << "Min imy: " << im_miny/1000 << std::endl;
  // std::cout << "Min imz: " << im_minz/1000 << std::endl;

  

  float fact=0.09;
  float ydiff=maxPt.y-minPt.y;
  float zdiff=maxPt.z-minPt.z;
  std::cout<<"\nstydiff="<<ydiff<<" stzdiff="<<zdiff;
  int ctr=0;

// float maxy=maxPt.y;
// float maxz=maxPt.z;

// float miny=minPt.y;
// float minz=minPt.z;

float maxy=maxPt.y;
float maxz=maxPt.z;
float maxx=maxPt.x;

float miny=minPt.y;
float minz=minPt.z;
float minx=minPt.x;
 // maxy=-0.9;
 // maxz=0.6;

  // miny=-2.0+lmin_y;
  // minz=-0.5+lmin_z;

 // maxy=im_maxx/1000;
 // maxz=im_maxy/1000;

 // miny=im_minx/1000;
 // minz=-im_miny/1000;

 // // std::cout << "Max imx: " << im_maxx/1000 << std::endl;
 //  std::cout << "Max imy: " << maxy<< std::endl;
 //  std::cout << "Max imz: " << maxz << std::endl;
 //  //std::cout << "Min imx: " << im_minx/1000 << std::endl;
 //  std::cout << "Min imy: " << miny<< std::endl;
 //  std::cout << "Min imz: " << minz << std::endl;

 //  maxy=(im_maxx/1000)+(0.108*2);
 // maxz=(im_maxy/1000)+(0.108*2);

 // miny=(im_minx/1000)+(0.108*2);
 // minz=-((im_miny/1000)+(0.108*2));

 // // std::cout << "Max imx: " << im_maxx/1000 << std::endl;
 //  std::cout << "2Max imy: " << maxy<< std::endl;
 //  std::cout << "2Max imz: " << maxz << std::endl;
 //  //std::cout << "Min imx: " << im_minx/1000 << std::endl;
 //  std::cout << "2Min imy: " << miny<< std::endl;
 //  std::cout << "2Min imz: " << minz << std::endl;

 int st_ctr_z=0;
 int st_ctr_y=0;

// maxy=roundF(maxy);
// maxz=roundF(maxz);
// maxx=roundF(maxx);

// miny=roundF(miny);
// minz=roundF(minz);
// minx=roundF(minx);

float fact_y=((maxy-miny)/nop_y);
float fact_z=((maxz-minz)/nop_z);
float fact_x=((maxx-minx)/nop_x);
std::cout<<"\nfacts="<<fact_y<<" facT_z="<<fact_z;

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_selected_icp1,0, 255, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(cloud_selected_icp1,255, 255, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_selected_icp1, 255, 0, 0);

  // for(float i=miny;i<maxy;i+=fact_y)
  // {
  //   st_ctr_z=0;
  //   for(float j=minz;j<maxz;j+=fact_z)
  //   {
  //    // cloud_selected_in1->points[ctr].y=i;
  //    // cloud_selected_in1->points[ctr].z=j;
  //    // cloud_selected_in1->points[ctr].x=(-1/(c))*((a*i)+(b*j)+d);
  //     ctr++;
  //     st_ctr_z++;
  //     if(st_ctr_z>=nop_z)break;

  //   }
  //   st_ctr_y++;
  //  if(st_ctr_y>=nop_y)break;
  // }
  std::cout<<"\nnop="<<ctr<<" ["<<st_ctr_y<<","<<st_ctr_z<<"]";
  cloud_selected_icp1->width=nop_z*nop_y;
  cloud_selected_icp1->height=1;
  cloud_selected_icp1->resize(nop_z*nop_y*1);
 ctr=0;
st_ctr_z=0;
st_ctr_y=0;


//c=-c;
std::cerr << "Model coefficients for stereo : " << a << " " 
                                      << b << " "
                                      << c << " " 
                                      << d<< std::endl;  

std::cout<<"\nlminy="<<lmin_y<<" lminz="<<lmin_z<<std::endl;

miny=roundF(miny);
minz=roundF(minz);

st_fact_y=0.1;
st_fact_z=0.1;

std::cout<<"\nST_FACTY="<<st_fact_y<<" ST_FACTZ="<<st_fact_z<<std::endl;
std::cout<<"\nSTminyY="<<miny<<" STminZ="<<minz<<std::endl;
 for(float i=miny;;i+=st_fact_y)
  {
    st_ctr_z=0;
    for(float j=minz;;j+=st_fact_z)
    {
      float dz=0;
      std::vector<int>::iterator it;
      it=find(ridx.begin(),ridx.end(),ctr);
        st_ctr_z++;
      // if(i==miny || j==miny || st_ctr_z==nop_z || st_ctr_y==nop_y)
      // {
      //  dz=0.1;
      // }

      if(it!=ridx.end())
      {
       dz=0.2;
      }
     cloud_selected_icp1->points[ctr].y=i+dz;
     cloud_selected_icp1->points[ctr].z=j+dz;
     cloud_selected_icp1->points[ctr].x=((-1/(a))*((b*i)+(c*j)+d))+dz;
     //+((5*i)*exp(-i*i-j*j));
    // if(ctr>1)
    // {
    //    cloud_selected_icp1->points[ctr].x=cloud_selected_icp1->points[ctr-1].x+dlx[ctr-1]+dz;
    // }
     //((-1/(a))*((b*i)+(c*j)+d))+dz;
     //d+(-j*exp(-i*i-j*j));
    // 
     // std::cerr << "\nPoints: " << cloud_selected_in1->points[ctr].x << " " 
     //                                  << cloud_selected_in1->points[ctr].y << " "
     //                                  << cloud_selected_in1->points[ctr].z << std::endl;  
      ctr++;
     
      if(st_ctr_z>=nop_z) break;

    }
    st_ctr_y++;
    if(st_ctr_y>=nop_y) break;
  }

  

    
     
    std::cout << "cloud_icp" << cloud_selected_icp->points.size() << std::endl;

       

    /// viewer_res->addPointCloud (cloud_selected_icp, cloud_icp_color_h1, "cloud_icp_v1", v1);
     viewer_res->addPointCloud(cloud_selected_icp1, red, "icp_clicked_points");
     pcl::io::savePCDFile("/media/ashu/25EA6BA0225EEBFD/ashu_ws/CoSM_ICP_v2/src/test_st_calib.pcd", *cloud_selected_icp1);
    
}
int
main (int argc,
      char* argv[])
{
 

  // Checking program arguments
  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide one ply file.\n");
    return (-1);
  }

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 2)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[2]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }
  

 if (argc > 3)
  {
    // If the user passed the number of iteration as an argument
    imidx = atoi (argv[3]);
    std::cout<<"\nINdexed image="<<imidx;
  }

  double per=30.0;
  int nop=(per*nop_y*nop_z)/100;
  


  
 //ofstream ofs1,ofs2;

  //ofs2.open("iter_data/idx_att.txt",ios::app);
  for(int i=0;i<(nop);i++)
  {
    int v1=rand()%(nop_y*nop_z);
     
   //  if((i>10 && i<30) || (i>60 && i<80) || (i>130 && i<150)) 
  ridx.push_back(v1);

  }
  pcl::console::TicToc time;
  time.tic ();
  loadFile (argv[1], *cloud_in);
  cv::Mat calib_image;
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

 // loadFile (argv[3], *cloud_icp);
//  calib_image=cv::imread(argv[3],cv::IMREAD_COLOR);
//  std::cout << "\nLoaded file " << argv[3] << " (" << cloud_icp->size () << " points) in " << time.toc () << " ms\n" << std::endl;

// if(calib_image.empty())
//     {
//         std::cout << "Could not read the image: " << std::endl;
//         return 1;
//     }
    
    // std::vector<cv::Point2f> pointBuf;
    // int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    // bool found = cv::findChessboardCorners( calib_image, cv::Size(7,5), pointBuf,chessBoardFlags);
    //   cv::drawChessboardCorners( calib_image,cv::Size(7,5), cv:: Mat(pointBuf), found );
    //   if(found) std::cout<<"found corners";


 
   // cv::imshow("Output", view);
   // cv::waitKey(0);
 
  

  viewer_in->createViewPort (0.0, 0.0, 1.0, 1.0, v1);
  viewer_icp->createViewPort (0.0, 0.0, 1.0, 1.0, v2);
  //viewer->createViewPort (0.0, 0.0, 0.5, 0.5, v3);

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
   pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_icp, 200, 0, 20);
  viewer_in->addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  //viewer_icp->addPointCloud (cloud_in, red, "lidarp", v2);

//  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 20, 200, 20);
 // viewer_icp->addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v1", v2);


  // Original point cloud is white
  
 // Display image.
    cv::String pattern=cv::String(argv[4])+"*.jpg";
    Ve ve;
    

  



 
  viewer_icp->registerAreaPickingCallback(ppp_callback_icp, (void*)&cloud_icp);
  viewer_in->registerAreaPickingCallback(ppp_callback_in, (void*)&cloud_in);
  //viewer->spin();
  
  int vctr=0;
  // // Display the visualiser
  while (!viewer_in->wasStopped () || !viewer_icp->wasStopped() || !viewer_res->wasStopped())
  {
     viewer_in->spinOnce(100);
     viewer_res->spinOnce(100);
     viewer_icp->spinOnce(100);

     if(cloud_selected_in->size()>0 && vctr==0)
     {
      ve.ReadImage(pattern);
      vctr++;
      // std::cout<<"\nNow we can perform something";
       

     }
     //boost::this_thread::sleep(boost::posix_time::microseconds(100000));

  }

return 0;


}