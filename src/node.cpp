#include <boost/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "../include/bvt_sdk.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sonar/toggle.h>
#include <sonar/sonar_pixel.h>
#include <sonar/sonar_pixelRequest.h>
#include <sonar/sonar_pixelResponse.h>
#include <mutex>

namespace sonar{

#define IP              "192.168.1.45"
#define SOUND_SPEED     1500
#define MINIMUM_RANGE   2
#define MAXIMUM_RANGE   15
#define IMAGE_TYPE      BVTHEAD_IMAGE_XY    // option(s): XY, RTHETA
#define RES_TYPE        BVTHEAD_RES_AUTO    // option(s): OFF, LOW, MED, HIGH, AUTO
#define PING_RATE       12
#define COLOR_MAP       "/home/bbauvsbc1/bbauv/src/sensors/sonar/colormaps/bone.cmap"
//#define COLOR_MAP       "/home/bbauvsbc1/bbauv/src/sensors/sonar_driver/colormaps/bone.cmap"
#define WIDTH           640
#define HEIGHT          480

class Sonar : public nodelet::Nodelet
{
    public:
        Sonar() : running(true) , imageSaved(false) {}
        ~Sonar(){
            running = false;
            BVTMagImage_Destroy(rangeImage);
            BVTSonar_Destroy(son);
            poller.join();
        }
        virtual void onInit(){

            // Create sonar object
            son = BVTSonar_Create();
            int ret;

            // Create sonar interface
            if(!son){
                ROS_ERROR("unable to create sonar interface (sdk)");
                ros::shutdown();
            }

            // Test sonar connectivity
            if((ret = BVTSonar_Open(son, "NET", IP)) != 0){
                ROS_ERROR("unable to connect to the sonar (TCP/IP)");
                ROS_ERROR("BVTSonar_Open() returned %d", ret);
                ros::shutdown();
            }

            // Serial number
            // char buffer[30];
            // BVTSonar_GetSerialNumber(son, buffer, 30)) != 0);
            // ROS_INFO(buffer,"%s");

            // Get sonar headd
            head = NULL;
            if((ret = BVTSonar_GetHead(son, 1, &head)) != 0){
                ROS_ERROR("unable to proc sonar head");
                ROS_ERROR("BVTSonar_GetHead() returned %d", ret);
                ros::shutdown();
            }

            // Set parameters
	    //BVTHEAD_FLUID_SALTWATER
            BVTHead_SetFluidType(head, BVTHEAD_FLUID_SALTWATER); 
            //BVTHead_SetFluidType(head, BVTHEAD_FLUID_FRESHWATER);
            BVTHead_SetSoundSpeed(head, SOUND_SPEED);

            // Set range
            if((ret = BVTHead_SetRange(head, MINIMUM_RANGE, MAXIMUM_RANGE)) != 0){
                ROS_ERROR("unable to set range on the sonar head");
                ROS_ERROR("BVTHead_SetRange() returned %d", ret);
            }

            // Set image settings
            if((ret = BVTHead_SetImageRes(head, RES_TYPE)) != 0){
                ROS_ERROR("error setting image resolution");
                ROS_ERROR("BVTHead_SetImageRes() returned %d", ret);
            }

            // Set image type
            if((ret = BVTHead_SetImageType(head, IMAGE_TYPE)) != 0){
                ROS_ERROR("error setting image type");
                ROS_ERROR("BVTHead_SetImageType() returned %d", ret);
            }

            // Set size
            if((ret = BVTHead_SetImageReqSize(head, HEIGHT, WIDTH)) != 0){
                ROS_ERROR("error setting image req size");
                ROS_ERROR("BVTHead_SetImageReqSize() returned %d", ret);
            }

            // Blueview sonar does not support changing gain or time variable gain
            // Set CV Image parameters
            _out.encoding = sensor_msgs::image_encodings::MONO8;
            _out.header.frame_id = std::string("sonar_raw_image");

            // Set Ping rate for polling and services
            rate = new ros::Rate(PING_RATE);
            magpub = getPrivateNodeHandle().advertise<sensor_msgs::Image>
                ("/sonar_image", 1);
            ros::ServiceServer _tsrv = getPrivateNodeHandle().advertiseService
                ("/sonar_toggle", &Sonar::toggle, this);
            _psrv = getPrivateNodeHandle().advertiseService
                ("/sonar_pixel", &Sonar::sonar_pixel, this);

            // NEED SERVICE FOR GETTING DISTANCE

            // Thread for polling
            poller = boost::thread(boost::bind(&Sonar::poll, this));
            ROS_INFO("sonar initialized successfully");
        }

        bool toggle(sonar::toggleRequest &req, sonar::toggleResponse &resp)
        {
            if(running){
                running = false;
            }
            resp.toggled = running;
            return true;
        }

        bool sonar_pixel(sonar::sonar_pixel::Request &req, sonar::sonar_pixel::Response &rsp)
        {

            mtx.lock();
            ROS_INFO("%d %d", req.x, req.y);

            int row = BVTMagImage_GetHeight(rangeImage);
            int col = BVTMagImage_GetWidth(rangeImage);
            
            rsp.range = BVTMagImage_GetPixelRange(rangeImage, (int)row, (int)col);
            rsp.bearing = BVTMagImage_GetPixelRelativeBearing(rangeImage, (int)row, (int)col);
            
            ROS_INFO("range bearing for pixel [%d  %d] = [%f  %f]", req.x, req.y, rsp.range, rsp.bearing);
            mtx.unlock();

            return true;
        }

        void poll(){
            while(running){

                BVTPing ping = NULL;
                int ret;

                // Send out ping
                if((ret = BVTHead_GetPing(head, -1, &ping)) != 0){
                    ROS_ERROR("error retrieving sonar ping");
                    return;
                }

                // Get raw sonar image
                BVTMagImage img;
                if((ret = BVTPing_GetImage(ping, &img)) != 0){
                    ROS_ERROR("error obtaining image from ping");
                    return;
                }

                // Get range sonar image
                // mtx.lock();
                // if((ret = BVTPing_GetImage(ping, &rangeImage)) != 0){
                //     ROS_ERROR("error obtaining image from ping");
                //     return;
                // }
                // mtx.unlock();

                // Colormap it
                BVTColorMapper colorMap;
                if((colorMap = BVTColorMapper_Create()) == NULL) {
                    ROS_ERROR("error creating color mapper");
                    return;
                }
                if((ret = BVTColorMapper_Load(colorMap, COLOR_MAP)) != 0) {
                    ROS_ERROR("error retrieving color map: %s", BVTError_GetString(ret));
                    return;
                }
                BVTColorImage colorImg;
                if((ret = BVTColorMapper_MapImage(colorMap, img, &colorImg)) != 0) {
                    ROS_ERROR("error mapping to color image: %s", BVTError_GetString(ret));
                    return;
                }

                double range_resolution = BVTMagImage_GetRangeResolution(img);
                double bearing_resolution = BVTMagImage_GetBearingResolution(img);
                // ROS_INFO("RangeRes BearingRes: %lf %lf", range_resolution, bearing_resolution);

                double min_angle = BVTMagImage_GetFOVMinAngle(img);
                double max_angle = BVTMagImage_GetFOVMaxAngle(img);
                // ROS_INFO("Min Max Angle: %lf %lf", min_angle, max_angle);


                int width_sonar = BVTMagImage_GetWidth(img);
                int height_sonar = BVTMagImage_GetHeight(img);
                // ROS_INFO("Image: %d %d", width_sonar, height_sonar);

                // Getting Range
                // BVTRangeData rangeData;
                // if((ret = BVTPing_GetRangeData(ping, &rangeData)) != 0){
                //     ROS_ERROR("error obtaining image for range");
                //     return;
                // }

                // int numberOfRanges = BVTRangeData_GetCount(rangeData);
                // std::vector<float> m_ranges;
                // std::vector<float> m_angles ;
                // std::vector<float> m_intensities ;
                // m_angles.reserve(numberOfRanges);
                // for ( int j = 0 ; j < numberOfRanges ; j++)
                //     m_angles.push_back(BVTRangeData_GetBearingValue(rangeData,j));
                // m_ranges.reserve(numberOfRanges);
                // for(int j = 0 ; j < numberOfRanges ; j++)
                //     m_ranges.push_back(BVTRangeData_GetRangeValue(rangeData, j ));
                // // m_intensities.reserve(numberOfRanges);
                // // for(int j = 0 ; j < numberOfRanges ; j++)
                // //     m_intensities.push_back(BVTRangeData_GetIntensityValue(rangeData , j ));
                // printf("\tAngle\tRange\tIntensity\n" );
                // for(int i = 0 ; i < numberOfRanges ; ++i )
                // {
                //     printf("\t%04f\t%04f\n", m_angles.at(i), m_ranges.at(i));

                //     //printf("\t%04f\t%04f\t%04f\n", m_angles.at(i), m_ranges.at(i), m_intensities.at(i));
                // }
                // printf("\n");
                // break;



                // Getting sizes
                // for(int x=0;x<width_sonar;x++){
                //     for(int y=0;y<height_sonar;y++){

                //         int row = y;
                //         int col = x;
                        
                //         double range = BVTMagImage_GetPixelRange(rangeImage, (int)row, (int)col);
                //         double bearing = BVTMagImage_GetPixelRelativeBearing(rangeImage, (int)row, (int)col);
                //         printf("%d %d %lf %lf \n", x, y, range, bearing);
                //     }
                // }
                // break;


                //Convert the color map to an image
                cv::Mat finalImage(
                    cv::Size(BVTMagImage_GetWidth(img), BVTMagImage_GetHeight(img)), 
                    CV_8UC4, 
                    BVTColorImage_GetBits(colorImg)
                );
                cv::cvtColor(finalImage, finalImage, CV_RGBA2GRAY);
                // cv::resize(finalImage, finalImage, cv::Size(WIDTH, HEIGHT));

                // Need a way to save one image for ranging

                // Publish image then sleep
                _out.image = finalImage;
                _out.header.stamp = ros::Time::now();
                magpub.publish(_out.toImageMsg());

                // Destructors
                BVTMagImage_Destroy(img);
                BVTColorMapper_Destroy(colorMap);
                BVTColorImage_Destroy(colorImg);
                BVTPing_Destroy(ping);
                rate->sleep();
            }
        }

    private:
        volatile bool running;
        bool imageSaved;
        BVTMagImage rangeImage;
        BVTSonar son;
        BVTHead head;
        boost::thread poller;
        ros::Rate *rate;
        ros::Publisher magpub;
        cv_bridge::CvImage _out;
        ros::ServiceServer _psrv;
        std::mutex mtx;
};

PLUGINLIB_DECLARE_CLASS(sonar, Sonar, sonar::Sonar, nodelet::Nodelet)
}
