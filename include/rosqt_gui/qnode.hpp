/**
 * @file /include/rosqt_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rosqt_gui_QNODE_HPP_
#define rosqt_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>//sudu
#include <map>
#include <nav_msgs/Odometry.h>//里程计话题
#include <std_msgs/Float32.h>//电池电压
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>//存放图像编码格式
#include <QImage>
#include <geometry_msgs/PoseWithCovarianceStamped.h>//位姿的消息类型
#include <geometry_msgs/PoseStamped.h>//导航目标点的消息类型

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <QDebug>

#include <QImage>
#include <QLabel>
#include <QSettings>
#include <QStringListModel>
#include <QtConcurrent/QtConcurrent>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rosqt_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    void set_cmd_vel(char k,float linear,float angular);//一个公共函数去连接键盘控制和速度的功能
    void move_base(char k, float speed_linear, float speed_trun);
    void sub_image(QString topic_name);
    //void set_goal(double x, double y, double z);
    void set_goal(QString frame, double x, double y, double z, double w);
    void Sub_Image(QString topic, int frame_id);
    void pub_imageMap(QImage map);
    QPointF transScenePoint2Word(QPointF pos);
    QPointF transWordPoint2Scene(QPointF pos);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void speed_vel(float,float);//因为这是两个类,ui界面是在mianw访问，所以这里需要我们创建自定义信号，把当前的X,Y轴线速度通过信号的方式发送到mainw类中
    void power_vel(float);
    void image_val(QImage);
    void position(double x,double y,double z);
    void batteryState(sensor_msgs::BatteryState);
    void updateLaserScan(QPolygonF points);
    void Master_shutdown();
    void Show_image(int, QImage);
    void updateMap(QImage map);
    void speed_x(double x);
    void speed_y(double y);
    void plannerPath(QPolygonF path);


private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher cmd_vel_pub;//声明一个话题发布者
    ros::Publisher cmd_pub;
    ros::Publisher goal_pub;//发布导航目标点的话题发布者S
    ros::Subscriber cmdVel_sub;
    ros::Subscriber m_laserSub;//雷达点云数据
    ros::Subscriber m_plannerPathSub;//全局path路径
    QStringListModel logging_model;
    ros::Subscriber chatter_sub;//创建一个订阅者
    ros::Subscriber odom_sub;//里程计话题订阅者

    ros::Subscriber power_sub;//电池电压
    ros::Subscriber battery_sub;

    ros::Subscriber amcl_pose_sub;//位姿的
    //图像
    image_transport::Subscriber image_sub;
    ros::Subscriber m_compressedImgSub0;
    ros::Subscriber m_compressedImgSub1;
    image_transport::Publisher m_imageMapPub;
    QImage Mat2QImage(cv::Mat const& src);
    cv::Mat QImage2Mat(QImage &image);
    QImage rotateMapWithY(QImage map);
    //图像订阅
    image_transport::Subscriber image_sub0;
    //地图订阅
    ros::Subscriber map_sub;
    //图像format
    QString video0_format;
    QString odom_topic;
    QString batteryState_topic;
    QString pose_topic;
    QString map_topic;

//    MoveBaseClient *movebase_client;
//    QStringListModel logging_model;
    QString show_mode = "control";
    //
    QString laser_topic;
    QString initPose_topic;
    QString naviGoal_topic;
    std::string path_topic;
    QPolygon mapPonits;
    QPolygonF plannerPoints;
    QPolygonF laserPoints;
    int m_threadNum = 4;
    int m_frameRate = 40;
    //地图 0 0点坐标对应世界坐标系的坐标
    float m_mapOriginX;
    float m_mapOriginY;
    //世界坐标系原点在图元坐标系坐标
    QPointF m_wordOrigin;
    //地图一个像素对应真实世界的距离
    float m_mapResolution;
    //地图是否被初始化
    bool m_bMapIsInit = false;
    // tf::TransformListener m_tfListener(ros::Duration(10));
    // ros::Timer m_rosTimer;
    tf::TransformListener *m_robotPoselistener;
    tf::TransformListener *m_Laserlistener;
    std::string base_frame, laser_frame, map_frame;

private:
    void chatter_callback(const std_msgs::String &msg);//shengminghuidiaohanshu
    void odom_callback(const nav_msgs::Odometry &msg);
    void power_callback(const std_msgs::Float32 &msg);
    void image_callback(const sensor_msgs::ImageConstPtr &msg);
    void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);

    void speedCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void batteryCallback(const sensor_msgs::BatteryState &message);
    void imageCallback0(const sensor_msgs::CompressedImageConstPtr &msg);
    void imageCallback1(const sensor_msgs::CompressedImageConstPtr &msg);
    void myCallback(const std_msgs::Float64 &message_holder);
    void mapCallback(nav_msgs::OccupancyGrid::ConstPtr map);
    void laserScanCallback(sensor_msgs::LaserScanConstPtr scan);
    void plannerPathCallback(nav_msgs::Path::ConstPtr path);
    void SubAndPubTopic();
};

}  // namespace rosqt_gui

#endif /* rosqt_gui_QNODE_HPP_ */
