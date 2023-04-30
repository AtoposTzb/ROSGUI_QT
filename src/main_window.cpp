/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rosqt_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rosqt_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    initUis();
	setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    //QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    //QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    //连接里程信息
    connect(&qnode,SIGNAL(speed_vel(float,float)),this,SLOT(slot_update_dashboard(float,float)));
    //连接电池电压
    //connect(&qnode,SIGNAL(power_vel(float)),this,SLOT(slot_update_power(float)));
    //连接图像话题
    connect(&qnode,SIGNAL(image_val(QImage)),this,SLOT(slot_update_image(QImage)));
    connect(ui.pushButton_sub_image,SIGNAL(clicked()),this,SLOT(slot_sub_image()));
    //激光雷达
    connect(ui.pushButton_laser,SIGNAL(clicked()),this,SLOT(slot_quick_cmd_laser()));
    //坐标 返航点
    connect(&qnode,SIGNAL(position(double,double,double)),this,SLOT(slot_update_pos(double,double,double)));
    //set start pose
    connect(ui.set_start_btn,SIGNAL(clicked()),this,SLOT(slot_set_start_pose()));
    connect(ui.set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_goal_pose()));

    //
    connect(ui.set_return_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_return_pos()));
    connect(ui.return_pos_btn,SIGNAL(clicked()),this,SLOT(slot_return_pos()));
    //连接遥感
    connect(rock_widget, SIGNAL(keyNumchanged(int)), this,SLOT(slot_rockKeyChange(int)));

    //连接角速度线速度进度条显示
    connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(slot_linear_value_change(int)));
    connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_dian,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_dou,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));


    connections();

}
void MainWindow::display_rviz()
{
    QSettings settings("ros_qt5_gui_app", "Displays");
    bool Grid_enable = settings.value("Grid/enable", bool(true)).toBool();
    double Grid_count = settings.value("Grid/count", double(20)).toDouble();

    bool Map_enable = settings.value("Map/enable", bool(true)).toBool();
    QString Map_topic = settings.value("Map/topic", QString("/map")).toString();
    double Map_alpha = settings.value("Map/alpha", double(0.7)).toDouble();
    QString Map_scheme = settings.value("Map/scheme", QString("map")).toString();
    bool Laser_enable = settings.value("Laser/enable", bool(true)).toBool();
    QString Laser_topic =
        settings.value("Laser/topic", QString("/scan")).toString();
    bool Polygon_enable = settings.value("Polygon/enable", bool(true)).toBool();
    QString Polygon_topic =
        settings
            .value("Polygon/topic", QString("/move_base/local_costmap/footprint"))
            .toString();

    bool RobotModel_enable =
        settings.value("RobotModel/enable", bool(true)).toBool();
    bool Navigation_enable =
        settings.value("Navigation/enable", bool(true)).toBool();
    QString GlobalMap_topic =
        settings
            .value("Navigation/GlobalMap/topic",
                   QString("/move_base/global_costmap/costmap"))
            .toString();
    QString GlobalMap_paln = settings
                                 .value("Navigation/GlobalPlan/topic",
                                        QString("/move_base/NavfnROS/plan"))
                                 .toString();
    QString LocalMap_topic =
        settings
            .value("Navigation/LocalMap/topic",
                   QString("/move_base/local_costmap/costmap"))
            .toString();
    QString LocalMap_plan =
        settings
            .value("Navigation/LocalPlan/topic",
                   QString("/move_base/DWAPlannerROS/local_plan"))
            .toString();
}

void MainWindow::initUis()
{
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    //初始化遥感UI
    rock_widget = new JoyStick(ui.JoyStick_widget);
    rock_widget->show();

    //时间动态显示
    m_timerCurrentTime = new QTimer;
    m_timerCurrentTime->setInterval(100);
    m_timerCurrentTime->start();

    /*********************
    ** Auto Start
    **********************/
//    if ( ui.checkbox_remember_settings->isChecked() ) {
//        on_button_connect_clicked(true);
//    }

    //速度仪表盘实现
    //初始化u
    speed_x_dashBoard = new CCtrlDashBoard(ui.widget_speed_x);
    speed_y_dashBoard = new CCtrlDashBoard(ui.widget_speed_y);
    speed_x_dashBoard->setGeometry(ui.widget_speed_x->rect());//这样，speed_x_dashBoard的大小和ui.widget_speed_x的大小一样
    speed_y_dashBoard->setGeometry(ui.widget_speed_y->rect());
    speed_x_dashBoard->setValue(0);
    speed_y_dashBoard->setValue(0);//设置仪表盘默认指向
    ui.horizontalSlider_linear->setValue(50);//设置默认
    ui.horizontalSlider_raw->setValue(50);

    ui.pushButton_status->setIcon(QIcon("://images/status/status_none.png"));
    ui.min_btn->setIcon(QIcon("://images/min.png"));
    ui.max_btn->setIcon(QIcon("://images/max.png"));
    ui.close_btn->setIcon(QIcon("://images/close.png"));

    if (m_showMode == SHOWMODE::robot) {
      this->showFullScreen();
    } else {
      QSettings windows_setting("rosqt_gui", "windows");
      int x = windows_setting.value("WindowGeometry/x").toInt();
      int y = windows_setting.value("WindowGeometry/y").toInt();
      int width = windows_setting.value("WindowGeometry/width").toInt();
      int height = windows_setting.value("WindowGeometry/height").toInt();
      QDesktopWidget *desktopWidget = QApplication::desktop();
      QRect clientRect = desktopWidget->availableGeometry();
      QRect targRect0 = QRect(clientRect.width() / 4, clientRect.height() / 4,
                              clientRect.width() / 2, clientRect.height() / 2);
      QRect targRect = QRect(x, y, width, height);
      if (width == 0 || height == 0 || x < 0 || x > clientRect.width() || y < 0 ||
          y > clientRect
                  .height())  //如果上一次关闭软件的时候，窗口位置不正常，则本次显示在显示器的正中央
      {
        targRect = targRect0;
      }
      this->setGeometry(targRect);  //设置主窗口的大小
    }
    //rviz
//    ui.treeWidget->setWindowTitle("Display");
//    ui.treeWidget->setWindowIcon(QIcon(":/images/display.png")); 使用label替换
    //header
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    ui.treeWidget->setHeaderHidden(true);

    //GLobal options
    QTreeWidgetItem* Global = new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon(":images/setting.png"));
    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);
    //FixFrame
    QTreeWidgetItem* Fixed_frame = new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    fixed_box = new QComboBox() ;
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);

    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));
    Global->addChild(Fixed_frame);
    ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);

    //Grid ui设计
    QTreeWidgetItem* Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    //设置图标
    Grid->setIcon(0,QIcon(":/images/Grid.png"));
    //checkbox
    QCheckBox* Grid_Check=new QCheckBox();
    //连接grid信号
    connect(Grid_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(Grid);
    //添加checkbox
    ui.treeWidget->setItemWidget(Grid,1 ,Grid_Check) ;
    //设置grid默认展开状态
    Grid->setExpanded(true) ;
    //添加Cell Count子 节点
    QTreeWidgetItem* Cell_Count = new QTreeWidgetItem(QStringList()<<"Plane Cell Count");
    Grid->addChild(Cell_Count);

    //CellCount添加SpinBox
    Cell_Count_Box = new QSpinBox();
    Cell_Count_Box->setValue(13) ;
    //设置QSpinBox的宽度
    Cell_Count_Box->setMaximumWidth(300);
    ui.treeWidget->setItemWidget(Cell_Count,1,Cell_Count_Box);
    //添加color子节点
    QTreeWidgetItem* Grid_color = new QTreeWidgetItem(QStringList()<<"Color");
    Grid->addChild(Grid_color) ;
    //Color添加ComboBox
    Grid_Color_Box = new QComboBox();
    Grid_Color_Box->addItem("160;160;160");
    //设置Comboox可编辑
    Grid_Color_Box->setEditable(true);
    //设置Combox的宽度
    Grid_Color_Box->setMaximumWidth(300);
    ui.treeWidget->setItemWidget(Grid_color,1,Grid_Color_Box);



    //TF ui设计
    QTreeWidgetItem* TF = new QTreeWidgetItem(QStringList()<<"TF");
    //设置图标
    TF->setIcon(0,QIcon(":/images/TF.png"));
    //checkbox
    QCheckBox* TF_Check = new QCheckBox();
    connect(TF_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));
    //向Treewidget添加Tf Top节点
    ui.treeWidget->addTopLevelItem(TF);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(TF,1,TF_Check);

    //LaserScan ui设计
    QTreeWidgetItem* LaserScan = new QTreeWidgetItem(QStringList()<<"LaserScan");
    //设置图标
    TF->setIcon(0,QIcon(":/images/rviz_images/LaserScan.png"));
    //checkbox
    QCheckBox* Laser_Check = new QCheckBox();
    connect(Laser_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_laser(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(LaserScan) ;
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(LaserScan,1,Laser_Check);
    //laser topic
    QTreeWidgetItem* LaserTopic = new QTreeWidgetItem(QStringList()<<"Topic");
    Laser_Topic_box=new QComboBox();
    Laser_Topic_box->addItem("/scan");
    Laser_Topic_box->setEditable(true) ;
    Laser_Topic_box->setMaximumWidth(150) ;
    LaserScan->addChild(LaserTopic);
    ui.treeWidget->setItemWidget(LaserTopic,1,Laser_Topic_box);

    //RobotModel
    QTreeWidgetItem* RobotModel = new QTreeWidgetItem(QStringList()<<"RobotModel");
    //设置图标
    TF->setIcon(0,QIcon(":/images/rviz_images/RobotModel.png"));
    //checkbox
    QCheckBox* RobotModel_Check = new QCheckBox();
    connect(RobotModel_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_RobotModel(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(RobotModel) ;
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(RobotModel,1,RobotModel_Check);

    //Map ui
    QTreeWidgetItem* Map = new QTreeWidgetItem(QStringList()<<"Map");
    //设置图标
    Map->setIcon(0,QIcon(":/images/rviz_images/Map.png"));
    //checkbox
    QCheckBox* Map_Check = new QCheckBox();
    connect(Map_Check,SIGNAL(stateChanged(int)) ,this ,SLOT(slot_display_Map(int)));
    //向Treewidget添加Map Top节点
    ui.treeWidget->addTopLevelItem(Map);
    //向Map添加checkbox
    ui.treeWidget->setItemWidget(Map,1,Map_Check);
    //Map topic
    QTreeWidgetItem* MapTopic = new QTreeWidgetItem(QStringList()<<"Topic");
    Map_Topic_box = new QComboBox();
    Map_Topic_box->addItem("/map");
    Map_Topic_box->setEditable(true);
    Map_Topic_box->setMaximumWidth(150);
    Map->addChild(MapTopic);
    ui.treeWidget->setItemWidget(MapTopic,1,Map_Topic_box);
    //Map color scheme
    QTreeWidgetItem* MapCoLorScheme = new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Map_Color_Scheme_box = new QComboBox();
    Map_Color_Scheme_box-> addItem("map");
    Map_Color_Scheme_box->addItem("costmap");
    Map_Color_Scheme_box->addItem("raw");
    Map_Color_Scheme_box->setMaximumWidth(150);
    Map->addChild(MapCoLorScheme);
    ui.treeWidget->setItemWidget(MapCoLorScheme,1,Map_Color_Scheme_box);

    //Path ui
    QTreeWidgetItem* Path = new QTreeWidgetItem(QStringList()<<"Path");
    //设置图标
    Path->setIcon(0,QIcon(":/images/rviz_images/Path.png"));
    //checkbox
    QCheckBox* Path_Check = new QCheckBox();
    connect(Path_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Path(int)));
    //向Treewidget添加Path Top节点
    ui.treeWidget->addTopLevelItem(Path);
    //向Path添加checkbox
    ui.treeWidget->setItemWidget(Path,1 ,Path_Check);
    //Path topic
    QTreeWidgetItem* PathTopic = new QTreeWidgetItem (QStringList()<<"Topic");
    Path_Topic_box = new QComboBox();
    Path_Topic_box-> addItem("/move_base/DWAPlannerROS/local_plan");
    Path_Topic_box->setEditable(true);
    Path_Topic_box->setMaximumWidth(150);
    Path-> addChild(PathTopic);
    ui. treeWidget->setItemWidget(PathTopic,1,Path_Topic_box) ;
    //Path color scheme
    QTreeWidgetItem* PathColorScheme=new QTreeWidgetItem(QStringList()<<"Color");
    Path_Color_box = new QComboBox();
    Path_Color_box->addItem("0;12;255");
    Path_Color_box->setEditable(true);
    Path_Color_box->setMaximumWidth(150);
    Path->addChild(PathColorScheme);
    ui.treeWidget->setItemWidget(PathColorScheme,1,Path_Color_box);

    //机器人Navigate相关**************************
    //Golabal map******************************
    QTreeWidgetItem* GlobalMap = new QTreeWidgetItem(QStringList()<<"Global Map");
    GlobalMap->setIcon(0,QIcon(":/images/default_package_icon.png"));
    QCheckBox* GlobalMap_Check=new QCheckBox();
    connect(GlobalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_global_map(int)));
    ui.treeWidget->addTopLevelItem(GlobalMap);
    ui.treeWidget->setItemWidget(GlobalMap,1,GlobalMap_Check);
    //Global CostMap
    QTreeWidgetItem* Global_CostMap = new QTreeWidgetItem(QStringList()<<"Costmap");
    //设置图标
    Global_CostMap->setIcon(0,QIcon(":/images/rviz_images/Map.png"));
    //Global Map添加子节点
    GlobalMap->addChild(Global_CostMap);
    //Map topic
    QTreeWidgetItem* Global_CostMap_Topic = new QTreeWidgetItem(QStringList()<<"Topic");
    Global_CostMap_Topic_box = new QComboBox();
    Global_CostMap_Topic_box->addItem("/move_base/global_costmap/costmap");
    Global_CostMap_Topic_box->setEditable(true);
    Global_CostMap_Topic_box->setMaximumWidth(150);
    Global_CostMap->addChild(Global_CostMap_Topic);
    ui.treeWidget->setItemWidget(Global_CostMap_Topic,1,Global_CostMap_Topic_box) ;
    //Map color scheme
    QTreeWidgetItem* GlobalMapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    GlobalMapColorScheme_box = new QComboBox() ;
    GlobalMapColorScheme_box->addItem("costmap");
    GlobalMapColorScheme_box->addItem("map");
    GlobalMapColorScheme_box->addItem("raw");
    GlobalMapColorScheme_box->setMaximumWidth(150);
    Global_CostMap->addChild(GlobalMapColorScheme);
    ui.treeWidget->setItemWidget(GlobalMapColorScheme,1,GlobalMapColorScheme_box);

    //Global Planner
    QTreeWidgetItem* Global_Planner = new QTreeWidgetItem(QStringList()<<"Planner");
    //设置图标
    Global_Planner->setIcon(0,QIcon(":/images/rviz_images/Path.png"));
    //向Global Map添加Path Top节点
    GlobalMap->addChild(Global_Planner);

    //Path topic
    QTreeWidgetItem* Global_Planner_Topic = new QTreeWidgetItem(QStringList()<<"Topic");
    Global_Planner_Topic_box = new QComboBox();
    Global_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/global_plan");
    Global_Planner_Topic_box->setEditable(true);
    Global_Planner_Topic_box->setMaximumWidth(150);
    Global_Planner->addChild(Global_Planner_Topic);
    ui.treeWidget->setItemWidget(Global_Planner_Topic, 1 ,Global_Planner_Topic_box);
    //Path color scheme
    QTreeWidgetItem* Global_Planner_Color_Scheme = new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Global_Planner_Color_box=new QComboBox() ;
    Global_Planner_Color_box->addItem("255;0;0");
    Global_Planner_Color_box->setEditable(true);
    Global_Planner_Color_box->setMaximumWidth(150) ;
    Global_Planner->addChild(Global_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget (Global_Planner_Color_Scheme,1 ,Global_Planner_Color_box);

    //Local map*************************************
    QTreeWidgetItem* LocalMap = new QTreeWidgetItem(QStringList()<<"Local Map");
    LocalMap->setIcon(0,QIcon(":/images/default_package_icon.png"));
    QCheckBox* LocalMap_Check = new QCheckBox();
    connect(LocalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_local_map(int)));
    ui.treeWidget->addTopLevelItem(LocalMap);
    ui.treeWidget->setItemWidget(LocalMap,1,LocalMap_Check);
    //Local CostMap
    QTreeWidgetItem* Local_CostMap = new QTreeWidgetItem(QStringList()<<"Costmap");
    //设置图标
    Local_CostMap->setIcon(0,QIcon(":/images/rviz_images/Map.png"));
    //Local Map添加子节点
    LocalMap-> addChild(Local_CostMap);
    //Map topic
    QTreeWidgetItem* Local_CostMap_Topic = new QTreeWidgetItem(QStringList()<<"Topic");
    Local_CostMap_Topic_box = new QComboBox();
    Local_CostMap_Topic_box-> addItem("/move_base/local_costmap/costmap");
    Local_CostMap_Topic_box->setEditable(true);
    Local_CostMap_Topic_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_CostMap_Topic);
    ui.treeWidget->setItemWidget(Local_CostMap_Topic,1,Local_CostMap_Topic_box);
    //Map color scheme
    QTreeWidgetItem* LocalMapColorScheme = new QTreeWidgetItem(QStringList()<<"Color Scheme");
    LocalMapColorScheme_box = new QComboBox();
    LocalMapColorScheme_box->addItem("costmap");
    LocalMapColorScheme_box->addItem("map") ;
    LocalMapColorScheme_box->addItem("raw");
    LocalMapColorScheme_box->setMaximumWidth(150);
    Local_CostMap->addChild(LocalMapColorScheme);
    ui.treeWidget->setItemWidget(LocalMapColorScheme,1,LocalMapColorScheme_box);
    //Local Planner
    QTreeWidgetItem* Local_Planner = new QTreeWidgetItem (QStringList()<<"Planner");
    //设置图标
    Local_Planner->setIcon(0,QIcon(":/images/rviz_images/Path.png"));
    //向TLocal Map添加Path Top节点
    LocalMap->addChild(Local_Planner);

    //Path topic
    QTreeWidgetItem* Local_Planner_Topic = new QTreeWidgetItem(QStringList()<<"Topic");
    Local_Planner_Topic_box = new QComboBox();
    Local_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Local_Planner_Topic_box->setEditable(true);
    Local_Planner_Topic_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Topic);
    ui.treeWidget->setItemWidget(Local_Planner_Topic,1,Local_Planner_Topic_box);
    //Path color scheme
    QTreeWidgetItem* Local_Planner_Color_Scheme = new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Local_Planner_Color_box = new QComboBox();
    Local_Planner_Color_box->addItem("0;12;255");
    Local_Planner_Color_box->setEditable(true);
    Local_Planner_Color_box->setMaximumWidth(150) ;
    Local_Planner->addChild(Local_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Local_Planner_Color_Scheme,1,Local_Planner_Color_box);

}

void MainWindow::initVideos()
{
    QSettings video_topic_setting("rosqt_gui", "settings");
    QStringList names = video_topic_setting.value("video/names").toStringList();
    QStringList topics = video_topic_setting.value("video/topics").toStringList();
    if (topics.size() == 4) {
      if (topics[0] != "") qnode.Sub_Image(topics[0], 0);
      if (topics[1] != "") qnode.Sub_Image(topics[1], 1);
      if (topics[2] != "") qnode.Sub_Image(topics[2], 2);
      if (topics[3] != "") qnode.Sub_Image(topics[3], 3);
    }

    //链接槽函数
    connect(&qnode, SIGNAL(Show_image(int, QImage)), this,
            SLOT(slot_show_image(int, QImage)));
}

void MainWindow::connections()
{

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                     SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this,
                     SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this,
                     SLOT(slot_rosShutdown()));
    QObject::connect(m_timerCurrentTime, &QTimer::timeout, [=]() {
      ui.label_time->setText(
          QDateTime::currentDateTime().toString("  hh:mm:ss  "));
    });
    // connect速度的信号
    connect(&qnode, SIGNAL(speed_x(double)), this, SLOT(slot_speed_x(double)));
    connect(&qnode, SIGNAL(speed_y(double)), this, SLOT(slot_speed_yaw(double)));
    //机器人状态
    connect(&qnode, SIGNAL(updateRobotStatus(RobotStatus)), this,
            SLOT(slot_updateRobotStatus(RobotStatus)));
    //电源的信号
    connect(&qnode, SIGNAL(batteryState(sensor_msgs::BatteryState)), this,
            SLOT(slot_batteryState(sensor_msgs::BatteryState)));
    //绑定slider的函数
//    connect(ui.horizontalSlider_raw, SIGNAL(valueChanged(int)), this,
//            SLOT(Slider_raw_valueChanged(int)));
//    connect(ui.horizontalSlider_linear, SIGNAL(valueChanged(int)), this,
//            SLOT(Slider_linear_valueChanged(int)));
    //设置界面
    connect(ui.settings_btn, SIGNAL(clicked()), this, SLOT(slot_setting_frame()));
    //绑定速度控制按钮
    connect(ui.pushButton_i, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
    connect(ui.pushButton_u, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
    connect(ui.pushButton_o, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
    connect(ui.pushButton_j, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
    connect(ui.pushButton_l, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
    connect(ui.pushButton_m, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
    connect(ui.pushButton_dou, SIGNAL(clicked()), this,
            SLOT(slot_cmd_control()));
    connect(ui.pushButton_dian, SIGNAL(clicked()), this,
            SLOT(slot_cmd_control()));
    connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(slot_dis_connect()));
    //返航
//    connect(ui.return_btn, SIGNAL(clicked()), this, SLOT(slot_return_point()));
    connect(ui.close_btn, SIGNAL(clicked()), this, SLOT(slot_closeWindows()));
    connect(ui.min_btn, SIGNAL(clicked()), this, SLOT(slot_minWindows()));
    connect(ui.max_btn, SIGNAL(clicked()), this, SLOT(slot_maxWindows()));
    connect(rock_widget, SIGNAL(keyNumchanged(int)), this,
            SLOT(slot_rockKeyChange(int)));
}
void MainWindow::slot_updateRobotStatus(RobotStatus status)
{
    switch (status) {
      case RobotStatus::none: {
        QTimer::singleShot(100, [this]() {
          ui.pushButton_status->setIcon(
              QIcon(":/images/status/status_none.png"));
          //m_roboItem->setRobotColor(eRobotColor::blue);
        });
      } break;
      case RobotStatus::normal: {
        QTimer::singleShot(200, [this]() {
          ui.pushButton_status->setIcon(
              QIcon(":/images/status/status_normal.png"));
          //m_roboItem->setRobotColor(eRobotColor::blue);
        });
      } break;
      case RobotStatus::error: {
        QTimer::singleShot(300, [this]() {
          ui.pushButton_status->setIcon(
              QIcon(":/images/status/status_error.png"));
          //m_roboItem->setRobotColor(eRobotColor::red);
        });
      } break;
      case RobotStatus::warn: {
        QTimer::singleShot(400, [this]() {
          ui.pushButton_status->setIcon(
              QIcon(":/images/status/status_warn.png"));
          //m_roboItem->setRobotColor(eRobotColor::yellow);
        });
      } break;
    }
}

void MainWindow::slot_display_global_map(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    QStringList qli = Global_Planner_Color_box->currentText().split(";");//冒号分割
    QColor color = QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());//类型转换
    myqrviz->Display_Global_Map(Global_CostMap_Topic_box->currentText(),GlobalMapColorScheme_box->currentText(),Global_Planner_Topic_box->currentText(),color,enable);
}

void MainWindow::slot_display_local_map(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    QStringList qli = Local_Planner_Color_box->currentText().split(";");//冒号分割
    QColor color = QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());//类型转换
    myqrviz->Display_Local_Map(Local_CostMap_Topic_box->currentText(),LocalMapColorScheme_box->currentText(),Local_Planner_Topic_box->currentText(),color,enable);
}

void MainWindow::slot_set_return_pos()
{
    ui.return_x->setText(ui.pos_x->text());
    ui.return_y->setText(ui.pos_x->text());
    ui.return_z->setText(ui.pos_x->text());
}

void MainWindow::slot_return_pos()
{
    //qnode.set_goal(ui.return_x->text().toDouble(),ui.return_y->text().toDouble(),ui.return_z->text().toDouble());
}

void MainWindow::slot_update_pos(double x, double y, double z)
{
    ui.pos_x->setText(QString::number(x));
    ui.pos_y->setText(QString::number(y));
    ui.pos_z->setText(QString::number(z));
}

void MainWindow::slot_set_start_pose()
{
    myqrviz->Set_Start_Pose();
}

void MainWindow::slot_set_goal_pose()
{
    myqrviz->Set_Goal_Pose();
}

void MainWindow::slot_display_Path(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    QStringList qli = Path_Color_box->currentText().split(";");//冒号分割
    QColor color = QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());//类型转换
    myqrviz->Display_Path(Path_Topic_box->currentText(),color,enable);

}

void MainWindow::slot_display_Map(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    myqrviz->Display_Map(Map_Topic_box->currentText(),Map_Color_Scheme_box->currentText(),enable);
}

void MainWindow::slot_display_RobotModel(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    myqrviz->Display_RobotModel(enable);
}

void MainWindow::slot_display_laser(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    myqrviz->Display_LaserScan(Laser_Topic_box->currentText(),enable);
}

void MainWindow::slot_display_tf(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    myqrviz->Display_TF(enable);
}

void MainWindow::slot_display_grid(int state)
{
    bool enable = state > 1?true:false;//三目运算符判断是否被选中
    QStringList qli=Grid_Color_Box->currentText().split(";");//冒号分割
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());//类型转换
    myqrviz->Display_Srid(Cell_Count_Box->text().toInt(),color,enable);
}

void MainWindow::slot_treewidget_value_change(QString)
{
    myqrviz->Set_FixedFrame(fixed_box->currentText());
}

void MainWindow::slot_quick_cmd_laser()
{
    laser_cmd=new QProcess;
    laser_cmd->start("bash");//通过这个对象去调用一些外部的系统程序，如bash程序：运行命令行的
    laser_cmd->write(ui.textEdit_laser_cmd->toPlainText().toLocal8Bit()+'\n');//通过write方法去写入我们要运行的命令，'/n'代表命令输入结束
    //再设计一个黑框框模拟信号回显
    connect(laser_cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_quick_output()));
    connect(laser_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));
}

void MainWindow::slot_quick_output()
{
    //在黑框框output中使用追加的方式显示并设置字体颜色
    ui.textEdit_quick_output->append("<font color=\"#FF0000\">"+laser_cmd->readAllStandardError()+"</font");
    ui.textEdit_quick_output->append("<font color=\"#FFFFFF\">"+laser_cmd->readAllStandardOutput()+"</font");
}

void MainWindow::slot_update_image(QImage im)
{
    ui.label_image->setPixmap(QPixmap::fromImage(im));
}
void MainWindow::slot_sub_image()
{
    qnode.sub_image(ui.lineEdit_image_topic->text());
}

//void MainWindow::slot_update_power(float value)
//{
//    ui.label_power_val->setText(QString::number(value).mid(0,5)+"V");//只取前5个字符
//    //进度条显示，先计算电压比
//    double n = (value-10.5)/(12.5-10.5);//12.5 and 10.5为实体机器人设置的最大和最小电压
//    int val = n*100;//转换为百分比
//    ui.progressBar->setValue(val);
//}

void MainWindow::slot_update_dashboard(float x,float y)
{
    //将节点发送来的信号响应设置到仪表盘上
    speed_x_dashBoard->setValue(abs(x)*100);
    speed_y_dashBoard->setValue(abs(y)*100);
    //方向
    ui.label_dir_x->setText(x>0?"正向":"反向");
    ui.label_dir_y->setText(x>0?"正向":"反向");

}

//sender()方法可以处理是哪个对象发送来的对象并处理
//按钮控制响应事件
void MainWindow::slot_pushbtn_click()
{
    QPushButton*btn = qobject_cast<QPushButton*> (sender());
    qDebug()<<btn->text();
    char k = btn->text().toStdString()[0];//获取按键按下的字符
    //判断是否使用全向轮
    bool is_all = ui.checkBox_isAll->isChecked();
    float linear = ui.label_linear->text().toFloat()*0.01;//cm / mm
    float angular = ui.label_raw->text().toFloat()*0.01;

    switch(k){
    case 'i':
        qnode.set_cmd_vel(is_all?'I':'i' ,linear ,angular);
        break;
    case 'u':
        qnode.set_cmd_vel(is_all?'U':'u' ,linear ,angular);
        break;
    case 'o':
        qnode.set_cmd_vel(is_all?'O':'o' ,linear ,angular);
        break;
    case 'j':
        qnode.set_cmd_vel(is_all?'J':'j' ,linear ,angular);
        break;
    case 'l':
        qnode.set_cmd_vel(is_all?'L':'l' ,linear ,angular);
        break;
    case 'm':
        qnode.set_cmd_vel(is_all?'M':'m' ,linear ,angular);
        break;
    case ',':
        qnode.set_cmd_vel(is_all?'<':',' ,linear ,angular);
        break;
    case '.':
        qnode.set_cmd_vel(is_all?'>':'.' ,linear ,angular);
        break;
    }
}
void MainWindow::slot_rockKeyChange(int key){
  qDebug()<<"key: "<<key;
  //速度
  float liner=ui.horizontalSlider_linear->value()*0.01;
  float turn=ui.horizontalSlider_raw->value()*0.01;
  bool is_all=ui.checkBox_isAll->isChecked();
  switch (key) {
      case upleft:
          qnode.move_base(is_all?'U':'u',liner,turn);
      break;
      case up:
          qnode.move_base(is_all?'I':'i',liner,turn);
      break;
      case upright:
          qnode.move_base(is_all?'O':'o',liner,turn);
      break;
      case left:
          qnode.move_base(is_all?'J':'j',liner,turn);
      break;
      case right:
          qnode.move_base(is_all?'L':'l',liner,turn);
      break;
      case down:
          qnode.move_base(is_all?'M':'m',liner,turn);
      break;
      case downleft:
          qnode.move_base(is_all?'<':',',liner,turn);
      break;
      case downright:
          qnode.move_base(is_all?'>':'.',liner,turn);
      break;
  }
}
void MainWindow::slot_cmd_control()
{
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    char key = btn->text().toStdString()[0];
    //速度
    float liner = ui.horizontalSlider_linear->value() * 0.01;
    float turn = ui.horizontalSlider_raw->value() * 0.01;
    bool is_all = ui.checkBox_isAll->isChecked();
    switch (key) {
      case 'u':
        qnode.move_base(is_all ? 'U' : 'u', liner, turn);
        break;
      case 'i':
        qnode.move_base(is_all ? 'I' : 'i', liner, turn);
        break;
      case 'o':
        qnode.move_base(is_all ? 'O' : 'o', liner, turn);
        break;
      case 'j':
        qnode.move_base(is_all ? 'J' : 'j', liner, turn);
        break;
      case 'l':
        qnode.move_base(is_all ? 'L' : 'l', liner, turn);
        break;
      case 'm':
        qnode.move_base(is_all ? 'M' : 'm', liner, turn);
        break;
      case ',':
        qnode.move_base(is_all ? '<' : ',', liner, turn);
        break;
      case '.':
        qnode.move_base(is_all ? '>' : '.', liner, turn);
        break;
    }
}
void MainWindow::slot_batteryState(sensor_msgs::BatteryState msg)
{
    ui.label_power_val->setText(QString::number(msg.voltage).mid(0, 5) + "V");
    double percentage = msg.percentage;
    //speedDashBoard->set_oil(percentage);
    ui.progressBar->setValue(percentage > 100 ? 100 : percentage);
    //当电量过低时发出提示
    if (percentage <= 20) {
      ui.progressBar->setStyleSheet(
          "QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar "
          "{border: 2px solid grey;border-radius: 5px;text-align: center;}");
      // QMessageBox::warning(NULL, "电量不足", "电量不足，请及时充电！",
      // QMessageBox::Yes , QMessageBox::Yes);
    } else {
      ui.progressBar->setStyleSheet(
          "QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: "
          "center;}");
    }
}

void MainWindow::slot_setting_frame()
{
}

void MainWindow::slot_speed_x(double x)
{
//    speedDashBoard->set_speed(abs(x * 100));
//    if (x > 0.001) {
//      speedDashBoard->set_gear(CCtrlDashBoard::kGear_D);
//    } else if (x < -0.001) {
//      speedDashBoard->set_gear(CCtrlDashBoard::kGear_R);
//    } else {
//      speedDashBoard->set_gear(CCtrlDashBoard::kGear_N);
//    }
//    QString number = QString::number(abs(x * 100)).mid(0, 2);
//    if (number[1] == ".") {
//      number = number.mid(0, 1);
//    }
    //    ui.label_speed->setText(number);
}

void MainWindow::slot_speed_yaw(double yaw)
{
//    if (yaw > m_turnLightThre) {
//      ui.label_turnLeft->setPixmap(
//          QPixmap::fromImage(QImage("://images/turnLeft_hl.png")));
//    } else if (yaw < -m_turnLightThre) {
//      ui.label_turnRight->setPixmap(
//          QPixmap::fromImage(QImage("://images/turnRight_hl.png")));
//    } else {
//      ui.label_turnLeft->setPixmap(
//          QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
//      ui.label_turnRight->setPixmap(
//          QPixmap::fromImage(QImage("://images/turnRight_l.png")));
//    }
}

void MainWindow::slot_move_camera_btn() { emit signalSetMoveCamera(); }

void MainWindow::slot_show_image(int frame_id, QImage image)
{
    switch (frame_id) {
      case 0:
        ui.label_video0->setPixmap(QPixmap::fromImage(image).scaled(
            ui.label_video0->width(), ui.label_video0->height()));
        break;
      case 1:
        ui.label_video1->setPixmap(QPixmap::fromImage(image).scaled(
            ui.label_video1->width(), ui.label_video1->height()));
        break;
      case 2:
        ui.label_video2->setPixmap(QPixmap::fromImage(image).scaled(
            ui.label_video2->width(), ui.label_video2->height()));
        break;
      case 3:
        ui.label_video3->setPixmap(QPixmap::fromImage(image).scaled(
            ui.label_video3->width(), ui.label_video3->height()));
        break;
    }
}

void MainWindow::slot_dis_connect()
{
    ros::shutdown();
    slot_rosShutdown();
    emit signalDisconnect();
    this->close();
}
bool MainWindow::connectMaster(QString master_ip, QString ros_ip,
                               bool use_envirment) {
  //如果使用环境变量
  if (use_envirment) {
    if (!qnode.init()) {
      return false;
    } else {
//      //初始化视频订阅的显示
      initVideos();
//      //显示话题列表
//      initTopicList();
//      initOthers();
    }
  }
  //如果不使用环境变量
  else {
    if (!qnode.init(master_ip.toStdString(), ros_ip.toStdString())) {
      return false;
    } else {
      //初始化视频订阅的显示
      initVideos();
//      //显示话题列表
//      initTopicList();
//      initOthers();
    }
  }
  ReadSettings();
  return true;
}

void MainWindow::slot_rosShutdown()
{
    slot_updateRobotStatus(RobotStatus::none);
}

//void MainWindow::slot_chartTimerTimeout()
//{
//    QImage image(600, 600, QImage::Format_RGB888);
//    QPainter painter(&image);
//    painter.setRenderHint(QPainter::Antialiasing);
//    m_qgraphicsScene->render(&painter);
//    qnode.pub_imageMap(image);
//}

//void MainWindow::slot_pubImageMapTimeOut()
//{
//    QImage image(600, 600, QImage::Format_RGB888);
//    QPainter painter(&image);
//    painter.setRenderHint(QPainter::Antialiasing);
//    m_qgraphicsScene->render(&painter);
//    qnode.pub_imageMap(image);
//}
//隐藏
//void MainWindow::slot_hide_table_widget()
//{
//    if (ui.stackedWidget_left->isHidden()) {
//      ui.stackedWidget_left->show();
//    } else {
//      ui.stackedWidget_left->hide();
//      // ui.table_hide_btn->setStyleSheet("QPushButton{background-image:
//      // url(://images/show.png);border:none;}");
//    }
//}

//滑动条处理槽函数
void MainWindow::slot_linear_value_change(int value)
{
    ui.label_linear->setText(QString::number(value));
}
//滑动条处理槽函数
void MainWindow::slot_raw_value_change(int value)
{
    ui.label_raw->setText(QString::number(value));
}

MainWindow::~MainWindow() {
    if (base_cmd) {
      delete base_cmd;
      base_cmd = NULL;
    }
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool check ) {
//	if ( ui.checkbox_use_environment->isChecked() ) {
//		if ( !qnode.init() ) {
//			showNoMasterMessage();
//            ui.treeWidget->setEnabled(false);//myrviz 对象没有连接上master时,设置为不可用
//		} else {
//			ui.button_connect->setEnabled(false);
//            ui.treeWidget->setEnabled(true);//连接成功时设置为可用,防止被意外调用
//            myqrviz = new qrviz(ui.Layout_rviz);
//		}
//	} else {
// 		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
//				   ui.line_edit_host->text().toStdString()) ) {
//			showNoMasterMessage();
//            ui.treeWidget->setEnabled(false);//myrviz 对象没有连接上master时,设置为不可用
//		} else {
//			ui.button_connect->setEnabled(false);
//			ui.line_edit_master->setReadOnly(true);
//			ui.line_edit_host->setReadOnly(true);
//			ui.line_edit_topic->setReadOnly(true);
//            ui.treeWidget->setEnabled(true);//连接成功时设置为可用,防止被意外调用
//            myqrviz = new qrviz(ui.Layout_rviz);
//		}
//	}
//}


//void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
//	bool enabled;
//	if ( state == 0 ) {
//		enabled = true;
//	} else {
//		enabled = false;
//	}
//	ui.line_edit_master->setEnabled(enabled);
//	ui.line_edit_host->setEnabled(enabled);
//	//ui.line_edit_topic->setEnabled(enabled);
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("rosqt_gui", "settings");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
    m_masterUrl =
        settings.value("connect/master_url", QString("http://192.168.1.2:11311/"))
            .toString();
    m_hostUrl =
        settings.value("connect/host_url", QString("192.168.1.3")).toString();
    m_useEnviorment =
        settings.value("connect/use_enviorment", bool(false)).toBool();
    m_autoConnect = settings.value("connect/auto_connect", bool(false)).toBool();
    m_turnLightThre =
        settings.value("connect/lineEdit_turnLightThre", double(0.1)).toDouble();
    if (settings.value("main/show_mode", "control").toString() == "control") {
      m_showMode = SHOWMODE::control;
    } else {
      m_showMode = SHOWMODE::robot;
    }
}

void MainWindow::WriteSettings() {
    QSettings windows_setting("rosqt_gui", "window");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    windows_setting.clear();  //清空当前配置文件中的内容
    windows_setting.setValue("WindowGeometry/x", this->x());
    windows_setting.setValue("WindowGeometry/y", this->y());
    windows_setting.setValue("WindowGeometry/width", this->width());
    windows_setting.setValue("WindowGeometry/height", this->height());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}
void MainWindow::mousePressEvent(QMouseEvent *event) {
  m_lastPos = event->globalPos();
  isPressedWidget = true;  // 当前鼠标按下的即是QWidget而非界面上布局的其它控件
}
void MainWindow::mouseMoveEvent(QMouseEvent *event) {
  if (isPressedWidget) {
    this->move(this->x() + (event->globalX() - m_lastPos.x()),
               this->y() + (event->globalY() - m_lastPos.y()));
    m_lastPos = event->globalPos();
  }
}
void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
  // 其实这里的mouseReleaseEvent函数可以不用重写
  m_lastPos = event->globalPos();
  isPressedWidget = false;  // 鼠标松开时，置为false
}
//最大化最小化关闭
void MainWindow::slot_closeWindows() { this->close(); }
void MainWindow::slot_minWindows() { this->showMinimized(); }
void MainWindow::slot_maxWindows() {
  if (this->isFullScreen()) {
    this->showNormal();
  } else {
    this->showFullScreen();
  }
}

}  // namespace rosqt_gui

