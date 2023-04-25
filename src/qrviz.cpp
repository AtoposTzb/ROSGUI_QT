#include "../include/rosqt_gui/qrviz.hpp"

/*
两个类，如何在mainW类中显示呢，链接两个类：
通过构造函数，在初始化对象时传入一个layout指针(QVBoxLayout *layout)，通过这个指针添加widget
*/

//创建rviz panel
qrviz::qrviz(QVBoxLayout *layout)
{
    render_panel = new rviz::RenderPanel();
    //向layout添加
    layout->addWidget(render_panel);
    //创建rvi z控制对象
    manager_ = new rviz::VisualizationManager(render_panel);
    tool_manager_ = manager_->getToolManager();   
    ROS_ASSERT(manager_ != NULL);//解决闪退BUG
    //初始化render_panel 实现放大缩小等操作
    render_panel->initialize(manager_->getSceneManager(), manager_);
//   manager_->setFixedFrame("map");     可能会产生BUG

    //初始化rviz控制对象
    manager_->initialize() ;
    manager_->startUpdate() ;//初始化render_panel需要放在startUpdate()前面,开始更新显示
    manager_->removeAllDisplays();


}

void qrviz::Set_FixedFrame(QString Frame_name)
{
    manager_->setFixedFrame(Frame_name);
    qDebug()<<manager_->getFixedFrame();
}

void qrviz::Display_Srid(int Cell_Count, QColor color, bool enable)
{
    if(Grid_ != NULL)//保持图层的唯一性
    {
        delete Grid_;
        Grid_ = NULL;
    }
    Grid_=manager_->createDisplay("rviz/Grid", "myGrid" , enable);
    //设置cell Count
    Grid_->subProp("Plane Cell Count")->setValue(Cell_Count);//Plane Cell Count属性设置需要用空格隔开，才能正常显示放大缩小
    //设置颜色
    Grid_->subProp("Color")->setValue(color);
    ROS_ASSERT(Grid_ != NULL);
}
//TF坐标变换显示接口
void qrviz::Display_TF(bool enable)
{
    if(TF_ != NULL)//保持图层的唯一性
    {
        delete TF_;
        TF_ = NULL;
    }
    TF_ = manager_->createDisplay("rviz/TF", "myTF" , enable);
    ROS_ASSERT(TF_ != NULL);
}

void qrviz::Display_LaserScan(QString laser_topic, bool enable)
{
    if(LaserScan_ != NULL)//保持图层的唯一性
    {
        delete LaserScan_;
        LaserScan_ = NULL;
    }
    LaserScan_ = manager_->createDisplay("rviz/LaserScan", "myLaser" , enable);
    LaserScan_->subProp("Topic")->setValue(laser_topic);//属性设置
    ROS_ASSERT(LaserScan_ != NULL);
}

rviz::Display* RobotModel_ = NULL;//可以解决闪退BUG,当然也可以放在.hpp文件中
void qrviz::Display_RobotModel(bool enable)
{
    if(RobotModel_ != NULL)//保持图层的唯一性
    {
        delete RobotModel_;
        RobotModel_ = NULL;
    }
    LaserScan_ = manager_->createDisplay("rviz/RobotModel", "myRobotModel" , enable);
    ROS_ASSERT(LaserScan_ != NULL);
}

void qrviz::Display_Map(QString topic, QString color_scheme, bool enable)
{
    if(Map_ != NULL)//保持图层的唯一性
    {
        delete Map_;
        Map_ = NULL;
    }
    Map_ = manager_->createDisplay("rviz/Map", "myMap" , enable);
    ROS_ASSERT(Map_ != NULL);
    Map_->subProp("Topic")->setValue(topic);//属性设置
    Map_->subProp("Color Scheme")->setValue(color_scheme) ;//图层样式
}

void qrviz::Display_Path(QString topic, QColor color, bool enable)
{
    if(Path_ != NULL)//保持图层的唯一性
    {
        delete Path_;
        Path_ = NULL;
    }
    Path_ = manager_->createDisplay("rviz/Path","myPath", enable);
    ROS_ASSERT(Path_ != NULL);
    Path_->subProp("Topic")->setValue(topic);//属性设置
    Path_->subProp("Color")->setValue(color) ;//图层样式

}

void qrviz::Set_Start_Pose()
{
    rviz::Tool* current_tool = tool_manager_->addTool("rviz/SetInitialPose");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool);
}

void qrviz::Set_Goal_Pose()
{
    rviz::Tool* current_tool = tool_manager_->addTool("rviz/SetGoal");
    //获取属性容器
    rviz::Property* pro=current_tool ->getPropertyContainer();
    //设置发布导航目标点的topic
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool);
}

void qrviz::Display_Local_Map(QString costmap_topic, QString costmap_color, QString path_topic, QColor path_color, bool enable)
{
    if(Local_Map_ != NULL){delete Local_Map_;Local_Map_ = NULL;}
    if(Local_Path_ != NULL){delete Local_Path_;Local_Path_ = NULL;}
    Local_Map_ =manager_->createDisplay("rviz/Map","localmap",enable);
    ROS_ASSERT(Local_Map_ !=NULL);
    Local_Path_ = manager_->createDisplay("rviz/Path", "localmap",enable);
    ROS_ASSERT(Local_Path_ !=NULL);
    Local_Map_->subProp("Topic")->setValue(costmap_topic);
    Local_Map_->subProp("Color Scheme")->setValue (costmap_color);
    //解决BUG不设置这两个path,也可以试试注释manager_->setFixedFrame("map");
    Local_Path_->subProp("Topic")->setValue (path_topic) ;
    Local_Path_->subProp("Color")->setValue(path_color);//设置这个path color颜色时也会出现一个BUG闪退
}

void qrviz::Display_Global_Map(QString costmap_topic, QString costmap_color, QString path_topic, QColor path_color, bool enable)
{
    if(Global_Map_ != NULL){delete Global_Map_;Global_Map_ = NULL;}
    if(Global_Path_ != NULL){delete Global_Path_;Global_Path_ = NULL;}
    Global_Map_ =manager_->createDisplay("rviz/Map","Globalmap",enable);
    ROS_ASSERT(Global_Map_ !=NULL);
    Global_Path_ = manager_->createDisplay("rviz/Path", "Globalmap",enable);
    ROS_ASSERT(Global_Path_ !=NULL);
    Global_Map_->subProp("Topic")->setValue(costmap_topic);
    Global_Map_->subProp("Color Scheme")->setValue (costmap_color);
    Global_Path_->subProp("Topic")->setValue (path_topic) ;
    Global_Path_->subProp("Color")->setValue(path_color);
}

