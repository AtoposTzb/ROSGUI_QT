/**
 * @file /include/rosqt_gui/main_window.hpp
 *
 * @brief Qt based gui for rosqt_gui.
 *
 * @date November 2010
 **/
#ifndef rosqt_gui_MAIN_WINDOW_H
#define rosqt_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "CCtrlDashBoard.hpp"
#include "qrviz.hpp"
#include <QImage>
#include <QProcess>//激光雷达
#include <QComboBox>
#include <QSpinBox>
#include "joystick.h"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rosqt_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    enum {upleft=0,up,upright,left,stop,right,downleft,down,downright};

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linear_value_change(int);//线速度改变槽函数
    void slot_raw_value_change(int);
    void slot_pushbtn_click();//几个按钮的点击事件,多个控制按钮连接同一个槽函数
    void slot_update_dashboard(float,float);//将节点发送来的信号响应设置到仪表盘上
    void slot_update_power(float);
    void slot_update_image(QImage);
    void slot_sub_image();
    void slot_quick_cmd_laser();
    void slot_quick_output();
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_laser(int);
    void slot_display_RobotModel(int);
    void slot_display_Map(int);
    void slot_display_Path(int);
    void slot_set_start_pose();
    void slot_set_goal_pose();
    void slot_display_global_map(int);
    void slot_display_local_map(int);
    void slot_update_pos(double,double,double);
    void slot_set_return_pos();
    void slot_return_pos();
    //遥感
    void slot_rockKeyChange(int);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    CCtrlDashBoard* speed_x_dashBoard;
    CCtrlDashBoard* speed_y_dashBoard;
    QProcess *laser_cmd;

//rviz
    qrviz *myqrviz;
    QComboBox* fixed_box;
    QSpinBox* Cell_Count_Box;
    QComboBox* Grid_Color_Box;
    QComboBox* Laser_Topic_box;
    QComboBox* Map_Topic_box;
    QComboBox* Map_Color_Scheme_box;
    QComboBox* Path_Topic_box;
    QComboBox* Path_Color_box;
    //Navigate
    QComboBox* Global_CostMap_Topic_box;
    QComboBox* GlobalMapColorScheme_box;
    QComboBox* Global_Planner_Topic_box;
    QComboBox* Global_Planner_Color_box;
    QComboBox* Local_CostMap_Topic_box;
    QComboBox* LocalMapColorScheme_box;
    QComboBox* Local_Planner_Topic_box;
    QComboBox* Local_Planner_Color_box;
    //遥感
    JoyStick *rock_widget;
};

}  // namespace rosqt_gui

#endif // rosqt_gui_MAIN_WINDOW_H
