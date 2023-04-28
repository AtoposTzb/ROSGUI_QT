#ifndef LOGINSETTING_H
#define LOGINSETTING_H

#include <QComboBox>
#include <QDesktopServices>
#include <QSpinBox>

#include "./basewidget/customwidget.h"
#include "main_window.hpp"
#include <QWidget>

namespace Ui {
class loginsetting;
}

class loginsetting : public QWidget
{
    Q_OBJECT

public:
    explicit loginsetting(QWidget *parent = nullptr);
    ~loginsetting();


signals:
 void signalRotate();
protected:
 void changeEvent(QEvent* e);

private slots:
 // 登陆
 void on_btnLogin_clicked();
 // 注册
 //    void on_btnRegedit_clicked();
 // 服务器信息返回处理

 void SltAnimationFinished();

 void SltEditFinished();

 void on_checkBoxAutoLogin_clicked(bool checked);
 void slot_autoLoad();
 void slot_ShowWindow();
 void slot_writeSettings();

 private:
 bool m_bConnected;
 rosqt_gui::MainWindow* mainWindow = NULL;
 QString m_qRosIp;
 QString m_qMasterIp;
 QComboBox* fixed_box;
 QSpinBox* Cell_Count_Box;
 QComboBox* Grid_Color_Box;
 QComboBox* Laser_Topic_box;
 QComboBox* Polygon_Topic_box;
 QComboBox* Map_Topic_box;
 QComboBox* Map_Color_Scheme_box;
 QComboBox* Path_Topic_box;
 QComboBox* Path_Color_box;
 // Navigate
 QComboBox* Global_CostMap_Topic_box;
 QComboBox* GlobalMapColorScheme_box;
 QComboBox* Local_CostMap_Topic_box;
 QComboBox* LocalMapColorScheme_box;
 QComboBox* Global_Planner_Topic_box;
 QComboBox* Global_Planner_Color_box;
 QComboBox* Local_Planner_Topic_box;
 QComboBox* Local_Planner_Color_box;
 QCheckBox* LocalMap_Check;
 QCheckBox* GlobalMap_Check;
 QCheckBox* Path_Check;
 QCheckBox* Grid_Check;
 QCheckBox* TF_Check;
 QCheckBox* Laser_Check;
 QCheckBox* Polygon_Check;
 QCheckBox* RobotModel_Check;
 QCheckBox* Map_Check;
 bool m_bIsConnect = true;

private:
 void InitWidget();
 void readSettings();
 void ConnectMaster();

protected:
 void paintEvent(QPaintEvent*);

private:
    Ui::loginsetting *ui;
};

#endif // LOGINSETTING_H
