#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <iostream>

#include <QApplication>
#include <QMainWindow>
#include <QTimer>
#include <QObject>
#include <QElapsedTimer>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDebug>

//#include "rclcpp/clock

#include <urdf/model.h>
// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "qrosnode.h"
#include "jntinfo.h"

class JointStateWidget : public QMainWindow
{
    Q_OBJECT

public:
    JointStateWidget(int argc, char** argv, QWidget* parent=nullptr);
//    virtual ~JointStateWidget() = default;
//    ~JointStateWidget() {std::cerr << __PRETTY_FUNCTION__ << "\n";}

    bool load_urdf_model_from_file(const std::string& urdf_file_path);
    bool load_urdf_model_from_string(const std::string& xmlstring);

    double get_rand_from_range(const double& min, const double& max);
    void print_kdl_tree_info(const KDL::Tree &tree);
    void print_kdl_chain_info(const KDL::Chain &chain);

    void create_custom_widget(QFormLayout *formLayout, QString title, QSlider* sld, QLabel* lbl);
    void update_joints();
    void publish_joints();

    double map_slider_pos_to_val(const int& ival, const int& jnt_nbr);

private slots:
    void slot_randomize();
    void slot_center();
    void robot_description_msg();
    void slot_process_universal_slider(int v);

private:
    QRosNode qnode;
    void create_central_widget();
    static constexpr int m_jnt_sld_steps = 60;
    static constexpr int m_jnt_sld_steps_2 = m_jnt_sld_steps / 2;

    std::vector<QSlider*> m_jnt_SLD;
    std::vector<QLabel*> m_jnt_LBL;
    QSlider* produce_slider(const int& steps);

    urdf::Model m_robot_model;
    JntInfo m_jnt_info;
    KDL::Tree m_kdl_tree;
    KDL::Chain m_kdl_chain;
    KDL::ChainFkSolverPos_recursive* m_fksolver;
};

inline
double JointStateWidget::map_slider_pos_to_val(const int& ival, const int& jnt_nbr)
{
//    const double step = (ival < 0) ?
//                fabs(m_jnt_info.min(jnt_nbr)) / m_jnt_sld_steps_2 :
//                m_jnt_info.max(jnt_nbr) / m_jnt_sld_steps_2;
//    return ival * step;
    return ival * ((ival < 0) ? fabs(m_jnt_info.min(jnt_nbr)) / m_jnt_sld_steps_2 : m_jnt_info.max(jnt_nbr) / m_jnt_sld_steps_2);
}

inline
QSlider* JointStateWidget::produce_slider(const int& steps)
{
    QSlider* sld = new QSlider(Qt::Horizontal);
    const int start_step = -steps/2;
    const int end_step = steps/2;

    sld->setMinimum(start_step);
    sld->setMaximum(end_step);
    sld->setSingleStep(1);
    sld->setValue(0);

    return sld;
}

inline
void JointStateWidget::update_joints()
{
    for (unsigned i(0); i<m_jnt_LBL.size(); i++){
        m_jnt_LBL[i]->setText(QString::number(m_jnt_info.cur(i), 'f', 2));
    }
    // Publish
    publish_joints();
}
inline
void JointStateWidget::publish_joints()
{
    if (m_jnt_info.size < 2)
        return;

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.name.reserve(m_jnt_info.size);
    joint_msg.position.reserve(m_jnt_info.size);
    joint_msg.velocity.reserve(m_jnt_info.size);
    joint_msg.header.stamp = qnode.m_ros_node->get_clock()->now();

    for (unsigned i(0); i<m_jnt_info.size; i++){
//        std::cerr << "joint name [" << i << "]: " << m_jnt_info.m_joint_names.at(i) << "\n";
        joint_msg.name.emplace_back(m_jnt_info.names.at(i));
        joint_msg.position.emplace_back(m_jnt_info.cur(i));
        joint_msg.velocity.emplace_back(0.0);
    }

    //send the joint state
    qnode.joint_pub->publish(joint_msg);
}

inline
void JointStateWidget::create_custom_widget(QFormLayout *formLayout, QString title, QSlider* sld, QLabel* lbl)
{
    QWidget* widget = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout(this);
    QLabel *name_lbl = new QLabel(title);
    name_lbl->setStyleSheet("font-weight: bold; color: black");
    layout->addWidget(name_lbl);
    layout->addWidget(lbl);
    widget->setLayout(layout);

    formLayout->addRow(widget);
    formLayout->addRow(sld);
}

inline
double JointStateWidget::get_rand_from_range(const double& min, const double& max)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(min, max);

    return dist(gen);
}

inline
void JointStateWidget::print_kdl_tree_info(const KDL::Tree &tree)
{
    std::cerr << "*** Print KDL Tree ***" << std::endl;
//    std::cerr << "Tree has " << tree.getNrOfJoints() << " joints" << std::endl;
    std::cerr << "Tree has " << tree.getNrOfSegments() << " segments" << std::endl;
    std::cerr << "The segments are:\n";
    KDL::SegmentMap segment_map = tree.getSegments();
    KDL::SegmentMap::iterator it;
    for(it=segment_map.begin(); it != segment_map.end(); it++)
        std::cerr << (*it).first << std::endl;
    std::cerr << "*** End print KDL Tree ***" << std::endl;
}

inline
void JointStateWidget::print_kdl_chain_info(const KDL::Chain &chain)
{
    std::cerr << "*** Print KDL Chain ***" << std::endl;
    std::cerr <<"Chain has " << chain.getNrOfSegments() << " segments" << std::endl;
    std::cerr <<"The segments are:\n";
    for (unsigned i=0;i<chain.getNrOfSegments();i++) {
        KDL::Segment seg = chain.getSegment(i);
        std::cerr << seg.getName() << " " << seg.getJoint().getName()
                  << std::endl;
    }
    std::cerr << "*** End print KDL Chain ***" << std::endl;
}
