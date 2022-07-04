#include "joint_state_widget.h"

JointStateWidget::JointStateWidget(int argc, char** argv, QWidget *parent):
    QMainWindow (parent),
    qnode(argc, argv)
{
    if (argc>2){
        std::cerr << "constructor argc: " << argc << "\n";
        std::cerr << "constructor urdf_model: " << argv[1] << "\n";
        std::string urdf_file_path = argv[1];
        if (load_urdf_model_from_file(urdf_file_path))
            create_central_widget();
    }

    setWindowTitle("Joint State Publisher");
    QObject::connect(&qnode, SIGNAL(msgSubscribed()), this, SLOT(robot_description_msg()));
}

void JointStateWidget::robot_description_msg()
{
    if(load_urdf_model_from_string(qnode.m_robot_descriptor_str)){
        create_central_widget();
    }
}

void JointStateWidget::create_central_widget()
{
    // Horizontal layout with a button
    QPushButton *randomize_btn = new QPushButton("Randomize");
    QPushButton *center_btn = new QPushButton("Center");

    QFormLayout *form_layout = new QFormLayout;
    form_layout->addRow(randomize_btn);
    form_layout->addRow(center_btn);
    for (unsigned i(0); i<m_jnt_info.size; i++){
        m_jnt_SLD.emplace_back(produce_slider(m_jnt_sld_steps));
        m_jnt_LBL.emplace_back(new QLabel());
        create_custom_widget(form_layout, QString::fromStdString(m_jnt_info.names[i]), m_jnt_SLD[i], m_jnt_LBL[i]);
        QObject::connect(m_jnt_SLD.at(i), &QSlider::valueChanged, this, &JointStateWidget::slot_process_universal_slider);
    }

    QWidget *centralWidget = new QWidget;
    centralWidget->setLayout(form_layout);
    setCentralWidget(centralWidget);

    QObject::connect(randomize_btn, SIGNAL(released()), this, SLOT(slot_randomize()));
    QObject::connect(center_btn, SIGNAL(released()), this, SLOT(slot_center()));

    QTimer::singleShot(2000, this, SLOT(slot_center()));
}

void JointStateWidget::slot_randomize()
{
    for (std::size_t i(0); i<m_jnt_info.size; i++) {
        m_jnt_info.cur(i) = get_rand_from_range(m_jnt_info.min(i), m_jnt_info.max(i));
    }

    update_joints();
}

void JointStateWidget::slot_center()
{
    if (m_jnt_info.size < 2)
        return;

    for (unsigned i(0); i<m_jnt_info.size; i++){
        m_jnt_info.cur(i) = 0.0;
    }
    update_joints();
}

bool JointStateWidget::load_urdf_model_from_string(const std::string& xmlstring)
{
    if (!m_robot_model.initString(xmlstring)) {
        std::cerr << "Error reading model from xmlstring\n";
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(m_robot_model, m_kdl_tree)) {
        std::cerr << "Failed to extract kdl tree from xmlstring urdf robot description" << std::endl;
        return false;
    }

    return m_jnt_info.read_joints(m_robot_model, m_kdl_tree);;
}

bool JointStateWidget::load_urdf_model_from_file(const std::string& urdf_file_path)
{
    if (urdf_file_path.empty()){
        std::cerr << "Error: load_urdf_model: no urdf file...\n";
        return false;
    }

    std::cerr << "load_urdf_model: " << urdf_file_path << "\n";
    if (!m_robot_model.initFile(urdf_file_path)) {
        std::cerr << "Error reading model\n";
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(m_robot_model, m_kdl_tree)) {
        std::cerr << "Failed to extract kdl tree from urdf robot description" << std::endl;
        return false;
    }

    return m_jnt_info.read_joints(m_robot_model, m_kdl_tree);
}

void JointStateWidget::slot_process_universal_slider(int v)
{
    QObject* obj = sender();

    for (unsigned i(0); i<m_jnt_SLD.size(); i++){
        if (obj == m_jnt_SLD[i]) {
            // rertrive double val from slider position
            const double val = map_slider_pos_to_val(v, i);
            // set val in m_jnt_info
            m_jnt_info.cur(i) = val;
            // set double val in m_jnt_LBL publish val to ros topic
            update_joints();
        }
    }
}
