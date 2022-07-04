#ifndef JNTINFO_H
#define JNTINFO_H

#include <iostream>
#include <string>

// KDL
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

class JntInfo
{
public:
    std::size_t size;
    std::vector<std::string> names;
    KDL::JntArray max;
    KDL::JntArray min;
    KDL::JntArray max_v;
    KDL::JntArray cur;

    bool read_joints(const urdf::Model &robot_model, const KDL::Tree &kdl_tree);
};

inline
bool JntInfo::read_joints(const urdf::Model &robot_model, const KDL::Tree &kdl_tree)
{
    const KDL::SegmentMap segment_map = kdl_tree.getSegments();
    size = kdl_tree.getNrOfJoints();
    cur.resize(size);
    max.resize(size);
    min.resize(size);
    max_v.resize(size);
    names.reserve(size);

    std::size_t cnt = 0;
    for (auto seg = segment_map.cbegin(); seg != segment_map.cend(); ++seg) {
        if (seg->second.segment.getJoint().getType() == KDL::Joint::None) {
            continue;
        }

        const urdf::JointConstSharedPtr joint = robot_model.getJoint(seg->second.segment.getJoint().getName().c_str());
        // check, if joint can be found in the URDF model of the robot
        if (!joint) {
            std::cerr << "Error: Joint " << joint->name.c_str() << "has not been found in the URDF robot model! Aborting ..." <<  std::endl;
            return false;
        }
        // extract joint information
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            double lower = 0, upper = 0;

            if (joint->type != urdf::Joint::CONTINUOUS) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
            }

            names.push_back(joint->name);
            min(cnt) = lower;
            max(cnt) = upper;
            max_v(cnt) = (joint->limits && joint->limits->velocity) ? joint->limits->velocity : 0;
            cur(cnt) = 0.0;
            // Debug:
//                std::cerr << "Jnt " << names[cnt] << " min|max: " << min(cnt) << "|" << max(cnt) << "\n";
            cnt++;
        }
    }

    return true;
}


#endif // JNTINFO_H
