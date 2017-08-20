#ifndef KINECTBVH_H
#define KINECTBVH_H

#define SCALE 1.0f
#define FPS 0.033333

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include "vec_math.h"
#include "catmull_rom.h"

using namespace std;
using namespace Vec_Math;

// Joint type.
typedef enum {
    JOINT_HEAD,
    JOINT_NECK,
    
    JOINT_LEFT_SHOULDER,
    JOINT_RIGHT_SHOULDER,
    JOINT_LEFT_ELBOW,
    JOINT_RIGHT_ELBOW,
    JOINT_LEFT_HAND,
    JOINT_RIGHT_HAND,
    
    JOINT_TORSO,
    
    JOINT_LEFT_HIP,
    JOINT_RIGHT_HIP,
    JOINT_LEFT_KNEE,
    JOINT_RIGHT_KNEE,
    JOINT_LEFT_FOOT,
    JOINT_RIGHT_FOOT,
    JOINT_SIZE
} JointType;

// Joint.
typedef struct Joint {
    Joint() : tracked(false) {}
    Quaternion quat;
    Vec3 pos;
    bool tracked;
} Joint;

// The most important class.
class KinectBVH {
public:
    // Constructor.
    KinectBVH() {
        // Generate parent joint map.
        parent_joint_map[JOINT_TORSO] = JOINT_TORSO;
        parent_joint_map[JOINT_NECK] = JOINT_TORSO;
        parent_joint_map[JOINT_HEAD] = JOINT_NECK;
        parent_joint_map[JOINT_LEFT_SHOULDER] = JOINT_NECK;
        parent_joint_map[JOINT_LEFT_ELBOW] = JOINT_LEFT_SHOULDER;
        parent_joint_map[JOINT_LEFT_HAND] = JOINT_LEFT_ELBOW;
        parent_joint_map[JOINT_RIGHT_SHOULDER] = JOINT_NECK;
        parent_joint_map[JOINT_RIGHT_ELBOW] = JOINT_RIGHT_SHOULDER;
        parent_joint_map[JOINT_RIGHT_HAND] = JOINT_RIGHT_ELBOW;
        parent_joint_map[JOINT_LEFT_HIP] = JOINT_TORSO;
        parent_joint_map[JOINT_LEFT_KNEE] = JOINT_LEFT_HIP;
        parent_joint_map[JOINT_LEFT_FOOT] = JOINT_LEFT_KNEE;
        parent_joint_map[JOINT_RIGHT_HIP] = JOINT_TORSO;
        parent_joint_map[JOINT_RIGHT_KNEE] = JOINT_RIGHT_HIP;
        parent_joint_map[JOINT_RIGHT_FOOT] = JOINT_RIGHT_KNEE;
    }
    
    // Destructor.
    ~KinectBVH() {
    }
    
    // Initial and generate 'T' pose skeleton.
    void CalibrateSkeleton() {
        // Clean data.
        m_nbFrame = 0;
        m_aOffsets.clear();
        m_vJointsOrientation.clear();
        
        // Hard code the 'T' pose skeleton.
        Vec3 offsets[JOINT_SIZE];
        Vec3 offset;
        
        offset.x = 0.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_TORSO] = offset;
        
        offset.x = 0.0f;
        offset.y = 43.63f;
        offset.z = 0.0f;
        offsets[JOINT_NECK] = offset;
        
        offset.x = 0.0f;
        offset.y = 18.49f;
        offset.z = 0.0f;
        offsets[JOINT_HEAD] = offset;
        
        offset.x = -14.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_LEFT_SHOULDER] = offset;
        
        offset.x = -25.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_LEFT_ELBOW] = offset;
        
        offset.x = -23.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_LEFT_HAND] = offset;
        
        offset.x = 14.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_RIGHT_SHOULDER] = offset;
        
        offset.x = 25.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_RIGHT_ELBOW] = offset;
        
        offset.x = 23.0f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_RIGHT_HAND] = offset;
        
        offset.x = -9.52f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_LEFT_HIP] = offset;
        
        offset.x = 0.0f;
        offset.y = -37.32f;
        offset.z = 0.0f;
        offsets[JOINT_LEFT_KNEE] = offset;
        
        offset.x = 0.0f;
        offset.y = -34.6f;
        offset.z = 0.0f;
        offsets[JOINT_LEFT_FOOT] = offset;
        
        offset.x = 9.52f;
        offset.y = 0.0f;
        offset.z = 0.0f;
        offsets[JOINT_RIGHT_HIP] = offset;
        
        offset.x = 0.0f;
        offset.y = -37.32f;
        offset.z = 0.0f;
        offsets[JOINT_RIGHT_KNEE] = offset;
        
        offset.x = 0.0f;
        offset.y = -34.6f;
        offset.z = 0.0f;
        offsets[JOINT_RIGHT_FOOT] = offset;
        
        // Add joints offset data.
        for (int i = 0; i < JOINT_SIZE; i++) {
            AddOffset(offsets[i]);
        }
    }
    
    // Add the positions of all joints.
    void AddAllJointsPosition(const Joint* joints) {
        for (int i = 0; i < JOINT_SIZE; i++) {
            m_vJointsOrientation.push_back(joints[i]);
        }
    }
    
    // Add the frame counter.
    void IncrementNbFrames() { ++m_nbFrame; }
    
    // Create the file, batch generate motion capture data, save to file, close the file.
    void SaveToBVHFile(const string& filename) {
        m_pFile.open(filename.c_str());
        if (m_pFile.is_open()) {
            FilterPositions();
            CorrectAngle(12.5f);
            CreateQuaternionInformation();
            CreateSkeletonInformation();
            CreateMotionInformation();
            m_pFile.close();
        }
    }
    
private:
    // Frame counter.
    int m_nbFrame;
    // The relative offset to it parent.
    vector<Vec3> m_aOffsets;
    // The positions and rotations of every frame.
    vector<Joint> m_vJointsOrientation;
    // Output file.
    ofstream m_pFile;
    // Parent joint map.
    JointType parent_joint_map[JOINT_SIZE];
    
    // Add the relative offset to it parent.
    void AddOffset(const Vec3& offset) {
        Vec3 one_offset;
        one_offset.x = offset.x * SCALE;
        one_offset.y = offset.y * SCALE;
        one_offset.z = offset.z * SCALE;
        m_aOffsets.push_back(one_offset);
    }
    
    // Write the motion capture data of a joint.
    void WriteJoint(stringstream& flux, const Joint* joints, const int idx) {
        Vec3 angles = GetEulers(joints, idx);
        flux << angles.z * kRadToDeg << " " << angles.y * kRadToDeg << " "
        << angles.x * kRadToDeg << " ";
    }
    
    // Calculate the Euler angle of joint's relative rotation to its parent.
    Vec3 GetEulers(const Joint* joints, const int idx) {
        // Get the quaternion of its parent.
        Quaternion q_parent;
        if (idx == JOINT_TORSO) {
            q_parent = quat_identity;
        } else {
            q_parent = vec4_create(joints[parent_joint_map[idx]].quat.x,
                                   joints[parent_joint_map[idx]].quat.y,
                                   joints[parent_joint_map[idx]].quat.z,
                                   joints[parent_joint_map[idx]].quat.w);
        }
        
        // Get the quaternion of the joint.
        Quaternion q_current = vec4_create(joints[idx].quat.x, joints[idx].quat.y,
                                           joints[idx].quat.z, joints[idx].quat.w);
        
        // Calculate the relative quaternion.
        Quaternion q_delta = quat_left_multiply(q_current, quat_inverse(q_parent));
        
        // Convert to Euler angle, roll->yaw->pitch order, which roll is outer, pitch is inner.
        Vec3 angle = euler_from_quat(q_delta);
        
        return angle;
    }
    
    // Generate 'T' pose skeleton and save to file.
    void CreateSkeletonInformation() {
        stringstream flux;
        
        // ROOT
        flux << "HIERARCHY" << endl;
        flux << "ROOT Hip" << endl;
        flux << "{" << endl;
        
        // Spine
        flux << "\tOFFSET " << m_aOffsets[JOINT_TORSO].x << " "
        << m_aOffsets[JOINT_TORSO].y << " " << m_aOffsets[JOINT_TORSO].z
        << endl;
        flux << "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation "
        "Xrotation"
        << endl;
        flux << "\tJOINT ShoulderCenter" << endl;
        flux << "\t{" << endl;
        // Head
        flux << "\t\tOFFSET " << m_aOffsets[JOINT_NECK].x << " "
        << m_aOffsets[JOINT_NECK].y << " " << m_aOffsets[JOINT_NECK].z << endl;
        flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\tJOINT Head" << endl;
        flux << "\t\t{" << endl;
        // End Site
        flux << "\t\t\tOFFSET " << m_aOffsets[JOINT_HEAD].x << " "
        << m_aOffsets[JOINT_HEAD].y << " " << m_aOffsets[JOINT_HEAD].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tEnd Site" << endl;
        flux << "\t\t\t{" << endl;
        flux << "\t\t\t\tOFFSET 0.0 " << 8.91 << " 0.0" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        
        // Shoulder Left
        flux << "\t\tJOINT ShoulderLeft" << endl;
        flux << "\t\t{" << endl;
        // Elbow Left
        flux << "\t\t\tOFFSET " << m_aOffsets[JOINT_LEFT_SHOULDER].x << " "
        << m_aOffsets[JOINT_LEFT_SHOULDER].y << " "
        << m_aOffsets[JOINT_LEFT_SHOULDER].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT ElbowLeft" << endl;
        flux << "\t\t\t{" << endl;
        // Wrist Left
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JOINT_LEFT_ELBOW].x << " "
        << m_aOffsets[JOINT_LEFT_ELBOW].y << " "
        << m_aOffsets[JOINT_LEFT_ELBOW].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tJOINT WristLeft" << endl;
        flux << "\t\t\t\t{" << endl;
        // Hand Left
        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[JOINT_LEFT_HAND].x << " "
        << m_aOffsets[JOINT_LEFT_HAND].y << " "
        << m_aOffsets[JOINT_LEFT_HAND].z << endl;
        flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t\t{" << endl;
        flux << "\t\t\t\t\t\tOFFSET -8.32 0.0 0.0" << endl;
        flux << "\t\t\t\t\t}" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        
        // Shoulder Right
        flux << "\t\tJOINT ShoulderRight" << endl;
        flux << "\t\t{" << endl;
        // Elbow Right
        flux << "\t\t\tOFFSET " << m_aOffsets[JOINT_RIGHT_SHOULDER].x << " "
        << m_aOffsets[JOINT_RIGHT_SHOULDER].y << " "
        << m_aOffsets[JOINT_RIGHT_SHOULDER].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT ElbowRight" << endl;
        flux << "\t\t\t{" << endl;
        // Wrist Right
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JOINT_RIGHT_ELBOW].x << " "
        << m_aOffsets[JOINT_RIGHT_ELBOW].y << " "
        << m_aOffsets[JOINT_RIGHT_ELBOW].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tJOINT WristRight" << endl;
        flux << "\t\t\t\t{" << endl;
        // Hand Right
        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[JOINT_RIGHT_HAND].x << " "
        << m_aOffsets[JOINT_RIGHT_HAND].y << " "
        << m_aOffsets[JOINT_RIGHT_HAND].z << endl;
        flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t\t{" << endl;
        flux << "\t\t\t\t\t\tOFFSET 8.32 0.0 0.0" << endl;
        flux << "\t\t\t\t\t}" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        
        flux << "\t}" << endl;
        
        // Hip Left
        flux << "\tJOINT HipLeft" << endl;
        flux << "\t{" << endl;
        
        // Knee Left
        flux << "\t\tOFFSET " << m_aOffsets[JOINT_LEFT_HIP].x << " "
        << m_aOffsets[JOINT_LEFT_HIP].y << " " << m_aOffsets[JOINT_LEFT_HIP].z
        << endl;
        flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\tJOINT KneeLeft" << endl;
        flux << "\t\t{" << endl;
        
        // Ankle Left
        flux << "\t\t\tOFFSET " << m_aOffsets[JOINT_LEFT_KNEE].x << " "
        << m_aOffsets[JOINT_LEFT_KNEE].y << " "
        << m_aOffsets[JOINT_LEFT_KNEE].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT AnkleLeft" << endl;
        flux << "\t\t\t{" << endl;
        
        // Foot Left
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JOINT_LEFT_FOOT].x << " "
        << m_aOffsets[JOINT_LEFT_FOOT].y << " "
        << m_aOffsets[JOINT_LEFT_FOOT].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t{" << endl;
        flux << "\t\t\t\t\tOFFSET 0.0 0.0 8.91" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        flux << "\t}" << endl;
        
        // Hip Right
        flux << "\tJOINT HipRight" << endl;
        flux << "\t{" << endl;
        
        // Knee Right
        flux << "\t\tOFFSET " << m_aOffsets[JOINT_RIGHT_HIP].x << " "
        << m_aOffsets[JOINT_RIGHT_HIP].y << " "
        << m_aOffsets[JOINT_RIGHT_HIP].z << endl;
        flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\tJOINT KneeRight" << endl;
        flux << "\t\t{" << endl;
        
        // Ankle Right
        flux << "\t\t\tOFFSET " << m_aOffsets[JOINT_RIGHT_KNEE].x << " "
        << m_aOffsets[JOINT_RIGHT_KNEE].y << " "
        << m_aOffsets[JOINT_RIGHT_KNEE].z << endl;
        flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\tJOINT AnkleRight" << endl;
        flux << "\t\t\t{" << endl;
        
        // Foot Right
        flux << "\t\t\t\tOFFSET " << m_aOffsets[JOINT_RIGHT_FOOT].x << " "
        << m_aOffsets[JOINT_RIGHT_FOOT].y << " "
        << m_aOffsets[JOINT_RIGHT_FOOT].z << endl;
        flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
        flux << "\t\t\t\tEnd Site" << endl;
        flux << "\t\t\t\t{" << endl;
        flux << "\t\t\t\t\tOFFSET 0.0 0.0 8.91" << endl;
        flux << "\t\t\t\t}" << endl;
        flux << "\t\t\t}" << endl;
        flux << "\t\t}" << endl;
        flux << "\t}" << endl;
        
        flux << "}" << endl;
        
        m_pFile << flux.str();
    }
    
    // Generate motion capture data and save to file.
    void CreateMotionInformation() {
        stringstream flux;
        
        flux << "MOTION" << endl;
        flux << "Frames: " << m_nbFrame << endl;
        flux << "Frame Time: " << FPS << endl;
        
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE); i++) {
            // The position of the root joint in centimeter, as the unit in Freenect is millimeter, we multiple it 0.1.
            Joint* joints = &m_vJointsOrientation[i * JOINT_SIZE];
            flux << joints[JOINT_TORSO].pos.x * SCALE * 0.1f << " " << joints[JOINT_TORSO].pos.y * SCALE * 0.1f << " "
            << joints[JOINT_TORSO].pos.z * SCALE * 0.1f << " ";
            
            // Write the Euler angle of every joint(ZYX).
            WriteJoint(flux, joints, JOINT_TORSO);
            WriteJoint(flux, joints, JOINT_NECK);
            WriteJoint(flux, joints, JOINT_HEAD);
            WriteJoint(flux, joints, JOINT_LEFT_SHOULDER);
            WriteJoint(flux, joints, JOINT_LEFT_ELBOW);
            WriteJoint(flux, joints, JOINT_LEFT_HAND);
            WriteJoint(flux, joints, JOINT_RIGHT_SHOULDER);
            WriteJoint(flux, joints, JOINT_RIGHT_ELBOW);
            WriteJoint(flux, joints, JOINT_RIGHT_HAND);
            WriteJoint(flux, joints, JOINT_LEFT_HIP);
            WriteJoint(flux, joints, JOINT_LEFT_KNEE);
            WriteJoint(flux, joints, JOINT_LEFT_FOOT);
            WriteJoint(flux, joints, JOINT_RIGHT_HIP);
            WriteJoint(flux, joints, JOINT_RIGHT_KNEE);
            WriteJoint(flux, joints, JOINT_RIGHT_FOOT);
            
            flux << endl;
        }
        
        m_pFile << flux.str();
    }
    
    // Correct the pitch angle of the camera.
    void CorrectAngle(const float& kinect_angle) {
        // Calculate the invert rotation matrix.
        Mat3 correct_matrix = mat3_rotation_x(-kinect_angle * kDegToRad);
        
        // Rotate the position for every joint.
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size()); i++) {
            m_vJointsOrientation[i].pos = mat3_mul_vector(m_vJointsOrientation[i].pos, correct_matrix);
        }
    }
    
    // Generate quaternions for a set of joints.
    void CreateQuaternionInformation() {
        // If the arms are not standard 'T' pose, you may set an offset angle.
        const float arm_angle = 0.0f;
        const float arm_angle_scaler = (arm_angle + 90.0f) / 90.0f;
        
        // we save last stable x axis for each joint to avoid trembling
        Vec3 last_stable_vx[JOINT_SIZE];
        for (int i = 0; i < JOINT_SIZE; i++) {
            last_stable_vx[i] = vec3_zero;
        }
        
        // loop through all records
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size()/ JOINT_SIZE); i++) {
            Joint* joints = &m_vJointsOrientation[i * JOINT_SIZE];
            
            const float MAX_STABLE_DOT = 0.925f;
            float dot;
            Vec3 p1, p2;
            Vec3 v1, v2;
            Vec3 vx, vy, vz;
            Vec3 v_body_x;
            Mat3 m, mr;
            Quaternion q;
            
            // JOINT_TORSO
            p1 = joints[JOINT_LEFT_HIP].pos;
            p2 = joints[JOINT_RIGHT_HIP].pos;
            vx = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_TORSO].pos;
            p2 = joints[JOINT_NECK].pos;
            vy = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            q = quat_from_mat3(m);
            joints[JOINT_TORSO].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // save body's axis x for later use
            v_body_x = vx;
            
            // JOINT_NECK
            p1 = joints[JOINT_LEFT_SHOULDER].pos;
            p2 = joints[JOINT_RIGHT_SHOULDER].pos;
            vx = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_NECK].pos;
            p2 = joints[JOINT_HEAD].pos;
            vy = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            q = quat_from_mat3(m);
            joints[JOINT_NECK].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_HEAD
            joints[JOINT_HEAD].quat = joints[JOINT_NECK].quat;
            
            // JOINT_LEFT_SHOULDER
            p1 = joints[JOINT_LEFT_SHOULDER].pos;
            p2 = joints[JOINT_LEFT_ELBOW].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_LEFT_ELBOW].pos;
            p2 = joints[JOINT_LEFT_HAND].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JOINT_LEFT_SHOULDER];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JOINT_LEFT_SHOULDER] = vx;
            }
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_LEFT_SHOULDER].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_ELBOW
            p1 = joints[JOINT_LEFT_SHOULDER].pos;
            p2 = joints[JOINT_LEFT_ELBOW].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_LEFT_ELBOW].pos;
            p2 = joints[JOINT_LEFT_HAND].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JOINT_LEFT_ELBOW];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JOINT_LEFT_ELBOW] = vx;
            }
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_LEFT_ELBOW].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_HAND
            joints[JOINT_LEFT_HAND].quat = joints[JOINT_LEFT_ELBOW].quat;
            
            // JOINT_RIGHT_SHOULDER
            p1 = joints[JOINT_RIGHT_SHOULDER].pos;
            p2 = joints[JOINT_RIGHT_ELBOW].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_RIGHT_ELBOW].pos;
            p2 = joints[JOINT_RIGHT_HAND].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JOINT_RIGHT_SHOULDER];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JOINT_RIGHT_SHOULDER] = vx;
            }
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(-kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_RIGHT_SHOULDER].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_ELBOW
            p1 = joints[JOINT_RIGHT_SHOULDER].pos;
            p2 = joints[JOINT_RIGHT_ELBOW].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_RIGHT_ELBOW].pos;
            p2 = joints[JOINT_RIGHT_HAND].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            if (fabsf(dot) > MAX_STABLE_DOT) {
                vx = last_stable_vx[JOINT_RIGHT_ELBOW];
            } else {
                vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
                last_stable_vx[JOINT_RIGHT_ELBOW] = vx;
            }
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(-kPiDiv2 * arm_angle_scaler));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_RIGHT_ELBOW].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_HAND
            joints[JOINT_RIGHT_HAND].quat = joints[JOINT_RIGHT_ELBOW].quat;
            
            // JOINT_LEFT_HIP
            p1 = joints[JOINT_LEFT_HIP].pos;
            p2 = joints[JOINT_LEFT_KNEE].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_LEFT_KNEE].pos;
            p2 = joints[JOINT_LEFT_FOOT].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            vx = vec3_negate(vx);
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_LEFT_HIP].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_KNEE
            p1 = joints[JOINT_LEFT_HIP].pos;
            p2 = joints[JOINT_LEFT_KNEE].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_LEFT_KNEE].pos;
            p2 = joints[JOINT_LEFT_FOOT].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            vx = vec3_negate(vx);
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_LEFT_KNEE].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_LEFT_FOOT
            joints[JOINT_LEFT_FOOT].quat = joints[JOINT_LEFT_KNEE].quat;
            
            // JOINT_RIGHT_HIP
            p1 = joints[JOINT_RIGHT_HIP].pos;
            p2 = joints[JOINT_RIGHT_KNEE].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_RIGHT_KNEE].pos;
            p2 = joints[JOINT_RIGHT_FOOT].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            vx = vec3_negate(vx);
            vy = v1;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_RIGHT_HIP].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_KNEE
            p1 = joints[JOINT_RIGHT_HIP].pos;
            p2 = joints[JOINT_RIGHT_KNEE].pos;
            v1 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            p1 = joints[JOINT_RIGHT_KNEE].pos;
            p2 = joints[JOINT_RIGHT_FOOT].pos;
            v2 = vec3_create(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            dot = vec3_dot(vec3_normalize(v1), vec3_normalize(v2));
            vx = vec3_cross(vec3_normalize(v1), vec3_normalize(v2));
            // constrain to body's axis x
            vx = vec3_add(vec3_mul_scalar(vec3_normalize(v_body_x), dot),
                          vec3_mul_scalar(vec3_normalize(vx), 1 - dot));
            // reverse the direction because knees can only bend to back
            vx = vec3_negate(vx);
            vy = v2;
            vz = vec3_zero;
            m = mat3_from_axis(vx, vy, vz);
            // inverse bind pose
            mr = mat3_inverse(mat3_rotation_z(kPi));
            m = mat3_multiply(mr, m);
            q = quat_from_mat3(m);
            joints[JOINT_RIGHT_KNEE].quat = vec4_create(q.x, q.y, q.z, q.w);
            
            // JOINT_RIGHT_FOOT
            joints[JOINT_RIGHT_FOOT].quat = joints[JOINT_RIGHT_KNEE].quat;
        }
    }
    // Recover lost positions, apply median filter.
    void FilterPositions() {
        // slerp positions lack in confidence
        int last_tracked_indices[JOINT_SIZE];
        bool last_tracked_status[JOINT_SIZE];
        // init all tracked indices to invalid value
        for (int j = 0; j < JOINT_SIZE; j++) {
            last_tracked_indices[j] = -1;
            last_tracked_status[j] = false;
        }
        
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE); i++) {
            Joint* joints = &m_vJointsOrientation[i * JOINT_SIZE];
            for (int j = 0; j < JOINT_SIZE; j++) {
                int index = i * JOINT_SIZE + j;
                // when lost tracking (--|__)
                if (last_tracked_status[j] != false && joints[j].tracked == false) {
                    last_tracked_status[j] = false;
                }
                // when restore tracking (__|--)
                if (last_tracked_indices[j] >= 0 && last_tracked_status[j] == false &&
                    joints[j].tracked != false) {
                    // lerp lost positions
                    int last_tracked_index = last_tracked_indices[j];
                    int current_tracked_index = index;
                    // start point and end point
                    Vec3 p1 = m_vJointsOrientation[last_tracked_index].pos;
                    Vec3 p2 = m_vJointsOrientation[current_tracked_index].pos;

                    // test if we can use better catmull-rom algorithm, otherwise we use stable linear algorithm.
                    int cat_head_index = last_tracked_index - JOINT_SIZE * 2;
                    int cat_tail_index = current_tracked_index + JOINT_SIZE * 2;
                    bool catmull_rom = (cat_head_index >= 0 &&
                                            m_vJointsOrientation[cat_head_index].tracked &&
                                            cat_tail_index < static_cast<int>(m_vJointsOrientation.size()) &&
                                            m_vJointsOrientation[cat_tail_index].tracked);
                    if (catmull_rom) {
                        Vec3 p0 = m_vJointsOrientation[cat_head_index].pos;
                        Vec3 p3 = m_vJointsOrientation[cat_tail_index].pos;
                        CubicPoly px, py, pz;
                        InitCentripetalCR(p0, p1, p2, p3,
                                          2.0f, (float)(current_tracked_index - last_tracked_index) / JOINT_SIZE, 2.0f,
                                          px, py, pz);
                        for (int k = last_tracked_index + JOINT_SIZE; k < current_tracked_index; k += JOINT_SIZE) {
                            float t = (float)(k - last_tracked_index) / (current_tracked_index - last_tracked_index);
                            m_vJointsOrientation[k].pos.x = px.eval(t);
                            m_vJointsOrientation[k].pos.y = py.eval(t);
                            m_vJointsOrientation[k].pos.z = pz.eval(t);
                        }
                    } else {
                        for (int k = last_tracked_index + JOINT_SIZE; k < current_tracked_index; k += JOINT_SIZE) {
                            float t = (float)(k - last_tracked_index) / (current_tracked_index - last_tracked_index);
                            m_vJointsOrientation[k].pos.x = p1.x * (1.0f - t) + p2.x * t;
                            m_vJointsOrientation[k].pos.y = p1.y * (1.0f - t) + p2.y * t;
                            m_vJointsOrientation[k].pos.z = p1.z * (1.0f - t) + p2.z * t;
                        }
                    }
                }
                // when tracked, save track index and status
                if (joints[j].tracked != false) {
                    last_tracked_indices[j] = index;
                    last_tracked_status[j] = joints[j].tracked;
                }
            }
        }
        
        // calculate median filter
        const int filter_radius = 2;
        int min_k = 0;
        int max_k = static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE - 1) * JOINT_SIZE;
        vector<float> temp_positions;
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size() / JOINT_SIZE); i++) {
            for (int j = 0; j < JOINT_SIZE; j++) {
                vector<float> px, py, pz;
                
                int index = i * JOINT_SIZE + j;
                for (int k = index - filter_radius * JOINT_SIZE;
                     k <= index + filter_radius * JOINT_SIZE; k += JOINT_SIZE) {
                    if (k - j < min_k) {
                        px.push_back(m_vJointsOrientation[min_k + j].pos.x);
                        py.push_back(m_vJointsOrientation[min_k + j].pos.y);
                        pz.push_back(m_vJointsOrientation[min_k + j].pos.z);
                    } else if (k - j > max_k) {
                        px.push_back(m_vJointsOrientation[max_k + j].pos.x);
                        py.push_back(m_vJointsOrientation[max_k + j].pos.y);
                        pz.push_back(m_vJointsOrientation[max_k + j].pos.z);
                    } else {
                        px.push_back(m_vJointsOrientation[k].pos.x);
                        py.push_back(m_vJointsOrientation[k].pos.y);
                        pz.push_back(m_vJointsOrientation[k].pos.z);
                    }
                }
                
                sort(px.begin(), px.end());
                sort(py.begin(), py.end());
                sort(pz.begin(), pz.end());
                
                temp_positions.push_back(px[filter_radius]);
                temp_positions.push_back(py[filter_radius]);
                temp_positions.push_back(pz[filter_radius]);
            }
        }
        // apply median filter
        for (int i = 0; i < static_cast<int>(m_vJointsOrientation.size()); i++) {
            float* positions = &temp_positions[i * 3];
            m_vJointsOrientation[i].pos.x = positions[0];
            m_vJointsOrientation[i].pos.y = positions[1];
            m_vJointsOrientation[i].pos.z = positions[2];
        }
    }
};

#endif // KINECTBVH_H
