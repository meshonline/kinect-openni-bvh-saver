#include "kinectbvh.h"

/**
* Constructeur
*/
KinectBVH::KinectBVH()
{
    // fill parent joint map
    parent_joint_map[nite::JOINT_TORSO] = nite::JOINT_TORSO;
    parent_joint_map[nite::JOINT_NECK] = nite::JOINT_TORSO;
    parent_joint_map[nite::JOINT_HEAD] = nite::JOINT_NECK;
    parent_joint_map[nite::JOINT_LEFT_SHOULDER] = nite::JOINT_NECK;
    parent_joint_map[nite::JOINT_LEFT_ELBOW] = nite::JOINT_LEFT_SHOULDER;
    parent_joint_map[nite::JOINT_LEFT_HAND] = nite::JOINT_LEFT_ELBOW;
    parent_joint_map[nite::JOINT_RIGHT_SHOULDER] = nite::JOINT_NECK;
    parent_joint_map[nite::JOINT_RIGHT_ELBOW] = nite::JOINT_RIGHT_SHOULDER;
    parent_joint_map[nite::JOINT_RIGHT_HAND] = nite::JOINT_RIGHT_ELBOW;
    parent_joint_map[nite::JOINT_LEFT_HIP] = nite::JOINT_TORSO;
    parent_joint_map[nite::JOINT_LEFT_KNEE] = nite::JOINT_LEFT_HIP;
    parent_joint_map[nite::JOINT_LEFT_FOOT] = nite::JOINT_LEFT_KNEE;
    parent_joint_map[nite::JOINT_RIGHT_HIP] = nite::JOINT_TORSO;
    parent_joint_map[nite::JOINT_RIGHT_KNEE] = nite::JOINT_RIGHT_HIP;
    parent_joint_map[nite::JOINT_RIGHT_FOOT] = nite::JOINT_RIGHT_KNEE;
}

/**
* Destructeur
*/
KinectBVH::~KinectBVH()
{
	if (m_pFile.is_open())
	{
		m_pFile.close();
	}
}

/**
* Ajoute un offset ?la description du BVH
*/
void KinectBVH::AddOffset(int i, const nite::Point3f &offset)
{
	nite::Point3f one_offset;
	one_offset.x = offset.x * SCALE;
	one_offset.y = offset.y * SCALE;
	one_offset.z = offset.z * SCALE;
	m_aOffsets.push_back(one_offset);
}

/**
* Créé un nouveau fichier en fonction du nom reçu en paramètre, renvoi true si réussi sinon false
*/
bool KinectBVH::CreateBVHFile(string filename)
{
	m_nbFrame = 0;

	m_aOffsets.clear();
	m_vPositions.clear();
	m_vBonesOrientation.clear();

	if (m_pFile.is_open())
	{
		m_pFile.close();
	}

	m_pFile.open(filename.c_str());

	return m_pFile.is_open();
}

/**
* Génère le fichier BVH
*/
void KinectBVH::FillBVHFile()
{
    const bool use_built_in_quaternion = false;
    if (!use_built_in_quaternion) {
        FilterPositions();
        CorrectAngle();
        CreateQuaternionInformation();
    }
	CreateSkeletonInformation();
	CreateMotionInformation();
	m_pFile.close();
}

/**
* Génère la description du squelette pour le BVH
*/
void KinectBVH::CreateSkeletonInformation()
{
	stringstream flux;

	// ROOT
	flux << "HIERARCHY" << endl;
	flux << "ROOT Hip" << endl;
	flux << "{" << endl;

		// Spine
    flux << "\tOFFSET " << m_aOffsets[nite::JOINT_TORSO].x << " " << m_aOffsets[nite::JOINT_TORSO].y << " " << m_aOffsets[nite::JOINT_TORSO].z << endl;
		flux << "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation" << endl;
			flux << "\t\tJOINT ShoulderCenter" << endl;
			flux << "\t\t{" << endl;
				// Head
				flux << "\t\t\tOFFSET " << m_aOffsets[nite::JOINT_NECK].x << " " << m_aOffsets[nite::JOINT_NECK].y << " " << m_aOffsets[nite::JOINT_NECK].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
				flux << "\t\t\tJOINT Head" << endl;
				flux << "\t\t\t{" << endl;
					// End Site
					flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_HEAD].x << " " << m_aOffsets[nite::JOINT_HEAD].y << " " << m_aOffsets[nite::JOINT_HEAD].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tEnd Site" << endl;
					flux << "\t\t\t\t{" << endl;
						flux << "\t\t\t\t\tOFFSET 0.0 " << 8.91 << " 0.0" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;

				// Shoulder Left
				flux << "\t\t\tJOINT ShoulderLeft" << endl;
				flux << "\t\t\t{" << endl;
					// Elbow Left
                    flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_SHOULDER].x << " " << m_aOffsets[nite::JOINT_LEFT_SHOULDER].y << " " << m_aOffsets[nite::JOINT_LEFT_SHOULDER].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowLeft" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Left
                        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_ELBOW].x << " " << m_aOffsets[nite::JOINT_LEFT_ELBOW].y << " " << m_aOffsets[nite::JOINT_LEFT_ELBOW].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristLeft" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Left
                            flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_HAND].x << " " << m_aOffsets[nite::JOINT_LEFT_HAND].y << " " << m_aOffsets[nite::JOINT_LEFT_HAND].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
								flux << "\t\t\t\t\t\t\tEnd Site" << endl;
								flux << "\t\t\t\t\t\t\t{" << endl;
									flux << "\t\t\t\t\t\t\t\tOFFSET -8.32 0.0 0.0" << endl;
								flux << "\t\t\t\t\t\t\t}" << endl;
						flux << "\t\t\t\t\t}" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;

				// Shoulder Right
				flux << "\t\t\tJOINT ShoulderRight" << endl;
				flux << "\t\t\t{" << endl;
					// Elbow Right
                    flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_SHOULDER].x << " " << m_aOffsets[nite::JOINT_RIGHT_SHOULDER].y << " " << m_aOffsets[nite::JOINT_RIGHT_SHOULDER].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowRight" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Right
                        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_ELBOW].x << " " << m_aOffsets[nite::JOINT_RIGHT_ELBOW].y << " " << m_aOffsets[nite::JOINT_RIGHT_ELBOW].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristRight" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Right
                            flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_HAND].x << " " << m_aOffsets[nite::JOINT_RIGHT_HAND].y << " " << m_aOffsets[nite::JOINT_RIGHT_HAND].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
								flux << "\t\t\t\t\t\t\tEnd Site" << endl;
								flux << "\t\t\t\t\t\t\t{" << endl;
									flux << "\t\t\t\t\t\t\t\tOFFSET 8.32 0.0 0.0" << endl;
								flux << "\t\t\t\t\t\t\t}" << endl;
						flux << "\t\t\t\t\t}" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;
			
			flux << "\t\t}" << endl;

		// Hip Left
		flux << "\tJOINT HipLeft" << endl;
		flux << "\t{" << endl;

			// Knee Left
        flux << "\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_HIP].x << " " << m_aOffsets[nite::JOINT_LEFT_HIP].y << " " << m_aOffsets[nite::JOINT_LEFT_HIP].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
			flux << "\t\tJOINT KneeLeft" << endl;
			flux << "\t\t{" << endl;

				// Ankle Left
				flux << "\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_KNEE].x << " " << m_aOffsets[nite::JOINT_LEFT_KNEE].y << " " << m_aOffsets[nite::JOINT_LEFT_KNEE].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
				flux << "\t\t\tJOINT AnkleLeft" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Left
                    flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_FOOT].x << " " << m_aOffsets[nite::JOINT_LEFT_FOOT].y << " " << m_aOffsets[nite::JOINT_LEFT_FOOT].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
						flux << "\t\t\t\t\tEnd Site" << endl;
						flux << "\t\t\t\t\t{" << endl;
							flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 8.91" << endl;
						flux << "\t\t\t\t\t}" << endl;	
				flux << "\t\t\t}" << endl;
			flux << "\t\t}" << endl;
		flux << "\t}" << endl;

		// Hip Right
		flux << "\tJOINT HipRight" << endl;
		flux << "\t{" << endl;

			// Knee Right
			flux << "\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_HIP].x << " " << m_aOffsets[nite::JOINT_RIGHT_HIP].y << " " << m_aOffsets[nite::JOINT_RIGHT_HIP].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
			flux << "\t\tJOINT KneeRight" << endl;
			flux << "\t\t{" << endl;

				// Ankle Right
				flux << "\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_KNEE].x << " " << m_aOffsets[nite::JOINT_RIGHT_KNEE].y << " " << m_aOffsets[nite::JOINT_RIGHT_KNEE].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
				flux << "\t\t\tJOINT AnkleRight" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Right
					flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_FOOT].x << " " << m_aOffsets[nite::JOINT_RIGHT_FOOT].y << " " << m_aOffsets[nite::JOINT_RIGHT_FOOT].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
						flux << "\t\t\t\t\tEnd Site" << endl;
						flux << "\t\t\t\t\t{" << endl;
							flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 8.91" << endl;
						flux << "\t\t\t\t\t}" << endl;	
				flux << "\t\t\t}" << endl;
			flux << "\t\t}" << endl;
		flux << "\t}" << endl;

	flux << "}" << endl;

	m_pFile << flux.str();
}

/**
* Incrémente le nombre de frames
*/
void KinectBVH::IncrementNbFrames()
{
	++m_nbFrame;
}

/**
* Ajoute un squelette et ses informations pour les données de la capture de mouvements
*/
void KinectBVH::AddBonesOrientation(KinectJoint *joints)
{
    for(int i = 0; i < (nite::JOINT_RIGHT_FOOT+1); i++) {
		m_vBonesOrientation.push_back(joints[i]);
	}
}

/**
* Ajoute une position du joint Hip Center pour les données de la capture de mouvements
*/
void KinectBVH::AddPosition(const nite::Point3f &position)
{
	nite::Point3f pos;
	pos.x = position.x * SCALE * 0.1f;
	pos.y = position.y * SCALE * 0.1f;
	pos.z = position.z * SCALE * 0.1f;
	m_vPositions.push_back(pos);
}

Vec_Math::Vec3 KinectBVH::GetEulers(KinectJoint *joints, int idx)
{
    // get parent's quaternion
    Vec_Math::Quaternion q_parent;
    if (idx == nite::JOINT_TORSO) {
        q_parent = Vec_Math::quat_identity;
    } else {
        q_parent = Vec_Math::vec4_create(joints[parent_joint_map[idx]].quat.x,
                                         joints[parent_joint_map[idx]].quat.y,
                                         joints[parent_joint_map[idx]].quat.z,
                                         joints[parent_joint_map[idx]].quat.w);
    }

    // get joint's quaternion
    Vec_Math::Quaternion q_current = Vec_Math::vec4_create(joints[idx].quat.x,
                                                           joints[idx].quat.y,
                                                           joints[idx].quat.z,
                                                           joints[idx].quat.w);

    // calculate between quaternion
	Vec_Math::Quaternion q_delta = Vec_Math::quat_left_multiply(q_current, Vec_Math::quat_inverse(q_parent));

	// convert the quaternion to euler angles by roll->yaw->pitch order, which roll is outer, pitch is inner.
    Vec_Math::Vec3 ret = Vec_Math::euler_from_quat(q_delta);

    // adjust kinect angle slightly
    if (idx == nite::JOINT_TORSO) {
//        ret.x -= 10.f * Vec_Math::kDegToRad;
    }
    
    return ret;
}

void KinectBVH::WriteJoint(stringstream& flux, KinectJoint *joints, nite::JointType j)
{
    Vec_Math::Vec3 angles = GetEulers(joints, j);
    flux << angles.z * Vec_Math::kRadToDeg << " " << angles.y * Vec_Math::kRadToDeg << " " << angles.x * Vec_Math::kRadToDeg << " ";
}

/**
* Génère les données des mouvements pour le BVH
*/
void KinectBVH::CreateMotionInformation()
{
	stringstream flux;

	flux << "\nMOTION" << endl;
	flux << "Frames: " << m_nbFrame << endl;
	flux << "Frame Time: " << FPS << endl;

	for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
		flux << m_vPositions[i].x << " " << m_vPositions[i].y << " " << -m_vPositions[i].z << " ";
        KinectJoint *joints = &m_vBonesOrientation[i * (nite::JOINT_RIGHT_FOOT+1)];

        WriteJoint(flux, joints, nite::JOINT_TORSO);
        WriteJoint(flux, joints, nite::JOINT_NECK);
        WriteJoint(flux, joints, nite::JOINT_HEAD);
        WriteJoint(flux, joints, nite::JOINT_LEFT_SHOULDER);
        WriteJoint(flux, joints, nite::JOINT_LEFT_ELBOW);
        WriteJoint(flux, joints, nite::JOINT_LEFT_HAND);
        WriteJoint(flux, joints, nite::JOINT_RIGHT_SHOULDER);
        WriteJoint(flux, joints, nite::JOINT_RIGHT_ELBOW);
        WriteJoint(flux, joints, nite::JOINT_RIGHT_HAND);
        WriteJoint(flux, joints, nite::JOINT_LEFT_HIP);
        WriteJoint(flux, joints, nite::JOINT_LEFT_KNEE);
        WriteJoint(flux, joints, nite::JOINT_LEFT_FOOT);
        WriteJoint(flux, joints, nite::JOINT_RIGHT_HIP);
        WriteJoint(flux, joints, nite::JOINT_RIGHT_KNEE);
        WriteJoint(flux, joints, nite::JOINT_RIGHT_FOOT);

        flux << endl;
	}

	m_pFile << flux.str();
}

void KinectBVH::CorrectAngle()
{
    // kinect's pitch angle
    const float kinect_angle = -5.f;
    Vec_Math::Mat3 correct_matrix = Vec_Math::mat3_rotation_x(kinect_angle * Vec_Math::kDegToRad);
    
    Vec_Math::Vec3 one_pos;
    for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
        one_pos.x = m_vPositions[i].x;
        one_pos.y = m_vPositions[i].y;
        one_pos.z = m_vPositions[i].z;

        one_pos = Vec_Math::mat3_mul_vector(one_pos, correct_matrix);

        m_vPositions[i].x = one_pos.x;
        m_vPositions[i].y = one_pos.y;
        m_vPositions[i].z = one_pos.z;

        KinectJoint *joints = &m_vBonesOrientation[i * (nite::JOINT_RIGHT_FOOT+1)];
        for (int j=0; j<nite::JOINT_RIGHT_FOOT+1; j++) {
            one_pos.x = joints[j].pos.x;
            one_pos.y = joints[j].pos.y;
            one_pos.z = joints[j].pos.z;

            one_pos = Vec_Math::mat3_mul_vector(one_pos, correct_matrix);

            joints[j].pos.x = one_pos.x;
            joints[j].pos.y = one_pos.y;
            joints[j].pos.z = one_pos.z;
        }
    }
}

void KinectBVH::CreateQuaternionInformation()
{
    // We correct special bind pose here
    const float arm_angle = 0.0f;
    const float arm_angle_scaler = (arm_angle + 90.0f) / 90.0f;
    
    // we save last stable x axis for each joint to avoid trembling
    Vec_Math::Vec3 last_stable_vx[nite::JOINT_RIGHT_FOOT+1];
    for (int i=0; i<nite::JOINT_RIGHT_FOOT+1; i++) {
        last_stable_vx[i] = Vec_Math::vec3_zero;
    }

    // loop through all records
    for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
        KinectJoint *joints = &m_vBonesOrientation[i * (nite::JOINT_RIGHT_FOOT+1)];

        const float MAX_STABLE_DOT = 0.925f;
        float dot;
        nite::Point3f p1, p2;
        Vec_Math::Vec3 v1, v2;
        Vec_Math::Vec3 vx, vy, vz;
        Vec_Math::Vec3 v_body_x;
        Vec_Math::Mat3 m, mr;
        Vec_Math::Quaternion q;

        // JOINT_TORSO
        p1 = joints[nite::JOINT_LEFT_HIP].pos;
        p2 = joints[nite::JOINT_RIGHT_HIP].pos;
        vx = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_TORSO].pos;
        p2 = joints[nite::JOINT_NECK].pos;
        vy = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_TORSO].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // save body's axis x for later use
        v_body_x = vx;
        
        // JOINT_NECK
        p1 = joints[nite::JOINT_LEFT_SHOULDER].pos;
        p2 = joints[nite::JOINT_RIGHT_SHOULDER].pos;
        vx = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_NECK].pos;
        p2 = joints[nite::JOINT_HEAD].pos;
        vy = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_NECK].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_HEAD
        joints[nite::JOINT_HEAD].quat = joints[nite::JOINT_NECK].quat;

        // JOINT_LEFT_SHOULDER
        p1 = joints[nite::JOINT_LEFT_SHOULDER].pos;
        p2 = joints[nite::JOINT_LEFT_ELBOW].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_LEFT_ELBOW].pos;
        p2 = joints[nite::JOINT_LEFT_HAND].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[nite::JOINT_LEFT_SHOULDER];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[nite::JOINT_LEFT_SHOULDER] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPiDiv2*arm_angle_scaler));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_LEFT_SHOULDER].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_LEFT_ELBOW
        p1 = joints[nite::JOINT_LEFT_SHOULDER].pos;
        p2 = joints[nite::JOINT_LEFT_ELBOW].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_LEFT_ELBOW].pos;
        p2 = joints[nite::JOINT_LEFT_HAND].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[nite::JOINT_LEFT_ELBOW];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[nite::JOINT_LEFT_ELBOW] = vx;
        }
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPiDiv2*arm_angle_scaler));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_LEFT_ELBOW].quat = nite::Quaternion(q.w, q.x, q.y, q.z);

        // JOINT_LEFT_HAND
        joints[nite::JOINT_LEFT_HAND].quat = joints[nite::JOINT_LEFT_ELBOW].quat;
        
        // JOINT_RIGHT_SHOULDER
        p1 = joints[nite::JOINT_RIGHT_SHOULDER].pos;
        p2 = joints[nite::JOINT_RIGHT_ELBOW].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_RIGHT_ELBOW].pos;
        p2 = joints[nite::JOINT_RIGHT_HAND].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[nite::JOINT_RIGHT_SHOULDER];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[nite::JOINT_RIGHT_SHOULDER] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(-Vec_Math::kPiDiv2*arm_angle_scaler));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_RIGHT_SHOULDER].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_RIGHT_ELBOW
        p1 = joints[nite::JOINT_RIGHT_SHOULDER].pos;
        p2 = joints[nite::JOINT_RIGHT_ELBOW].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_RIGHT_ELBOW].pos;
        p2 = joints[nite::JOINT_RIGHT_HAND].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[nite::JOINT_RIGHT_ELBOW];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[nite::JOINT_RIGHT_ELBOW] = vx;
        }
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(-Vec_Math::kPiDiv2*arm_angle_scaler));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_RIGHT_ELBOW].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_RIGHT_HAND
        joints[nite::JOINT_RIGHT_HAND].quat = joints[nite::JOINT_RIGHT_ELBOW].quat;
        
        // JOINT_LEFT_HIP
        p1 = joints[nite::JOINT_LEFT_HIP].pos;
        p2 = joints[nite::JOINT_LEFT_KNEE].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_LEFT_KNEE].pos;
        p2 = joints[nite::JOINT_LEFT_FOOT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        // constrain to body's axis x
        vx = Vec_Math::vec3_add(Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(v_body_x), dot), Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(vx), 1-dot));
        // reverse the direction because knees can only bend to back
        vx = Vec_Math::vec3_negate(vx);
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_LEFT_HIP].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_LEFT_KNEE
        p1 = joints[nite::JOINT_LEFT_HIP].pos;
        p2 = joints[nite::JOINT_LEFT_KNEE].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_LEFT_KNEE].pos;
        p2 = joints[nite::JOINT_LEFT_FOOT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        // constrain to body's axis x
        vx = Vec_Math::vec3_add(Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(v_body_x), dot), Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(vx), 1-dot));
        // reverse the direction because knees can only bend to back
        vx = Vec_Math::vec3_negate(vx);
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_LEFT_KNEE].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_LEFT_FOOT
        joints[nite::JOINT_LEFT_FOOT].quat = joints[nite::JOINT_LEFT_KNEE].quat;
        
        // JOINT_RIGHT_HIP
        p1 = joints[nite::JOINT_RIGHT_HIP].pos;
        p2 = joints[nite::JOINT_RIGHT_KNEE].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_RIGHT_KNEE].pos;
        p2 = joints[nite::JOINT_RIGHT_FOOT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        // constrain to body's axis x
        vx = Vec_Math::vec3_add(Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(v_body_x), dot), Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(vx), 1-dot));
        // reverse the direction because knees can only bend to back
        vx = Vec_Math::vec3_negate(vx);
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_RIGHT_HIP].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_RIGHT_KNEE
        p1 = joints[nite::JOINT_RIGHT_HIP].pos;
        p2 = joints[nite::JOINT_RIGHT_KNEE].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[nite::JOINT_RIGHT_KNEE].pos;
        p2 = joints[nite::JOINT_RIGHT_FOOT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
        // constrain to body's axis x
        vx = Vec_Math::vec3_add(Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(v_body_x), dot), Vec_Math::vec3_mul_scalar(Vec_Math::vec3_normalize(vx), 1-dot));
        // reverse the direction because knees can only bend to back
        vx = Vec_Math::vec3_negate(vx);
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        // inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
        joints[nite::JOINT_RIGHT_KNEE].quat = nite::Quaternion(q.w, q.x, q.y, q.z);
        
        // JOINT_RIGHT_FOOT
        joints[nite::JOINT_RIGHT_FOOT].quat = joints[nite::JOINT_RIGHT_KNEE].quat;
    }
}

void KinectBVH::FilterPositions()
{
    // slerp positions lack in confidence
    int last_tracked_indices[nite::JOINT_RIGHT_FOOT+1];
    bool last_tracked_status[nite::JOINT_RIGHT_FOOT+1];
    // init all tracked indices to invalid value
    for(int j = 0; j < nite::JOINT_RIGHT_FOOT+1; j++) {
        last_tracked_indices[j] = -1;
        last_tracked_status[j] = false;
    }
    
    for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
        KinectJoint *joints = &m_vBonesOrientation[i * (nite::JOINT_RIGHT_FOOT+1)];
        for(int j = 0; j < nite::JOINT_RIGHT_FOOT+1; j++) {
            // when lost tracking (--|__)
            if (last_tracked_status[j] != false && joints[j].tracked == false) {
                last_tracked_status[j] = false;
            }
            // when restore tracking (__|--)
            if (last_tracked_indices[j] >= 0 && last_tracked_status[j] == false && joints[j].tracked != false) {
                // lerp lost positions
                int last_tracked_index = last_tracked_indices[j];
                int current_tracked_index = i * (nite::JOINT_RIGHT_FOOT+1) + j;
                nite::Point3f p1 = m_vBonesOrientation[last_tracked_index].pos;
                nite::Point3f p2 = m_vBonesOrientation[current_tracked_index].pos;
                
                for (int k=last_tracked_index+(nite::JOINT_RIGHT_FOOT+1);
                     k<current_tracked_index;
                     k+=(nite::JOINT_RIGHT_FOOT+1)) {
                    float t = (float)(k-last_tracked_index)/(current_tracked_index-last_tracked_index);
                    m_vBonesOrientation[k].pos.x = p1.x * (1.0f - t) + p2.x * t;
                    m_vBonesOrientation[k].pos.y = p1.y * (1.0f - t) + p2.y * t;
                    m_vBonesOrientation[k].pos.z = p1.z * (1.0f - t) + p2.z * t;
                }
            }
            // when tracked, save track index and status
            if (joints[j].tracked != false) {
                last_tracked_indices[j] = i * (nite::JOINT_RIGHT_FOOT+1) + j;
                last_tracked_status[j] = joints[j].tracked;
            }
        }
    }
    
    // apply median filter to smooth positions
    const int filter_size = 5;
    for (int i = filter_size/2; i < static_cast<int>(m_vPositions.size())-filter_size/2; i++) {
        KinectJoint *joints = &m_vBonesOrientation[i * (nite::JOINT_RIGHT_FOOT+1)];
        for(int j = 0; j < nite::JOINT_RIGHT_FOOT+1; j++) {
            vector<float> px,py,pz;

            for(int k = (i * (nite::JOINT_RIGHT_FOOT+1) + j) - (filter_size/2) * (nite::JOINT_RIGHT_FOOT+1);
                k <= (i * (nite::JOINT_RIGHT_FOOT+1) + j) + (filter_size/2) * (nite::JOINT_RIGHT_FOOT+1);
                k += (nite::JOINT_RIGHT_FOOT+1)) {
                px.push_back(m_vBonesOrientation[k].pos.x);
                py.push_back(m_vBonesOrientation[k].pos.y);
                pz.push_back(m_vBonesOrientation[k].pos.z);
            }

            sort(px.begin(), px.end());
            sort(py.begin(), py.end());
            sort(pz.begin(), pz.end());

            joints[j].pos.x = px[filter_size/2];
            joints[j].pos.y = py[filter_size/2];
            joints[j].pos.z = pz[filter_size/2];
        }
    }
}
