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
		flux << "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation" << endl;
		//flux << "\tJOINT Spine" << endl;
		//flux << "\t{" << endl;

			// Shoulder Center
            //flux << "\t\tOFFSET " << -m_aOffsets[nite::JOINT_NECK].x << " " << m_aOffsets[nite::JOINT_NECK].y << " " << m_aOffsets[nite::JOINT_NECK].z << endl;
			//flux << "\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
			flux << "\t\tJOINT ShoulderCenter" << endl;
			flux << "\t\t{" << endl;
				// Head
				flux << "\t\t\tOFFSET " << m_aOffsets[nite::JOINT_NECK].x << " " << m_aOffsets[nite::JOINT_NECK].y << " " << m_aOffsets[nite::JOINT_NECK].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
				flux << "\t\t\tJOINT Head" << endl;
				flux << "\t\t\t{" << endl;
					// End Site
					flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_HEAD].x << " " << m_aOffsets[nite::JOINT_HEAD].y << " " << m_aOffsets[nite::JOINT_HEAD].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
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
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowLeft" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Left
                        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_ELBOW].x << " " << m_aOffsets[nite::JOINT_LEFT_ELBOW].y << " " << m_aOffsets[nite::JOINT_LEFT_ELBOW].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristLeft" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Left
                            flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_HAND].x << " " << m_aOffsets[nite::JOINT_LEFT_HAND].y << " " << m_aOffsets[nite::JOINT_LEFT_HAND].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
							//flux << "\t\t\t\t\t\tJOINT HandLeft" << endl;
							//flux << "\t\t\t\t\t\t{" << endl;
								// End Site
								//flux << "\t\t\t\t\t\t\tOFFSET " << -m_aOffsets[7].x << " " << m_aOffsets[7].y << " " << m_aOffsets[7].z << endl;
								//flux << "\t\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
								flux << "\t\t\t\t\t\t\tEnd Site" << endl;
								flux << "\t\t\t\t\t\t\t{" << endl;
									flux << "\t\t\t\t\t\t\t\tOFFSET -8.32 0.0 0.0" << endl;
								flux << "\t\t\t\t\t\t\t}" << endl;
							//flux << "\t\t\t\t\t\t}" << endl;
						flux << "\t\t\t\t\t}" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;

				// Shoulder Right
				flux << "\t\t\tJOINT ShoulderRight" << endl;
				flux << "\t\t\t{" << endl;
					// Elbow Right
                    flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_SHOULDER].x << " " << m_aOffsets[nite::JOINT_RIGHT_SHOULDER].y << " " << m_aOffsets[nite::JOINT_RIGHT_SHOULDER].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowRight" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Right
                        flux << "\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_ELBOW].x << " " << m_aOffsets[nite::JOINT_RIGHT_ELBOW].y << " " << m_aOffsets[nite::JOINT_RIGHT_ELBOW].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristRight" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Right
                            flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_HAND].x << " " << m_aOffsets[nite::JOINT_RIGHT_HAND].y << " " << m_aOffsets[nite::JOINT_RIGHT_HAND].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
							//flux << "\t\t\t\t\t\tJOINT HandRight" << endl;
							//flux << "\t\t\t\t\t\t{" << endl;
								// End Site
								//flux << "\t\t\t\t\t\t\tOFFSET " << -m_aOffsets[11].x << " " << m_aOffsets[11].y << " " << m_aOffsets[11].z << endl;
								//flux << "\t\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
								flux << "\t\t\t\t\t\t\tEnd Site" << endl;
								flux << "\t\t\t\t\t\t\t{" << endl;
									flux << "\t\t\t\t\t\t\t\tOFFSET 8.32 0.0 0.0" << endl;
								flux << "\t\t\t\t\t\t\t}" << endl;
							//flux << "\t\t\t\t\t\t}" << endl;
						flux << "\t\t\t\t\t}" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;
			
			flux << "\t\t}" << endl;

		//flux << "\t}" << endl;

		// Hip Left
		flux << "\tJOINT HipLeft" << endl;
		flux << "\t{" << endl;

			// Knee Left
        flux << "\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_HIP].x << " " << m_aOffsets[nite::JOINT_LEFT_HIP].y << " " << m_aOffsets[nite::JOINT_LEFT_HIP].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
			flux << "\t\tJOINT KneeLeft" << endl;
			flux << "\t\t{" << endl;

				// Ankle Left
				flux << "\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_KNEE].x << " " << m_aOffsets[nite::JOINT_LEFT_KNEE].y << " " << m_aOffsets[nite::JOINT_LEFT_KNEE].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
				flux << "\t\t\tJOINT AnkleLeft" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Left
                    flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_LEFT_FOOT].x << " " << m_aOffsets[nite::JOINT_LEFT_FOOT].y << " " << m_aOffsets[nite::JOINT_LEFT_FOOT].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					//flux << "\t\t\t\tJOINT FootLeft" << endl;
					//flux << "\t\t\t\t{" << endl;
					
						// End Site
						//flux << "\t\t\t\t\tOFFSET " << -m_aOffsets[15].x << " " << m_aOffsets[15].y << " " << m_aOffsets[15].z << endl;
						//flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tEnd Site" << endl;
						flux << "\t\t\t\t\t{" << endl;
							flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 8.91" << endl;
						flux << "\t\t\t\t\t}" << endl;	
					//flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;
			flux << "\t\t}" << endl;
		flux << "\t}" << endl;

		// Hip Right
		flux << "\tJOINT HipRight" << endl;
		flux << "\t{" << endl;

			// Knee Right
			flux << "\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_HIP].x << " " << m_aOffsets[nite::JOINT_RIGHT_HIP].y << " " << m_aOffsets[nite::JOINT_RIGHT_HIP].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
			flux << "\t\tJOINT KneeRight" << endl;
			flux << "\t\t{" << endl;

				// Ankle Right
				flux << "\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_KNEE].x << " " << m_aOffsets[nite::JOINT_RIGHT_KNEE].y << " " << m_aOffsets[nite::JOINT_RIGHT_KNEE].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
				flux << "\t\t\tJOINT AnkleRight" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Right
					flux << "\t\t\t\tOFFSET " << m_aOffsets[nite::JOINT_RIGHT_FOOT].x << " " << m_aOffsets[nite::JOINT_RIGHT_FOOT].y << " " << m_aOffsets[nite::JOINT_RIGHT_FOOT].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					//flux << "\t\t\t\tJOINT FootRight" << endl;
					//flux << "\t\t\t\t{" << endl;
					
						// End Site
						//flux << "\t\t\t\t\tOFFSET " << -m_aOffsets[19].x << " " << m_aOffsets[19].y << " " << m_aOffsets[19].z << endl;
						//flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tEnd Site" << endl;
						flux << "\t\t\t\t\t{" << endl;
							flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 8.91" << endl;
						flux << "\t\t\t\t\t}" << endl;	
					//flux << "\t\t\t\t}" << endl;
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
    // get parent's quaternion, convert to right hand coordinate
    Vec_Math::Quaternion q_parent;
    if (idx == nite::JOINT_TORSO) {
        q_parent = Vec_Math::quat_identity;
    } else {
        q_parent = Vec_Math::vec4_create(-joints[parent_joint_map[idx]].quat.x,
                                         -joints[parent_joint_map[idx]].quat.y,
                                         joints[parent_joint_map[idx]].quat.z,
                                         joints[parent_joint_map[idx]].quat.w);
    }

    // get joint's quaternion, convert to right hand coordinate
    Vec_Math::Quaternion q_current = Vec_Math::vec4_create(-joints[idx].quat.x,
                                                           -joints[idx].quat.y,
                                                           joints[idx].quat.z,
                                                           joints[idx].quat.w);

    // calculate between quaternion
	Vec_Math::Quaternion q_delta = Vec_Math::quat_left_multiply(q_current, Vec_Math::quat_inverse(q_parent));

	// convert the quaternion to euler angles by default roll->pitch->yaw order, which roll is outer, yaw is inner.
    Vec_Math::Vec3 ret = Vec_Math::euler_from_quat(q_delta);

    return ret;
}

void KinectBVH::WriteJoint(stringstream& flux, KinectJoint *joints, nite::JointType j)
{
    Vec_Math::Vec3 angles = GetEulers(joints, j);
    flux << angles.z * Vec_Math::kRadToDeg << " " << angles.x * Vec_Math::kRadToDeg << " " << angles.y * Vec_Math::kRadToDeg << " ";
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
