#include "BulletOpenGLApplication.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyInternals.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"

//not all of these might be needed for soft body import
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional> 
#include <cctype>

class BasicDemo : public BulletOpenGLApplication {
public:
	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;

	void CreateObjects();
	void CreateHindgedObjects(int numLinks, btVector3 &start, btVector3 &stop);
	void CreateBoxes(int num, btVector3 &dimension, float weight, btVector3 &color, btVector3 &start, btVector3 &stop);
	void CreateClothPatch(unsigned int clothWidth,
		                  unsigned int clothHeight,
						  btVector3 &leftButtom,
						  btVector3 &leftTop, 
						  btVector3 &rightButtom, 
						  btVector3 &rightTop,
						  btVector3 &globalLocation);
	bool CreateTetgenMesh(const std::string& eleFilename, const std::string& faceFilename, const std::string& nodeFilename, btVector3 &globalLocation, float totalMass, float k);
	void CreateCompoundJack(int numJacks, btVector3 &start, btVector3 &stop);
	btQuaternion EulerRotation(float psi, float phi, float theta);
	void trim(std::string& s);

	static const int clothShininess = 20;

private:
};