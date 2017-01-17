#ifndef _BULLETOPENGLAPP_H_
#define _BULLETOPENGLAPP_H_

#include <Windows.h>
#include <GL/GL.h>
#include <GL/freeglut.h>
#include "Vector3.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyInternals.h"

// include our custom Motion State object
#include "OpenGLMotionState.h"

	// Our custom debug renderer
	#include "DebugDrawer.h"

#include "GameObject.h"
#include <vector>

//Tiff Stuff
#include <sstream>
#include <iostream>
#include "tiffio.h"

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

// a convenient typedef to reference an STL vector of GameObjects
typedef std::vector<GameObject*> GameObjects;


class BulletOpenGLApplication {
public:
	BulletOpenGLApplication();
	~BulletOpenGLApplication();
	void Initialize();
	// FreeGLUT callbacks //
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void KeyboardUp(unsigned char key, int x, int y);
	virtual void Special(int key, int x, int y);
	virtual void SpecialUp(int key, int x, int y);
	virtual void Reshape(int w, int h);
	virtual void Idle();
	virtual void Mouse(int button, int state, int x, int y);
	virtual void PassiveMotion(int x, int y);
	virtual void Motion(int x, int y);
	virtual void Display();

	// rendering. Can be overrideen by derived classes
	virtual void RenderScene();

	// scene updating. Can be overridden by derived classes
	virtual void UpdateScene(float dt);

	// physics functions. Can be overrideen by derived classes (like BasicDemo)
	virtual void InitializePhysics() {};
	virtual void ShutdownPhysics() {};

	// camera functions
	void UpdateCamera();
	void RotateCamera(float &angle, float value);
	void ZoomCamera(float distance);

	// drawing functions
	void DrawBox(const btVector3 &halfSize);
    void DrawSphere(const btScalar &radius);
	void DrawCylinder(const btScalar &radius, const btScalar &halfHeight);
	void DrawConvexHull(const btCollisionShape* shape);
	void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color, btTransform* bttransform);
	void DrawCloth(const btCollisionShape* shape, btTransform* bttransform);

	// object functions
	GameObject* CreateGameObject(btCollisionShape* pShape, 
			const float &mass, 
			const btVector3 &color = btVector3(1.0f,1.0f,1.0f), 
			const btVector3 &initialPosition = btVector3(0.0f,0.0f,0.0f), 
			const btQuaternion &initialRotation = btQuaternion(0,0,1,1),
			const btVector3 &initialVelocity = btVector3(.0f,.0f,.0f));

	GameObject* AcceptGameObject(GameObject* pObject);
	void AcceptConstraint(btPoint2PointConstraint *testingConstraints);

protected:
	// camera control
	btVector3 m_cameraPosition; // the camera's current position
	btVector3 m_cameraTarget;	 // the camera's lookAt target
	float m_nearPlane; // minimum distance the camera will render
	float m_farPlane; // farthest distance the camera will render
	btVector3 m_upVector; // keeps the camera rotated correctly
	float m_cameraDistance; // distance from the camera to its target
	float m_cameraPitch; // pitch of the camera 
	float m_cameraYaw; // yaw of the camera

	int m_screenWidth;
	int m_screenHeight;

	// core Bullet components
	btBroadphaseInterface* m_pBroadphase;
	btCollisionConfiguration* m_pCollisionConfiguration;
	btCollisionDispatcher* m_pDispatcher;
	btConstraintSolver* m_pSolver;
	btSoftRigidDynamicsWorld* m_pWorld;

	// a simple clock for counting time
	btClock m_clock;

	// an array of our game objects
	GameObjects m_objects;

			// debug renderer
			DebugDrawer* m_pDebugDrawer;

	// time since last saveFrame
	
	static const int SAVEFRAMELIMIT = 4;
	void BulletOpenGLApplication::SaveFrame(float dt);
	bool SaveTiffImage(const std::string& filename, const std::string& description, int x, int y, int width, int height, int compression);
	float m_saveFrameTime;
	int m_frameID;
	const std::string m_frameName;
	int m_frameCount;
	bool m_saveTiff;
};
#endif
