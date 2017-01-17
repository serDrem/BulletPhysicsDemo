#ifdef   OTHERMAIN_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional> 
#include <cctype>

/* Glut */
#include <gl/glut.h>

/* Bullet Core */
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>

/* Bullet SoftBody */
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodySolvers.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

/* Vector3 Class */
#include "Vector3.h"

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

/* Bullet Physics Simulation */
btBroadphaseInterface* broadphase = nullptr;
btDefaultCollisionConfiguration* collisionConfiguration = nullptr;
btCollisionDispatcher* dispatcher = nullptr;
btSequentialImpulseConstraintSolver* solver = nullptr;
btSoftBodySolver* softBodySolver = nullptr;
btSoftRigidDynamicsWorld* dynamicsWorld = nullptr;
const btVector3 GRAVITY = btVector3(0.0f, -9.81f, 0.0f);

/* Bullet Physics Object: Cloth */
float tetMeshAmbient[] = { 0.3f, 0.3f, 0.3f, 0.0f };
float tetMeshDiffuse[] = { 0.4f, 0.9f, 0.4f, 0.0f };
float tetMeshSpecular[] = { 0.3f, 0.3f, 0.3f, 0.0f };
float tetMeshShininess = 20.0;
btSoftBody* tetMesh;

/* Bullet Physics Object: Sphere */
btVector3 groundScale = btVector3(5.0f, 0.2f, 5.0f);
float groundAmbient[] = { 0.3f, 0.3f, 0.3f, 0.0f };
float groundDiffuse[] = { 0.6f, 0.6f, 0.6f, 0.0f };
float groundSpecular[] = { 0.3f, 0.3f, 0.3f, 0.0f };
float groundShininess = 30.0;
btRigidBody* ground = nullptr;
btCollisionShape* groundCollisionShape = nullptr;
btDefaultMotionState* groundMotionState = nullptr;
float groundTransformation[16];

/* Frame Rate and Timing */
int curTimeMilli = 0;
int lastTimeMilli = 0;
int dtMilli = 0;
float curTime = 0.0f;
const unsigned int FPS = 60;
const float INV_FPS = 1.0f / static_cast<float>(FPS);
bool paused = true;

/* Static Light Properties */
float lightAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
float lightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
float lightSpecular[] = { 0.8f, 0.8f, 0.8f, 1.0f };
float lightPosition[] = { 3.0f, 5.0f, 3.0f, 1.0f };

/* Static Camera Parameters */
const float NEAR_PLANE = 0.1f;
const float FAR_PLANE = 100.0f;
const float FOV = 45.0f;

void initializeBulletWorld() {
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver();

	dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration, softBodySolver);
	dynamicsWorld->setGravity(GRAVITY);
}

void trim(std::string& s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}

bool initializeTetgenMesh(const std::string& eleFilename, const std::string& faceFilename, const std::string& nodeFilename) {
	std::ifstream ele;
	std::ifstream face;
	std::ifstream node;

	ele.open(eleFilename.c_str());
	if ( ele.is_open() == false ) {
		std::cerr << "[initializeTetgenMesh] Error: Cannot open file: " << eleFilename << std::endl;
		return false;
	}

	face.open(faceFilename.c_str());
	if ( ele.is_open() == false ) {
		std::cerr << "[initializeTetgenMesh] Error: Cannot open file: " << faceFilename << std::endl;
		return false;
	}

	node.open(nodeFilename.c_str());
	if ( ele.is_open() == false ) {
		std::cerr << "[initializeTetgenMesh] Error: Cannot open file: " << faceFilename << std::endl;
		return false;
	}

	std::stringstream stream;
	stream << ele.rdbuf();
	std::string eleContent = stream.str();
	stream.str("");

	stream << node.rdbuf();
	std::string nodeContent = stream.str();
	stream.str("");

	stream << face.rdbuf();
	std::string faceContent = stream.str();

	tetMesh = btSoftBodyHelpers::CreateFromTetGenData(dynamicsWorld->getWorldInfo(), eleContent.c_str(), faceContent.c_str(), nodeContent.c_str(), true, true, true);
	tetMesh->m_cfg.viterations = 60;
	tetMesh->m_cfg.piterations = 60;
	tetMesh->m_cfg.kDG = 0.008f;
	tetMesh->m_cfg.kDF = 0.1f;
	tetMesh->m_cfg.kMT = 0.1f;
	tetMesh->m_materials[0]->m_kLST = 0.04f;
	tetMesh->m_materials[0]->m_kAST = 0.04f;
	tetMesh->setTotalMass(10.0f);

	// Manually 
	unsigned int faceCount, index0, index1, index2, throwAway;
	std::sscanf(faceContent.c_str(), "%d %d", &faceCount, &throwAway);
	std::string line;
	std::getline(stream, line);
	for ( unsigned int i = 0; i < faceCount; i++ ) {
		std::getline(stream, line);
		trim(line);
		std::sscanf(line.c_str(), "%d %d %d %d", &throwAway, &index0, &index1, &index2);
		tetMesh->appendFace(index0, index1, index2);
	}

	for ( unsigned int i = 0; i < tetMesh->m_nodes.size(); i++ ) 
		tetMesh->m_nodes[i].m_x.setY(tetMesh->m_nodes[i].m_x.y() + 2.0);

	dynamicsWorld->addSoftBody(tetMesh);


	ele.close();
	face.close();
	node.close();
}

void initializeGround() {
	groundCollisionShape = new btBoxShape(groundScale);
	groundMotionState = new btDefaultMotionState();

	btVector3 inertia(0.0f, 0.0f, 0.0f);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, groundMotionState, groundCollisionShape, inertia);
	ground = new btRigidBody(rigidBodyCI);
	dynamicsWorld->addRigidBody(ground);
}

void initializeLight() {
	glEnable(GL_LIGHTING);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
}

void onInit() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	initializeBulletWorld();
	initializeTetgenMesh("BendTorus.ele", "BendTorus.face", "BendTorus.node");
	initializeGround();
	initializeLight();
}

void renderCloth() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, tetMeshAmbient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, tetMeshDiffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, tetMeshSpecular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &tetMeshShininess);

	// Calculate cloth face normals.
	float x, y, z;
	Vector3f a, b, c;
	Vector3f faceNormal;
	glBegin(GL_TRIANGLES);
	for ( int i = 0; i < tetMesh->m_faces.size(); i++ ) {
		for ( unsigned int j = 0; j < 3; j++ ) {
			x = tetMesh->m_faces[i].m_n[j]->m_x.x();
			y = tetMesh->m_faces[i].m_n[j]->m_x.y();
			z = tetMesh->m_faces[i].m_n[j]->m_x.z();

			if ( j == 0 ) a = Vector3f(x, y, z);
			if ( j == 1 ) b = Vector3f(x, y, z);
			if ( j == 2 ) c = Vector3f(x, y, z);
		}

		faceNormal = -Vector3f::Cross(b - a, c - a);
		Vector3f::Normalize(faceNormal);
		glNormal3fv(faceNormal);
		glVertex3fv(a);
		glNormal3fv(faceNormal);
		glVertex3fv(b);
		glNormal3fv(faceNormal);
		glVertex3fv(c);
	}
	glEnd();
}

void renderGround() {
	glPushMatrix();
		glScalef(groundScale.getX() * 2.0f, groundScale.getY() * 2.0f, groundScale.getZ() * 2.0f);
		glColor3f(0.3f, 0.3f, 0.3f);
		glutSolidCube(1.0f);
	glPopMatrix();
}

void onRender() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
		gluLookAt(8.0f, 8.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

		renderCloth();
		renderGround();

	glPopMatrix();
	glutSwapBuffers();
}

void onResize(int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if ( height == 0 ) height = 1;
	gluPerspective(FOV, width / static_cast<float>(height), NEAR_PLANE, FAR_PLANE);
	glMatrixMode(GL_MODELVIEW);
}

void onExit() {
	dynamicsWorld->removeSoftBody(tetMesh);
	delete tetMesh;

	dynamicsWorld->removeRigidBody(ground);
	delete ground;
	delete groundCollisionShape;
	delete groundMotionState;

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
}

void onIdle() {
	curTimeMilli = glutGet(GLUT_ELAPSED_TIME);
	dtMilli = curTimeMilli - lastTimeMilli;
	lastTimeMilli = curTimeMilli;

	float elapsedTime = dtMilli / static_cast<float>(1000.0f);
	curTime += elapsedTime;

	if ( curTime > INV_FPS ) {
		if ( !paused ) {
			dynamicsWorld->stepSimulation(INV_FPS, 4);

			curTime = 0.0f;
			glutPostRedisplay();
		}
	}
}

void onKeyboard(unsigned char key, int x, int y) {
	if ( key == ' ' ) paused = !paused;
	glutPostRedisplay();
}

int main(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("Bullet Triangle Mesh");
		
	glutDisplayFunc(onRender);
	glutReshapeFunc(onResize);	
	glutIdleFunc(onIdle);
	glutKeyboardFunc(onKeyboard);
	std::atexit(onExit);

	onInit();

	glutMainLoop();
	return 0;
}

#endif
