#ifdef   OTHERMAIN_H

#include <iostream>
#include <gl/glut.h>

#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletDynamics/Dynamics/btSimpleDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

/* Bullet Physics Simulation */
btBroadphaseInterface* broadphase = nullptr;
btDefaultCollisionConfiguration* collisionConfiguration = nullptr;
btCollisionDispatcher* dispatcher = nullptr;
btSequentialImpulseConstraintSolver* solver = nullptr;
btSimpleDynamicsWorld* dynamicsWorld = nullptr;
const btVector3 GRAVITY = btVector3(0.0f, -9.81f, 0.0f);

/* Bullet Physics Object: Ground */
btVector3 groundScale = btVector3(5.0f, 0.2f, 5.0f);
btRigidBody* ground = nullptr;
btCollisionShape* groundCollisionShape = nullptr;
btDefaultMotionState* groundMotionState = nullptr;

/* Bullet Physics Object: Box */
btVector3 boxScale = btVector3(0.1f, 0.1f, 2.0f);
btQuaternion boxRotation = btQuaternion(0.0f, 0.0f, 0.0f, 1.0f);
btVector3 boxPosition = btVector3(0.0f, 5.0f, 0.0f);
btRigidBody *box[300];
int currBox = 0;
float boxMass = 1.0f;
btCollisionShape* boxCollisionShape = nullptr;
btDefaultMotionState* boxMotionState = nullptr;
float boxTransformation[16];
bool firstBox = true;

/* Bullet Physics Object: Box2 */
btVector3 boxScale2 = btVector3(0.1f, 2.0f, 0.1f);
btDefaultMotionState* boxMotionState2 = nullptr;
btCollisionShape* boxCollisionShape2 = nullptr;
btVector3 boxPosition2 = btVector3(0.0f, 5.0f, 0.0f);
float box2Angle = 0.78525f;

/* Bullet Physics Object: Box3 */
btVector3 boxScale3 = btVector3(2.0f, 0.1f, 0.1f);
btDefaultMotionState* boxMotionState3 = nullptr;
btCollisionShape* boxCollisionShape3 = nullptr;
btVector3 boxPosition3 = btVector3(0.0f, 5.0f, 0.0f);
float box3Angle = 0.78525f;

/* Bullet Physics Compound Object: Jack   */
btCompoundShape* jack = nullptr;
btDefaultMotionState* jackMotionState = nullptr;

/* Frame Rate and Timing */
int curTimeMilli = 0;
int lastTimeMilli = 0;
int dtMilli = 0;
float curTime = 0.0f;
float curTimeSec = 0.0f;
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

btQuaternion EulerRotation(float psi, float phi, float theta) {
	float halfTheta = theta * 0.5f;
	float halfPhi = phi * 0.5f;
	float halfPsi = psi * 0.5f;

	float cosHalfPhi = std::cos(halfPhi);
	float sinHalfPhi = std::sin(halfPhi);

	float cosHalfTheta = std::cos(halfTheta);
	float sinHalfTheta = std::sin(halfTheta);

	float cosHalfPsi = std::cos(halfPsi);
	float sinHalfPsi = std::sin(halfPsi);

	btQuaternion quat;
	quat.setW(cosHalfPhi * cosHalfTheta * cosHalfPsi + sinHalfPhi * sinHalfTheta * sinHalfPsi);
	quat.setY(sinHalfPhi * cosHalfTheta * cosHalfPsi - cosHalfPhi * sinHalfTheta * sinHalfPsi);
	quat.setZ(cosHalfPhi * sinHalfTheta * cosHalfPsi + sinHalfPhi * cosHalfTheta * sinHalfPsi);
	quat.setX(cosHalfPhi * cosHalfTheta * sinHalfPsi - sinHalfPhi * sinHalfTheta * cosHalfPsi);
	return quat;
}

void initializeBulletWorld() {
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver();
	
	dynamicsWorld = new btSimpleDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(GRAVITY);
}

void initializeGround() {
	groundCollisionShape = new btBoxShape(groundScale);
	groundMotionState = new btDefaultMotionState();

	btVector3 inertia(0.0f, 0.0f, 0.0f);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, groundMotionState, groundCollisionShape, inertia);
	ground = new btRigidBody(rigidBodyCI);
	dynamicsWorld->addRigidBody(ground);
}

void initializeBox() {

	jack = new btCompoundShape();

	boxCollisionShape = new btBoxShape(boxScale);

	//
	// Box
	//  
	boxRotation = EulerRotation(0.6234f*currBox/3, 0.23423f*currBox/3, 0.923f*currBox/3);
	btTransform defaultTransform(boxRotation, boxPosition);

	jack->addChildShape (defaultTransform, boxCollisionShape);

	//
	// box2
	//

	boxCollisionShape = new btBoxShape(boxScale2);

	boxRotation = EulerRotation(0.6234f*currBox/3 + box2Angle, 0.23423f*currBox/3 + box2Angle, 0.923f*currBox/3 + box2Angle);
	btTransform defaultTransform2(boxRotation, boxPosition2); 

	jack->addChildShape (defaultTransform2, boxCollisionShape);

	//
	// box3
	//

	boxCollisionShape = new btBoxShape(boxScale3);

	boxRotation = EulerRotation(0.6234f*currBox/3 + box2Angle*2, 0.23423f*currBox/3 + box2Angle*2, 0.923f*currBox/3 + box2Angle*2);
	btTransform defaultTransform3(boxRotation, boxPosition3); 

	jack->addChildShape (defaultTransform3, boxCollisionShape);

	//
	// Jack
	//
	boxMotionState = new btDefaultMotionState(defaultTransform);
	
	btVector3 inertia;
	boxCollisionShape->calculateLocalInertia(boxMass, inertia);


	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(boxMass, boxMotionState, jack, inertia);
	if (currBox % 10 > 5){
		 rigidBodyCI.m_friction = 0.0f;
		 rigidBodyCI.m_restitution = 3.0f;
	}
	
	box[currBox] = new btRigidBody(rigidBodyCI);
	currBox++;

	box[currBox - 1]->setLinearVelocity(btVector3(currBox/5, currBox/5, currBox/5));
	box[currBox - 1]->setAngularVelocity(btVector3(currBox/10, currBox/10, currBox/10));
	 
	dynamicsWorld->addRigidBody(box[currBox - 1]);
	printf("add happened\n");
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
	glEnable (GL_DEPTH_TEST);

	initializeBulletWorld();
	initializeGround();
	initializeBox();
	initializeLight();
}

void renderGround() {
	glPushMatrix();
		glScalef(groundScale.getX() * 2.0f, groundScale.getY() * 2.0f, groundScale.getZ() * 2.0f);
		glutSolidCube(1.0f);
	glPopMatrix();
}

void renderBox() {
	btTransform transform;
	for (int i = 0;i < currBox; i++){
		box[i]->getMotionState()->getWorldTransform(transform);
		transform.getOpenGLMatrix(boxTransformation);

		glPushMatrix();
			glMultMatrixf(boxTransformation);
			glScalef(boxScale.getX() * 2.0f, boxScale.getY() * 2.0f, boxScale.getZ() * 2.0f);
			glutSolidCube(1.0f);
		glPopMatrix();
		glPushMatrix();
			glMultMatrixf(boxTransformation);
			glScalef(boxScale2.getX() * 2.0f, boxScale2.getY() * 2.0f, boxScale2.getZ() * 2.0f);
			glutSolidCube(1.0f);
		glPopMatrix();
		glPushMatrix();
			glMultMatrixf(boxTransformation);
			glScalef(boxScale3.getX() * 2.0f, boxScale3.getY() * 2.0f, boxScale3.getZ() * 2.0f);
			glutSolidCube(1.0f);
		glPopMatrix();
	}
}

void onRender() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
		gluLookAt(8.0f, 8.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

		renderGround();
		renderBox();

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
	dynamicsWorld->removeRigidBody(ground);
	delete ground;
	delete groundMotionState;
	delete groundCollisionShape;

	for (int i = 0; i < currBox; i++){
		dynamicsWorld->removeRigidBody(box[i]);
		delete box[i];
	}
	delete boxMotionState;
	delete boxCollisionShape;

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
}

void onIdle() {
	curTimeMilli = glutGet(GLUT_ELAPSED_TIME);
	dtMilli = curTimeMilli - lastTimeMilli;
	float elapsedTime = dtMilli / static_cast<float>(1000.0f);
	lastTimeMilli = curTimeMilli;
	curTime += elapsedTime;
	curTimeSec += elapsedTime;

	if (curTimeSec > INV_FPS*60 && !paused){
		 initializeBox();
		 curTimeSec = 0.0f;
		 glutPostRedisplay();
	}

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
