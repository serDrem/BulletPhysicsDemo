#include "GameObject.h"
GameObject::GameObject(btCollisionShape* pShape, float mass, const btVector3 &color, const btVector3 &initialPosition, const btQuaternion &initialRotation) {
	// store the shape for later usage
	m_pShape = pShape;

	// store the color
	m_color = color;

	// create the initial transform
	transform.setIdentity();
	transform.setOrigin(initialPosition);
	transform.setRotation(initialRotation);

	// create the motion state from the
	// initial transform
	m_pMotionState = new OpenGLMotionState(transform);

	// calculate the local inertia
	btVector3 localInertia(0,0,0);

	// objects of infinite mass can't
	// move or rotate
	if (mass != 0.0f)
		pShape->calculateLocalInertia(mass, localInertia);

	if (pShape->isSoftBody()){
		m_pSoftBody =  ((btSoftBodyCollisionShape*) pShape)->m_body;
		m_pSoftBody->transform(transform);
	} else { 
		// create the rigid body construction
		// info using the mass, motion state
		// and shape
		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, m_pMotionState, pShape, localInertia);

#ifdef HW3
		if ( (rand() % 100) > 50 ){
			cInfo.m_friction = 0.0f;
			cInfo.m_restitution = 3.0f;
		}
#endif
		// create the rigid body
		m_pBody = new btRigidBody(cInfo);

#ifdef HW3		
	    m_pBody->setLinearVelocity(btVector3((rand() % 100) / 20.f, (rand() % 100) / 20.f, (rand() % 100) / 20.f));
	    if ( (rand() % 100) > 50 ){
			m_pBody->setAngularVelocity(btVector3((rand() % 100) / 20.f, (rand() % 100) / 20.f, (rand() % 100) / 20.f));
		}
#endif

	}
}

GameObject::~GameObject() {
	delete m_pBody;
	delete m_pMotionState;
	delete m_pShape;
}
