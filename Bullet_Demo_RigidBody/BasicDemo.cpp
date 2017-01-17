#include "BasicDemo.h"

void BasicDemo::InitializePhysics() {
	// create the collision configuration
	m_pCollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the constraint solver
	m_pSolver = new btSequentialImpulseConstraintSolver();
	// create the world
	m_pWorld = new btSoftRigidDynamicsWorld (m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);

	//works without
	const btVector3 GRAVITY = btVector3(0.0f, -9.81f, 0.0f);
	m_pWorld->setGravity(GRAVITY);

	// create our scene's physics objects
	CreateObjects();
}

void BasicDemo::ShutdownPhysics() {
	delete m_pWorld;
	delete m_pSolver;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;
}

btQuaternion BasicDemo::EulerRotation(float psi, float phi, float theta) {
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

void BasicDemo::CreateObjects() {
	// SERGEY
	// create our original black ball of destruction
	CreateGameObject(new btSphereShape(4.f), 100.0, btVector3(0.0f, 0.0f, 0.0f), btVector3(-50.0f, 20.0f, 5.0f), btQuaternion(0,0,1,1), btVector3(30.0f, 0.0f, 0.0f));
	// create our second black ball of destruction just for the tetmesh cone
	CreateGameObject(new btBoxShape(btVector3(1,4,4)), 100.0, btVector3(0.0f, 0.0f, 0.0f), btVector3(-30.0f, 20.0f, -35.0f), btQuaternion(0,0,1,1), btVector3(40.0f, -30.0f, 0.0f));
	// create a ground plane
	CreateGameObject(new btBoxShape(btVector3(1,50,50)), 0, btVector3(0.2f, 0.6f, 0.6f), btVector3(0.0f, 0.0f, 0.0f));
	// create a second box
	//CreateGameObject(new btBoxShape(btVector3(1,1,1)), 10.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(5.0f, 5.0f, 0.0f));
	//CreateHindgedObjects(30, btVector3(0.0f, 10.0f, 0.0f), btVector3(50.0f, 10.0f, 0.0f));
	
	
	//Creating the base
	for (float i = 0; i < 5; i++){
		for (float y = 0; y < 5; y++){
			CreateBoxes(10, btVector3(1.0f, 1.0f, 1.0f), 1.f,  btVector3(1.0f, 0.0f, 0.0f),  btVector3(0.0f, y*2.3, i), btVector3(20.0f, y*2.3, i));
		}
	}
	
	//create balloons on the front side and left side
	CreateHindgedObjects(10, btVector3(0.0f, 5.0f, -10.0f), btVector3(20.0f, 5.0f, -10.0f));
	CreateHindgedObjects(13, btVector3(20.0f, 5.0f, -14.0f), btVector3(20.0f, 5.0f, 6.0f));
	

	//create a cylinder on top of the base
	btQuaternion cylinderRotation = EulerRotation(.0f, 3.141/2, .0f);
	CreateGameObject(new btCylinderShape(btVector3(8,3,0)), 10, btVector3( (rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f),
		btVector3(10.0f, 14.0f, 5.0f), cylinderRotation );
	
//	CreateBoxes(10, btVector3(1.0f, 1.0f, 1.0f), 1.f,  btVector3(1.0f, 0.0f, 0.0f),  btVector3(0.0f, 0.0f, 0.0f), btVector3(20.0f, 0.0f, 0.0f));
//	CreateBoxes(10, btVector3(1.0f, 1.0f, 1.0f), 1.f,  btVector3(1.0f, 0.0f, 0.0f),  btVector3(0.0f, 0.0f, 1.0f), btVector3(20.0f, 0.0f, 1.0f));
//	CreateBoxes(10, btVector3(1.0f, 1.0f, 1.0f), 1.f,  btVector3(1.0f, 0.0f, 0.0f),  btVector3(0.0f, 0.0f, 2.0f), btVector3(20.0f, 0.0f, 2.0f));
//	CreateBoxes(10, btVector3(1.0f, 1.0f, 1.0f), 1.f,  btVector3(1.0f, 0.0f, 0.0f),  btVector3(0.0f, 0.0f, 3.0f), btVector3(20.0f, 0.0f, 3.0f));

	// create sample cloth patch
	float s = 5.0f;
	float h = 5.0f;
 	//Creates a cloth patch, but the patch does not interact with other solid objects
	//The motion just slows down when the objects are colliding, and then they seem to be 
	//coming to a different rest state, but an incorrect one.
	//Perhaps that is a rendering issues for the cloth, but I'm not sure how to solve it.
	CreateClothPatch(60, //more like resolution x: 60 
		             60,	//more like resolution y: 60
					 //opposite the camera, change along the x,y axis
					 /*
					 btVector3(-s, -s, h), 
					 btVector3(s*2, -s, h),
					 btVector3(-s, s, h), 
					 btVector3(s*2, s, h),
					 */
					 //along a wall on the right, change along the x,z axis
					 
					 btVector3(-s, h, -s), 
					 btVector3(s*2, h, -s),
					 btVector3(-s, h, s), 
     				 btVector3(s*2, h, s),
					 
					 //on the ceiling, change along the y,z axis
			/*	
					 btVector3( h, -s, -s), 
					 btVector3( h, s*2, -s),
					 btVector3( h, -s, s), 
					 btVector3( h, s*2, s),
			*/	
					 btVector3(5.0f, 15.0f, 5.0f)); 
	
	// soft bodies dont' collide with each other
	
	CreateTetgenMesh("BendTorus.ele", "BendTorus.face", "BendTorus.node", btVector3(30.0f, 3.0f, 10.0f), 10.0f, 0.04);
	CreateTetgenMesh("BendTorus.ele", "BendTorus.face", "BendTorus.node", btVector3(30.0f, 3.0f, 5.0f), 10.0f, 0.04);
	CreateTetgenMesh("BendTorus.ele", "BendTorus.face", "BendTorus.node", btVector3(30.0f, 3.0f, 0.0f), 10.0f, 0.04);
	

	CreateTetgenMesh("cone4.1.ele", "cone4.1.face", "cone4.1.node", btVector3(25.0f, 5.0f, -35.0f), 10.0f, 0.008);
	
	//CreateCompoundJack(10, btVector3(8.0f, 19.0f, 5.0f), btVector3(8.0f, 49.0f, 5.0f));
	CreateCompoundJack(10, btVector3(10.0f, 19.0f, 5.0f), btVector3(10.0f, 50.0f, 5.0f));
	//CreateCompoundJack(10, btVector3(16.0f, 18.0f, 5.0f), btVector3(16.0f, 49.0f, 5.0f));
}

void BasicDemo::CreateCompoundJack(int numJacks, btVector3 &start, btVector3 &stop){


	for (int i = 0; i < numJacks; i++) {

		// create three boxes for the load
		btCollisionShape* pBox1 = new btBoxShape(btVector3(1.5f, 0.2f,
			0.2f));
		btCollisionShape* pBox2 = new btBoxShape(btVector3(0.2f, 1.5f,
			0.2f));
		btCollisionShape* pBox3 = new btBoxShape(btVector3(0.2f, 0.2f,
			1.5f));
		// create a transform we'll use to set each object's position
		btTransform trans;
		trans.setIdentity();
		// create our compound shape
		btCompoundShape* pCompound = new btCompoundShape();
		// add the boxes
		pCompound->addChildShape(trans, pBox1);
		pCompound->addChildShape(trans, pBox2);
		pCompound->addChildShape(trans, pBox3);

		btVector3 intermidiate( (stop.getX() - start.getX()) * i / (float) (numJacks) + start.getX(),
								(stop.getY() - start.getY()) * i / (float) (numJacks) + start.getY(), 
								(stop.getZ() - start.getZ()) * i / (float) (numJacks) + start.getZ());
		// create a game object using the compound shape
		CreateGameObject(pCompound, 1.0f,  btVector3( (rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f),
			 intermidiate);
	}
}





void BasicDemo::CreateHindgedObjects(int numLinks, btVector3 &start, btVector3 &stop) {
	// SERGEY
	GameObject* lastLink = new GameObject(new btSphereShape(1.0), 0.0, btVector3( (rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f), start);
	GameObject* nextLink = NULL;
	AcceptGameObject(lastLink);
	// define the 'strength' of our constraint (each axis)
	float cfm = 0.9f;
	// define the 'error reduction' of our constraint (each axis)
	float erp = 0.5f;
	for(int i = 0; i < numLinks - 1; i++){
		btVector3 intermidiate( (stop.getX() - start.getX()) * i / (float) (numLinks) + start.getX(),
								(stop.getY() - start.getY()) * i / (float) (numLinks) + start.getY(), 
								(stop.getZ() - start.getZ()) * i / (float) (numLinks) + start.getZ());
		nextLink = new GameObject(new btSphereShape(1.0), 1.0, btVector3((rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f), intermidiate);
		btPoint2PointConstraint *newConstraint = new btPoint2PointConstraint(*(lastLink->GetRigidBody()),
                        *(nextLink->GetRigidBody()),
						btVector3(0.0f, 0.5f, 0.0f),
                        btVector3(0.0f, -0.5f, 0.0f));
		newConstraint->setParam(BT_CONSTRAINT_STOP_CFM,cfm,-1);
		newConstraint->setParam(BT_CONSTRAINT_STOP_ERP,erp,-1);
		AcceptGameObject(nextLink);
		AcceptConstraint(newConstraint);
		lastLink = nextLink;
	}
	//Making a circle
	nextLink = new GameObject(new btSphereShape(1.0), 0.0, btVector3((rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f), stop);
	btPoint2PointConstraint *newConstraint = new btPoint2PointConstraint(*(lastLink->GetRigidBody()),
                    *(nextLink->GetRigidBody()),
					btVector3(0.0f, 0.5f, 0.0f),
                    btVector3(0.0f, -0.5f, 0.0f));
		newConstraint->setParam(BT_CONSTRAINT_STOP_CFM,cfm,-1);
		newConstraint->setParam(BT_CONSTRAINT_STOP_ERP,erp,-1);
	AcceptGameObject(nextLink);
	AcceptConstraint(newConstraint);
}

void BasicDemo::CreateBoxes(int num, btVector3 &dimension, float weight, btVector3 &color,  btVector3 &start, btVector3 &stop){

	for(int i = 0; i < num ; i++){
		btVector3 intermidiate( (stop.getX() - start.getX()) * i / (float) (num) + start.getX(),
								(stop.getY() - start.getY()) * i / (float) (num) + start.getY(), 
								(stop.getZ() - start.getZ()) * i / (float) (num) + start.getZ());
		CreateGameObject(new btBoxShape(dimension), weight, color, intermidiate);
	}
}

void BasicDemo::CreateClothPatch(unsigned int clothWidth, //more like resolution x: 60 
		                  unsigned int clothHeight,	//more like resolution y: 60
						  btVector3 &leftButtom,
						  btVector3 &leftTop, 
						  btVector3 &rightButtom, 
						  btVector3 &rightTop,
						  btVector3 &globalLocation){

	//Declaring btSoftBody here instead of in the GameObject.cc
	//is bad oop, but is localising my own changes, and does not require changing the constructor
	btSoftBody* cloth  = btSoftBodyHelpers::CreatePatch(
		m_pWorld->getWorldInfo(), 
		leftButtom,  
		leftTop,
		rightButtom, 
		rightTop, 
		clothWidth, clothHeight, 5, true);
		
	cloth->m_cfg.viterations = 60;
	cloth->m_cfg.piterations = 60;
	cloth->m_cfg.kDG = 0.008f;
	cloth->m_cfg.kDF = 1.0f;
	cloth->m_cfg.kMT = 1.0f;
	cloth->m_materials[0]->m_kLST = 1.0f;
	cloth->m_materials[0]->m_kAST = 1.0f;
	cloth->setTotalMass(5.0f);

	GameObject* clothGameObject = new GameObject(new btSoftBodyCollisionShape(cloth), 0.0, btVector3((rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f), globalLocation);

	AcceptGameObject(clothGameObject);
}

void BasicDemo::trim(std::string& s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}

bool BasicDemo::CreateTetgenMesh(const std::string& eleFilename, const std::string& faceFilename, const std::string& nodeFilename, btVector3 &globalLocation, float totalMass, float k) {
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

	btSoftBody* tetMesh = btSoftBodyHelpers::CreateFromTetGenData(m_pWorld->getWorldInfo(), eleContent.c_str(), faceContent.c_str(), nodeContent.c_str(), true, true, true);
	tetMesh->m_cfg.viterations = 60;
	tetMesh->m_cfg.piterations = 60;
	tetMesh->m_cfg.kDG = 0.008f;
	tetMesh->m_cfg.kDF = 0.1f;
	tetMesh->m_cfg.kMT = 0.1f;
	tetMesh->m_materials[0]->m_kLST = k;
	tetMesh->m_materials[0]->m_kAST = k;
	tetMesh->setTotalMass(totalMass);


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

	GameObject* clothGameObject = new GameObject(new btSoftBodyCollisionShape(tetMesh), 0.0, btVector3((rand() % 100) / 100.f, (rand() % 100) / 100.f, (rand() % 100) / 100.f), globalLocation);
	
	AcceptGameObject(clothGameObject);

	ele.close();
	face.close();
	node.close();
}
