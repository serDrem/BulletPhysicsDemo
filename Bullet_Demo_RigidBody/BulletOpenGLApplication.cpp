#include "BulletOpenGLApplication.h"

// Some constants for 3D math and the camera speed
#define RADIANS_PER_DEGREE 0.01745329f
#define CAMERA_STEP_SIZE 5.0f

BulletOpenGLApplication::BulletOpenGLApplication() 
:
m_cameraPosition(0.0f, 0.0f, 0.0f),
m_cameraTarget(30.0f, 10.0f, 0.0f),
m_cameraDistance(65.0f),
m_cameraPitch(20.0f),
m_cameraYaw(0.0f),
m_upVector(0.0f, 1.0f, 0.0f),
m_nearPlane(1.0f),
m_farPlane(1000.0f),
m_pBroadphase(0),
m_pCollisionConfiguration(0),
m_pDispatcher(0),
m_pSolver(0),
m_pWorld(0),
m_saveFrameTime(0),
m_frameID(0),
m_frameName("frame"),
m_frameCount(0),
m_saveTiff(true)
{
}

BulletOpenGLApplication::~BulletOpenGLApplication() {
	// shutdown the physics system
	ShutdownPhysics();
}

void BulletOpenGLApplication::Initialize() {
	// this function is called inside glutmain() after
	// creating the window, but before handing control
	// to FreeGLUT

	// create some floats for our ambient, diffuse, specular and position
	GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f }; // dark grey
	GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat position[] = { 5.0f, 10.0f, 1.0f, 0.0f };
	
	// set the ambient, diffuse, specular and position for LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glEnable(GL_LIGHTING); // enables lighting
	glEnable(GL_LIGHT0); // enables the 0th light
	glEnable(GL_COLOR_MATERIAL); // colors materials when lighting is enabled
		
	// enable specular lighting via materials
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMateriali(GL_FRONT, GL_SHININESS, 15);
	
	// enable smooth shading
	glShadeModel(GL_SMOOTH);
	
	// enable depth testing to be 'less than'
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	// set the backbuffer clearing color to a lightish blue
	glClearColor(0.6, 0.65, 0.85, 0);

	// initialize the physics system
	InitializePhysics();

		// create the debug drawer
		m_pDebugDrawer = new DebugDrawer();
		// set the initial debug level to 0
		m_pDebugDrawer->setDebugMode(0);
		// add the debug drawer to the world
		m_pWorld->setDebugDrawer(m_pDebugDrawer);
}
void BulletOpenGLApplication::Keyboard(unsigned char key, int x, int y) {
	// This function is called by FreeGLUT whenever
	// generic keys are pressed down.
	switch(key) {
		// 'z' zooms in
	case 'z': ZoomCamera(+CAMERA_STEP_SIZE); break;
		// 'x' zoom out
	case 'x': ZoomCamera(-CAMERA_STEP_SIZE); break;
		case 'w':
			// toggle wireframe debug drawing
			m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawWireframe);
			break;
	
		case 'b':
			// toggle AABB debug drawing
			m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawAabb);
			break;
	}
}

void BulletOpenGLApplication::KeyboardUp(unsigned char key, int x, int y) {}

void BulletOpenGLApplication::Special(int key, int x, int y) {
	// This function is called by FreeGLUT whenever special keys
	// are pressed down, like the arrow keys, or Insert, Delete etc.
	switch(key) {
		// the arrow keys rotate the camera up/down/left/right
	case GLUT_KEY_LEFT: 
		RotateCamera(m_cameraYaw, +CAMERA_STEP_SIZE); break;
	case GLUT_KEY_RIGHT:
		RotateCamera(m_cameraYaw, -CAMERA_STEP_SIZE); break;
	case GLUT_KEY_UP:	
		RotateCamera(m_cameraPitch, +CAMERA_STEP_SIZE); break;
	case GLUT_KEY_DOWN:	
		RotateCamera(m_cameraPitch, -CAMERA_STEP_SIZE); break;
	}
}

void BulletOpenGLApplication::SpecialUp(int key, int x, int y) {}

void BulletOpenGLApplication::Reshape(int w, int h) {
	// this function is called once during application intialization
	// and again every time we resize the window

	// grab the screen width/height
	m_screenWidth = w;
	m_screenHeight = h;
	// set the viewport
	glViewport(0, 0, w, h);
	// update the camera
	UpdateCamera();
}

void BulletOpenGLApplication::Idle() {
	// this function is called frequently, whenever FreeGlut
	// isn't busy processing its own events. It should be used
	// to perform any updating and rendering tasks

	// clear the backbuffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	// get the time since the last iteration
	float dt = m_clock.getTimeMilliseconds();
	// reset the clock to 0
	m_clock.reset();
	// update the scene (convert ms to s)
	UpdateScene(dt / 1000.0f);

	// update the camera
	UpdateCamera();

	// render the scene
	RenderScene();

	// swap the front and back buffers
	glutSwapBuffers();

	//saveFrame for tiff
	SaveFrame(dt);
}

void BulletOpenGLApplication::Mouse(int button, int state, int x, int y) {}
void BulletOpenGLApplication::PassiveMotion(int x, int y) {}
void BulletOpenGLApplication::Motion(int x, int y) {}
void BulletOpenGLApplication::Display() {}

void BulletOpenGLApplication::UpdateCamera() {
	// exit in erroneous situations
	if (m_screenWidth == 0 && m_screenHeight == 0)
		return;
	
	// select the projection matrix
	glMatrixMode(GL_PROJECTION);
	// set it to the matrix-equivalent of 1
	glLoadIdentity();
	// determine the aspect ratio of the screen
	float aspectRatio = m_screenWidth / (float)m_screenHeight;
	// create a viewing frustum based on the aspect ratio and the
	// boundaries of the camera
	glFrustum (-aspectRatio * m_nearPlane, aspectRatio * m_nearPlane, -m_nearPlane, m_nearPlane, m_nearPlane, m_farPlane);
	// the projection matrix is now set

	// select the view matrix
	glMatrixMode(GL_MODELVIEW);
	// set it to '1'
	glLoadIdentity();

	// our values represent the angles in degrees, but 3D 
	// math typically demands angular values are in radians.
	float pitch = m_cameraPitch * RADIANS_PER_DEGREE;
	float yaw = m_cameraYaw * RADIANS_PER_DEGREE;

	// create a quaternion defining the angular rotation 
	// around the up vector
	btQuaternion rotation(m_upVector, yaw);

	// set the camera's position to 0,0,0, then move the 'z' 
	// position to the current value of m_cameraDistance.
	btVector3 cameraPosition(0,0,0);
	cameraPosition[2] = -m_cameraDistance;

	// create a Bullet Vector3 to represent the camera 
	// position and scale it up if its value is too small.
	btVector3 forward(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
	if (forward.length2() < SIMD_EPSILON) {
		forward.setValue(1.f,0.f,0.f);
	}

	// figure out the 'right' vector by using the cross 
	// product on the 'forward' and 'up' vectors
	btVector3 right = m_upVector.cross(forward);

	// create a quaternion that represents the camera's roll
	btQuaternion roll(right, - pitch);

	// turn the rotation (around the Y-axis) and roll (around 
	// the forward axis) into transformation matrices and 
	// apply them to the camera position. This gives us the 
	// final position
	cameraPosition = btMatrix3x3(rotation) * btMatrix3x3(roll) * cameraPosition;

	// save our new position in the member variable, and 
	// shift it relative to the target position (so that we 
	// orbit it)
	m_cameraPosition[0] = cameraPosition.getX();
	m_cameraPosition[1] = cameraPosition.getY();
	m_cameraPosition[2] = cameraPosition.getZ();
	m_cameraPosition += m_cameraTarget;

	// create a view matrix based on the camera's position and where it's
	// looking
	gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], m_cameraTarget[0], m_cameraTarget[1], m_cameraTarget[2], m_upVector.getX(), m_upVector.getY(), m_upVector.getZ());
	// the view matrix is now set
}

void BulletOpenGLApplication::DrawBox(const btVector3 &halfSize) {
	
	float halfWidth = halfSize.x();
	float halfHeight = halfSize.y();
	float halfDepth = halfSize.z();

	// create the vertex positions
	btVector3 vertices[8]={	
	btVector3(halfWidth,halfHeight,halfDepth),
	btVector3(-halfWidth,halfHeight,halfDepth),
	btVector3(halfWidth,-halfHeight,halfDepth),	
	btVector3(-halfWidth,-halfHeight,halfDepth),	
	btVector3(halfWidth,halfHeight,-halfDepth),
	btVector3(-halfWidth,halfHeight,-halfDepth),	
	btVector3(halfWidth,-halfHeight,-halfDepth),	
	btVector3(-halfWidth,-halfHeight,-halfDepth)};

	// create the indexes for each triangle, using the 
	// vertices above. Make it static so we don't waste 
	// processing time recreating it over and over again
	static int indices[36] = {
		0,1,2,
		3,2,1,
		4,0,6,
		6,0,2,
		5,1,4,
		4,1,0,
		7,3,1,
		7,1,5,
		5,4,7,
		7,4,6,
		7,2,3,
		7,6,2};

	// start processing vertices as triangles
	glBegin (GL_TRIANGLES);

	// increment the loop by 3 each time since we create a 
	// triangle with 3 vertices at a time.

	for (int i = 0; i < 36; i += 3) {
		// get the three vertices for the triangle based
		// on the index values set above
		// use const references so we don't copy the object
		// (a good rule of thumb is to never allocate/deallocate
		// memory during *every* render/update call. This should 
		// only happen sporadically)
		const btVector3 &vert1 = vertices[indices[i]];
		const btVector3 &vert2 = vertices[indices[i+1]];
		const btVector3 &vert3 = vertices[indices[i+2]];

		// create a normal that is perpendicular to the 
		// face (use the cross product)
		btVector3 normal = (vert3-vert1).cross(vert2-vert1);
		normal.normalize ();

		// set the normal for the subsequent vertices
		glNormal3f(normal.getX(),normal.getY(),normal.getZ());

		// create the vertices
		glVertex3f (vert1.x(), vert1.y(), vert1.z());
		glVertex3f (vert2.x(), vert2.y(), vert2.z());
		glVertex3f (vert3.x(), vert3.y(), vert3.z());
	}

	// stop processing vertices
	glEnd();
}

void BulletOpenGLApplication::DrawSphere(const btScalar &radius) {
	// some constant values for more many segments to build the sphere from
	static int lateralSegments = 25;
	static int longitudinalSegments = 25;

	// iterate laterally
	for(int i = 0; i <= lateralSegments; i++) {
		// do a little math to find the angles of this segment
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lateralSegments);
		btScalar z0  = radius*sin(lat0);
		btScalar zr0 =  radius*cos(lat0);

		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lateralSegments);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);

		// start rendering strips of quads (polygons with 4 poins)
		glBegin(GL_QUAD_STRIP);
		// iterate longitudinally
		for(int j = 0; j <= longitudinalSegments; j++) {
			// determine the points of the quad from the lateral angles
			btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longitudinalSegments;
			btScalar x = cos(lng);
			btScalar y = sin(lng);
			// draw the normals and vertices for each point in the quad
			// since it is a STRIP of quads, we only need to add two points
			// each time to create a whole new quad
			
			// calculate the normal
			btVector3 normal = btVector3(x*zr0, y*zr0, z0);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the first vertex
			glVertex3f(x * zr0, y * zr0, z0);
			
			// calculate the next normal
			normal = btVector3(x*zr1, y*zr1, z1);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the second vertex
			glVertex3f(x * zr1, y * zr1, z1);
			
			// and repeat...
		}
		glEnd();
	}
}

void BulletOpenGLApplication::DrawCylinder(const btScalar &radius, const btScalar &halfHeight) {
	static int slices = 15;
	static int stacks = 10;
	// tweak the starting position of the
	// cylinder to match the physics object
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.0, -halfHeight);
	// create a quadric object to render with
	GLUquadricObj *quadObj = gluNewQuadric();
	// set the draw style of the quadric
	gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
	gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
	// create a disk to cap the cylinder
	gluDisk(quadObj, 0, radius, slices, stacks);
	// create the main hull of the cylinder (no caps)
	gluCylinder(quadObj, radius, radius, 2.f*halfHeight, slices, stacks);
	// shift the position and rotation again
	glTranslatef(0.0f, 0.0f, 2.f*halfHeight);
	glRotatef(-180.0f, 0.0f, 1.0f, 0.0f);
	// draw the cap on the other end of the cylinder
	gluDisk(quadObj, 0, radius, slices, stacks);
	// don't need the quadric anymore, so remove it
	// to save memory
	gluDeleteQuadric(quadObj);
}

void BulletOpenGLApplication::DrawConvexHull(const btCollisionShape* shape) {
	// get the polyhedral data from the convex hull
	const btConvexPolyhedron* pPoly = shape->isPolyhedral() ? ((btPolyhedralConvexShape*) shape)->getConvexPolyhedron() : 0;
	if (!pPoly) return;

	// begin drawing triangles
	glBegin (GL_TRIANGLES);

	// iterate through all faces
	for (int i = 0; i < pPoly->m_faces.size(); i++) {
		// get the indices for the face
		int numVerts = pPoly->m_faces[i].m_indices.size();
		if (numVerts>2)	{
			// iterate through all index triplets
			for (int v = 0; v <pPoly->m_faces[i].m_indices.size()-2;v++) {
				// grab the three vertices
				btVector3 v1 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[0]];
				btVector3 v2 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v+1]];
				btVector3 v3 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v+2]];
				// calculate the normal
				btVector3 normal = (v3-v1).cross(v2-v1);
				normal.normalize ();
				// draw the triangle
				glNormal3f(normal.getX(),normal.getY(),normal.getZ());
				glVertex3f (v1.x(), v1.y(), v1.z());
				glVertex3f (v2.x(), v2.y(), v2.z());
				glVertex3f (v3.x(), v3.y(), v3.z());
			}
		}
	}
	// done drawing
	glEnd ();
}

void BulletOpenGLApplication::DrawCloth(const btCollisionShape* shape, btTransform* bttransform) {
 
	const btSoftBody* pCloth = shape->isSoftBody() ? ((btSoftBodyCollisionShape*) shape)->m_body : 0;
	if (!pCloth) return;

	float x, y, z;
	//Vector3f a, b, c;
	btVector3  v1, v2, v3;
	Vector3f faceNormal;

	glRotatef(-90.0, 0.0, 0.0, 1.0);
	glTranslatef(-1 * bttransform->getOrigin().getX(), -1 * bttransform->getOrigin().getY(), -1 * bttransform->getOrigin().getZ());

	glBegin(GL_TRIANGLES);

	for ( int i = 0; i < pCloth->m_faces.size(); i++ ) {
		for ( unsigned int j = 0; j < 3; j++ ) {
			x = pCloth->m_faces[i].m_n[j]->m_x.x();
			y = pCloth->m_faces[i].m_n[j]->m_x.y();
			z = pCloth->m_faces[i].m_n[j]->m_x.z();

			//if ( j == 0 ) a = Vector3f(x, y, z);
			//if ( j == 1 ) b = Vector3f(x, y, z);
			//if ( j == 2 ) c = Vector3f(x, y, z);
			
			if ( j == 0 ) v1 = btVector3(x, y, z);
			if ( j == 1 ) v2 = btVector3(x, y, z);
			if ( j == 2 ) v3 = btVector3(x, y, z);
		}

				// calculate the normal
				btVector3 normal = (v3-v1).cross(v2-v1);
				normal.normalize ();
				// draw the triangle
				glNormal3f(normal.getX(),normal.getY(),normal.getZ());
				glVertex3f (v1.x(), v1.y(), v1.z());
				glVertex3f (v2.x(), v2.y(), v2.z());
				glVertex3f (v3.x(), v3.y(), v3.z());

		/*
		faceNormal = -Vector3f::Cross(b - a, c - a);
		Vector3f::Normalize(faceNormal);
		glNormal3fv(faceNormal);
		glVertex3fv(a);
		glNormal3fv(faceNormal);
		glVertex3fv(b);
		glNormal3fv(faceNormal);
		glVertex3fv(c);
		*/ 
	}
	
	glEnd();
}

void BulletOpenGLApplication::RotateCamera(float &angle, float value) {
	// change the value (it is passed by reference, so we
	// can edit it here)
	angle -= value; 
	// keep the value within bounds
	if (angle < 0) angle += 360; 
	if (angle >= 360) angle -= 360;
	// update the camera since we changed the angular value
	UpdateCamera(); 
}

void BulletOpenGLApplication::ZoomCamera(float distance) {
	// change the distance value
	m_cameraDistance -= distance;
	// prevent it from zooming in too far
	if (m_cameraDistance < 0.1f) m_cameraDistance = 0.1f;
	// update the camera since we changed the zoom distance
	UpdateCamera();
}

/* 
 * Function directly reads the OpenGL frame buffer pixels and then writes them
 * directly to an open TIFF file.
 */
bool BulletOpenGLApplication::SaveTiffImage(const std::string& filename, const std::string& description, int x, int y, int width, int height, int compression) {
	TIFF *file;
	GLubyte *image, *p;

	file = TIFFOpen(filename.c_str(), "w");
	if ( file == NULL ) {
		std::cerr << "[SaveTiffImage] Error: Cannot open file: " << filename << std::endl;
		return false;
	}

	printf("width %i height %i sizeof(GLubyte) %i = %lli\n", width, height, sizeof(GLubyte) , width * height * sizeof(GLubyte) * 3); 
	image = new GLubyte[width * height * sizeof(GLubyte) * 3];

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

	TIFFSetField(file, TIFFTAG_IMAGEWIDTH, (uint32) width);
	TIFFSetField(file, TIFFTAG_IMAGELENGTH, (uint32) height);
	TIFFSetField(file, TIFFTAG_BITSPERSAMPLE, 8);
	TIFFSetField(file, TIFFTAG_COMPRESSION, compression);
	TIFFSetField(file, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
	TIFFSetField(file, TIFFTAG_SAMPLESPERPIXEL, 3);
	TIFFSetField(file, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(file, TIFFTAG_ROWSPERSTRIP, 1);
	TIFFSetField(file, TIFFTAG_IMAGEDESCRIPTION, description.c_str());

	p = image;
	//--------------------------------------------------------------------------
	// Write each scanline to the open tiff file.
	//--------------------------------------------------------------------------
	for ( int i = height - 1; i >= 0; i-- ) {
    	if ( TIFFWriteScanline(file, p, i, 0) < 0 ) {
			free(image);
			TIFFClose(file);
			return false;
    	}
    	p += width * sizeof(GLubyte) * 3;
	}

	TIFFClose(file);
	return true;
}

void BulletOpenGLApplication::RenderScene() {
	// create an array of 16 floats (representing a 4x4 matrix)
	btScalar transform[16];

	// iterate through all of the objects in our world
	for(GameObjects::iterator i = m_objects.begin(); i != m_objects.end(); ++i) {
		// get the object from the iterator
		GameObject* pObj = *i;

		// read the transform
		pObj->GetTransform(transform);
		btTransform bttransform = btTransform(pObj->GetbtTransform());

		// get data from the object and draw it
		DrawShape(transform, pObj->GetShape(), pObj->GetColor(), &bttransform);
	}

		// after rendering all game objects, perform debug rendering
		// Bullet will figure out what needs to be drawn then call to
		// our DebugDrawer class to do the rendering for us
		m_pWorld->debugDrawWorld();
}



void BulletOpenGLApplication::SaveFrame(float dt) {
	
	m_saveFrameTime += dt;

	if ( m_saveFrameTime >= SAVEFRAMELIMIT && m_saveTiff) {
		m_frameID++;
		std::stringstream stream;
		stream << "F:\\tiff\\slow\\";
		stream << m_frameName;
		stream << m_frameID;
		stream << ".tiff";

		//SERGEY - FIXIT  m_screenWidth, m_screenHeight not defined
		SaveTiffImage(stream.str(), "screen-frame", 0, 0, 1024, 768, 1);
		std::this_thread::sleep_for (std::chrono::milliseconds(200));
		m_frameCount++;
		if (m_frameCount == 600){
			exit(0);
		}
		m_saveFrameTime = 0;
	}
}

void BulletOpenGLApplication::UpdateScene(float dt) {
	// check if the world object exists
	if (m_pWorld) {
		// step the simulation through time. This is called
		// every update and the amount of elasped time was 
		// determined back in ::Idle() by our clock object.
		m_pWorld->stepSimulation(dt);
	}
}

void BulletOpenGLApplication::DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color, btTransform* bttransform) {
	// set the color
	glColor3f(color.x(), color.y(), color.z());

	// push the matrix stack
	glPushMatrix();
	glMultMatrixf(transform);

	// make a different draw call based on the object type
	switch(pShape->getShapeType()) {
		// an internal enum used by Bullet for boxes
	case BOX_SHAPE_PROXYTYPE:
		{
			// assume the shape is a box, and typecast it
			const btBoxShape* box = static_cast<const btBoxShape*>(pShape);
			// get the 'halfSize' of the box
			btVector3 halfSize = box->getHalfExtentsWithMargin();
			// draw the box
			DrawBox(halfSize);
			break;
		}
	case COMPOUND_SHAPE_PROXYTYPE:
		{
			// get the shape
			const btCompoundShape* pCompound = static_cast<const
				btCompoundShape*>(pShape);
			// iterate through the children
			for (int i = 0; i < pCompound->getNumChildShapes(); ++i) {
				// get the transform of the sub-shape
				btTransform thisTransform = pCompound->getChildTransform(i);
				btScalar thisMatrix[16];
				thisTransform.getOpenGLMatrix(thisMatrix);
				// call drawshape recursively for each child. The matrix
				// stack takes care of positioning/orienting the object for us
				DrawShape(thisMatrix, pCompound->getChildShape(i), color, bttransform);
			}
			break;
		}
	case SPHERE_SHAPE_PROXYTYPE:
		{
			// assume the shape is a sphere and typecast it
			const btSphereShape* sphere = static_cast<const btSphereShape*>(pShape);
			// get the sphere's size from the shape
			float radius = sphere->getMargin();
			// draw the sphere
			DrawSphere(radius);
			break;
		}

	case CYLINDER_SHAPE_PROXYTYPE:
		{
			// assume the object is a cylinder
			const btCylinderShape* pCylinder = static_cast<const btCylinderShape*>(pShape);
			// get the relevant data
			float radius = pCylinder->getRadius();
			float halfHeight = pCylinder->getHalfExtentsWithMargin()[1];
			// draw the cylinder
			DrawCylinder(radius,halfHeight);

			break;
		}

	case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			// draw the convex hull shape...whatever it is
			DrawConvexHull(pShape);
			break;
		}
    case SOFTBODY_SHAPE_PROXYTYPE:
		{
			DrawCloth(pShape, bttransform);
			break;
		}
		

	default:
		// unsupported type
		break;
	}

	// pop the stack
	glPopMatrix();
}


GameObject* BulletOpenGLApplication::CreateGameObject(btCollisionShape* pShape, const float &mass, const btVector3 &color, const btVector3 &initialPosition,
													  const btQuaternion &initialRotation, const btVector3 &initialVelocity) {
	// create a new game object
	GameObject* pObject = new GameObject(pShape, mass, color, initialPosition, initialRotation);

	// push it to the back of the list
	m_objects.push_back(pObject);

	pObject->GetRigidBody()->setLinearVelocity(initialVelocity);

	// check if the world object is valid
	if (m_pWorld) {
		// add the object's rigid body to the world
		m_pWorld->addRigidBody(pObject->GetRigidBody());
	}
	return pObject;
}

GameObject* BulletOpenGLApplication::AcceptGameObject(GameObject* pObject) {

	// push it to the back of the list
	m_objects.push_back(pObject);

	// check if the world object is valid
	if (m_pWorld) {
		if(pObject->GetShape()->isSoftBody()){
			m_pWorld->addSoftBody(pObject->GetSoftBody());

		} else {
			// add the object's rigid body to the world
			m_pWorld->addRigidBody(pObject->GetRigidBody());
		}
	}
	return pObject;
}

void BulletOpenGLApplication::AcceptConstraint(btPoint2PointConstraint *testingConstraints) {

	// check if the world object is valid
	if (m_pWorld) {
		m_pWorld->addConstraint(testingConstraints);
	}
}
