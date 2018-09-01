/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// September 2006: VehicleDemo is work in progress, this file is mostly just a placeholderw
/// This VehicleDemo file is very early in development, please check it later
/// One todo is a basic engine model:
/// A function that maps user input (throttle) into torque/force applied on the wheels
/// with gears etc.
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include <stdlib.h>
#include <time.h>

extern char MyHeightfield[];
//
// By default, Bullet Vehicle uses Y as up axis.
// You can override the up axis, for example Z-axis up. Enable this define to see how to:
//#define FORCE_ZAXIS_UP 1
//

#define ARRAY_SIZE_X 3
#define ARRAY_SIZE_Y 10
#define ARRAY_SIZE_Z 3

#define SCALING 1.
#define START_POS_X 50
#define START_POS_Y 4
#define START_POS_Z 50

#define MIN_TOWERS 10
#define MAX_TOWERS 20
#define MIN_TOWER_FLOORS 12
#define MAX_TOWER_FLOORS 24

#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616
#define M_PI_8     0.5 * M_PI_4

#ifdef FORCE_ZAXIS_UP
		int rightIndex = 0; 
		int upIndex = 2; 
		int forwardIndex = 1;
		btVector3 wheelDirectionCS0(0,0,-1);
		btVector3 wheelAxleCS(1,0,0);
#else
		int rightIndex = 0;
		int upIndex = 1;
		int forwardIndex = 2;
		btVector3 wheelDirectionCS0(0,-1,0);
		btVector3 wheelAxleCS(-1,0,0);
#endif

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "VehicleDemo.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float	gEngineForce = 0.f;
float	gBreakingForce = 0.f;

float	maxEngineForce = 1000.f;//this should be engine/velocity dependent
float	maxBreakingForce = 100.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.04f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
float	wheelFriction = 1000;//BT_LARGE_FLOAT;
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;

bool	boostEngine = false;

btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1

////////////////////////////////////

static inline btScalar	UnitRand()
{
	return(rand() / (btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand() * 2 - 1);
}
VehicleDemo::VehicleDemo() : 
m_carChassis(0),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
	m_maxCameraDistance(16.f)
{
	m_vehicle = 0;
	m_wheelShape = 0;
	m_cameraPosition = btVector3(45, 45, 45);
}

VehicleDemo::~VehicleDemo()
{
		//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;

	delete m_vehicleRayCaster;

	delete m_vehicle;

	delete m_wheelShape;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}


void VehicleDemo::createTower(btScalar posX, btScalar posZ)
{
	//TODO: Construir una torre similar a la creada en el codigo del proyecto basicDemo
	// la torre tiene  3x3x10 cajas y altura aleatoria entre MIN_TOWER_FLOORS y MAX_TOWER_FLOORS
	// la torre tiene que aprecer sin derrumbarse
	//los objetos de la  torre tienen que dormirse  y despertarse en dinamicas solo cuando le golpea algun otro objeto

	btTransform startTransform;
	startTransform.setIdentity();

	//esta es la posicion de la torre en el terreno
	float start_x = posX - ARRAY_SIZE_X / 2;
	float start_y = START_POS_Y;
	float start_z = posZ - ARRAY_SIZE_Z / 2;

	btBoxShape* colShape = new btBoxShape(btVector3(SCALING * 1, SCALING * 1, SCALING * 1));
	m_collisionShapes.push_back(colShape);

	int towerFloors = MIN_TOWER_FLOORS + UnitRand() * (MAX_TOWER_FLOORS - MIN_TOWER_FLOORS);

	for (int k = 0; k < towerFloors; k++)
	{
		for (int i = 0; i < ARRAY_SIZE_X; i++)
		{
			for (int j = 0; j < ARRAY_SIZE_Z; j++)
			{
				startTransform.setOrigin(SCALING*btVector3(
					btScalar(2.0*i + start_x),
					btScalar(2.0*k + start_y),
					btScalar(2.0*j + start_z)));

				btScalar	mass(1.f);
				bool isDynamic = (mass != 0.f);

				btVector3 localInertia(0, 0, 0);
				if (isDynamic)
					colShape->calculateLocalInertia(mass, localInertia);

				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);

				rbInfo.m_restitution = 1.0f;

				btRigidBody* body = new btRigidBody(rbInfo);


				m_dynamicsWorld->addRigidBody(body);
			}
		}
	}
}

btRaycastVehicle* VehicleDemo::createVagon( btRaycastVehicle* parent_vehicle)
{
	//TODO: constuir un vehiculo de bullet in engancharlo con una constraint al padre como si fuera un remolque
	// devuelve un puntero al vehiculo creado
	//NOTA: el vehiculo de bullet no tiene ruedas fisicas, para ponerserlas al remolque teneis que hacerlo en el metodo
	// void VehicleDemo::renderme()
	// similar a como se hace con el coche


	if (!parent_vehicle) return 0;

	btRigidBody* parent = parent_vehicle->getRigidBody();//get the rigid body of the car

	btVector3 parentPos = parent->getCenterOfMassPosition();//posicion del coche
	btVector3 aabbMin, aabbMax;
	parent->getAabb(aabbMin, aabbMax);
	btScalar parentSizeX = (aabbMax.getX() - aabbMin.getX()); //ancho del coche
	btScalar parentSizeY = (aabbMax.getY() - aabbMin.getY()); //altura del coche
	btScalar parentSizeZ = (aabbMax.getZ() - aabbMin.getZ()); //longitud del coche

	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 1, 0));

	compound->addChildShape(localTrans, chassisShape);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(parentPos + btVector3(0.0f, 0, -(parentSizeZ + 0.5f)));

	btRigidBody* carChassis = localCreateRigidBody(100, tr, compound);//chassisShape);
														   //m_carChassis->setDamping(0.2,0.2);
	
	btVehicleRaycaster* vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
	btRaycastVehicle* vehicle = new btRaycastVehicle(m_tuning, carChassis, vehicleRayCaster);

	//never deactivate the vehicle
	carChassis->setActivationState(DISABLE_DEACTIVATION);
	m_dynamicsWorld->addVehicle(vehicle);

	float connectionHeight = 1.2f;
	bool isFrontWheel = true;
	//choose coordinate system
	vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

	btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);
	vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
	connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);
	vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
	connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
	isFrontWheel = false;
	vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
	connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (0.3*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
	vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
	
	for (int i = 0; i< vehicle->getNumWheels(); i++)
	{
		btWheelInfo& wheel = vehicle->getWheelInfo(i);
		wheel.m_suspensionStiffness = suspensionStiffness;
		wheel.m_wheelsDampingRelaxation = suspensionDamping;
		wheel.m_wheelsDampingCompression = suspensionCompression;
		wheel.m_frictionSlip = 100.0f;
		wheel.m_rollInfluence = 1.0f;
	}
	
	btPoint2PointConstraint* constraint = new btPoint2PointConstraint(*parent, *carChassis, btVector3(0.0f, -parentSizeY * 0.5f, -(parentSizeZ * 0.5f + 0.5f)), btVector3(0.0f, -parentSizeY * 0.5f, parentSizeZ * 0.5f + 0.5f));
	m_dynamicsWorld->addConstraint(constraint, true);
	
	return vehicle;
}

void VehicleDemo::initPhysics()
{
	
#ifdef FORCE_ZAXIS_UP
	m_cameraUp = btVector3(0,0,1);
	m_forwardAxis = 1;
#endif

	btCollisionShape* groundShape = new btBoxShape(btVector3(500,3,500));
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
#ifdef FORCE_ZAXIS_UP
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));
#endif 

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
btTransform tr;
tr.setIdentity();
	m_collisionShapes.push_back(groundShape);

	//create ground object
	localCreateRigidBody(0,tr,groundShape);
	tr.setOrigin(btVector3(0,0,0));//-64.5f,0));

#ifdef FORCE_ZAXIS_UP
//   indexRightAxis = 0; 
//   indexUpAxis = 2; 
//   indexForwardAxis = 1; 
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,2.f, 0.5f));
	btCompoundShape* compound = new btCompoundShape();
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,0,1));
#else
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound); 
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,1,0));
#endif

	compound->addChildShape(localTrans,chassisShape);

	tr.setOrigin(btVector3(0,0.f,0));

	m_carChassis = localCreateRigidBody(800,tr,compound);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);

	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	
//	clientResetScene();

	/// create vehicle
	{
		
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(m_vehicle);

		float connectionHeight = 1.2f;

		bool isFrontWheel=true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}

	m_firstVagon = createVagon(m_vehicle);
	m_secondVagon= createVagon(m_firstVagon);

	for (size_t i = 0; i < 10; ++i)
	{
		createTower(SignedUnitRand() * 100, SignedUnitRand() * 100);
	}

	setCameraDistance(26.f);

}


//to be implemented by the demo
void VehicleDemo::renderme()
{
	
	updateCamera();

	btScalar m[16];
	int i;


	btVector3 wheelColor(1,0,0);

	btVector3	worldBoundsMin,worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);



	for (i=0;i<m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i,true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
	}

	for (i = 0; i< m_firstVagon->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_firstVagon->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_firstVagon->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		m_shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, getDebugMode(), worldBoundsMin, worldBoundsMax);
	}

	for (i = 0; i< m_secondVagon->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_secondVagon->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_secondVagon->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		m_shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, getDebugMode(), worldBoundsMin, worldBoundsMax);
	}


	DemoApplication::renderme();

}

void VehicleDemo::clientMoveAndDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	
	{			
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);

		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		printf("wheel1: %f\n", m_vehicle->getSteeringValue(wheelIndex));
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		printf("wheel1: %f\n", m_vehicle->getSteeringValue(wheelIndex));
		
	}

	if (boostEngine)
	{

		float speed = m_vehicle->getCurrentSpeedKmHour();
		if (speed > 0)
		{
			float strenght = speed*4.0f;
			btRigidBody* body = m_vehicle->getRigidBody();
			btVector3 dir = m_vehicle->getForwardVector();
			dir += btVector3(0, 1.0f, 0);
			dir.normalize();

			body->applyCentralImpulse(strenght*dir);

		}
		boostEngine = false;
	}


	float dt = getDeltaTimeMicroseconds() * 0.000001f;
	
	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_FEEDBACK

	}

	
	




#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 


	renderme(); 

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	

	glFlush();
	glutSwapBuffers();

}

void VehicleDemo::displayCallback(void) 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void VehicleDemo::clientResetScene()
{
	gVehicleSteering = 0.f;
	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}

}

void VehicleDemo::specialKeyboardUp(int key, int x, int y)
{
   switch (key) 
    {
    case GLUT_KEY_UP :
		{
			gEngineForce = 0.f;
		break;
		}
	case GLUT_KEY_DOWN :
		{			
			gBreakingForce = 0.f; 
		break;
		}
	default:
		DemoApplication::specialKeyboardUp(key,x,y);
        break;
    }

}


void VehicleDemo::specialKeyboard(int key, int x, int y)
{

//	printf("key = %i x=%i y=%i\n",key,x,y);

	int state = glutGetModifiers();
	if (state & GLUT_ACTIVE_SHIFT)
	{
		switch (key)
		{
			case GLUT_KEY_UP:
			{
				boostEngine = true;
				break;
			}
			default:
				DemoApplication::specialKeyboard(key, x, y);
				break;
		}
	}
	else
	{
		switch (key)
		{
		case GLUT_KEY_LEFT:
		{
			gVehicleSteering += steeringIncrement;
			if (gVehicleSteering > steeringClamp)
				gVehicleSteering = steeringClamp;

			break;
		}
		case GLUT_KEY_RIGHT:
		{
			gVehicleSteering -= steeringIncrement;
			if (gVehicleSteering < -steeringClamp)
				gVehicleSteering = -steeringClamp;

			break;
		}
		case GLUT_KEY_UP:
		{
			gEngineForce = maxEngineForce;
			gBreakingForce = 0.f;
			break;
		}
		case GLUT_KEY_DOWN:
		{
			gBreakingForce = maxBreakingForce;
			gEngineForce = 0.f;
			break;
		}
		default:
			DemoApplication::specialKeyboard(key, x, y);
			break;
		}
	}

//	glutPostRedisplay();


}



void	VehicleDemo::updateCamera()
{
	
//#define DISABLE_CAMERA 1
#ifdef DISABLE_CAMERA
	DemoApplication::updateCamera();
	return;
#endif //DISABLE_CAMERA

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btTransform chassisWorldTrans;

	//look at the vehicle
	m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
	m_cameraTargetPosition = chassisWorldTrans.getOrigin();

	//interpolate the camera height
#ifdef FORCE_ZAXIS_UP
	m_cameraPosition[2] = (15.0*m_cameraPosition[2] + m_cameraTargetPosition[2] + m_cameraHeight)/16.0;
#else
	m_cameraPosition[1] = (15.0*m_cameraPosition[1] + m_cameraTargetPosition[1] + m_cameraHeight)/16.0;
#endif 

	btVector3 camToObject = m_cameraTargetPosition - m_cameraPosition;

	//keep distance between min and max distance
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	if (cameraDistance < m_minCameraDistance)
	{
		correctionFactor = 0.15*(m_minCameraDistance-cameraDistance)/cameraDistance;
	}
	if (cameraDistance > m_maxCameraDistance)
	{
		correctionFactor = 0.15*(m_maxCameraDistance-cameraDistance)/cameraDistance;
	}
	m_cameraPosition -= correctionFactor*camToObject;
	
	  btScalar aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
        glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

         glMatrixMode(GL_MODELVIEW);
         glLoadIdentity();

    gluLookAt(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],
                      m_cameraTargetPosition[0],m_cameraTargetPosition[1], m_cameraTargetPosition[2],
                          m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());



}

