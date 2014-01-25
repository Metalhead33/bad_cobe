#include <iostream>
#include <fstream>
#include <irrlicht.h>
#include <tokamak.h>
#include <time.h>
#include "lua.hpp"
#include "luadef.h"
#include "initialize.h"
using namespace std;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

#define PI 3.1415926
#define CUBECOUNT 30
#define CUBEX 5.0
#define CUBEY 5.0
#define CUBEZ 5.0
#define CUBEMASS 50.0f
#define FLOORSIZE 300

class InitializeLua
{
public:
    InitializeLua(){
	L = lua_open();
	luaL_openlibs(L);
	luaL_dofile(L, "races.lua");
	Initialize("races","InitializeRace");
	luaL_dofile(L, "classes.lua");
	Initialize("classes","InitializeClass");
	luaL_dofile(L, "materials.lua");
	Initialize("materials","InitializeMaterial");}};

irr::scene::ICameraSceneNode* camera = 0;

neSimulator *gSim = NULL;

neRigidBody *gCubes[CUBECOUNT];
neAnimatedBody *gFloor = NULL;

bool gbUseHFTimer;
int gCurrentTime;
float gfTimeScale;

enum
{
    // I use this ISceneNode ID to indicate a scene node that is
    // not pickable by getSceneNodeAndCollisionPointFromRay()
    ID_IsNotPickable = 0,

    // I use this flag in ISceneNode IDs to indicate that the
    // scene node can be picked by ray selection.
    IDFlag_IsPickable = 1 << 0,

    // I use this flag in ISceneNode IDs to indicate that the
    // scene node can be highlighted.  In this example, the
    // homonids can be highlighted, but the level mesh can't.
    IDFlag_IsHighlightable = 1 << 1
};

class MyEventReceiver : public irr::IEventReceiver
{
public:
    virtual bool OnEvent(const irr::SEvent& event)
    {
        if (camera)
            return camera->OnEvent(event);

        return false;
    }
};

class PhysicsCubeNode: public irr::scene::ISceneNode
{
    irr::core::aabbox3d<irr::f32> Box;
    irr::video::S3DVertex Vertices[8];
    irr::video::SMaterial Material;

public:

    PhysicsCubeNode(irr::scene::ISceneNode* parent,
                    irr::scene::ISceneManager* mgr, irr::s32 id)
        : irr::scene::ISceneNode(parent, mgr, id)
    {
        Material.Wireframe = false;
        Material.Lighting = false;

        Vertices[0] = irr::video::S3DVertex(
                          -CUBEX/2,-CUBEY / 2,-CUBEZ/2, 1,1,0,
                          irr::video::SColor(255,0,255,255), 0, 1);
        Vertices[1] = irr::video::S3DVertex(
                          CUBEX/2,-CUBEY / 2,-CUBEZ/2, 1,0,0,
                          irr::video::SColor(255,255,0,255), 1, 1);
        Vertices[2] = irr::video::S3DVertex(
                          CUBEX/2,-CUBEY / 2, CUBEZ/2, 0,1,1,
                          irr::video::SColor(255,255,255,0), 1, 0);
        Vertices[3] = irr::video::S3DVertex(
                          -CUBEX/2,-CUBEY / 2, CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,0,255,0), 0, 0);
        Vertices[4] = irr::video::S3DVertex(
                          -CUBEX/2, CUBEY / 2,-CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,0,0,255), 0, 0);
        Vertices[5] = irr::video::S3DVertex(
                          CUBEX/2, CUBEY / 2,-CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,255,0,0), 0, 0);
        Vertices[6] = irr::video::S3DVertex(
                          CUBEX/2, CUBEY / 2, CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,0,0,0), 0, 0);
        Vertices[7] = irr::video::S3DVertex(
                          -CUBEX/2, CUBEY / 2, CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,255,255,255), 0, 0);

        Box.reset(Vertices[0].Pos);
        for (s32 i=1; i<8; ++i)
            Box.addInternalPoint(Vertices[i].Pos);
    }


    virtual void OnRegisterSceneNode()
    {
        if (IsVisible)
            SceneManager->registerNodeForRendering(this);

        ISceneNode::OnRegisterSceneNode();
    }

    virtual void render()
    {
        irr::u16 indices[] = {0,1,2,
                              0,2,3,
                              1,6,2,
                              1,5,6,
                              0,5,1,
                              0,4,5,
                              0,3,7,
                              0,7,4,
                              2,7,3,
                              2,6,7,
                              4,6,5,
                              4,7,6
                             };

        irr::video::IVideoDriver* driver = SceneManager->getVideoDriver();

        driver->setMaterial(Material);
        driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);
        driver->drawIndexedTriangleList(&Vertices[0], 8, &indices[0], 12);
    }

    virtual const irr::core::aabbox3d<f32>& getBoundingBox() const
    {
        return Box;
    }

    virtual s32 getMaterialCount()
    {
        return 1;
    }

    virtual irr::video::SMaterial& getMaterial(s32 i)
    {
        return Material;
    }
};


class PhysicsFloorNode: public irr::scene::ISceneNode
{
private:
    irr::core::aabbox3d<irr::f32> Box;
    irr::video::S3DVertex Vertices[4];
    irr::video::SMaterial Material;

public:

    PhysicsFloorNode(irr::scene::ISceneNode* parent,
                     irr::scene::ISceneManager* mgr, irr::s32 id)
        : irr::scene::ISceneNode(parent, mgr, id)
    {
        Material.Wireframe = false;
        Material.Lighting = false;

        Vertices[0] = irr::video::S3DVertex(
                          -FLOORSIZE/2,0.0,-FLOORSIZE/2, 1,1,0,
                          irr::video::SColor(255,0,0,0), 0, 1);
        Vertices[1] = irr::video::S3DVertex(
                          FLOORSIZE/2,0.0,-FLOORSIZE/2, 1,0,0,
                          irr::video::SColor(255,0,0,0), 1, 1);
        Vertices[2] = irr::video::S3DVertex(
                          FLOORSIZE/2,0.0, FLOORSIZE/2, 0,1,1,
                          irr::video::SColor(255,0,0,0), 1, 0);
        Vertices[3] = irr::video::S3DVertex(
                          -FLOORSIZE/2,0.0, FLOORSIZE/2, 0,0,1,
                          irr::video::SColor(255,0,0,0), 0, 0);

        Box.reset(Vertices[0].Pos);
        for (s32 i=1; i<4; ++i)
            Box.addInternalPoint(Vertices[i].Pos);
    }

    virtual void OnRegisterSceneNode()
    {
        if (IsVisible)
            SceneManager->registerNodeForRendering(this);

        ISceneNode::OnRegisterSceneNode();
    }

    virtual void render()
    {
        irr::u16 indices[] = {0,1,2,
                              0,2,3,
                              0,2,1,
                              0,3,2
                             };

        irr::video::IVideoDriver* driver = SceneManager->getVideoDriver();

        driver->setMaterial(Material);
        driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);
        driver->drawIndexedTriangleList(&Vertices[0], 4, &indices[0], 4);
    }

    virtual const irr::core::aabbox3d<f32>& getBoundingBox() const
    {
        return Box;
    }

    virtual s32 getMaterialCount()
    {
        return 1;
    }

    virtual irr::video::SMaterial& getMaterial(s32 i)
    {
        return Material;
    }
};

bool InitPhysics(void)
{
    neGeometry *geom; // Pointer to a Geometry object
    // which we'll use to define the shape/size
    // of each cube
    neV3 boxSize1;    // A variable to store the length, width
    // and height of the cube
    neV3 gravity;     // A vector to store the direction and
    //intensity of gravity
    neV3 pos;         // The position of a cube
    f32 mass;         // The mass of our cubes
    neSimulatorSizeInfo sizeInfo; // SizeInfo stores data
    // about how manyobjects we are going to model
    int i;

    sizeInfo.rigidBodiesCount = CUBECOUNT;

    sizeInfo.animatedBodiesCount = 1;

    s32 totalBody = sizeInfo.rigidBodiesCount +
                    sizeInfo.animatedBodiesCount;
    sizeInfo.geometriesCount = totalBody;

    sizeInfo.overlappedPairsCount =
        totalBody * (totalBody - 1) / 2;

    sizeInfo.rigidParticleCount = 0;
    sizeInfo.constraintsCount = 0;
    sizeInfo.terrainNodesStartCount = 0;

    gravity.Set(0.0f, -10.0f, 0.0f);

    gSim = neSimulator::CreateSimulator(sizeInfo, NULL, &gravity);

    srand(time(0));

    for (i=0; i<CUBECOUNT; i++)
    {
        // Create a rigid body
        gCubes[i] = gSim->CreateRigidBody();

        // Add geometry to the body and set it to be a box
        // of dimensions 1, 1, 1
        geom = gCubes[i]->AddGeometry();
        boxSize1.Set(CUBEX, CUBEY, CUBEZ);
        geom->SetBoxSize(boxSize1[0], boxSize1[1], boxSize1[2]);

        // Update the bounding info of the object -- must always call this
        // after changing a body's geometry.
        gCubes[i]->UpdateBoundingInfo();

        // Set other properties of the object (mass, position, etc.)
        mass =CUBEMASS;
        gCubes[i]->SetInertiaTensor(
            neBoxInertiaTensor(boxSize1[0], boxSize1[1], boxSize1[2], mass));
        gCubes[i]->SetMass(mass);

        // Vary the position so the cubes don't all exactly
        // stack on top of each other
        // (makes for a more interesting simulation)

        // MC - You may need to play with the randomization
        // stuff, since your cubemay be to big to
        // be easily destabelized
        pos.Set((rand()%10) / 20.0f * CUBEX, 4.0f + i * (CUBEY + 1),
                (rand()%10)/ 20.0f * CUBEZ);
        gCubes[i]->SetPos(pos);
    }

    gFloor = gSim->CreateAnimatedBody();

    geom = gFloor->AddGeometry();
    boxSize1.Set(FLOORSIZE, 0.0f, FLOORSIZE);
    geom->SetBoxSize(boxSize1[0],boxSize1[1],boxSize1[2]);
    gFloor->UpdateBoundingInfo();

    pos.Set(0.0f, 0.0f, 0.0f);
    gFloor->SetPos(pos);
// All done
    return true;
}

void KillPhysics(void)
{
    if (gSim)
    {
        // Destroy the simulator.
        // Note that this will release all related
        // resources that we've allocated.
        neSimulator::DestroySimulator(gSim);
        gSim = NULL;
    }
}

irr::ITimer* TIMER = 0;
bool InitTimer(void)
{
    gfTimeScale = 0.001f;
    gCurrentTime =TIMER->getTime();
    gbUseHFTimer = false;
    return true;
}

float GetElapsedTime()
{
    int newTime;
    float fElapsed;

    newTime=TIMER->getTime();

    // Scale accordingly
    int diff = newTime - gCurrentTime;
    fElapsed = (float)(diff * gfTimeScale);

    // Save the new time value for the next time we're called
    gCurrentTime = newTime;

    return fElapsed;
}

int main()
{
    //irr - variable
    //MyEventReceiver receiver;

    //tok - timer stuff
    float fElapsed;
    static float fLastElapsed;
    int i;

    //MC - I needed a vector3df to pass to the Node movement functions
    irr::core::vector3df TempVect;

    video::E_DRIVER_TYPE driverType;
    driverType = video::EDT_OPENGL;
	// create device


	IrrlichtDevice *device =
		createDevice(driverType, core::dimension2d<u32>(640, 480), 16, false);

	if (device == 0)
		return 1; // could not create selected driver.

    TIMER = device->getTimer();

	video::IVideoDriver* driver = device->getVideoDriver();
	scene::ISceneManager* smgr = device->getSceneManager();

	device->getFileSystem()->addFileArchive("../maps/map-20kdm2.pk3");

	scene::IAnimatedMesh* q3levelmesh = smgr->getMesh("20kdm2.bsp");
	scene::IMeshSceneNode* q3node = 0;

	// The Quake mesh is pickable, but doesn't get highlighted.
	if (q3levelmesh)
		q3node = smgr->addOctreeSceneNode(q3levelmesh->getMesh(0), 0, IDFlag_IsPickable);

	/*
	So far so good, we've loaded the quake 3 level like in tutorial 2. Now,
	here comes something different: We create a triangle selector. A
	triangle selector is a class which can fetch the triangles from scene
	nodes for doing different things with them, for example collision
	detection. There are different triangle selectors, and all can be
	created with the ISceneManager. In this example, we create an
	OctreeTriangleSelector, which optimizes the triangle output a little
	bit by reducing it like an octree. This is very useful for huge meshes
	like quake 3 levels. After we created the triangle selector, we attach
	it to the q3node. This is not necessary, but in this way, we do not
	need to care for the selector, for example dropping it after we do not
	need it anymore.
	*/

	scene::ITriangleSelector* selector = 0;

	if (q3node)
	{
		q3node->setPosition(core::vector3df(-1350,-130,-1400));

		selector = smgr->createOctreeTriangleSelector(
				q3node->getMesh(), q3node, 128);
		q3node->setTriangleSelector(selector);
		// We're not done with this selector yet, so don't drop it.
	}


	/*
	We add a first person shooter camera to the scene so that we can see and
	move in the quake 3 level like in tutorial 2. But this, time, we add a
	special animator to the camera: A Collision Response animator. This
	animator modifies the scene node to which it is attached to in order to
	prevent it moving through walls, and to add gravity to it. The
	only thing we have to tell the animator is how the world looks like,
	how big the scene node is, how much gravity to apply and so on. After the
	collision response animator is attached to the camera, we do not have to do
	anything more for collision detection, anything is done automatically.
	The rest of the collision detection code below is for picking. And please
	note another cool feature: The collision response animator can be
	attached also to all other scene nodes, not only to cameras. And it can
	be mixed with other scene node animators. In this way, collision
	detection and response in the Irrlicht engine is really easy.

	Now we'll take a closer look on the parameters of
	createCollisionResponseAnimator(). The first parameter is the
	TriangleSelector, which specifies how the world, against collision
	detection is done looks like. The second parameter is the scene node,
	which is the object, which is affected by collision detection, in our
	case it is the camera. The third defines how big the object is, it is
	the radius of an ellipsoid. Try it out and change the radius to smaller
	values, the camera will be able to move closer to walls after this. The
	next parameter is the direction and speed of gravity.  We'll set it to
	(0, -10, 0), which approximates to realistic gravity, assuming that our
	units are metres. You could set it to (0,0,0) to disable gravity. And the
	last value is just a translation: Without this, the ellipsoid with which
	collision detection is done would be around the camera, and the camera would
	be in the middle of the ellipsoid. But as human beings, we are used to have our
	eyes on top of the body, with which we collide with our world, not in
	the middle of it. So we place the scene node 50 units over the center
	of the ellipsoid with this parameter. And that's it, collision
	detection works now.
	*/

	// Set a jump speed of 3 units per second, which gives a fairly realistic jump
	// when used with the gravity of (0, -10, 0) in the collision response animator.
	scene::ICameraSceneNode* camera =
		smgr->addCameraSceneNodeFPS(0, 100.0f, .3f, ID_IsNotPickable, 0, 0, true, 3.f);
	camera->setPosition(core::vector3df(50,50,-60));
	camera->setTarget(core::vector3df(-70,30,-60));

	if (selector)
	{
		scene::ISceneNodeAnimator* anim = smgr->createCollisionResponseAnimator(
			selector, camera, core::vector3df(30,50,30),
			core::vector3df(0,-10,0), core::vector3df(0,30,0));
		selector->drop(); // As soon as we're done with the selector, drop it.
		camera->addAnimator(anim);
		anim->drop();  // And likewise, drop the animator when we're done referring to it.
	}

	// Now I create three animated characters which we can pick, a dynamic light for
	// lighting them, and a billboard for drawing where we found an intersection.

	// First, let's get rid of the mouse cursor.  We'll use a billboard to show
	// what we're looking at.
	device->getCursorControl()->setVisible(false);

//MC - This is a major step, in case it's not obvious.
// Initialize Tokamak
    InitPhysics();

//MC - Here we set up our array of cubes then create them.
// Make sure you DO NOT drop() the cubes, since we need to
// use the pointer to refer back to the
// cubes to position them
    PhysicsCubeNode *CubeNode[CUBECOUNT];

    for (i=0; i<CUBECOUNT; i++)
    {
        CubeNode[i] = new PhysicsCubeNode(
            smgr->getRootSceneNode(), smgr, 666);
    }

	// Add the billboard.
	scene::IBillboardSceneNode * bill = smgr->addBillboardSceneNode();
	bill->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR );
	bill->setMaterialTexture(0, driver->getTexture("../maps/particle.bmp"));
	bill->setMaterialFlag(video::EMF_LIGHTING, false);
	bill->setMaterialFlag(video::EMF_ZBUFFER, false);
	bill->setSize(core::dimension2d<f32>(20.0f, 20.0f));
	bill->setID(ID_IsNotPickable); // This ensures that we don't accidentally ray-pick it

//MC - I'm one of those programmers that will fit 2
// or more commands on one line if I can.  Plus, I
// think this is a really cute way of calling and
// dropping a node without using a variable. I use
// the magic of C++ to create a new Node and drop
// that same node in the same look.  If I was doing
// this the long way I would need a PhysicsFloorNode
// pointer
    /*(new PhysicsFloorNode(smgr->getRootSceneNode(),
                          smgr, 666))->drop();*/

	/* Add 3 animated hominids, which we can pick using a ray-triangle intersection.
	They all animate quite slowly, to make it easier to see that accurate triangle
	selection is being performed. */
	scene::IAnimatedMeshSceneNode* node = 0;

	video::SMaterial material;

	// Add an MD2 node, which uses vertex-based animation.
	node = smgr->addAnimatedMeshSceneNode(smgr->getMesh("../maps/faerie.md2"),
						0, IDFlag_IsPickable | IDFlag_IsHighlightable);
	node->setPosition(core::vector3df(-90,-15,-140)); // Put its feet on the floor.
	node->setScale(core::vector3df(1.6f)); // Make it appear realistically scaled
	node->setMD2Animation(scene::EMAT_POINT);
	node->setAnimationSpeed(20.f);
	material.setTexture(0, driver->getTexture("../maps/faerie2.bmp"));
	material.Lighting = true;
	material.NormalizeNormals = true;
	node->getMaterial(0) = material;

	// Now create a triangle selector for it.  The selector will know that it
	// is associated with an animated node, and will update itself as necessary.
	selector = smgr->createTriangleSelector(node);
	node->setTriangleSelector(selector);
	selector->drop(); // We're done with this selector, so drop it now.

	// And this B3D file uses skinned skeletal animation.
	node = smgr->addAnimatedMeshSceneNode(smgr->getMesh("../maps/ninja.b3d"),
						0, IDFlag_IsPickable | IDFlag_IsHighlightable);
	node->setScale(core::vector3df(10));
	node->setPosition(core::vector3df(-75,-66,-80));
	node->setRotation(core::vector3df(0,90,0));
	node->setAnimationSpeed(8.f);
	node->getMaterial(0).NormalizeNormals = true;
	node->getMaterial(0).Lighting = true;
	// Just do the same as we did above.
	selector = smgr->createTriangleSelector(node);
	node->setTriangleSelector(selector);
	selector->drop();

	// This X files uses skeletal animation, but without skinning.
	node = smgr->addAnimatedMeshSceneNode(smgr->getMesh("../maps/dwarf.x"),
						0, IDFlag_IsPickable | IDFlag_IsHighlightable);
	node->setPosition(core::vector3df(-70,-66,-30)); // Put its feet on the floor.
	node->setRotation(core::vector3df(0,-90,0)); // And turn it towards the camera.
	node->setAnimationSpeed(20.f);
	node->getMaterial(0).Lighting = true;
	selector = smgr->createTriangleSelector(node);
	node->setTriangleSelector(selector);
	selector->drop();


	// And this mdl file uses skinned skeletal animation.
	node = smgr->addAnimatedMeshSceneNode(smgr->getMesh("../maps/yodan.mdl"),
						0, IDFlag_IsPickable | IDFlag_IsHighlightable);
	node->setPosition(core::vector3df(-90,-25,20));
	node->setScale(core::vector3df(0.8f));
	node->getMaterial(0).Lighting = true;
	node->setAnimationSpeed(20.f);

	// Just do the same as we did above.
	selector = smgr->createTriangleSelector(node);
	node->setTriangleSelector(selector);
	selector->drop();

	material.setTexture(0, 0);
	material.Lighting = false;

	// Add a light, so that the unselected nodes aren't completely dark.
	scene::ILightSceneNode * light = smgr->addLightSceneNode(0, core::vector3df(-60,100,400),
		video::SColorf(1.0f,1.0f,1.0f,1.0f), 600.0f);
	light->setID(ID_IsNotPickable); // Make it an invalid target for selection.

//tok - Initialize the timer and the variable to
// count the last number of milliseconds.
//      this makes sure there isn't a wierd jump in the program.
    InitTimer();
    fLastElapsed = 0;

	// Remember which scene node is highlighted
	scene::ISceneNode* highlightedSceneNode = 0;
	scene::ISceneCollisionManager* collMan = smgr->getSceneCollisionManager();
	int lastFPS = -1;

	// draw the selection triangle only as wireframe
	material.Wireframe=true;

    while(device->run())
    {

        //tok - Find out how much time has elapsed
        //      since we were last called
        fElapsed = GetElapsedTime();

        if (fLastElapsed+fElapsed > 0.001)
        {
            if (fLastElapsed+fElapsed < 0.01)//fix the time step
                gSim->Advance(fLastElapsed+fElapsed);
            else
                gSim->Advance(0.01);
            fLastElapsed = 0;
        }
        else
            fLastElapsed = fLastElapsed+fElapsed;

        //MC - here where the real magic happens.
        //     We loop through each cube;
        for (i=0; i<CUBECOUNT; i++)
        {

            //First, we get the position from the Tokamak cube

            neV3 p = gCubes[i]->GetPos();

            //Now, we set up out temporary vector
            TempVect.X = p[0];
            TempVect.Y = p[1];
            TempVect.Z = p[2];

            //And set the position of the cube that Irrlicht is going to draw
            CubeNode[i]->setPosition(TempVect);

            //Last, we get the rotation from the Tokamak cube
            // This is a the rotation quaternion, I believe
            // More information is found here:
            // http://www.gamedev.net/reference/articl ... le1095.asp
            // (Thanks to unnamed forum user)
            neQ q = gCubes[i]->GetRotationQ();

            //Again, the temporary vector is set
            TempVect.Y = q.Y;
            TempVect.X = q.X;
            TempVect.Z = q.Z;

            //we convert it to degrees and set the Irrlicht cube's rotation
            TempVect *= 180 / PI;
            CubeNode[i]->setRotation(TempVect);
        }

        //irr - Now we draw it all
        driver->beginScene(true, true, irr::video::SColor(0,100,100,100));

        smgr->drawAll();

        driver->endScene();

        int fps = driver->getFPS();

        if (lastFPS != fps)
        {
            wchar_t tmp[1024];
            swprintf(tmp, 1024, L"Physics Example- Irrlicht & Tokamak"\
                     L" Engines(fps:%d)", fps);

            device->setWindowCaption(tmp);
            lastFPS = fps;
        }
    }

	if (device->isWindowActive())
	{
		driver->beginScene(true, true, 0);
		smgr->drawAll();

		// Unlight any currently highlighted scene node
		if (highlightedSceneNode)
		{
			highlightedSceneNode->setMaterialFlag(video::EMF_LIGHTING, true);
			highlightedSceneNode = 0;
		}

		// All intersections in this example are done with a ray cast out from the camera to
		// a distance of 1000.  You can easily modify this to check (e.g.) a bullet
		// trajectory or a sword's position, or create a ray from a mouse click position using
		// ISceneCollisionManager::getRayFromScreenCoordinates()
		core::line3d<f32> ray;
		ray.start = camera->getPosition();
		ray.end = ray.start + (camera->getTarget() - ray.start).normalize() * 1000.0f;

		// Tracks the current intersection point with the level or a mesh
		core::vector3df intersection;
		// Used to show with triangle has been hit
		core::triangle3df hitTriangle;

		// This call is all you need to perform ray/triangle collision on every scene node
		// that has a triangle selector, including the Quake level mesh.  It finds the nearest
		// collision point/triangle, and returns the scene node containing that point.
		// Irrlicht provides other types of selection, including ray/triangle selector,
		// ray/box and ellipse/triangle selector, plus associated helpers.
		// See the methods of ISceneCollisionManager
		scene::ISceneNode * selectedSceneNode =
			collMan->getSceneNodeAndCollisionPointFromRay(
					ray,
					intersection, // This will be the position of the collision
					hitTriangle, // This will be the triangle hit in the collision
					IDFlag_IsPickable, // This ensures that only nodes that we have
							// set up to be pickable are considered
					0); // Check the entire scene (this is actually the implicit default)

		// If the ray hit anything, move the billboard to the collision position
		// and draw the triangle that was hit.
		if(selectedSceneNode)
		{
			bill->setPosition(intersection);

			// We need to reset the transform before doing our own rendering.
			driver->setTransform(video::ETS_WORLD, core::matrix4());
			driver->setMaterial(material);
			driver->draw3DTriangle(hitTriangle, video::SColor(0,255,0,0));

			// We can check the flags for the scene node that was hit to see if it should be
			// highlighted. The animated nodes can be highlighted, but not the Quake level mesh
			if((selectedSceneNode->getID() & IDFlag_IsHighlightable) == IDFlag_IsHighlightable)
			{
				highlightedSceneNode = selectedSceneNode;

				// Highlighting in this case means turning lighting OFF for this node,
				// which means that it will be drawn with full brightness.
				highlightedSceneNode->setMaterialFlag(video::EMF_LIGHTING, false);
			}
		}

		// We're all done drawing, so end the scene.
		driver->endScene();

		int fps = driver->getFPS();

		if (lastFPS != fps)
		{
			core::stringw str = L"Collision detection example - Irrlicht Engine [";
			str += driver->getName();
			str += "] FPS:";
			str += fps;

			device->setWindowCaption(str.c_str());
			lastFPS = fps;
		}
	}


//tok - When we're one, we kill the physics engine
    KillPhysics();

//MC - And drop all the node pointers that we don't need anymore
    for (i=0; i<CUBECOUNT; i++)
        CubeNode[i]->drop();

	device->drop();

	return 0;
}
