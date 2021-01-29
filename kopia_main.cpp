#include "glew.h"
#include "freeglut.h"
#include "glm.hpp"
#include "ext.hpp"
#include <iostream>
#include <cmath>

#include "Texture.h"
#include "Shader_Loader.h"
#include "Render_Utils.h"
#include "Camera.h"

#include "Box.cpp"
#include "Physics.h"

using namespace std;
PxRigidDynamic* sphereBody = nullptr;
PxRigidDynamic* shipBody = nullptr;
PxRigidDynamic* shipBody_buffor = nullptr;
PxRigidDynamic* pxSunBody = nullptr;
int textureArrayLength = 4;
GLuint pxProgramColor;
GLuint pxProgramTexture;


GLuint program;
GLuint programSun;
GLuint programTex;
GLuint texLoaded;
GLuint texLoadedsaturn;
GLuint texLoadedMars, texLoadedSaturn2;
GLuint texLoadedSkybox;
GLuint programProc;
GLuint programSkybox;
Core::Shader_Loader shaderLoader;

obj::Model shipModel;
obj::Model sphereModel;
Core::RenderContext shipContext;
Core::RenderContext sphereContext;


float cameraAngle = 0;
glm::vec3 cameraPos = glm::vec3(-30, 0, 0);
glm::vec3 cameraDir;

glm::mat4 cameraMatrix, perspectiveMatrix;

// contact pairs filtering function
static PxFilterFlags simulationFilterShader(PxFilterObjectAttributes attributes0,
	PxFilterData filterData0, PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	pairFlags =
		PxPairFlag::eCONTACT_DEFAULT | // default contact processing
		PxPairFlag::eNOTIFY_CONTACT_POINTS | // contact points will be available in onContact callback
		PxPairFlag::eNOTIFY_TOUCH_FOUND | // onContact callback will be called for this pair
		PxPairFlag::eNOTIFY_TOUCH_PERSISTS;

	return physx::PxFilterFlag::eDEFAULT;
}

class SimulationEventCallback : public PxSimulationEventCallback
{
public:
	void onContact(const PxContactPairHeader& pairHeader,
		const PxContactPair* pairs, PxU32 nbPairs)
	{
		// HINT: You can check which actors are in contact
		// using pairHeader.actors[0] and pairHeader.actors[1]

		cout << "nbPairs: ";
		cout << nbPairs;
		cout << "\n";
		for (PxU32 i = 0; i < nbPairs; i++)
		{
			const PxContactPair& cp = pairs[i];

			// HINT: two get the contact points, use
			// PxContactPair::extractContacts
			// You need to provide the function with a buffer
			// in which the contact points will be stored.
			// Create an array (vector) of type PxContactPairPoint
			// The number of elements in array should be at least
			// cp.contactCount (which is the number of contact points)
			// You also need to provide the function with the
			// size of the buffer (it should equal the size of the
			// array in bytes)
			// Finally, for every extracted point, you can access
			// its details, such as position
			std::vector<PxContactPairPoint> pairPoints(cp.contactCount);
			PxU32 nbContacts = cp.extractContacts(data(pairPoints), sizeof(pairPoints));
			cout << "nbContacts: ";
			cout << nbContacts;
			cout << "\n";
			nbContacts;
			for (int j = 0; j < nbContacts; j++) {
				cout << "Positions: ";
				cout << pairPoints[j].position.x;
				cout << " ";
				cout << pairPoints[j].position.y;
				cout << " ";
				cout << pairPoints[j].position.z;
				cout << "\n";
			}
		}
	}

	// The functions below are not used in this exercise.
	// However, they need to be defined for the class to compile.
	virtual void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) {}
	virtual void onWake(PxActor** actors, PxU32 count) {}
	virtual void onSleep(PxActor** actors, PxU32 count) {}
	virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) {}
	virtual void onAdvance(const PxRigidBody* const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count) {}
};

SimulationEventCallback simulationEventCallback;
Physics pxScene(0, simulationFilterShader,
	&simulationEventCallback);

// fixed timestep for stable and deterministic simulation
const double physicsStepTime = 1.f / 60.f;
double physicsTimeToProcess = 0;

struct Renderable {
	Core::RenderContext* context;
	glm::mat4 modelMatrix;
	GLuint textureId;
};
std::vector<Renderable*> renderables;

PxMaterial* pxMaterial = nullptr;
std::vector<PxRigidDynamic*> pxBodies;
GLuint pxTexture, pxTexture2;
obj::Model pxSphereModel;
obj::Model pxShipModel;
Core::RenderContext pxSphereContext;
Core::RenderContext pxShipContext;



void initRenderables()
{
	// load models
	pxSphereModel = obj::loadModelFromFile("models/sphere.obj");
	pxShipModel = obj::loadModelFromFile("models/spaceship.obj");

	// load textures
	pxTexture = Core::LoadTexture("textures/saturn.png");
	pxTexture2 = Core::LoadTexture("textures/2k_mars.png");
	pxSphereContext.initFromOBJ(pxSphereModel);
	pxShipContext.initFromOBJ(pxShipModel);


	Renderable* sphere = new Renderable();
	sphere->context = &pxSphereContext;
	sphere->textureId = pxTexture;
	renderables.emplace_back(sphere);

	Renderable* ship = new Renderable();
	ship->context = &pxShipContext;
	ship->textureId = pxTexture2;
	renderables.emplace_back(ship);

	Renderable* sun = new Renderable();
	sun->context = &pxSphereContext;
	sun->textureId = texLoaded;
	renderables.emplace_back(sun);


	const GLuint textures[50] = { texLoaded, texLoadedsaturn, texLoadedMars, texLoadedSaturn2 };
	for (int j = 0; j < textureArrayLength; j++) {
		// create box
		Renderable* box = new Renderable();
		box->context = &pxSphereContext;
		box->textureId = textures[j];
		renderables.emplace_back(box);
	}


}

void initPhysicsScene()
{
	//jedna przykladowa planeta
	sphereBody = pxScene.physics->createRigidDynamic(PxTransform(-20, 0, -1));
	pxMaterial = pxScene.physics->createMaterial(0.5, 0.5, 0.6);
	PxShape* sphereShape = pxScene.physics->createShape(PxSphereGeometry(1), *pxMaterial);
	sphereBody->attachShape(*sphereShape);
	//sphereBody->setLinearVelocity(PxVec3(cos(time*20), 0, sin(time * 20)));
	sphereShape->release();
	sphereBody->userData = renderables[0];
	sphereBody->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	pxScene.scene->addActor(*sphereBody);

	//statek
	shipBody_buffor = pxScene.physics->createRigidDynamic(PxTransform(cameraPos.x, cameraPos.y, cameraPos.z));
	PxShape* boxShape = pxScene.physics->createShape(PxSphereGeometry(1), *pxMaterial);
	shipBody_buffor->attachShape(*boxShape);
	boxShape->release();
	shipBody_buffor->userData = renderables[1];
	shipBody_buffor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	pxScene.scene->addActor(*shipBody_buffor);

	//slonce, ale nie do konco prawdziwe, bo w sloncu jest obiekt, ktory tylko detektuje kolizje
	pxSunBody = pxScene.physics->createRigidDynamic(PxTransform(0, 0, 0));
	PxShape* sunShape = pxScene.physics->createShape(PxSphereGeometry(5), *pxMaterial);
	pxSunBody->attachShape(*sunShape);
	sunShape->release();
	pxSunBody->userData = renderables[2];
	//boxBody_buffor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	pxScene.scene->addActor(*pxSunBody);


	for (int j = 0; j < textureArrayLength; j++) {
		PxRigidDynamic* boxBody_buffor2 = pxScene.physics->createRigidDynamic(PxTransform(-10 * j + 100, 10 * j, -1));
		PxShape* boxShape = pxScene.physics->createShape(PxSphereGeometry(1), *pxMaterial);
		boxBody_buffor2->attachShape(*boxShape);
		boxShape->release();
		boxBody_buffor2->userData = renderables[j + 3];
		boxBody_buffor2->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		pxScene.scene->addActor(*boxBody_buffor2);
		pxBodies.push_back(boxBody_buffor2);
	}


	//shipBody = pxScene.physics->createRigidDynamic(PxTransform(-25, -2, 0));
	//pxMaterial = pxScene.physics->createMaterial(0.5, 0.5, 0.6);
	//PxShape* shipShape = pxScene.physics->createShape(PxSphereGeometry(1), *pxMaterial);
	//shipBody->attachShape(*shipShape);
	////sphereBody->setLinearVelocity(PxVec3(0, 0, -30));
	//shipShape->release();
	//shipShape->userData = renderables[1];
	////shipBody->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	//pxScene.scene->addActor(*shipBody);
}

void updateTransforms()
{
	// Here we retrieve the current transforms of the objects from the physical simulation.
	auto actorFlags = PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC;
	PxU32 nbActors = pxScene.scene->getNbActors(actorFlags);
	if (nbActors)
	{
		float time = glutGet(GLUT_ELAPSED_TIME) / 1000.f;

		std::vector<PxRigidActor*> actors(nbActors);
		pxScene.scene->getActors(actorFlags, (PxActor**)&actors[0], nbActors);
		for (auto actor : actors)
		{
			// We use the userData of the objects to set up the model matrices
			// of proper renderables.
			if (!actor->userData) continue;
			Renderable* renderable = (Renderable*)actor->userData;

			// get world matrix of the object (actor)
			PxMat44 transform = actor->getGlobalPose();
			auto& c0 = transform.column0;
			auto& c1 = transform.column1;
			auto& c2 = transform.column2;
			auto& c3 = transform.column3;

			// set up the model matrix used for the rendering
			renderable->modelMatrix = glm::mat4(
				c0.x, c0.y, c0.z, c0.w,
				c1.x, c1.y, c1.z, c1.w,
				c2.x, c2.y, c2.z, c2.w,
				c3.x, c3.y, c3.z, c3.w)
				* glm::rotate(time / 2, glm::vec3(0, 1, 0));
		}



		shipBody_buffor->setKinematicTarget(PxTransform(cameraPos.x, cameraPos.y, cameraPos.z));
		sphereBody->setKinematicTarget(PxTransform(-7 * sin(time), -7 * cos(time), -7 * cos(time)));

		for (int i = 0; i < textureArrayLength; i++) {
			if (i % 2 == 0) {
				pxBodies[i]->setKinematicTarget(PxTransform((i * 4 + 10) * cos((time + 2 * i) / 10), 0, (i * 4 + 10) * sin((time + 2 * i) / 10)));
			}
			else {
				pxBodies[i]->setKinematicTarget(PxTransform((i * 4 + 10) * -cos((time + 2 * i) / 10), 0, (i * 4 + 10) * -sin((time + 2 * i) / 10)));
			}
		};
	}
}


void keyboard(unsigned char key, int x, int y)
{
	float angleSpeed = 0.1f;
	float moveSpeed = 0.1f;
	switch (key)
	{
	case 'z': cameraAngle -= angleSpeed; break;
	case 'x': cameraAngle += angleSpeed; break;
	case 'w': cameraPos += cameraDir * moveSpeed; break;
	case 's': cameraPos -= cameraDir * moveSpeed; break;
	case 'd': cameraPos += glm::cross(cameraDir, glm::vec3(0, 1, 0)) * moveSpeed; break;
	case 'a': cameraPos -= glm::cross(cameraDir, glm::vec3(0, 1, 0)) * moveSpeed; break;
	case 'e': cameraPos += glm::cross(cameraDir, glm::vec3(1, 0, 0)) * moveSpeed; break;
	case 'q': cameraPos -= glm::cross(cameraDir, glm::vec3(1, 0, 0)) * moveSpeed; break;
	}
}

glm::mat4 createCameraMatrix()
{
	// Obliczanie kierunku patrzenia kamery (w plaszczyznie x-z) przy uzyciu zmiennej cameraAngle kontrolowanej przez klawisze.
	cameraDir = glm::vec3(cosf(cameraAngle), 0.0f, sinf(cameraAngle));
	glm::vec3 up = glm::vec3(0, 1, 0);

	return Core::createViewMatrix(cameraPos, cameraDir, up);
}

void drawObject(GLuint program, Core::RenderContext context, glm::mat4 modelMatrix, glm::vec3 color)
{
	glUniform3f(glGetUniformLocation(program, "objectColor"), color.x, color.y, color.z);

	glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
	glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);
	glUniformMatrix4fv(glGetUniformLocation(program, "transformation"), 1, GL_FALSE, (float*)&transformation);

	Core::DrawContext(context);
}

void drawObjectTexture(GLuint program, Core::RenderContext context, glm::mat4 modelMatrix, GLuint id, int textureUnit)
{
	Core::SetActiveTexture(id, "colorTexture", program, textureUnit);
	//glUniform3f(glGetUniformLocation(program, "colorTexture"), Core::SetActiveTexture(),);

	glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
	glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);
	glUniformMatrix4fv(glGetUniformLocation(program, "transformation"), 1, GL_FALSE, (float*)&transformation);

	Core::DrawContext(context);
}

void drawPxObjectTexture(GLuint program, Core::RenderContext* context, glm::mat4 modelMatrix, GLuint id, int textureUnit)
{
	Core::SetActiveTexture(id, "colorTexture", program, textureUnit);
	//glUniform3f(glGetUniformLocation(program, "colorTexture"), Core::SetActiveTexture(),);

	glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
	glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);
	glUniformMatrix4fv(glGetUniformLocation(program, "transformation"), 1, GL_FALSE, (float*)&transformation);

	Core::DrawContext(*context);
}

void renderScene()
{
	// Aktualizacja macierzy widoku i rzutowania. Macierze sa przechowywane w zmiennych globalnych, bo uzywa ich funkcja drawObject.
	// (Bardziej elegancko byloby przekazac je jako argumenty do funkcji, ale robimy tak dla uproszczenia kodu.
	//  Jest to mozliwe dzieki temu, ze macierze widoku i rzutowania sa takie same dla wszystkich obiektow!)
	float time = glutGet(GLUT_ELAPSED_TIME) / 1000.f;
	static double prevTime = time;
	double dtime = time - prevTime;
	prevTime = time;

	// Update physics
	if (dtime < 1.f) {
		physicsTimeToProcess += dtime;
		while (physicsTimeToProcess > 0) {
			// here we perform the physics simulation step
			pxScene.step(physicsStepTime);
			physicsTimeToProcess -= physicsStepTime;
		}
	}

	cameraMatrix = createCameraMatrix();
	perspectiveMatrix = Core::createPerspectiveMatrix();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.3f, 0.3f, 1.0f);

	// Macierz statku "przyczepia" go do kamery. Warto przeanalizowac te linijke i zrozumiec jak to dziala.
	glm::mat4 shipModelMatrix = glm::translate(cameraPos + cameraDir * 0.5f + glm::vec3(0, -0.25f, 0)) * glm::rotate(-cameraAngle + glm::radians(90.0f), glm::vec3(0, 1, 0)) * glm::scale(glm::vec3(0.25f));
	glm::vec3 lightPos = glm::vec3(0, 0, 0);


	//SKYBOX
	glUseProgram(programSkybox);
	glDepthMask(GL_FALSE);
	drawObjectTexture(programSkybox, sphereContext, glm::translate(cameraPos + cameraDir * 0.5f) * glm::scale(glm::vec3(70.f)), texLoadedSkybox, 5);
	glDepthMask(GL_TRUE);

	//fizyczne obiekty 
	updateTransforms();
	int i = 0;
	//renderables[1] to statek
	renderables[1]->modelMatrix = shipModelMatrix;
	for (Renderable* renderable : renderables) {
		drawPxObjectTexture(programTex, renderable->context, renderable->modelMatrix, renderable->textureId, 13 + i);
		i += 1;
	}

	//rysowanie slonca
	glUseProgram(programSun);
	glUniform3f(glGetUniformLocation(programSun, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(programSun, "cameraPos"), cameraPos.x, cameraPos.y, cameraPos.z);
	drawObject(programSun, sphereContext, glm::translate(lightPos) * glm::scale(glm::vec3(5.0f)), glm::vec3(1.0f, 0.8f, 0.2f));

	glUseProgram(0);
	glutSwapBuffers();
}

void init()
{
	srand(time(0));
	glEnable(GL_DEPTH_TEST);
	program = shaderLoader.CreateProgram("shaders/shader_4_1.vert", "shaders/shader_4_1.frag");
	programSun = shaderLoader.CreateProgram("shaders/shader_4_2.vert", "shaders/shader_4_2.frag");
	programTex = shaderLoader.CreateProgram("shaders/shader_4_tex.vert", "shaders/shader_4_tex.frag");
	programProc = shaderLoader.CreateProgram("shaders/shader_proc_tex.vert", "shaders/shader_proc_tex.frag");

	pxProgramColor = shaderLoader.CreateProgram("shaders/shader_color.vert", "shaders/shader_color.frag");

	programSkybox = shaderLoader.CreateProgram("shaders/shader_skybox.vert", "shaders/shader_skybox.frag");
	texLoaded = Core::LoadTexture("textures/earth.png");
	texLoadedsaturn = Core::LoadTexture("textures/mercury.png");
	texLoadedSkybox = Core::LoadTexture("textures/galaxy.png");
	texLoadedMars = Core::LoadTexture("textures/2k_mars.png");
	texLoadedSaturn2 = Core::LoadTexture("textures/saturn.png");
	sphereModel = obj::loadModelFromFile("models/sphere.obj");
	shipModel = obj::loadModelFromFile("models/spaceship.obj");
	shipContext.initFromOBJ(shipModel);
	sphereContext.initFromOBJ(sphereModel);

	initRenderables();
	initPhysicsScene();
}

void shutdown()
{
	shaderLoader.DeleteProgram(program);
	pxSphereContext.initFromOBJ(pxSphereModel);
	pxShipContext.initFromOBJ(pxShipModel);
}

void idle()
{
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(200, 200);
	glutInitWindowSize(600, 600);
	glutCreateWindow("OpenGL Pierwszy Program");
	glewInit();

	init();
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(renderScene);
	glutIdleFunc(idle);

	glutMainLoop();

	shutdown();

	return 0;
}
