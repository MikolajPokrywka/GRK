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

GLuint program;
GLuint programSun;
GLuint programTex;
GLuint texLoaded;
GLuint texLoadedsaturn;
GLuint texLoadedMars;
GLuint programProc;
Core::Shader_Loader shaderLoader;

obj::Model shipModel;
obj::Model sphereModel;
Core::RenderContext shipContext;
Core::RenderContext sphereContext;


float cameraAngle = 0;
glm::vec3 cameraPos = glm::vec3(-30, 0, 0);
glm::vec3 cameraDir;

glm::mat4 cameraMatrix, perspectiveMatrix;

void keyboard(unsigned char key, int x, int y)
{
	float angleSpeed = 0.1f;
	float moveSpeed = 0.1f;
	switch(key)
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
	glm::vec3 up = glm::vec3(0,1,0);

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

void renderScene()
{
	// Aktualizacja macierzy widoku i rzutowania. Macierze sa przechowywane w zmiennych globalnych, bo uzywa ich funkcja drawObject.
	// (Bardziej elegancko byloby przekazac je jako argumenty do funkcji, ale robimy tak dla uproszczenia kodu.
	//  Jest to mozliwe dzieki temu, ze macierze widoku i rzutowania sa takie same dla wszystkich obiektow!)
	cameraMatrix = createCameraMatrix();
	perspectiveMatrix = Core::createPerspectiveMatrix();
	float time = glutGet(GLUT_ELAPSED_TIME)/1000.f;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.3f, 0.3f, 1.0f);


	// Macierz statku "przyczepia" go do kamery. Warto przeanalizowac te linijke i zrozumiec jak to dziala.
	glm::mat4 shipModelMatrix = glm::translate(cameraPos + cameraDir * 0.5f + glm::vec3(0,-0.25f,0)) * glm::rotate(-cameraAngle + glm::radians(90.0f), glm::vec3(0,1,0)) * glm::scale(glm::vec3(0.25f));
	glm::vec3 lightPos = glm::vec3(0, 0, 0);
	//glUniform3f(glGetUniformLocation(program, "light_dir"), 1, 1, 0);
	glUseProgram(programProc);
	glUniform3f(glGetUniformLocation(programTex, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(programTex, "cameraPos"), cameraPos.x, cameraPos.y, cameraPos.z);
	drawObject(programProc, shipContext, shipModelMatrix, glm::vec3(1, 1, 1));
	//drawObject(program, sphereContext, glm::eulerAngleY(time / 2) * glm::translate(glm::vec3(-5, 0, 0))*glm::scale(glm::vec3(0.7f)), glm::vec3(1.0f, 0.0f, 1.0f));
	/*glUseProgram(program);
	glUniform3f(glGetUniformLocation(program, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(program, "cameraPos"), cameraPos.x, cameraPos.y, cameraPos.z);*/
	

	//RYSOWANIE PLANET
	glUseProgram(programTex);
	glUniform3f(glGetUniformLocation(programTex, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(programTex, "cameraPos"), cameraPos.x, cameraPos.y, cameraPos.z);
	drawObjectTexture(programTex, sphereContext, glm::eulerAngleY(time / 8) * glm::translate(glm::vec3(-6, 0, -6)) *glm::scale(glm::vec3(0.7f)), texLoadedsaturn, 1);
	drawObjectTexture(programTex, sphereContext, glm::eulerAngleY(time / 6) * glm::translate(glm::vec3(-8, 0, -8)) *glm::scale(glm::vec3(0.4f)), texLoadedMars, 2);
	
	//ksiezyc i ziemia
	drawObjectTexture(programTex, sphereContext, glm::eulerAngleY(time / 10) * glm::translate(glm::vec3(-20, 0, 0)) *glm::scale(glm::vec3(1.0f)), texLoaded, 3);
	drawObjectTexture(programTex, sphereContext, glm::eulerAngleY(time / 10) * glm::translate(glm::vec3(-20, 0, 0))* glm::eulerAngleXZ(0.4f, 0.6f) * glm::eulerAngleY(time/3) * glm::translate(glm::vec3(-2, 0, 0)) *glm::scale(glm::vec3(0.2f)), texLoadedsaturn, 4);

	glUseProgram(programSun);
	glUniform3f(glGetUniformLocation(programSun, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(programSun, "cameraPos"), cameraPos.x, cameraPos.y, cameraPos.z);
	drawObject(programSun, sphereContext, glm::translate(lightPos)  *glm::scale(glm::vec3(5.0f)), glm::vec3(1.0f, 0.8f, 0.2f));
	glUniform3f(glGetUniformLocation(programTex, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(programTex, "cameraPos"), cameraPos.x, cameraPos.y, cameraPos.z);
	//glUniform3f(glGetUniformLocation(programTex, "objectColor"), 1.0, 0.0, 1.0);


	glUseProgram(0);
	glutSwapBuffers();
}

void init()
{
	glEnable(GL_DEPTH_TEST);
	program = shaderLoader.CreateProgram("shaders/shader_4_1.vert", "shaders/shader_4_1.frag");
	programSun = shaderLoader.CreateProgram("shaders/shader_4_2.vert", "shaders/shader_4_2.frag");
	programTex = shaderLoader.CreateProgram("shaders/shader_4_tex.vert", "shaders/shader_4_tex.frag");
	programProc = shaderLoader.CreateProgram("shaders/shader_proc_tex.vert", "shaders/shader_proc_tex.frag");
	texLoaded = Core::LoadTexture("textures/earth.png");
	texLoadedsaturn = Core::LoadTexture("textures/mercury.png");
	texLoadedMars = Core::LoadTexture("textures/saturn.png");
	sphereModel = obj::loadModelFromFile("models/sphere.obj");
	shipModel = obj::loadModelFromFile("models/spaceship.obj");
	shipContext.initFromOBJ(shipModel);
	sphereContext.initFromOBJ(sphereModel);
}

void shutdown()
{
	shaderLoader.DeleteProgram(program);
}

void idle()
{
	glutPostRedisplay();
}

int main(int argc, char ** argv)
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
