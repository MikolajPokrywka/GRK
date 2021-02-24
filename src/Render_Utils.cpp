#include "Render_Utils.h"

#include <algorithm>

#include "glew.h"
#include "freeglut.h"


void Core::RenderContext::initFromOBJ(obj::Model& model)
{
    vertexArray = 0;
    vertexBuffer = 0;
    vertexIndexBuffer = 0;
    unsigned int vertexDataBufferSize = sizeof(float) * model.vertex.size();
    unsigned int vertexNormalBufferSize = sizeof(float) * model.normal.size();
    unsigned int vertexTexBufferSize = sizeof(float) * model.texCoord.size();

    unsigned int vertexTangentBufferSize = sizeof(float) * model.tangent.size();
    unsigned int vertexBitangentBufferSize = sizeof(float) * model.bitangent.size();

    size = model.faces["default"].size();
    unsigned int vertexElementBufferSize = sizeof(unsigned short) * size;


    glGenVertexArrays(1, &vertexArray);
    glBindVertexArray(vertexArray);


    glGenBuffers(1, &vertexIndexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertexIndexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, vertexElementBufferSize, &model.faces["default"][0], GL_STATIC_DRAW);

    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    //czy tutaj tez powiniem dodac
    glEnableVertexAttribArray(3);
    glEnableVertexAttribArray(4);
    
    glBufferData(GL_ARRAY_BUFFER, vertexDataBufferSize + vertexNormalBufferSize + vertexTexBufferSize + vertexTangentBufferSize + vertexBitangentBufferSize, NULL, GL_STATIC_DRAW);

    glBufferSubData(GL_ARRAY_BUFFER, 0, vertexDataBufferSize, &model.vertex[0]);

    glBufferSubData(GL_ARRAY_BUFFER, vertexDataBufferSize, vertexNormalBufferSize, &model.normal[0]);

    glBufferSubData(GL_ARRAY_BUFFER, vertexDataBufferSize + vertexNormalBufferSize, vertexTexBufferSize, &model.texCoord[0]);

    //analogicznie przekazac dane w subData
    glBufferSubData(GL_ARRAY_BUFFER, vertexDataBufferSize + vertexNormalBufferSize + vertexTexBufferSize, vertexTangentBufferSize, &model.tangent[0]);

    glBufferSubData(GL_ARRAY_BUFFER, vertexDataBufferSize + vertexNormalBufferSize + vertexTexBufferSize + vertexTangentBufferSize, vertexBitangentBufferSize, &model.bitangent[0]);

    //layout(location = 0) in vec3 vertexPosition;
  /*  layout(location = 1) in vec2 vertexTexCoord;
    layout(location = 2) in vec3 vertexNormal;
    layout(location = 3) in vec3 vertexTangent;
    layout(location = 4) in vec3 vertexBitangent;*/
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)(0));
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)(vertexDataBufferSize));
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)(vertexNormalBufferSize + vertexDataBufferSize));

    //analogicznie podpiac tangenty i bitangenty
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (void*)(vertexDataBufferSize + vertexNormalBufferSize + vertexTexBufferSize));
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, (void*)(vertexDataBufferSize + vertexNormalBufferSize + vertexTexBufferSize + vertexTangentBufferSize));
}

void Core::DrawVertexArray(const float * vertexArray, int numVertices, int elementSize )
{
	glVertexAttribPointer(0, elementSize, GL_FLOAT, false, 0, vertexArray);
	glEnableVertexAttribArray(0);

	glDrawArrays(GL_TRIANGLES, 0, numVertices);
}

void Core::DrawVertexArrayIndexed( const float * vertexArray, const int * indexArray, int numIndexes, int elementSize )
{
	glVertexAttribPointer(0, elementSize, GL_FLOAT, false, 0, vertexArray);
	glEnableVertexAttribArray(0);

	glDrawElements(GL_TRIANGLES, numIndexes, GL_UNSIGNED_INT, indexArray);
}


//void Core::DrawVertexArray( const VertexData & data )
//{
//	int numAttribs = std::min(VertexData::MAX_ATTRIBS, data.NumActiveAttribs);
//	for(int i = 0; i < numAttribs; i++)
//	{
//		glVertexAttribPointer(i, data.Attribs[i].Size, GL_FLOAT, false, 0, data.Attribs[i].Pointer);
//		glEnableVertexAttribArray(i);
//	}
//	glDrawArrays(GL_TRIANGLES, 0, data.NumVertices);
//}

void Core::DrawContext(Core::RenderContext& context)
{

	glBindVertexArray(context.vertexArray);
	glDrawElements(
		GL_TRIANGLES,      // mode
		context.size,    // count
		GL_UNSIGNED_SHORT,   // type
		(void*)0           // element array buffer offset
	);
	glBindVertexArray(0);
}


void Core::ParticleContext::initParticle(GLuint programID, const int MaxPart, GLfloat* position_size_data, GLubyte* color_data)
{
    VertexArrayID = 0;
    billboard_vertex_buffer = 0;
    particles_position_buffer = 0;
    particles_color_buffer = 0;
	MaxParticles = MaxPart;
	g_particule_position_size_data = position_size_data;
	g_particule_color_data = color_data;

	CameraRight_worldspace_ID = glGetUniformLocation(programID, "CameraRight_worldspace");
	CameraUp_worldspace_ID = glGetUniformLocation(programID, "CameraUp_worldspace");
	ViewProjMatrixID = glGetUniformLocation(programID, "VP");

    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    GLfloat g_vertex_buffer_data[] = {
         -0.05f, -0.05f, 0.0f,
          0.05f, -0.05f, 0.0f,
         -0.05f,  0.05f, 0.0f,
          0.05f,  0.05f, 0.0f,
    };

    glGenBuffers(1, &billboard_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    // The VBO containing the positions and sizes of the particles

    glGenBuffers(1, &particles_position_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
    // Initialize with empty (NULL) buffer : it will be updated later, each frame.
    glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW);

    // The VBO containing the colors of the particles

    glGenBuffers(1, &particles_color_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
    // Initialize with empty (NULL) buffer : it will be updated later, each frame.
    glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW);

}

void Core::DrawParticles(Core::ParticleContext& context, GLuint programID, GLuint TextureID, GLuint Texture, int ParticlesCount, glm::mat4 cameraMatrix, glm::mat4 perspectiveMatrix)
{

	glBindVertexArray(context.VertexArrayID);





	glBindBuffer(GL_ARRAY_BUFFER, context.particles_position_buffer);
	glBufferData(GL_ARRAY_BUFFER, context.MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
	glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLfloat) * 4, context.g_particule_position_size_data);

	glBindBuffer(GL_ARRAY_BUFFER, context.particles_color_buffer);
	glBufferData(GL_ARRAY_BUFFER, context.MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
	glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLubyte) * 4, context.g_particule_color_data);


	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Use our shader
	glUseProgram(programID);

	// Bind our texture in Texture Unit 0s
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture);
	// Set our "myTextureSampler" sampler to use Texture Unit 0
	glUniform1i(TextureID, 0);

	// Same as the billboards tutorial
	glUniform3f(context.CameraRight_worldspace_ID, cameraMatrix[0][0], cameraMatrix[1][0], cameraMatrix[2][0]);
	glUniform3f(context.CameraUp_worldspace_ID, cameraMatrix[0][1], cameraMatrix[1][1], cameraMatrix[2][1]);

	glm::mat4 ViewProjectionMatrix = perspectiveMatrix * cameraMatrix;
	glUniformMatrix4fv(context.ViewProjMatrixID, 1, GL_FALSE, &ViewProjectionMatrix[0][0]);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, context.billboard_vertex_buffer);
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	// 2nd attribute buffer : positions of particles' centers
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, context.particles_position_buffer);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		4,                                // size : x + y + z + size => 4
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// 3rd attribute buffer : particles' colors
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, context.particles_color_buffer);
	glVertexAttribPointer(
		2,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		4,                                // size : r + g + b + a => 4
		GL_UNSIGNED_BYTE,                 // type
		GL_TRUE,                          // normalized?    *** YES, this means that the unsigned char[4] will be accessible with a vec4 (floats) in the shader ***
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// These functions are specific to glDrawArrays*Instanced*.
	// The first parameter is the attribute buffer we're talking about.
	// The second parameter is the "rate at which generic vertex attributes advance when rendering multiple instances"
	// http://www.opengl.org/sdk/docs/man/xhtml/glVertexAttribDivisor.xml
	glVertexAttribDivisor(0, 0); // particles vertices : always reuse the same 4 vertices -> 0
	glVertexAttribDivisor(1, 1); // positions : one per quad (its center)                 -> 1
	glVertexAttribDivisor(2, 1); // color : one per quad                                  -> 1

	// Draw the particules !
	// This draws many times a small triangle_strip (which looks like a quad).
	// This is equivalent to :
	// for(i in ParticlesCount) : glDrawArrays(GL_TRIANGLE_STRIP, 0, 4), 
	// but faster.
	glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, ParticlesCount);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);



	glBindVertexArray(0);

}