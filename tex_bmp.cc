#include <cstdio>
#include <cassert>
#include <string>
#include <iostream>
#include <bits/stdc++.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include "tex_bmp.h"

GLuint LoadBitmap(const char * imgpath){
	unsigned char header[54];
	unsigned int dataPos;
	unsigned int width,height;
	unsigned int imageSize;
	
	unsigned char *data;
	
	FILE *file = fopen(imgpath,"rb");
	if(!file){
		printf("File %s is missing.\n",imgpath);
		return 0;
		assert(file);//Can open file?
	}
	if( fread(header , 1, 54,file) != 54 || header[0]!='B' || header[1]!='M'){
		printf("File %s is Not correct BMP file\n",imgpath);
		return 0;
		assert( fread(header , 1, 54,file) == 54 || header[0]!='B' || header[1]!='M' );//Correct BMP File
	}
	
	dataPos = *(int*) &(header[0x0A]);
	imageSize = *(int*) &(header[0x22]);
	width = *(int*) &(header[0x12]);
	height = *(int*) &(header[0x16]);
	
	if(imageSize==0)
		imageSize=width*height*3;
	if(dataPos==0)
		dataPos=54;
	data = new unsigned char [imageSize];
	fread(data,1,imageSize,file);
	fclose(file);
	
	//create one opengl texture
	GLuint textureID;
	//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(	1 , &textureID);
	//bind the newly createdd texture : all future texture function will modify this texture
	glBindTexture(GL_TEXTURE_2D  , textureID);
	
	//Give the image to opengl
	
	glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER , GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER , GL_NEAREST);

glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
// When MINifying the image, use a LINEAR blend of two mipmaps, each filtered LINEARLY too
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
// Generate mipmaps, by the way.
//glGenerateMipmap(GL_TEXTURE_2D);

glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);

	glTexImage2D( GL_TEXTURE_2D ,0 , GL_RGB , width , height , 0 , GL_BGR , GL_UNSIGNED_BYTE , data);
/*
glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

// When MAGnifying the image (no bigger mipmap available), use LINEAR filtering

	 */
	return textureID;
}


GLuint LoadBitmapy(const char* FilePath)
{
	
	
int width = 0;
int height = 0;
short BitsPerPixel = 0;
std::vector<unsigned char> Pixels;

	
    std::fstream hFile(FilePath, std::ios::in | std::ios::binary);
    if (!hFile.is_open()) throw std::invalid_argument("Error: File Not Found.");

    hFile.seekg(0, std::ios::end);
    int Length = hFile.tellg();
    hFile.seekg(0, std::ios::beg);
    std::vector<uint8_t> FileInfo(Length);
    hFile.read(reinterpret_cast<char*>(FileInfo.data()), 54);

    if(FileInfo[0] != 'B' && FileInfo[1] != 'M')
    {
        hFile.close();
        throw std::invalid_argument("Error: Invalid File Format. Bitmap Required.");
    }

    if (FileInfo[28] != 24 && FileInfo[28] != 32)
    {
        hFile.close();
        throw std::invalid_argument("Error: Invalid File Format. 24 or 32 bit Image Required.");
    }
	GLuint texture;
    BitsPerPixel = FileInfo[28];
    width = FileInfo[18] + (FileInfo[19] << 8);
    height = FileInfo[22] + (FileInfo[23] << 8);
    uint32_t PixelsOffset = FileInfo[10] + (FileInfo[11] << 8);
    uint32_t size = ((width * BitsPerPixel + 31) / 32) * 4 * height;
    Pixels.resize(size);

    hFile.seekg (PixelsOffset, std::ios::beg);
    hFile.read(reinterpret_cast<char*>(Pixels.data()), size);
    hFile.close();
    
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,0, GL_RGB, GL_UNSIGNED_BYTE, Pixels.data());

	glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER , GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER , GL_NEAREST);
	
    return texture;
    
}

GLuint loadShaderFromFile( std::string path, GLenum shaderType ) { 
	//Open file 
	GLuint shaderID = 0; 
	std::string shaderString; 
	std::ifstream sourceFile( path.c_str() ); 
	//Source file loaded 
	if( sourceFile ) { 
		//Get shader source 
		shaderString.assign( ( std::istreambuf_iterator< char >( sourceFile ) ), std::istreambuf_iterator< char >() );
		//Create shader ID 
		shaderID = glCreateShader( shaderType ); 
		//Set shader source 
		const GLchar* shaderSource = shaderString.c_str(); 
		glShaderSource( shaderID, 1, (const GLchar**)&shaderSource, NULL ); 
		//Compile shader source 
		glCompileShader( shaderID ); 
		//Check shader for errors 
		GLint shaderCompiled = GL_FALSE; 
		glGetShaderiv( shaderID, GL_COMPILE_STATUS, &shaderCompiled ); 
		if( shaderCompiled != GL_TRUE ) { 
			printf( "Unable to compile shader %d!\n\nSource:\n%s\n", shaderID, shaderSource ); 
			//printShaderLog( shaderID ); 
			glDeleteShader( shaderID ); 
			shaderID = 0; 
			} 
		} 
		else { 
			printf( "Unable to open file %s\n", path.c_str() );
		}
		 return shaderID; 
}
