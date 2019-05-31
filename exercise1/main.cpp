#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include "Eigen.h"

#include "utils/VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool is_Vertex_Valid(Vertex& v){
    return v.position != Vector4f(MINF,MINF,MINF,MINF);
}

double edge_length(Vertex& x, Vertex& y){
    return (x.position - y.position).norm();
}

bool is_Face_Valid(Vertex& x, Vertex& y, Vertex& z){
    float edge_threshold = 0.01f; // 1cm

    if(is_Vertex_Valid(x) && is_Vertex_Valid(y) && is_Vertex_Valid(z)){
        if(edge_length(x,y) < edge_threshold && edge_length(y,z) < edge_threshold && edge_length(x,z) < edge_threshold)
        {
            return true;
        }
    }

    return false;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width*height;


	// TODO: Get number of faces
	unsigned int nFaces = 0;

	std::vector<std::array<unsigned int, 3>> faces;

	for(size_t r=0; r < width-1; r++){
	    for(size_t c = 0; c < height-1; c++){
	        size_t index_v1 = r + c * width;
	        size_t index_v2 = (r) + ((c+1)*width);
	        size_t index_v3 = r+1 + (c*width);
	        size_t index_v4 = (r+1) + ((c+1)*width);

            if(is_Face_Valid(vertices[index_v1],vertices[index_v2],vertices[index_v3])){

                faces.push_back(std::array<unsigned int, 3>({index_v1,index_v2,index_v3}));
                nFaces++;
            }

            if(is_Face_Valid(vertices[index_v2],vertices[index_v3],vertices[index_v4])){

                faces.push_back(std::array<unsigned int, 3>({index_v2,index_v3,index_v4}));
                nFaces++;
            }

	    }
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(unsigned int i = 0 ; i< width*height ; i++){
	    if(vertices[i].position[0] == MINF){
            outFile << 0 << " "<< 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
	    }
	    else {
            outFile << vertices[i].position[0] << " " << vertices[i].position[1] << " " << vertices[i].position[2]
                    << " " << (unsigned int)vertices[i].color[0] << " " << (unsigned int)vertices[i].color[1] << " " << (unsigned int)vertices[i].color[2] << " "
                    << (unsigned int)vertices[i].color[3] << std::endl;
        }
	}

	// TODO: save faces
	for(size_t i = 0 ; i < faces.size() ; i++){
	    outFile << 3 << " " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << "\n";
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		unsigned int image_height = sensor.GetDepthImageHeight();
		unsigned int image_width = sensor.GetDepthImageWidth();

        for(size_t i = 0 ; i < image_height*image_width; i++){
            auto Z = depthMap[i];
            auto color = colorMap + i*4;

            if(Z == MINF){
                vertices[i].position = Vector4f(MINF,MINF,MINF,MINF);
                vertices[i].color = Vector4uc(0,0,0,0);
            }
            else{
                auto v = int(i / image_width);
                auto u = int(i % image_width);

                auto X = (u-cX) / fovX * Z;
                auto Y = (v-cY) / fovY * Z;

                Vector4f point_3d(X,Y,Z,1.0);

                point_3d = depthExtrinsicsInv * point_3d;
                point_3d = trajectoryInv * point_3d;

                vertices[i].position = point_3d;
                vertices[i].color = Vector4uc(color[0],color[1],color[2],color[3]);
            }

        }

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
