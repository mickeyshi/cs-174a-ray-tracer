//
// template-rt.cpp
//

#define _CRT_SECURE_NO_WARNINGS
#include "matm.h"
#include <stdlib.h>     //for using the function sleep
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

int g_width;
int g_height;
const int THRESHOLD_LENGTH = .005;
const int MAX_RECURSION = 3;
struct Ray
{
    vec4 origin;
    vec4 dir;
};



// TODO: add structs for spheres, lights and anything else you may need.
struct Sphere
{
	vec4 center;
	vec4 scale;
	vec4 color;
	float Ka;
	float Kd;
	float Ks;
	float Kr;
	float n;
	mat4 inverseMat;

};

struct Light {
	vec4 center;
	vec4 color;
	vec4 brightness;
};

vector<vec4> g_colors;
vector<Sphere> g_spheres;
vector<Light> g_lights;
vec4 g_bg;
vec4 g_ambient;
float g_left;
float g_right;
float g_top;
float g_bottom;
float g_near;
string g_output;


//toVec3 function
inline
vec3 toVec3(vec4 in) {
	return vec3(in[0], in[1], in[2]);
}

// -------------------------------------------------------------------
// Input file parsing

vec4 toVec4(const string& s1, const string& s2, const string& s3)
{
    stringstream ss(s1 + " " + s2 + " " + s3);
    vec4 result;
    ss >> result.x >> result.y >> result.z;
    result.w = 1.0f;
    return result;
}

float toFloat(const string& s)
{
    stringstream ss(s);
    float f;
    ss >> f;
    return f;
}

void parseLine(const vector<string>& vs)
{
    
	const int num_labels = 11; //0    1      2        3       4     5      6        7       8        9          10
	const string labels[] = { "NEAR","LEFT","RIGHT","BOTTOM","TOP","RES","SPHERE","LIGHT","BACK","AMBIENT", "OUTPUT" };
	unsigned label_id = find(labels, labels + num_labels, vs[0]) - labels;
	switch (label_id)
	{
	case 0:	g_near = toFloat(vs[1]);	break;
	case 1: g_left = toFloat(vs[1]);	break;
	case 2: g_right = toFloat(vs[1]);	break;
	case 3: g_bottom = toFloat(vs[1]);	break;
	case 4: g_top = toFloat(vs[1]);		break;
	case 5:
		g_width = (int)toFloat(vs[1]);
		g_height = (int)toFloat(vs[2]);
		g_colors.resize(g_width * g_height);
		break;
	case 6:
	{
		Sphere s;
		s.center = vec4(toFloat(vs[2]), toFloat(vs[3]), toFloat(vs[4]), 1);
		s.scale = vec4(toFloat(vs[5]), toFloat(vs[6]), toFloat(vs[7]), 0);
		s.color = vec4(toFloat(vs[8]), toFloat(vs[9]), toFloat(vs[10]), 0);
		s.Ka = toFloat(vs[11]);
		s.Kd = toFloat(vs[12]);
		s.Ks = toFloat(vs[13]);
		s.Kr = toFloat(vs[14]);
		s.n = toFloat(vs[15]);
		mat4 inv = Translate(toVec3(s.center)) * Scale(toVec3(s.scale)) ;
		InvertMatrix(inv, inv);
		s.inverseMat = inv;
		g_spheres.push_back(s);
		break;
	}
	case 7:
	{
		Light l;
		l.center = vec4(toFloat(vs[2]), toFloat(vs[3]), toFloat(vs[4]), 1);
		l.color = vec4(toFloat(vs[5]), toFloat(vs[6]), toFloat(vs[7]), 0);
		g_lights.push_back(l);
		break;
	}
	case 8:
	{
		vec4 bg;
		bg = vec4(toFloat(vs[1]), toFloat(vs[2]), toFloat(vs[3]), 0);
		g_bg = bg;
		break;
	}
	case 9:
	{
		vec4 amb;
		amb = vec4(toFloat(vs[1]), toFloat(vs[2]), toFloat(vs[3]), 0);
		g_ambient = amb;
		break;
	}
	case 10:
		g_output = vs[1]; 
	
	}
}

void loadFile(const char* filename)
{
    ifstream is(filename);
    if (is.fail())
    {
        cout << "Could not open file " << filename << endl;
        exit(1);
    }
    string s;
    vector<string> vs;
    while(!is.eof())
    {
        vs.clear();
        getline(is, s);
        istringstream iss(s);
        while (!iss.eof())
        {
            string sub;
            iss >> sub;
            vs.push_back(sub);
        }
        parseLine(vs);
    }
}


// -------------------------------------------------------------------
// Utilities

void setColor(int ix, int iy, const vec4& color)
{
    int iy2 = g_height - iy - 1; // Invert iy coordinate.
    g_colors[iy2 * g_width + ix] = color;
}


// -------------------------------------------------------------------
// Intersection routine
vec4 intersects(const Ray& ray, int sphereIndex, int recNum) {
	

	Ray transformedRay;
	transformedRay.origin = (g_spheres[sphereIndex]).inverseMat * ray.origin;
	transformedRay.dir = (g_spheres[sphereIndex]).inverseMat * (ray.dir);
	transformedRay.dir = (transformedRay.dir);

	float tempa = length(transformedRay.origin);
	//cout << "Length of a " << length(transformedRay.origin) << endl;
	float a = pow(length(transformedRay.dir), 2);
	float b = dot(transformedRay.dir, transformedRay.origin);
	float c = pow(length(toVec3(transformedRay.origin)), 2) - 1.0;

	float discriminant = b*b - a*c;
	if (discriminant < 0) {
		//No intersection
		return vec4((float)(INTMAX_MAX), (float)(INTMAX_MAX), (float)(INTMAX_MAX), 0.0f);
	}
	if (discriminant == 0) {
		//1 intersection
		vec4 tempt = (-1 * b / a) * ray.dir;
		
		if ((recNum < MAX_RECURSION && tempt > 0) || (recNum == MAX_RECURSION && tempt[2] > g_near))
			return ray.origin + (-1 * b / a) * ray.dir;
		else
			return g_bg;
		
	}
	//2 intersections, so return closer intersection
	float t1 = ((-1 * b - sqrt(discriminant)) / (a));
	float t2 = ((-1 * b + sqrt(discriminant)) / (a));

	vec4 r1 = ray.origin + t1 * (ray.dir);
	vec4 r2 = ray.origin + t2 * (ray.dir);

	//Check for depth of field based on the level of the recursive call 
	//If we're on the the third recursion, modify the parameters accordingly
	if (recNum == MAX_RECURSION && r1[2] > -1)
		t1 = -1 * g_near;
	if (recNum == MAX_RECURSION && r2[2] > -1)
		t2 = -1 * g_near;

	if (t1 < 0) //Intersects at a time < 0, so no intersection
	{
		if (t2 < 0) {
			return vec4((float)(INTMAX_MAX), (float)(INTMAX_MAX), (float)(INTMAX_MAX), 0.0f);
		}
		return r2;
	}
	if (t2 < 0) //Intersects at a time < 0, so no intersection
	{
		if (t1 < 0) {
			return vec4((float)(INTMAX_MAX), (float)(INTMAX_MAX), (float)(INTMAX_MAX), 0.0f);
		}
		return r1;
	}
	return (t1 < t2 ? r1 : r2);

	//return ray.origin + t2 * ray.dir;
	/*Check for near point*/


}


// -------------------------------------------------------------------
// Ray tracing

vec4 trace(const Ray& ray, int recNum)
{
	// TODO: implement your ray tracing routine here.
	

	//Get sphere
	vec4 currClosestColPoint;
	int closestSphere = -1;
	currClosestColPoint[0] = INTMAX_MAX;
	currClosestColPoint[1] = INTMAX_MAX;
	currClosestColPoint[2] = INTMAX_MAX;
	currClosestColPoint[3] = 1;
	for (int currSphere = 0; currSphere < g_spheres.size(); currSphere++) {
		vec4 collisionPoint = intersects(ray, currSphere, recNum);
		if (collisionPoint[3] > 0.0f && length(collisionPoint - ray.origin) < length(currClosestColPoint - ray.origin))
		{
			currClosestColPoint = collisionPoint;
			closestSphere = currSphere;
		}
	}
	if (closestSphere == -1 || recNum < 0) {
		if (recNum == MAX_RECURSION)
			return g_bg;
		else
			return vec4(0.0f, 0.0f, 0.0f, 1.0f);
	}

	
	vec4 normal = (currClosestColPoint - g_spheres[closestSphere].center);
	normal = (transpose(g_spheres[closestSphere].inverseMat) * normal);
	normal = normalize(vec4(normal[0], normal[1], normal[2], 0));
	normal = (transpose(g_spheres[closestSphere].inverseMat) * normal);
	normal = normalize(vec4(normal[0], normal[1], normal[2], 0));
	//return vec4(abs(normal[0]), abs(normal[1]), abs(normal[2]), 1);

	//Check diffuse light
	vec4 totalColor = vec4(0.0f, 0.0f, 0.0f, 1.0f);
	for (int currLight = 0; currLight < g_lights.size(); currLight++) {
		Ray toLight;
		toLight.origin = currClosestColPoint + .01 * normal;
		toLight.dir = (g_lights[currLight].center - (currClosestColPoint + .01 * normal));
		//std::cout << "toLight: " << toLight.dir[0] << "," << toLight.dir[1] << "," << toLight.dir[2] << "," << toLight.dir[3] << "\n";

		bool hasCollided = false;
		for (int currSphere = 0; currSphere < g_spheres.size(); currSphere++) {
			vec4 collisionPoint = intersects(toLight, currSphere, recNum-1);
			if (collisionPoint[3] > 0 && length(collisionPoint - currClosestColPoint) > THRESHOLD_LENGTH){
				hasCollided = true;
			}
		}
		if (!hasCollided) {
			
			float lightCoeff = dot(normal, normalize(toLight.dir));
			if (lightCoeff < 0)
				lightCoeff = 0;	
			//Diffuse component
			totalColor += vec4(g_spheres[closestSphere].color[0]* g_lights[currLight].color[0],
				g_spheres[closestSphere].color[1] * g_lights[currLight].color[1], 
				g_spheres[closestSphere].color[2] * g_lights[currLight].color[2],
				0)*lightCoeff* g_spheres[closestSphere].Kd; //g_lights[currLight].color
			//Specular component
			vec4 refVec = normalize(toLight.dir - 2 * (dot(normal, toLight.dir)) * normal);
			float specCoeff = (dot(ray.dir, refVec));
			if (specCoeff < 0)
				specCoeff = 0;
			vec4 specColor = g_lights[currLight].color * pow(specCoeff, g_spheres[closestSphere].n) ;
			totalColor += specColor * g_spheres[closestSphere].Ks;
			if (totalColor[0] > 1)
				totalColor[0] = 1;
			if (totalColor[1] > 1)
				totalColor[1] = 1;
			if (totalColor[2] > 1)
				totalColor[2] = 1;
		}
	}
	//Check reflection
	vec4 refcolor = vec4(0, 0, 0, 0);
	if (g_spheres[closestSphere].Kr > 0.01) {
		Ray newray;
		newray.origin = currClosestColPoint + .01 * normal;
		newray.dir = ray.dir - 2 * dot(normal, ray.dir)*normal;
		refcolor = trace(newray, recNum - 1) * g_spheres[closestSphere].Kr;
	}
	//cout << "KR: " << g_spheres[closestSphere].Kr << " refcolor: " << refcolor << endl;
	//std::cout << "Dcolor: " << totalColor[0] <<","<< totalColor[1] << ","<< totalColor[2] << "\n";
	totalColor = vec4(g_spheres[closestSphere].color[0] * g_ambient[0], g_spheres[closestSphere].color[1] * g_ambient[1],
		g_spheres[closestSphere].color[2] * g_ambient[2], 1.0) * g_spheres[closestSphere].Ka + totalColor + refcolor ;
	if (totalColor[0] > 1)
		totalColor[0] = 1;
	if (totalColor[1] > 1)
		totalColor[1] = 1;
	if (totalColor[2] > 1)
		totalColor[2] = 1;
	totalColor[3] = 1;
	return totalColor;
}

vec4 getDir(int ix, int iy)
{
   
    vec4 dir;
	float ratiox = (float)ix / (float)g_width;
	float ratioy = (float)iy / (float)g_height;
    dir = normalize(vec4(ratiox*(g_right - g_left) + g_left, ratioy*(g_top - g_bottom) + g_bottom, -1.0f, 0.0f));
	return dir;

}

void renderPixel(int ix, int iy)
{
    Ray ray;
    ray.origin = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    ray.dir = getDir(ix, iy);
    vec4 color = trace(ray, MAX_RECURSION);
    setColor(ix, iy, color);
}

void render()
{
    for (int iy = 0; iy < g_height; iy++)
        for (int ix = 0; ix < g_width; ix++)
            renderPixel(ix, iy);
}


// -------------------------------------------------------------------
// PPM saving

void savePPM(int Width, int Height, char* fname, unsigned char* pixels) 
{
    FILE *fp;
    const int maxVal=255;

    printf("Saving image %s: %d x %d\n", fname, Width, Height);
    fp = fopen(fname,"wb");
    if (!fp) {
        printf("Unable to open file '%s'\n", fname);
        return;
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", Width, Height);
    fprintf(fp, "%d\n", maxVal);

    for(int j = 0; j < Height; j++) {
        fwrite(&pixels[j*Width*3], 3, Width, fp);
    }

    fclose(fp);
}

void saveFile()
{
    // Convert color components from floats to unsigned chars.
    // TODO: clamp values if out of range.
    unsigned char* buf = new unsigned char[g_width * g_height * 3];
    for (int y = 0; y < g_height; y++)
        for (int x = 0; x < g_width; x++)
            for (int i = 0; i < 3; i++)
                buf[y*g_width*3+x*3+i] = (unsigned char)(((float*)g_colors[y*g_width+x])[i] * 255.9f);
    
    savePPM(g_width, g_height, (char*)(g_output.c_str()), buf);
    delete[] buf;
}


// -------------------------------------------------------------------
// Main

int main(int argc, char* argv[])
{
	
    if (argc < 2)
    {
        cout << "Usage: template-rt <input_file.txt>" << endl;
        exit(1);
    }
    loadFile(argv[1]);
	if (!argv[1])
		g_output = "output.ppm";
	else {
		g_output = string(argv[1]);
		g_output = g_output.substr(g_output.find_last_of("/") + 1);
		g_output = g_output.substr(0, g_output.size() - 4) + "output.ppm";
	}
    render();
    saveFile();
	
	return 0;
}

