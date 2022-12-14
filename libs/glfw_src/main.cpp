// demonstrates usage of the quadric fitting library
// pass a mesh to fit on the command line (./aqd_demo meshname)
// and this will fit all quadric types, then render the results
// press tab or left/right arrows to cycle through fitting results

#include "view.h"

#include "quadricfitting.h"

#include <fstream>
#include <map>

using namespace std;

extern "C" { FILE __iob_func[3] = { *stdin,*stdout,*stderr }; }

void drawMesh(allquadrics::TriangleMesh &mesh) {
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < mesh.triangles.size(); i++) {
        
        if (i < mesh.triangleTags.size()) {
            if (mesh.triangleTags[i] == mesh.activeTag) {
                glColor3d(1,0,0);
            } else {
                glColor3d(1,1,1);
            }
        }
		glNormal3dv(&mesh.triangleNormals[i][0]);
		for (int ii = 0; ii < 3; ii++) {
			glVertex3dv(&mesh.vertices[ mesh.triangles[i].ind[ii] ][0]);
		}
	}
	glEnd();
}

void display(Viewport &viewport, allquadrics::TriangleMesh &reference, allquadrics::TriangleMesh &quadric) {

	// setup gl state
	glClearColor(.5f, .7f, 1, 1);
	glEnable(GL_NORMALIZE);
	glDisable(GL_CULL_FACE);

	// clear the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// setup the camera
	viewport.loadView();

	glColor3d(1, 1, 1);
	drawMesh(reference);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(.7, .3, .1, .3);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	drawMesh(quadric);
	glCullFace(GL_BACK);
	drawMesh(quadric);

	glfwSwapBuffers();
}


void reshape(int w, int h) {
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(70.0, ((double)w / MAX(h, 1)), .0001, 5.0);
	//glOrtho(-10,10,-10,10,1,100);

    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

int main( int argc, char **argv )
{
	Viewport viewport;
	allquadrics::TriangleMesh inputMesh, quadricMesh;

	const char *defaultInput = "../example_data/default.obj";
	const char *meshfile = argc > 1 ? argv[1] : 0;
	if (!meshfile) {
		cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
		meshfile = defaultInput;
	}
	if (!inputMesh.loadObj(meshfile)) {
        cerr << "Couldn't load file " << meshfile << endl;
        return 1;
    }

	const char *quadricTypeNames[] = { "general", "rotationally symmetric", "plane", "sphere",
		"general cylinder", "circular cylinder", "general cone", "circular cone",
		"ellipsoid (BIASED METHOD)", "hyperboloid (BIASED METHOD)",
		"ellipsoid (IMPROVED)", "hyperboloid (IMPROVED)",
		"hyperboloid (1 sheet)", "hyperboloid (2 sheet)", "paraboloid", 
		"paraboloid (elliptical)", "paraboloid (hyperbolic)", 
		"elliptical cylinder", "hyperbolic cylinder", "parabolic cylinder" };

	
	// Always recenter and scale your data before fitting!
	inputMesh.centerAndScale(1);

	// fit quadric
	vector<allquadrics::Quadric> qfits;
	fitAllQuadricTypes(inputMesh, qfits);
	vector<allquadrics::TriangleMesh> meshes; meshes.resize(qfits.size());

	// generate meshes
	for (size_t i = 0; i < qfits.size(); i++) {
        vec3 range_min(-1,-1,-1), range_max(1,1,1);
		qfits[i].buildMeshFromQuadric(meshes[i], range_min, range_max);
	}

	int showQuadricFit = 0;

    glfwInit();

    // default window size:
    int W = 800, H = 500;
    // Open window
    int ok = glfwOpenWindow(W, H, 8, 8, 8, 8, 24, 8, GLFW_WINDOW);
    if( !ok ) { glfwTerminate(); return 0; }
    // setup gl window/perspective based on window height
    reshape(W,H);

    // Set window title
    glfwSetWindowTitle( "Quadric Fitting Demo" );

    // Enable sticky keys
    glfwEnable( GLFW_STICKY_KEYS );


    // set some lights
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .4f, .3f, .3f };
	   float pos[4] = { 0, 2, 0, 0 };
       
       glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT1, GL_POSITION, pos);
       glEnable(GL_LIGHT1);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .1f, .2f, .2f};
       float pos[4] = { 0, 0, -2, 0 };
       glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT2, GL_POSITION, pos);
       glEnable(GL_LIGHT2);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .1f, .2f, .1f};
       float pos[4] = { -1, 0, 0, 0 };
       glLightfv(GL_LIGHT3, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT3, GL_POSITION, pos);
       glEnable(GL_LIGHT3);
    }
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

	srand(95);



    viewport.resetCam();



    int mx, my;
    //glfwDisable( GLFW_MOUSE_CURSOR );
    glfwGetMousePos(&mx, &my);

	vec3 mousePos;
    double lastTime = 0, timePerFrame = 1.0/30.0;

    vec4 lightPos(-5,5,5,0);

    bool mouseWasUp = false;

	cout << endl << "press TAB to change quadric type" << endl;
	cout << "now showing quadric type: " << quadricTypeNames[showQuadricFit] << endl;
    
    string windowTitle = "Quadric Fitting Demo: ";
    windowTitle += quadricTypeNames[showQuadricFit];
    windowTitle += " (use arrows or tab to change fitted type)";
    glfwSetWindowTitle(windowTitle.c_str());

    // Main rendering loop
    while (true) {

        bool changedLast = false;

		int logics = 0;
		while (glfwGetTime() - lastTime > timePerFrame) {
			{
				// UI/interaction code:
				if (keyHit(GLFW_KEY_TAB) || keyHit(GLFW_KEY_SPACE) || keyHit(GLFW_KEY_RIGHT)) {
					showQuadricFit++;
					showQuadricFit = showQuadricFit % meshes.size();
					cout << endl << "now showing quadric type: " << quadricTypeNames[showQuadricFit] << endl;
                    windowTitle = "Quadric Fitting Demo: ";
                    windowTitle += quadricTypeNames[showQuadricFit];
                    windowTitle += " (use arrows or tab to change fitted type)";
                    glfwSetWindowTitle(windowTitle.c_str());
				}
				if (keyHit(GLFW_KEY_LEFT)) {
					showQuadricFit--;
					if (showQuadricFit < 0) showQuadricFit = (int)(meshes.size()) - 1;
					cout << endl << "now showing quadric type: " << quadricTypeNames[showQuadricFit] << endl;
                    windowTitle = "Quadric Fitting Demo: ";
                    windowTitle += quadricTypeNames[showQuadricFit];
                    windowTitle += " (use arrows or tab to change fitted type)";
                    glfwSetWindowTitle(windowTitle.c_str());
				}
                
				int nmx, nmy;
				glfwGetMousePos(&nmx, &nmy);

                if (!glfwGetMouseButton(GLFW_MOUSE_BUTTON_1) && !glfwGetMouseButton(GLFW_MOUSE_BUTTON_2)) {
                    mouseWasUp = true;
                }

                if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_1)) { // mouse movements update the view
                    // Rotate viewport orientation proportional to mouse motion
                    viewport.mousePos = vec2((double)mx / (double)W,(double)my / (double)H);
                    vec2 newMouse = vec2((double)nmx / (double)W,(double)nmy / (double)H);
                    vec2 diff = (newMouse - viewport.mousePos);
                    double len = diff.length();
					if (!glfwGetKey('Z') && !glfwGetKey(GLFW_KEY_LALT) && len > .001) {
                        vec3 axis = vec3(diff[1]/len, diff[0]/len, 0);
                        viewport.orientation = rotation3D(axis, 180 * len) * viewport.orientation;
                    }
                    if ( (glfwGetKey('Z') || glfwGetKey(GLFW_KEY_LALT)) && fabs(diff[1]) > .001) {
    	                viewport.zoom += diff[1];
    	                if (viewport.zoom < .001) viewport.zoom = .001;
                    }

                    //Record the mouse location for drawing crosshairs
                    viewport.mousePos = newMouse;
                }
                
                mx = nmx; my = nmy;
			}

			lastTime += timePerFrame;
			logics++;

			if (changedLast || logics > 10) // slow down if you really can't keep up
				break;
		}

		if (logics > 0) {

            display(viewport, inputMesh, meshes[showQuadricFit]);
		}
		 else {
            glfwSleep( .001 ); // else release control to OS for 5 ms
        }

        // Check if the escape key was pressed, or if the window was closed
        if (glfwGetKey( GLFW_KEY_ESC ) || !glfwGetWindowParam( GLFW_OPENED )) {
            break;
        }
    }

    // cleanup and exit
    glfwTerminate();
    return 0;
}
