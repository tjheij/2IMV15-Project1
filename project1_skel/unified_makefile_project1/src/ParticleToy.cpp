// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "CollisionLine.h"
#include "imageio.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <string>

/* macros */

/* external definitions (from solver) */
extern void simulation_step( std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, std::vector<CollisionLine*> &collisionVector, float dt, int scheme);
extern void initScenario(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints, std::vector<CollisionLine*> &colliders, int scenarioId);

/* global variables */
static int scenarioId;
static int scheme = 0;
static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;
static int update_number;

// static Particle *pList;
static std::vector<Particle*> pVector;
static std::vector<Force*> fVector;
static std::vector<Constraint*> cVector;
static std::vector<CollisionLine*> collisionVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

Particle* mouseParticle = NULL;
SpringForce* mouseSpring = NULL;
/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{	
	for (Particle *p : pVector) {
		if (p) {
			delete p;
		}
	}

	for (Force *f : fVector) {
		if (f) {
			delete f;
		}
	}

	for (Constraint *c : cVector) {
		if (c) {
			delete c;
		}
	}

	for (CollisionLine *c : collisionVector) {
		if (c) {
			delete c;
		}
	}

	pVector.clear();
	fVector.clear();
	cVector.clear();
	collisionVector.clear();
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}
}

static void init_system(void)
{
	initScenario(pVector, fVector, cVector, collisionVector, scenarioId);
	if (scenarioId == 1) {
		mouseParticle = pVector[1];
		//pVector.push_back(mouseParticle);
	} else if (scenarioId == 4) {
		mouseParticle = pVector[pVector.size()/2];
		//mouseSpring = new SpringForce(pVector[0], mouseParticle, 0.1f, 1.0f, 0.1f);

	} else {
		mouseParticle = NULL;
	}
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	for(Particle *p : pVector)
	{
		p->draw();
	}
}

static void draw_forces ( void )
{
	for(Force *f : fVector)
	{
		f->draw();
	}
}

static void draw_constraints ( void )
{
	for(Constraint *c : cVector)
	{
		c->draw();
	}
}

static void draw_colliders ( void )
{
	for(CollisionLine *c : collisionVector)
	{
		c->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/
// create a function that makes the mouse particle follow the mouse


void mouse_interact(){
	int i, j;
	float x, y;
	//Return if 
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       (mx) /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

	x = ((float) i / N)-0.5f;
	y = ((float) j / N)-0.5f;

	if ( mouse_down[0] && scenarioId == 1 ) {
		mouseParticle->set_state(Vec2f(x,y),Vec2f(0,0));
	}
	if (mouse_down[0] && scenarioId == 3 && !cVector.empty()) {
	}
	if ( mouse_down[0] && scenarioId == 4 ) {
		mouseParticle->set_state(Vec2f(x,y),Vec2f(0,0));
	}
	if (mouse_release[0] && scenarioId == 4) {
		mouse_release[0] = 0;
	}
}
static void get_from_UI ()
{
	int i, j;
	int size, flag;
	int hi, hj;
	// float x, y;
	//Return if 
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {
		
	}


	if ( mouse_down[2] ) {
	
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	if( mouse_release[0] ) {
		mouse_release[0] = 0;
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	try {
		scenarioId = std::stoi(std::string() + ((char)key));
		free_data();
		init_system();
		dt = 0.025f;
	} catch (const std::invalid_argument &ex) {
		switch ( key )
		{
		case 'c':
		case 'C':
			clear_data ();
			break;

		case 'd':
		case 'D':
			dump_frames = !dump_frames;
			break;

		case 'q':
		case 'Q':
			free_data ();
			exit ( 0 );
			break;
		case 'e':
		case 'E':
			scheme = 0;
			printf("SemiImplicit Euler\n");
			break;
		case 'm':
		case 'M':
			scheme = 1;
			printf("Midpoint\n");
			break;
		case 'r':
		case 'R':
			scheme = 2;
			printf("RungeKutta\n");
			break;
		case 'i':
		case 'I':
			scheme = 3;
			printf("Implicit Euler\n");
			break;
		case 'j':
		case 'J':
			scheme = 4;
			printf("Explicit Euler\n");
			break;
		case 'v':
		case 'V':
			scheme = 5;
			printf("Verlet\n");
			break;
		case ' ':
			dsim = !dsim;
			break;
		}
	}
}

static void special_key_func(int key, int x, int y)
{
	switch(key)
	{
		case GLUT_KEY_UP:
			dt += 0.005f;
			printf("dt = %f\n", dt);
			break;
		case GLUT_KEY_DOWN:
			if (dt > 0.005f) {
				dt -= 0.005f;
				printf("dt = %f\n", dt);
			}
			break;
		case GLUT_KEY_LEFT:
			break;
		case GLUT_KEY_RIGHT:
			// printf("right\n");
			break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;
	
	//Not left mouse down, set hmx, hmy
	if(!mouse_down[0]){hmx=x; hmy=y;}
	//set down/release/shiftclick to state of this button press
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
	//printf("%d\n", mouse_down[button]);

}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if ( dsim ) {
		
	} else {
		get_from_UI(); //remap_GUI();
	}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void update_func(int state) {
	if (dsim) {
		simulation_step( pVector, fVector, cVector, collisionVector, dt, scheme);
		mouse_interact();
		update_number++;

		//std::cout << "Update: " << update_number << "\r" << std::flush;
	}

	glutTimerFunc((int)(1000.f * dt), update_func, 0);
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();
	draw_colliders();

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
	glutSpecialFunc ( special_key_func);

	glutTimerFunc((int)(1000.f * dt), update_func, 0);
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 0.025f;
		//dt = 0.1f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Press 0 for gravity scenario\n" );
	printf ( "\t Press 1 for spring with mouse interaction scenario\n" );
	printf ( "\t Press 2 for constraints scenario\n" );
	printf ( "\t Press 3 for cloth with circular constraints scenario\n" );
	printf ( "\t Press 4 for cloth with mouse interaction scenario\n" );
	printf ( "\t Press 'e' for semi-euler integration\n" );
	printf ( "\t Press 'm' for midpoint integration\n" );
	printf ( "\t Press 'r' for rungekutta integration\n" );
	printf ( "\t Press 'v' for verlet integration\n" );
	printf( "\t Press 'i' for implicit euler integration\n" );
	printf( "\t Press 'j' for explicit euler integration\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	update_number = 0;
	
	init_system();
	
	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

