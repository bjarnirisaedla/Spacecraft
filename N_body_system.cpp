#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>


#include <iostream>
#include <fstream>
#include "vtkhelper.hpp"
#include "modes.hpp"

#include <ctime>
#include <omp.h>

static void show_usage(std::string name);

int main(int argc, char* argv[])
{
	// setting up the system and planets
	double mu = 0.012150585609624;
	// double mass_moon = mu/(1.0 - mu);
	mysystem.addBody(0.01, Vector2d(0.0, 0.0), 1.0 - mu, Vector3d(1.,0.,0.));		// earth, red
	mysystem.addBody(0.01, Vector2d(1.0, 0.0), mu, Vector3d(0.,0.,1.));		// moon, blue
	// 0.012299
	
	for(int i = 0; i < argc; ++i){
		if((!strcmp(argv[i],"-rk4") || !strcmp(argv[i],"--rungekutta4"))){
			mysystem.setIntegrator(Integrator_RK4);
			if(i == 1){
				trajectoryMode();
				return 0;
			}
			break;
		}
		if((!strcmp(argv[i],"-vv") || !strcmp(argv[i],"--velocityverlet"))){
			mysystem.setIntegrator(Integrator_Verlet);
			if(i == 1){
				trajectoryMode();
				return 0;
			}
			break;
		}		
	}

	if(argc == 1)
		trajectoryMode();
	else if((!strcmp(argv[1],"-t") || !strcmp(argv[1],"--trajectory")))
		trajectoryMode();
	else if((!strcmp(argv[1],"-h") || !strcmp(argv[1],"--help"))){
		show_usage(argv[0]);
		return 0;
	}
	else if((!strcmp(argv[1],"-d") || !strcmp(argv[1],"--dense"))){
		int N = 0;
		for(int i = 1; i < argc-2; i++){
			// input initial velocity, otherwise it is set to zero (in vtkhelper.hpp)
			if(!strcmp(argv[i],"-v0"))
				InputVelocity = Vector2d(atof(argv[i+1]), atof(argv[i+2]));
		}
		for(int i = 1; i < argc-1; i++){
			if(!strcmp(argv[i],"-N"))
				N = atoi(argv[i+1]);
		}
		(N != 0) ? denseMode(N) : denseMode();
	}	
	else if((!strcmp(argv[1],"-g") || !strcmp(argv[1],"--grid"))){
		gridMode();
	}
	else if((!strcmp(argv[1],"-b") || !strcmp(argv[1],"--backwards"))){
		backwardsMode(false);
	}
	else if((!strcmp(argv[1],"-cb") || !strcmp(argv[1],"--backwardsimage"))){
		backwardsMode(true);
	}
	else if((!strcmp(argv[1],"-c") || !strcmp(argv[1],"--condition"))){
		condMode();
	}
	else if((!strcmp(argv[1],"-f") || !strcmp(argv[1],"--ftle"))){
		FTLEMode(false);
	}
	else if((!strcmp(argv[1],"-cf") || !strcmp(argv[1],"--ftleimage"))){
		FTLEMode(true);
	}
	else{
		show_usage(argv[0]);
		return 0;
	}

	return EXIT_SUCCESS;
}


static void show_usage(std::string name){
	std::cerr << "Usage: " << name << " <option> <integrator>\n"
	          << "Options:\n"
	          << "\t<empty>\t\t\tTrajectory mode\n"
	          << "\t-h,--help\t\tShow this help message\n"
	          << "\t-t,--trajectory\t\tTrajectory mode\n"
	          << "\t-d,--dense\t\tDense grid mode, can take more arguments\n" 
	          << "\t\t-v0\t\tMust be followed by 2 numbers. Sets initial velocity (default is zero)\n"
	          << "\t\t-N\t\tMust be followed by a number. Set resolution of dense graph\n"
	          << "\t\t\t\t(default is 30, larger takes some time)\n"
	          << "\t-g,--grid\t\tGrid mode\n"
	          << "\t-b,--backwards\t\tBackwards integration mode\n"
	          << "\t-cb,--backwardsimage\tCreate an image from backwards integration\n"
	          << "\t-f,--ftle\t\tFTLE mode\n"
	          << "\t-cf,--ftleimage\t\tCreate FTLE image\n"
	          << "Integrators:\n"
	          << "\t-rk4,--rungekutta4\tUse Runge Kutta step integration (default)\n"
	          << "\t-vv,--velocityverlet\tUse velocity Verlet step integration\n"
	          << std::endl;
}
