#include "isolines.hpp"
#include "denseMouseInteractor.hpp"
#include "gridMouseInteractor.hpp"
#include "trajectoryMouseInteractor.hpp"
#include "backwardsMouseInteractor.hpp"
#include "ftleMouseInteractor.hpp"

#include <vtkImageActor.h>
#include <vtkPNGReader.h>

void writeJacobianPropertiesToFile(const particle& p, char c);

enum Lclass {Saddle, Center, Att_Node, Att_Focus, Rep_Node, Rep_Focus};

void trajectoryMode(){
	// create VTK actors for Lagrange points
	std::vector<Vector2d> Lpoints = mysystem.findLPoints();
	std::vector<vtkActorPtr> actorLpoints;
	for(Vector2d l: Lpoints){
		vtkActorPtr actorLpointTmp = createCircleActor(0.01, l, Vector3d(1.,0,1.)); // magenta
		actorLpoints.push_back(actorLpointTmp);
		// std::cout << "L point found:\tx = " + std::to_string(l.x()) + ",\ty = " + std::to_string(l.y()) << std::endl;
	}
	particle p(Lpoints[4], Vector2d(0.0,0.0));
	// mysystem.JacobianMat(p);
	// writeJacobianPropertiesToFile(p, 'w');

	// create a VTK actor for all bodies
	std::vector<body_t> bodies = mysystem.getBodies();
	std::vector<vtkActorPtr> actorBodies;
	for(body_t b: bodies){
		vtkActorPtr actorTmp = createCircleActor(b.radius, b.position_curr);
		actorBodies.push_back(actorTmp);
	}
	
	// create border as a line actor
	Vector2d xlimit = mysystem.getXlimits();
	Vector2d ylimit = mysystem.getYlimits();
	MatrixXd m1(2, 5);
	m1 << xlimit.x(), xlimit.y(), xlimit.y(), xlimit.x(), xlimit.x(), ylimit.x(), ylimit.x(), ylimit.y(), ylimit.y(), ylimit.x();
	vtkActorPtr borderActor = createLineActor(m1, Vector3d(1.,1.,1.));

	// create a test trajectory
	Vector2d dummyPos = Vector2d(-0.652123, 0.177244);
	Vector2d dummyVel = Vector2d(0.516682, 0.190620);
	p.pos = dummyPos;
	p.vel = dummyVel;
	Vector3d dummyColor(.3,.3,.5);
	traj_pos = mysystem.integrator<maxIteration>(dummyPos, dummyVel, dummyColor);
	vtkActorPtr testtraj = createLineActor(Vector3d(.3,.3,.5));

	// create actors for isolines
	Isolines iso(mysystem.getJacobiConstantAt(p));
	std::vector<vtkActorPtr> actorsIsolines = iso.compute();

	// create actors for eigenvalues around Lagrange points
	std::vector<vtkActorPtr> actorsLeig;
	for(Vector2d l: Lpoints){
		particle lp(l, Vector2d(0.0,0.0));
		
		MatrixXd Jac = mysystem.JacobianMat(lp);
		EigenSolver<MatrixXd> solver;
		solver.compute(Jac);
		// classifying with colors
		int Rp = 0, Rm = 0, Ip = 0, Im = 0;
		for(int i = 0; i < 4; ++i){
			if(solver.eigenvalues()[i].real() > 1e-8) ++Rp;
			else if(solver.eigenvalues()[i].real() < -1e-8) --Rm;
			if(solver.eigenvalues()[i].imag() > 1e-8) ++Ip;
			else if(solver.eigenvalues()[i].imag() < -1e-8) --Im;
		}

		// if(Rm && Rp)
		// 	std::cout << "Found saddle" << std::endl;
		// if(!Rm && !Rp)
		// 	std::cout << "Found center" << std::endl;
		// if(Rm && !Rp)
		// 	std::cout << "Found attracting node" << std::endl;
		// if(!Rm && Rp)
		// 	std::cout << "Found repelling node" << std::endl;

		// Eigenvectors
		for (int i = 0; i < 4; ++i){
			if(std::abs(solver.eigenvalues()[i].real()) < 1e-8)
				continue;
			Vector2d lpos = solver.eigenvectors().col(i).real().head(2).normalized() * 0.1;
			Vector3d eigColor = (solver.eigenvalues()[i].real() > 0) ? Vector3d(1.,0.,0.) : Vector3d(0.,0.,1.);
			MatrixXd lina(2,2);
			lina << l, l+lpos;
			actorsLeig.push_back(createLineActor(lina, eigColor));
			lina << l, l-lpos;
			actorsLeig.push_back(createLineActor(lina, eigColor));
		}
	}

	
	vtkSmartPointer<vtkRenderer> renderer =
	    vtkSmartPointer<vtkRenderer>::New();
	// renderer->AddActor(actorJacobi);
	for(vtkActorPtr a: actorsLeig)
		renderer->AddActor(a);
	for(vtkActorPtr a: actorLpoints)
		renderer->AddActor(a);
	for(vtkActorPtr a: actorBodies)
		renderer->AddActor(a);
	renderer->AddActor(borderActor);
	// for(vtkActorPtr a: actorsIsolines)
	// 	renderer->AddActor(a);
	// renderer->AddActor(testtraj);	
	renderer->SetBackground(.1,.1,.1); // Background color dark grey
	 
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	    vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1200,1200);
	 

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = 
	  vtkSmartPointer<vtkWorldPointPicker>::New();
	renderWindowInteractor->SetPicker(worldPointPicker);

	vtkSmartPointer<TrajectoryMouseInteractorStyle> style = 
	  vtkSmartPointer<TrajectoryMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle( style );

	renderWindow->Render();
	renderWindowInteractor->Start();
}


void denseMode(int M = 30){
	// create a VTK actor for all bodies
	std::vector<body_t> bodies = mysystem.getBodies();
	std::vector<vtkActorPtr> actorBodies;
	for(body_t b: bodies){
		vtkActorPtr actorTmp = createCircleActor(b.radius, b.position_curr, b.color);
		actorTmp->GetProperty()->EdgeVisibilityOn();
		actorBodies.push_back(actorTmp);
	}
	
	// create border as a line actor
	Vector2d xlimit = mysystem.getXlimits();
	Vector2d ylimit = mysystem.getYlimits();
	MatrixXd m1(2, 5);
	m1 << xlimit.x(), xlimit.y(), xlimit.y(), xlimit.x(), xlimit.x(), ylimit.x(), ylimit.x(), ylimit.y(), ylimit.y(), ylimit.x();
	vtkActorPtr borderActor = createLineActor(m1, Vector3d(1.,1.,1.));

	// color whole area, programmed should be called with ./N_body_system dense v_0x v_0y
	std::vector<vtkActorPtr> areaActors;
	Vector2d xlimits = mysystem.getXlimits();
	Vector2d ylimits = mysystem.getYlimits();
	int N = 10; // divide that area to 10x10 cells
	double dx = (xlimits.y() - xlimits.x())/N;
	double dy = (ylimits.y() - ylimits.x())/N;
	#pragma omp parallel for
	for(int i = 0; i < N; ++i){
		for(int j = 0; j < N; ++j){
			vtkActorPtr areaActorTmp = colorArea(Vector2d(xlimits.x()+dx*i,xlimits.x()+dx*(i+1)),
				Vector2d(ylimits.x()+dy*j,ylimits.x()+dy*(j+1)), InputVelocity, M);
			areaActors.push_back(areaActorTmp);
		}
	}
	
	vtkSmartPointer<vtkRenderer> renderer =
	    vtkSmartPointer<vtkRenderer>::New();
	for(vtkActorPtr a: actorBodies)
		renderer->AddActor(a);
	renderer->AddActor(borderActor);
	for(vtkActorPtr a: areaActors)
		renderer->AddActor(a);
	renderer->SetBackground(.1,.1,.1); // Background color dark grey
	 
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	    vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1200,1200);
	 

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = 
	  vtkSmartPointer<vtkWorldPointPicker>::New();
	renderWindowInteractor->SetPicker(worldPointPicker);

	vtkSmartPointer<DenseMouseInteractorStyle> style = 
	  vtkSmartPointer<DenseMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle( style );

	renderWindow->Render();
	renderWindowInteractor->Start();
}

void gridMode(){
	// create a VTK actor for all bodies
	std::vector<body_t> bodies = mysystem.getBodies();
	std::vector<vtkActorPtr> actorBodies;
	for(body_t b: bodies){
		vtkActorPtr actorTmp = createCircleActor(b.radius, b.position_curr, b.color);
		actorTmp->GetProperty()->EdgeVisibilityOn();
		actorBodies.push_back(actorTmp);
	}
	
	// create border as a line actor
	Vector2d xlimit = mysystem.getXlimits();
	Vector2d ylimit = mysystem.getYlimits();
	MatrixXd m1(2, 5);
	m1 << xlimit.x(), xlimit.y(), xlimit.y(), xlimit.x(), xlimit.x(), ylimit.x(), ylimit.x(), ylimit.y(), ylimit.y(), ylimit.x();
	vtkActorPtr borderActor = createLineActor(m1, Vector3d(1.,1.,1.));
	
	vtkSmartPointer<vtkRenderer> renderer =
	    vtkSmartPointer<vtkRenderer>::New();
	for(vtkActorPtr a: actorBodies)
		renderer->AddActor(a);
	renderer->AddActor(borderActor);
	renderer->SetBackground(.1,.1,.1); // Background color dark grey
	 
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	    vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1200,1200);
	 

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = 
	  vtkSmartPointer<vtkWorldPointPicker>::New();
	renderWindowInteractor->SetPicker(worldPointPicker);

	vtkSmartPointer<GridMouseInteractorStyle> style = 
	  vtkSmartPointer<GridMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle( style );

	renderWindow->Render();
	renderWindowInteractor->Start();
}

void backwardsMode(bool createImage = false){
	if(createImage){
		createBackwardsImage();
		return;
	}

	std::string inputFilename = "../figures/backwards.png";
	vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
	if(!reader->CanReadFile(inputFilename.c_str())){
		std::cerr << "Can't read image: " + inputFilename << std::endl;
		return;
	}
	reader->SetFileName(inputFilename.c_str());
	reader->Update();
	vtkSmartPointer<vtkImageData> imageData;
	imageData = reader->GetOutput();
	vtkSmartPointer<vtkImageActor> imageActor = vtkSmartPointer<vtkImageActor>::New();
	imageActor->SetInputData(imageData);

	// create VTK actors for Lagrange points
	std::vector<Vector2d> Lpoints = mysystem.findLPoints();
	std::vector<vtkActorPtr> actorLpoints;
	for(Vector2d l: Lpoints){
		vtkActorPtr actorLpointTmp = createCircleActor(0.005, l, Vector3d(1.,0,1.)); // magenta
		actorLpoints.push_back(actorLpointTmp);
		// std::cout << "L point found:\tx = " + std::to_string(l.x()) + ",\ty = " + std::to_string(l.y()) << std::endl;
	}

	// create a VTK actor for all bodies
	std::vector<body_t> bodies = mysystem.getBodies();
	std::vector<vtkActorPtr> actorBodies;
	for(body_t b: bodies){
		vtkActorPtr actorTmp = createCircleActor(b.radius, b.position_curr);
		actorBodies.push_back(actorTmp);
	}
	
	// create border as a line actor
	Vector2d xlimit = mysystem.getXlimits();
	Vector2d ylimit = mysystem.getYlimits();
	MatrixXd m1(2, 5);
	m1 << xlimit.x(), xlimit.y(), xlimit.y(), xlimit.x(), xlimit.x(), ylimit.x(), ylimit.x(), ylimit.y(), ylimit.y(), ylimit.x();
	vtkActorPtr borderActor = createLineActor(m1, Vector3d(1.,1.,1.));
	
	vtkSmartPointer<vtkRenderer> renderer =
	    vtkSmartPointer<vtkRenderer>::New();
	for(vtkActorPtr a: actorLpoints)
		renderer->AddActor(a);
	for(vtkActorPtr a: actorBodies)
		renderer->AddActor(a);
	renderer->AddActor(borderActor);
	// renderer->AddActor(actorScalar);
	renderer->SetBackground(.1,.1,.1); // Background color dark grey
	 
    vtkSmartPointer<vtkRenderer> backgroundRenderer =
        vtkSmartPointer<vtkRenderer>::New();
	backgroundRenderer->AddActor(imageActor);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
	    vtkSmartPointer<vtkRenderWindow>::New();

    backgroundRenderer->SetLayer(0);
    backgroundRenderer->InteractiveOff();
    renderer->SetLayer(1);
    renderWindow->SetNumberOfLayers(2);
    renderWindow->AddRenderer(renderer);
    renderWindow->AddRenderer(backgroundRenderer);
	renderWindow->SetSize(1200,1200);
	 

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = 
	  vtkSmartPointer<vtkWorldPointPicker>::New();
	renderWindowInteractor->SetPicker(worldPointPicker);

	vtkSmartPointer<BackwardsMouseInteractorStyle> style = 
	  vtkSmartPointer<BackwardsMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle( style );

	renderWindow->Render();
	renderWindowInteractor->Start();
}


void FTLEMode(bool createImage = false){
	if(createImage){
		createFTLEImage();
		return;
	}

	std::string inputFilename = "../figures/FTLE.png";
	vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
	if(!reader->CanReadFile(inputFilename.c_str())){
		std::cerr << "Can't read image" << std::endl;
		return;
	}
	reader->SetFileName(inputFilename.c_str());
	reader->Update();
	vtkSmartPointer<vtkImageData> imageData;
	imageData = reader->GetOutput();
	vtkSmartPointer<vtkImageActor> imageActor = vtkSmartPointer<vtkImageActor>::New();
	imageActor->SetInputData(imageData);

	// create VTK actors for Lagrange points
	std::vector<Vector2d> Lpoints = mysystem.findLPoints();
	std::vector<vtkActorPtr> actorLpoints;
	for(Vector2d l: Lpoints){
		vtkActorPtr actorLpointTmp = createCircleActor(0.005, l, Vector3d(1.,0,1.)); // magenta
		actorLpoints.push_back(actorLpointTmp);
		// std::cout << "L point found:\tx = " + std::to_string(l.x()) + ",\ty = " + std::to_string(l.y()) << std::endl;
	}

	// create a VTK actor for all bodies
	std::vector<body_t> bodies = mysystem.getBodies();
	std::vector<vtkActorPtr> actorBodies;
	for(body_t b: bodies){
		vtkActorPtr actorTmp = createCircleActor(b.radius, b.position_curr);
		actorBodies.push_back(actorTmp);
	}
	
	// create border as a line actor
	Vector2d xlimit = mysystem.getXlimits();
	Vector2d ylimit = mysystem.getYlimits();
	MatrixXd m1(2, 5);
	m1 << xlimit.x(), xlimit.y(), xlimit.y(), xlimit.x(), xlimit.x(), ylimit.x(), ylimit.x(), ylimit.y(), ylimit.y(), ylimit.x();
	vtkActorPtr borderActor = createLineActor(m1, Vector3d(1.,1.,1.));
	
	vtkSmartPointer<vtkRenderer> renderer =
	    vtkSmartPointer<vtkRenderer>::New();
	for(vtkActorPtr a: actorLpoints)
		renderer->AddActor(a);
	for(vtkActorPtr a: actorBodies)
		renderer->AddActor(a);
	renderer->AddActor(borderActor);
	renderer->SetBackground(.1,.1,.1); // Background color dark grey
	 
    vtkSmartPointer<vtkRenderer> backgroundRenderer =
        vtkSmartPointer<vtkRenderer>::New();
	backgroundRenderer->AddActor(imageActor);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
	    vtkSmartPointer<vtkRenderWindow>::New();

    backgroundRenderer->SetLayer(0);
    backgroundRenderer->InteractiveOff();
    renderer->SetLayer(1);
    renderWindow->SetNumberOfLayers(2);
    renderWindow->AddRenderer(renderer);
    renderWindow->AddRenderer(backgroundRenderer);
	renderWindow->SetSize(1200,1200);
	 

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = 
	  vtkSmartPointer<vtkWorldPointPicker>::New();
	renderWindowInteractor->SetPicker(worldPointPicker);

	vtkSmartPointer<FTLEMouseInteractorStyle> style = 
	  vtkSmartPointer<FTLEMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle( style );

	renderWindow->Render();
	renderWindowInteractor->Start();
}


void condMode(int M = 50){
	// create a VTK actor for all bodies
	std::vector<body_t> bodies = mysystem.getBodies();
	std::vector<vtkActorPtr> actorBodies;
	for(body_t b: bodies){
		vtkActorPtr actorTmp = createCircleActor(b.radius, b.position_curr);
		actorTmp->GetProperty()->EdgeVisibilityOn();
		actorBodies.push_back(actorTmp);
	}
	
	// create border as a line actor
	Vector2d xlimit = mysystem.getXlimits();
	Vector2d ylimit = mysystem.getYlimits();
	MatrixXd m1(2, 5);
	m1 << xlimit.x(), xlimit.y(), xlimit.y(), xlimit.x(), xlimit.x(), ylimit.x(), ylimit.x(), ylimit.y(), ylimit.y(), ylimit.x();
	vtkActorPtr borderActor = createLineActor(m1, Vector3d(1.,1.,1.));

	// color whole area, programmed should be called with ./N_body_system dense v_0x v_0y
	std::vector<vtkActorPtr> areaActors;
	Vector2d xlimits = mysystem.getXlimits();
	Vector2d ylimits = mysystem.getYlimits();
	int N = 10; // divide that area to 10x10 cells
	double dx = (xlimits.y() - xlimits.x())/N;
	double dy = (ylimits.y() - ylimits.x())/N;
	#pragma omp parallel for
	for(int i = 0; i < N; ++i){
		for(int j = 0; j < N; ++j){
			vtkActorPtr areaActorTmp = colorADot(Vector2d(xlimits.x()+dx*i,xlimits.x()+dx*(i+1)),
				Vector2d(ylimits.x()+dy*j,ylimits.x()+dy*(j+1)), M);
			areaActors.push_back(areaActorTmp);
		}
	}
	
	vtkSmartPointer<vtkRenderer> renderer =
	    vtkSmartPointer<vtkRenderer>::New();
	for(vtkActorPtr a: actorBodies)
		renderer->AddActor(a);
	renderer->AddActor(borderActor);
	for(vtkActorPtr a: areaActors)
		renderer->AddActor(a);
	renderer->SetBackground(.1,.1,.1); // Background color dark grey
	 
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	    vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1200,1200);
	 

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	    vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = 
	  vtkSmartPointer<vtkWorldPointPicker>::New();
	renderWindowInteractor->SetPicker(worldPointPicker);

	vtkSmartPointer<TrajectoryMouseInteractorStyle> style = 
	  vtkSmartPointer<TrajectoryMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle( style );

	renderWindow->Render();
	renderWindowInteractor->Start();
}

// char* c should be "w" or "a" depending on if you want to overwrite or 
// append to the file eigenanalysis.txt
void writeJacobianPropertiesToFile(const particle& p, char c = 'w'){
	const int dim = 4;  // concrete since we have 4 dimensionsal state vector
	const char* mode = &c;
	Matrix<double, dim, dim> Jac = mysystem.JacobianMat(p);
	EigenSolver<MatrixXd> solver;
	solver.compute(Jac);

	// ofstream myfile;
	FILE * myfile;
	myfile = fopen("../eigenanalysis.txt", mode);
	fprintf(myfile, "# position and velocity of particle and dimension of problem\n");
	fprintf(myfile, "%.8f  %.8f  %.8f  %.8f %d\n", p.pos.x(), p.pos.y(), p.vel.x(), p.vel.y(), dim);
	for(int i = 0; i < dim; ++i)
		fprintf(myfile, "%.8f %.8f\n", solver.eigenvalues()[i].real(), solver.eigenvalues()[i].imag());

	for(int i = 0; i < dim; ++i){
		for(int j = 0; j < dim; ++j)
			fprintf(myfile, "%.8f %.8f\n", solver.eigenvectors().col(i)[j].real(), solver.eigenvectors().col(i)[j].imag());
	}

	fclose(myfile);
}
