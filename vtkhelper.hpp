#ifndef INCLUDE_VTK_HELPER
#define INCLUDE_VTK_HELPER

#include <vtkRegularPolygonSource.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkObjectFactory.h>
#include <vtkWorldPointPicker.h>
#include <vtkRendererCollection.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDelaunay2D.h>
#include <vtkLookupTable.h>
#include <vtkIndent.h>

#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkPNGWriter.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageCast.h>

#include <stdio.h>
#include "mysystem.hpp"

typedef vtkSmartPointer<vtkActor> vtkActorPtr;

// Global variables
MySystem mysystem;
vtkActorPtr actorTrajectory;
Vector2d InputVelocity = Vector2d(0,0);



vtkActorPtr createCircleActor(double radius, Vector2d center, Vector3d color = Vector3d(1.,1.,1.)){
	// Create a circle
	vtkSmartPointer<vtkRegularPolygonSource> polygonSource =
	  vtkSmartPointer<vtkRegularPolygonSource>::New();
	
	polygonSource->SetNumberOfSides(50);
	polygonSource->SetRadius(radius);
	polygonSource->SetCenter(center.x(), center.y(), 0);

	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper =
	  vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(polygonSource->GetOutputPort());;
	
	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(color.x(), color.y(), color.z());

	return actor;
}


// creates an actor that draws a line
vtkSmartPointer<vtkActor> createLineActor(Vector3d color)
{
	// trim the trajectory
	int endCol = traj_pos.cols();
	for(int i = 0; i < traj_pos.cols(); i++){
		if(traj_pos.col(i) == Vector2d(0,0)){
			endCol = i;
			break;
		}
	}

	// create polyline vertices
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < endCol; i++)
		points->InsertNextPoint(traj_pos(0,i), traj_pos(1,i), 0);


	// create polyline indices
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);

	// Create a cell array to store the lines in and add the lines to it
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(polyLine);

	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);	// Add the points to the dataset
	polyData->SetLines(cells);			// Add the lines to the dataset

	// Setup actor and mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(color.x(), color.y(), color.z());
	actor->GetProperty()->SetLineWidth(2);
	return actor;
}

// creates an actor that draws a line
vtkSmartPointer<vtkActor> createLineActor(const MatrixXd& line, Vector3d color)
{
	if(line.rows() != 2)
		std::cout << "createLineActor: Matrix should only have two rows" << std::endl;

	// trim the trajectory
	int endCol = line.cols();
	for(int i = 0; i < line.cols(); i++){
		if(line.col(i) == Vector2d(0,0)){
			endCol = i;
			break;
		}
	}

	// create polyline vertices
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < endCol; i++)
		points->InsertNextPoint(line(0,i), line(1,i), 0);


	// create polyline indices
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);

	// Create a cell array to store the lines in and add the lines to it
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(polyLine);

	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);	// Add the points to the dataset
	polyData->SetLines(cells);			// Add the lines to the dataset

	// Setup actor and mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(color.x(), color.y(), color.z()); // yellow
	actor->GetProperty()->SetLineWidth(2);
	return actor;
}

// using points to color most pixels
// x and y represent an interval [x_1, x_2]
vtkActorPtr colorArea(const Vector2d& x, const Vector2d& y, Vector2d velocity, int N){
	Vector3d color;
	VectorXd xlin = VectorXd::LinSpaced(N, x.x(), x.y());
	VectorXd ylin = VectorXd::LinSpaced(N, y.x(), y.y());

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for(int i = 0; i < N; i++){
		for(int j = 0; j < N; j++){
			points->InsertNextPoint(xlin(i), ylin(j), 0);
		}
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->AddInputData(pointsPolydata);
	vertexGlyphFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexGlyphFilter->GetOutput());

	// setup of colors
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	for(int i = 0; i < N; i++){
		for(int j = 0; j < N; j++){
			mysystem.flowmap(Vector2d(xlin(i), ylin(j)), velocity, color);
			unsigned char r = (int) 255*color.x();
			unsigned char g = (int) 255*color.y();
			unsigned char b = (int) 255*color.z();
			unsigned char colT[3] = {r, g, b};
			colors->InsertNextTypedTuple(colT);
		}
	}
	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(1);

	return actor;
}

// using points to color most pixels
// x and y represent an interval [x_1, x_2]
vtkActorPtr colorADot(const Vector2d& x, const Vector2d& y, int N){
	Vector3d color;
	VectorXd xlin = VectorXd::LinSpaced(N, x.x(), x.y());
	VectorXd ylin = VectorXd::LinSpaced(N, y.x(), y.y());

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for(int i = 0; i < N; i++){
		for(int j = 0; j < N; j++){
			points->InsertNextPoint(xlin(i), ylin(j), 0);
		}
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->AddInputData(pointsPolydata);
	vertexGlyphFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexGlyphFilter->GetOutput());

	// setup of colors
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	for(int i = 0; i < N; i++){
		for(int j = 0; j < N; j++){
			if(mysystem.cond_One(Vector2d(xlin(i), ylin(j))) == 1){
				unsigned char colT[3] = {255, 0, 255};
				colors->InsertNextTypedTuple(colT);
			}
			else if(mysystem.cond_One(Vector2d(xlin(i), ylin(j))) == 2){
				unsigned char colT[3] = {0, 255, 0};
				colors->InsertNextTypedTuple(colT);
			}
			else{
				unsigned char colT[3] = {255, 255, 255};
				colors->InsertNextTypedTuple(colT);
			}
		}
	}
	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(1);

	return actor;
}

vtkActorPtr createJacobiConstActor(int GridSize = 100)
{
	// Create a grid of points (height/terrian map)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> pointsFlat = vtkSmartPointer<vtkPoints>::New();
	 
	Vector2d xlim = mysystem.getXlimits();
	Vector2d ylim = mysystem.getYlimits();
	VectorXd xx = VectorXd::LinSpaced(GridSize, xlim.x(), xlim.y());
	VectorXd yy = VectorXd::LinSpaced(GridSize, ylim.x(), ylim.y());
	double zz;
	for(int i = 0; i < GridSize; ++i){
	    for(int j = 0; j < GridSize; ++j){
	    	zz = mysystem.getJacobiConstantAt(Vector2d(xx[i], yy[j]), Vector2d(0.0,0.0));
	      	points->InsertNextPoint(xx[i], yy[j], zz);
	      	pointsFlat->InsertNextPoint(xx[i], yy[j], 0.0);
	    }
	}
	 
	// Add the grid points to a polydata object
	vtkSmartPointer<vtkPolyData> inputPolyData = vtkSmartPointer<vtkPolyData>::New();
	inputPolyData->SetPoints(points);
	vtkSmartPointer<vtkPolyData> inputPolyDataFlat = vtkSmartPointer<vtkPolyData>::New();
	inputPolyDataFlat->SetPoints(pointsFlat);

	// Triangulate the grid points
	vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunay->SetInputData(inputPolyData);
	delaunay->Update();
	vtkPolyData* outputPolyData = delaunay->GetOutput();
	vtkSmartPointer<vtkDelaunay2D> delaunayFlat = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunayFlat->SetInputData(inputPolyDataFlat);
	delaunayFlat->Update();
	vtkPolyData* outputPolyDataFlat = delaunayFlat->GetOutput();
	 
	double bounds[6];
	outputPolyData->GetBounds(bounds);
	  
	// Find min and max z
	double minz = bounds[4];
	double maxz = bounds[5];
	
	 
	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(minz, maxz);
	colorLookupTable->Build();
	 
	// Generate the colors for each point based on the color map
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	 
	for(int i = 0; i < outputPolyData->GetNumberOfPoints(); ++i){
	    double p[3];
	    outputPolyData->GetPoint(i,p);
	 
	    double dcolor[3];
	    colorLookupTable->GetColor(p[2], dcolor);
	    unsigned char color[3];
	    for(unsigned int j = 0; j < 3; j++){
	    	color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
	    }
	 
	    colors->InsertNextTypedTuple(color);
	}
	 
	outputPolyDataFlat->GetPointData()->SetScalars(colors);
	 
	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(outputPolyDataFlat);
	 
	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	 
	return actor;
}

vtkActorPtr createBackwardsScalarActor(int N = 100)
{
	// Create a grid of points (height/terrian map)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> pointsFlat = vtkSmartPointer<vtkPoints>::New();
	 
	Vector2d xlim = mysystem.getXlimits();
	Vector2d ylim = mysystem.getYlimits();
	VectorXd xx = VectorXd::LinSpaced(N, xlim.x(), xlim.y());
	VectorXd yy = VectorXd::LinSpaced(N, ylim.x(), ylim.y());
	double zz;
	Vector2d vel(0.0,0.0), pos(0.0,0.0);
	Vector3d dummyColor(0,0,0);
	// #pragma omp parallel for private(pos, vel)
	for(int i = 0; i < N; ++i){
	    for(int j = 0; j < N; ++j){
	    	pos = Vector2d(xx[i], yy[j]);
	    	mysystem.flowmapPosAndVel(pos, vel, dummyColor, false);								// Forwards
	    	mysystem.flowmapPosAndVel(pos, vel, dummyColor, -mysystem.getTimestep(), false);	// Backwards
	    	// Estimate the error as the length from the starting value in position space
	    	zz = log((pos - Vector2d(xx[i], yy[j])).norm()+1); // +1 so it's positive
	      	points->InsertNextPoint(xx[i], yy[j], zz);
	      	pointsFlat->InsertNextPoint(xx[i], yy[j], 0.0);
	      	vel = Vector2d(0.0,0.0);
	    }
	}
	 
	// Add the grid points to a polydata object
	vtkSmartPointer<vtkPolyData> inputPolyData = vtkSmartPointer<vtkPolyData>::New();
	inputPolyData->SetPoints(points);
	vtkSmartPointer<vtkPolyData> inputPolyDataFlat = vtkSmartPointer<vtkPolyData>::New();
	inputPolyDataFlat->SetPoints(pointsFlat);

	// Triangulate the grid points
	vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunay->SetInputData(inputPolyData);
	delaunay->Update();
	vtkPolyData* outputPolyData = delaunay->GetOutput();
	vtkSmartPointer<vtkDelaunay2D> delaunayFlat = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunayFlat->SetInputData(inputPolyDataFlat);
	delaunayFlat->Update();
	vtkPolyData* outputPolyDataFlat = delaunayFlat->GetOutput();
	 
	double bounds[6];
	outputPolyData->GetBounds(bounds);
	  
	// Find min and max z
	double len = xlim.y() - xlim.x();
	double minz = bounds[4];
	// double maxz = (bounds[5] < log(len)+1) ? bounds[5] : log(len)+1;
	double maxz = bounds[5];
	std::cout << "minz: " << minz << " maxz: " << maxz << std::endl;
	
	 
	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(minz, maxz);
	colorLookupTable->SetNumberOfTableValues(N*N);
	colorLookupTable->Build();
	std::cout << "Number of colors: " << colorLookupTable->GetNumberOfColors() << std::endl;
	 
	// Generate the colors for each point based on the color map
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	 
	for(int i = 0; i < outputPolyData->GetNumberOfPoints(); ++i){
	    double p[3];
	    outputPolyData->GetPoint(i,p);
	 
	    double dcolor[3];
	    colorLookupTable->GetColor(p[2], dcolor);
	    unsigned char color[3];
	    for(unsigned int j = 0; j < 3; j++){
	    	color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
	    }
	 
	    colors->InsertNextTypedTuple(color);
	}
	 
	outputPolyDataFlat->GetPointData()->SetScalars(colors);
	 
	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(outputPolyDataFlat);
	 
	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	 
	return actor;
}

vtkActorPtr createBackwardsScalarActorDots(int N = 300)
{
	// Create a grid of points (height/terrian map)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	 
	Vector2d xlim = mysystem.getXlimits();
	Vector2d ylim = mysystem.getYlimits();
	VectorXd xx = VectorXd::LinSpaced(N, xlim.x(), xlim.y());
	VectorXd yy = VectorXd::LinSpaced(N, ylim.x(), ylim.y());
	double zz;
	Vector2d vel(0.0,0.0), pos(0.0,0.0);
	Vector3d dummyColor(0,0,0);
	#pragma omp parallel for private(pos, vel, zz)
	for(int i = 0; i < N; ++i){
	    for(int j = 0; j < N; ++j){
	    	pos = Vector2d(xx[i], yy[j]);
	      	vel.setZero();
	    	mysystem.flowmapPosAndVel(pos, vel, dummyColor, false);								// Forwards
	    	mysystem.flowmapPosAndVel(pos, vel, dummyColor, -mysystem.getTimestep(), false);	// Backwards
	    	// Estimate the error as the length from the starting value in position space
	    	zz = log((pos - Vector2d(xx[i], yy[j])).norm()+1); // +1 so it's positive
	      	points->InsertNextPoint(xx[i], yy[j], zz);
	    }
	}
	 
	// Add the grid points to a polydata object
	vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->AddInputData(pointsPolydata);
	vertexGlyphFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexGlyphFilter->GetOutput());

	double bounds[6];
	polydata->GetBounds(bounds);

	double len = xlim.y() - xlim.x();
	double minz = bounds[4];
	double maxz = (bounds[5] < log(len)+1) ? bounds[5] : log(len)+1;

	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(minz, maxz);
	colorLookupTable->SetNumberOfTableValues(N*N);
	colorLookupTable->Build();
	 
	// Generate the colors for each point based on the color map
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	 
	for(int i = 0; i < polydata->GetNumberOfPoints(); ++i){
	    double p[3];
	    polydata->GetPoint(i,p);
	 
	    double dcolor[3];
	    colorLookupTable->GetColor(p[2], dcolor);

	    unsigned char color[3];
	    for(unsigned int j = 0; j < 3; j++){
	    	color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
	    }

	    colors->InsertNextTypedTuple(color);
	}
	 
	// Reset the z-values of polydata to zero
	vtkSmartPointer<vtkPoints> polydataPoints = vtkSmartPointer<vtkPoints>::New();
	polydataPoints = polydata->GetPoints();
	for (int i = 0; i < polydataPoints->GetNumberOfPoints(); ++i)
	{
		double p[3];
		polydataPoints->GetPoint(i,p);
		p[2] = 0.0;
		polydataPoints->SetPoint(i,p);
	}

	polydata->SetPoints(polydataPoints);
	polydata->GetPointData()->SetScalars(colors);
	 
	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);
	 
	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	 
	return actor;
}

vtkActorPtr createFTLEScalarActorDots(const int N = 3)
{
	// Create a grid of points (height/terrian map)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	 
	Vector2d xlim = mysystem.getXlimits();
	Vector2d ylim = mysystem.getYlimits();
	VectorXd xx = VectorXd::LinSpaced(N, xlim.x(), xlim.y());
	VectorXd yy = VectorXd::LinSpaced(N, ylim.x(), ylim.y());
	double zz;

	// Build the grid and store final position values
	MatrixXd finPosX(N,N); 
	MatrixXd finPosY(N,N);
	Vector2d vel(0.0,0.0), pos(0.0,0.0);
	Vector3d dummyColor(0,0,0);
	// #pragma omp parallel for private(pos, vel)
	for (int i = 0; i < N; ++i){
		for (int j = 0; j < N; ++j){
	    	pos = Vector2d(xx[i], yy[j]);
			mysystem.flowmapPosAndVel(pos, vel, dummyColor, false);
			finPosX(i,j) = pos.x();
			finPosY(i,j) = pos.y();
			pos.setZero();
			vel.setZero();
		}
	}

	// Calculate the FTLE value
	const double dx = (xlim.y() - xlim.x())/N;
	const double dy = (ylim.y() - ylim.x())/N;
	MatrixXd phi(2,2);
	MatrixXd flowMat(2,2);
	EigenSolver<MatrixXd> MatSolver;
	// #pragma omp parallel for private(zz, phi, flowMat, MatSolver)
	for(int i = 1; i < N-1; ++i){
	    for(int j = 1; j < N-1; ++j){
	    	phi(0,0) = (finPosX(i+1,j)-finPosX(i-1,j))/(2.0*dx);
	    	phi(1,0) = (finPosX(i,j+1)-finPosX(i,j-1))/(2.0*dy);

	    	phi(0,1) = (finPosY(i+1,j)-finPosY(i-1,j))/(2.0*dx);
	    	phi(1,1) = (finPosY(i,j+1)-finPosY(i,j-1))/(2.0*dy);
	    	// WRITE OUT MATRIX phi
	    	std::cout << "Phi matrix" << std::endl;
	    	for(int i = 0; i < 2; ++i){
	    		for(int j = 0; j < 2; ++j)
	    			std::cout << phi(i,j) << "\t";
	    		std::cout << std::endl;
	    	}
	    	std::cout 	<< std::endl;

	    	flowMat = phi.transpose()*phi;

	    	// WRITE OUT MATRIX flowmat
	    	std::cout << "Flowmat matrix" << std::endl;
	    	for(int i = 0; i < 2; ++i){
	    		for(int j = 0; j < 2; ++j)
	    			std::cout << flowMat(i,j) << "\t";
	    		std::cout << std::endl;
	    	}
	    	std::cout 	<< std::endl;

	    	MatSolver.compute(flowMat);
	    	std::cout << "Eigenvalues" << std::endl;
	    	std::cout << MatSolver.eigenvalues() << std::endl;

	    	zz = 1.0/maxIteration * log(std::sqrt(MatSolver.eigenvalues().real().maxCoeff()));
	    	std::cout << "Final value" << std::endl;
	    	std::cout << zz << std::endl;
	      	points->InsertNextPoint(xx[i], yy[j], zz);
	    }
	}
	 
	// Add the grid points to a polydata object
	vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->AddInputData(pointsPolydata);
	vertexGlyphFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexGlyphFilter->GetOutput());

	double bounds[6];
	polydata->GetBounds(bounds);

	double minz = bounds[4];
	// double maxz = (bounds[5] < log(xlim.norm())+1) ? bounds[5] : log(xlim.norm()+1);
	double maxz = bounds[5];
	// std::cout << "minz: " << minz << " maxz: " << maxz << " bounds[5]: " << bounds[5] << std::endl; 

	// Create the color map
	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetTableRange(minz, maxz);
	colorLookupTable->SetNumberOfTableValues(N*N);
	colorLookupTable->Build();
	 
	// Generate the colors for each point based on the color map
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	 
	for(int i = 0; i < polydata->GetNumberOfPoints(); ++i){
	    double p[3];
	    polydata->GetPoint(i,p);
	 
	    double dcolor[3];
	    colorLookupTable->GetColor(p[2], dcolor);

	    unsigned char color[3];
	    for(unsigned int j = 0; j < 3; j++){
	    	color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
	    }

	    colors->InsertNextTypedTuple(color);
	}
	 
	// Reset the z-values of polydata to zero
	vtkSmartPointer<vtkPoints> polydataPoints = vtkSmartPointer<vtkPoints>::New();
	polydataPoints = polydata->GetPoints();
	for (int i = 0; i < polydataPoints->GetNumberOfPoints(); ++i)
	{
		double p[3];
		polydataPoints->GetPoint(i,p);
		p[2] = 0.0;
		polydataPoints->SetPoint(i,p);
	}

	polydata->SetPoints(polydataPoints);
	polydata->GetPointData()->SetScalars(colors);
	 
	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);
	 
	vtkActorPtr actor = vtkActorPtr::New();
	actor->SetMapper(mapper);
	 
	return actor;
}

void getColorBlueToRed(const double& value, double &r, double &g, double &b, double minVal, double maxVal)
{
	double t = std::min(std::max(0.0, ((value - minVal) /
		(maxVal - minVal))), 1.0);
	Vector3d color =
		(((Vector3d(5.0048, 7.4158, 6.1246)*t +
			Vector3d(-8.0915, -15.9415, -16.2287))*t +
			Vector3d(1.1657, 7.4696, 11.9910))*t +
			Vector3d(1.4380, 1.2767, -1.4886))*t +
			Vector3d(0.6639, -0.0013, 0.1685);
	r = std::min(std::max(0.0, color.x()), 1.0);
	g = std::min(std::max(0.0, color.y()), 1.0);
	b = std::min(std::max(0.0, color.z()), 1.0);
}

void getColorHeatmap(const double& value, double &r, double &g, double &b, double minVal, double maxVal)
{
	double t = std::min(std::max(0.0, ((value - minVal) /
		(maxVal - minVal))), 1.0);
	Vector3d color =
		((Vector3d(-1.0411, 1.7442, 0.9397)*t +
			Vector3d(0.4149, -2.7388, -0.2565))*t +
			Vector3d(0.0162, 0.1454, -1.5570))*t +
			Vector3d(0.9949, 0.9971, 0.9161);
	r = std::min(std::max(0.0, color.x()), 1.0);
	g = std::min(std::max(0.0, color.y()), 1.0);
	b = std::min(std::max(0.0, color.z()), 1.0);
}

void createFTLEImage(const int N = 1000)
{
	std::string outputFilename = "../figures/FTLE.png";
	 
	Vector2d xlim = mysystem.getXlimits();
	Vector2d ylim = mysystem.getYlimits();
	VectorXd xx = VectorXd::LinSpaced(N, xlim.x(), xlim.y());
	VectorXd yy = VectorXd::LinSpaced(N, ylim.x(), ylim.y());
	MatrixXd ZZ = MatrixXd::Zero(N,N);
	double zz;
	double minz = std::numeric_limits<double>::max();
	double maxz = 0.0;
	double tau = maxIteration * mysystem.getTimestep() * mysystem.getIterationPerTimestep();

	// Build the grid and store final position values
	MatrixXd finPosX(N,N); 
	MatrixXd finPosY(N,N);
	Vector2d vel(0.0,0.0), pos(0.0,0.0);
	Vector3d dummyColor(0,0,0);
#ifdef NDEBUG
	#pragma omp parallel for private(pos, vel)
#endif
	for(int i = 0; i < N; ++i){
		for(int j = 0; j < N; ++j){
	    	pos = Vector2d(xx[i], yy[j]);
			mysystem.flowmapPosAndVel(pos, vel, dummyColor, false);
			finPosX(i,j) = pos.x();
			finPosY(i,j) = pos.y();
			pos.setZero();
			vel.setZero();
		}
	}

	// Calculate the FTLE value
	const double dx = (xlim.y() - xlim.x())/N;
	const double dy = (ylim.y() - ylim.x())/N;
	Matrix<double,2,2> phi;
	Matrix<double,2,2> flowMat;
	EigenSolver<MatrixXd> MatSolver;
	#pragma omp parallel for private(zz, phi, flowMat, MatSolver)
	for(int k = 1; k < N-1; ++k){
	    for(int l = 1; l < N-1; ++l){
	    	phi(0,0) = (finPosX(k+1,l)-finPosX(k-1,l))/(2.0*dx);
	    	phi(1,0) = (finPosX(k,l+1)-finPosX(k,l-1))/(2.0*dy);
	    	phi(0,1) = (finPosY(k+1,l)-finPosY(k-1,l))/(2.0*dx);
	    	phi(1,1) = (finPosY(k,l+1)-finPosY(k,l-1))/(2.0*dy);

	    	flowMat = phi.transpose()*phi;
	    	MatSolver.compute(flowMat);

    		zz = 1.0/tau * log(std::sqrt(MatSolver.eigenvalues().real().maxCoeff()));
	    	ZZ(k,l) = zz;
	    	if(zz > maxz) maxz = zz;
	    	if(zz < minz) minz = zz;
	    }
	}
	
	// Create an image data
	vtkSmartPointer<vtkImageData> imageData = 
	  vtkSmartPointer<vtkImageData>::New();

	// Specify the size of the image data
	imageData->SetDimensions(N,N,1);
	imageData->AllocateScalars(VTK_UNSIGNED_CHAR,3);
	int* dims = imageData->GetDimensions();
	for (int i = 0; i < N; ++i){
		for (int j = 0; j < N; ++j){
			unsigned char* pixel = static_cast<unsigned char*>(imageData->GetScalarPointer(i,j,0));
			double r,g,b;
			getColorBlueToRed(ZZ(i,j), r, g, b, minz, maxz);
			double dcolor[3] = {r,g,b};
			// colorLookupTable->GetColor(ZZ(i,j), dcolor);
			for(unsigned int j = 0; j < 3; j++){
				pixel[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
			}
		}
	}

	 vtkSmartPointer<vtkImageCanvasSource2D> canvas = vtkSmartPointer<vtkImageCanvasSource2D>::New();
	 canvas->InitializeCanvasVolume( imageData );
	 canvas->Update();

	 vtkSmartPointer<vtkImageCast> castFilter =
	   vtkSmartPointer<vtkImageCast>::New();
	 castFilter->SetOutputScalarTypeToUnsignedChar ();
	 castFilter->SetInputConnection(canvas->GetOutputPort());
	 castFilter->Update();

	 vtkSmartPointer<vtkPNGWriter> writer =
	   vtkSmartPointer<vtkPNGWriter>::New();
	 writer->SetFileName( outputFilename.c_str() );
	 writer->SetInputConnection( castFilter->GetOutputPort() );
	 writer->Write();
}

void createBackwardsImage(const int N = 1000)
{
	std::string outputFilename = "../figures/backwards.png";
	 
	Vector2d xlim = mysystem.getXlimits();
	Vector2d ylim = mysystem.getYlimits();
	VectorXd xx = VectorXd::LinSpaced(N, xlim.x(), xlim.y());
	VectorXd yy = VectorXd::LinSpaced(N, ylim.x(), ylim.y());
	MatrixXd ZZ = MatrixXd::Zero(N,N);
	double zz;
	double minz = std::numeric_limits<double>::max();
	double maxz = 0.0;
	Vector2d vel(0.0,0.0), pos(0.0,0.0);
	Vector3d dummyColor(0,0,0);
	#pragma omp parallel for private(pos, vel, zz)
	for(int i = 0; i < N; ++i){
	    for(int j = 0; j < N; ++j){
	    	pos = Vector2d(xx[i], yy[j]);
	      	vel.setZero();
	    	mysystem.flowmapPosAndVel(pos, vel, dummyColor, false);								// Forwards
	    	mysystem.flowmapPosAndVel(pos, vel, dummyColor, -mysystem.getTimestep(), false);	// Backwards
	    	// Estimate the error as the length from the starting value in position space
	    	zz = log((pos - Vector2d(xx[i], yy[j])).norm()+1); // +1 so it's positive
	    	ZZ(i,j) = zz;
	    	if(zz > maxz) maxz = zz;
	    	if(zz < minz) minz = zz;
	    }
	}
	
	// Create an image data
	vtkSmartPointer<vtkImageData> imageData = 
	  vtkSmartPointer<vtkImageData>::New();

	// Specify the size of the image data
	imageData->SetDimensions(N,N,1);
	imageData->AllocateScalars(VTK_UNSIGNED_CHAR,3);
	int* dims = imageData->GetDimensions();
	for (int i = 0; i < N; ++i){
		for (int j = 0; j < N; ++j){
			unsigned char* pixel = static_cast<unsigned char*>(imageData->GetScalarPointer(i,j,0));
			double r,g,b;
			getColorHeatmap(ZZ(i,j), r, g, b, minz, maxz);
			double dcolor[3] = {r,g,b};
			for(unsigned int j = 0; j < 3; j++){
				pixel[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
			}
		}
	}

	 vtkSmartPointer<vtkImageCanvasSource2D> canvas = vtkSmartPointer<vtkImageCanvasSource2D>::New();
	 canvas->InitializeCanvasVolume( imageData );
	 canvas->Update();

	 vtkSmartPointer<vtkImageCast> castFilter =
	   vtkSmartPointer<vtkImageCast>::New();
	 castFilter->SetOutputScalarTypeToUnsignedChar ();
	 castFilter->SetInputConnection(canvas->GetOutputPort());
	 castFilter->Update();

	 vtkSmartPointer<vtkPNGWriter> writer =
	   vtkSmartPointer<vtkPNGWriter>::New();
	 writer->SetFileName( outputFilename.c_str() );
	 writer->SetInputConnection( castFilter->GetOutputPort() );
	 writer->Write();
}



#endif // !INCLUDE_VTK_HELPER