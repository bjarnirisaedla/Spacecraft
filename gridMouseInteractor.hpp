// Define interaction style for dot mode
class GridMouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static GridMouseInteractorStyle* New();
    vtkTypeMacro(GridMouseInteractorStyle, vtkInteractorStyleTrackballActor);
 
       virtual void OnLeftButtonDown() 
       {
    		leftButtonIsDown = true;
    		Vector3d hitColor(0,0,0);
    		if(textActor){
    			textActor->GetTextProperty()->SetFontSize (18);
    			textActor->SetPosition2 (10.0, 40.0);
    			this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor2D(textActor);
    			textActor->GetTextProperty()->SetColor (1.,1.,1.);
    		}
    		if(actorDots.empty()){
    			int index = 0;
    			for(int i=0; i < numberOfDots; i++){
    				for(int j=0; j < numberOfDots; j++){
    					// mysystem.flowmap(Vector2d(xdots(i), ydots(j)), Vector2d(0,0), hitColor);
    					actorDots.push_back(createCircleActor(0.01, Vector2d(xdots(i), ydots(j)), hitColor));
    					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actorDots.back());
    					indexMat(i,j) = index++;
    				}
    			}
    		}

    		// Recolor dots
    		#pragma omp parallel for private(hitColor)
    		for(int i=0; i < numberOfDots; i++){
    			for(int j=0; j < numberOfDots; j++){
    				mysystem.flowmap(Vector2d(xdots(i), ydots(j)), Vector2d(0,0), hitColor);
    				actorDots[indexMat(i,j)]->GetProperty()->SetColor(hitColor.x(),hitColor.y(),hitColor.z());
    			}
    		}

	       	this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
	                           this->Interactor->GetEventPosition()[1], 
	                           0,  // always zero.
	                           this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	       	double picked[3];
	       	this->Interactor->GetPicker()->GetPickPosition(picked);

	       	position = Vector2d(picked[0],picked[1]);

	       	//setup the text and add it to the window
			std::string s = "Position: " + std::to_string(position.x()) + ", " + std::to_string(position.y()) + 
				"\n" + "Velocity: 0.0, 0.0" ;
	       	textActor->SetInput(s.c_str());
	       	
	       	
	       	// Forward events
	       	vtkInteractorStyleTrackballActor::OnLeftButtonDown();
       }

       virtual void OnLeftButtonUp() 
       {
    		leftButtonIsDown = false;
    		// Remove unwanted old actors
    		if(arrowActor)
    				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(arrowActor);
			if(headOfTrajectory)
					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(headOfTrajectory);
			if(actorTrajectory)
					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorTrajectory);
    		arrowActor = nullptr;
    		headOfTrajectory = nullptr;
    		actorTrajectory = nullptr;

			this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
		                    this->Interactor->GetEventPosition()[1], 
		                    0,  // always zero.
		                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);

    		// // plot a new trajectory
    		Vector3d hitColor;
            Vector2d vel = calcVelocity(Vector2d(picked[0], picked[1]));
    		traj_pos = mysystem.integrator<maxIteration>(position, vel, hitColor);
    		actorTrajectory = createLineActor(hitColor);
    		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actorTrajectory);
    		this->Interactor->GetRenderWindow()->Render();

    		// Recolor dots
			Vector2d velocity = calcVelocity(Vector2d(picked[0], picked[1]));
			#pragma omp parallel for private(hitColor)
			for(int i=0; i < numberOfDots; i++){
				for(int j=0; j < numberOfDots; j++){
					mysystem.flowmap(Vector2d(xdots(i), ydots(j)), velocity, hitColor);
					actorDots[indexMat(i,j)]->GetProperty()->SetColor(hitColor.x(),hitColor.y(),hitColor.z());
				}
			}

       		// Forward events
       		vtkInteractorStyleTrackballActor::OnLeftButtonUp();
       }

        virtual void OnMouseMove() 
        {
       		if(leftButtonIsDown){
				this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
			                       this->Interactor->GetEventPosition()[1], 
			                       0,  // always zero.
			                       this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
				double picked[3];
				this->Interactor->GetPicker()->GetPickPosition(picked);

				// remove previous and add a green line to indicate velocity and direction
				if(arrowActor)
						this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(arrowActor);
			    Eigen::Matrix<double, 2, 2> m1;
				m1.col(0) = position;
				m1.col(1) = Vector2d(picked[0],picked[1]);
				arrowActor = createLineActor(m1, Vector3d(0.,1.,0.));
				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(arrowActor);

				// // remove previous and add a predicting trajectory
				if(headOfTrajectory)
			   		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(headOfTrajectory);
				Vector2d velocity = calcVelocity(Vector2d(picked[0],picked[1]));
				Vector3d hitColor;
				Eigen::Matrix<double, 2, predictIteration> m2;
				m2 = mysystem.integrator<predictIteration>(position, velocity, hitColor);
				headOfTrajectory = createLineActor(m2, hitColor);
				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(headOfTrajectory);

				// Recolor dots
				#pragma omp parallel for private(hitColor)
				for(int i=0; i < numberOfDots; i++){
					for(int j=0; j < numberOfDots; j++){
						mysystem.flowmap(Vector2d(xdots(i), ydots(j)), velocity, hitColor);
						actorDots[indexMat(i,j)]->GetProperty()->SetColor(hitColor.x(),hitColor.y(),hitColor.z());
					}
				}

				// add text to update velocity input
				std::string s = "Position: " + std::to_string(position.x()) + ", " + std::to_string(position.y()) + 
					"\n" + "Velocity: " + std::to_string(velocity.x()) + ", " + std::to_string(velocity.y());
				textActor->SetInput(s.c_str());

				this->Interactor->GetRenderWindow()->Render();

       		}
	        // Forward events
	        // vtkInteractorStyleTrackballActor::OnMouseMove();
        }
    private:
        bool leftButtonIsDown = false;
        Vector2d position = Vector2d(0.0,0.0);
        vtkActorPtr arrowActor = nullptr;
        vtkActorPtr headOfTrajectory = nullptr;
        vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();

        const int numberOfDots = 50; // remember to change indexMat too. some stupid reason
        // Eigen::Matrix<int, 10, 10> indexMat;
        Eigen::MatrixXi indexMat = MatrixXi::Zero(numberOfDots, numberOfDots);
        Vector2d xlimit = mysystem.getXlimits();
        Vector2d ylimit = mysystem.getYlimits();
        Eigen::VectorXd xdots = Eigen::VectorXd::LinSpaced(numberOfDots+2,xlimit.x(),xlimit.y()).segment(1,numberOfDots);
        Eigen::VectorXd ydots = Eigen::VectorXd::LinSpaced(numberOfDots+2,ylimit.x(),ylimit.y()).segment(1,numberOfDots);
        std::vector<vtkActorPtr> actorDots;

        Vector2d calcVelocity(Vector2d picked) const {
        	return (picked - position) * 1.5;
        }
};
vtkStandardNewMacro(GridMouseInteractorStyle);