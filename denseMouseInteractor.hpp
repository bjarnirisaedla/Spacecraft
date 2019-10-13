// Define interaction style for dense area mode
class DenseMouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static DenseMouseInteractorStyle* New();
    vtkTypeMacro(DenseMouseInteractorStyle, vtkInteractorStyleTrackballActor);
 
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

	       	this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
	                           this->Interactor->GetEventPosition()[1], 
	                           0,  // always zero.
	                           this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	       	double picked[3];
	       	this->Interactor->GetPicker()->GetPickPosition(picked);

	       	position = Vector2d(picked[0],picked[1]);
	       	addTraj(position);

            //setup the text and add it to the window
            std::string s = "Position: " + std::to_string(position.x()) + ", " + std::to_string(position.y()) + 
                "\n" + "Velocity: " +  std::to_string(InputVelocity.x()) + ", " + std::to_string(InputVelocity.y());
            textActor->SetInput(s.c_str());
            
            
            this->Interactor->GetRenderWindow()->Render();
	       	// Forward events
	       	vtkInteractorStyleTrackballActor::OnLeftButtonDown();
       }

       virtual void OnLeftButtonUp() 
       {
    		leftButtonIsDown = false;
    		// Remove unwanted old actors
			if(actorTrajectory)
					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorTrajectory);
    		actorTrajectory = nullptr;

			this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
		                    this->Interactor->GetEventPosition()[1], 
		                    0,  // always zero.
		                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);

			position = Vector2d(picked[0], picked[1]);
			addTraj(position);
    		this->Interactor->GetRenderWindow()->Render();

       		// Forward events
       		vtkInteractorStyleTrackballActor::OnLeftButtonUp();
       }

        virtual void OnMouseMove() {
            if(leftButtonIsDown){
                // Remove unwanted old actors
                if(actorTrajectory)
                        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorTrajectory);
                actorTrajectory = nullptr;

                this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
                                this->Interactor->GetEventPosition()[1], 
                                0,  // always zero.
                                this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
                double picked[3];
                this->Interactor->GetPicker()->GetPickPosition(picked);

                position = Vector2d(picked[0], picked[1]);
                addTraj(position);
                this->Interactor->GetRenderWindow()->Render();
            }
        }

    private:
        bool leftButtonIsDown = false;
        Vector2d position = Vector2d(0.0,0.0);
        vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();

        // add a trajectory at point pos with specified initial velocity. Removes old trajectories
        void addTraj(Vector2d& pos){
            Vector2d keepPos(pos.x(), pos.y());
            // Remove unwanted old actors
            if(actorTrajectory)
                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorTrajectory);
            actorTrajectory = nullptr;

            // // plot a new trajectory
            Vector3d hitColor;
            traj_pos = mysystem.integrator<maxIteration>(pos, InputVelocity, hitColor);
            actorTrajectory = createLineActor(hitColor);
            this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actorTrajectory);
            pos = keepPos;
        }

};
vtkStandardNewMacro(DenseMouseInteractorStyle);