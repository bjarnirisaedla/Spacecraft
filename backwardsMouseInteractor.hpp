#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkIdList.h>

// Interaction style for backwards integration
class BackwardsMouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static BackwardsMouseInteractorStyle* New();
    vtkTypeMacro(BackwardsMouseInteractorStyle, vtkInteractorStyleTrackballActor);
 
       virtual void OnLeftButtonDown() 
       {
    		leftButtonIsDown = true;
            backTrajVisible = true;

	       	this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
	                           this->Interactor->GetEventPosition()[1], 
	                           0,  // always zero.
	                           this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	       	double picked[3];
	       	this->Interactor->GetPicker()->GetPickPosition(picked);

	       	position = Vector2d(picked[0],picked[1]);

	       	//setup the text and add it to the window
	       	if(textActor){
		       	textActor->GetTextProperty()->SetFontSize (22);
		       	this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor2D (textActor);
		       	textActor->SetPosition2 (100.0, 40.0);
	       		textActor->GetTextProperty()->SetColor (1.,1.,1.);
	       	}
			std::string s = "Position: " + std::to_string(position.x()) + ", " + std::to_string(position.y()) + 
					"\n" + "Velocity: 0.0, 0.0" ;
	       	textActor->SetInput(s.c_str());
	       	
	       	// Forward events
	       	vtkInteractorStyleTrackballActor::OnLeftButtonDown();
       }

       virtual void OnLeftButtonUp() 
       {
    		leftButtonIsDown = false;
    		mysystem.setIsRRFrame(true);
    		// Remove unwanted old actors
    		if(arrowActor)
    				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(arrowActor);
			if(headOfTrajectory)
					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(headOfTrajectory);
			if(actorTrajectory)
					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorTrajectory);
            if(backwardsTrajectory)
                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(backwardsTrajectory);
			for(vtkActorPtr a: planetActorsInertial)
				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(a);
			planetActorsInertial.clear();

			this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
		                    this->Interactor->GetEventPosition()[1], 
		                    0,  // always zero.
		                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);

    		// plot a new trajectory
    		Vector3d hitColor(0,0,0);
            Vector2d vel = calcVelocity(Vector2d(picked[0], picked[1]));
    		traj_pos = mysystem.integrator<maxIteration>(position, vel, hitColor, false);
    		actorTrajectory = createLineActor(Vector3d(.3,.3,.5));
    		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actorTrajectory);
            
            // plot backwards trajectory
            Matrix<double, 2, maxIteration> m;
            Vector2d finalPosition = position;
            Vector2d finalVelocity = vel;
            mysystem.flowmapPosAndVel(finalPosition, finalVelocity, hitColor, false);
            m = mysystem.integrator<maxIteration>(finalPosition, finalVelocity, hitColor, -mysystem.getTimestep(), false);
            backwardsTrajectory = createLineActor(m, Vector3d(.3,.5,.3));
            this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(backwardsTrajectory);

            this->Interactor->GetRenderWindow()->Render();

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

				// remove previous and add a predicting trajectory
				if(headOfTrajectory)
			   		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(headOfTrajectory);
			   	Vector3d hitColor(0,0,0);
				Vector2d velocity = calcVelocity(Vector2d(picked[0],picked[1]));
				Eigen::Matrix<double, 2, predictIteration> m2;
				m2 = mysystem.integrator<predictIteration>(position, velocity, hitColor);
				headOfTrajectory = createLineActor(m2, Vector3d(1.,1.,0.));
				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(headOfTrajectory);

				// add text to update velocity input
				std::string s = "Position: " + std::to_string(position.x()) + ", " + std::to_string(position.y()) + 
					"\n" + "Velocity: " + std::to_string(velocity.x()) + ", " + std::to_string(velocity.y());
				textActor->SetInput (s.c_str());

				this->Interactor->GetRenderWindow()->Render();

       		}
	        // Forward events
	        // vtkInteractorStyleTrackballActor::OnMouseMove();
        }

        virtual void OnKeyPress() 
        {
        	if(leftButtonIsDown)
        		return;

          	std::string key = this->Interactor->GetKeySym();          
          	// Handle switching between reference frames
          	if(key == "I" || key == "i"){
          		if(actorTrajectory)
          				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actorTrajectory);
      			for(vtkActorPtr a: planetActorsInertial)
      				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(a);
      			planetActorsInertial.clear();
          		bool inRRFrame = mysystem.getIsRRFrame();

          		if(inRRFrame){
          			mysystem.trajectoryToIFrame();

          			const int k = 100;
          			std::vector<Vector2d> bodyCenters = mysystem.getBodyPositions();
          			Eigen::VectorXd circle = Eigen::VectorXd::LinSpaced(k, 0, 2*pi);
          			Eigen::Matrix<double, 2, k> m;
          			for(Vector2d& b: bodyCenters){
	          			for(int i = 0; i < k; ++i)
	          				m.col(i) = b.norm()*Vector2d(std::cos(circle(i)), std::sin(circle(i)));
          				planetActorsInertial.push_back(createLineActor(m, Vector3d(0.,1.,0.)));
          			}

          			for(vtkActorPtr a: planetActorsInertial)
          				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(a);
          		}
          		else
	          		mysystem.trajectoryToRRFrame();

	          	mysystem.setIsRRFrame(!inRRFrame);

          		actorTrajectory = createLineActor(Vector3d(.3,.3,.5));
          		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actorTrajectory);
          		this->Interactor->GetRenderWindow()->Render();
          	}

            if(key == "Up"){
                vtkSmartPointer<vtkRendererCollection> collection;
                collection = this->Interactor->GetRenderWindow()->GetRenderers();
                collection->InitTraversal();
                for(vtkIdType i = 0; i < collection->GetNumberOfItems(); ++i){
                    vtkCamera* cam = collection->GetNextItem()->GetActiveCamera();
                    cam->Zoom(1.2);
                    this->Interactor->GetRenderWindow()->Render();
                }
            }
            if(key == "Down"){
                vtkSmartPointer<vtkRendererCollection> collection;
                collection = this->Interactor->GetRenderWindow()->GetRenderers();
                collection->InitTraversal();
                for(vtkIdType i = 0; i < collection->GetNumberOfItems(); ++i){
                    vtkCamera* cam = collection->GetNextItem()->GetActiveCamera();
                    cam->Zoom(0.8);
                    this->Interactor->GetRenderWindow()->Render();
                }
            }

            // toggle visibility of the backwards trajectory
            if(key == "B" || key == "b"){
                if(backTrajVisible){
                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(backwardsTrajectory);
                }
                else{
                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(backwardsTrajectory);
                }
                backTrajVisible = !backTrajVisible;
                this->Interactor->GetRenderWindow()->Render();
            }

          	// Forward events
          	vtkInteractorStyleTrackballActor::OnKeyPress();
        }

    private:
        bool leftButtonIsDown = false;
        bool backTrajVisible = true;
        Vector2d position = Vector2d(0.0,0.0);
        vtkActorPtr arrowActor = nullptr;
        vtkActorPtr headOfTrajectory = nullptr;
        vtkActorPtr backwardsTrajectory = nullptr;
        std::vector<vtkActorPtr> planetActorsInertial;
        vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
        double pi = 3.14159265358979323846;

        Vector2d calcVelocity(Vector2d picked) const {
        	return (picked - position) * 1.5;
        }

};
vtkStandardNewMacro(BackwardsMouseInteractorStyle);