#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkIdList.h>

// Define interaction style
class FTLEMouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static FTLEMouseInteractorStyle* New();
    vtkTypeMacro(FTLEMouseInteractorStyle, vtkInteractorStyleTrackballActor);

       virtual void OnLeftButtonUp() 
       {
    		// Remove unwanted old actors
            for(vtkActorPtr a: fiveTrajs)
                this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(a);
            fiveTrajs.clear();

			this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
		                    this->Interactor->GetEventPosition()[1], 
		                    0,  // always zero.
		                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);

    		// plot a new trajectory
    		Vector3d hitColor(0,0,0);
            Vector2d vel(0.0,0.0);
            Vector2d xlim = mysystem.getXlimits();
            Vector2d ylim = mysystem.getYlimits();
            double dx = (xlim.y()-xlim.x())/resolution;
            double dy = (ylim.y()-ylim.x())/resolution;

            Matrix<double, 2, maxIteration> m, mr, ml, mu, md;
            vtkActorPtr a;

            // middle
            Vector2d pos(picked[0],picked[1]);
            m = mysystem.integrator<maxIteration>(pos, vel, hitColor, false);
            // a = createLineActor(m, Vector3d(.3,.3,.5));
            // fiveTrajs.push_back(a);
            vel.setZero();

            // right
            pos = Vector2d(picked[0]+dx, picked[1]);
            mr = mysystem.integrator<maxIteration>(pos, vel, hitColor, false);
            // a = createLineActor(mr, Vector3d(.3,.3,.5));
            // fiveTrajs.push_back(a);
            vel.setZero();

            // left
            pos = Vector2d(picked[0]-dx, picked[1]);
            ml = mysystem.integrator<maxIteration>(pos, vel, hitColor, false);
            // a = createLineActor(ml, Vector3d(.3,.3,.5));
            // fiveTrajs.push_back(a);
            vel.setZero();

            // up
            pos = Vector2d(picked[0], picked[1]+dy);
            mu = mysystem.integrator<maxIteration>(pos, vel, hitColor, false);
            // a = createLineActor(mu, Vector3d(.3,.3,.5));
            // fiveTrajs.push_back(a);
            vel.setZero();

            //down 
            pos = Vector2d(picked[0], picked[1]-dy);
            md = mysystem.integrator<maxIteration>(pos, vel, hitColor, false);
            // a = createLineActor(md, Vector3d(.3,.3,.5));
            // fiveTrajs.push_back(a);
            vel.setZero();


    		// for(vtkActorPtr act: fiveTrajs)
    		//   this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(act);

      //       this->Interactor->GetRenderWindow()->Render();

            for(vtkActorPtr act: fiveTrajs)
              this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(act);
            fiveTrajs.clear();

            // creates an animation of all five trajectories
            int step = 5;
            bool kill = false;
            for(int i = 0; i < maxIteration; i += step-1){
                if(i > maxIteration-step){
                    step = maxIteration-i;
                    kill = true;
                }
               a = createLineActor(m.block(0, i, 2, step), Vector3d(.0,1.,.0)); // green
               fiveTrajs.push_back(a);
               a = createLineActor(mr.block(0, i, 2, step), Vector3d(1.,.0,.0)); // red
               fiveTrajs.push_back(a);
               a = createLineActor(ml.block(0, i, 2, step), Vector3d(.0,.0,1.)); // blue
               fiveTrajs.push_back(a);
               a = createLineActor(mu.block(0, i, 2, step), Vector3d(1.,.0,1.)); // magenta
               fiveTrajs.push_back(a);
               a = createLineActor(md.block(0, i, 2, step), Vector3d(.0,1.,1.)); // cyan
               fiveTrajs.push_back(a);

               for(vtkActorPtr act: fiveTrajs)
                 this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(act);
                this->Interactor->GetRenderWindow()->Render();
                if(kill) break;
            }

       		// Forward events
       		vtkInteractorStyleTrackballActor::OnLeftButtonUp();
       }

       virtual void OnKeyPress() 
       {
            std::string key = this->Interactor->GetKeySym();          
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

            // Forward events
            vtkInteractorStyleTrackballActor::OnKeyPress();
       }

    private:
        std::vector<vtkActorPtr> fiveTrajs;
        int resolution = 1000;
};
vtkStandardNewMacro(FTLEMouseInteractorStyle);