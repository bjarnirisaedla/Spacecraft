#include <vector>
#include <Eigen/Core>

#include <vtkhelper.hpp>

// points are ordered column-wise
typedef Eigen::Matrix<double,2,2>	Line;

class Isolines
{
public:
	Isolines(double val){
		isovalue = val;
	}

	// Uses the marching square algorithm to find isolines
	std::vector<vtkActorPtr> compute(){
		results.clear();
		Vector2d xlim = mysystem.getXlimits();
		Vector2d ylim = mysystem.getYlimits();
		const double dx = (xlim.y() - xlim.x())/resolution;
		const double dy = (ylim.y() - ylim.x())/resolution;
		Vector2d initVelo(0.0,0.0);

		for (int i = 0; i < resolution-1; ++i)
		{
			for (int j = 0; j < resolution-1; ++j)
			{
				Vector2d x00(xlim.x()+i*dx, ylim.x()+j*dy);
				Vector2d x10(xlim.x()+(i+1)*dx, ylim.x()+j*dy);
				Vector2d x01(xlim.x()+i*dx, ylim.x()+(j+1)*dy);
				Vector2d x11(xlim.x()+(i+1)*dx, ylim.x()+(j+1)*dy);

				// v01 ----- v11
				//  |         |
				//  |         |
				// v00 ----- v10

				double v00 = mysystem.getJacobiConstantAt(x00, initVelo);
				double v10 = mysystem.getJacobiConstantAt(x10, initVelo);
				double v01 = mysystem.getJacobiConstantAt(x01, initVelo);
				double v11 = mysystem.getJacobiConstantAt(x11, initVelo);
				
				int b00 = sign(v00, isovalue);
				int b10 = sign(v10, isovalue);
				int b01 = sign(v01, isovalue);
				int b11 = sign(v11, isovalue);

				Vector2d x0c, x1c, xc0, xc1;
				std::vector<Vector2d> points;

				if(b00 == b10 && b00==b01 && b00==b11) continue;
				
				// crossing on the bottom
				if(b00 != b10){
					double t = (isovalue - v00)/(v10 - v00);
					xc0 = x00 + (x10 - x00)*t;
					points.push_back(xc0);
				}

				// crossing on the top
				if(b01 != b11){
					double t = (isovalue - v01)/(v11 - v01);
					xc1 = x01 + (x11-x01)*t;
					points.push_back(xc1);
				}

				// crossing on the left
				if(b00 != b01){
					double t = (isovalue - v00)/(v01 - v00);
					x0c = x00 + (x01-x00)*t;
					points.push_back(x0c);
				}

				// crossing on the right
				if(b10 != b11){
					double t = (isovalue - v10)/(v11 - v10);
					x1c = x10 + (x11-x10)*t;
					points.push_back(x1c);
				}

				switch(points.size()){
				case 1:
					// corner case
					break;
				case 2:
				{
					// There are only two points, create line segment
					Line lina;
					lina.col(0) = points[0];
					lina.col(1) = points[1];
					results.push_back(lina);
					break;
				}
				case 3:
					// this case needs some heuristics
					break;
				case 4:
				{
					// use the midpoint decider
					double vcc = mysystem.getJacobiConstantAt((x00 + x11)*0.5, initVelo);
					int bcc = sign(vcc, isovalue);

					if (b00 == bcc)
					{
						Line lina1;
						lina1.col(0) = xc0;
						lina1.col(1) = x1c;
						results.push_back(lina1);

						Line lina2;
						lina2.col(0) = x0c;
						lina2.col(1) = xc1;
						results.push_back(lina2);
					}
					else
					{
						Line lina1;
						lina1.col(0) = xc0;
						lina1.col(1) = x0c;
						results.push_back(lina1);

						Line lina2;
						lina2.col(0) = x1c;
						lina2.col(1) = xc1;
						results.push_back(lina2);
					}
					break;
				}
				}
			}
		}

		return mergeResults();
	}

	// merges many lines into a longer curves more suitable for vtk actors
	std::vector<vtkActorPtr> mergeResults(){
		std::vector<vtkActorPtr> actors;
		if(results.empty()) return actors;
		double tol = 1e-10;
		std::vector<std::vector<Vector2d>> LFA;
		std::vector<Vector2d> lfa;
		lfa.push_back(results.front().col(0));
		lfa.push_back(results.front().col(1));
		results.erase(results.begin());


		while(results.begin() != results.end()){
			bool added = false;
			for(auto it = results.begin(); it != results.end(); ){
				if((*lfa.end() - (*it).col(0)).norm() < tol){
					added = true;
					lfa.push_back((*it).col(1));			
					it = results.erase(it);				
				}
				else{
					++it;
				}
			}
			// no point was added to the line and there are points left
			if(!added && results.begin() != results.end()){
				std::vector<Vector2d> tmplfa = lfa;
				LFA.push_back(tmplfa);
				lfa.clear();
				lfa.push_back(results.front().col(0));
				lfa.push_back(results.front().col(1));
				results.erase(results.begin());
			}
		}

		for(const std::vector<Vector2d> &l: LFA){
			// build an eigen matrix out of l
			MatrixXd lmat(2,l.size()+1);
			for (int i = 0; i < l.size(); ++i)
				lmat.col(i) = l.at(i);
			lmat.col(l.size()) = l.at(0); 							// should be a closed loop
			vtkActorPtr a = createLineActor(lmat, Vector3d(1.,0.64706,0.));
			actors.push_back(a);
		}
		return actors;
	}


private:
	int sign(double val, double ref){
		return (val > ref) ? 1 : ((val < ref) ? -1 : 0);
	}

	std::vector<Line> results;
	int resolution = 400;
	double isovalue;
};