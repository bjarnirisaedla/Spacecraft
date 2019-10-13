#ifndef MY_SYSTEM_HEADER
#define MY_SYSTEM_HEADER

#include <complex>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

using namespace Eigen;

typedef Matrix<double, 2, 1> 		Vector2d;
typedef Matrix<double, 3, 1> 		Vector3d;
typedef Matrix<double, 4, 1> 		Vector4d;


// Global variables
const int maxIteration = 1000;
const int predictIteration = 250;
Matrix<double, 2, maxIteration> traj_pos = MatrixXd::Zero(2,maxIteration);

enum EIntegrator {
	Integrator_Verlet, Integrator_RK4
};

// properties of every body
struct body_t {
	double radius;
	Vector2d position_curr;
	Vector2d position_original;
	double mass; 
	Vector3d color;

	body_t() {}

	body_t(double r, Vector2d p, double m, Vector3d c)
		: radius(r), position_curr(p), position_original(p), mass(m), color(c) {}

};

// massless particle
struct particle {
	Vector2d pos;
	Vector2d vel;

	particle(const Vector2d& p, const Vector2d v) : pos(p), vel(v) {}
};

particle operator+(const particle& vecA, const particle& vecB) { return particle(vecA.pos+vecB.pos, vecA.vel+vecB.vel); }
particle operator-(const particle& vecA, const particle& vecB) { return particle(vecA.pos-vecB.pos, vecA.vel-vecB.vel); }
particle operator*(const particle& vec, double scalar) { return particle(vec.pos*scalar, vec.vel*scalar); }
particle operator/(const particle& vec, double scalar) { return particle(vec.pos/scalar, vec.vel/scalar); }

class MySystem 
{
public:
	void addBody(double radius, const Vector2d& pos, double mass, Vector3d color = Vector3d(0.,0.,0.)) {
		body_t body_tmp(radius, pos, mass, color);
		bodies.push_back(body_tmp);
		findMassCenter();
	}

	const std::vector<body_t>& getBodies() const{
		return bodies;
	}

	bool hitBody(const Vector2d& point, Vector3d& color) const{
		double dist;
		for(body_t b: bodies){
			dist = (double) (point - b.position_curr).norm();
			color = b.color;
			if( dist < b.radius) return true;
		}
		color = Vector3d(1.,1.,1.); // returns white if it doesn't hit a body
		return false;
	}

	bool hitBody(const Vector2d& point) const{
		double dist;
		for(body_t b: bodies){
			dist = (double) (point - b.position_curr).norm();
			if( dist < b.radius) return true;
		}
		return false;
	}


	Vector2d getGravAccel(const Vector2d& point_rr, const Vector2d& vel_rr) const {
		particle p(point_rr, vel_rr);
		return getGravAccel(p);
	}

	// Takes a particle in rotational frame and returns rotational acceleration in rotation frame
	Vector2d getGravAccel(const particle& p) const {
		Vector2d a_rr(0.0,0.0);
		Vector2d dist(0.0,0.0);

		// calculate inertial acceleration in rotating frame
		// TODO: will only work for bodies stationary in the rotating frame
		for(body_t planet: bodies){
			dist = planet.position_curr - p.pos;
			a_rr += G*planet.mass/dist.squaredNorm()*dist.normalized();
		}

		// add fictituos forces (assume constant angular velocity so the F_Euler = 0)
		Vector2d v_ir = p.vel - Vector2d(-w_0z*p.pos.y(), w_0z*p.pos.x()); 		// inertial velocity in rotational frame
		a_rr += -w_0z*w_0z*p.pos; 												// centrifugal + coriolis
		a_rr += 2.0*Vector2d(-w_0z*v_ir.y(), w_0z*v_ir.x());					// Coriolis vector term

		return a_rr;
	}

	void velocityVerlet(particle& p) const {
		velocityVerlet(p, dt);
	}

	void velocityVerlet(particle& p, const double T) const {
		Vector2d aold = getGravAccel(p);
		p.pos += p.vel*T + 0.5*aold*T*T;
		Vector2d anew = getGravAccel(p);
		p.vel += (aold+anew)/2.0*T;
	}

	particle rungeKutta4(const particle& p) const {
		return rungeKutta4(p, dt);
	}

	particle rungeKutta4(const particle& p, const double T) const {
		particle sample0 = sampleGravity(p);
		particle sample1 = sampleGravity(p + sample0*0.5*T);
		particle sample2 = sampleGravity(p + sample1*0.5*T);
		particle sample3 = sampleGravity(p + sample2*T);
		return p + (sample0 + (sample1+sample2)*2.0 + sample3)/6.0*T;
	}


	template <int iter>
	Matrix<double, 2, iter> integrator(Vector2d pos_r, Vector2d vel_rr, Vector3d& color, bool b = true) const {
		return integrator<iter>(pos_r, vel_rr, color, dt, b);
	}

	template <int iter>
	Matrix<double, 2, iter> integrator(Vector2d pos_r, Vector2d vel_rr, Vector3d& color, double T, bool boundaryOn = true) const {
		Matrix<double, 2, iter> trajectory = MatrixXd::Zero(2,iter);

		trajectory.col(0) = pos_r;
		particle p(pos_r,vel_rr);
		
		for(int i = 1; i < iter; i++){
			for(int k = 0; k < steps_for_integrator; ++k){
				if(step_method == Integrator_RK4){
					p = rungeKutta4(p, T);
					pos_r = p.pos;
					vel_rr = p.vel;
				}
				else if(step_method == Integrator_Verlet){
					velocityVerlet(p, T);
					pos_r = p.pos;
					vel_rr = p.vel;
				}
			}
			if(boundaryOn && hitBody(pos_r, color))
				break;
			if(boundaryOn && (pos_r.x() < min_x || pos_r.x() > max_x || pos_r.y() < min_y || pos_r.y() > max_y))
				break;
			trajectory.col(i) = pos_r;
		}
		return trajectory;
	}

	void flowmap(Vector2d pos, Vector2d vel, Vector3d& color) const {
		color = Vector3d(1.,1.,1.); // default is white
		particle p(pos,vel);
		for(int i = 1; i < maxIteration; i++){
			if(step_method == Integrator_RK4){
				p = rungeKutta4(p);
				pos = p.pos;
			}
			else if(step_method == Integrator_Verlet){
				velocityVerlet(p);
				pos = p.pos;
			}
			if(hitBody(pos, color))
				break;
			if(pos.x() < min_x || pos.x() > max_x || pos.y() < min_y || pos.y() > max_y)
				break;
		}
	}

	void flowmapPosAndVel(Vector2d& pos, Vector2d& vel, Vector3d& color, bool boundaryOn = true) const {
		flowmapPosAndVel(pos, vel, color, dt, boundaryOn);
	}

	void flowmapPosAndVel(Vector2d& pos, Vector2d& vel, Vector3d& color, double T, bool boundaryOn = true) const {
		color = Vector3d(1.,1.,1.); // default is white
		particle p(pos,vel);
		for(int i = 1; i < maxIteration; i++){
			for(int k = 0; k < steps_for_integrator; ++k){
				if(step_method == Integrator_RK4){
					p = rungeKutta4(p, T);
					// pos = p.pos;
					// vel = p.vel;
				}
				else if(step_method == Integrator_Verlet){
					velocityVerlet(p, T);
					// pos = p.pos;
					// vel = p.vel;
				}
			}
			if(boundaryOn && hitBody(pos, color))
				break;
			if(boundaryOn && (pos.x() < min_x || pos.x() > max_x || pos.y() < min_y || pos.y() > max_y))
				break;
		}
		pos = p.pos;
		vel = p.vel;
	}

	std::vector<Vector2d> findLPoints() const {
		int N = 200;
		std::vector<Vector2d> Lpoints;
		VectorXd xdots = VectorXd::LinSpaced(N, min_x, max_x);
		VectorXd ydots = VectorXd::LinSpaced(N, min_y, max_y);

		#pragma omp parallel for
		for(int i = 0; i < N-1; i++){
			for(int j = 0; j < N-1; j++){
				Vector2d res(0,0);
				bool worked = helperLPoints(Vector2d(xdots(i), xdots(i+1)), Vector2d(ydots(j), ydots(j+1)), res);
				if(worked && !hitBody(res)){
					Lpoints.push_back(res);
				}
			}
		}

		return Lpoints;
	};

	Matrix<double, 4, 4> JacobianMat(const particle& p) const {
		Matrix<double, 2, 2> m11, m12, m21, m22;
		m11 = MatrixXd::Zero(2,2);
		// m11 << 0.0, -2.0*w_0z, 2.0*w_0z, 0.0;
		m12 = MatrixXd::Identity(2,2);
		m22 << 0.0, -2.0*w_0z, 2.0*w_0z, 0.0;

		double a11 = 0.0, a21 = 0.0, a12 = 0.0, a22 = 0.0, len = 0.0;
		Vector2d d;
		for(const body_t &b: bodies){
			d = b.position_curr - p.pos;
			len = d.norm();
			a11 += 3.0*G*b.mass*d.x()/std::pow(len,5)*d.x() - G*b.mass/std::pow(len,3);
			a21 += 3.0*G*b.mass*d.x()/std::pow(len,5)*d.y();
			a12 += 3.0*G*b.mass*d.y()/std::pow(len,5)*d.x();
			a22 += 3.0*G*b.mass*d.y()/std::pow(len,5)*d.y() - G*b.mass/std::pow(len,3);
		}
		m21 << a11+w_0z*w_0z, a12, a21, a22+w_0z*w_0z;

		Matrix<double, 4, 4> Jac;
		Jac << m11, m12, m21, m22;

		// // WRITE OUT MATRIX
		// for(int i = 0; i < 4; ++i){
		// 	for(int j = 0; j < 4; ++j)
		// 		std::cout << Jac(i,j) << "\t";
		// 	std::cout << std::endl;
		// }
		// std::cout 	<< std::endl;

		EigenSolver<MatrixXd> solver;
		solver.compute(Jac);
		// std::cout << "The eigenvalues of the jacobian are: \n" << solver.eigenvalues() << std::endl;
		// std::cout << "The first eigenvector of the jacobian is: \n" << solver.eigenvectors().col(0) << std::endl;
		// std::cout << "The 2 eigenvector of the jacobian is: \n" << solver.eigenvectors().col(1) << std::endl;
		// std::cout << "The 3 eigenvector of the jacobian is: \n" << solver.eigenvectors().col(2) << std::endl;
		// std::cout << "The 4 eigenvector of the jacobian is: \n" << solver.eigenvectors().col(3) << std::endl;
		// std::cout << "\n" << std::endl;

		// TEST EIGENVALUES
		Matrix<double,2,2> K;
		K << a11, a12, a21, a22;
		EigenSolver<MatrixXd> Ksolver;
		Ksolver.compute(K);
		std::complex<double> kappa1 = Ksolver.eigenvalues()[0];
		std::complex<double> kappa2 = Ksolver.eigenvalues()[1];

		std::complex<double> k11 = a11;
		std::complex<double> k12 = a12;
		std::complex<double> k21 = a21;
		std::complex<double> k22 = a22;

		// std::complex<double> mylambda1 = -std::sqrt(+std::sqrt(1.0/4.0*(kappa1-kappa2)*(kappa1-kappa2) - 2.0*w_0z*w_0z*(kappa1+kappa2))-w_0z*w_0z+(kappa1+kappa2)/2.0);
		std::complex<double> mylambda1 = -std::sqrt(std::sqrt(-(8.0*k22+8.0*k11)*w_0z*w_0z+k22*k22-2.0*k11*k22+4.0*k12*k12+k11*k11) - 2.0*w_0z*w_0z+k22+k11)/std::sqrt(2.0);
		std::complex<double> mylambda2 = +std::sqrt(+std::sqrt(1.0/4.0*(kappa1-kappa2)*(kappa1-kappa2) - 2.0*w_0z*w_0z*(kappa1+kappa2))-w_0z*w_0z+(kappa1+kappa2)/2.0);
		std::complex<double> mylambda3 = -std::sqrt(-std::sqrt(1.0/4.0*(kappa1-kappa2)*(kappa1-kappa2) - 2.0*w_0z*w_0z*(kappa1+kappa2))-w_0z*w_0z+(kappa1+kappa2)/2.0);
		std::complex<double> mylambda4 = +std::sqrt(-std::sqrt(1.0/4.0*(kappa1-kappa2)*(kappa1-kappa2) - 2.0*w_0z*w_0z*(kappa1+kappa2))-w_0z*w_0z+(kappa1+kappa2)/2.0);

		// std::cout << "Eigenvalues of the Jacobian are: \n" << solver.eigenvalues() << std::endl;
		// std::cout << "My guess is eigenvalues are: \n";
		// std::cout << mylambda1.real() << "\t" << mylambda1.imag() << "\n"; 
		// std::cout << mylambda2 << "\n"; 
		// std::cout << mylambda3 << "\n"; 
		// std::cout << mylambda4 << "\n" << std::endl; 

		// if(std::abs(8.0*(kappa1+kappa2)) > std::abs((kappa1-kappa2)*(kappa1-kappa2)))
		// 	std::cout << "Condition 1 holds. Eigenvalues should be complex" << std::endl;
		// if(w_0z*w_0z > std::abs((kappa1+kappa2)/2.0 - std::sqrt(0.25*(kappa1-kappa2)*(kappa1-kappa2) - 2.0*(kappa1+kappa2))))
		// 	std::cout << "Condition 2 holds. Eigenvalues should be complex" << std::endl;


		return Jac;
	}

	int cond_One(const Vector2d& pos){
		int res = 0;
		double k11 = 0.0;
		double k21 = 0.0;
		double k12 = 0.0;
		double k22 = 0.0;
		Vector2d d;
		for(const body_t &b: bodies){
			d = b.position_curr - pos;
			k11 += 2.0*d.x()*G*b.mass/(d.squaredNorm()*d.squaredNorm())*d.normalized().x();
			k21 += 2.0*d.x()*G*b.mass/(d.squaredNorm()*d.squaredNorm())*d.normalized().y();
			k12 += 2.0*d.y()*G*b.mass/(d.squaredNorm()*d.squaredNorm())*d.normalized().x();
			k22 += 2.0*d.y()*G*b.mass/(d.squaredNorm()*d.squaredNorm())*d.normalized().y();
		}
		// TEST EIGENVALUES
		Matrix<double,2,2> K;
		K << k11, k12, k21, k22;
		EigenSolver<MatrixXd> Ksolver;
		Ksolver.compute(K);
		std::complex<double> kappa1 = Ksolver.eigenvalues()[0];
		std::complex<double> kappa2 = Ksolver.eigenvalues()[1];
		if(std::abs(8.0*(kappa1+kappa2)) > std::abs((kappa1-kappa2)*(kappa1-kappa2)))
			res++;
		if(w_0z*w_0z > std::abs((kappa1+kappa2)/2.0 - std::sqrt(0.25*(kappa1-kappa2)*(kappa1-kappa2) - 2.0*(kappa1+kappa2))))
			res++;
		return res;
	}

	Vector2d getXlimits() const { return Vector2d(min_x, max_x); }

	Vector2d getYlimits() const { return Vector2d(min_y, max_y); }

	void setIntegrator(EIntegrator ig){ step_method = ig; }

	double getTimestep() const { return dt; }

	int getIterationPerTimestep() const { return steps_for_integrator; }

	// maps from inertial frame to rotating reference frame
	// same as multiplying posI with R the rotation matrix
	Vector2d toRRFrame(const Vector2d& posI, int iteration) const {
		double T = dt*iteration*steps_for_integrator;
		return Vector2d(posI.x()*std::cos(w_0z*T) - posI.y()*std::sin(w_0z*T), 
						posI.x()*std::sin(w_0z*T) + posI.y()*std::cos(w_0z*T));
	}

	// maps from inertial frame to rotating reference frame
	// same as multiplying posRR with R^-1, inverse of the rotation matrix
	Vector2d toIFrame(const Vector2d& posRR, int iteration) const {
		double T = dt*iteration*steps_for_integrator;
		return Vector2d(posRR.x()*std::cos(w_0z*T) + posRR.y()*std::sin(w_0z*T), 
						-posRR.x()*std::sin(w_0z*T) + posRR.y()*std::cos(w_0z*T));
	}

	void trajectoryToRRFrame(){
		if(!inRRFrame){
			for(int i = 0; i < maxIteration; ++i){
				traj_pos.col(i) = toRRFrame(traj_pos.col(i), i);
			}
			inRRFrame = true;
		}
	}

	void trajectoryToIFrame(){
		if(inRRFrame){
			for(int i = 0; i < maxIteration; ++i){
				traj_pos.col(i) = toIFrame(traj_pos.col(i), i);
			}
			inRRFrame = false;
		}
	}

	bool getIsRRFrame() const { return inRRFrame;}

	void setIsRRFrame(bool frame){ inRRFrame = frame;}

	std::vector<Vector2d> getBodyPositions() const {
		std::vector<Vector2d> centers;
		for(const body_t &b: bodies)
			centers.push_back(b.position_curr);
		return centers;
	}

	double getPotentialAt(const Vector2d& point) const {
		double U = 0.0;
		U = -w_0z*w_0z*point.squaredNorm()/2.0;
		for(const body_t &b: bodies){
			U -= G*b.mass/(b.position_curr - point).norm();
		}
		return U;
	}

	double getJacobiConstantAt(const particle& p) const {
		return getJacobiConstantAt(p.pos, p.vel);
	}

	// TODO: review
	double getJacobiConstantAt(const Vector2d& point, const Vector2d& vel_r) const {
		double U = getPotentialAt(point);
		// Vector2d vel_in = vel_r + w_0z*Vector2d(-point.y(), point.x());
		return -2.0*U - vel_r.squaredNorm();
	}

private:
	std::vector<body_t> bodies;
	EIntegrator step_method = Integrator_RK4;			// default integration method
	double min_x = -2.5;									// boundaries of the system
	double max_x = 2.5;
	double min_y = -2.5;
	double max_y = 2.5;

	double G = 1.0;										// Gravity constant
	// TODO: update this so grav force matches centrifugal acceleration
	double w_0z = 1.0;									// angular velocity in z-direction
	int steps_for_integrator = 10;
	double dt = 1e-3;
	bool inRRFrame = true;

	void findMassCenter() {
		double M = 0.0;
		Vector2d center(0.0,0.0);
		for(body_t &b: bodies)
			M += b.mass;
		for(body_t &b: bodies)
			center += b.mass/M * b.position_original;
		// Correct positions of bodies to fit the rotating frame
		for(body_t &b: bodies)
			b.position_curr = b.position_original - center;
	};

	void setw_0z(){
		double d = (bodies[0].position_curr - bodies[1].position_curr).norm();
		w_0z = std::sqrt(G*(bodies[0].mass + bodies[1].mass))/(d*d*d);
	}

	// Returns true if there is a possibility of a Lagrange Point inside the box
	bool checkBox(const Vector2d& x, const Vector2d& y) const {
		Vector2d ll, lr, ul, ur; // uppper, lower, left, right
		Vector2d vel_0(0.0,0.0); // zero initial velocity
		ll = getGravAccel(Vector2d(x.x(), y.x()), vel_0);
		lr = getGravAccel(Vector2d(x.y(), y.x()), vel_0);
		ul = getGravAccel(Vector2d(x.x(), y.y()), vel_0);
		ur = getGravAccel(Vector2d(x.y(), y.y()), vel_0);

		double tol = 1e-8;
		// checks if anypoint is the LPoint
		if(ll.norm()<tol || lr.norm()<tol || ul.norm()<tol || ur.norm()<tol)
			return true;

		if((ll.y()<0.0 && lr.y()<0.0 && ul.y()<0.0 && ur.y()<0.0) || (ll.y()>0.0 && lr.y()>0.0 && ul.y()>0.0 && ur.y()>0.0))
			return false;
		if((ll.x()<0.0 && ul.x()<0.0 && lr.x()<0.0 && ur.x()<0.0) || (ll.x()>0.0 && ul.x()>0.0 && lr.x()>0.0 && ur.x()>0.0))
			return false;
		return true;
	} 

	// x and y form the box [x.x(), x.y()] X [y.x(), y.y()]
	// assumes only one Lpoint inside of the box
	bool helperLPoints(const Vector2d& x, const Vector2d& y, Vector2d& result) const {
		double tol = 1e-8;
		if(abs(x.y()-x.x()) < tol && abs(y.y()-y.x()) < tol){
			result = Vector2d(x.x()+(x.y()-x.x())/2.0, y.x()+(y.y()-y.x())/2.0);
			return true;
		}
		if(checkBox(x, y)){
			Vector2d mid(x.x()+(x.y()-x.x())/2.0, y.x()+(y.y()-y.x())/2.0);
			bool rect1, rect2, rect3, rect4;
			rect1 = helperLPoints(Vector2d(x.x(), mid.x()), Vector2d(y.x(), mid.y()), result);
			rect2 = helperLPoints(Vector2d(mid.x(), x.y()), Vector2d(y.x(), mid.y()), result);
			rect3 = helperLPoints(Vector2d(x.x(), mid.x()), Vector2d(mid.y(), y.y()), result);
			rect4 = helperLPoints(Vector2d(mid.x(), x.y()), Vector2d(mid.y(), y.y()), result);
			return rect1 || rect2 || rect3 || rect4;			
		}
		return false;
	}
	
	particle sampleGravity(const particle& p) const {
		return particle(p.vel, getGravAccel(p));
	}

};

#endif